#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/sdp.h>

#ifndef GST_USE_UNSTABLE_API
#define GST_USE_UNSTABLE_API
#endif
#include <gst/webrtc/webrtc.h>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "cuda_runtime.h"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <regex>
#include <vector>
#include <optional>
#include <sstream>
#include <cstring>
#include <functional>
#include <algorithm>
#include <cmath>
#include <chrono>

using WsClient = websocketpp::client<websocketpp::config::asio_client>;
using websocketpp::connection_hdl;

// ---------------- Helpers ----------------
static std::string base64_encode(const std::string &in) {
  static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  int val = 0, valb = -6;
  for (unsigned char c : in) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(chars[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6) out.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
  while (out.size() % 4) out.push_back('=');
  return out;
}

static std::string base64_decode(const std::string &in) {
  static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<int> T(256, -1);
  for (int i = 0; i < 64; i++) T[chars[i]] = i;

  std::string out;
  int val = 0, valb = -8;
  for (unsigned char c : in) {
    if (c == '=') break;
    if (T[c] == -1) continue;
    val = (val << 6) + T[c];
    valb += 6;
    if (valb >= 0) {
      out.push_back(char((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return out;
}

static std::string json_escape(const std::string& in) {
  std::string out; out.reserve(in.size() + 32);
  for (char c : in) {
    if (c == '"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else out += c;
  }
  return out;
}

static std::optional<std::string> extract_string(const std::string& s, const std::string& key) {
  std::regex re("\"" + key + "\"\\s*:\\s*\"((?:\\\\.|[^\"\\\\])*)\"");
  std::smatch m;
  if (!std::regex_search(s, m, re)) return std::nullopt;
  return m[1].str();
}

static std::optional<int> extract_int(const std::string& s, const std::string& key) {
  std::regex re("\"" + key + "\"\\s*:\\s*(\\d+)");
  std::smatch m;
  if (!std::regex_search(s, m, re)) return std::nullopt;
  return std::stoi(m[1].str());
}

static std::optional<double> extract_float(const std::string& s, const std::string& key) {
  std::regex re("\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)");
  std::smatch m;
  if (!std::regex_search(s, m, re)) return std::nullopt;
  return std::stod(m[1].str());
}

static int parse_h264_pt_from_sdp(const std::string& sdp) {
  std::regex re(R"(a=rtpmap:(\d+)\s+H264\/90000)");
  std::smatch m;
  if (std::regex_search(sdp, m, re)) return std::stoi(m[1].str());
  return -1;
}

// ---------------- Peer Context ----------------
struct PeerContext {
  std::string id;
  GstElement* webrtcbin = nullptr;
  GstElement* queue = nullptr;
  GstElement* capsfilter = nullptr;
  GstElement* pay = nullptr;                    // per-peer rtph264pay
  int pt = -1;                                  // negotiated payload type
  GstWebRTCDataChannel* data_channel = nullptr; // optional
  bool ice_connected = false;                   // NEW: for "network disconnected" overlay decision
};

// ---------------- Node ----------------
class WebRTCStreamer : public rclcpp::Node {
public:
  WebRTCStreamer() : Node("webrtc_streamer") {
    declare_parameter<std::string>("image_topic", "/zed/zed_node/rgb/image_rect_color");
    declare_parameter<std::string>("ws_url", "ws://127.0.0.1:9002");
    declare_parameter<int>("fps", 30);
    declare_parameter<int>("bitrate_kbps", 2000);

    // cmd mapping
    declare_parameter<double>("max_lin", 0.8);
    declare_parameter<double>("max_ang", 1.5);
    declare_parameter<double>("cmd_rate_hz", 20.0);
    declare_parameter<int>("deadman_ms", 300);

    // heartbeat
    declare_parameter<double>("heartbeat_timeout_s", 0.5);

    // NEW: overlay timeout (separate from heartbeat)
    declare_parameter<double>("frame_timeout_s", 2.0);

    image_topic_  = get_parameter("image_topic").as_string();
    ws_url_       = get_parameter("ws_url").as_string();
    fps_          = get_parameter("fps").as_int();
    bitrate_kbps_ = get_parameter("bitrate_kbps").as_int();

    max_lin_ = get_parameter("max_lin").as_double();
    max_ang_ = get_parameter("max_ang").as_double();
    cmd_rate_hz_ = get_parameter("cmd_rate_hz").as_double();
    deadman_ms_ = get_parameter("deadman_ms").as_int();

    hb_timeout_s_ = get_parameter("heartbeat_timeout_s").as_double();
    frame_timeout_s_ = get_parameter("frame_timeout_s").as_double();

    // Publishers
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
    hb_pub_  = create_publisher<std_msgs::msg::UInt8>("zed_heartbeat", 10);

    // command timer (keeps rover moving smoothly + deadman)
    cmd_timer_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / std::max(1.0, cmd_rate_hz_))),
      std::bind(&WebRTCStreamer::cmd_timer_cb_, this)
    );

    // heartbeat timer
    hb_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&WebRTCStreamer::heartbeat_timer_cb_, this)
    );

    // NEW: stale/network overlay push timer (only pushes frames when needed)
    stale_timer_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / std::max(1, fps_))),
      std::bind(&WebRTCStreamer::stale_timer_cb_, this)
    );

    gst_init(nullptr, nullptr);

    gmain_thread_ = std::thread([this]() {
      context_ = g_main_context_new();
      loop_ = g_main_loop_new(context_, FALSE);
      g_main_loop_run(loop_);
    });

    start_ws_();

    rclcpp::SensorDataQoS qos;
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      std::bind(&WebRTCStreamer::image_cb_, this, std::placeholders::_1));

    last_image_time_ = now();
    last_cmd_time_ = now();

    last_ros_frame_tp_ = std::chrono::steady_clock::time_point{}; // none yet

    RCLCPP_INFO(get_logger(), "Streamer Ready (video ok) + DataChannel cmd_vel_out + zed_heartbeat + OpenCV overlay");
  }

  ~WebRTCStreamer() override {
    shutting_down_ = true;
    stop_ws_();

    // cleanup data channels
    for (auto& kv : peers_) {
      if (kv.second.data_channel) {
        gst_object_unref(kv.second.data_channel);
        kv.second.data_channel = nullptr;
      }
    }

    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }

    if (loop_) {
      g_main_loop_quit(loop_);
      if (gmain_thread_.joinable()) gmain_thread_.join();
      g_main_loop_unref(loop_);
      loop_ = nullptr;

      g_main_context_unref(context_);
      context_ = nullptr;
    }
  }

private:
  // Ensure gst calls run in g_main_loop thread
  void invoke_gst_(std::function<void()> fn) {
    auto* f = new std::function<void()>(std::move(fn));
    g_main_context_invoke(context_, [](gpointer d)->gboolean {
      auto* func = (std::function<void()>*)d;
      (*func)();
      delete func;
      return G_SOURCE_REMOVE;
    }, f);
  }

  static std::string pick_gst_format_(const std::string& enc) {
    if (enc == "bgr8") return "BGR";
    if (enc == "rgb8") return "RGB";
    if (enc == "bgra8") return "BGRA";
    if (enc == "rgba8") return "RGBA";
    return "";
  }

  int get_bpp_(const std::string& fmt) {
    if (fmt == "BGR" || fmt == "RGB") return 3;
    if (fmt == "BGRA" || fmt == "RGBA") return 4;
    return 0;
  }

  // -------- OpenCV overlay helpers --------
  static void draw_green_path_overlay_bgr_(cv::Mat& bgr) {
    if (bgr.empty()) return;
    cv::Mat overlay = bgr.clone();
    const int h = bgr.rows;
    const int w = bgr.cols;

    std::vector<cv::Point> pts;
    pts.emplace_back((int)(w * 0.33), (int)(h * 0.70));
    pts.emplace_back((int)(w * 0.68), (int)(h * 0.70));
    pts.emplace_back((int)(w * 0.88), h);
    pts.emplace_back((int)(w * 0.20), h);

    const std::vector<std::vector<cv::Point>> polys{pts};
    cv::fillPoly(overlay, polys, cv::Scalar(0, 255, 0)); // green in BGR

    // Same alpha as your python code: 0.3
    cv::addWeighted(overlay, 0.3, bgr, 0.7, 0.0, bgr);
  }

  static void draw_center_text_bgr_(cv::Mat& bgr, const std::string& msg) {
    if (bgr.empty()) return;
    const int h = bgr.rows;
    const int w = bgr.cols;

    int baseline = 0;
    const double fontScale = 1.0;
    const int thickness = 2;
    cv::Size ts = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
    const int x = std::max(0, (w - ts.width) / 2);
    const int y = std::max(ts.height, (h + ts.height) / 2);

    cv::putText(bgr, msg, cv::Point(x, y),
                cv::FONT_HERSHEY_SIMPLEX, fontScale,
                cv::Scalar(0, 0, 255), thickness, cv::LINE_AA); // red
  }

  // Convert incoming ROS image to BGR for drawing, then later convert to desired gst fmt for appsrc
  static bool rosimg_to_bgr_(const sensor_msgs::msg::Image& msg, cv::Mat& out_bgr) {
    if (msg.data.empty() || msg.width == 0 || msg.height == 0) return false;

    if (msg.encoding == "bgr8") {
      cv::Mat bgr((int)msg.height, (int)msg.width, CV_8UC3,
                  const_cast<unsigned char*>(msg.data.data()), (size_t)msg.step);
      out_bgr = bgr.clone();
      return true;
    }
    if (msg.encoding == "rgb8") {
      cv::Mat rgb((int)msg.height, (int)msg.width, CV_8UC3,
                  const_cast<unsigned char*>(msg.data.data()), (size_t)msg.step);
      cv::cvtColor(rgb, out_bgr, cv::COLOR_RGB2BGR);
      return true;
    }
    if (msg.encoding == "bgra8") {
      cv::Mat bgra((int)msg.height, (int)msg.width, CV_8UC4,
                   const_cast<unsigned char*>(msg.data.data()), (size_t)msg.step);
      cv::cvtColor(bgra, out_bgr, cv::COLOR_BGRA2BGR);
      return true;
    }
    if (msg.encoding == "rgba8") {
      cv::Mat rgba((int)msg.height, (int)msg.width, CV_8UC4,
                   const_cast<unsigned char*>(msg.data.data()), (size_t)msg.step);
      cv::cvtColor(rgba, out_bgr, cv::COLOR_RGBA2BGR);
      return true;
    }
    return false;
  }

  // Convert BGR mat to the configured gst format buffer bytes
  bool bgr_to_gst_bytes_(const cv::Mat& bgr, const std::string& gst_fmt,
                        std::vector<uint8_t>& out_bytes, int& out_bpp) {
    if (bgr.empty()) return false;

    if (gst_fmt == "BGR") {
      out_bpp = 3;
      out_bytes.assign(bgr.data, bgr.data + (size_t)bgr.cols * (size_t)bgr.rows * 3);
      return true;
    }
    if (gst_fmt == "RGB") {
      out_bpp = 3;
      cv::Mat rgb;
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      out_bytes.assign(rgb.data, rgb.data + (size_t)rgb.cols * (size_t)rgb.rows * 3);
      return true;
    }
    if (gst_fmt == "BGRA") {
      out_bpp = 4;
      cv::Mat bgra;
      cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
      out_bytes.assign(bgra.data, bgra.data + (size_t)bgra.cols * (size_t)bgra.rows * 4);
      return true;
    }
    if (gst_fmt == "RGBA") {
      out_bpp = 4;
      cv::Mat rgba;
      cv::cvtColor(bgr, rgba, cv::COLOR_BGR2RGBA);
      out_bytes.assign(rgba.data, rgba.data + (size_t)rgba.cols * (size_t)rgba.rows * 4);
      return true;
    }
    return false;
  }

  // -------- WebSocket --------
  void start_ws_() {
    ws_.clear_access_channels(websocketpp::log::alevel::all);
    ws_.init_asio();

    ws_.set_open_handler([this](connection_hdl h) {
      ws_hdl_ = h;
      ws_connected_ = true;
      RCLCPP_INFO(get_logger(), "WS Connected");
      send_ws_("{\"type\":\"register\", \"role\":\"ROBOT\", \"id\":\"ROBOT\"}");
    });

    ws_.set_message_handler([this](connection_hdl, WsClient::message_ptr msg) {
      handle_signal_(msg->get_payload());
    });

    websocketpp::lib::error_code ec;
    auto con = ws_.get_connection(ws_url_, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "WS connect error: %s", ec.message().c_str());
      return;
    }
    ws_.connect(con);
    ws_thread_ = std::thread([this]() { ws_.run(); });
  }

  void stop_ws_() {
    ws_.stop();
    if (ws_thread_.joinable()) ws_thread_.join();
  }

  void send_ws_(const std::string& msg) {
    if (!ws_connected_) return;
    websocketpp::lib::error_code ec;
    ws_.send(ws_hdl_, msg, websocketpp::frame::opcode::text, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "WS send error: %s", ec.message().c_str());
    }
  }

  void handle_signal_(const std::string& payload) {
    auto type = extract_string(payload, "type");
    auto from = extract_string(payload, "from");
    if (!from) from = extract_string(payload, "clientId");

    // OFFER
    if (type && *type == "offer" && from) {
      std::string sdp;

      if (auto b64 = extract_string(payload, "sdp_base64")) {
        sdp = base64_decode(*b64);
      } else if (auto s = extract_string(payload, "sdp")) {
        sdp = *s;
      } else if (auto d_pos = payload.find("\"data\""); d_pos != std::string::npos) {
        if (auto b64 = extract_string(payload.substr(d_pos), "sdp_base64")) sdp = base64_decode(*b64);
        else if (auto s2 = extract_string(payload.substr(d_pos), "sdp")) sdp = *s2;
      }

      if (!sdp.empty()) {
        invoke_gst_([this, id=*from, s=sdp]() { process_offer_(id, s); });
      }
      return;
    }

    // ICE
    if (type && *type == "ice" && from) {
      std::string cand;
      int mline = -1;

      if (auto c = extract_string(payload, "candidate")) cand = *c;
      if (auto m = extract_int(payload, "sdpMLineIndex")) mline = *m;

      if (cand.empty()) {
        if (auto d_pos = payload.find("\"data\""); d_pos != std::string::npos) {
          if (auto c2 = extract_string(payload.substr(d_pos), "candidate")) cand = *c2;
          if (auto m2 = extract_int(payload.substr(d_pos), "sdpMLineIndex")) mline = *m2;
        }
      }

      if (!cand.empty() && mline != -1) {
        invoke_gst_([this, id=*from, c=cand, m=mline]() {
          auto it = peers_.find(id);
          if (it != peers_.end() && it->second.webrtcbin) {
            g_signal_emit_by_name(it->second.webrtcbin, "add-ice-candidate", m, c.c_str());
          }
        });
      }
      return;
    }
  }

  // -------- Pipeline --------
  void ensure_base_pipeline_(int w, int h, const std::string& fmt) {
    if (pipeline_) return;

    gst_fmt_ = fmt;
    last_w_ = w;
    last_h_ = h;

    // tee outputs H264 elementary stream (NOT RTP)
    std::ostringstream ss;
    ss << "appsrc name=src is-live=true do-timestamp=true format=time "
       << "caps=video/x-raw,format=" << fmt << ",width=" << w << ",height=" << h
       << ",framerate=" << fps_ << "/1 "
       << "! videoconvert ! video/x-raw,format=I420 "
       << "! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=" << fps_
       << " bitrate=" << bitrate_kbps_ << " "
       << "! video/x-h264,profile=constrained-baseline "
       << "! h264parse config-interval=-1 "
       << "! video/x-h264,stream-format=(string)byte-stream,alignment=(string)au "
       << "! tee name=t "
       << "t. ! queue leaky=downstream ! fakesink async=false";

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(ss.str().c_str(), &err);
    if (err) {
      RCLCPP_ERROR(get_logger(), "Pipeline: %s", err->message);
      g_error_free(err);
      return;
    }

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    tee_    = gst_bin_get_by_name(GST_BIN(pipeline_), "t");

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    // Run buffered offers
    for (auto& pair : pending_offers_) {
      process_offer_impl_(pair.first, pair.second);
    }
    pending_offers_.clear();
  }

  void process_offer_(const std::string& peer_id, const std::string& sdp_str) {
    if (!pipeline_) {
      pending_offers_[peer_id] = sdp_str;
      return;
    }
    process_offer_impl_(peer_id, sdp_str);
  }

  // CallbackData forward-declared BEFORE use
  struct CallbackData { WebRTCStreamer* self; std::string id; };

  static void on_answer_static(GstPromise* p, gpointer d) {
    auto* cd = (CallbackData*)d;
    cd->self->on_answer_(p, cd->id);
    delete cd;
  }

  void set_peer_pt_and_caps_(const std::string& peer_id, int pt) {
    auto it = peers_.find(peer_id);
    if (it == peers_.end()) return;
    auto& pc = it->second;

    if (!pc.pay || !pc.capsfilter) return;
    if (pt <= 0) return;

    pc.pt = pt;

    g_object_set(pc.pay, "pt", pt, nullptr);

    std::string caps_str =
      "application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,"
      "packetization-mode=(string)1,payload=(int)" + std::to_string(pt);

    GstCaps* rtp_caps2 = gst_caps_from_string(caps_str.c_str());
    g_object_set(pc.capsfilter, "caps", rtp_caps2, nullptr);
    gst_caps_unref(rtp_caps2);
  }

  // ---- DataChannel callbacks ----
  static void on_data_channel_static(GstElement* webrtcbin, GstWebRTCDataChannel* channel, gpointer user_data) {
    auto* self = static_cast<WebRTCStreamer*>(user_data);
    auto* idptr = static_cast<std::string*>(g_object_get_data(G_OBJECT(webrtcbin), "pid"));
    if (!idptr || !channel) return;

    const std::string peer_id = *idptr;

    auto it = self->peers_.find(peer_id);
    if (it == self->peers_.end()) return;

    // store ref
    if (it->second.data_channel) {
      gst_object_unref(it->second.data_channel);
      it->second.data_channel = nullptr;
    }
    it->second.data_channel = channel;
    gst_object_ref(it->second.data_channel);

    // listen for string messages
    g_signal_connect(channel, "on-message-string", G_CALLBACK(on_dc_message_string_static), self);

    RCLCPP_INFO(self->get_logger(), "DataChannel connected for peer %s", peer_id.c_str());
  }

  // Robust key support (fixes "joystick not sending cmd_vel_out")
  static void on_dc_message_string_static(GstWebRTCDataChannel* /*channel*/, gchar* str, gpointer user_data) {
    auto* self = static_cast<WebRTCStreamer*>(user_data);
    if (!str) return;

    std::string msg(str);

    // Accept multiple schemas:
    // {"lin":0.12,"ang":-0.34}
    // {"linear_x":0.12,"angular_z":-0.34}
    // {"linear":0.12,"angular":-0.34}
    double lin = 0.0;
    double ang = 0.0;

    if (auto v = extract_float(msg, "lin")) lin = *v;
    else if (auto v2 = extract_float(msg, "linear_x")) lin = *v2;
    else if (auto v3 = extract_float(msg, "linear")) lin = *v3;

    if (auto w = extract_float(msg, "ang")) ang = *w;
    else if (auto w2 = extract_float(msg, "angular_z")) ang = *w2;
    else if (auto w3 = extract_float(msg, "angular")) ang = *w3;

    lin = std::clamp(lin, -self->max_lin_, self->max_lin_);
    ang = std::clamp(ang, -self->max_ang_, self->max_ang_);


    // If joystick is centered (0,0), don't treat it as an active command.
    // This prevents spamming zeros which can override other controllers.
    constexpr double kZeroEps = 1e-3;
    if (std::abs(lin) < kZeroEps && std::abs(ang) < kZeroEps) {
      return;
    }
    {
      std::lock_guard<std::mutex> lk(self->cmd_mtx_);
      self->last_cmd_.linear.x = lin;
      self->last_cmd_.angular.z = ang;
      self->last_cmd_time_ = self->now();
    }
  }

  // NEW: notify::ice-connection-state (safe on Humble; avoids GstWebRTCConnectionState compile issue)
  static void on_ice_state_notify_static(GObject* obj, GParamSpec*, gpointer user_data) {
    auto* self = static_cast<WebRTCStreamer*>(user_data);
    auto* idptr = static_cast<std::string*>(g_object_get_data(G_OBJECT(obj), "pid"));
    if (!idptr) return;

    GstWebRTCICEConnectionState st = GST_WEBRTC_ICE_CONNECTION_STATE_NEW;
    g_object_get(obj, "ice-connection-state", &st, nullptr);

    auto it = self->peers_.find(*idptr);
    if (it == self->peers_.end()) return;

    const bool connected =
      (st == GST_WEBRTC_ICE_CONNECTION_STATE_CONNECTED) ||
      (st == GST_WEBRTC_ICE_CONNECTION_STATE_COMPLETED);

    it->second.ice_connected = connected;

    // recompute count
    int count = 0;
    for (const auto& kv : self->peers_) if (kv.second.ice_connected) count++;
    self->connected_peers_.store(count);
  }

  void process_offer_impl_(const std::string& peer_id, const std::string& sdp_str) {
    // Create per-peer chain (queue -> rtph264pay -> capsfilter -> webrtcbin)
    if (peers_.find(peer_id) == peers_.end()) {
      PeerContext pc;
      pc.id = peer_id;

      pc.queue      = gst_element_factory_make("queue", nullptr);
      pc.pay        = gst_element_factory_make("rtph264pay", nullptr);
      pc.capsfilter = gst_element_factory_make("capsfilter", nullptr);
      pc.webrtcbin  = gst_element_factory_make("webrtcbin", nullptr);

      if (!pc.queue || !pc.pay || !pc.capsfilter || !pc.webrtcbin) {
        RCLCPP_ERROR(get_logger(), "Failed to create elements for peer %s", peer_id.c_str());
        return;
      }

      g_object_set(pc.pay, "config-interval", 1, "mtu", 1200, nullptr);

      GstCaps* rtp_caps = gst_caps_from_string(
        "application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,packetization-mode=(string)1"
      );
      g_object_set(pc.capsfilter, "caps", rtp_caps, nullptr);
      gst_caps_unref(rtp_caps);

      g_object_set(pc.webrtcbin,
                   "bundle-policy", 3,
                   "stun-server", "stun://stun.l.google.com:19302",
                   nullptr);

      gst_bin_add_many(GST_BIN(pipeline_), pc.queue, pc.pay, pc.capsfilter, pc.webrtcbin, nullptr);

      // tee -> queue
      {
        GstPad* ts = gst_element_request_pad_simple(tee_, "src_%u");
        GstPad* qsink = gst_element_get_static_pad(pc.queue, "sink");
        if (ts && qsink) {
          gst_pad_link(ts, qsink);
        }
        if (ts) gst_object_unref(ts);
        if (qsink) gst_object_unref(qsink);
      }

      // queue -> pay -> capsfilter
      gst_element_link(pc.queue, pc.pay);
      gst_element_link(pc.pay, pc.capsfilter);

      // capsfilter(src) -> webrtcbin(sink_%u)
      {
        GstPad* csrc = gst_element_get_static_pad(pc.capsfilter, "src");
        GstPad* wsink = gst_element_request_pad_simple(pc.webrtcbin, "sink_%u");
        if (csrc && wsink) {
          gst_pad_link(csrc, wsink);
        }
        if (csrc) gst_object_unref(csrc);
        if (wsink) gst_object_unref(wsink);
      }

      // Sync states
      gst_element_sync_state_with_parent(pc.queue);
      gst_element_sync_state_with_parent(pc.pay);
      gst_element_sync_state_with_parent(pc.capsfilter);
      gst_element_sync_state_with_parent(pc.webrtcbin);

      // ICE callback
      g_signal_connect(pc.webrtcbin, "on-ice-candidate", G_CALLBACK(on_ice_static), this);

      // DataChannel callback (browser creates the channel)
      g_signal_connect(pc.webrtcbin, "on-data-channel", G_CALLBACK(on_data_channel_static), this);

      // NEW: track ice state
      g_signal_connect(pc.webrtcbin, "notify::ice-connection-state", G_CALLBACK(on_ice_state_notify_static), this);

      // Store peer id pointer
      g_object_set_data(G_OBJECT(pc.webrtcbin), "pid", new std::string(peer_id));

      peers_[peer_id] = pc;
    }

    // Set PT early from OFFER
    {
      int pt_offer = parse_h264_pt_from_sdp(sdp_str);
      if (pt_offer > 0) set_peer_pt_and_caps_(peer_id, pt_offer);
    }

    GstElement* webrtc = peers_[peer_id].webrtcbin;

    // Parse SDP offer
    GstSDPMessage* sdp_msg = nullptr;
    gst_sdp_message_new(&sdp_msg);

    if (gst_sdp_message_parse_buffer((guint8*)sdp_str.data(), sdp_str.size(), sdp_msg) != GST_SDP_OK) {
      RCLCPP_ERROR(get_logger(), "Failed to parse SDP offer for %s", peer_id.c_str());
      gst_sdp_message_free(sdp_msg);
      return;
    }

    GstWebRTCSessionDescription* desc =
      gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_msg);

    auto* cd = new CallbackData{this, peer_id};

    GstPromise* p_set = gst_promise_new_with_change_func(
      +[](GstPromise* p, gpointer user_data) {
        auto* cd = (CallbackData*)user_data;
        gst_promise_unref(p);

        GstPromise* p_ans = gst_promise_new_with_change_func(on_answer_static, cd, nullptr);
        g_signal_emit_by_name(cd->self->peers_[cd->id].webrtcbin, "create-answer", nullptr, p_ans);
      },
      cd, nullptr
    );

    g_signal_emit_by_name(webrtc, "set-remote-description", desc, p_set);
    gst_webrtc_session_description_free(desc);
  }

  void on_answer_(GstPromise* p, const std::string& id) {
    const GstStructure* r = gst_promise_get_reply(p);
    GstWebRTCSessionDescription* ans = nullptr;
    gst_structure_get(r, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &ans, nullptr);
    gst_promise_unref(p);

    if (!ans) {
      RCLCPP_ERROR(get_logger(), "Failed to create answer for %s!", id.c_str());
      return;
    }

    gchar* txt = gst_sdp_message_as_text(ans->sdp);

    // Parse PT from ANSWER and apply
    {
      std::string sdp_txt(txt);
      int pt_ans = parse_h264_pt_from_sdp(sdp_txt);
      if (pt_ans > 0) set_peer_pt_and_caps_(id, pt_ans);
    }

    // Set local description
    GstPromise* p2 = gst_promise_new();
    g_signal_emit_by_name(peers_[id].webrtcbin, "set-local-description", ans, p2);
    gst_promise_unref(p2);

    // Send base64 answer
    std::string b64 = base64_encode(txt);
    std::ostringstream ss;
    ss << "{\"type\":\"answer\", \"to\":\"" << id << "\", \"sdp_base64\":\"" << b64 << "\"}";
    send_ws_(ss.str());

    g_free(txt);
    gst_webrtc_session_description_free(ans);
  }

  // Static ICE callback
  static void on_ice_static(GstElement* w, guint m, gchar* c, gpointer d) {
    auto* self = (WebRTCStreamer*)d;
    std::string* id = (std::string*)g_object_get_data(G_OBJECT(w), "pid");
    if (!id || !c) return;

    std::ostringstream ss;
    ss << "{\"type\":\"ice\", \"to\":\"" << *id << "\", \"data\":{\"candidate\":\""
       << json_escape(c) << "\", \"sdpMLineIndex\":" << m << "}}";
    self->send_ws_(ss.str());
  }

  // -------- Image callback --------
  void image_cb_(sensor_msgs::msg::Image::SharedPtr msg) {
    if (shutting_down_.load()) return;

    // heartbeat mark
    last_image_time_ = now();

    const std::string gst_fmt = pick_gst_format_(msg->encoding);
    if (gst_fmt.empty()) return;

    if (!pipeline_built_.load()) {
      invoke_gst_([this, w=(int)msg->width, h=(int)msg->height, f=gst_fmt]() {
        ensure_base_pipeline_(w, h, f);
        pipeline_built_.store(true);
      });
      return;
    }

    // Convert to BGR for OpenCV, draw overlay, keep last good frame
    cv::Mat bgr;
    if (!rosimg_to_bgr_(*msg, bgr)) return;

    draw_green_path_overlay_bgr_(bgr);

    {
      std::lock_guard<std::mutex> lk(last_frame_mtx_);
      last_frame_bgr_ = bgr;
      last_w_ = bgr.cols;
      last_h_ = bgr.rows;
      last_ros_frame_tp_ = std::chrono::steady_clock::now();
      have_frame_.store(true);
    }

    // Convert back to gst fmt and push (same pipeline logic; still appsrc push)
    std::vector<uint8_t> bytes;
    int bpp = 0;
    if (!bgr_to_gst_bytes_(bgr, gst_fmt_, bytes, bpp)) return;

    std::lock_guard<std::mutex> lk(gst_mtx_);
    if (!pipeline_ || !appsrc_) return;

    const gsize size = (gsize)last_w_ * (gsize)last_h_ * (gsize)bpp;
    if (bytes.size() < size) return;

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
      const guint row_bytes = (guint)last_w_ * (guint)bpp;
      for (int y = 0; y < last_h_; ++y) {
        std::memcpy(map.data + ((size_t)y * row_bytes),
                    bytes.data() + ((size_t)y * row_bytes),
                    row_bytes);
      }
      gst_buffer_unmap(buf, &map);
      gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf);
    } else {
      gst_buffer_unref(buf);
    }
  }

  // NEW: push overlay frames when topic stalled or network disconnected
  void stale_timer_cb_() {
    if (shutting_down_.load()) return;
    if (!pipeline_built_.load()) return;

    // Decide if we need to push a synthetic frame
    const int connected = connected_peers_.load();
    const bool have_frame = have_frame_.load();

    std::string msg;
    if (connected <= 0) {
      msg = "NETWORK DISCONNECTED";
    } else if (!have_frame) {
      msg = "WAITING FOR TOPIC...";
    } else {
      std::chrono::steady_clock::time_point tp;
      {
        std::lock_guard<std::mutex> lk(last_frame_mtx_);
        tp = last_ros_frame_tp_;
      }
      if (tp == std::chrono::steady_clock::time_point{}) {
        msg = "WAITING FOR TOPIC...";
      } else {
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - tp).count();
        if (age > frame_timeout_s_) msg = "NO ROS DATA / TOPIC STALLED";
      }
    }

    if (msg.empty()) return; // no need to push anything if stream is healthy

    int w = 640, h = 480;
    {
      std::lock_guard<std::mutex> lk(last_frame_mtx_);
      if (last_w_ > 0 && last_h_ > 0) { w = last_w_; h = last_h_; }
    }

    cv::Mat bgr(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    draw_center_text_bgr_(bgr, msg);
    draw_green_path_overlay_bgr_(bgr); // keep same marker size/shape

    std::vector<uint8_t> bytes;
    int bpp = 0;
    if (!bgr_to_gst_bytes_(bgr, gst_fmt_, bytes, bpp)) return;

    std::lock_guard<std::mutex> lk(gst_mtx_);
    if (!pipeline_ || !appsrc_) return;

    const gsize size = (gsize)w * (gsize)h * (gsize)bpp;
    if (bytes.size() < size) return;

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, size, nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
      const guint row_bytes = (guint)w * (guint)bpp;
      for (int y = 0; y < h; ++y) {
        std::memcpy(map.data + ((size_t)y * row_bytes),
                    bytes.data() + ((size_t)y * row_bytes),
                    row_bytes);
      }
      gst_buffer_unmap(buf, &map);
      gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf);
    } else {
      gst_buffer_unref(buf);
    }
  }

  // -------- cmd timer (deadman + periodic publish) --------
  void cmd_timer_cb_() {
    geometry_msgs::msg::Twist out;
    rclcpp::Time now_t = now();

    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      const double age_ms = (now_t - last_cmd_time_).seconds() * 1000.0;
      if (age_ms > (double)deadman_ms_) {
        out.linear.x = 0.0;
        out.angular.z = 0.0;
      } else {
        out = last_cmd_;
      }
    }
    // Avoid spamming 0,0 when joystick is centered / deadman triggers.
    constexpr double kZeroEps = 1e-3;
    const bool is_zero = (std::abs(out.linear.x) < kZeroEps) && (std::abs(out.angular.z) < kZeroEps);
    if (is_zero) {
      if (last_pub_was_zero_) return;
      last_pub_was_zero_ = true;
    } else {
      last_pub_was_zero_ = false;
    }

    cmd_pub_->publish(out);
  }

  // -------- heartbeat timer --------
  void heartbeat_timer_cb_() {
    std_msgs::msg::UInt8 hb;
    const double age_s = (now() - last_image_time_).seconds();
    hb.data = (age_s <= hb_timeout_s_) ? 1 : 0;
    hb_pub_->publish(hb);
  }

  // -------- Members --------
  std::string image_topic_;
  std::string ws_url_;
  int fps_{30};
  int bitrate_kbps_{2000};

  double max_lin_{0.8};
  double max_ang_{1.5};
  double cmd_rate_hz_{20.0};
  int deadman_ms_{300};

  double hb_timeout_s_{0.5};
  double frame_timeout_s_{2.0}; // NEW

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hb_pub_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr stale_timer_; // NEW

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0,0,RCL_ROS_TIME};
  std::mutex cmd_mtx_;

  bool last_pub_was_zero_{false}; // suppress repeated zero publishes
  rclcpp::Time last_image_time_{0,0,RCL_ROS_TIME};

  GstElement *pipeline_{nullptr}, *appsrc_{nullptr}, *tee_{nullptr};

  std::unordered_map<std::string, PeerContext> peers_;
  std::unordered_map<std::string, std::string> pending_offers_;

  GMainLoop* loop_{nullptr};
  GMainContext* context_{nullptr};
  std::thread gmain_thread_;

  std::mutex gst_mtx_;
  std::atomic<bool> pipeline_built_{false};

  WsClient ws_;
  connection_hdl ws_hdl_;
  std::thread ws_thread_;
  std::atomic<bool> ws_connected_{false};
  std::atomic<bool> shutting_down_{false};

  // NEW: last good frame storage for overlay frames
  std::mutex last_frame_mtx_;
  cv::Mat last_frame_bgr_;
  int last_w_{640};
  int last_h_{480};
  std::string gst_fmt_{"BGR"};
  std::atomic<bool> have_frame_{false};
  std::chrono::steady_clock::time_point last_ros_frame_tp_{};

  // NEW: connected peers count (for "NETWORK DISCONNECTED" overlay)
  std::atomic<int> connected_peers_{0};
};

// ---------------- main ----------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebRTCStreamer>());
  rclcpp::shutdown();
  return 0;
}