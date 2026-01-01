// web/server.js
const WebSocket = require("ws");

const PORT = 9002;
const MAX_VIDEO_SLOTS = 2;
const ADMIN_PASSWORD = "acu@bullwork";

const wss = new WebSocket.Server({ port: PORT, host: "0.0.0.0" });

/**
 * clients: id -> {
 *   ws, role, admin, requestedMode, grantedMode
 * }
 */
const clients = new Map();

function now() {
  return new Date().toISOString();
}

function safeSend(ws, obj) {
  try {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  } catch {}
}

function videoSlotsUsed() {
  let used = 0;
  for (const c of clients.values()) {
    if (c.role === "BROWSER" && c.grantedMode === "video" && !c.admin) used++;
  }
  return used;
}

function broadcastStats() {
  const used = videoSlotsUsed();
  const online = clients.size;

  const payload = {
    type: "stats",
    online,
    videoSlots: { used, max: MAX_VIDEO_SLOTS },
    browsers: [...clients.values()].filter(c => c.role === "BROWSER").length,
    robotOnline: clients.has("ROBOT"),
  };

  for (const c of clients.values()) safeSend(c.ws, payload);
}

function grantModeForBrowser(requestedMode, isAdmin) {
  if (isAdmin) return "video"; // admin always gets video

  // Normal user:
  if (requestedMode !== "video") return "control";

  const used = videoSlotsUsed();
  if (used < MAX_VIDEO_SLOTS) return "video";
  return "control";
}

wss.on("connection", (ws, req) => {
  const ip = req.socket.remoteAddress;
  console.log(`[${now()}] [CONNECT] ${ip}`);

  ws.on("message", (buf) => {
    let msg;
    try {
      msg = JSON.parse(buf.toString());
    } catch {
      return;
    }

    // -------- REGISTER --------
    if (msg.type === "register") {
      const id = String(msg.id || "").trim();
      const role = String(msg.role || "").trim(); // ROBOT or BROWSER
      const requestedMode = String(msg.mode || "video"); // "video" or "control"

      // admin only if correct password provided
      const admin = (msg.admin_password && msg.admin_password === ADMIN_PASSWORD);

      if (!id || !role) return;

      // Replace old client with same id
      if (clients.has(id)) {
        try { clients.get(id).ws.close(); } catch {}
        clients.delete(id);
      }

      let grantedMode = requestedMode;
      if (role === "BROWSER") grantedMode = grantModeForBrowser(requestedMode, admin);
      if (role === "ROBOT") grantedMode = "video";

      clients.set(id, { ws, role, admin, requestedMode, grantedMode });

      console.log(
        `[${now()}] [REGISTER] ${id} (${role}) mode=${grantedMode} admin=${admin} Online=${clients.size} videoSlots=${videoSlotsUsed()}/${MAX_VIDEO_SLOTS}`
      );

      // tell client what it got
      safeSend(ws, {
        type: "registered",
        id,
        role,
        admin,
        mode_granted: grantedMode,
        videoSlots: { used: videoSlotsUsed(), max: MAX_VIDEO_SLOTS },
        robotOnline: clients.has("ROBOT"),
      });

      broadcastStats();
      return;
    }

    // -------- ROUTING --------
    const to = msg.to || msg.target || msg.dest || msg.peer || null;
    const from = msg.from || msg.clientId || null;
    if (!to) return;

    const dst = clients.get(to);
    if (!dst) {
      console.log(`[${now()}] [DROP] ${from || "?"} -> ${to} (${msg.type}) target offline`);
      return;
    }

    // Ensure from included
    if (!msg.from && from) msg.from = from;

    safeSend(dst.ws, msg);

    if (msg.type === "offer") {
      console.log(`[${now()}] [ROUTE] ${from || "?"} -> ${to} (offer)`);
    } else if (msg.type === "answer") {
      const l = (msg.sdp || "").length || (msg.sdp_base64 || "").length || 0;
      console.log(`[${now()}] [ROUTE] ${from || "?"} -> ${to} (answer) sdp_len=${l}`);
    } else if (msg.type === "ice") {
      console.log(`[${now()}] [ROUTE] ${from || "?"} -> ${to} (ice)`);
    }
  });

  ws.on("close", () => {
    let removedId = null;
    for (const [id, c] of clients.entries()) {
      if (c.ws === ws) {
        removedId = id;
        clients.delete(id);
        break;
      }
    }
    if (removedId) {
      console.log(`[${now()}] [DISCONNECT] ${removedId}`);
      console.log(`[${now()}] videoSlots=${videoSlotsUsed()}/${MAX_VIDEO_SLOTS}`);
      broadcastStats();
    }
  });
});

console.log(`--- SIGNALING SERVER RUNNING ON ws://0.0.0.0:${PORT} ---`);
