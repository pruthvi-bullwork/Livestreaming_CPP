// web/http_admin.js
const http = require("http");
const fs = require("fs");
const path = require("path");

const HOST = "0.0.0.0";
const PORT = process.env.PORT ? Number(process.env.PORT) : 8080;

const webRoot = __dirname;

function send(res, code, contentType, body) {
  res.writeHead(code, { "Content-Type": contentType });
  res.end(body);
}

function mimeType(file) {
  const ext = path.extname(file).toLowerCase();
  if (ext === ".html") return "text/html";
  if (ext === ".js") return "application/javascript";
  if (ext === ".css") return "text/css";
  if (ext === ".png") return "image/png";
  if (ext === ".jpg" || ext === ".jpeg") return "image/jpeg";
  if (ext === ".svg") return "image/svg+xml";
  if (ext === ".ico") return "image/x-icon";
  return "application/octet-stream";
}

const server = http.createServer((req, res) => {
  let urlPath = req.url.split("?")[0];

  // Routes
  if (urlPath === "/") urlPath = "/index.html";
  if (urlPath === "/admin") urlPath = "/admin.html";

  // Prevent path traversal
  urlPath = path.normalize(urlPath).replace(/^(\.\.(\/|\\|$))+/, "");
  const filePath = path.join(webRoot, urlPath);

  fs.readFile(filePath, (err, data) => {
    if (err) {
      send(res, 404, "text/plain", "404 Not Found");
      return;
    }
    send(res, 200, mimeType(filePath), data);
  });
});

server.listen(PORT, HOST, () => {
  console.log(`--- HTTP SERVER RUNNING ON http://${HOST}:${PORT} ---`);
  console.log(`Normal: http://<IP>:${PORT}/`);
  console.log(`Admin : http://<IP>:${PORT}/admin`);
});
