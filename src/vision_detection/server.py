"""LOCAL OUTPUT: snapshots and read-only HTTP. No remote transmission/motors."""

import json
import os
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from .settings import PROJECT_DIRECTORY


class DetectionSnapshot:
    """Share a consistent metadata/JPEG pair between detector and HTTP threads."""

    def __init__(self):
        """Initialize an empty snapshot and the lock used by producer/HTTP threads."""
        self.lock = threading.Lock()
        self.metadata = {
            "schema_version": 1,
            "status": "starting",
            "camera_active": False,
            "detections": [],
        }
        self.jpeg = None

    def publish(self, metadata, jpeg):
        """Atomically replace metadata and preview for the same processed frame."""
        with self.lock:
            self.metadata, self.jpeg = metadata, jpeg

    def read(self):
        """Return the current snapshot; consumers must inspect its timestamp."""
        with self.lock:
            return dict(self.metadata), self.jpeg


class LocalDetectionServer(ThreadingHTTPServer):
    """Refuse duplicate port bindings, including on Windows."""

    allow_reuse_address = False

    def server_bind(self):
        """Prevent a second local server from silently sharing this port."""
        if os.name == "nt":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def start_local_server(snapshot, port, host="127.0.0.1"):
    """Start background request handling; the caller must shut the server down.

    host defaults to this machine only. "0.0.0.0" lets a laptop on the same
    trusted network view a headless robot's dashboard; it stays read-only.
    """

    # NAV-OUTPUT: localhost HTTP is a prototype observation transport. A future
    # controller needs a bounded-age heartbeat and explicit handling when this
    # process stops; the last successful JSON response must not stay actionable.
    class DetectionRequestHandler(BaseHTTPRequestHandler):
        """Expose only approved local viewer routes and current observations."""
        def log_message(self, *_):
            """Suppress routine access logs to keep the detector terminal readable."""
            pass  # Avoid a terminal line for every preview refresh.

        def do_GET(self):
            """Serve only the dashboard and current read-only detection/JPEG snapshot."""
            route = urlsplit(self.path).path
            metadata, jpeg = snapshot.read()
            # A fixed allowlist avoids exposing other project files.
            static_files = {
                "/": ("index.html", "text/html; charset=utf-8"),
                "/styles.css": ("styles.css", "text/css; charset=utf-8"),
                "/viewer.js": ("viewer.js", "application/javascript; charset=utf-8"),
            }
            if route in static_files:
                filename, content_type = static_files[route]
                body = (PROJECT_DIRECTORY / "web" / filename).read_bytes()
            elif route == "/api/detections":
                body, content_type = (
                    json.dumps(metadata, allow_nan=False).encode(),
                    "application/json",
                )
            elif route == "/frame.jpg" and jpeg is not None:
                body, content_type = jpeg, "image/jpeg"
            else:
                self.send_error(503 if route == "/frame.jpg" else 404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Closing a browser tab midway through a request is normal.

    server = LocalDetectionServer((host, port), DetectionRequestHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
