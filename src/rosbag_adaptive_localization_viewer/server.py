from __future__ import annotations

import dataclasses
import io
import json
import tempfile
from email.parser import BytesParser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from rosbag_adaptive_localization_viewer.builder import build_experiment_payload
from rosbag_adaptive_localization_viewer.loaders.rosbag2 import inspect_bag_topics


# ---------------------------------------------------------------------------
# Multipart/form-data parser — replaces the deprecated cgi.FieldStorage
# (removed in Python 3.13).
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class _FormPart:
    """A single part from a multipart/form-data body."""

    name: str
    filename: str | None
    data: bytes

    @property
    def value(self) -> str:
        return self.data.decode("utf-8", errors="replace")

    @property
    def file(self) -> io.BytesIO:
        """Return a fresh BytesIO over the raw bytes."""
        return io.BytesIO(self.data)


def _parse_multipart(headers, body: bytes) -> dict[str, _FormPart]:
    """Parse a multipart/form-data body using Python's email module.

    Compatible with Python 3.11–3.13+ (does not use the removed cgi module).
    """
    content_type = headers.get("Content-Type", "")
    # Construct a minimal MIME email so BytesParser can handle the multipart body
    msg_bytes = f"Content-Type: {content_type}\r\n\r\n".encode("latin-1") + body
    msg = BytesParser().parsebytes(msg_bytes)

    parts: dict[str, _FormPart] = {}
    for part in msg.walk():
        if part.get_content_maintype() == "multipart":
            continue
        disposition = part.get("Content-Disposition", "")
        if not disposition:
            continue

        name: str | None = None
        filename: str | None = None
        for segment in disposition.split(";"):
            segment = segment.strip()
            if segment.startswith("name="):
                name = segment[5:].strip('"')
            elif segment.startswith("filename="):
                filename = segment[9:].strip('"')

        if name is None:
            continue

        payload = part.get_payload(decode=True) or b""
        parts[name] = _FormPart(name=name, filename=filename, data=payload)

    return parts


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

class ViewerApiHandler(BaseHTTPRequestHandler):
    server_version = "RALVLocalAPI/0.1"

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self._write_cors_headers()
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/health":
            self._send_json({"status": "ok"})
            return
        self._send_json({"error": "Not found"}, status=HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/topics":
            self._handle_topics()
        elif path == "/build-experiment":
            self._handle_build_experiment()
        else:
            self._send_json({"error": "Not found"}, status=HTTPStatus.NOT_FOUND)

    # ------------------------------------------------------------------
    # Request handlers
    # ------------------------------------------------------------------

    def _handle_topics(self) -> None:
        """POST /topics — upload a .db3 bag, return its topic list.

        Body: multipart/form-data with a single field ``bag_file``.
        Response: ``{"topics": [{id, name, type, message_count}, ...]}``
        """
        form = self._read_multipart()
        if form is None:
            return

        bag_part = form.get("bag_file")
        if bag_part is None or not bag_part.data:
            self._send_json({"error": "Missing bag_file"}, status=HTTPStatus.BAD_REQUEST)
            return

        with tempfile.TemporaryDirectory(prefix="ralv-topics-") as tmp_dir:
            bag_path = self._save_part(Path(tmp_dir), bag_part, ".db3")
            try:
                topics = inspect_bag_topics(bag_path)
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return

        self._send_json({"topics": topics})

    def _handle_build_experiment(self) -> None:
        """POST /build-experiment — upload a bag (and optional OptiTrack CSV),
        return the full experiment payload as JSON.
        """
        form = self._read_multipart()
        if form is None:
            return

        bag_part = form.get("bag_file")
        if bag_part is None or not bag_part.data:
            self._send_json({"error": "Missing bag_file"}, status=HTTPStatus.BAD_REQUEST)
            return

        def _str(key: str, default: str) -> str:
            return form[key].value if key in form else default

        experiment_name = _str("experiment_name", "uploaded_experiment")
        primary_topic = _str("primary_topic", "/odom")
        comparison_raw = _str("comparison_topics", "/odom_raw,/odom_raw_adapted")
        comparison_topics = [t.strip() for t in comparison_raw.split(",") if t.strip()]
        rigid_body_name = _str("rigid_body_name", "ROBOT")
        ros_distro = _str("ros_distro", "humble")
        include_optitrack = _str("include_optitrack", "false") == "true"
        optitrack_part = form.get("optitrack_file")

        with tempfile.TemporaryDirectory(prefix="ralv-upload-") as tmp_dir:
            tmp_path = Path(tmp_dir)
            bag_path = self._save_part(tmp_path, bag_part, ".db3")
            optitrack_path = None
            if include_optitrack and optitrack_part is not None and optitrack_part.data:
                optitrack_path = self._save_part(tmp_path, optitrack_part, ".csv")

            try:
                payload = build_experiment_payload(
                    experiment_name=experiment_name,
                    bag_path=bag_path,
                    primary_topic=primary_topic,
                    comparison_topics=comparison_topics,
                    optitrack_csv_path=optitrack_path,
                    rigid_body_name=rigid_body_name,
                    ros_distro=ros_distro,
                )
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return

        self._send_json(payload)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _read_multipart(self) -> dict[str, _FormPart] | None:
        content_type = self.headers.get("Content-Type", "")
        if "multipart/form-data" not in content_type:
            self._send_json({"error": "Expected multipart/form-data"}, status=HTTPStatus.BAD_REQUEST)
            return None
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length)
        return _parse_multipart(self.headers, body)

    def _save_part(self, directory: Path, part: _FormPart, fallback_suffix: str) -> Path:
        raw_name = Path(part.filename or f"upload{fallback_suffix}").name
        target = directory / raw_name
        target.write_bytes(part.data)
        return target

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return

    def _write_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self._write_cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def serve(host: str = "127.0.0.1", port: int = 8765) -> None:
    server = ThreadingHTTPServer((host, port), ViewerApiHandler)
    print(f"RALV local API listening on http://{host}:{port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
