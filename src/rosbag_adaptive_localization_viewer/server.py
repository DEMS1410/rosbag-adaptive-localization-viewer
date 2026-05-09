from __future__ import annotations

import cgi
import json
import tempfile
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from rosbag_adaptive_localization_viewer.builder import build_experiment_payload


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
        if path != "/build-experiment":
            self._send_json({"error": "Not found"}, status=HTTPStatus.NOT_FOUND)
            return

        content_type = self.headers.get("Content-Type", "")
        if "multipart/form-data" not in content_type:
            self._send_json({"error": "Expected multipart/form-data"}, status=HTTPStatus.BAD_REQUEST)
            return

        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={
                "REQUEST_METHOD": "POST",
                "CONTENT_TYPE": content_type,
            },
        )

        bag_file = form["bag_file"] if "bag_file" in form else None
        if bag_file is None or not getattr(bag_file, "file", None):
            self._send_json({"error": "Missing bag_file"}, status=HTTPStatus.BAD_REQUEST)
            return

        experiment_name = self._field_value(form, "experiment_name") or "uploaded_experiment"
        primary_topic = self._field_value(form, "primary_topic") or "/odom"
        comparison_raw = self._field_value(form, "comparison_topics") or "/odom_raw,/odom_raw_adapted"
        comparison_topics = [topic.strip() for topic in comparison_raw.split(",") if topic.strip()]
        rigid_body_name = self._field_value(form, "rigid_body_name") or "ROBOT"
        include_optitrack = self._field_value(form, "include_optitrack") == "true"
        optitrack_file = form["optitrack_file"] if "optitrack_file" in form else None

        with tempfile.TemporaryDirectory(prefix="ralv-upload-") as tmp_dir:
            tmp_path = Path(tmp_dir)
            bag_path = self._save_upload(tmp_path, bag_file, ".db3")
            optitrack_path = None
            if include_optitrack and optitrack_file is not None and getattr(optitrack_file, "file", None):
                optitrack_path = self._save_upload(tmp_path, optitrack_file, ".csv")

            try:
                payload = build_experiment_payload(
                    experiment_name=experiment_name,
                    bag_path=bag_path,
                    primary_topic=primary_topic,
                    comparison_topics=comparison_topics,
                    optitrack_csv_path=optitrack_path,
                    rigid_body_name=rigid_body_name,
                )
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return

        self._send_json(payload)

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return

    def _save_upload(self, directory: Path, field: cgi.FieldStorage, fallback_suffix: str) -> Path:
        raw_name = Path(field.filename or f"upload{fallback_suffix}").name
        target = directory / raw_name
        data = field.file.read()
        target.write_bytes(data)
        return target

    def _field_value(self, form: cgi.FieldStorage, key: str) -> str | None:
        if key not in form:
            return None
        value = form[key]
        if isinstance(value, list):
            value = value[0]
        return value.value

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


def serve(host: str = "127.0.0.1", port: int = 8765) -> None:
    server = ThreadingHTTPServer((host, port), ViewerApiHandler)
    print(f"RALV local API listening on http://{host}:{port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
