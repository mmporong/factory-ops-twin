"""Bounded operator-saved dashboard snapshots for read-only replay."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Callable
import uuid


MAX_FRAMES = 120
MAX_FILE_BYTES = 16 * 1024 * 1024
_SAFE_ID = re.compile(r"[a-z0-9][a-z0-9-]{0,63}-[0-9a-f]{12}")
_SELECTED_KEYS = {
    "schema_version", "run_id", "revision", "mode", "connection", "observed_at",
    "sim_time_s", "paused", "robots", "tasks", "events", "inventory", "kpis",
    "commands", "capabilities", "charging", "model_scope",
}


def _canonical(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
        sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha(value: Any) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _stamp(value: str) -> datetime:
    stamp = datetime.fromisoformat(value)
    if stamp.tzinfo is None:
        raise ValueError("scenario timestamp needs timezone")
    return stamp


class ScenarioReplayStore:
    def __init__(self, directory: Path, validator: Callable[[Any], dict[str, Any]],
                 scene: dict[str, Any]) -> None:
        self.directory = directory
        self.validator = validator
        self.scene_sha256 = _sha(scene)
        self._history: deque[dict[str, Any]] = deque(maxlen=MAX_FRAMES)
        self._run_id: str | None = None

    def observe(self, snapshot: dict[str, Any]) -> bool:
        selected = {key: deepcopy(value) for key, value in snapshot.items() if key in _SELECTED_KEYS}
        selected = self.validator(selected)
        if selected["connection"] != "connected" or selected["mode"] == "replay":
            return False
        if self._run_id != selected["run_id"]:
            self._history.clear()
            self._run_id = selected["run_id"]
        if self._history:
            previous = self._history[-1]
            if (selected["revision"] < previous["revision"]
                    or selected["sim_time_s"] < previous["sim_time_s"]
                    or _stamp(selected["observed_at"]) < _stamp(previous["observed_at"])):
                raise ValueError("scenario history snapshot regressed")
            if (selected["revision"] == previous["revision"]
                    and selected["sim_time_s"] == previous["sim_time_s"]):
                return False
        self._history.append(selected)
        return True

    @staticmethod
    def _name(value: Any) -> str:
        if (not isinstance(value, str) or not value.strip() or len(value.strip()) > 80
                or any(not char.isprintable() for char in value)):
            raise ValueError("scenario name must be 1-80 visible characters")
        return value.strip()

    def save(self, value: Any) -> dict[str, Any]:
        if not isinstance(value, dict) or set(value) != {"name"}:
            raise ValueError("scenario request must contain only name")
        name = self._name(value["name"])
        if not self._history or self._run_id is None:
            raise ValueError("no connected snapshot history is available")
        slug = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")[:48] or "scenario"
        scenario_id = f"{slug}-{uuid.uuid4().hex[:12]}"
        if _SAFE_ID.fullmatch(scenario_id) is None:
            raise AssertionError("generated scenario identity is unsafe")
        frames = []
        for index, snapshot in enumerate(self._history):
            frames.append({"index": index, "source_mode": snapshot["mode"],
                "snapshot_sha256": _sha(snapshot), "snapshot": deepcopy(snapshot)})
        payload = {"schema_version": 1, "scenario_id": scenario_id, "name": name,
            "created_at": datetime.now(timezone.utc).isoformat(), "source_run_id": self._run_id,
            "frame_count": len(frames), "scene_sha256": self.scene_sha256,
            "scope": "selected dashboard snapshots; no physics simulation rerun",
            "frames": frames}
        document = {**payload, "payload_sha256": _sha(payload)}
        raw = json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2).encode("utf-8") + b"\n"
        if len(raw) > MAX_FILE_BYTES:
            raise ValueError("scenario file exceeds the bounded size")
        self.directory.mkdir(parents=True, exist_ok=True)
        if self.directory.is_symlink():
            raise ValueError("scenario directory must not be a symlink")
        target = self.directory / f"{scenario_id}.json"
        try:
            fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError as error:
            raise ValueError("scenario already exists; refusing overwrite") from error
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException:
            try:
                target.unlink()
            except FileNotFoundError:
                pass
            raise
        return self._metadata(document, hashlib.sha256(raw).hexdigest())

    def _path(self, scenario_id: str) -> Path:
        if not isinstance(scenario_id, str) or _SAFE_ID.fullmatch(scenario_id) is None:
            raise ValueError("scenario id is unsafe")
        target = self.directory / f"{scenario_id}.json"
        if target.parent.resolve() != self.directory.resolve() or target.is_symlink():
            raise ValueError("scenario path escapes its directory")
        return target

    def _load(self, scenario_id: str) -> tuple[dict[str, Any], str]:
        target = self._path(scenario_id)
        try:
            size = target.stat().st_size
            if not 0 < size <= MAX_FILE_BYTES or not target.is_file():
                raise ValueError("scenario file size is invalid")
            raw = target.read_bytes()
        except OSError as error:
            raise ValueError("scenario does not exist") from error
        if len(raw) != size:
            raise ValueError("scenario file changed while reading")
        try:
            document = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError("scenario file is incomplete or invalid") from error
        required = {"schema_version", "scenario_id", "name", "created_at", "source_run_id",
            "frame_count", "scene_sha256", "scope", "frames", "payload_sha256"}
        if (not isinstance(document, dict) or set(document) != required
                or type(document.get("schema_version")) is not int or document["schema_version"] != 1):
            raise ValueError("scenario document schema differs")
        payload = {key: document[key] for key in required - {"payload_sha256"}}
        if document["scenario_id"] != scenario_id or document["payload_sha256"] != _sha(payload):
            raise ValueError("scenario identity or payload hash differs")
        if document["scene_sha256"] != self.scene_sha256:
            raise ValueError("scenario factory layout differs from the current scene")
        self._name(document["name"])
        _stamp(document["created_at"])
        frames = document["frames"]
        if (not isinstance(document["source_run_id"], str) or not document["source_run_id"]
                or type(document["frame_count"]) is not int
                or not isinstance(frames, list) or not 1 <= len(frames) <= MAX_FRAMES
                or document["frame_count"] != len(frames)):
            raise ValueError("scenario frame inventory is invalid")
        previous: dict[str, Any] | None = None
        for index, frame in enumerate(frames):
            if (not isinstance(frame, dict) or set(frame) != {"index", "source_mode", "snapshot_sha256", "snapshot"}
                    or frame["index"] != index or frame["source_mode"] not in ("local_model", "live")):
                raise ValueError("scenario frame sequence is invalid")
            snapshot = self.validator(frame["snapshot"])
            if (snapshot["run_id"] != document["source_run_id"] or snapshot["mode"] != frame["source_mode"]
                    or snapshot["connection"] != "connected" or frame["snapshot_sha256"] != _sha(snapshot)):
                raise ValueError("scenario frame provenance or hash differs")
            if previous is not None and (snapshot["revision"] < previous["revision"]
                    or snapshot["sim_time_s"] < previous["sim_time_s"]
                    or _stamp(snapshot["observed_at"]) < _stamp(previous["observed_at"])
                    or snapshot["revision"] == previous["revision"] and snapshot["sim_time_s"] == previous["sim_time_s"]):
                raise ValueError("scenario frame order is not monotonic")
            previous = snapshot
        return document, hashlib.sha256(raw).hexdigest()

    @staticmethod
    def _metadata(document: dict[str, Any], file_sha256: str) -> dict[str, Any]:
        return {key: document[key] for key in
            ("scenario_id", "name", "created_at", "source_run_id", "frame_count", "scene_sha256", "scope")} | {
                "file_sha256": file_sha256}

    def list(self) -> dict[str, Any]:
        if not self.directory.exists():
            return {"scenarios": []}
        if self.directory.is_symlink() or not self.directory.is_dir():
            raise ValueError("scenario directory is invalid")
        paths = sorted(self.directory.glob("*.json"))
        if len(paths) > 1000:
            raise ValueError("scenario directory exceeds the bounded inventory")
        scenarios = []
        for path in paths:
            scenario_id = path.stem
            document, file_sha = self._load(scenario_id)
            scenarios.append(self._metadata(document, file_sha))
        return {"scenarios": scenarios}

    def frame(self, scenario_id: str, index: int) -> dict[str, Any]:
        if type(index) is not int or index < 0:
            raise ValueError("scenario frame index is invalid")
        document, file_sha = self._load(scenario_id)
        if index >= document["frame_count"]:
            raise ValueError("scenario frame does not exist")
        stored = document["frames"][index]
        result = deepcopy(stored["snapshot"])
        result["mode"] = "replay"
        result["capabilities"] = []
        result["replay"] = {"scenario_id": scenario_id, "frame_index": index,
            "frame_count": document["frame_count"], "source_run_id": document["source_run_id"],
            "source_mode": stored["source_mode"], "source_revision": result["revision"],
            "source_observed_at": result["observed_at"], "source_snapshot_sha256": stored["snapshot_sha256"],
            "scenario_file_sha256": file_sha, "scene_sha256": document["scene_sha256"], "scope": document["scope"]}
        return self.validator(result)
