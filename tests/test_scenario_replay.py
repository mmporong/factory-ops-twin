import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from factory_ops.dashboard import DashboardService, make_server


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"


class ScenarioReplayTests(unittest.TestCase):
    def test_record_cannot_be_displayed_against_another_factory_layout(self):
        with tempfile.TemporaryDirectory() as folder:
            service = DashboardService(CONFIG, scenario_dir=Path(folder))
            service.snapshot()
            saved = service.save_scenario({"name": "공장 배치 확인"})
            other = DashboardService(CONFIG.parent / "factory-semiconductor", scenario_dir=Path(folder))
            with self.assertRaisesRegex(ValueError, "layout differs"):
                other.replay_frame(saved["scenario_id"], 0)

    def test_http_save_list_and_read_only_frame_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as folder:
            scenario_dir = Path(folder) / "scenarios"
            service = DashboardService(CONFIG, scenario_dir=scenario_dir)
            server = make_server(service, 0)
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            base = f"http://127.0.0.1:{server.server_port}"
            try:
                with urlopen(base + "/api/snapshot", timeout=2) as response:
                    first = json.load(response)
                service.tick(1.0)
                with urlopen(base + "/api/snapshot", timeout=2) as response:
                    second = json.load(response)
                with urlopen(base + "/api/snapshot", timeout=2):
                    pass
                request = Request(base + "/api/scenarios",
                    data=json.dumps({"name": "교대조 운영 기록"}).encode("utf-8"),
                    headers={"Content-Type": "application/json"}, method="POST")
                with urlopen(request, timeout=2) as response:
                    self.assertEqual(201, response.status)
                    saved = json.load(response)
                self.assertEqual(2, saved["frame_count"])
                self.assertEqual(first["run_id"], saved["source_run_id"])

                with urlopen(base + "/api/scenarios", timeout=2) as response:
                    listed = json.load(response)
                self.assertEqual([saved], listed["scenarios"])
                with urlopen(base + f"/api/scenarios/{saved['scenario_id']}/frames/1", timeout=2) as response:
                    replay = json.load(response)
                self.assertEqual("replay", replay["mode"])
                self.assertEqual([], replay["capabilities"])
                self.assertEqual(second["run_id"], replay["run_id"])
                self.assertEqual("local_model", replay["replay"]["source_mode"])
                self.assertEqual(second["observed_at"], replay["replay"]["source_observed_at"])

                path = scenario_dir / f"{saved['scenario_id']}.json"
                tampered = json.loads(path.read_text(encoding="utf-8"))
                tampered["frames"][0]["snapshot"]["robots"][0]["label"] = "tampered"
                path.write_text(json.dumps(tampered), encoding="utf-8")
                with self.assertRaises(HTTPError) as error:
                    urlopen(base + f"/api/scenarios/{saved['scenario_id']}/frames/0", timeout=2)
                self.assertEqual(400, error.exception.code)
            finally:
                server.shutdown()
                server.server_close()
                worker.join(timeout=2)

    def test_generated_identity_cannot_overwrite_and_unsafe_id_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            service = DashboardService(CONFIG, scenario_dir=Path(folder))
            service.snapshot()
            fixed = SimpleNamespace(hex="a" * 32)
            with patch("factory_ops.scenario_replay.uuid.uuid4", return_value=fixed):
                saved = service.save_scenario({"name": "shift A"})
                with self.assertRaisesRegex(ValueError, "overwrite"):
                    service.save_scenario({"name": "shift A"})
            with self.assertRaisesRegex(ValueError, "unsafe"):
                service.replay_frame("../" + saved["scenario_id"], 0)


if __name__ == "__main__":
    unittest.main()
