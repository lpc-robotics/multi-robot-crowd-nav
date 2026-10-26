#!/usr/bin/env python3
"""Exercise cleanup and startup exclusion using private real process groups."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("active_run", SCRIPTS / "check_active_run.py")
active_run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(active_run)


class ProcessLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="arena-process-test-")
        self.root = Path(self.temp.name)
        (self.root / "install").mkdir()
        self.worker = self.root / "install/worker.py"
        self.worker.write_text("import time\ntime.sleep(120)\n")
        self.processes = []

    def tearDown(self):
        for proc in self.processes:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
        self.temp.cleanup()

    def launch(self, domain):
        env = {**os.environ, "ROS_DOMAIN_ID": str(domain),
               "ARENA_MULTI_WS": str(self.root), "ARENA_MULTI_RUN_DIR": str(self.root / "run")}
        proc = subprocess.Popen([sys.executable, str(self.worker)], env=env,
                                start_new_session=True)
        self.processes.append(proc)
        return proc

    def test_same_domain_excludes_second_start(self):
        proc = self.launch(229)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "229"}):
            self.assertIn(proc.pid, active_run.active_runs())

    def test_different_domain_is_independent(self):
        proc = self.launch(228)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "229"}):
            self.assertNotIn(proc.pid, active_run.active_runs())

    def test_cleanup_terminates_orphan_after_leader_exits(self):
        # The leader exits immediately, exactly the failure seen after Ctrl+C.
        orphan = self.root / "orphan.pid"
        self.worker.write_text(
            "import os,time\nfrom pathlib import Path\n"
            "child=os.fork()\n"
            "if child: os._exit(0)\n"
            f"Path({str(orphan)!r}).write_text(str(os.getpid()))\n"
            "time.sleep(120)\n")
        proc = self.launch(229)
        proc.wait()
        deadline = time.monotonic() + 3
        while not orphan.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertTrue(orphan.exists())
        result = subprocess.run(
            ["bash", "-c", 'source "$1"; arena_stop_group "$2"; ! arena_group_alive "$2"',
             "test", str(SCRIPTS / "process_group.sh"), str(proc.pid)],
            capture_output=True, text=True, timeout=35)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
