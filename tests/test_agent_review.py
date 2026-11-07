import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from check_agent_review import changed_paths, check_pull_request, high_risk_paths, validate_review


class AgentReviewTest(unittest.TestCase):
    def setUp(self):
        self.head = "a" * 40
        self.base = "b" * 40
        self.report = {
            "head_sha": self.head, "base_sha": self.base,
            "reviewer": "independent-agent/session-123", "verdict": "pass",
            "summary": "No blocking findings in the final diff.",
            "checks": ["Review changed control flow and test coverage."], "blocking_findings": [],
        }

    def body(self):
        return "PR description\n\n```agent-review\n" + json.dumps(self.report) + "\n```\n"

    def test_current_review_passes_in_lf_and_crlf_body(self):
        self.assertEqual(validate_review(self.body(), self.head, self.base), self.report)
        self.assertEqual(validate_review(self.body().replace("\n", "\r\n"), self.head, self.base), self.report)

    def test_new_head_or_new_base_invalidates_review(self):
        for head, base in (("c" * 40, self.base), (self.head, "c" * 40)):
            with self.subTest(head=head, base=base), self.assertRaises(ValueError):
                validate_review(self.body(), head, base)

    def test_missing_duplicate_or_invalid_block_fails(self):
        for body in ("", self.body() * 2, "```agent-review\ninvalid\n```", "```agent-review\n[]\n```"):
            with self.subTest(body=body), self.assertRaises(ValueError):
                validate_review(body, self.head, self.base)

    def test_pending_failed_or_blocking_review_fails(self):
        for patch in ({"verdict": "pending"}, {"verdict": "fail"}, {"blocking_findings": ["P1 issue"]}):
            with self.subTest(patch=patch):
                self.setUp()
                self.report.update(patch)
                with self.assertRaises(ValueError):
                    validate_review(self.body(), self.head, self.base)

    def test_placeholder_or_empty_fields_fail(self):
        for patch in ({"reviewer": " "}, {"summary": ""}, {"checks": []}, {"checks": [""]}, {"blocking_findings": None}):
            with self.subTest(patch=patch):
                self.setUp()
                self.report.update(patch)
                with self.assertRaises(ValueError):
                    validate_review(self.body(), self.head, self.base)
        with self.assertRaises(ValueError):
            validate_review(self.body(), "a" * 7, self.base)

    def live_pr(self, body=None):
        return {"state": "open", "head": {"sha": self.head}, "base": {"sha": self.base}, "body": body}

    def test_ordinary_changes_do_not_require_review(self):
        paths = ["docs/usage.md", "README.md", "User_File/2_Device/Plotter/Vofa/dvc_vofa.cpp"]
        self.assertEqual(check_pull_request(self.live_pr(), self.head, self.base, paths), [])

    def test_hardware_motor_and_guard_paths_require_review(self):
        for path in ("dm02_test.ioc", "Core/Src/fdcan.c", "startup_chip.s", "layout.ld", "CMakeLists.txt",
                     "User_File/2_Device/Motor/Motor_DJI/dvc_motor_dji.cpp", "User_File/2_Device/BSP/Power/bsp_power.cpp",
                     "User_File/4_Task/tsk_config_and_callback.cpp", ".github/workflows/ci.yml", "AGENTS.md", "scripts/guard.py"):
            with self.subTest(path=path):
                self.assertTrue(high_risk_paths([path]))
                with self.assertRaises(ValueError):
                    check_pull_request(self.live_pr(), self.head, self.base, [path])
                self.assertEqual(check_pull_request(self.live_pr(self.body()), self.head, self.base, [path]), [path])

    def test_rerun_reads_live_body_and_rejects_stale_event(self):
        # A previously passing body must not be reused after a live edit or new commit.
        pr = self.live_pr(self.body())
        self.assertTrue(check_pull_request(pr, self.head, self.base, ["AGENTS.md"]))
        pr["body"] = "review removed"
        with self.assertRaises(ValueError):
            check_pull_request(pr, self.head, self.base, ["AGENTS.md"])
        pr = self.live_pr(self.body())
        pr["head"]["sha"] = "c" * 40
        with self.assertRaises(ValueError):
            check_pull_request(pr, self.head, self.base, ["docs/usage.md"])
        pr = self.live_pr(self.body())
        pr["base"]["sha"] = "c" * 40
        with self.assertRaises(ValueError):
            check_pull_request(pr, self.head, self.base, ["AGENTS.md"])

    def test_deleted_or_renamed_sensitive_path_still_requires_review(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp)
            def git(*args):
                return subprocess.check_output(["git", "-C", temp, *args], stderr=subprocess.PIPE).decode().strip()
            git("init")
            git("config", "user.name", "Review test")
            git("config", "user.email", "review@example.invalid")
            sensitive = repo / "Core/Src/fdcan.c"
            sensitive.parent.mkdir(parents=True)
            sensitive.write_text("original control code\n")
            git("add", ".")
            git("commit", "-m", "base")
            base = git("rev-parse", "HEAD")
            (repo / "docs").mkdir()
            sensitive.rename(repo / "docs/old-control.txt")
            git("add", "--all")
            git("commit", "-m", "move sensitive file")
            head = git("rev-parse", "HEAD")
            paths = changed_paths(base, head, cwd=temp)
            self.assertIn("Core/Src/fdcan.c", paths)
            self.assertEqual(high_risk_paths(paths), ["Core/Src/fdcan.c"])


if __name__ == "__main__":
    unittest.main()
