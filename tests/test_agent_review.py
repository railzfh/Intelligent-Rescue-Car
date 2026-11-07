import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from check_agent_review import validate_review


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


if __name__ == "__main__":
    unittest.main()
