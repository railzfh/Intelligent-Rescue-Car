"""Validate an agent's recorded review; this script does not perform AI review."""
import json
import os
import re
import sys


REVIEW_BLOCK = re.compile(r"^```agent-review[ \t]*\r?\n(.*?)^```[ \t]*\r?$", re.MULTILINE | re.DOTALL)
COMMIT_SHA = re.compile(r"[0-9a-f]{40}")


def validate_review(body, head_sha, base_sha):
    if not COMMIT_SHA.fullmatch(head_sha) or not COMMIT_SHA.fullmatch(base_sha):
        raise ValueError("Expected full PR head/base commit SHAs from GitHub.")
    blocks = REVIEW_BLOCK.findall(body)
    if len(blocks) != 1:
        raise ValueError("PR body must contain exactly one agent-review JSON block.")
    try:
        report = json.loads(blocks[0])
    except json.JSONDecodeError:
        raise ValueError("agent-review must contain valid JSON.") from None
    if not isinstance(report, dict):
        raise ValueError("agent-review must be a JSON object.")
    if report.get("head_sha") != head_sha or report.get("base_sha") != base_sha:
        raise ValueError("Review is stale: review the current diff and update head/base SHA.")
    if report.get("verdict") != "pass":
        raise ValueError("Independent agent review has not passed.")
    for field in ("reviewer", "summary"):
        if not isinstance(report.get(field), str) or not report[field].strip():
            raise ValueError(f"Review requires a non-empty {field}.")
    checks = report.get("checks")
    if not isinstance(checks, list) or not checks or any(not isinstance(item, str) or not item.strip() for item in checks):
        raise ValueError("Review must describe the checks actually performed.")
    if report.get("blocking_findings") != []:
        raise ValueError("Resolve blocking findings before merging.")
    return report


def main():
    try:
        validate_review(
            os.environ.get("PR_BODY", ""),
            os.environ.get("REVIEW_HEAD_SHA", ""),
            os.environ.get("REVIEW_BASE_SHA", ""),
        )
    except ValueError as exc:
        # Never echo the PR body or values supplied by untrusted PR content.
        print(f"Agent review record failed: {exc}", file=sys.stderr)
        return 1
    print("Agent review record matches current PR head/base and reports no blocking findings.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
