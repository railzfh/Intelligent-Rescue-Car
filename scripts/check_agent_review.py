"""Validate an agent's recorded review; this script does not perform AI review."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


REVIEW_BLOCK = re.compile(r"^```agent-review[ \t]*\r?\n(.*?)^```[ \t]*\r?$", re.MULTILINE | re.DOTALL)
COMMIT_SHA = re.compile(r"[0-9a-f]{40}")
HIGH_RISK_FILES = {
    "agents.md", "claude.md", "contributing.md", ".gitignore", ".gitattributes",
    ".mxproject", "cmakepresets.json", "cmakelists.txt", "cmakelists_template.txt",
}
HIGH_RISK_PREFIXES = (
    ".github/", ".githooks/", ".cursor/", "scripts/", "tests/",
    "core/", "drivers/", "middlewares/", "usb_device/", "cmake/", "user_config/",
    "user_file/1_middleware/driver/", "user_file/1_middleware/system/",
    "user_file/1_middleware/algorithm/pid/", "user_file/2_device/motor/",
    "user_file/2_device/bsp/power/", "user_file/3_robot/", "user_file/4_task/",
)


def high_risk_paths(paths):
    return [path for path in paths if (
        path.lower() in HIGH_RISK_FILES
        or path.lower().startswith(HIGH_RISK_PREFIXES)
        or path.lower().endswith((".ioc", ".ld", ".s"))
    )]


def changed_paths(base_sha, head_sha, cwd=None):
    # No rename detection: both the removed sensitive path and new path must be inspected.
    output = subprocess.check_output(
        ["git", "diff", "--no-renames", "--name-only", "-z", f"{base_sha}...{head_sha}"],
        cwd=cwd, stderr=subprocess.PIPE,
    )
    return [path.decode("utf-8", errors="surrogateescape") for path in output.split(b"\x00") if path]


def check_pull_request(pr, expected_head, expected_base, paths=None):
    if not COMMIT_SHA.fullmatch(expected_head) or not COMMIT_SHA.fullmatch(expected_base):
        raise ValueError("Expected full PR head/base commit SHAs from GitHub.")
    if not isinstance(pr, dict) or pr.get("state") != "open":
        raise ValueError("Current PR must be open.")
    if pr.get("head", {}).get("sha") != expected_head or pr.get("base", {}).get("sha") != expected_base:
        raise ValueError("Workflow event is stale: rerun checks for the current PR head/base.")
    paths = changed_paths(expected_base, expected_head) if paths is None else paths
    risky = high_risk_paths(paths)
    if risky:
        validate_review(pr.get("body") or "", expected_head, expected_base)
    return risky


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pr-json", type=Path, required=True, help="current PR metadata fetched from GitHub")
    args = parser.parse_args()
    try:
        risky = check_pull_request(
            json.loads(args.pr_json.read_text(encoding="utf-8")),
            os.environ.get("REVIEW_HEAD_SHA", ""),
            os.environ.get("REVIEW_BASE_SHA", ""),
        )
    except ValueError as exc:
        # Never echo the PR body or values supplied by untrusted PR content.
        print(f"Agent review record failed: {exc}", file=sys.stderr)
        return 1
    except (OSError, subprocess.CalledProcessError):
        print("Agent review record failed: could not load current PR or changed paths.", file=sys.stderr)
        return 1
    if risky:
        print(f"High-risk paths changed ({len(risky)}): current agent review record passed.")
    else:
        print("No high-risk paths changed: independent agent review is not required. Other CI still applies.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
