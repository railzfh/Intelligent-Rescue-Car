"""Fail closed on protected refs, deletions, and non-fast-forward updates."""
import re
import subprocess
import sys

from check_repository import check_ref, git


TASK_BRANCH = re.compile(r"refs/heads/(?:feat|fix|docs|chore|refactor|test)/[A-Za-z0-9-]+/[a-z0-9][a-z0-9._-]*")


def is_zero(sha):
    return bool(sha) and set(sha) == {"0"}


def validate_update(local_ref, local_sha, remote_ref, remote_sha, default_ref):
    if is_zero(local_sha):
        raise ValueError("Remote deletions are reserved for human repository maintenance.")
    if remote_ref == default_ref or not TASK_BRANCH.fullmatch(remote_ref):
        raise ValueError("Push only a type/login/topic task branch; integration branches and tags are blocked.")
    if local_ref != remote_ref:
        raise ValueError("Use the same named local/remote task branch, not HEAD, an object ID, or a renamed target.")
    git("cat-file", "-e", local_sha + "^{commit}")
    if not is_zero(remote_sha):
        # A missing remote object also blocks the push; fetch before retrying.
        git("cat-file", "-e", remote_sha + "^{commit}")
        ancestor = subprocess.run(
            ["git", "merge-base", "--is-ancestor", remote_sha, local_sha],
            capture_output=True, check=False,
        )
        if ancestor.returncode:
            raise ValueError("Non-fast-forward update blocked. Fetch, inspect, and merge the remote changes.")


def main():
    try:
        if len(sys.argv) != 3:
            raise ValueError("This guard must be called by the pre-push hook.")
        remote_name = sys.argv[1]
        # Look up only local symbolic metadata, without displaying or requesting credentials.
        sym = subprocess.run(
            ["git", "symbolic-ref", "--quiet", f"refs/remotes/{remote_name}/HEAD"],
            capture_output=True, text=True, check=False,
        )
        default_ref = sym.stdout.strip().replace(f"refs/remotes/{remote_name}/", "refs/heads/", 1) if sym.returncode == 0 else None
        updates = [line.split() for line in sys.stdin if line.strip()]
        if not updates:
            return 0  # Nothing to push.
        checked = set()
        for fields in updates:
            if len(fields) != 4:
                raise ValueError("Malformed pre-push update record.")
            local_ref, local_sha, remote_ref, remote_sha = fields
            validate_update(local_ref, local_sha, remote_ref, remote_sha, default_ref)
            # Inspect every outgoing commit, including files added then removed before HEAD.
            revisions = [local_sha] if is_zero(remote_sha) else [f"{remote_sha}..{local_sha}"]
            commits = [local_sha, *git("rev-list", *revisions).decode().splitlines()]
            for commit in commits:
                if commit not in checked:
                    errors = check_ref(commit)
                    if errors:
                        raise ValueError("Repository policy failed in an outgoing commit:\n" + "\n".join(errors))
                    checked.add(commit)
        print("Push guard passed; continuing with Git LFS upload.")
        return 0
    except ValueError as exc:
        print(f"Push blocked: {exc}", file=sys.stderr)
        return 1
    except (subprocess.CalledProcessError, OSError):
        print("Push blocked: Git state could not be verified. Fetch and inspect before retrying.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
