"""Install this repository's guard without dropping Git LFS's pre-push step."""
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    try:
        top = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], cwd=root, text=True
        ).strip()
        if Path(top).resolve() != root:
            raise ValueError("Run this installer in the intended project repository.")
        current = subprocess.run(
            ["git", "config", "--get", "core.hooksPath"], cwd=root,
            text=True, capture_output=True, check=False
        ).stdout.strip()
        if current and current != ".githooks":
            raise ValueError("An existing hooksPath is configured; integrate it explicitly first.")
        if current != ".githooks":
            old_path = subprocess.check_output(
                ["git", "rev-parse", "--git-path", "hooks/pre-push"], cwd=root, text=True
            ).strip()
            old_hook = root / old_path
            if old_hook.exists():
                lines = old_hook.read_text(encoding="utf-8").strip().splitlines()
                # Accept the standard standalone LFS hook; preserve any other custom hook.
                lfs_only = (
                    len(lines) == 3 and lines[0] == "#!/bin/sh"
                    and lines[1].startswith("command -v git-lfs >/dev/null 2>&1 || {")
                    and lines[2] == 'git lfs pre-push "$@"'
                )
                if not lfs_only:
                    raise ValueError("A custom pre-push hook exists; integrate it explicitly first.")
        subprocess.run(["git", "lfs", "version"], cwd=root, check=True)
        # Configure LFS filters without installing its standalone pre-push hook.
        subprocess.run(["git", "lfs", "install", "--local", "--skip-repo"], cwd=root, check=True)
        subprocess.run(["git", "config", "--local", "core.hooksPath", ".githooks"], cwd=root, check=True)
        subprocess.run(["git", "config", "--local", "push.default", "simple"], cwd=root, check=True)
        subprocess.run(["git", "config", "--local", "pull.ff", "only"], cwd=root, check=True)
        hook = root / ".githooks" / "pre-push"
        hook.chmod(hook.stat().st_mode | 0o111)
        print("Installed: repository push guard + Git LFS. GitHub rulesets must be enabled separately.")
        return 0
    except (subprocess.CalledProcessError, ValueError, OSError) as exc:
        print(f"Hook installation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
