"""Exercise push decisions with real Git history and isolated repositories."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from check_repository import MATERIAL_PACK, REQUIRED_FILES, check_ref, inspect_blob


class RepositoryGuardsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.repo = Path(self.temp.name) / "repo"
        self.repo.mkdir()
        self.previous = Path.cwd()
        os.chdir(self.repo)
        self.addCleanup(self.temp.cleanup)
        self.addCleanup(os.chdir, self.previous)
        self.git("init", "-b", "feat/alice/control")
        self.git("config", "user.name", "Guard Test")
        self.git("config", "user.email", "guard@example.invalid")
        self.git("config", "core.autocrlf", "false")
        # Policy tests inspect literal Git blobs, independent of machine-wide LFS filters.
        self.git("config", "filter.lfs.clean", "")
        self.git("config", "filter.lfs.smudge", "")
        self.git("config", "filter.lfs.process", "")
        self.git("config", "filter.lfs.required", "false")
        for filename in REQUIRED_FILES:
            p = self.repo / filename
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("# fixture\n", encoding="utf-8")
        pack = b"safe material test fixture"
        pointer = (
            "version https://git-lfs.github.com/spec/v1\n"
            f"oid sha256:{hashlib.sha256(pack).hexdigest()}\nsize {len(pack)}\n"
        )
        (self.repo / MATERIAL_PACK).write_bytes(pointer.encode("utf-8"))
        (self.repo / ".gitattributes").write_text(
            f"/{MATERIAL_PACK} filter=lfs diff=lfs merge=lfs -text\n", encoding="utf-8"
        )
        self.initial = self.commit("initial fixture")

    def git(self, *args):
        return subprocess.check_output(["git", *args], stderr=subprocess.PIPE).decode().strip()

    def commit(self, message):
        self.git("add", "--all")
        self.git("commit", "-m", message)
        return self.git("rev-parse", "HEAD")

    def guard(self, local=None, remote=None, old=None, sha=None):
        ref = "refs/heads/feat/alice/control"
        record = f"{local or ref} {sha or self.git('rev-parse', 'HEAD')} {remote or ref} {old or '0' * 40}\n"
        return subprocess.run(
            [sys.executable, str(ROOT / "scripts/pre_push.py"), "origin", "unused-url"],
            input=record, text=True, capture_output=True, check=False,
        )

    def test_clean_task_branch_and_fast_forward_allowed(self):
        result = self.guard()
        self.assertEqual(result.returncode, 0, result.stderr)
        Path("control.c").write_text("int enabled = 0;\n")
        self.commit("add control")
        self.assertEqual(self.guard(old=self.initial).returncode, 0)

    def test_protected_branches_and_tags_blocked(self):
        for ref in ("refs/heads/main", "refs/heads/master", "refs/heads/develop", "refs/heads/release/v1", "refs/tags/v1"):
            with self.subTest(ref=ref):
                self.assertNotEqual(self.guard(remote=ref).returncode, 0)

    def test_default_branch_blocked_even_if_it_looks_like_a_task_branch(self):
        self.git("symbolic-ref", "refs/remotes/origin/HEAD", "refs/remotes/origin/feat/alice/control")
        self.assertNotEqual(self.guard().returncode, 0)

    def test_deletion_and_renamed_target_blocked(self):
        self.assertNotEqual(self.guard(sha="0" * 40).returncode, 0)
        self.assertNotEqual(self.guard(remote="refs/heads/fix/bob/other").returncode, 0)
        self.assertNotEqual(self.guard(local="HEAD").returncode, 0)

    def test_history_rewrite_blocked(self):
        Path("control.c").write_text("int value = 1;\n")
        newest = self.commit("new work")
        self.assertNotEqual(self.guard(sha=self.initial, old=newest).returncode, 0)

    def test_unknown_remote_commit_fails_closed(self):
        self.assertNotEqual(self.guard(old="f" * 40).returncode, 0)

    def test_intermediate_secret_cannot_be_hidden_by_a_later_delete(self):
        secret = "ghp_" + "a" * 36  # synthetic, never a real credential
        Path("control.c").write_text(f'const char *value = "{secret}";\n')
        self.commit("accidental credential")
        Path("control.c").unlink()
        self.commit("remove credential")
        self.assertEqual(check_ref("HEAD"), [])
        result = self.guard(old=self.initial)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn(secret, result.stderr)

    def test_staged_check_does_not_read_unstaged_secret(self):
        secret = "ghp_" + "b" * 36
        Path("control.c").write_text("int enabled = 0;\n")
        self.git("add", "control.c")
        Path("control.c").write_text(f'const char *value = "{secret}";\n')
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts/check_repository.py"), "--staged"],
            capture_output=True, text=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_guard_file_blocked(self):
        Path("AGENTS.md").unlink()
        self.commit("remove rules")
        self.assertNotEqual(self.guard(old=self.initial).returncode, 0)

    def test_sensitive_and_generated_paths_blocked(self):
        for path in (".env", "cfg/.env.production", "cfg/token.pem", ".aws/credentials", ".npmrc", ".vscode/settings.json", "build/a.o", "other.7z"):
            with self.subTest(path=path):
                self.assertTrue(inspect_blob(path, "100644", 1, lambda: b"x"))
        self.assertFalse(inspect_blob(".env.example", "100644", 16, lambda: b"TOKEN=CHANGE_ME\n"))
        self.assertFalse(inspect_blob("lib/math.a", "100644", 8, lambda: b"!<arch>\n\x00"))

    def test_archive_requires_pointer_and_lfs_attributes(self):
        Path(MATERIAL_PACK).write_bytes(b"not an LFS pointer")
        self.commit("raw archive")
        self.assertTrue(check_ref("HEAD"))
        Path(MATERIAL_PACK).write_bytes((
            "version https://git-lfs.github.com/spec/v1\noid sha256:" + "0" * 64 + "\nsize 1\n"
        ).encode("utf-8"))
        Path(".gitattributes").write_text("* text=auto\n")
        self.commit("missing LFS rule")
        self.assertTrue(any("LFS rule" in error for error in check_ref("HEAD")))

    def test_large_blob_and_symlink_blocked(self):
        self.assertTrue(inspect_blob("large.c", "100644", 11 * 1024 * 1024, lambda: b""))
        self.assertTrue(inspect_blob("linked.c", "120000", 1, lambda: b"x"))

    def test_installer_is_local_and_hook_keeps_lfs(self):
        if subprocess.run(["git", "lfs", "version"], capture_output=True).returncode:
            self.skipTest("Git LFS unavailable")
        for relative in ("scripts/install_hooks.py", "scripts/pre_push.py", "scripts/check_repository.py", ".githooks/pre-push"):
            dest = self.repo / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, dest)
        for key in ("clean", "smudge", "process", "required"):
            self.git("config", "--unset", "filter.lfs." + key)
        result = subprocess.run(
            [sys.executable, "scripts/install_hooks.py"], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git("config", "--local", "core.hooksPath"), ".githooks")
        hook = Path(".githooks/pre-push").read_bytes()
        self.assertNotIn(b"\r\n", hook)
        self.assertIn(b'git lfs pre-push "$@"', hook)

    def test_actual_hook_uploads_lfs_and_rejects_main(self):
        if subprocess.run(["git", "lfs", "version"], capture_output=True).returncode:
            self.skipTest("Git LFS unavailable")
        # A new history contains a real LFS object rather than the policy fixture pointer.
        self.git("checkout", "--orphan", "feat/alice/lfs")
        for relative in ("scripts/install_hooks.py", "scripts/pre_push.py", "scripts/check_repository.py", ".githooks/pre-push"):
            dest = self.repo / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, dest)
        for key in ("clean", "smudge", "process", "required"):
            self.git("config", "--unset", "filter.lfs." + key)
        subprocess.run([sys.executable, "scripts/install_hooks.py"], check=True, capture_output=True)
        Path(MATERIAL_PACK).write_bytes(b"actual test archive bytes\n")
        self.commit("real LFS material")
        remote = Path(self.temp.name) / "remote.git"
        self.git("init", "--bare", str(remote))
        self.git("remote", "add", "origin", str(remote))
        pushed = subprocess.run(
            ["git", "push", "origin", "feat/alice/lfs"], capture_output=True, text=True
        )
        self.assertEqual(pushed.returncode, 0, pushed.stdout + pushed.stderr)
        self.assertTrue(any((remote / "lfs/objects").rglob("*")))
        blocked = subprocess.run(
            ["git", "push", "origin", "feat/alice/lfs:main"], capture_output=True, text=True
        )
        self.assertNotEqual(blocked.returncode, 0)
        self.assertIn("Push blocked", blocked.stderr)

    def test_installer_preserves_custom_hook_configuration(self):
        dest = self.repo / "scripts/install_hooks.py"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / "scripts/install_hooks.py", dest)
        hook = self.repo / ".git/hooks/pre-push"
        hook.write_bytes(b"#!/bin/sh\necho custom\n")
        result = subprocess.run(
            [sys.executable, str(dest)], capture_output=True, text=True
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(hook.read_bytes(), b"#!/bin/sh\necho custom\n")
        self.git("config", "--local", "core.hooksPath", "custom-hooks")
        result = subprocess.run(
            [sys.executable, str(dest)], capture_output=True, text=True
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.git("config", "--local", "core.hooksPath"), "custom-hooks")


class RulesetTest(unittest.TestCase):
    def test_rulesets_require_external_approval_checks_and_no_bypass(self):
        ruleset = json.loads((ROOT / ".github/rulesets/protect-main.json").read_text())
        self.assertEqual(ruleset["enforcement"], "active")
        self.assertEqual(ruleset["bypass_actors"], [])
        rules = {r["type"]: r.get("parameters", {}) for r in ruleset["rules"]}
        self.assertIn("deletion", rules)
        self.assertIn("non_fast_forward", rules)
        self.assertEqual(rules["pull_request"]["required_approving_review_count"], 1)
        self.assertTrue(rules["pull_request"]["dismiss_stale_reviews_on_push"])
        self.assertTrue(rules["pull_request"]["require_last_push_approval"])
        self.assertEqual(
            {c["context"] for c in rules["required_status_checks"]["required_status_checks"]},
            {"Repository policy", "Firmware build"},
        )
        all_branches = json.loads((ROOT / ".github/rulesets/no-force-push.json").read_text())
        self.assertEqual(all_branches["conditions"]["ref_name"]["include"], ["~ALL"])
        self.assertEqual(all_branches["bypass_actors"], [])


if __name__ == "__main__":
    unittest.main()
