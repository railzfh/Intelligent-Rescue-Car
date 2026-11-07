"""Check committed/index blobs, not just .gitignore. Never print secret values."""
import argparse
from pathlib import PurePosixPath
import re
import subprocess
import sys


MATERIAL_PACK = "资料.7z"
MAX_BLOB_BYTES = 10 * 1024 * 1024
REQUIRED_FILES = (
    "AGENTS.md", "CONTRIBUTING.md", ".gitignore", ".gitattributes",
    ".github/workflows/ci.yml", ".github/rulesets/protect-main.json",
    ".github/rulesets/no-force-push.json", ".githooks/pre-push",
    "scripts/check_repository.py", "scripts/pre_push.py", MATERIAL_PACK,
)
BLOCKED_DIRS = {
    "build", ".settings", ".vscode", ".idea", "__pycache__", ".git",
    ".ssh", ".aws", ".azure", "secrets", "credentials",
}
BLOCKED_NAMES = {
    ".env", "id_rsa", "id_ed25519", ".git-credentials", "credentials.json", "secrets.json",
    ".project", ".cproject", "cmakeuserpresets.json",
    ".netrc", ".npmrc", ".pypirc", "credentials",
}
BLOCKED_SUFFIXES = {
    ".pem", ".key", ".p12", ".pfx", ".o", ".obj", ".elf", ".hex", ".bin", ".map",
    ".pyc", ".log", ".zip", ".rar", ".7z",
}
SECRET_PATTERNS = (
    re.compile(rb"gh[pousr]_[A-Za-z0-9]{36,255}"),
    re.compile(rb"github_pat_[A-Za-z0-9_]{50,}"),
    re.compile(rb"(?:AKIA|ASIA)[0-9A-Z]{16}"),
    re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY-----"),
)
LFS_POINTER = re.compile(
    rb"version https://git-lfs.github.com/spec/v1\noid sha256:[0-9a-f]{64}\nsize [1-9][0-9]*\n"
)


def git(*args):
    return subprocess.check_output(["git", *args], stderr=subprocess.PIPE)


def inspect_blob(path, mode, size, read):
    """Return diagnostics using metadata first; read content only when needed."""
    pure = PurePosixPath(path)
    name = pure.name.lower()
    if mode not in ("100644", "100755"):
        return [f"{path}: symlinks/submodules need an explicit policy change"]
    if path == MATERIAL_PACK:
        if size > 256 or not LFS_POINTER.fullmatch(read()):
            return [f"{path}: must be a Git LFS pointer, not the archive bytes"]
        return []
    if (
        any(part.lower() in BLOCKED_DIRS or part.lower().startswith("cmake-build-") for part in pure.parts[:-1])
        or name in BLOCKED_NAMES
        or (name.startswith(".env.") and name not in (".env.example", ".env.sample"))
        or pure.suffix.lower() in BLOCKED_SUFFIXES
    ):
        return [f"{path}: generated, local, credential, or unapproved archive file"]
    if size > MAX_BLOB_BYTES:
        return [f"{path}: regular Git blob exceeds 10 MiB; review storage before adding"]
    content = read()
    # Binary dependencies such as the required CMSIS-DSP .a library are permitted.
    if b"\x00" not in content[:8192]:
        for pattern in SECRET_PATTERNS:
            match = pattern.search(content)
            if match:
                line = content[:match.start()].count(b"\n") + 1
                return [f"{path}:{line}: possible credential/private key (value hidden)"]
    return []


def check_ref(ref):
    tree = git("ls-tree", "-r", "-l", "-z", ref)
    entries = {}
    errors = []
    for record in tree.split(b"\x00"):
        if not record:
            continue
        metadata, raw_path = record.split(b"\t", 1)
        mode, kind, oid, raw_size = metadata.decode("ascii").split()
        path = raw_path.decode("utf-8", errors="surrogateescape")
        entries[path] = oid
        errors.extend(inspect_blob(
            path, mode, int(raw_size) if kind == "blob" else 0,
            lambda oid=oid: git("cat-file", "blob", oid),
        ))
    for path in REQUIRED_FILES:
        if path not in entries:
            errors.append(f"{path}: required repository file is missing")
    if ".gitattributes" in entries:
        attrs = git("cat-file", "blob", entries[".gitattributes"])
        rule = f"/{MATERIAL_PACK} filter=lfs diff=lfs merge=lfs -text".encode()
        if rule not in [line.strip() for line in attrs.splitlines()]:
            errors.append(".gitattributes: material pack LFS rule is missing")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--staged", action="store_true", help="check the index tree")
    group.add_argument("--ref", default="HEAD", help="check a commit/tree, default HEAD")
    args = parser.parse_args()
    try:
        ref = git("write-tree").decode().strip() if args.staged else args.ref
        errors = check_ref(ref)
    except (subprocess.CalledProcessError, OSError, ValueError) as exc:
        # Do not print stderr from git: it may contain a credential-bearing remote URL.
        print(f"Repository check failed ({type(exc).__name__}). Verify Git/index state.", file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print("Repository policy passed. This basic scan does not certify archive contents or all secret formats.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
