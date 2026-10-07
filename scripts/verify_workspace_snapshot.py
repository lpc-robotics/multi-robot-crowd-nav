#!/usr/bin/env python3
"""Read-only comparison of a restored workspace against a full snapshot manifest."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat


def manifest(root):
    result = {}
    def visit(path):
        metadata = path.lstat()
        item = {"mode": stat.S_IMODE(metadata.st_mode), "uid": metadata.st_uid, "gid": metadata.st_gid}
        if stat.S_ISLNK(metadata.st_mode):
            item.update(type="link", target=os.readlink(path), mtime_ns=metadata.st_mtime_ns)
        elif stat.S_ISREG(metadata.st_mode):
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024*1024), b""):
                    digest.update(block)
            item.update(type="file", size=metadata.st_size, sha256=digest.hexdigest(), mtime_ns=metadata.st_mtime_ns)
        elif stat.S_ISDIR(metadata.st_mode):
            item["type"] = "directory"
        else:
            raise ValueError(f"unsupported filesystem object: {path}")
        result[str(path.relative_to(root))] = item
        if item["type"] == "directory":
            for child in sorted(path.iterdir()):
                visit(child)
    visit(root)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path)
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()
    expected = json.loads(args.manifest.read_text())
    actual = manifest(args.workspace.absolute())
    changed = sorted(key for key in set(expected) | set(actual) if expected.get(key) != actual.get(key))
    print(json.dumps({"passed": not changed, "entries": len(expected), "differences": changed}, indent=2))
    return bool(changed)


if __name__ == "__main__":
    raise SystemExit(main())
