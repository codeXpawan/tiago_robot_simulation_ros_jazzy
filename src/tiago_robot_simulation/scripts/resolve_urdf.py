#!/usr/bin/env python3
"""
Reads a URDF from stdin (or a file), replaces all package:// URIs
with absolute file:// paths using ament_index, then writes to stdout (or a file).

Usage:
    xacro robot.urdf.xacro | python3 resolve_urdf.py
    python3 resolve_urdf.py input.urdf output.urdf
"""

import re
import sys
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def resolve(content: str) -> str:
    def replacer(match):
        pkg = match.group(1)
        rel = match.group(2)
        try:
            share = get_package_share_directory(pkg)
            abs_path = f"{share}/{rel}"
            # print(f"[resolve_urdf] {pkg}/{rel} -> {abs_path}", file=sys.stderr)
            return f"file://{abs_path}"
        except PackageNotFoundError:
            print(f"[resolve_urdf] WARNING: package not found: {pkg}", file=sys.stderr)
            return match.group(0)  # leave unchanged

    return re.sub(r'package://([^/]+)/([^"<\s]+)', replacer, content)


if __name__ == '__main__':
    if len(sys.argv) == 3:
        # File mode: python3 resolve_urdf.py input.urdf output.urdf
        with open(sys.argv[1], 'r') as f:
            content = f.read()
        resolved = resolve(content)
        with open(sys.argv[2], 'w') as f:
            f.write(resolved)
        print(f"[resolve_urdf] Written to {sys.argv[2]}", file=sys.stderr)

    else:
        # Pipe mode: xacro robot.xacro | python3 resolve_urdf.py
        content = sys.stdin.read()
        sys.stdout.write(resolve(content))