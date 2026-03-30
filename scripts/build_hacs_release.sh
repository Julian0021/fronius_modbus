#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-$repo_root/dist}"
staging_dir="$output_dir/_staging"
package_root="custom_components/fronius_modbus"
archive_name="$(
    python - "$repo_root/hacs.json" <<'PY'
from __future__ import annotations

import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    data = json.load(handle)

if not data.get("zip_release"):
    raise SystemExit("hacs.json must set zip_release to true")

filename = data.get("filename")
if not filename:
    raise SystemExit("hacs.json must set filename when zip_release is true")

print(filename)
PY
)"

rm -rf "$staging_dir"
mkdir -p "$staging_dir" "$output_dir"
cp -R "$repo_root/$package_root/." "$staging_dir/"

find "$staging_dir" -type d -name "__pycache__" -prune -exec rm -rf {} +
find "$staging_dir" -type f \( -name "*.pyc" -o -name ".DS_Store" \) -delete

(
    cd "$staging_dir"
    rm -f "$output_dir/$archive_name"
    zip -qr "$output_dir/$archive_name" .
)

python - "$output_dir/$archive_name" <<'PY'
from __future__ import annotations

import sys
import zipfile

archive_path = sys.argv[1]

with zipfile.ZipFile(archive_path) as archive:
    names = archive.namelist()

if not names:
    raise SystemExit("package is empty")

if "manifest.json" not in names:
    raise SystemExit("manifest.json is missing from package")

unexpected = [
    name
    for name in names
    if name.startswith("custom_components/")
    or name.startswith("tests/")
    or name.startswith(".github/")
]
if unexpected:
    raise SystemExit(f"unexpected package entries: {unexpected}")

print(archive_path)
PY
