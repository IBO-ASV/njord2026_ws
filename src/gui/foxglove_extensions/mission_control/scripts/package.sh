#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cp "${package_dir}/src/extension.js" "${package_dir}/dist/extension.js"
cd "${package_dir}"
zip -q -r -FS mission-control-0.1.0.foxe package.json dist
