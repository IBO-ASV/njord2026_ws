#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
generated_file="$(mktemp)"

{
  printf '%s\n' '"use strict";' '' 'const L = (() => {' \
    '  const leafletModule = {exports: {}};' \
    '  const module = leafletModule;' \
    '  const exports = leafletModule.exports;'
  cat "${package_dir}/vendor/leaflet.js"
  printf '%s\n' '  return leafletModule.exports;' '})();' ''
  cat "${package_dir}/src/extension.js"
} > "${generated_file}"

mv "${generated_file}" "${package_dir}/dist/extension.js"
cd "${package_dir}"
zip -r -FS gnss-map-telemetry-0.4.0.foxe package.json dist
