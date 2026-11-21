// Run from the repository root: gjs this-file, or node this-file.
const sourcePath = 'src/gui/foxglove_extensions/gnss_map_telemetry/src/extension.js';
const source = typeof require === 'function'
  ? require('fs').readFileSync(sourcePath, 'utf8')
  : new TextDecoder().decode(imports.gi.GLib.file_get_contents(sourcePath)[1]);
const api = new Function('module', `${source}\nreturn module.exports;`)({exports: {}});
function check(value, message) { if (!value) throw new Error(message); }
const north = {x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2};
const east = {x: 0, y: 0, z: 0, w: 1};
const transforms = new Map([['base_link', {parent: 'odom', rotation: east}]]);
check(Math.abs(api.quaternionToBearingDegrees(north)) < 1e-9, 'ENU north must point north on map');
check(api.quaternionToBearingDegrees(east) === 90, 'ENU east must point east on map');
const gnss = {orientation: north, timeSeconds: 10};
check(api.quaternionToBearingDegrees(api.selectMapOrientation(gnss, transforms, 11)) < 1e-9,
      'fresh absolute GNSS must take precedence over local TF');
check(api.selectMapOrientation(gnss, transforms, 13) === undefined, 'stale GNSS must not fall back to local TF');
check(api.selectMapOrientation(gnss, transforms, 9) === undefined, 'future GNSS must be hidden on replay seek');
check(api.selectMapOrientation({orientation: {x:0,y:0,z:0,w:0}, timeSeconds:10}, transforms, 10) === undefined,
      'zero quaternion must not produce a heading');
check(api.quaternionToBearingDegrees(api.selectMapOrientation(undefined, transforms, 11)) === 90,
      'legacy TF remains available before any GNSS heading is received');
if (typeof print === 'function') print('7 heading checks passed'); else console.log('7 heading checks passed');
check(api.qualityLabel({value: 4, timeSeconds: 10}, 11) === 'RTK FIX (4)', 'quality 4 is RTK FIX');
check(api.qualityLabel({value: 5, timeSeconds: 10}, 11) === 'RTK FLOAT (5)', 'quality 5 is RTK FLOAT');
check(api.qualityLabel({value: 0, timeSeconds: 10}, 11) === 'NO FIX (0)', 'quality 0 is no fix');
check(api.qualityLabel({value: 2, timeSeconds: 10}, 11) === 'DGPS (2)', 'quality 2 is DGPS');
check(api.qualityLabel({value: 9, timeSeconds: 10}, 11) === 'UNKNOWN (9)', 'unknown quality is explicit');
check(api.qualityLabel({value: 4, timeSeconds: 10}, 14) === '--', 'stale quality is hidden');
check(api.qualityLabel({value: 4, timeSeconds: 10}, 9) === '--', 'future quality is hidden after seek');
