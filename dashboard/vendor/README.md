# Vendored dashboard dependencies

The dashboard used to load its libraries and font from jsDelivr, unpkg and Google Fonts, so on a rover network without internet (the normal case at a competition site) it came up blank. Everything it needs is now committed here, pinned to exact versions (about 930 KB) and served by the same local web server as the dashboard.

| Package | Version | Files used | Licence |
|---|---|---|---|
| eventemitter2 | 6.4.9 | `lib/eventemitter2.js` (the package ships no minified build; the old `.min.js` URL was minified on the fly by the CDN) | MIT |
| roslib | 1.4.1 (was the floating `@1`) | `build/roslib.min.js` | BSD-3-Clause |
| three | 0.89.0 | `build/three.min.js` | MIT |
| ros3d | 0.17.0 | `build/ros3d.min.js` | BSD-3-Clause |
| leaflet | 1.9.4 | `dist/leaflet.{js,css}`, `dist/images/*.png` | BSD-2-Clause |
| @fontsource/outfit | 5.3.0 | Latin 300/400/600/800 `.woff2` (see `fonts.css`) | SIL OFL 1.1 |

Each package's own licence file is next to its files.

**Integrity.** `fetch.sh` downloads each file from jsDelivr and compares its SHA-256 with the hash jsDelivr publishes for that exact version; a mismatch is deleted and reported. `fetch.sh --verify` checks the committed copies without downloading them (it needs internet for the published hashes). To upgrade a library, change its version in `fetch.sh` and in `index.html`, run `fetch.sh`, and test the 3D view (ros3d 0.17 needs three 0.89; the `EventDispatcher.apply` polyfill in `index.html` bridges them).

## Map tiles

Map tiles are the one thing still online by default. The dashboard first looks for a local tile folder:

```
dashboard/tiles/tiles.json          {"maxZoom": 19, "attribution": "..."}
dashboard/tiles/{z}/{x}/{y}.png     standard XYZ (slippy-map) tiles
```

If `tiles/tiles.json` exists, the map uses only the local tiles; otherwise it uses online OpenStreetMap tiles. Without either, the map is blank but the rover marker, track and click-to-target still work.

Do **not** bulk-download tiles from `tile.openstreetmap.org`: its tile usage policy forbids it. To prepare the competition area, either render tiles yourself from OpenStreetMap data (for example an extract of the site area with a tile renderer), or use a provider whose terms allow offline use, or export your own imagery (drone orthophoto, satellite imagery you are licensed to use) to XYZ tiles with GDAL (`gdal2tiles.py --xyz`). Put the result in `dashboard/tiles/` and set the attribution the source requires in `tiles.json`. Tiles are not committed (large, and licence-dependent); `dashboard/tiles/` is in `.gitignore`.
