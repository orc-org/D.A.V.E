#!/usr/bin/env bash
# Fetch (or, with --verify, only check) the dashboard's vendored libraries.
#
# Every file is pinned to an exact package version and compared against the SHA-256 that jsDelivr
# publishes for that version (data.jsdelivr.com). A mismatch deletes the file and fails.
# Needs internet, curl and python3. The dashboard itself needs neither at run time.
#
#   dashboard/vendor/fetch.sh            download into dashboard/vendor
#   dashboard/vendor/fetch.sh --verify   check the committed copies without downloading them
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERIFY=0; [ "${1:-}" = "--verify" ] && VERIFY=1

FILES="
eventemitter2@6.4.9 lib/eventemitter2.js LICENSE.txt
roslib@1.4.1 build/roslib.min.js LICENSE
three@0.89.0 build/three.min.js LICENSE
ros3d@0.17.0 build/ros3d.min.js LICENSE
leaflet@1.9.4 dist/leaflet.css dist/leaflet.js dist/images/layers.png dist/images/layers-2x.png dist/images/marker-icon.png dist/images/marker-icon-2x.png dist/images/marker-shadow.png LICENSE
@fontsource/outfit@5.3.0 files/outfit-latin-300-normal.woff2 files/outfit-latin-400-normal.woff2 files/outfit-latin-600-normal.woff2 files/outfit-latin-800-normal.woff2 LICENSE
"

FAILS=0
while read -r PKG PATHS; do
    [ -n "$PKG" ] || continue
    META="$(curl -fsS "https://data.jsdelivr.com/v1/packages/npm/$PKG?structure=flat")"
    DIR="$HERE/$(echo "$PKG" | sed -e 's/^@//' -e 's#/#-#')"
    for P in $PATHS; do
        WANT="$(printf '%s' "$META" | python3 -c 'import json,sys; p="/"+sys.argv[1]; print(next((f["hash"] for f in json.load(sys.stdin)["files"] if f["name"]==p), ""))' "$P")"
        [ -n "$WANT" ] || { echo "MISSING in package: $PKG/$P"; FAILS=$((FAILS+1)); continue; }
        OUT="$DIR/$P"
        if [ "$VERIFY" -eq 0 ]; then
            mkdir -p "$(dirname "$OUT")"
            curl -fsS "https://cdn.jsdelivr.net/npm/$PKG/$P" -o "$OUT"
        fi
        HAVE="$( [ -f "$OUT" ] && openssl dgst -sha256 -binary "$OUT" | base64 || echo none)"
        if [ "$HAVE" = "$WANT" ]; then
            echo "ok    $PKG/$P"
        else
            echo "BAD   $PKG/$P"; FAILS=$((FAILS+1))
            [ "$VERIFY" -eq 0 ] && rm -f "$OUT"
        fi
    done
done <<< "$FILES"
[ "$FAILS" -eq 0 ] && echo "all files match" || { echo "$FAILS file(s) failed"; exit 1; }
