#!/bin/bash
set -euo pipefail

# geometry2 0.25.24 fixes the waitForTransform/testTransformableRequests ABBA deadlock.
minimum=0.25.24
versions=$(dpkg-query -W -f='${Package} ${Version} ${db:Status-Status}\n' \
    'ros-humble-tf2*' ros-humble-geometry2)
failed=0
for required in ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-msgs ros-humble-geometry2; do
    if ! grep -q "^${required} .* installed$" <<< "$versions"; then
        echo "Missing required TF2 package: $required" >&2
        failed=1
    fi
done
while read -r package version status; do
    [[ "$status" == installed ]] || continue
    if ! dpkg --compare-versions "$version" ge "$minimum"; then
        echo "Unsafe TF2 package: $package=$version; require >= $minimum. Rebuild ros2-base." >&2
        failed=1
    fi
done <<< "$versions"
(( failed == 0 )) || exit 1
printf 'TF2 version check passed (minimum %s):\n%s\n' "$minimum" "$versions"
