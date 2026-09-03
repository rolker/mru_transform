#!/bin/bash
# ci_local upstream pruning hook (see .agent/scripts/ci_local.sh, #577).
# Run inside the CI container with cwd /ci/upstream_ws, after the
# upstream.repos clones and before the upstream build.
#
# upstream.repos declares unh_marine_autonomy because mru_transform needs ONE
# package from it: marine_vertical_datum, the ROS-free vertical-datum library
# chart_datum_node consumes (ADR-0010 D6, #41). vcs clones whole repos, so
# without pruning colcon also tries to build the other fifteen packages in
# that repo -- several of which depend on repos we do NOT clone (e.g.
# mission_manager_interfaces needs marine_nav_interfaces from
# unh_marine_navigation). Those fail, and take the run down with them, even
# though nothing mru_transform builds touches them.
#
# COLCON_IGNORE every package in that repo except the one we need. This is
# industrial_ci's AFTER_SETUP_UPSTREAM_WORKSPACE pruning, done locally.
# geographic_info is NOT pruned: mru_transform needs geodesy from it.
set -euo pipefail

REPO_DIR="src/unh_marine_autonomy"
KEEP="marine_vertical_datum"

if [[ ! -d "$REPO_DIR" ]]; then
    echo "ci_local_upstream_extra: $REPO_DIR not present; nothing to prune" >&2
    exit 0
fi

pruned=0
while IFS= read -r manifest; do
    pkg_dir=$(dirname "$manifest")
    if [[ "$(basename "$pkg_dir")" == "$KEEP" ]]; then
        continue
    fi
    touch "$pkg_dir/COLCON_IGNORE"
    pruned=$((pruned + 1))
done < <(find "$REPO_DIR" -name package.xml)

echo "ci_local_upstream_extra: kept $KEEP, pruned $pruned package(s) from $REPO_DIR"

if [[ ! -f "$REPO_DIR/$KEEP/package.xml" ]] && \
   ! find "$REPO_DIR" -path "*/$KEEP/package.xml" -print -quit | grep -q .; then
    echo "ci_local_upstream_extra: ERROR: $KEEP not found in $REPO_DIR" >&2
    exit 1
fi
