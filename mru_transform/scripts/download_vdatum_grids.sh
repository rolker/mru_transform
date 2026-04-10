#!/usr/bin/env bash
# download_vdatum_grids.sh — Download and cache VDatum + PROJ grids
#
# Downloads:
#   1. PROJ GEOID18 grid (us_noaa_g2018u0.tif, ~16MB) via projsync
#   2. NOAA VDatum regional grids (vdatum_regional_*.zip, ~849MB)
#      Extracts *_mllw.gtx + metadata to the cache
#
# Cache location: ~/.cache/mru_transform/ (default, override with $VDATUM_CACHE_DIR)
# Idempotent — skips downloads if cache is populated.
#
# Usage:
#   ./download_vdatum_grids.sh
#   VDATUM_CACHE_DIR=/path/to/cache ./download_vdatum_grids.sh

set -euo pipefail

CACHE_DIR="${VDATUM_CACHE_DIR:-${HOME}/.cache/mru_transform}"
GEOID_DIR="${CACHE_DIR}/geoid"
VDATUM_DIR="${CACHE_DIR}/vdatum"
GEOID_GRID="us_noaa_g2018u0.tif"
VDATUM_URL="https://vdatum.noaa.gov/download/data/vdatum_regional_20250917.zip"
VDATUM_ZIP="${CACHE_DIR}/vdatum_regional.zip"

echo "VDatum grid cache: ${CACHE_DIR}"

# Step 1: PROJ geoid grid
if [ -f "${GEOID_DIR}/${GEOID_GRID}" ]; then
    echo "PROJ geoid grid already cached: ${GEOID_DIR}/${GEOID_GRID}"
else
    echo "Downloading PROJ geoid grid (${GEOID_GRID})..."
    mkdir -p "${GEOID_DIR}"
    projsync --file "${GEOID_GRID}" --target-dir "${GEOID_DIR}"
    echo "Geoid grid cached."
fi

# Step 2: VDatum MLLW grids
MLLW_COUNT=$(find "${VDATUM_DIR}" -name "*_mllw.gtx" 2>/dev/null | wc -l)
if [ "${MLLW_COUNT}" -gt 0 ]; then
    echo "VDatum MLLW grids already cached: ${MLLW_COUNT} files in ${VDATUM_DIR}"
else
    # Download zip if not cached
    if [ -f "${VDATUM_ZIP}" ]; then
        echo "VDatum zip already downloaded: ${VDATUM_ZIP}"
    else
        echo "Downloading VDatum regional grids (~849MB)..."
        mkdir -p "${CACHE_DIR}"
        curl -fL -o "${VDATUM_ZIP}" "${VDATUM_URL}"
        echo "Downloaded."
    fi

    # Extract MLLW grids + metadata
    echo "Extracting MLLW grids and metadata..."
    mkdir -p "${VDATUM_DIR}"
    cd "${VDATUM_DIR}"
    unzip -o "${VDATUM_ZIP}" "*_mllw.gtx" "*.bnd" "*.met"
    MLLW_COUNT=$(find . -name "*_mllw.gtx" | wc -l)
    echo "Extracted ${MLLW_COUNT} MLLW grids."
fi

echo ""
echo "Cache summary:"
echo "  Geoid grid: ${GEOID_DIR}/${GEOID_GRID}"
echo "  VDatum dir: ${VDATUM_DIR}"
echo "  MLLW grids: ${MLLW_COUNT}"
echo ""
echo "Use these parameters for chart_datum_node:"
echo "  geoid_grid: ${GEOID_DIR}/${GEOID_GRID}"
echo "  vdatum_grid_dir: ${VDATUM_DIR}"
