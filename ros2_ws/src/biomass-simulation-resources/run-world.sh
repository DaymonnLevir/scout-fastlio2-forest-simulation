#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: run-world <world-name-or-path> [extra gz args]"
  echo "Examples:"
  echo "  run-world world_jean"
  echo "  run-world world_template_empty.world"
  echo "  run-world /simulation-ws/src/biomass-simulation-resources/worlds/world_jean.world"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Resolve package root for both source-tree and installed execution.
if [ -d "${SCRIPT_DIR}/worlds" ] && [ -d "${SCRIPT_DIR}/models" ]; then
  PKG_ROOT="${SCRIPT_DIR}"
elif [ -d "${SCRIPT_DIR}/../../share/biomass_simulation_resources/worlds" ]; then
  PKG_ROOT="${SCRIPT_DIR}/../../share/biomass_simulation_resources"
else
  echo "Could not resolve package root from ${SCRIPT_DIR}" >&2
  exit 3
fi

WORLD_ARG="$1"
shift || true

# Accept absolute/relative path, name, or name.world
if [[ "$WORLD_ARG" == */* ]] || [[ "$WORLD_ARG" == *.world ]]; then
  if [ -f "$WORLD_ARG" ]; then
    WORLD_FILE="$(cd "$(dirname "$WORLD_ARG")" && pwd)/$(basename "$WORLD_ARG")"
  else
    CANDIDATE="${PKG_ROOT}/worlds/${WORLD_ARG}"
    if [ -f "$CANDIDATE" ]; then
      WORLD_FILE="$CANDIDATE"
    else
      echo "World not found: $WORLD_ARG"
      exit 2
    fi
  fi
else
  CANDIDATE="${PKG_ROOT}/worlds/${WORLD_ARG}.world"
  if [ ! -f "$CANDIDATE" ]; then
    echo "World not found: ${CANDIDATE}"
    exit 2
  fi
  WORLD_FILE="$CANDIDATE"
fi

MODELS_1="${PKG_ROOT}/models"
MODELS_2="${PKG_ROOT}/models/forest-gen-models"
WORLDS_DIR="${PKG_ROOT}/worlds"

export GZ_SIM_RESOURCE_PATH="${MODELS_1}:${MODELS_2}:${WORLDS_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="${MODELS_1}:${MODELS_2}:${SDF_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${MODELS_1}:${MODELS_2}:${WORLDS_DIR}:${IGN_GAZEBO_RESOURCE_PATH:-}"

exec ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="${WORLD_FILE} $*"
