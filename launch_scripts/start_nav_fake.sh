#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

HAS_FAKE=false
for arg in "$@"; do
    if [ "$arg" = "--fake-base" ]; then
        HAS_FAKE=true
        break
    fi
done

if [ "$HAS_FAKE" = true ]; then
    exec "$SCRIPT_DIR/robot.sh" navigation "$@"
else
    exec "$SCRIPT_DIR/robot.sh" navigation "$@" --fake-base
fi
