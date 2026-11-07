#!/bin/bash
# Kept for the links already published: the reproduction script now lives in tools/toa23/.
# Writes toa.log into tools/toa23/work/ (or into the directory given as argument).
exec "$(dirname "$0")/tools/toa23/reproduce.sh" "$@"
