#!/bin/bash
set -e

# XAUTH=/tmp/.docker.xauth

# # Only do XAUTH setup if we actually have a DISPLAY
# if [ -n "$DISPLAY" ]; then
#     # Create /tmp/.docker.xauth if it does not exist
#     if [ ! -f "$XAUTH" ]; then
#         touch $XAUTH
#         xauth nlist "$DISPLAY" 2>/dev/null \
#           | sed -e 's/^..../ffff/' \
#           | xauth -f $XAUTH nmerge - 2>/dev/null || true
#     fi
# fi

# Setup environment
source $HOME/.bashrc

