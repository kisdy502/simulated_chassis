#!/bin/bash
# RViz wrapper with software rendering
export LIBGL_ALWAYS_SOFTWARE=1
exec rviz2 "$@"
