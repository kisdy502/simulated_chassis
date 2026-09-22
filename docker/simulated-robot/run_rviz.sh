#!/bin/bash
# RViz wrapper with software rendering
# QT_QPA_PLATFORM=xcb: WSLg 下强制 X11 后端，绕开 Wayland 合成器的 Qt 事件循环卡死
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
exec rviz2 "$@"
