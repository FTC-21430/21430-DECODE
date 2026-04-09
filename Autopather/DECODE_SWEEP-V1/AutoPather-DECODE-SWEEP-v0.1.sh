#!/bin/sh
printf '\033c\033]0;%s\a' AutoPather-IntoTheDeep-v2
base_path="$(dirname "$(realpath "$0")")"
"$base_path/AutoPather-DECODE-SWEEP-v0.1.x86_64" "$@"
