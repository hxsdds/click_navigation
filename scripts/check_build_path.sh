#!/usr/bin/env bash
# ROS Jazzy's IDL file(STRINGS) parsing breaks paths containing non-ASCII text.
check_build_path() {
    local LC_ALL=C path="$1"
    if [[ "$path" == *[!\ -~]* || "$path" == *' '* ]]; then
        echo 'Unsupported build path: use an ASCII path without spaces, e.g. ~/work/click_navigation. Clone again there; do not reuse build/install/log.' >&2
        return 2
    fi
}
check_build_path "$root"
