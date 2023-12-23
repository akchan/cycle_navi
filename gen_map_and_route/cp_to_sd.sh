#!/bin/zsh

set -u


# Parse command line options with zsh built-in command
local -A opthash
zparseopts -D -A opthash -- -delete

if [[ -n "${opthash[(i)--delete]}" ]]; then
    opt_map_delete=1
else
    opt_map_delete=0
fi


# Check arguments
if [[ $# != 2 ]]; then
    echo "Usage: $0 <map_route path> <SD card path>"
    echo ""
    echo "  --delete: delete map files which is not in the map_route path"
    exit 1
fi

route_map_path="${1}"
sd_path="${2}"

# Copy map files
if [[ ${opt_map_delete} == 1 ]]; then
    rsync -auv --delete "${route_map_path}/map" "${sd_path}"
else
    cp -nrv "${route_map_path}/map" "${sd_path}"
fi

# Copy route and point files
rm -Rf "${sd_path}/route_dat" "${sd_path}/point_dat"
cp -rv "${route_map_path}/route_dat" "${route_map_path}/point_dat" "${sd_path}"

exit 0
