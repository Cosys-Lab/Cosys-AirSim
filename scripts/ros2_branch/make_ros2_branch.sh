#!/usr/bin/env bash
# Copyright (c) Cosys-Lab, University of Antwerp. 2026
#
# Creates or updates the "ros2" branch: a stand-alone ROS2 workspace with only the ROS2 packages and the
# C++ client library they build on (AirLib, MavLinkCom, the CMake files), plus the setup.sh, README.md and
# .gitignore from scripts/ros2_branch/template.
#
# It works on git objects only: your working tree, index and current branch are not touched. Every run adds
# one commit to the ros2 branch (none if nothing changed), so the branch keeps a normal history.
#
# Usage: scripts/ros2_branch/make_ros2_branch.sh [source branch or commit, default HEAD] [target branch, default ros2]
# Then publish it with: git push origin ros2

set -e

src="${1:-HEAD}"
branch="${2:-ros2}"
cd "$(git rev-parse --show-toplevel)"

src_sha="$(git rev-parse --verify "$src^{commit}")"
src_name="$(git rev-parse --abbrev-ref "$src" 2>/dev/null || echo "$src")"
if [ "$src_name" = "HEAD" ]; then src_name="$(git rev-parse --abbrev-ref HEAD)"; fi

# folders and files taken as they are from the source branch
paths=(
    AirLib
    MavLinkCom
    cmake/AirLib
    cmake/MavLinkCom
    cmake/rpclib_wrapper
    cmake/cmake-modules
    ros2
    docs/ros2.md
    LICENSE
    THIRD_PARTY_LICENSES.md
)
template="scripts/ros2_branch/template"

for p in "${paths[@]}" "$template/README.md" "$template/setup.sh" "$template/.gitignore"; do
    if ! git cat-file -e "$src_sha:$p" 2>/dev/null; then
        echo "$p does not exist in $src ($src_sha)"
        exit 1
    fi
done

# build the tree in a temporary index
tmp_index="$(mktemp)"
rm -f "$tmp_index"
trap 'rm -f "$tmp_index"' EXIT
export GIT_INDEX_FILE="$tmp_index"
git read-tree --empty
git ls-tree -r "$src_sha" -- "${paths[@]}" | git update-index --index-info
printf "100644 blob %s\tREADME.md\n" "$(git rev-parse "$src_sha:$template/README.md")" | git update-index --index-info
printf "100755 blob %s\tsetup.sh\n" "$(git rev-parse "$src_sha:$template/setup.sh")" | git update-index --index-info
printf "100644 blob %s\t.gitignore\n" "$(git rev-parse "$src_sha:$template/.gitignore")" | git update-index --index-info
tree="$(git write-tree)"
unset GIT_INDEX_FILE

message="Update the ROS2 workspace from $src_name ($(git rev-parse --short "$src_sha"))"
if old="$(git rev-parse --verify -q "refs/heads/$branch")"; then
    if [ "$(git rev-parse "$old^{tree}")" = "$tree" ]; then
        echo "The $branch branch is already up to date with $src_name."
        exit 0
    fi
    commit="$(git commit-tree "$tree" -p "$old" -m "$message")"
    git update-ref "refs/heads/$branch" "$commit" "$old"
else
    commit="$(git commit-tree "$tree" -m "ROS2 workspace of Cosys-AirSim, from $src_name ($(git rev-parse --short "$src_sha"))")"
    git update-ref "refs/heads/$branch" "$commit"
fi

echo "Branch $branch is now $(git rev-parse --short "$commit") ($(git ls-tree -r --name-only "$commit" | wc -l) files)."
echo "Check it with: git log --stat -1 $branch --"
echo "Publish it with: git push origin $branch"
