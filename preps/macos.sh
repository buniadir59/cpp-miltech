#!/usr/bin/env bash
set -euo pipefail

brew install colima docker docker-buildx

mkdir -p ~/.docker/cli-plugins
ln -sfn "$(brew --prefix)/opt/docker-buildx/bin/docker-buildx" ~/.docker/cli-plugins/docker-buildx

colima start
