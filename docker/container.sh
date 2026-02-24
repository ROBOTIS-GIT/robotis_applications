#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="robotis-applications"

detect_arch() {
    case "$(uname -m)" in
        x86_64) echo "amd64" ;;
        aarch64|arm64) echo "arm64" ;;
        *)
            echo "Error: Unsupported host architecture: $(uname -m)" >&2
            exit 1
            ;;
    esac
}

TARGET_ARCH="${TARGET_ARCH:-$(detect_arch)}"

compose_cmd() {
    TARGET_ARCH="${TARGET_ARCH}" docker compose -f "${COMPOSE_FILE}" "$@"
}

setup_x11() {
    if [ -n "${DISPLAY:-}" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi
}

show_help() {
    cat <<EOF
Usage: $0 [command]

Commands:
  help                     Show this help message
  build                    Build image only
  start                    Build (if needed) and start container
  enter                    Enter the running container
  stop                     Stop and remove container
  status                   Show compose service status
  logs                     Follow container logs

Environment:
  TARGET_ARCH              amd64 or arm64 (default: auto-detected)

Examples:
  $0 build
  $0 start
  TARGET_ARCH=amd64 $0 start
  $0 enter
  $0 stop
EOF
}

build_container() {
    echo "Building robotis-applications image (TARGET_ARCH=${TARGET_ARCH})..."
    compose_cmd build
}

start_container() {
    setup_x11
    echo "Starting robotis-applications container (TARGET_ARCH=${TARGET_ARCH})..."
    compose_cmd up -d --build
}

enter_container() {
    setup_x11
    if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        echo "Error: Container is not running."
        echo "Hint: run '$0 start' first."
        exit 1
    fi
    docker exec -it "${CONTAINER_NAME}" bash
}

stop_container() {
    if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        echo "Error: Container is not running."
        exit 1
    fi

    echo "Warning: This will stop and remove the container."
    read -r -p "Are you sure you want to continue? [y/N] " REPLY
    if [[ "${REPLY}" =~ ^[Yy]$ ]]; then
        compose_cmd down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

status_container() {
    compose_cmd ps
}

logs_container() {
    compose_cmd logs -f --tail=200
}

case "${1:-help}" in
    "help") show_help ;;
    "build") build_container ;;
    "start") start_container ;;
    "enter") enter_container ;;
    "stop") stop_container ;;
    "status") status_container ;;
    "logs") logs_container ;;
    *)
        echo "Error: Unknown command '$1'"
        show_help
        exit 1
        ;;
esac
