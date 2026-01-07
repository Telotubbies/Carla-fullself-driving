#!/bin/bash
# Run Cloudflare Tunnel

cd "$(dirname "$0")"
CONFIG_FILE="config.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Config file not found. Run setup_tunnel.sh first"
    exit 1
fi

echo "🚀 Starting Cloudflare Tunnel..."
cloudflared tunnel --config "$CONFIG_FILE" run
