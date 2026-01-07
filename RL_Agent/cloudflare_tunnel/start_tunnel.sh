#!/bin/bash
# Start Cloudflare Tunnel

cd "$(dirname "$0")"

# Check if config exists
if [ -f "config.yaml" ]; then
    echo "🚀 Starting tunnel with config..."
    cloudflared tunnel --config config.yaml run
else
    echo "🚀 Starting quick tunnel..."
    cloudflared tunnel --url http://localhost:5000
fi
