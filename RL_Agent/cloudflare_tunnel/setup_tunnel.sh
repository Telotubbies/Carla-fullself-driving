#!/bin/bash
# Cloudflare Tunnel Setup Script

set -e

TUNNEL_NAME="carla-dashboard"
LOCAL_PORT=5000
CONFIG_FILE="config.yaml"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🌐 Cloudflare Tunnel Setup"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Check if cloudflared is installed
if ! command -v cloudflared &> /dev/null; then
    echo "❌ cloudflared not found. Installing..."
    cd /tmp
    wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -O cloudflared
    chmod +x cloudflared
    sudo mv cloudflared /usr/local/bin/
    echo "✅ cloudflared installed"
fi

echo "📋 Steps to setup:"
echo ""
echo "1. Login to Cloudflare:"
echo "   cloudflared tunnel login"
echo ""
echo "2. Create tunnel (if not exists):"
echo "   cloudflared tunnel create ${TUNNEL_NAME}"
echo ""
echo "3. Get tunnel ID:"
echo "   cloudflared tunnel list"
echo ""
echo "4. Enter your domain (e.g., dashboard.yourdomain.com):"
read -p "   Domain: " DOMAIN
echo ""
echo "5. Creating config file..."

# Get tunnel ID
TUNNEL_ID=$(cloudflared tunnel list 2>/dev/null | grep "${TUNNEL_NAME}" | awk '{print $1}' || echo "")

if [ -z "$TUNNEL_ID" ]; then
    echo "⚠️  Tunnel not found. Please create it first:"
    echo "   cloudflared tunnel create ${TUNNEL_NAME}"
    exit 1
fi

# Create config file
cat > "${CONFIG_FILE}" << EOF
tunnel: ${TUNNEL_ID}
credentials-file: ${HOME}/.cloudflared/${TUNNEL_ID}.json

ingress:
  - hostname: ${DOMAIN}
    service: http://localhost:${LOCAL_PORT}
  - service: http_status:404
EOF

echo "✅ Config file created: ${CONFIG_FILE}"
echo ""
echo "6. Setup DNS:"
echo "   cloudflared tunnel route dns ${TUNNEL_NAME} ${DOMAIN}"
echo ""
echo "7. Run tunnel:"
echo "   cloudflared tunnel --config ${CONFIG_FILE} run"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
