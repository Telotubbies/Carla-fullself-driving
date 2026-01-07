#!/bin/bash
# Setup Domain for Cloudflare Tunnel

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🌐 Setup Domain for Cloudflare Tunnel"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

read -p "Enter your domain (e.g., dashboard.yourdomain.com): " DOMAIN

if [ -z "$DOMAIN" ]; then
    echo "❌ Domain is required"
    exit 1
fi

echo ""
echo "📋 Setting up DNS route..."
cloudflared tunnel route dns carla-dashboard "$DOMAIN"

if [ $? -eq 0 ]; then
    echo "✅ DNS route created"
    
    # Update config.yaml
    TUNNEL_ID="e33ae128-e7b1-44cf-a202-ab2d67e7ee70"
    cat > config.yaml << EOF
tunnel: ${TUNNEL_ID}
credentials-file: ${HOME}/.cloudflared/${TUNNEL_ID}.json

ingress:
  - hostname: ${DOMAIN}
    service: http://localhost:5000
  - service: http_status:404
EOF
    
    echo "✅ Config updated"
    echo ""
    echo "🔄 Restarting tunnel..."
    pkill -f "cloudflared tunnel"
    sleep 2
    cd "$(dirname "$0")"
    nohup cloudflared tunnel --config config.yaml run > tunnel.log 2>&1 &
    echo "✅ Tunnel restarted"
    echo ""
    echo "🌐 Your dashboard is now available at:"
    echo "   https://${DOMAIN}"
else
    echo "❌ Failed to setup DNS route"
    exit 1
fi
