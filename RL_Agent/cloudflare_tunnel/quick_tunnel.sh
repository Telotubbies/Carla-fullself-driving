#!/bin/bash
# Start Quick Tunnel (no domain needed)

echo "🚀 Starting Quick Tunnel..."
pkill -f "cloudflared tunnel" 2>/dev/null
sleep 2

cloudflared tunnel --url http://localhost:5000 2>&1 | tee quick_tunnel.log &
TUNNEL_PID=$!

sleep 5
echo ""
echo "✅ Quick Tunnel started (PID: $TUNNEL_PID)"
echo ""
echo "📋 Check log for URL:"
echo "   tail -f quick_tunnel.log | grep 'https://'"
echo ""
echo "🌐 Or check:"
tail -20 quick_tunnel.log 2>/dev/null | grep -E "https://|Your quick Tunnel" | head -3
