#!/bin/bash
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 Check Cloudflare Tunnel Status"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "1. Tunnel Process:"
ps aux | grep "cloudflared tunnel" | grep -v grep || echo "   ❌ Not running"

echo ""
echo "2. Dashboard Status:"
curl -s http://localhost:5000/api/status > /dev/null && echo "   ✅ Running" || echo "   ❌ Not running"

echo ""
echo "3. Tunnel Log (last 5 lines):"
tail -5 tunnel.log 2>/dev/null | grep -E "INF|ERR|WRN" | tail -3 || echo "   No log"

echo ""
echo "4. DNS Check:"
nslookup dashboard.carlatraining.local.flowtester.online 2>/dev/null | grep -E "Name|Address" | head -3 || echo "   DNS not propagated yet"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
