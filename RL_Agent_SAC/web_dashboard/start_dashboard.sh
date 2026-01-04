#!/bin/bash
# Start Training Dashboard

cd "$(dirname "$0")/.."
source venv/bin/activate

echo "=" | head -c 70; echo
echo "🚀 Starting Training Dashboard"
echo "=" | head -c 70; echo
echo ""
echo "📊 Dashboard: http://localhost:5000"
echo "📡 API: http://localhost:5000/api/status"
echo ""
echo "⚠️  Note: Dashboard อ่านข้อมูลเท่านั้น ไม่กระทบ training performance"
echo "=" | head -c 70; echo
echo ""

python3 web_dashboard/app.py

