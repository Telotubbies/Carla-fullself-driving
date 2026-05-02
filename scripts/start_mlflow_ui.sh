#!/bin/bash
# Start MLflow UI

echo "🌐 Starting MLflow UI..."
echo "URL: http://localhost:5000"
echo ""

cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate

mlflow ui --backend-store-uri ./mlruns --port 5000
