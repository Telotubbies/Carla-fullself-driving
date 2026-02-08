#!/bin/bash
# Quick status check script

cd "$(dirname "$0")/.."

python3 utils/status_logger.py --scan --summary

