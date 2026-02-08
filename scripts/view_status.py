#!/usr/bin/env python3
"""
View system status from centralized status file.
Outputs to log file.
"""

import sys
from pathlib import Path
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.status_logger import StatusLogger, scan_system_status

def main():
    status_logger = StatusLogger()
    
    # Scan and update
    scan_system_status(status_logger)
    
    # Get summary
    summary = status_logger.get_summary()
    
    # Output to log file
    log_dir = Path(__file__).parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"status_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    
    with open(log_file, 'w', encoding='utf-8') as f:
        f.write(summary)
        f.write("\n\n")
        f.write("=" * 60 + "\n")
        f.write("DETAILED STATUS\n")
        f.write("=" * 60 + "\n\n")
        
        import json
        f.write(json.dumps(status_logger.get_status(), indent=2, ensure_ascii=False))
    
    # Also print to console
    print(summary)
    print(f"\n📝 Status saved to: {log_file}")
    
    # Show detailed JSON if requested
    if len(sys.argv) > 1 and sys.argv[1] == '--json':
        import json
        print("\n" + "=" * 60)
        print("DETAILED JSON STATUS")
        print("=" * 60)
        print(json.dumps(status_logger.get_status(), indent=2, ensure_ascii=False))

if __name__ == '__main__':
    main()

