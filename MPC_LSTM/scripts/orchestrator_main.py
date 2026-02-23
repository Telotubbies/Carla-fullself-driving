#!/usr/bin/env python3
"""
Central Orchestrator Script for CARLA LSTM-MPC Pipeline.

Manages the complete workflow in phases with state tracking and error recovery.

Usage:
    python3 scripts/orchestrator_main.py [--phases PHASE1,PHASE2,...] [--resume] [--reset]
"""

import sys
import argparse
import logging
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core.orchestrator import PipelineOrchestrator
from core.phases import create_phase_functions
from core.config import ConfigManager

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """Main orchestrator entry point."""
    parser = argparse.ArgumentParser(
        description='CARLA LSTM-MPC Pipeline Orchestrator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all phases
  python3 scripts/orchestrator_main.py
  
  # Run specific phases
  python3 scripts/orchestrator_main.py --phases collect_data,finetune_resnet
  
  # Resume from last checkpoint
  python3 scripts/orchestrator_main.py --resume
  
  # Reset and start fresh
  python3 scripts/orchestrator_main.py --reset
  
  # Show status
  python3 scripts/orchestrator_main.py --status
        """
    )
    
    parser.add_argument(
        '--phases',
        type=str,
        default=None,
        help='Comma-separated list of phases to run (default: all)'
    )
    parser.add_argument(
        '--resume',
        action='store_true',
        help='Resume from last completed phase'
    )
    parser.add_argument(
        '--reset',
        action='store_true',
        help='Reset all phases and start fresh'
    )
    parser.add_argument(
        '--status',
        action='store_true',
        help='Show current pipeline status'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='config.yaml',
        help='Path to configuration file'
    )
    parser.add_argument(
        '--stop-on-error',
        action='store_true',
        default=True,
        help='Stop on first error (default: True)'
    )
    
    args = parser.parse_args()
    
    # Initialize orchestrator
    orchestrator = PipelineOrchestrator(
        config_path=args.config,
        state_file=str(PROJECT_ROOT / "logs" / "pipeline_state.json")
    )
    
    # Initialize config manager
    config_manager = ConfigManager(args.config)
    
    # Create phase functions
    phase_functions = create_phase_functions(config_manager, PROJECT_ROOT)
    
    # Register phases
    orchestrator.register_phase(
        'collect_data',
        'Collect training data from CARLA autopilot',
        phase_functions['collect_data']
    )
    
    orchestrator.register_phase(
        'create_lane_labels',
        'Create lane masks from CARLA map',
        phase_functions['create_lane_labels'],
        dependencies=['collect_data']
    )
    
    orchestrator.register_phase(
        'finetune_resnet',
        'Fine-tune ResNet for lane detection',
        phase_functions['finetune_resnet'],
        dependencies=['create_lane_labels']
    )
    
    orchestrator.register_phase(
        'extract_features',
        'Extract features using ResNet',
        phase_functions['extract_features'],
        dependencies=['finetune_resnet']
    )
    
    orchestrator.register_phase(
        'train_lstm',
        'Train LSTM model for temporal prediction',
        phase_functions['train_lstm'],
        dependencies=['extract_features']
    )
    
    orchestrator.register_phase(
        'update_config',
        'Update config.yaml with trained models',
        phase_functions['update_config'],
        dependencies=['train_lstm', 'finetune_resnet']
    )
    
    orchestrator.register_phase(
        'run_inference',
        'Run inference with trained models',
        phase_functions['run_inference'],
        dependencies=['update_config']
    )
    
    # Handle commands
    if args.reset:
        orchestrator.reset_all()
        logger.info("🔄 All phases reset")
        return 0
    
    if args.status:
        status = orchestrator.get_status()
        print("\n📊 Pipeline Status:")
        print("=" * 60)
        for phase_name, phase_info in status['phases'].items():
            status_icon = {
                'pending': '⏳',
                'running': '▶️',
                'completed': '✅',
                'failed': '❌',
                'skipped': '⏭️'
            }.get(phase_info['status'], '❓')
            
            print(f"{status_icon} {phase_name}: {phase_info['status']}")
            if phase_info.get('duration'):
                print(f"   Duration: {phase_info['duration']:.1f}s")
            if phase_info.get('error'):
                print(f"   Error: {phase_info['error']}")
        
        if status.get('current_phase'):
            print(f"\n▶️  Current Phase: {status['current_phase']}")
        
        return 0
    
    # Parse phases
    phases = None
    if args.phases:
        phases = [p.strip() for p in args.phases.split(',')]
    
    # Execute pipeline
    success = orchestrator.execute(
        phases=phases,
        resume=args.resume,
        stop_on_error=args.stop_on_error
    )
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())

