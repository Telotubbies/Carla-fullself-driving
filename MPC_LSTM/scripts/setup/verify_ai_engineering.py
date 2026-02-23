#!/usr/bin/env python3
"""
Verify AI Engineering improvements are properly implemented.
"""

import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

def verify_core_modules():
    """Verify core modules can be imported."""
    print("🔍 Verifying Core Modules...")
    
    try:
        from core.interfaces import (
            IPerceptionModule, ITemporalModule, 
            IControlModule, IVisualizationModule
        )
        print("  ✅ Interfaces imported")
    except Exception as e:
        print(f"  ❌ Interfaces: {e}")
        return False
    
    try:
        from core.exceptions import (
            ProjectError, CARLAConnectionError, 
            ModelLoadError, DataValidationError
        )
        print("  ✅ Exceptions imported")
    except Exception as e:
        print(f"  ❌ Exceptions: {e}")
        return False
    
    try:
        from core.config import ConfigManager, ConfigSchema
        print("  ✅ Config imported")
    except Exception as e:
        print(f"  ❌ Config: {e}")
        return False
    
    try:
        from core.factories import (
            PerceptionFactory, TemporalFactory,
            ControlFactory, VisualizationFactory
        )
        print("  ✅ Factories imported")
    except Exception as e:
        print(f"  ❌ Factories: {e}")
        return False
    
    try:
        from core.validators import (
            ImageValidator, StateValidator,
            FeatureValidator, PredictionValidator, ControlValidator
        )
        print("  ✅ Validators imported")
    except Exception as e:
        print(f"  ❌ Validators: {e}")
        return False
    
    try:
        from core.system import AutonomousDrivingSystem
        print("  ✅ System imported")
    except Exception as e:
        print(f"  ❌ System: {e}")
        return False
    
    return True

def verify_module_implementations():
    """Verify modules implement interfaces."""
    print("\n🔍 Verifying Module Implementations...")
    
    try:
        from perception import ResNetEncoder
        from core.interfaces import IPerceptionModule
        assert issubclass(ResNetEncoder, IPerceptionModule), "ResNetEncoder should implement IPerceptionModule"
        print("  ✅ ResNetEncoder implements IPerceptionModule")
    except Exception as e:
        print(f"  ❌ ResNetEncoder: {e}")
        return False
    
    try:
        from temporal import LSTMPredictor
        from core.interfaces import ITemporalModule
        assert issubclass(LSTMPredictor, ITemporalModule), "LSTMPredictor should implement ITemporalModule"
        print("  ✅ LSTMPredictor implements ITemporalModule")
    except Exception as e:
        print(f"  ❌ LSTMPredictor: {e}")
        return False
    
    try:
        from control import MPCController
        from core.interfaces import IControlModule
        assert issubclass(MPCController, IControlModule), "MPCController should implement IControlModule"
        print("  ✅ MPCController implements IControlModule")
    except Exception as e:
        print(f"  ❌ MPCController: {e}")
        return False
    
    try:
        from visualization import VisualizationDisplay
        from core.interfaces import IVisualizationModule
        assert issubclass(VisualizationDisplay, IVisualizationModule), "VisualizationDisplay should implement IVisualizationModule"
        print("  ✅ VisualizationDisplay implements IVisualizationModule")
    except Exception as e:
        print(f"  ❌ VisualizationDisplay: {e}")
        return False
    
    return True

def verify_tests():
    """Verify test files exist."""
    print("\n🔍 Verifying Tests...")
    
    test_files = [
        'tests/unit/test_perception.py',
        'tests/unit/test_temporal.py',
        'tests/unit/test_control.py',
        'tests/unit/test_validators.py',
        'tests/integration/test_system.py',
        'tests/run_tests.sh'
    ]
    
    all_exist = True
    for test_file in test_files:
        if Path(test_file).exists():
            print(f"  ✅ {test_file}")
        else:
            print(f"  ❌ {test_file} not found")
            all_exist = False
    
    return all_exist

def main():
    """Main verification function."""
    print("=" * 60)
    print("AI Engineering Verification")
    print("=" * 60)
    print()
    
    results = []
    
    results.append(("Core Modules", verify_core_modules()))
    results.append(("Module Implementations", verify_module_implementations()))
    results.append(("Tests", verify_tests()))
    
    print("\n" + "=" * 60)
    print("Verification Summary")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{name}: {status}")
        if not passed:
            all_passed = False
    
    print()
    if all_passed:
        print("🎉 All verifications passed!")
        return 0
    else:
        print("⚠️  Some verifications failed")
        return 1

if __name__ == '__main__':
    sys.exit(main())

