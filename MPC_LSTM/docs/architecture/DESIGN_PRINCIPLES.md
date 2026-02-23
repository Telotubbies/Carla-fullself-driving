# Design Principles & Architecture

## 🎯 Core Design Principles

### 1. **Separation of Concerns**
- **Perception**: Image processing and feature extraction
- **Temporal**: State prediction and sequence modeling
- **Control**: Control policy and optimization
- **Visualization**: Display and monitoring (decoupled)

### 2. **Dependency Injection**
- All dependencies injected via constructor
- Configuration passed as structured objects
- No global state or singletons

### 3. **Interface-Based Design**
- Abstract base classes for all major components
- Easy to swap implementations (e.g., different encoders, controllers)
- Testable with mocks

### 4. **Error Handling Strategy**
- Custom exception hierarchy
- Graceful degradation
- Comprehensive logging

### 5. **Configuration Management**
- Environment-based configs (dev, prod, test)
- Schema validation
- Type-safe access

## 📐 Architecture Layers

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│              (main.py, AutonomousDrivingSystem)          │
└─────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
┌───────▼────────┐  ┌───────▼────────┐  ┌───────▼────────┐
│  Perception    │  │   Temporal      │  │    Control     │
│  (ResNet)      │  │   (LSTM)        │  │    (MPC)       │
└───────┬────────┘  └───────┬────────┘  └───────┬────────┘
        │                   │                   │
┌───────▼───────────────────────────────────────▼────────┐
│              Infrastructure Layer                      │
│  (CARLA Client, Sensors, Device Utils, Logging)       │
└───────────────────────────────────────────────────────┘
```

## 🔧 Design Patterns

### Factory Pattern
- Model creation (ResNet, LSTM)
- Controller instantiation

### Strategy Pattern
- Different control strategies
- Different feature extractors

### Observer Pattern
- Event handling for visualization
- Status updates

### Builder Pattern
- Complex object construction (MPC setup)

## 📊 Data Flow

```
Camera → ResNet → Features → Sequence Buffer → LSTM → Predicted State
                                                              │
                                                              ▼
Vehicle State ────────────────────────────────────────→ MPC Controller
                                                              │
                                                              ▼
                                                         Control Output
```

