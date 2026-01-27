# 🗺️ CARLA 0.9.16 - Complete Project Architecture Diagram

## 📊 Overall Project Architecture (Block Diagram)

```mermaid
flowchart TB
    subgraph BLOCK1["┌─────────────────────────────────────────────────┐<br/>│  CARLA SIMULATOR CORE (Unreal Engine 4)  │<br/>└─────────────────────────────────────────────────┘"]
        UE4["Unreal Engine 4<br/>CarlaUE4"]
        SERVER["CARLA Server<br/>Port 2000"]
        WORLD["World Manager<br/>Maps & Actors"]
        PHYSICS["Physics Engine<br/>Vehicle Dynamics"]
    end
    
    subgraph BLOCK2["┌─────────────────────────────────────────────────┐<br/>│  PYTHON API LAYER                        │<br/>└─────────────────────────────────────────────────┘"]
        CLIENT["Python Client<br/>carla.Client"]
        ACTORS["Actor Management<br/>Vehicles, Sensors"]
        SENSORS["Sensor Suite<br/>RGB, Depth, GPS, LiDAR"]
        MAPS["Map & Navigation<br/>Waypoints, Routes"]
    end
    
    subgraph BLOCK3["┌─────────────────────────────────────────────────┐<br/>│  RL AGENT SAC SYSTEM                     │<br/>└─────────────────────────────────────────────────┘"]
        ENV["CarlaRLEnv<br/>Gym Environment"]
        POLICY["SAC Policy<br/>Actor-Critic Network"]
        VISION["Vision Encoder<br/>ResNet18 Pretrained"]
        TRAIN["Training Loop<br/>train_sac.py"]
        BUFFER["Replay Buffer<br/>100K Transitions"]
        CHECKPOINT["Checkpoint Manager<br/>SQLite Database<br/>training_checkpoints.db"]
    end
    
    subgraph BLOCK4["┌─────────────────────────────────────────────────┐<br/>│  AUTO-MANAGEMENT SYSTEM                  │<br/>└─────────────────────────────────────────────────┘"]
        AUTO["Auto Manager<br/>auto_manage.py"]
        MONITOR["Process Monitor<br/>Health Checks"]
        RESTART["Auto Restart<br/>Failure Recovery"]
        DASHBOARD["Web Dashboard<br/>Port 5001"]
    end
    
    subgraph BLOCK5["┌─────────────────────────────────────────────────┐<br/>│  CO-SIMULATION TOOLS                     │<br/>└─────────────────────────────────────────────────┘"]
        SUMO["SUMO Integration<br/>Traffic Simulation"]
        VISSIM["PTV-Vissim<br/>Traffic Analysis"]
        CHRONO["Chrono<br/>Vehicle Physics"]
        CARSIM["CarSim<br/>Vehicle Dynamics"]
    end
    
    subgraph BLOCK6["┌─────────────────────────────────────────────────┐<br/>│  SUPPORTING TOOLS                        │<br/>└─────────────────────────────────────────────────┘"]
        UTILS["Utilities<br/>Logging, GPU, Augmentation"]
        CONFIG["Configuration<br/>YAML Files"]
        LOGS["Logging System<br/>Training Logs"]
        WEB["Web Dashboard<br/>React + FastAPI"]
    end
    
    %% Styling
    classDef blockStyle fill:#2d5aa0,stroke:#1a3d7a,stroke-width:3px,color:#fff
    classDef coreStyle fill:#4a90e2,stroke:#2d5aa0,stroke-width:2px,color:#fff
    classDef apiStyle fill:#50c878,stroke:#2d7a4a,stroke-width:2px,color:#fff
    classDef rlStyle fill:#e24a4a,stroke:#7a1a1a,stroke-width:2px,color:#fff
    classDef autoStyle fill:#e2a04a,stroke:#7a4a1a,stroke-width:2px,color:#fff
    classDef coStyle fill:#9b4ae2,stroke:#4a1a7a,stroke-width:2px,color:#fff
    classDef utilStyle fill:#4ae2a0,stroke:#1a7a4a,stroke-width:2px,color:#fff
    
    class BLOCK1,UE4,SERVER,WORLD,PHYSICS blockStyle,coreStyle
    class BLOCK2,CLIENT,ACTORS,SENSORS,MAPS blockStyle,apiStyle
    class BLOCK3,ENV,POLICY,VISION,TRAIN,BUFFER,CHECKPOINT blockStyle,rlStyle
    class BLOCK4,AUTO,MONITOR,RESTART,DASHBOARD blockStyle,autoStyle
    class BLOCK5,SUMO,VISSIM,CHRONO,CARSIM blockStyle,coStyle
    class BLOCK6,UTILS,CONFIG,LOGS,WEB blockStyle,utilStyle
    
    %% Connections
    UE4 --> SERVER
    SERVER --> WORLD
    WORLD --> PHYSICS
    
    CLIENT --> SERVER
    CLIENT --> ACTORS
    CLIENT --> SENSORS
    CLIENT --> MAPS
    
    ENV --> CLIENT
    ENV --> SENSORS
    ENV --> MAPS
    POLICY --> VISION
    TRAIN --> ENV
    TRAIN --> POLICY
    TRAIN --> BUFFER
    TRAIN --> CHECKPOINT
    
    AUTO --> SERVER
    AUTO --> TRAIN
    AUTO --> DASHBOARD
    MONITOR --> AUTO
    RESTART --> AUTO
    
    SUMO --> SERVER
    VISSIM --> SERVER
    CHRONO --> PHYSICS
    CARSIM --> PHYSICS
    
    TRAIN --> UTILS
    TRAIN --> CONFIG
    TRAIN --> LOGS
    DASHBOARD --> WEB
    CHECKPOINT --> DASHBOARD
```

## 💾 Database Architecture

```mermaid
flowchart TB
    subgraph DB_SYSTEM["╔═══════════════════════════════════════════════╗<br/>║  DATABASE SYSTEM: SQLite ONLY            ║<br/>╚═══════════════════════════════════════════════╝"]
        direction TB
        
        DB_FILE["┌─────────────────────────────────────┐<br/>│ training_checkpoints.db                │<br/>│ SQLite Database File                    │<br/>│ Location: checkpoints/                  │<br/>└─────────────────────────────────────┘"]
        
        subgraph TABLES["Database Tables"]
            T1["┌─────────────────────┐<br/>│ checkpoints          │<br/>│ - id (PK)            │<br/>│ - timestep           │<br/>│ - episode            │<br/>│ - reward             │<br/>│ - model_data (BLOB)  │<br/>│ - optimizer_data     │<br/>│ - training_state     │<br/>│ - metadata (JSON)     │<br/>└─────────────────────┘"]
            
            T2["┌─────────────────────┐<br/>│ training_stats       │<br/>│ - id (PK)            │<br/>│ - timestep           │<br/>│ - mean_reward        │<br/>│ - learning_rate      │<br/>│ - value_loss         │<br/>│ - policy_loss        │<br/>│ - entropy_loss       │<br/>│ - fps                │<br/>└─────────────────────┘"]
            
            T3["┌─────────────────────┐<br/>│ episodes             │<br/>│ - id (PK)            │<br/>│ - episode_num        │<br/>│ - reward             │<br/>│ - length             │<br/>│ - collision          │<br/>│ - goal_reached       │<br/>│ - distance_to_goal   │<br/>└─────────────────────┘"]
        end
        
        subgraph FEATURES["SQLite Features Used"]
            F1["✅ WAL Mode<br/>(Write-Ahead Logging)"]
            F2["✅ Thread-Safe<br/>(Multiple Connections)"]
            F3["✅ Compression<br/>(Gzip for BLOBs)"]
            F4["✅ Indexing<br/>(Fast Queries)"]
        end
        
        DB_FILE --> TABLES
        DB_FILE --> FEATURES
    end
    
    subgraph USERS["Database Users"]
        U1["Training Loop<br/>train_sac.py"]
        U2["Auto Manager<br/>auto_manage.py"]
        U3["Web Dashboard<br/>app_fastapi.py"]
        U4["Checkpoint Manager<br/>sqlite_checkpoint.py"]
    end
    
    U1 -->|"Save/Load Checkpoints"| DB_SYSTEM
    U2 -->|"Query Checkpoints"| DB_SYSTEM
    U3 -->|"Read Stats"| DB_SYSTEM
    U4 -->|"Manage Database"| DB_SYSTEM
    
    %% Styling
    classDef dbStyle fill:#2d5aa0,stroke:#1a3d7a,stroke-width:4px,color:#fff
    classDef tableStyle fill:#50c878,stroke:#2d7a4a,stroke-width:2px,color:#fff
    classDef featureStyle fill:#e2a04a,stroke:#7a4a1a,stroke-width:2px,color:#fff
    classDef userStyle fill:#9b4ae2,stroke:#4a1a7a,stroke-width:2px,color:#fff
    
    class DB_SYSTEM,DB_FILE dbStyle
    class TABLES,T1,T2,T3 tableStyle
    class FEATURES,F1,F2,F3,F4 featureStyle
    class USERS,U1,U2,U3,U4 userStyle
```

**Note:** โปรเจกต์นี้ใช้ **SQLite เท่านั้น** ไม่มี PostgreSQL, MySQL, MongoDB หรือ database อื่นๆ

## 📦 Simplified Block Diagram

```mermaid
flowchart TB
    subgraph BLOCK_CARLA["╔═══════════════════════════════════════════════╗<br/>║  CARLA SIMULATOR CORE                    ║<br/>╚═══════════════════════════════════════════════╝"]
        direction TB
        B1["┌─────────────────────┐<br/>│ Unreal Engine 4      │<br/>│ CarlaUE4              │<br/>└─────────────────────┘"]
        B2["┌─────────────────────┐<br/>│ CARLA Server         │<br/>│ Port 2000            │<br/>└─────────────────────┘"]
        B3["┌─────────────────────┐<br/>│ World Manager        │<br/>│ Maps & Actors        │<br/>└─────────────────────┘"]
        B1 --> B2 --> B3
    end
    
    subgraph BLOCK_API["╔═══════════════════════════════════════════════╗<br/>║  PYTHON API LAYER                       ║<br/>╚═══════════════════════════════════════════════╝"]
        direction TB
        B4["┌─────────────────────┐<br/>│ Python Client        │<br/>│ carla.Client          │<br/>└─────────────────────┘"]
        B5["┌─────────────────────┐<br/>│ Sensors              │<br/>│ RGB, Depth, GPS      │<br/>└─────────────────────┘"]
        B6["┌─────────────────────┐<br/>│ Actor Management     │<br/>│ Vehicles, Sensors    │<br/>└─────────────────────┘"]
        B4 --> B5
        B4 --> B6
    end
    
    subgraph BLOCK_RL["╔═══════════════════════════════════════════════╗<br/>║  RL AGENT SAC SYSTEM                    ║<br/>╚═══════════════════════════════════════════════╝"]
        direction TB
        B7["┌─────────────────────┐<br/>│ CarlaRLEnv            │<br/>│ Gym Environment       │<br/>└─────────────────────┘"]
        B8["┌─────────────────────┐<br/>│ Vision Encoder        │<br/>│ ResNet18 Pretrained   │<br/>└─────────────────────┘"]
        B9["┌─────────────────────┐<br/>│ SAC Policy            │<br/>│ Actor-Critic Network  │<br/>└─────────────────────┘"]
        B10["┌─────────────────────┐<br/>│ Replay Buffer         │<br/>│ 100K Transitions      │<br/>└─────────────────────┘"]
        B11["┌─────────────────────┐<br/>│ Training Loop         │<br/>│ train_sac.py          │<br/>└─────────────────────┘"]
        B7 --> B8 --> B9
        B9 --> B10
        B11 --> B9
    end
    
    subgraph BLOCK_AUTO["╔═══════════════════════════════════════════════╗<br/>║  AUTO-MANAGEMENT SYSTEM                 ║<br/>╚═══════════════════════════════════════════════╝"]
        direction TB
        B12["┌─────────────────────┐<br/>│ Auto Manager          │<br/>│ auto_manage.py        │<br/>└─────────────────────┘"]
        B13["┌─────────────────────┐<br/>│ Process Monitor      │<br/>│ Health Checks         │<br/>└─────────────────────┘"]
        B14["┌─────────────────────┐<br/>│ Web Dashboard        │<br/>│ Port 5001            │<br/>└─────────────────────┘"]
        B12 --> B13
        B12 --> B14
    end
    
    subgraph BLOCK_UTILS["╔═══════════════════════════════════════════════╗<br/>║  SUPPORTING TOOLS                       ║<br/>╚═══════════════════════════════════════════════╝"]
        direction LR
        B15["┌─────────────────────┐<br/>│ Checkpoint Manager   │<br/>│ SQLite DB             │<br/>└─────────────────────┘"]
        B16["┌─────────────────────┐<br/>│ Utilities           │<br/>│ Logging, GPU          │<br/>└─────────────────────┘"]
        B17["┌─────────────────────┐<br/>│ Configuration        │<br/>│ YAML Files            │<br/>└─────────────────────┘"]
        B15 -.-> B16 -.-> B17
    end
    
    %% Main Flow
    BLOCK_CARLA -->|"TCP Connection"| BLOCK_API
    BLOCK_API -->|"Environment Setup"| BLOCK_RL
    BLOCK_RL -->|"Training Control"| BLOCK_AUTO
    BLOCK_RL -->|"Save/Load"| BLOCK_UTILS
    BLOCK_AUTO -->|"Monitor"| BLOCK_RL
    BLOCK_AUTO -->|"Display Metrics"| BLOCK_UTILS
    
    %% Styling
    classDef carlaBlock fill:#2d5aa0,stroke:#1a3d7a,stroke-width:4px,color:#fff
    classDef apiBlock fill:#50c878,stroke:#2d7a4a,stroke-width:4px,color:#fff
    classDef rlBlock fill:#e24a4a,stroke:#7a1a1a,stroke-width:4px,color:#fff
    classDef autoBlock fill:#e2a04a,stroke:#7a4a1a,stroke-width:4px,color:#fff
    classDef utilBlock fill:#4ae2a0,stroke:#1a7a4a,stroke-width:4px,color:#fff
    classDef innerBlock fill:#f0f0f0,stroke:#333,stroke-width:2px
    
    class BLOCK_CARLA,B1,B2,B3 carlaBlock
    class BLOCK_API,B4,B5,B6 apiBlock
    class BLOCK_RL,B7,B8,B9,B10,B11 rlBlock
    class BLOCK_AUTO,B12,B13,B14 autoBlock
    class BLOCK_UTILS,B15,B16,B17 utilBlock
```

## 🎯 Main System Blocks (Top-Down View)

```mermaid
flowchart TD
    START([START]) --> BLOCK1["╔═══════════════════════════════════════════════╗<br/>║  BLOCK 1: CARLA SIMULATOR                   ║<br/>║  - Unreal Engine 4                          ║<br/>║  - Server (Port 2000)                        ║<br/>║  - World & Physics Engine                    ║<br/>╚═══════════════════════════════════════════════╝"]
    
    BLOCK1 --> BLOCK2["╔═══════════════════════════════════════════════╗<br/>║  BLOCK 2: PYTHON API                       ║<br/>║  - Client Connection                        ║<br/>║  - Sensor Management                        ║<br/>║  - Actor Control                            ║<br/>╚═══════════════════════════════════════════════╝"]
    
    BLOCK2 --> BLOCK3["╔═══════════════════════════════════════════════╗<br/>║  BLOCK 3: RL ENVIRONMENT                   ║<br/>║  - CarlaRLEnv (Gym)                         ║<br/>║  - Observation Collection                   ║<br/>║  - Reward Calculation                       ║<br/>╚═══════════════════════════════════════════════╝"]
    
    BLOCK3 --> BLOCK4["╔═══════════════════════════════════════════════╗<br/>║  BLOCK 4: SAC AGENT                        ║<br/>║  - Vision Encoder (ResNet18)                ║<br/>║  - Policy Network (Actor-Critic)             ║<br/>║  - Replay Buffer (100K)                     ║<br/>╚═══════════════════════════════════════════════╝"]
    
    BLOCK4 --> BLOCK5["╔═══════════════════════════════════════════════╗<br/>║  BLOCK 5: TRAINING SYSTEM                  ║<br/>║  - Training Loop (train_sac.py)             ║<br/>║  - Checkpoint Manager (SQLite)              ║<br/>║  - Auto Manager (auto_manage.py)             ║<br/>╚═══════════════════════════════════════════════╝"]
    
    BLOCK5 --> BLOCK6["╔═══════════════════════════════════════════════╗<br/>║  BLOCK 6: MONITORING & UTILITIES          ║<br/>║  - Web Dashboard (Port 5001)                ║<br/>║  - Logging System                          ║<br/>║  - Configuration (YAML)                    ║<br/>╚═══════════════════════════════════════════════╝"]
    
    BLOCK6 --> END([END])
    
    %% Styling
    classDef block1 fill:#2d5aa0,stroke:#1a3d7a,stroke-width:4px,color:#fff
    classDef block2 fill:#50c878,stroke:#2d7a4a,stroke-width:4px,color:#fff
    classDef block3 fill:#e24a4a,stroke:#7a1a1a,stroke-width:4px,color:#fff
    classDef block4 fill:#e2a04a,stroke:#7a4a1a,stroke-width:4px,color:#fff
    classDef block5 fill:#9b4ae2,stroke:#4a1a7a,stroke-width:4px,color:#fff
    classDef block6 fill:#4ae2a0,stroke:#1a7a4a,stroke-width:4px,color:#fff
    
    class BLOCK1 block1
    class BLOCK2 block2
    class BLOCK3 block3
    class BLOCK4 block4
    class BLOCK5 block5
    class BLOCK6 block6
```

## 🔄 Training Flow Sequence

```mermaid
sequenceDiagram
    participant User
    participant AutoMgr as Auto Manager
    participant CARLA as CARLA Server
    participant Env as CarlaRLEnv
    participant Agent as SAC Agent
    participant Buffer as Replay Buffer
    participant Trainer as Training Loop
    participant Checkpoint as Checkpoint Manager
    participant Dashboard as Web Dashboard
    
    User->>AutoMgr: Start Training
    AutoMgr->>CARLA: Launch CARLA Server
    CARLA-->>AutoMgr: Server Ready (Port 2000)
    AutoMgr->>Trainer: Start Training Process
    Trainer->>Env: Initialize Environment
    Env->>CARLA: Connect Client
    CARLA-->>Env: Connection Established
    Env->>CARLA: Spawn Vehicle & Sensors
    CARLA-->>Env: Vehicle & Sensors Ready
    
    loop Training Episode
        Env->>CARLA: Get Observation
        CARLA-->>Env: RGB, Depth, GPS, Velocity
        Env->>Agent: Get Action
        Agent->>Agent: Policy Forward Pass
        Agent-->>Env: Action (Steer, Throttle, Brake)
        Env->>CARLA: Apply Action
        CARLA-->>Env: New State & Reward
        Env->>Buffer: Store Transition
        Env->>Trainer: Episode Metrics
    end
    
    loop Training Update (Every N Steps)
        Trainer->>Buffer: Sample Batch
        Buffer-->>Trainer: Batch Data
        Trainer->>Agent: Update Policy (SAC)
        Agent->>Agent: Q-Learning Update
        Agent->>Agent: Policy Update
        Trainer->>Checkpoint: Save Checkpoint
        Checkpoint->>Checkpoint: SQLite DB Update
        Trainer->>Dashboard: Update Metrics
        Dashboard->>User: Display Progress
    end
    
    AutoMgr->>Trainer: Monitor Health
    AutoMgr->>CARLA: Monitor Status
    alt Process Crashed or Stuck
        AutoMgr->>Trainer: Kill Process
        AutoMgr->>CARLA: Restart if Needed
        AutoMgr->>Trainer: Restart Training
        Trainer->>Checkpoint: Load Latest Checkpoint
        Checkpoint-->>Trainer: Resume Training
    end
```

## 🏗️ Component Structure

```mermaid
graph LR
    subgraph "RL_Agent_SAC/"
        subgraph "carla_env/"
            ENV1[carla_rl_env.py<br/>Main Environment]
            ENV2[carla_connection.py<br/>Connection Manager]
            ENV3[world_manager.py<br/>World Management]
            ENV4[lane_detector.py<br/>Lane Detection]
        end
        
        subgraph "models/"
            MOD1[custom_policy.py<br/>SAC Policy]
            MOD2[vision_encoder.py<br/>ResNet18 Encoder]
            MOD3[sac_policy.py<br/>SAC Implementation]
        end
        
        subgraph "training/"
            TRN1[train_sac.py<br/>Main Training Script]
        end
        
        subgraph "utils/"
            UTIL1[sqlite_checkpoint.py<br/>Checkpoint Manager]
            UTIL2[mixed_device_manager.py<br/>GPU/CPU Manager]
            UTIL3[logging_utils.py<br/>Logging System]
            UTIL4[data_augmentation.py<br/>Image Augmentation]
        end
        
        subgraph "scripts/training/"
            SCR1[auto_manage.py<br/>Auto Management]
        end
        
        subgraph "web_dashboard/"
            WEB1[app_fastapi.py<br/>FastAPI Backend]
            WEB2[react_dashboard/<br/>React Frontend]
        end
    end
    
    subgraph "PythonAPI/"
        API1[carla/<br/>Python Client Library]
        API2[examples/<br/>Example Scripts]
        API3[util/<br/>Utilities]
    end
    
    subgraph "Co-Simulation/"
        CO1[Sumo/<br/>SUMO Integration]
        CO2[PTV-Vissim/<br/>Vissim Integration]
        CO3[Chrono/<br/>Physics Integration]
        CO4[Carsim/<br/>CarSim Integration]
    end
    
    ENV1 --> API1
    MOD1 --> ENV1
    TRN1 --> ENV1
    TRN1 --> MOD1
    TRN1 --> UTIL1
    SCR1 --> TRN1
    SCR1 --> ENV1
    WEB1 --> UTIL1
```

## 🔌 Data Flow Diagram

```mermaid
flowchart TD
    START([Start Training]) --> INIT[Initialize CARLA Connection]
    INIT --> SPAWN[Spawn Vehicle & Sensors]
    SPAWN --> RESET[Reset Environment]
    
    RESET --> OBS[Collect Observations]
    OBS --> RGB[RGB Camera<br/>160x90x3]
    OBS --> DEPTH[Depth Camera<br/>160x90x1]
    OBS --> GPS[GPS Location<br/>x, y, z]
    OBS --> VEL[Velocity<br/>Linear + Angular]
    OBS --> WP[Waypoints<br/>8D Vector]
    
    RGB --> STACK[Frame Stacking<br/>4 Frames]
    DEPTH --> STACK
    STACK --> ENCODER[ResNet18 Encoder<br/>ImageNet Pretrained]
    
    GPS --> FUSION[Feature Fusion]
    VEL --> FUSION
    WP --> FUSION
    ENCODER --> FUSION
    
    FUSION --> POLICY[SAC Policy Network]
    POLICY --> ACTION[Action Output<br/>Steer, Throttle, Brake]
    
    ACTION --> EXEC[Execute in CARLA]
    EXEC --> REWARD[Calculate Reward]
    REWARD --> STORE[Store in Replay Buffer]
    
    STORE --> CHECK{Enough<br/>Samples?}
    CHECK -->|No| OBS
    CHECK -->|Yes| TRAIN[Training Update]
    
    TRAIN --> SAMPLE[Sample Batch from Buffer]
    SAMPLE --> UPDATE[Update SAC Networks]
    UPDATE --> SAVE[Save Checkpoint]
    SAVE --> LOG[Log Metrics]
    LOG --> DASH[Update Dashboard]
    
    DASH --> CHECK2{Episode<br/>Done?}
    CHECK2 -->|No| OBS
    CHECK2 -->|Yes| RESET
    
    SAVE --> MONITOR[Auto Manager Monitor]
    MONITOR --> HEALTH{Health<br/>Check}
    HEALTH -->|OK| CHECK2
    HEALTH -->|Failed| RESTART[Restart Process]
    RESTART --> INIT
```

## 🎯 System Interaction Diagram

```mermaid
graph TB
    subgraph "External Interfaces"
        USER[User/Researcher]
        CARLA_UI[CARLA Simulator UI]
    end
    
    subgraph "Core Services"
        CARLA_SRV[CARLA Server<br/>localhost:2000]
        TRAIN_PROC[Training Process<br/>train_sac.py]
        AUTO_PROC[Auto Manager<br/>auto_manage.py]
        DASH_SRV[Dashboard Server<br/>localhost:5001]
    end
    
    subgraph "Data Storage"
        CHECKPOINT_DB[(SQLite DB<br/>Checkpoints)]
        LOG_FILES[(Log Files<br/>.log)]
        MODEL_FILES[(Model Files<br/>.zip)]
    end
    
    subgraph "Neural Networks"
        ACTOR_NET[Actor Network<br/>Policy]
        CRITIC_NET[Critic Networks<br/>Q-Functions]
        VISION_NET[Vision Encoder<br/>ResNet18]
    end
    
    USER -->|Start/Stop| AUTO_PROC
    USER -->|View Metrics| DASH_SRV
    USER -->|Monitor| CARLA_UI
    
    AUTO_PROC -->|Launch| CARLA_SRV
    AUTO_PROC -->|Start| TRAIN_PROC
    AUTO_PROC -->|Monitor| DASH_SRV
    AUTO_PROC -->|Restart on Failure| TRAIN_PROC
    
    TRAIN_PROC -->|Connect| CARLA_SRV
    TRAIN_PROC -->|Save| CHECKPOINT_DB
    TRAIN_PROC -->|Write| LOG_FILES
    TRAIN_PROC -->|Save Models| MODEL_FILES
    TRAIN_PROC -->|Update Metrics| DASH_SRV
    
    TRAIN_PROC -->|Use| ACTOR_NET
    TRAIN_PROC -->|Use| CRITIC_NET
    TRAIN_PROC -->|Use| VISION_NET
    
    CARLA_SRV -->|Render| CARLA_UI
    CARLA_SRV -->|State/Reward| TRAIN_PROC
    
    DASH_SRV -->|Read| CHECKPOINT_DB
    DASH_SRV -->|Read| LOG_FILES
    DASH_SRV -->|Display| USER
```

## 📦 Module Dependencies

```mermaid
graph TD
    subgraph "External Dependencies"
        SB3[Stable-Baselines3<br/>SAC Algorithm]
        TORCH[PyTorch<br/>Deep Learning]
        CARLA_LIB[CARLA Python API<br/>carla package]
        NUMPY[NumPy]
        GYM[Gymnasium]
    end
    
    subgraph "RL_Agent_SAC Modules"
        TRAIN_MOD[train_sac.py]
        ENV_MOD[carla_rl_env.py]
        POLICY_MOD[custom_policy.py]
        VISION_MOD[vision_encoder.py]
        AUTO_MOD[auto_manage.py]
        CHECK_MOD[sqlite_checkpoint.py]
        UTIL_MOD[utils/*]
    end
    
    subgraph "CARLA Core"
        CARLA_API[PythonAPI/carla/]
    end
    
    TRAIN_MOD --> SB3
    TRAIN_MOD --> ENV_MOD
    TRAIN_MOD --> POLICY_MOD
    TRAIN_MOD --> CHECK_MOD
    TRAIN_MOD --> UTIL_MOD
    
    ENV_MOD --> CARLA_LIB
    ENV_MOD --> GYM
    ENV_MOD --> NUMPY
    
    POLICY_MOD --> TORCH
    POLICY_MOD --> VISION_MOD
    POLICY_MOD --> SB3
    
    VISION_MOD --> TORCH
    
    AUTO_MOD --> ENV_MOD
    AUTO_MOD --> TRAIN_MOD
    
    CHECK_MOD --> NUMPY
    
    CARLA_LIB --> CARLA_API
```

## 🚗 Vehicle Control Flow

```mermaid
stateDiagram-v2
    [*] --> Idle: System Start
    
    Idle --> Connecting: Auto Manager Starts
    Connecting --> Connected: CARLA Server Ready
    Connected --> Initializing: Create Environment
    Initializing --> Ready: Vehicle Spawned
    
    Ready --> Collecting: Episode Start
    Collecting --> Processing: Observation Received
    Processing --> Deciding: Policy Forward Pass
    Deciding --> Acting: Action Selected
    Acting --> Executing: Apply to Vehicle
    Executing --> Observing: Get New State
    Observing --> Rewarding: Calculate Reward
    Rewarding --> Storing: Store in Buffer
    
    Storing --> Training: Buffer Full Enough
    Training --> Updating: Update Networks
    Updating --> Saving: Save Checkpoint
    Saving --> Collecting: Continue Episode
    
    Storing --> Collecting: Continue Episode
    Collecting --> EpisodeEnd: Max Steps/Collision/Goal
    EpisodeEnd --> Ready: Reset Environment
    
    Ready --> Monitoring: Auto Manager Check
    Monitoring --> Ready: Health OK
    Monitoring --> Restarting: Process Failed
    Restarting --> Connecting: Restart System
    
    EpisodeEnd --> [*]: Training Complete
```

## 📈 Training Metrics Flow

```mermaid
graph LR
    subgraph "Data Collection"
        EPISODE[Episode Data]
        STEP[Step Data]
        REWARD[Reward Signal]
    end
    
    subgraph "Training Metrics"
        EP_REWARD[Episode Reward]
        AVG_REWARD[Average Reward]
        SUCCESS[Success Rate]
        COLLISION[Collision Rate]
        DISTANCE[Distance Traveled]
    end
    
    subgraph "Model Metrics"
        LOSS[Loss Values]
        Q_VALUE[Q-Values]
        ENTROPY[Entropy]
        LR[Learning Rate]
    end
    
    subgraph "Storage & Visualization"
        LOGGER[Logger]
        DB[(SQLite DB)]
        DASHBOARD[Dashboard]
        TENSORBOARD[TensorBoard]
    end
    
    EPISODE --> EP_REWARD
    STEP --> REWARD
    REWARD --> AVG_REWARD
    EPISODE --> SUCCESS
    EPISODE --> COLLISION
    EPISODE --> DISTANCE
    
    EP_REWARD --> LOGGER
    AVG_REWARD --> LOGGER
    SUCCESS --> LOGGER
    COLLISION --> LOGGER
    DISTANCE --> LOGGER
    
    LOGGER --> DB
    LOGGER --> DASHBOARD
    LOGGER --> TENSORBOARD
    
    LOSS --> LOGGER
    Q_VALUE --> LOGGER
    ENTROPY --> LOGGER
    LR --> LOGGER
```

---

## 📝 Diagram Descriptions

### 1. Overall Project Architecture
แสดงโครงสร้างหลักของโปรเจกต์ทั้งหมด รวมถึง:
- CARLA Simulator Core (Unreal Engine)
- Python API Layer
- RL Agent SAC System
- Auto-Management System
- Co-Simulation Tools
- Supporting Tools

### 2. Training Flow Sequence
แสดงลำดับการทำงานของระบบ training ตั้งแต่เริ่มต้นจนถึงการ monitor และ restart

### 3. Component Structure
แสดงโครงสร้างไฟล์และโมดูลต่างๆ ในโปรเจกต์

### 4. Data Flow Diagram
แสดง flow ของข้อมูลตั้งแต่การ collect observations จนถึงการ train และ save checkpoint

### 5. System Interaction Diagram
แสดงการทำงานร่วมกันระหว่าง services ต่างๆ

### 6. Module Dependencies
แสดง dependencies ระหว่าง modules และ external libraries

### 7. Vehicle Control Flow
แสดง state machine ของการควบคุม vehicle

### 8. Training Metrics Flow
แสดง flow ของ metrics ตั้งแต่ collection จนถึง visualization

---

**Generated for CARLA 0.9.16 Project**
**Date:** 2026-01-28
**Repository:** [GitHub](https://github.com/Telotubbies/Carla-fullself-driving)

