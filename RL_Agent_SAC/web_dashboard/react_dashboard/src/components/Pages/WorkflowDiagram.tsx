import { Card, CardContent, Typography, Box } from '@mui/material';
import DashboardLayout from '../Layout/DashboardLayout';
import { useQuery } from '@tanstack/react-query';
import { api } from '../../services/api';
import { memo } from 'react';

function WorkflowDiagram() {
  const { data: status } = useQuery({
    queryKey: ['status'],
    queryFn: api.getStatus,
  });

  return (
    <DashboardLayout isRunning={status?.status.running || false}>
      <Box sx={{ maxWidth: 1400, mx: 'auto' }}>
        <Typography variant="h4" gutterBottom sx={{ mb: 4, fontWeight: 700 }}>
          Workflow Diagram
        </Typography>

        <Card>
          <CardContent>
            <Box
              sx={{
                display: 'flex',
                flexDirection: 'column',
                gap: 3,
                p: 3,
                bgcolor: 'background.default',
                borderRadius: 2,
              }}
            >
              {/* Training Loop */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  1. Training Loop
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    flexWrap: 'wrap',
                    gap: 2,
                    mt: 2,
                    p: 2,
                    bgcolor: 'background.paper',
                    borderRadius: 2,
                    border: '2px solid',
                    borderColor: 'primary.main',
                  }}
                >
                  <Box sx={{ flex: 1, minWidth: 200 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Environment Step
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Agent takes action → CARLA simulates → Returns observation & reward
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1, minWidth: 200 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Experience Collection
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Store (state, action, reward, next_state) in replay buffer
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1, minWidth: 200 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Policy Update
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Sample batch → Update Q-networks & policy network
                    </Typography>
                  </Box>
                </Box>
              </Box>

              {/* Data Flow */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  2. Data Flow
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    flexDirection: 'column',
                    gap: 2,
                    mt: 2,
                    p: 2,
                    bgcolor: 'background.paper',
                    borderRadius: 2,
                    border: '2px solid',
                    borderColor: 'info.main',
                  }}
                >
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                    <Box
                      sx={{
                        width: 40,
                        height: 40,
                        borderRadius: '50%',
                        bgcolor: 'info.main',
                        color: 'white',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontWeight: 700,
                      }}
                    >
                      1
                    </Box>
                    <Box sx={{ flex: 1 }}>
                      <Typography variant="subtitle2" fontWeight={600}>
                        CARLA Environment
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Generates RGB+Depth images, GPS, velocity, goal, waypoints
                      </Typography>
                    </Box>
                  </Box>
                  <Box sx={{ pl: 6, color: 'info.main' }}>↓</Box>
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                    <Box
                      sx={{
                        width: 40,
                        height: 40,
                        borderRadius: '50%',
                        bgcolor: 'info.main',
                        color: 'white',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontWeight: 700,
                      }}
                    >
                      2
                    </Box>
                    <Box sx={{ flex: 1 }}>
                      <Typography variant="subtitle2" fontWeight={600}>
                        Data Preprocessing & Normalization
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Normalize GPS/Goal to [-1,1], Distance to [0,1], validate data quality
                      </Typography>
                    </Box>
                  </Box>
                  <Box sx={{ pl: 6, color: 'info.main' }}>↓</Box>
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                    <Box
                      sx={{
                        width: 40,
                        height: 40,
                        borderRadius: '50%',
                        bgcolor: 'info.main',
                        color: 'white',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontWeight: 700,
                      }}
                    >
                      3
                    </Box>
                    <Box sx={{ flex: 1 }}>
                      <Typography variant="subtitle2" fontWeight={600}>
                        Data Augmentation
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Apply color jitter, Gaussian noise, motion blur, random erasing
                      </Typography>
                    </Box>
                  </Box>
                  <Box sx={{ pl: 6, color: 'info.main' }}>↓</Box>
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                    <Box
                      sx={{
                        width: 40,
                        height: 40,
                        borderRadius: '50%',
                        bgcolor: 'info.main',
                        color: 'white',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontWeight: 700,
                      }}
                    >
                      4
                    </Box>
                    <Box sx={{ flex: 1 }}>
                      <Typography variant="subtitle2" fontWeight={600}>
                        Vision Policy Network (ResNet-18 + LSTM)
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        ResNet-18 processes images → LSTM handles temporal → Outputs action (steering, throttle, brake)
                      </Typography>
                    </Box>
                  </Box>
                  <Box sx={{ pl: 6, color: 'info.main' }}>↓</Box>
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                    <Box
                      sx={{
                        width: 40,
                        height: 40,
                        borderRadius: '50%',
                        bgcolor: 'info.main',
                        color: 'white',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontWeight: 700,
                      }}
                    >
                      5
                    </Box>
                    <Box sx={{ flex: 1 }}>
                      <Typography variant="subtitle2" fontWeight={600}>
                        Replay Buffer (250K transitions)
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Stores validated experiences for off-policy learning (SAC)
                      </Typography>
                    </Box>
                  </Box>
                </Box>
              </Box>

              {/* Curriculum Learning */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  3. Curriculum Learning & Reward Optimization
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    gap: 2,
                    mt: 2,
                    p: 2,
                    bgcolor: 'background.paper',
                    borderRadius: 2,
                    border: '2px solid',
                    borderColor: 'success.main',
                  }}
                >
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Gradual Difficulty
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Start easy (difficulty 0.0) → Gradually increase to 1.0
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      No obstacles → Add pedestrians → Add vehicles → Full traffic
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Reward-Based Progression
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Increase difficulty faster when performing well (avg reward > 200)
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Adaptive based on last 50 episodes
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Optimized Rewards
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Episode timeout: 60s, increased positive rewards, reduced penalties
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Better learning signal for stable training
                    </Typography>
                  </Box>
                </Box>
              </Box>

              {/* Checkpoint System */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  4. Checkpoint System
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    gap: 2,
                    mt: 2,
                    p: 2,
                    bgcolor: 'background.paper',
                    borderRadius: 2,
                    border: '2px solid',
                    borderColor: 'success.main',
                  }}
                >
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Automatic Compression
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      ZIP_DEFLATED compression (~58% size reduction)
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Every 2000 steps, auto-compressed
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      SQLite Checkpoints
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Compressed model with metadata stored in database
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Efficient storage, fast resume
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Auto-Cleanup
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Automatic disk space management
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Keeps only latest checkpoint, cleans old logs
                    </Typography>
                  </Box>
                </Box>
              </Box>

              {/* Monitoring */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  5. Monitoring & Dashboard
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    gap: 2,
                    mt: 2,
                    p: 2,
                    bgcolor: 'background.paper',
                    borderRadius: 2,
                    border: '2px solid',
                    borderColor: 'warning.main',
                  }}
                >
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Production Dashboard
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      FastAPI backend with auto-refresh (every 1 min)
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Real-time metrics, system resources, CPU temp
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Logs Viewer
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      System logs and training logs with auto-refresh (every 2s)
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Real-time log streaming
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Training Charts
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Reward trends, episode statistics, curriculum progress
                    </Typography>
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Auto-update every 2 seconds
                    </Typography>
                  </Box>
                </Box>
              </Box>

              {/* Auto-Management */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  6. Auto-Management System
                </Typography>
                <Box
                  sx={{
                    display: 'flex',
                    gap: 2,
                    mt: 2,
                    p: 2,
                    bgcolor: 'background.paper',
                    borderRadius: 2,
                    border: '2px solid',
                    borderColor: 'error.main',
                  }}
                >
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Process Monitoring
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Health checks every 30s for CARLA, training, dashboard
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Automatic Restart
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Auto-restart on failures, stuck detection (30min threshold)
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Disk Space Management
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Automatic cleanup every 6 hours or when disk < 10GB free
                    </Typography>
                  </Box>
                </Box>
              </Box>
            </Box>
          </CardContent>
        </Card>
      </Box>
    </DashboardLayout>
  );
}

export default memo(WorkflowDiagram);

