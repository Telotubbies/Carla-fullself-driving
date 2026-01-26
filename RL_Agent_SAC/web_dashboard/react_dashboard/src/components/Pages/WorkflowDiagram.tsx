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
                        Generates RGB images, GPS, velocity, distance to goal
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
                        Data Augmentation
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Apply color jitter, noise, blur, random erasing to images
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
                        Vision Policy Network
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        CNN processes images → Outputs action (steering, throttle, brake)
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
                        Replay Buffer
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Stores experiences for off-policy learning
                      </Typography>
                    </Box>
                  </Box>
                </Box>
              </Box>

              {/* Checkpoint System */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  3. Checkpoint System
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
                      Regular Checkpoints
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Saved every 1000 steps (ZIP format)
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      SQLite Checkpoints
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Compressed model stored in database
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Enhanced Checkpoints
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Full training state with metadata
                    </Typography>
                  </Box>
                </Box>
              </Box>

              {/* Monitoring */}
              <Box>
                <Typography variant="h6" gutterBottom fontWeight={700} color="primary">
                  4. Monitoring & Dashboard
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
                      Real-time Metrics
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Training progress, rewards, system resources
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      Logs Viewer
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      System logs and training logs with auto-refresh
                    </Typography>
                  </Box>
                  <Box sx={{ flex: 1 }}>
                    <Typography variant="subtitle2" fontWeight={600} gutterBottom>
                      History Charts
                    </Typography>
                    <Typography variant="body2" color="text.secondary">
                      Reward trends and training statistics
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

