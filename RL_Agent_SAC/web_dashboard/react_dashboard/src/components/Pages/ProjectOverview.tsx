import { Card, CardContent, CardHeader, Typography, Box, Grid, Chip } from '@mui/material';
import DashboardLayout from '../Layout/DashboardLayout';
import { useQuery } from '@tanstack/react-query';
import { api } from '../../services/api';
import { memo } from 'react';

function ProjectOverview() {
  const { data: status } = useQuery({
    queryKey: ['status'],
    queryFn: api.getStatus,
  });

  return (
    <DashboardLayout isRunning={status?.status.running || false}>
      <Box sx={{ maxWidth: 1200, mx: 'auto' }}>
        <Typography variant="h4" gutterBottom sx={{ mb: 4, fontWeight: 700 }}>
          Project Overview
        </Typography>

        <Grid container spacing={3}>
          {/* Project Info */}
          <Grid item xs={12} md={6}>
            <Card>
              <CardHeader title="Project Information" />
              <CardContent>
                <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Project Name
                    </Typography>
                    <Typography variant="h6" fontWeight={600}>
                      CARLA RL Agent - Vision-Based Autonomous Driving
                    </Typography>
                  </Box>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Algorithm
                    </Typography>
                    <Chip label={status?.algorithm || 'SAC'} color="primary" />
                  </Box>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Environment
                    </Typography>
                    <Typography variant="body1">
                      CARLA Simulator (Town01_Opt)
                    </Typography>
                  </Box>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Observation Space
                    </Typography>
                    <Typography variant="body1">
                      Vision-based (RGB images) + GPS + Velocity + Distance to Goal
                    </Typography>
                  </Box>
                </Box>
              </CardContent>
            </Card>
          </Grid>

          {/* Training Status */}
          <Grid item xs={12} md={6}>
            <Card>
              <CardHeader title="Training Status" />
              <CardContent>
                <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Current Step
                    </Typography>
                    <Typography variant="h5" fontWeight={700} color="primary">
                      {status?.progress.current.toLocaleString() || '0'}
                    </Typography>
                  </Box>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Target Steps
                    </Typography>
                    <Typography variant="h6" fontWeight={600}>
                      {status?.progress.target.toLocaleString() || '500,000'}
                    </Typography>
                  </Box>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Progress
                    </Typography>
                    <Typography variant="h6" fontWeight={600} color="success.main">
                      {status?.progress.percentage.toFixed(2) || '0'}%
                    </Typography>
                  </Box>
                  <Box>
                    <Typography variant="body2" color="text.secondary" gutterBottom>
                      Status
                    </Typography>
                    <Chip
                      label={status?.status.running ? 'Training' : 'Stopped'}
                      color={status?.status.running ? 'success' : 'error'}
                      size="small"
                    />
                  </Box>
                </Box>
              </CardContent>
            </Card>
          </Grid>

          {/* Vision Network Architecture */}
          <Grid item xs={12}>
            <Card>
              <CardHeader title="Vision Network Architecture" />
              <CardContent>
                <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                  <Box>
                    <Typography variant="h6" gutterBottom>
                      Backbone Network
                    </Typography>
                    <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1, mt: 1 }}>
                      <Chip 
                        label="ResNet-18" 
                        color="primary" 
                        sx={{ fontWeight: 600 }}
                      />
                      <Chip 
                        label="ImageNet Pretrained" 
                        color="success" 
                        sx={{ fontWeight: 600 }}
                      />
                      <Chip 
                        label="4-Channel Input (RGB + Depth)" 
                        color="info" 
                      />
                    </Box>
                    <Typography variant="body2" color="text.secondary" sx={{ mt: 2 }}>
                      Using ResNet-18 pretrained on ImageNet as the vision encoder backbone. 
                      The first convolutional layer is adapted to accept 4-channel input (RGB + depth).
                      Pretrained weights are transferred from ImageNet for better feature extraction.
                    </Typography>
                  </Box>
                  <Box>
                    <Typography variant="h6" gutterBottom>
                      Architecture Details
                    </Typography>
                    <Box component="ul" sx={{ pl: 3, mt: 1 }}>
                      <li>
                        <Typography variant="body2">
                          <strong>Encoder:</strong> ResNet-18 (ImageNet pretrained)
                        </Typography>
                      </li>
                      <li>
                        <Typography variant="body2">
                          <strong>Temporal:</strong> LSTM (2 layers, 256 hidden units)
                        </Typography>
                      </li>
                      <li>
                        <Typography variant="body2">
                          <strong>Input:</strong> 4 stacked frames (90x160) with RGB + depth channels
                        </Typography>
                      </li>
                      <li>
                        <Typography variant="body2">
                          <strong>Output:</strong> 512-dimensional feature vector
                        </Typography>
                      </li>
                    </Box>
                  </Box>
                </Box>
              </CardContent>
            </Card>
          </Grid>

          {/* System Architecture */}
          <Grid item xs={12}>
            <Card>
              <CardHeader title="System Architecture" />
              <CardContent>
                <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                  <Box>
                    <Typography variant="h6" gutterBottom>
                      Components
                    </Typography>
                    <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1, mt: 1 }}>
                      <Chip label="CARLA Environment" color="primary" />
                      <Chip label="SAC Algorithm" color="primary" />
                      <Chip label="ResNet-18 Vision Encoder" color="primary" />
                      <Chip label="LSTM Temporal Encoder" color="primary" />
                      <Chip label="Replay Buffer" color="primary" />
                      <Chip label="Checkpoint Manager" color="primary" />
                      <Chip label="Web Dashboard" color="primary" />
                    </Box>
                  </Box>
                  <Box>
                    <Typography variant="h6" gutterBottom>
                      Features
                    </Typography>
                    <Box component="ul" sx={{ pl: 3, mt: 1 }}>
                      <li>
                        <Typography variant="body2">ResNet-18 pretrained on ImageNet for vision encoding</Typography>
                      </li>
                      <li>
                        <Typography variant="body2">Vision-based lane detection (multiscale Canny)</Typography>
                      </li>
                      <li>
                        <Typography variant="body2">Data augmentation (color jitter, noise, blur, erasing)</Typography>
                      </li>
                      <li>
                        <Typography variant="body2">Mixed device training (GPU/CPU)</Typography>
                      </li>
                      <li>
                        <Typography variant="body2">SQLite checkpoint management</Typography>
                      </li>
                      <li>
                        <Typography variant="body2">Real-time monitoring dashboard</Typography>
                      </li>
                    </Box>
                  </Box>
                </Box>
              </CardContent>
            </Card>
          </Grid>
        </Grid>
      </Box>
    </DashboardLayout>
  );
}

export default memo(ProjectOverview);
