import { Card, CardContent, CardHeader, Typography, Box, LinearProgress } from '@mui/material';
import { TrendingUp, Speed, CheckCircle } from '@mui/icons-material';
import { memo } from 'react';
import type { StatusResponse } from '../../types';
import { format } from 'date-fns';

interface StatusCardProps {
  status: StatusResponse;
}

function StatusCard({ status }: StatusCardProps) {
  const progress = status.progress.percentage;
  const currentStep = status.progress.current;
  const targetStep = status.progress.target;

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader
        title="Training Progress"
        subheader={`Last update: ${format(new Date(status.timestamp), 'PPpp')}`}
        sx={{ pb: 1 }}
      />
      <CardContent sx={{ flex: 1, display: 'flex', flexDirection: 'column', pt: 1 }}>
        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: { xs: 'repeat(2, 1fr)', sm: 'repeat(4, 1fr)' },
            gap: 2,
            mb: 2,
          }}
        >
          <Box textAlign="center">
            <Typography variant="h5" color="primary" fontWeight={700}>
              {currentStep.toLocaleString()}
            </Typography>
            <Typography variant="caption" color="text.secondary">
              Current Step
            </Typography>
          </Box>
          <Box textAlign="center">
            <Typography variant="h5" color="text.secondary" fontWeight={700}>
              {targetStep.toLocaleString()}
            </Typography>
            <Typography variant="caption" color="text.secondary">
              Target Steps
            </Typography>
          </Box>
          <Box textAlign="center">
            <Typography variant="h5" color="success.main" fontWeight={700}>
              {progress.toFixed(1)}%
            </Typography>
            <Typography variant="caption" color="text.secondary">
              Progress
            </Typography>
          </Box>
          <Box textAlign="center">
            <Typography variant="h5" color="info.main" fontWeight={700}>
              {status.metrics.episode_length || '-'}
            </Typography>
            <Typography variant="caption" color="text.secondary">
              Episode Length
            </Typography>
          </Box>
        </Box>

        <Box sx={{ mb: 2 }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
            <Typography variant="caption" color="text.secondary">
              Training Progress
            </Typography>
            <Typography variant="caption" color="text.secondary">
              {progress.toFixed(2)}%
            </Typography>
          </Box>
          <LinearProgress
            variant="determinate"
            value={progress}
            sx={{
              height: 10,
              borderRadius: 5,
              bgcolor: 'grey.200',
              '& .MuiLinearProgress-bar': {
                borderRadius: 5,
                background: 'linear-gradient(90deg, #667eea 0%, #764ba2 100%)',
              },
            }}
          />
        </Box>

        <Box
          sx={{
            display: 'grid',
            gridTemplateColumns: { xs: '1fr', sm: 'repeat(3, 1fr)' },
            gap: 1.5,
            mt: 'auto',
          }}
        >
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <TrendingUp color="primary" fontSize="small" />
            <Box>
              <Typography variant="caption" color="text.secondary">
                Last Reward
              </Typography>
              <Typography variant="body2" fontWeight={600}>
                {status.metrics.last_reward?.toFixed(2) || '-'}
              </Typography>
            </Box>
          </Box>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <CheckCircle color="success" fontSize="small" />
            <Box>
              <Typography variant="caption" color="text.secondary">
                Episode Reward
              </Typography>
              <Typography variant="body2" fontWeight={600}>
                {status.metrics.episode_reward?.toFixed(2) || '-'}
              </Typography>
            </Box>
          </Box>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <Speed color="info" fontSize="small" />
            <Box>
              <Typography variant="caption" color="text.secondary">
                Algorithm
              </Typography>
              <Typography variant="body2" fontWeight={600}>
                {status.algorithm}
              </Typography>
            </Box>
          </Box>
        </Box>
      </CardContent>
    </Card>
  );
}

export default memo(StatusCard);
