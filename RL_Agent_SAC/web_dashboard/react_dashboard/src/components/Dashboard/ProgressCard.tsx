import { Card, CardContent, CardHeader, Typography, Box, CircularProgress } from '@mui/material';
import { memo, useMemo } from 'react';

interface ProgressCardProps {
  progress: {
    current: number;
    started: number;
    target: number;
    remaining: number;
    percentage: number;
  };
}

function ProgressCard({ progress }: ProgressCardProps) {
  const remainingSteps = useMemo(() => progress.remaining.toLocaleString(), [progress.remaining]);

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader title="Training Overview" sx={{ pb: 1 }} />
      <CardContent sx={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', pt: 1 }}>
        <Box sx={{ position: 'relative', display: 'inline-flex', mb: 2 }}>
          <CircularProgress
            variant="determinate"
            value={progress.percentage}
            size={100}
            thickness={4}
            sx={{
              color: 'primary.main',
              '& .MuiCircularProgress-circle': {
                strokeLinecap: 'round',
              },
            }}
          />
          <Box
            sx={{
              top: 0,
              left: 0,
              bottom: 0,
              right: 0,
              position: 'absolute',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
            }}
          >
            <Typography variant="h6" component="div" color="primary" fontWeight={700}>
              {progress.percentage.toFixed(1)}%
            </Typography>
          </Box>
        </Box>

        <Box sx={{ textAlign: 'center', width: '100%', mb: 2 }}>
          <Typography variant="body2" color="text.secondary" gutterBottom>
            Remaining Steps
          </Typography>
          <Typography variant="h6" color="primary" fontWeight={700}>
            {remainingSteps}
          </Typography>
        </Box>

        <Box sx={{ width: '100%', mt: 'auto' }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
            <Typography variant="caption" color="text.secondary">
              Started
            </Typography>
            <Typography variant="caption" fontWeight={600}>
              {progress.started.toLocaleString()}
            </Typography>
          </Box>
          <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
            <Typography variant="caption" color="text.secondary">
              Target
            </Typography>
            <Typography variant="caption" fontWeight={600}>
              {progress.target.toLocaleString()}
            </Typography>
          </Box>
        </Box>
      </CardContent>
    </Card>
  );
}

export default memo(ProgressCard);
