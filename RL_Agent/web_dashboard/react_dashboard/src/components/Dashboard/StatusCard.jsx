import React from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  Chip,
  LinearProgress,
  Fade,
  Grid,
} from '@mui/material'
import {
  PlayCircle as PlayIcon,
  StopCircle as StopIcon,
  CheckCircle as CheckIcon,
} from '@mui/icons-material'
import { format } from 'date-fns'

function StatusCard({ status, checkpoint, timestamp }) {
  const isRunning = status?.running === true
  const statusColor = isRunning ? 'success' : 'error'
  const statusIcon = isRunning ? <PlayIcon /> : <StopIcon />
  const statusText = isRunning ? 'Training' : 'Stopped'

  return (
    <Fade in={true} timeout={300}>
      <Card sx={{ 
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        color: 'white',
      }}>
        <CardContent>
          <Box display="flex" justifyContent="space-between" alignItems="center" mb={2}>
            <Box>
              <Typography variant="h5" fontWeight={700} gutterBottom sx={{ color: 'white' }}>
                Training Status
              </Typography>
              <Typography variant="body2" sx={{ color: 'rgba(255,255,255,0.8)' }}>
                Last updated: {timestamp ? format(new Date(timestamp), 'PPpp') : 'Never'}
              </Typography>
            </Box>
            <Chip
              icon={statusIcon}
              label={statusText}
              color={statusColor}
              size="large"
              sx={{ 
                fontWeight: 600,
                bgcolor: isRunning ? 'rgba(16, 185, 129, 0.9)' : 'rgba(239, 68, 68, 0.9)',
                color: 'white',
                '& .MuiChip-icon': {
                  color: 'white',
                },
              }}
            />
          </Box>

          {isRunning && (
            <Box sx={{ bgcolor: 'rgba(255,255,255,0.1)', p: 2, borderRadius: 2 }}>
              <Grid container spacing={2}>
                <Grid item xs={4}>
                  <Typography variant="caption" sx={{ color: 'rgba(255,255,255,0.8)' }}>
                    Process ID
                  </Typography>
                  <Typography variant="body1" fontWeight={600} sx={{ color: 'white' }}>
                    {status.pid || 'N/A'}
                  </Typography>
                </Grid>
                <Grid item xs={4}>
                  <Typography variant="caption" sx={{ color: 'rgba(255,255,255,0.8)' }}>
                    CPU Usage
                  </Typography>
                  <Typography variant="body1" fontWeight={600} sx={{ color: 'white' }}>
                    {status.cpu || '0.0'}%
                  </Typography>
                </Grid>
                <Grid item xs={4}>
                  <Typography variant="caption" sx={{ color: 'rgba(255,255,255,0.8)' }}>
                    Memory
                  </Typography>
                  <Typography variant="body1" fontWeight={600} sx={{ color: 'white' }}>
                    {status.memory || '0.0'}%
                  </Typography>
                </Grid>
              </Grid>
              {checkpoint && (
                <Box mt={2} pt={2} sx={{ borderTop: '1px solid rgba(255,255,255,0.2)' }}>
                  <Typography variant="caption" sx={{ color: 'rgba(255,255,255,0.8)' }}>
                    Current Checkpoint
                  </Typography>
                  <Typography variant="h6" fontWeight={700} sx={{ color: 'white' }}>
                    {checkpoint.timestep?.toLocaleString() || 'N/A'} steps
                  </Typography>
                </Box>
              )}
            </Box>
          )}
        </CardContent>
      </Card>
    </Fade>
  )
}

export default StatusCard
