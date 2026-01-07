import React from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  LinearProgress,
} from '@mui/material'
import { TrendingUp as TrendingUpIcon } from '@mui/icons-material'

function ProgressCard({ progress }) {
  const current = progress?.current || 0
  const target = progress?.target || 500000
  const percentage = progress?.percentage || 0
  const remaining = progress?.remaining || target

  return (
    <Card>
      <CardContent>
        <Box display="flex" alignItems="center" mb={2}>
          <TrendingUpIcon sx={{ mr: 1, color: 'primary.main' }} />
          <Typography variant="h6" fontWeight={600}>
            Training Progress
          </Typography>
        </Box>

        <Box mb={2}>
          <Box display="flex" justifyContent="space-between" mb={1}>
            <Typography variant="body2" color="text.secondary">
              Current Steps
            </Typography>
            <Typography variant="h6" fontWeight={600} color="primary.main">
              {current.toLocaleString()}
            </Typography>
          </Box>
          <Box display="flex" justifyContent="space-between" mb={2}>
            <Typography variant="body2" color="text.secondary">
              Target Steps
            </Typography>
            <Typography variant="body1" fontWeight={500}>
              {target.toLocaleString()}
            </Typography>
          </Box>
          
          <LinearProgress
            variant="determinate"
            value={Math.min(percentage, 100)}
            sx={{
              height: 12,
              borderRadius: 6,
              bgcolor: 'grey.200',
              '& .MuiLinearProgress-bar': {
                borderRadius: 6,
                bgcolor: 'primary.main',
              },
            }}
          />
          
          <Box display="flex" justifyContent="space-between" mt={1}>
            <Typography variant="caption" color="text.secondary">
              {percentage.toFixed(1)}% Complete
            </Typography>
            <Typography variant="caption" color="text.secondary">
              {remaining.toLocaleString()} steps remaining
            </Typography>
          </Box>
        </Box>
      </CardContent>
    </Card>
  )
}

export default ProgressCard

