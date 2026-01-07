import React from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  Chip,
} from '@mui/material'
import {
  Save as SaveIcon,
  CalendarToday as CalendarIcon,
} from '@mui/icons-material'
import { format } from 'date-fns'

function CheckpointInfo({ checkpoint }) {
  if (!checkpoint) {
    return (
      <Card>
        <CardContent>
          <Typography variant="h6" fontWeight={600} gutterBottom>
            Checkpoint
          </Typography>
          <Typography variant="body2" color="text.secondary">
            No checkpoint available
          </Typography>
        </CardContent>
      </Card>
    )
  }

  return (
    <Card>
      <CardContent>
        <Box display="flex" alignItems="center" mb={2}>
          <SaveIcon sx={{ mr: 1, color: 'primary.main' }} />
          <Typography variant="h6" fontWeight={600}>
            Latest Checkpoint
          </Typography>
        </Box>

        <Box mb={2}>
          <Chip
            label={`${checkpoint.timestep?.toLocaleString() || 0} steps`}
            color="primary"
            size="small"
            sx={{ mb: 1 }}
          />
        </Box>

        {checkpoint.created_at && (
          <Box display="flex" alignItems="center" color="text.secondary">
            <CalendarIcon fontSize="small" sx={{ mr: 0.5 }} />
            <Typography variant="caption">
              {checkpoint.created_at}
            </Typography>
          </Box>
        )}

        {checkpoint.episode && (
          <Box mt={1}>
            <Typography variant="caption" color="text.secondary">
              Episode: {checkpoint.episode}
            </Typography>
          </Box>
        )}

        {checkpoint.reward !== null && (
          <Box mt={1}>
            <Typography variant="caption" color="text.secondary">
              Reward: {checkpoint.reward.toFixed(2)}
            </Typography>
          </Box>
        )}
      </CardContent>
    </Card>
  )
}

export default CheckpointInfo

