import React from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  Grid,
  Divider,
} from '@mui/material'
import {
  EmojiEvents as TrophyIcon,
  Speed as SpeedIcon,
  Route as RouteIcon,
  Timer as TimerIcon,
  Layers as BatchIcon,
  PlaylistPlay as RolloutIcon,
  Replay as IterationIcon,
} from '@mui/icons-material'

const MetricItem = ({ icon, label, value, color = 'text.primary' }) => (
  <Box>
    <Box display="flex" alignItems="center" mb={1}>
      <Box sx={{ color, mr: 1 }}>{icon}</Box>
      <Typography variant="caption" color="text.secondary">
        {label}
      </Typography>
    </Box>
    <Typography variant="h6" fontWeight={600} color={color}>
      {value}
    </Typography>
  </Box>
)

function MetricsCard({ metrics }) {
  const reward = metrics?.reward ?? null
  const episodeReward = metrics?.episode_reward ?? null
  const episodeLength = metrics?.episode_length ?? null
  const fps = metrics?.fps ?? null
  const iterations = metrics?.iterations ?? null
  const batchSize = metrics?.batch_size ?? null
  const nSteps = metrics?.n_steps ?? null
  const rolloutCount = metrics?.rollout_count ?? null
  const episodeCount = metrics?.episode_count ?? null

  return (
    <Card>
      <CardContent>
        <Typography variant="h6" fontWeight={600} gutterBottom>
          Training Metrics
        </Typography>
        <Divider sx={{ my: 2 }} />
        
        {/* Reward Section */}
        <Typography variant="subtitle2" fontWeight={600} gutterBottom sx={{ mt: 1 }}>
          Rewards
        </Typography>
        <Grid container spacing={2} sx={{ mb: 2 }}>
          <Grid item xs={6}>
            <MetricItem
              icon={<TrophyIcon fontSize="small" />}
              label="Current Reward"
              value={reward !== null ? reward.toFixed(2) : '-'}
              color="success.main"
            />
          </Grid>
          <Grid item xs={6}>
            <MetricItem
              icon={<TrophyIcon fontSize="small" />}
              label="Episode Reward"
              value={episodeReward !== null ? episodeReward.toFixed(2) : '-'}
              color="success.main"
            />
          </Grid>
        </Grid>

        <Divider sx={{ my: 2 }} />

        {/* Episode Section */}
        <Typography variant="subtitle2" fontWeight={600} gutterBottom>
          Episodes
        </Typography>
        <Grid container spacing={2} sx={{ mb: 2 }}>
          <Grid item xs={6}>
            <MetricItem
              icon={<RouteIcon fontSize="small" />}
              label="Episode Length"
              value={episodeLength ?? '-'}
              color="info.main"
            />
          </Grid>
          <Grid item xs={6}>
            <MetricItem
              icon={<TimerIcon fontSize="small" />}
              label="Total Episodes"
              value={episodeCount ?? '-'}
              color="info.main"
            />
          </Grid>
        </Grid>

        <Divider sx={{ my: 2 }} />

        {/* Batch & Rollout Section */}
        <Typography variant="subtitle2" fontWeight={600} gutterBottom>
          Batch & Rollout
        </Typography>
        <Grid container spacing={2} sx={{ mb: 2 }}>
          <Grid item xs={6}>
            <MetricItem
              icon={<BatchIcon fontSize="small" />}
              label="Batch Size"
              value={batchSize ?? '-'}
              color="primary.main"
            />
          </Grid>
          <Grid item xs={6}>
            <MetricItem
              icon={<RolloutIcon fontSize="small" />}
              label="Rollout Steps (n_steps)"
              value={nSteps ?? '-'}
              color="primary.main"
            />
          </Grid>
          <Grid item xs={6}>
            <MetricItem
              icon={<IterationIcon fontSize="small" />}
              label="Iterations"
              value={iterations ?? '-'}
              color="primary.main"
            />
          </Grid>
          <Grid item xs={6}>
            <MetricItem
              icon={<RolloutIcon fontSize="small" />}
              label="Rollout Count"
              value={rolloutCount ?? '-'}
              color="primary.main"
            />
          </Grid>
        </Grid>

        <Divider sx={{ my: 2 }} />

        {/* Performance Section */}
        <Typography variant="subtitle2" fontWeight={600} gutterBottom>
          Performance
        </Typography>
        <Grid container spacing={2}>
          <Grid item xs={6}>
            <MetricItem
              icon={<SpeedIcon fontSize="small" />}
              label="FPS"
              value={fps ?? '-'}
              color="warning.main"
            />
          </Grid>
        </Grid>
      </CardContent>
    </Card>
  )
}

export default MetricsCard
