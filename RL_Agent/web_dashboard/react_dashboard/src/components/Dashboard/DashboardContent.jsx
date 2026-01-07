import React, { useState, useEffect } from 'react'
import { Grid, Box, CircularProgress, Alert, Fade } from '@mui/material'
import { useQuery } from '@tanstack/react-query'
import { dashboardAPI } from '../../services/api'
import StatusCard from './StatusCard'
import ProgressCard from './ProgressCard'
import MetricsCard from './MetricsCard'
import SystemResourcesCard from './SystemResourcesCard'
import TrainingChart from './TrainingChart'
import CheckpointAndLogsCard from './CheckpointAndLogsCard'

function DashboardContent() {
  const [refreshInterval, setRefreshInterval] = useState(2000)

  // Fetch status with auto-refresh
  const { data: statusData, isLoading, error } = useQuery({
    queryKey: ['dashboardStatus'],
    queryFn: () => dashboardAPI.getStatus().then(res => res.data),
    refetchInterval: refreshInterval,
    refetchOnWindowFocus: true,
    staleTime: 1000,
  })

  if (isLoading && !statusData) {
    return (
      <Box display="flex" justifyContent="center" alignItems="center" minHeight="60vh">
        <CircularProgress size={60} />
      </Box>
    )
  }

  if (error) {
    return (
      <Alert severity="error" sx={{ mb: 2 }}>
        Failed to load dashboard data: {error.message}
      </Alert>
    )
  }

  const status = statusData?.status || {}
  const metrics = statusData?.metrics || {}
  const system = statusData?.system || {}
  const progress = statusData?.progress || {}
  const checkpoint = statusData?.checkpoint || null
  const power = statusData?.power || {}

  return (
    <Fade in={true} timeout={500}>
      <Box>
        {/* Header Status Bar */}
        <Box mb={3}>
          <StatusCard
            status={status}
            checkpoint={checkpoint}
            timestamp={statusData?.timestamp}
          />
        </Box>

        {/* Main Grid */}
        <Grid container spacing={3}>
        {/* Left Column - Training Info */}
        <Grid item xs={12} lg={8}>
          <Grid container spacing={3}>
            {/* Progress Card */}
            <Grid item xs={12}>
              <ProgressCard progress={progress} />
            </Grid>

            {/* Metrics Card */}
            <Grid item xs={12}>
              <MetricsCard metrics={metrics} />
            </Grid>

            {/* Training Chart */}
            <Grid item xs={12}>
              <TrainingChart />
            </Grid>
          </Grid>
        </Grid>

        {/* Right Column - System & Logs */}
        <Grid item xs={12} lg={4}>
          <Grid container spacing={3}>
            {/* System Resources */}
            <Grid item xs={12}>
              <SystemResourcesCard
                system={system}
                power={power}
                status={status}
              />
            </Grid>

            {/* Checkpoint & Logs */}
            <Grid item xs={12}>
              <CheckpointAndLogsCard checkpoint={checkpoint} />
            </Grid>
          </Grid>
        </Grid>
      </Grid>
      </Box>
    </Fade>
  )
}

export default DashboardContent

