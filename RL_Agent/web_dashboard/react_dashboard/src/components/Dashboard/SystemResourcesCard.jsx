import React from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  LinearProgress,
  Grid,
  Divider,
} from '@mui/material'
import {
  Memory as MemoryIcon,
  Speed as SpeedIcon,
  DeviceThermostat as TempIcon,
  Bolt as PowerIcon,
} from '@mui/icons-material'

function SystemResourcesCard({ system, power, status }) {
  const cpu = system?.cpu || {}
  const gpu = system?.gpu || {}
  const cpuUsage = cpu.usage || 0
  const gpuUsage = gpu.usage || 0
  const gpuTemp = gpu.temp || null
  const cpuTemp = cpu.temp || null
  const totalPower = power?.current_power_watt || 0
  const energyKwh = power?.cumulative_energy_kwh || 0
  const costBaht = power?.cumulative_cost_baht || (energyKwh * (power?.rate_per_kwh || 7))
  const ratePerKwh = power?.rate_per_kwh || 7
  const sessionHours = power?.session_runtime_hours || 0
  
  // Ensure costBaht is a valid number
  const displayCost = (typeof costBaht === 'number' && !isNaN(costBaht)) ? costBaht : 0

  return (
    <Card>
      <CardContent>
        <Typography variant="h6" fontWeight={600} gutterBottom>
          System Resources
        </Typography>
        <Divider sx={{ my: 2 }} />

        {/* CPU */}
        <Box mb={3}>
          <Box display="flex" justifyContent="space-between" mb={1}>
            <Typography variant="body2" color="text.secondary">
              CPU Usage
            </Typography>
            <Typography variant="body2" fontWeight={600}>
              {cpuUsage.toFixed(1)}%
            </Typography>
          </Box>
          <LinearProgress
            variant="determinate"
            value={cpuUsage}
            sx={{ height: 8, borderRadius: 4 }}
            color="primary"
          />
          {cpuTemp && (
            <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
              Temperature: {cpuTemp.toFixed(1)}°C
            </Typography>
          )}
        </Box>

        {/* GPU */}
        {gpu.name && (
          <Box mb={3}>
            <Box display="flex" justifyContent="space-between" mb={1}>
              <Typography variant="body2" color="text.secondary">
                GPU: {gpu.name}
              </Typography>
              <Typography variant="body2" fontWeight={600}>
                {gpuUsage.toFixed(1)}%
              </Typography>
            </Box>
            <LinearProgress
              variant="determinate"
              value={gpuUsage}
              sx={{ height: 8, borderRadius: 4 }}
              color="secondary"
            />
            {gpuTemp && (
              <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                Temperature: {gpuTemp.toFixed(1)}°C
              </Typography>
            )}
            {gpu.memory_percent && (
              <Box mt={1}>
                <Typography variant="caption" color="text.secondary">
                  Memory: {gpu.memory_percent.toFixed(1)}% ({gpu.memory_used_gb?.toFixed(2) || 0} / {gpu.memory_total_gb?.toFixed(2) || 0} GB)
                </Typography>
              </Box>
            )}
          </Box>
        )}

        {/* Power Consumption */}
        {totalPower > 0 && (
          <Box>
            <Divider sx={{ my: 2 }} />
            <Box display="flex" alignItems="center" mb={1}>
              <PowerIcon sx={{ mr: 1, color: 'warning.main', fontSize: 20 }} />
              <Typography variant="subtitle2" fontWeight={600}>
                Power Consumption
              </Typography>
            </Box>
            <Grid container spacing={1.5}>
              <Grid item xs={6}>
                <Typography variant="caption" color="text.secondary">
                  Current Power
                </Typography>
                <Typography variant="body1" fontWeight={600}>
                  {totalPower.toFixed(0)} W
                </Typography>
              </Grid>
              <Grid item xs={6}>
                <Typography variant="caption" color="text.secondary">
                  Energy Used
                </Typography>
                <Typography variant="body1" fontWeight={600}>
                  {energyKwh.toFixed(2)} kWh
                </Typography>
              </Grid>
              <Grid item xs={12}>
                <Box 
                  sx={{ 
                    bgcolor: 'success.light', 
                    p: 1.5, 
                    borderRadius: 2,
                    border: '1px solid',
                    borderColor: 'success.main',
                  }}
                >
                  <Typography variant="caption" color="text.secondary" display="block">
                    Total Cost (Rate: ฿{ratePerKwh.toFixed(0)}/kWh)
                  </Typography>
                  <Typography variant="h6" fontWeight={700} color="success.dark">
                    ฿{displayCost.toFixed(2)}
                  </Typography>
                </Box>
              </Grid>
              {sessionHours > 0 && (
                <Grid item xs={12}>
                  <Typography variant="caption" color="text.secondary">
                    Session Runtime: {sessionHours.toFixed(1)} hours
                  </Typography>
                </Grid>
              )}
            </Grid>
          </Box>
        )}
      </CardContent>
    </Card>
  )
}

export default SystemResourcesCard

