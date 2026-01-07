import React, { useState, useEffect } from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  ToggleButton,
  ToggleButtonGroup,
} from '@mui/material'
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from 'recharts'
import { useQuery } from '@tanstack/react-query'
import { dashboardAPI } from '../../services/api'

function TrainingChart() {
  const [metricType, setMetricType] = useState('reward')

  const { data: historyData } = useQuery({
    queryKey: ['metricsHistory'],
    queryFn: () => dashboardAPI.getMetricsHistory().then(res => res.data),
    refetchInterval: 10000,
    staleTime: 5000,
  })

  const chartData = React.useMemo(() => {
    if (!historyData) return []
    
    const timesteps = historyData.timesteps || []
    const rewards = historyData.rewards || []
    const episodes = historyData.episodes || []

    return timesteps.map((ts, idx) => ({
      index: idx + 1, // Use index for x-axis (easier to read)
      timestep: ts,   // Keep timestep for tooltip
      reward: rewards[idx] || 0,
      episode: episodes[idx] || 0,
    }))
  }, [historyData])

  const handleMetricChange = (event, newMetric) => {
    if (newMetric !== null) {
      setMetricType(newMetric)
    }
  }

  return (
    <Card>
      <CardContent>
        <Box display="flex" justifyContent="space-between" alignItems="center" mb={2}>
          <Typography variant="h6" fontWeight={600}>
            Training History
          </Typography>
          <ToggleButtonGroup
            value={metricType}
            exclusive
            onChange={handleMetricChange}
            size="small"
          >
            <ToggleButton value="reward">Reward</ToggleButton>
            <ToggleButton value="episode">Episode</ToggleButton>
          </ToggleButtonGroup>
        </Box>

        <Box sx={{ width: '100%', height: 300 }}>
          <ResponsiveContainer>
            <LineChart data={chartData}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis
                dataKey="index"
                type="number"
                scale="linear"
                label={{ value: 'Data Point', position: 'insideBottom', offset: -5 }}
                tickFormatter={(value) => `#${value}`}
                domain={['dataMin', 'dataMax']}
              />
              <YAxis />
              <Tooltip
                formatter={(value) => value.toFixed(2)}
                labelFormatter={(label, payload) => {
                  if (payload && payload[0]) {
                    const timestep = payload[0].payload.timestep;
                    return `Point #${label} (Timestep: ${timestep.toLocaleString()})`;
                  }
                  return `Point #${label}`;
                }}
              />
              <Legend />
              {metricType === 'reward' ? (
                <Line
                  type="monotone"
                  dataKey="reward"
                  stroke="#667eea"
                  strokeWidth={2}
                  dot={false}
                  name="Reward"
                />
              ) : (
                <Line
                  type="monotone"
                  dataKey="episode"
                  stroke="#764ba2"
                  strokeWidth={2}
                  dot={false}
                  name="Episode"
                />
              )}
            </LineChart>
          </ResponsiveContainer>
        </Box>
      </CardContent>
    </Card>
  )
}

export default TrainingChart

