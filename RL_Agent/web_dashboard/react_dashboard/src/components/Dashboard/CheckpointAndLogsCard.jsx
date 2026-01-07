import React, { useState } from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  Chip,
  IconButton,
  Paper,
  Tabs,
  Tab,
} from '@mui/material'
import {
  Save as SaveIcon,
  CalendarToday as CalendarIcon,
  Refresh as RefreshIcon,
  Description as LogsIcon,
} from '@mui/icons-material'
import { useQuery } from '@tanstack/react-query'
import { dashboardAPI } from '../../services/api'

function CheckpointAndLogsCard({ checkpoint }) {
  const [tabValue, setTabValue] = useState(0)
  const [autoRefresh, setAutoRefresh] = useState(true)

  const { data: logsData, refetch } = useQuery({
    queryKey: ['autoManageLogs'],
    queryFn: () => dashboardAPI.getAutoManageLogs().then(res => res.data),
    refetchInterval: autoRefresh ? 3000 : false,
    staleTime: 1000,
  })

  const logs = logsData?.lines || []
  const lastUpdate = logsData?.last_update

  return (
    <Card>
      <CardContent>
        <Box display="flex" justifyContent="space-between" alignItems="center" mb={2}>
          <Typography variant="h6" fontWeight={600}>
            Checkpoint & System Logs
          </Typography>
          {tabValue === 1 && (
            <IconButton size="small" onClick={() => refetch()}>
              <RefreshIcon fontSize="small" />
            </IconButton>
          )}
        </Box>

        <Tabs
          value={tabValue}
          onChange={(e, newValue) => setTabValue(newValue)}
          sx={{ mb: 2, borderBottom: 1, borderColor: 'divider' }}
        >
          <Tab
            icon={<SaveIcon fontSize="small" />}
            iconPosition="start"
            label="Latest Checkpoint"
          />
          <Tab
            icon={<LogsIcon fontSize="small" />}
            iconPosition="start"
            label="System Logs"
          />
        </Tabs>

        {tabValue === 0 && (
          <Box>
            {!checkpoint ? (
              <Box textAlign="center" py={3}>
                <Typography variant="body2" color="text.secondary">
                  No checkpoint available
                </Typography>
              </Box>
            ) : (
              <Box>
                <Box display="flex" alignItems="center" mb={2}>
                  <SaveIcon sx={{ mr: 1, color: 'primary.main' }} />
                  <Typography variant="subtitle1" fontWeight={600}>
                    Latest Checkpoint
                  </Typography>
                </Box>

                <Box mb={2}>
                  <Chip
                    label={`${checkpoint.timestep?.toLocaleString() || 0} steps`}
                    color="primary"
                    size="medium"
                    sx={{ mb: 1 }}
                  />
                </Box>

                {checkpoint.created_at && (
                  <Box display="flex" alignItems="center" color="text.secondary" mb={1}>
                    <CalendarIcon fontSize="small" sx={{ mr: 0.5 }} />
                    <Typography variant="body2">
                      {checkpoint.created_at}
                    </Typography>
                  </Box>
                )}

                {checkpoint.episode && (
                  <Box mt={1} mb={1}>
                    <Typography variant="body2" color="text.secondary">
                      Episode: {checkpoint.episode}
                    </Typography>
                  </Box>
                )}

                {checkpoint.reward !== null && checkpoint.reward !== undefined && (
                  <Box mt={1}>
                    <Typography variant="body2" color="text.secondary">
                      Reward: {checkpoint.reward.toFixed(2)}
                    </Typography>
                  </Box>
                )}
              </Box>
            )}
          </Box>
        )}

        {tabValue === 1 && (
          <Box>
            <Paper
              variant="outlined"
              sx={{
                p: 1,
                bgcolor: '#1e1e1e',
                color: '#d4d4d4',
                fontFamily: 'monospace',
                fontSize: '0.75rem',
                maxHeight: 400,
                overflow: 'auto',
                '&::-webkit-scrollbar': {
                  width: 8,
                },
                '&::-webkit-scrollbar-track': {
                  bgcolor: '#2d2d2d',
                },
                '&::-webkit-scrollbar-thumb': {
                  bgcolor: '#555',
                  borderRadius: 4,
                },
              }}
            >
              {logs.length === 0 ? (
                <Typography variant="body2" color="text.secondary" sx={{ p: 2 }}>
                  No logs available
                </Typography>
              ) : (
                logs.slice(-50).map((line, idx) => (
                  <Box
                    key={idx}
                    sx={{
                      py: 0.5,
                      px: 1,
                      borderLeft: '3px solid transparent',
                      '&:hover': {
                        bgcolor: 'rgba(255,255,255,0.05)',
                      },
                    }}
                  >
                    <Typography
                      component="pre"
                      sx={{
                        margin: 0,
                        fontFamily: 'monospace',
                        fontSize: '0.75rem',
                        whiteSpace: 'pre-wrap',
                        wordBreak: 'break-word',
                      }}
                    >
                      {line}
                    </Typography>
                  </Box>
                ))
              )}
            </Paper>

            {lastUpdate && (
              <Typography variant="caption" color="text.secondary" sx={{ mt: 1, display: 'block' }}>
                Last update: {lastUpdate}
              </Typography>
            )}
          </Box>
        )}
      </CardContent>
    </Card>
  )
}

export default CheckpointAndLogsCard
