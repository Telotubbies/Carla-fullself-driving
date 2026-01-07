import React, { useState } from 'react'
import {
  Card,
  CardContent,
  Typography,
  Box,
  TextField,
  IconButton,
  Paper,
} from '@mui/material'
import {
  Refresh as RefreshIcon,
  Clear as ClearIcon,
} from '@mui/icons-material'
import { useQuery } from '@tanstack/react-query'
import { dashboardAPI } from '../../services/api'

function LogsViewer() {
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
            System Logs
          </Typography>
          <Box>
            <IconButton size="small" onClick={() => refetch()}>
              <RefreshIcon fontSize="small" />
            </IconButton>
          </Box>
        </Box>

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
      </CardContent>
    </Card>
  )
}

export default LogsViewer

