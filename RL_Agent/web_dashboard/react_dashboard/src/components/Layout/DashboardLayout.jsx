import React from 'react'
import {
  Box,
  AppBar,
  Toolbar,
  Typography,
  Chip,
} from '@mui/material'

function DashboardLayout({ children }) {

  return (
    <Box>
      <AppBar
        position="fixed"
        sx={{
          width: '100%',
          bgcolor: 'background.paper',
          color: 'text.primary',
          boxShadow: '0 1px 3px rgba(0,0,0,0.1)',
        }}
      >
        <Toolbar>
          <Typography variant="h6" noWrap component="div" sx={{ flexGrow: 1, fontWeight: 600 }}>
            🚗 CARLA RL Training Dashboard
          </Typography>
          <Chip
            label="Live"
            color="success"
            size="small"
            sx={{ mr: 2 }}
          />
        </Toolbar>
      </AppBar>
      
      <Box
        component="main"
        sx={{
          width: '100%',
          p: 3,
          mt: 8,
        }}
      >
        {children}
      </Box>
    </Box>
  )
}

export default DashboardLayout

