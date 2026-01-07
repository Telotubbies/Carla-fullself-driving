import React from 'react'
import { Box } from '@mui/material'
import DashboardLayout from './components/Layout/DashboardLayout'
import DashboardContent from './components/Dashboard/DashboardContent'

function App() {
  return (
    <Box sx={{ minHeight: '100vh', bgcolor: 'background.default' }}>
      <DashboardLayout>
        <DashboardContent />
      </DashboardLayout>
    </Box>
  )
}

export default App

