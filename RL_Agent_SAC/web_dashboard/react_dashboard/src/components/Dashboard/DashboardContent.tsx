import { useQuery } from '@tanstack/react-query';
import { Box } from '@mui/material';
import { memo } from 'react';
import DashboardLayout from '../Layout/DashboardLayout';
import { api } from '../../services/api';
import StatusCard from './StatusCard';
import MetricsCard from './MetricsCard';
import SystemResourcesCard from './SystemResourcesCard';
import TrainingChart from './TrainingChart';
import CheckpointAndLogsCard from './CheckpointAndLogsCard';

function DashboardContent() {
  const { data: status, isLoading } = useQuery({
    queryKey: ['status'],
    queryFn: api.getStatus,
  });

  if (isLoading || !status) {
    return (
      <DashboardLayout isRunning={false}>
        <Box>Loading...</Box>
      </DashboardLayout>
    );
  }

  return (
    <DashboardLayout isRunning={status.status.running}>
      <Box
        sx={{
          display: 'grid',
          gridTemplateColumns: { xs: '1fr', md: 'repeat(12, 1fr)' },
          gap: 2,
          height: 'calc(100vh - 100px)',
          overflow: 'hidden',
        }}
      >
        {/* Top Row: Training Progress (Full Width) */}
        <Box sx={{ gridColumn: { xs: '1', md: '1 / 13' } }}>
          <StatusCard status={status} />
        </Box>

        {/* Second Row: Metrics, System Resources & Training Chart */}
        <Box sx={{ gridColumn: { xs: '1', md: '1 / 4' } }}>
          <MetricsCard metrics={status.metrics} />
        </Box>
        <Box sx={{ gridColumn: { xs: '1', md: '4 / 7' } }}>
          <SystemResourcesCard system={status.system} power={status.power} />
        </Box>
        <Box sx={{ gridColumn: { xs: '1', md: '7 / 13' } }}>
          <TrainingChart />
        </Box>

        {/* Bottom Row: Checkpoint & Logs (20% height, scrollable) */}
        <Box 
          sx={{ 
            gridColumn: { xs: '1', md: '1 / 13' },
            height: '20%',
            minHeight: '200px',
            maxHeight: '400px',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
          }}
        >
          <CheckpointAndLogsCard checkpoint={status.checkpoint} />
        </Box>
      </Box>
    </DashboardLayout>
  );
}

export default memo(DashboardContent);
