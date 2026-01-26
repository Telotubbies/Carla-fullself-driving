import {
  Card,
  CardContent,
  CardHeader,
  Typography,
  LinearProgress,
  Box,
} from '@mui/material';
import { Memory, Storage, Computer, Bolt, LocalFireDepartment } from '@mui/icons-material';
import { memo, useMemo } from 'react';

interface SystemResourcesCardProps {
  system: {
    cpu: { usage: number; percent: number; used_gb: number; total_gb: number; free_gb: number; temperature?: number | null };
    memory: { percent: number; used_gb: number; total_gb: number; free_gb: number };
    disk?: { used_percent: number; used_gb: number; total_gb: number; free_gb: number };
    gpu?: Array<{
      name: string;
      memory_used: number;
      memory_total: number;
      utilization: number;
      temperature: number;
    }>;
  };
  power?: {
    power_draw?: number;
    energy_used?: number;
    cost_estimate?: number;
  };
}

function SystemResourcesCard({ system, power }: SystemResourcesCardProps) {
  const getColor = (percent: number) => {
    if (percent < 50) return 'success';
    if (percent < 80) return 'warning';
    return 'error';
  };

  const diskColor = useMemo(
    () => (system.disk ? getColor(system.disk.used_percent) : 'success'),
    [system.disk]
  );
  const memColor = useMemo(() => getColor(system.memory.percent), [system.memory.percent]);
  const cpuColor = useMemo(() => getColor(system.cpu.percent), [system.cpu.percent]);

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader title="System Resources" sx={{ pb: 1 }} />
      <CardContent sx={{ flex: 1, pt: 1, overflow: 'auto' }}>
        {/* CPU */}
        <Box sx={{ mb: 2 }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
              <Computer fontSize="small" color="info" />
              <Typography variant="caption" fontWeight={600}>
                CPU
              </Typography>
            </Box>
            <Typography variant="caption" color="text.secondary">
              {system.cpu.percent.toFixed(1)}%
            </Typography>
          </Box>
          <LinearProgress
            variant="determinate"
            value={system.cpu.percent}
            color={cpuColor}
            sx={{ height: 8, borderRadius: 4, mb: 0.5 }}
          />
          {system.cpu.temperature !== null && 
           system.cpu.temperature !== undefined && 
           !isNaN(system.cpu.temperature) && (
            <Box sx={{ display: 'flex', justifyContent: 'flex-end', mt: 0.5 }}>
              <Typography variant="caption" color="text.secondary">
                {system.cpu.temperature.toFixed(1)}°C
              </Typography>
            </Box>
          )}
        </Box>

        {/* Memory */}
        <Box sx={{ mb: 2 }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
              <Memory fontSize="small" color="warning" />
              <Typography variant="caption" fontWeight={600}>
                Memory
              </Typography>
            </Box>
            <Typography variant="caption" color="text.secondary">
              {system.memory.percent.toFixed(1)}%
            </Typography>
          </Box>
          <LinearProgress
            variant="determinate"
            value={system.memory.percent}
            color={memColor}
            sx={{ height: 8, borderRadius: 4 }}
          />
          <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
            {system.memory.used_gb.toFixed(1)} / {system.memory.total_gb.toFixed(1)} GB
          </Typography>
        </Box>

        {/* Disk */}
        {system.disk && (
          <Box sx={{ mb: 2 }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
                <Storage fontSize="small" color="error" />
                <Typography variant="caption" fontWeight={600}>
                  Disk
                </Typography>
              </Box>
              <Typography variant="caption" color="text.secondary">
                {system.disk.used_percent.toFixed(1)}%
              </Typography>
            </Box>
            <LinearProgress
              variant="determinate"
              value={system.disk.used_percent}
              color={diskColor}
              sx={{ height: 8, borderRadius: 4 }}
            />
            <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
              {system.disk.used_gb.toFixed(1)} / {system.disk.total_gb.toFixed(1)} GB
            </Typography>
          </Box>
        )}

        {/* GPU */}
        {system.gpu && system.gpu.length > 0 && (
          <Box sx={{ mb: 2 }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
                <Computer fontSize="small" color="secondary" />
                <Typography variant="caption" fontWeight={600}>
                  GPU: {system.gpu[0].name}
                </Typography>
              </Box>
              <Typography variant="caption" color="text.secondary">
                {system.gpu[0].utilization.toFixed(1)}%
              </Typography>
            </Box>
            <LinearProgress
              variant="determinate"
              value={system.gpu[0].utilization}
              color={getColor(system.gpu[0].utilization) as 'success' | 'warning' | 'error'}
              sx={{ height: 8, borderRadius: 4, mb: 0.5 }}
            />
            <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
              <Typography variant="caption" color="text.secondary">
                {system.gpu[0].memory_used.toFixed(1)} / {system.gpu[0].memory_total.toFixed(1)} GB
                {system.gpu[0].memory_total > 0 && (
                  <span> ({(system.gpu[0].memory_used / system.gpu[0].memory_total * 100).toFixed(1)}%)</span>
                )}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                {system.gpu[0].temperature}°C
              </Typography>
            </Box>
          </Box>
        )}

        {/* Power & Cost - Compact */}
        {power && (
          <Box sx={{ mt: 2, pt: 2, borderTop: '1px solid', borderColor: 'divider' }}>
            <Box
              sx={{
                display: 'grid',
                gridTemplateColumns: 'repeat(3, 1fr)',
                gap: 1,
              }}
            >
              {power.power_draw !== undefined && (
                <Box sx={{ textAlign: 'center' }}>
                  <Bolt color="warning" fontSize="small" />
                  <Typography variant="body2" fontWeight={600}>
                    {power.power_draw.toFixed(0)}W
                  </Typography>
                </Box>
              )}
              {power.energy_used !== undefined && (
                <Box sx={{ textAlign: 'center' }}>
                  <LocalFireDepartment color="error" fontSize="small" />
                  <Typography variant="body2" fontWeight={600}>
                    {power.energy_used.toFixed(2)}kWh
                  </Typography>
                </Box>
              )}
              {power.cost_estimate !== undefined && (
                <Box sx={{ textAlign: 'center' }}>
                  <Typography variant="body2" fontWeight={600} color="success.main">
                    {power.cost_estimate.toFixed(2)}฿
                  </Typography>
                </Box>
              )}
            </Box>
          </Box>
        )}
      </CardContent>
    </Card>
  );
}

export default memo(SystemResourcesCard);
