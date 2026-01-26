import { Card, CardContent, CardHeader, Box, CircularProgress } from '@mui/material';
import { useQuery } from '@tanstack/react-query';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { memo } from 'react';
import { api } from '../../services/api';

function TrainingChart() {
  const { data: history, isLoading } = useQuery({
    queryKey: ['metrics-history'],
    queryFn: api.getMetricsHistory,
    refetchInterval: 2000, // Refresh every 2 seconds for real-time chart updates
    staleTime: 1000,
  });

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader title="Training History (Reward)" sx={{ pb: 1 }} />
      <CardContent sx={{ flex: 1, display: 'flex', flexDirection: 'column', pt: 1 }}>
        {isLoading ? (
          <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', flex: 1 }}>
            <CircularProgress size={40} />
          </Box>
        ) : !history || history.length === 0 ? (
          <Box sx={{ textAlign: 'center', py: 4, color: 'text.secondary', flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            No training data available yet
          </Box>
        ) : (
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={history}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
              <XAxis
                dataKey="step"
                domain={[0, 'dataMax']}
                type="number"
                scale="linear"
                stroke="#666"
                tick={{ fontSize: 11 }}
                label={{ value: 'Steps', position: 'insideBottom', offset: -5, style: { fontSize: 11 } }}
              />
              <YAxis
                stroke="#666"
                tick={{ fontSize: 11 }}
                label={{ value: 'Reward', angle: -90, position: 'insideLeft', style: { fontSize: 11 } }}
              />
              <Tooltip
                contentStyle={{
                  backgroundColor: 'rgba(255, 255, 255, 0.95)',
                  border: '1px solid #ccc',
                  borderRadius: 8,
                  fontSize: 12,
                }}
              />
              <Legend wrapperStyle={{ fontSize: 11 }} />
              <Line
                type="monotone"
                dataKey="reward"
                stroke="#667eea"
                strokeWidth={2}
                dot={false}
                name="Episode Reward"
              />
            </LineChart>
          </ResponsiveContainer>
        )}
      </CardContent>
    </Card>
  );
}

export default memo(TrainingChart);
