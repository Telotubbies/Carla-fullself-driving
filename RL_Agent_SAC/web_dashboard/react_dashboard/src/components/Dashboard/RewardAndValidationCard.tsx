import { Card, CardContent, CardHeader, Box, CircularProgress, ToggleButton, ToggleButtonGroup, Typography, Chip, Tooltip } from '@mui/material';
import { useQuery } from '@tanstack/react-query';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip as RechartsTooltip, Legend, ResponsiveContainer } from 'recharts';
import { memo, useState, useMemo } from 'react';
import { api } from '../../services/api';

interface EvaluationReport {
  timestep: number;
  mean_reward: number;
  std_reward?: number;
  mean_length?: number;
  timestamp?: string;
}

interface EvaluationsResponse {
  latest: EvaluationReport | null;
  history: EvaluationReport[];
}

function RewardAndValidationCard() {
  const [viewMode, setViewMode] = useState<'reward' | 'validation'>('reward');

  const { data: status } = useQuery({
    queryKey: ['status'],
    queryFn: api.getStatus,
    refetchInterval: 5000,
    staleTime: 2000,
  });

  const { data: history, isLoading: historyLoading } = useQuery({
    queryKey: ['metrics-history'],
    queryFn: api.getMetricsHistory,
    refetchInterval: 2000,
    staleTime: 1000,
  });

  const { data: evaluations, isLoading: evalLoading, error: evalError } = useQuery<EvaluationsResponse>({
    queryKey: ['evaluations'],
    queryFn: api.getEvaluations,
    refetchInterval: 60000,
    staleTime: 30000,
  });

  const handleViewChange = (_event: React.MouseEvent<HTMLElement>, newView: 'reward' | 'validation' | null) => {
    if (newView !== null) {
      setViewMode(newView);
    }
  };

  const latestEval = evaluations?.latest ?? null;

  // Process history data to ensure it starts from step 0 and is easy to read
  const processedHistory = useMemo(() => {
    if (!history || history.length === 0) return [];
    
    // Sort by step
    const sorted = [...history].sort((a, b) => a.step - b.step);
    
    // If first step is not 0, add a placeholder at step 0
    if (sorted[0]?.step > 0) {
      return [{ step: 0, reward: sorted[0].reward, timestamp: null }, ...sorted];
    }
    
    return sorted;
  }, [history]);

  // Format step numbers for X-axis (e.g., 1000 -> 1k, 10000 -> 10k)
  const formatStep = (value: number) => {
    if (value >= 10000) {
      return `${(value / 1000).toFixed(0)}k`;
    } else if (value >= 1000) {
      return `${(value / 1000).toFixed(1)}k`;
    }
    return value.toString();
  };

  // Calculate max step for domain - use current step from status if available
  const maxStep = useMemo(() => {
    const historyMax = processedHistory && processedHistory.length > 0 
      ? Math.max(...processedHistory.map(h => h.step))
      : 0;
    const statusStep = status?.progress?.current || 0;
    // Use the maximum of history and current status step
    return Math.max(historyMax, statusStep, 1000);
  }, [processedHistory, status]);

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader
        title={
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', width: '100%', gap: 2 }}>
            <Typography variant="h6" sx={{ flex: 1, minWidth: 0 }}>
              {viewMode === 'reward' ? 'Training History (Reward)' : 'Validation (Eval Episodes)'}
            </Typography>
            <ToggleButtonGroup
              value={viewMode}
              exclusive
              onChange={handleViewChange}
              size="medium"
              sx={{ 
                height: '40px',
                '& .MuiToggleButton-root': {
                  px: 3,
                  py: 1,
                  fontSize: '0.875rem',
                  fontWeight: 600,
                  textTransform: 'none',
                  border: '1px solid',
                  borderColor: 'divider',
                  '&:hover': {
                    backgroundColor: 'action.hover',
                  },
                  '&.Mui-selected': {
                    backgroundColor: 'primary.main',
                    color: 'white',
                    borderColor: 'primary.main',
                    '&:hover': {
                      backgroundColor: 'primary.dark',
                    },
                  },
                },
              }}
            >
              <ToggleButton value="reward" aria-label="reward history">
                📈 Reward
              </ToggleButton>
              <ToggleButton value="validation" aria-label="validation">
                ✅ Validation
              </ToggleButton>
            </ToggleButtonGroup>
          </Box>
        }
        sx={{ pb: 1 }}
      />
      <CardContent sx={{ flex: 1, display: 'flex', flexDirection: 'column', pt: 1 }}>
        {viewMode === 'reward' ? (
          // Reward History Chart View
          historyLoading ? (
            <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', flex: 1 }}>
              <CircularProgress size={40} />
            </Box>
          ) : !history || history.length === 0 ? (
            <Box sx={{ textAlign: 'center', py: 4, color: 'text.secondary', flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              No training data available yet
            </Box>
          ) : (
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={processedHistory} margin={{ top: 5, right: 10, left: 5, bottom: 20 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" opacity={0.5} />
                <XAxis
                  dataKey="step"
                  domain={[0, maxStep]}
                  type="number"
                  scale="linear"
                  stroke="#666"
                  tick={{ fontSize: 12, fill: '#666' }}
                  tickFormatter={formatStep}
                  ticks={(() => {
                    // Generate nice tick values: 0, maxStep/4, maxStep/2, 3*maxStep/4, maxStep
                    const ticks = [0];
                    if (maxStep > 0) {
                      for (let i = 1; i < 5; i++) {
                        ticks.push(Math.round((maxStep * i) / 4));
                      }
                    }
                    return ticks;
                  })()}
                  label={{ 
                    value: 'Training Steps', 
                    position: 'insideBottom', 
                    offset: -10, 
                    style: { fontSize: 12, fontWeight: 600, fill: '#333' } 
                  }}
                />
                <YAxis
                  stroke="#666"
                  tick={{ fontSize: 12, fill: '#666' }}
                  tickFormatter={(value) => value.toFixed(0)}
                  label={{ 
                    value: 'Episode Reward', 
                    angle: -90, 
                    position: 'insideLeft', 
                    style: { fontSize: 12, fontWeight: 600, fill: '#333' } 
                  }}
                />
                <RechartsTooltip
                  contentStyle={{
                    backgroundColor: 'rgba(255, 255, 255, 0.98)',
                    border: '1px solid #ccc',
                    borderRadius: 8,
                    fontSize: 12,
                    padding: '8px 12px',
                    boxShadow: '0 2px 8px rgba(0,0,0,0.15)',
                  }}
                  formatter={(value: number) => [value.toFixed(2), 'Reward']}
                  labelFormatter={(label) => `Step: ${label.toLocaleString()}`}
                />
                <Legend 
                  wrapperStyle={{ fontSize: 12, paddingTop: '10px' }} 
                  iconType="line"
                />
                <Line
                  type="monotone"
                  dataKey="reward"
                  stroke="#667eea"
                  strokeWidth={2.5}
                  dot={false}
                  activeDot={{ r: 4, fill: '#667eea' }}
                  name="Episode Reward"
                  isAnimationActive={false}
                />
              </LineChart>
            </ResponsiveContainer>
          )
        ) : (
          // Validation View
          evalLoading && !evaluations ? (
            <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              <CircularProgress size={32} />
            </Box>
          ) : evalError ? (
            <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              <Typography variant="body2" color="error">
                Failed to load validation metrics
              </Typography>
            </Box>
          ) : !latestEval ? (
            <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', textAlign: 'center' }}>
              <Typography variant="body2" color="text.secondary">
                No evaluation runs yet.
                <br />
                Validation will appear here once the first EvalCallback is executed.
              </Typography>
            </Box>
          ) : (
            <>
              <Box sx={{ textAlign: 'center', mb: 2 }}>
                <Typography variant="subtitle2" color="text.secondary">
                  Latest Evaluation @ Step
                </Typography>
                <Typography variant="h5" color="primary" fontWeight={700}>
                  {latestEval.timestep.toLocaleString()}
                </Typography>
              </Box>

              <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 2 }}>
                <Box sx={{ textAlign: 'center', flex: 1 }}>
                  <Typography variant="caption" color="text.secondary">
                    Mean Reward
                  </Typography>
                  <Typography variant="h6" fontWeight={700} color={latestEval.mean_reward >= 0 ? 'primary.main' : 'error.main'}>
                    {latestEval.mean_reward.toFixed(1)}
                  </Typography>
                </Box>
                <Box sx={{ textAlign: 'center', flex: 1 }}>
                  <Typography variant="caption" color="text.secondary">
                    Std Reward
                  </Typography>
                  <Typography variant="h6" fontWeight={700}>
                    {(latestEval.std_reward ?? 0).toFixed(1)}
                  </Typography>
                </Box>
              </Box>

              {typeof latestEval.mean_length === 'number' && (
                <Box sx={{ textAlign: 'center', mb: 2 }}>
                  <Typography variant="caption" color="text.secondary">
                    Mean Episode Length
                  </Typography>
                  <Typography variant="body1" fontWeight={600}>
                    {latestEval.mean_length.toFixed(0)} steps
                  </Typography>
                </Box>
              )}

              {latestEval.timestamp && (
                <Typography variant="caption" color="text.secondary" sx={{ mt: 'auto', textAlign: 'center' }}>
                  Last updated: {latestEval.timestamp}
                </Typography>
              )}

              {evaluations?.history && evaluations.history.length > 1 && (
                <Box sx={{ mt: 1, display: 'flex', flexWrap: 'wrap', gap: 0.5, justifyContent: 'center' }}>
                  {evaluations.history.slice(0, 3).map((h) => (
                    <Tooltip
                      key={h.timestep}
                      title={`Reward: ${h.mean_reward.toFixed(1)} ± ${(h.std_reward ?? 0).toFixed(1)} | Len: ${
                        typeof h.mean_length === 'number' ? h.mean_length.toFixed(0) : 'N/A'
                      }`}
                    >
                      <Chip
                        size="small"
                        label={`@${h.timestep.toLocaleString()}`}
                        color={h.mean_reward >= 0 ? 'primary' : 'default'}
                        sx={{ fontSize: '0.7rem' }}
                      />
                    </Tooltip>
                  ))}
                </Box>
              )}
            </>
          )
        )}
      </CardContent>
    </Card>
  );
}

export default memo(RewardAndValidationCard);

