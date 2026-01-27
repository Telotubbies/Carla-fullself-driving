import { Card, CardContent, CardHeader, Typography, Box, CircularProgress, Chip, Tooltip } from '@mui/material';
import { useQuery } from '@tanstack/react-query';
import { memo } from 'react';
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

function EvaluationCard() {
  const { data, isLoading, error } = useQuery<EvaluationsResponse>({
    queryKey: ['evaluations'],
    queryFn: api.getEvaluations,
    refetchInterval: 60000,
    staleTime: 30000,
  });

  const latest = data?.latest ?? null;

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader
        title="Validation (Eval Episodes)"
        subheader="Periodic evaluation of SAC + LSTM policy"
        sx={{ pb: 1 }}
      />
      <CardContent sx={{ flex: 1, display: 'flex', flexDirection: 'column', pt: 1 }}>
        {isLoading && !data ? (
          <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <CircularProgress size={32} />
          </Box>
        ) : error ? (
          <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <Typography variant="body2" color="error">
              Failed to load validation metrics
            </Typography>
          </Box>
        ) : !latest ? (
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
                {latest.timestep.toLocaleString()}
              </Typography>
            </Box>

            <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 2 }}>
              <Box sx={{ textAlign: 'center', flex: 1 }}>
                <Typography variant="caption" color="text.secondary">
                  Mean Reward
                </Typography>
                <Typography variant="h6" fontWeight={700} color={latest.mean_reward >= 0 ? 'primary.main' : 'error.main'}>
                  {latest.mean_reward.toFixed(1)}
                </Typography>
              </Box>
              <Box sx={{ textAlign: 'center', flex: 1 }}>
                <Typography variant="caption" color="text.secondary">
                  Std Reward
                </Typography>
                <Typography variant="h6" fontWeight={700}>
                  {(latest.std_reward ?? 0).toFixed(1)}
                </Typography>
              </Box>
            </Box>

            {typeof latest.mean_length === 'number' && (
              <Box sx={{ textAlign: 'center', mb: 2 }}>
                <Typography variant="caption" color="text.secondary">
                  Mean Episode Length
                </Typography>
                <Typography variant="body1" fontWeight={600}>
                  {latest.mean_length.toFixed(0)} steps
                </Typography>
              </Box>
            )}

            {latest.timestamp && (
              <Typography variant="caption" color="text.secondary" sx={{ mt: 'auto', textAlign: 'center' }}>
                Last updated: {latest.timestamp}
              </Typography>
            )}

            {data?.history && data.history.length > 1 && (
              <Box sx={{ mt: 1, display: 'flex', flexWrap: 'wrap', gap: 0.5, justifyContent: 'center' }}>
                {data.history.slice(0, 3).map((h) => (
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
        )}
      </CardContent>
    </Card>
  );
}

export default memo(EvaluationCard);



