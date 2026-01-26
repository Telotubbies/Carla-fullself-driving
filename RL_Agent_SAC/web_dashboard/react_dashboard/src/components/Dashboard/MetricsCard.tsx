import { Card, CardContent, CardHeader, Typography, Box, Divider } from '@mui/material';
import { Speed, TrendingUp, EmojiEvents, Timeline } from '@mui/icons-material';
import { memo, useMemo } from 'react';
import { useQuery } from '@tanstack/react-query';
import { api } from '../../services/api';

interface MetricsCardProps {
  metrics: {
    current_step: number;
    last_reward?: number;
    episode_reward?: number;
    episode_length?: number;
  };
}

function MetricsCard({ metrics }: MetricsCardProps) {
  const { data: history } = useQuery({
    queryKey: ['metrics-history'],
    queryFn: api.getMetricsHistory,
    refetchInterval: 60000, // Auto-refresh every 1 minute
    staleTime: 30000, // Consider data stale after 30 seconds
  });

  const stats = useMemo(() => {
    if (!history || history.length === 0) {
      return {
        totalEpisodes: 0,
        bestReward: 0,
        worstReward: 0,
        avgReward: 0,
        recentAvgReward: 0,
        avgEpisodeLength: 0,
      };
    }

    const rewards = history.map((h) => h.reward).filter((r) => r != null);
    const recentRewards = rewards.slice(-10); // Last 10 episodes

    return {
      totalEpisodes: history.length,
      bestReward: Math.max(...rewards),
      worstReward: Math.min(...rewards),
      avgReward: rewards.reduce((a, b) => a + b, 0) / rewards.length,
      recentAvgReward: recentRewards.length > 0 
        ? recentRewards.reduce((a, b) => a + b, 0) / recentRewards.length 
        : 0,
      avgEpisodeLength: metrics.episode_length || 0,
    };
  }, [history, metrics.episode_length]);

  const formatNumber = (num: number, decimals: number = 2) => {
    if (num === 0 || !isFinite(num)) return 'N/A';
    return num.toFixed(decimals);
  };

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <CardHeader title="Training Statistics" sx={{ pb: 1 }} />
      <CardContent sx={{ flex: 1, pt: 1, display: 'flex', flexDirection: 'column', gap: 1.5 }}>
        {/* Episode Count */}
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', p: 1.5, bgcolor: 'background.default', borderRadius: 1.5 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <Timeline color="info" fontSize="small" />
            <Typography variant="body2" color="text.secondary">
              Total Episodes
            </Typography>
          </Box>
          <Typography variant="h6" fontWeight={700} color="info.main">
            {stats.totalEpisodes}
          </Typography>
        </Box>

        <Divider />

        {/* Best Reward */}
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', p: 1.5, bgcolor: 'background.default', borderRadius: 1.5 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <EmojiEvents color="success" fontSize="small" />
            <Typography variant="body2" color="text.secondary">
              Best Reward
            </Typography>
          </Box>
          <Typography variant="h6" fontWeight={700} color="success.main">
            {formatNumber(stats.bestReward)}
          </Typography>
        </Box>

        {/* Average Reward */}
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', p: 1.5, bgcolor: 'background.default', borderRadius: 1.5 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <TrendingUp color="primary" fontSize="small" />
            <Typography variant="body2" color="text.secondary">
              Avg Reward
            </Typography>
          </Box>
          <Typography variant="body2" fontWeight={600}>
            {formatNumber(stats.avgReward)}
          </Typography>
        </Box>

        {/* Recent Average (Last 10) */}
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', p: 1.5, bgcolor: 'background.default', borderRadius: 1.5 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <Speed color="warning" fontSize="small" />
            <Typography variant="body2" color="text.secondary">
              Recent Avg (10)
            </Typography>
          </Box>
          <Typography variant="body2" fontWeight={600} color="warning.main">
            {formatNumber(stats.recentAvgReward)}
          </Typography>
        </Box>

        {/* Episode Length */}
        {stats.avgEpisodeLength > 0 && (
          <>
            <Divider />
            <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', p: 1.5, bgcolor: 'background.default', borderRadius: 1.5 }}>
              <Typography variant="body2" color="text.secondary">
                Avg Episode Length
              </Typography>
              <Typography variant="body2" fontWeight={600}>
                {stats.avgEpisodeLength} steps
              </Typography>
            </Box>
          </>
        )}
      </CardContent>
    </Card>
  );
}

export default memo(MetricsCard);
