import { Card, CardContent, CardHeader, Typography, Box, Tabs, Tab, Paper, IconButton, Tooltip, Chip, CircularProgress, Alert } from '@mui/material';
import { useState, useRef, useEffect, memo, useCallback } from 'react';
import { useQuery } from '@tanstack/react-query';
import { Refresh, VerticalAlignBottom, VerticalAlignTop, Error as ErrorIcon } from '@mui/icons-material';
import { api } from '../../services/api';

interface CheckpointAndLogsCardProps {
  checkpoint: { timestep: number; path?: string } | null;
}

function CheckpointAndLogsCard({ checkpoint }: CheckpointAndLogsCardProps) {
  const [tabValue, setTabValue] = useState(0);
  const [autoScroll, setAutoScroll] = useState(true);
  const logEndRef = useRef<HTMLDivElement>(null);
  const sacLogEndRef = useRef<HTMLDivElement>(null);
  const logContainerRef = useRef<HTMLDivElement>(null);
  const sacLogContainerRef = useRef<HTMLDivElement>(null);
  const [scrollState, setScrollState] = useState({ system: true, sac: true });

  const { data: systemLogs, isLoading: systemLogsLoading, error: systemLogsError, refetch: refetchSystemLogs } = useQuery({
    queryKey: ['logs'],
    queryFn: api.getLogs,
    refetchInterval: 2000,
    staleTime: 1000,
    retry: 3,
    retryDelay: 1000,
  });

  const { data: sacLogs, isLoading: sacLogsLoading, error: sacLogsError, refetch: refetchSACLogs } = useQuery({
    queryKey: ['sac-logs'],
    queryFn: api.getSACTrainingLogs,
    refetchInterval: 2000,
    staleTime: 1000,
    retry: 3,
    retryDelay: 1000,
  });

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    if (tabValue === 1 && autoScroll && logEndRef.current && logContainerRef.current) {
      const container = logContainerRef.current;
      const isNearBottom = container.scrollHeight - container.scrollTop <= container.clientHeight + 100;
      if (isNearBottom) {
        logEndRef.current.scrollIntoView({ behavior: 'smooth' });
      }
    }
  }, [systemLogs, tabValue, autoScroll]);

  useEffect(() => {
    if (tabValue === 2 && autoScroll && sacLogEndRef.current && sacLogContainerRef.current) {
      const container = sacLogContainerRef.current;
      const isNearBottom = container.scrollHeight - container.scrollTop <= container.clientHeight + 100;
      if (isNearBottom) {
        sacLogEndRef.current.scrollIntoView({ behavior: 'smooth' });
      }
    }
  }, [sacLogs, tabValue, autoScroll]);

  // Disable auto-scroll when user scrolls up
  const handleScroll = useCallback((containerRef: React.RefObject<HTMLDivElement>, logType: 'system' | 'sac') => {
    if (!containerRef.current) return;
    const container = containerRef.current;
    const isAtBottom = container.scrollHeight - container.scrollTop <= container.clientHeight + 50;
    if (!isAtBottom && autoScroll) {
      setAutoScroll(false);
      setScrollState(prev => ({ ...prev, [logType]: false }));
    } else if (isAtBottom && !autoScroll) {
      setAutoScroll(true);
      setScrollState(prev => ({ ...prev, [logType]: true }));
    }
  }, [autoScroll]);

  const formatLogLine = useCallback((line: string) => {
    if (!line || line.trim() === '') {
      return { text: ' ', color: '#10b981' };
    }
    const upperLine = line.toUpperCase();
    if (upperLine.includes('ERROR') || upperLine.includes('EXCEPTION') || upperLine.includes('TRACEBACK')) {
      return { text: line, color: '#ef4444' };
    }
    if (upperLine.includes('WARNING') || upperLine.includes('WARN')) {
      return { text: line, color: '#f59e0b' };
    }
    if (upperLine.includes('INFO')) {
      return { text: line, color: '#3b82f6' };
    }
    if (upperLine.includes('DEBUG')) {
      return { text: line, color: '#8b5cf6' };
    }
    return { text: line, color: '#10b981' };
  }, []);

  const scrollToTop = useCallback((containerRef: React.RefObject<HTMLDivElement>) => {
    if (containerRef.current) {
      containerRef.current.scrollTo({ top: 0, behavior: 'smooth' });
    }
  }, []);

  const scrollToBottom = useCallback((containerRef: React.RefObject<HTMLDivElement>, logType: 'system' | 'sac') => {
    if (containerRef.current) {
      containerRef.current.scrollTo({ top: containerRef.current.scrollHeight, behavior: 'smooth' });
      setAutoScroll(true);
      setScrollState(prev => ({ ...prev, [logType]: true }));
    }
  }, []);

  const logLines = systemLogs?.content?.split('\n').filter(line => line.trim() !== '') || [];
  const sacLogLines = sacLogs?.content?.split('\n').filter(line => line.trim() !== '') || [];

  return (
    <Card sx={{ height: '100%', display: 'flex', flexDirection: 'column', minHeight: 0 }}>
      <CardHeader 
        title="Checkpoint & Logs"
        sx={{ pb: 1, flexShrink: 0 }}
        action={
          <Box sx={{ display: 'flex', gap: 0.5, alignItems: 'center' }}>
            {tabValue === 1 && (
              <>
                <Tooltip title="Refresh logs">
                  <IconButton 
                    size="small" 
                    onClick={() => refetchSystemLogs()}
                    disabled={systemLogsLoading}
                  >
                    <Refresh fontSize="small" sx={{ animation: systemLogsLoading ? 'spin 1s linear infinite' : 'none' }} />
                  </IconButton>
                </Tooltip>
                {!scrollState.system && (
                  <Chip 
                    label="Auto-scroll off" 
                    size="small" 
                    color="warning"
                    sx={{ height: 24, fontSize: '0.7rem' }}
                  />
                )}
              </>
            )}
            {tabValue === 2 && (
              <>
                <Tooltip title="Refresh logs">
                  <IconButton 
                    size="small" 
                    onClick={() => refetchSACLogs()}
                    disabled={sacLogsLoading}
                  >
                    <Refresh fontSize="small" sx={{ animation: sacLogsLoading ? 'spin 1s linear infinite' : 'none' }} />
                  </IconButton>
                </Tooltip>
                {!scrollState.sac && (
                  <Chip 
                    label="Auto-scroll off" 
                    size="small" 
                    color="warning"
                    sx={{ height: 24, fontSize: '0.7rem' }}
                  />
                )}
              </>
            )}
          </Box>
        }
      />
      <CardContent sx={{ flex: 1, display: 'flex', flexDirection: 'column', pt: 1, overflow: 'hidden', minHeight: 0 }}>
        <Tabs 
          value={tabValue} 
          onChange={(_, v) => {
            setTabValue(v);
            setAutoScroll(true);
            setScrollState({ system: true, sac: true });
          }} 
          sx={{ mb: 1, minHeight: 40, borderBottom: 1, borderColor: 'divider' }}
        >
          <Tab 
            label="Checkpoint" 
            sx={{ minHeight: 40, fontSize: '0.85rem', textTransform: 'none' }} 
          />
          <Tab 
            label="System Log" 
            sx={{ minHeight: 40, fontSize: '0.85rem', textTransform: 'none' }} 
          />
          <Tab 
            label="SAC Training Log" 
            sx={{ minHeight: 40, fontSize: '0.85rem', textTransform: 'none' }} 
          />
        </Tabs>

        {tabValue === 0 && (
          <Box 
            sx={{ 
              flex: 1, 
              display: 'flex', 
              flexDirection: 'column',
              justifyContent: 'center',
              alignItems: 'center',
              gap: 2,
              p: 3,
              minHeight: 0,
            }}
          >
            {checkpoint ? (
              <Box sx={{ textAlign: 'center', width: '100%' }}>
                <Typography variant="h5" color="primary" fontWeight={700} gutterBottom>
                  Latest Checkpoint
                </Typography>
                <Box 
                  sx={{ 
                    bgcolor: 'background.default',
                    borderRadius: 2,
                    p: 3,
                    mt: 2,
                    border: '2px solid',
                    borderColor: 'primary.main'
                  }}
                >
                  <Typography variant="h4" color="primary" fontWeight={700} gutterBottom>
                    Step: {checkpoint.timestep?.toLocaleString() || '0'}
                  </Typography>
                  {checkpoint.path && (
                    <Typography 
                      variant="body2" 
                      color="text.secondary" 
                      sx={{ mt: 1, wordBreak: 'break-all' }}
                    >
                      {checkpoint.path}
                    </Typography>
                  )}
                </Box>
              </Box>
            ) : (
              <Typography variant="body1" color="text.secondary">
                No checkpoint available yet
              </Typography>
            )}
          </Box>
        )}

        {tabValue === 1 && (
          <Box sx={{ flex: 1, display: 'flex', flexDirection: 'column', overflow: 'hidden', position: 'relative', minHeight: 0, maxHeight: '100%' }}>
            {systemLogsError && (
              <Alert severity="error" icon={<ErrorIcon />} sx={{ mb: 1 }}>
                Failed to load system logs: {systemLogsError instanceof Error ? systemLogsError.message : 'Unknown error'}
              </Alert>
            )}
            <Box sx={{ mb: 1, display: 'flex', justifyContent: 'space-between', alignItems: 'center', flexShrink: 0 }}>
              <Typography variant="caption" color="text.secondary">
                {systemLogs?.filename ? `File: ${systemLogs.filename}` : 'Loading...'}
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
                {systemLogsLoading && <CircularProgress size={16} />}
                <Typography variant="caption" color="text.secondary">
                  {logLines.length} lines
                </Typography>
                <Tooltip title="Scroll to top">
                  <IconButton 
                    size="small" 
                    onClick={() => scrollToTop(logContainerRef)}
                    sx={{ width: 24, height: 24 }}
                  >
                    <VerticalAlignTop fontSize="small" />
                  </IconButton>
                </Tooltip>
                <Tooltip title="Scroll to bottom">
                  <IconButton 
                    size="small" 
                    onClick={() => scrollToBottom(logContainerRef, 'system')}
                    sx={{ width: 24, height: 24 }}
                  >
                    <VerticalAlignBottom fontSize="small" />
                  </IconButton>
                </Tooltip>
              </Box>
            </Box>
            <Paper
              ref={logContainerRef}
              onScroll={() => handleScroll(logContainerRef, 'system')}
              sx={{
                bgcolor: '#1e1e1e',
                color: '#10b981',
                p: 2,
                flex: 1,
                minHeight: 0,
                maxHeight: '100%',
                overflowY: 'auto',
                overflowX: 'hidden',
                fontFamily: 'monospace',
                fontSize: '0.75rem',
                lineHeight: 1.6,
                position: 'relative',
                '&::-webkit-scrollbar': {
                  width: '12px',
                },
                '&::-webkit-scrollbar-track': {
                  background: '#2d2d2d',
                  borderRadius: '6px',
                },
                '&::-webkit-scrollbar-thumb': {
                  background: '#555',
                  borderRadius: '6px',
                  '&:hover': {
                    background: '#666',
                  },
                },
              }}
            >
              {systemLogsLoading && logLines.length === 0 ? (
                <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
                  <CircularProgress size={30} />
                </Box>
              ) : logLines.length === 0 ? (
                <Typography variant="body2" color="text.secondary" sx={{ textAlign: 'center', py: 4 }}>
                  No logs available
                </Typography>
              ) : (
                <>
                  {logLines.map((line, idx) => {
                    const formatted = formatLogLine(line);
                    return (
                      <div 
                        key={`log-${idx}`}
                        style={{ 
                          color: formatted.color,
                          marginBottom: '1px',
                          whiteSpace: 'pre-wrap',
                          wordBreak: 'break-word'
                        }}
                      >
                        {formatted.text}
                      </div>
                    );
                  })}
                  <div ref={logEndRef} />
                </>
              )}
            </Paper>
          </Box>
        )}

        {tabValue === 2 && (
          <Box sx={{ flex: 1, display: 'flex', flexDirection: 'column', overflow: 'hidden', position: 'relative', minHeight: 0, maxHeight: '100%' }}>
            {sacLogsError && (
              <Alert severity="error" icon={<ErrorIcon />} sx={{ mb: 1 }}>
                Failed to load SAC training logs: {sacLogsError instanceof Error ? sacLogsError.message : 'Unknown error'}
              </Alert>
            )}
            <Box sx={{ mb: 1, display: 'flex', justifyContent: 'space-between', alignItems: 'center', flexShrink: 0 }}>
              <Typography variant="caption" color="text.secondary">
                {sacLogs?.filename ? `File: ${sacLogs.filename}` : 'Loading...'}
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
                {sacLogsLoading && <CircularProgress size={16} />}
                <Typography variant="caption" color="text.secondary">
                  {sacLogLines.length} lines
                </Typography>
                <Tooltip title="Scroll to top">
                  <IconButton 
                    size="small" 
                    onClick={() => scrollToTop(sacLogContainerRef)}
                    sx={{ width: 24, height: 24 }}
                  >
                    <VerticalAlignTop fontSize="small" />
                  </IconButton>
                </Tooltip>
                <Tooltip title="Scroll to bottom">
                  <IconButton 
                    size="small" 
                    onClick={() => scrollToBottom(sacLogContainerRef, 'sac')}
                    sx={{ width: 24, height: 24 }}
                  >
                    <VerticalAlignBottom fontSize="small" />
                  </IconButton>
                </Tooltip>
              </Box>
            </Box>
            <Paper
              ref={sacLogContainerRef}
              onScroll={() => handleScroll(sacLogContainerRef, 'sac')}
              sx={{
                bgcolor: '#1e1e1e',
                color: '#10b981',
                p: 2,
                flex: 1,
                minHeight: 0,
                maxHeight: '100%',
                overflowY: 'auto',
                overflowX: 'hidden',
                fontFamily: 'monospace',
                fontSize: '0.75rem',
                lineHeight: 1.6,
                position: 'relative',
                '&::-webkit-scrollbar': {
                  width: '12px',
                },
                '&::-webkit-scrollbar-track': {
                  background: '#2d2d2d',
                  borderRadius: '6px',
                },
                '&::-webkit-scrollbar-thumb': {
                  background: '#555',
                  borderRadius: '6px',
                  '&:hover': {
                    background: '#666',
                  },
                },
              }}
            >
              {sacLogsLoading && sacLogLines.length === 0 ? (
                <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
                  <CircularProgress size={30} />
                </Box>
              ) : sacLogLines.length === 0 ? (
                <Typography variant="body2" color="text.secondary" sx={{ textAlign: 'center', py: 4 }}>
                  No logs available
                </Typography>
              ) : (
                <>
                  {sacLogLines.map((line, idx) => {
                    const formatted = formatLogLine(line);
                    return (
                      <div 
                        key={`sac-log-${idx}`}
                        style={{ 
                          color: formatted.color,
                          marginBottom: '1px',
                          whiteSpace: 'pre-wrap',
                          wordBreak: 'break-word'
                        }}
                      >
                        {formatted.text}
                      </div>
                    );
                  })}
                  <div ref={sacLogEndRef} />
                </>
              )}
            </Paper>
          </Box>
        )}
      </CardContent>
    </Card>
  );
}

export default memo(CheckpointAndLogsCard);
