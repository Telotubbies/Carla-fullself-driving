import { lazy, Suspense, useEffect, useState } from 'react';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import { Box, CircularProgress, ThemeProvider, Alert, Snackbar } from '@mui/material';
import CssBaseline from '@mui/material/CssBaseline';
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';
import { getTheme } from './styles/theme';
import ErrorBoundary from './components/ErrorBoundary';

const DashboardContent = lazy(() => import('./components/Dashboard/DashboardContent'));
const ProjectOverview = lazy(() => import('./components/Pages/ProjectOverview'));
const WorkflowDiagram = lazy(() => import('./components/Pages/WorkflowDiagram'));

// Create QueryClient with production-ready configuration
const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      refetchOnWindowFocus: false,
      refetchOnReconnect: true,
      retry: 3,
      retryDelay: (attemptIndex) => Math.min(1000 * 2 ** attemptIndex, 30000),
      staleTime: 5000, // 5 seconds
      gcTime: 300000, // 5 minutes (formerly cacheTime)
    },
    mutations: {
      retry: 1,
    },
  },
});

function App() {
  const [theme, setTheme] = useState(getTheme());
  const [offline, setOffline] = useState(!navigator.onLine);
  const [error, setError] = useState<string | null>(null);

  // Update theme every minute based on time
  useEffect(() => {
    const interval = setInterval(() => {
      setTheme(getTheme());
    }, 60000); // Check every minute

    return () => clearInterval(interval);
  }, []);

  // Handle online/offline status
  useEffect(() => {
    const handleOnline = () => {
      setOffline(false);
      setError(null);
    };
    const handleOffline = () => {
      setOffline(true);
      setError('You are currently offline. Some features may not work.');
    };

    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, []);

  // Global error handler
  useEffect(() => {
    const handleError = (event: ErrorEvent) => {
      console.error('Global error:', event.error);
      setError('An unexpected error occurred. Please refresh the page.');
    };

    const handleUnhandledRejection = (event: PromiseRejectionEvent) => {
      console.error('Unhandled promise rejection:', event.reason);
      setError('A network error occurred. Please check your connection.');
    };

    window.addEventListener('error', handleError);
    window.addEventListener('unhandledrejection', handleUnhandledRejection);

    return () => {
      window.removeEventListener('error', handleError);
      window.removeEventListener('unhandledrejection', handleUnhandledRejection);
    };
  }, []);

  return (
    <ErrorBoundary>
      <QueryClientProvider client={queryClient}>
        <ThemeProvider theme={theme}>
          <CssBaseline />
          <BrowserRouter>
            <Suspense
              fallback={
                <Box
                  display="flex"
                  justifyContent="center"
                  alignItems="center"
                  minHeight="100vh"
                  bgcolor="background.default"
                >
                  <CircularProgress />
                </Box>
              }
            >
              <Routes>
                <Route path="/" element={<DashboardContent />} />
                <Route path="/overview" element={<ProjectOverview />} />
                <Route path="/workflow" element={<WorkflowDiagram />} />
              </Routes>
            </Suspense>
          </BrowserRouter>

          {/* Global error/status notifications */}
          <Snackbar
            open={offline}
            anchorOrigin={{ vertical: 'top', horizontal: 'center' }}
            autoHideDuration={null}
          >
            <Alert severity="warning" onClose={() => setOffline(false)}>
              You are offline. Some features may not work.
            </Alert>
          </Snackbar>

          <Snackbar
            open={!!error}
            anchorOrigin={{ vertical: 'top', horizontal: 'center' }}
            autoHideDuration={6000}
            onClose={() => setError(null)}
          >
            <Alert severity="error" onClose={() => setError(null)}>
              {error}
            </Alert>
          </Snackbar>
        </ThemeProvider>
      </QueryClientProvider>
    </ErrorBoundary>
  );
}

export default App;
