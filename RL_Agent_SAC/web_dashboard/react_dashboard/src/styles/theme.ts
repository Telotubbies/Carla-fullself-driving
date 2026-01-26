import { createTheme, Theme } from '@mui/material/styles';

const getThemeMode = (): 'light' | 'dark' => {
  const hour = new Date().getHours();
  // Dark mode: 18:00 - 06:00 (6 PM - 6 AM)
  // Light mode: 06:00 - 18:00 (6 AM - 6 PM)
  return hour >= 18 || hour < 6 ? 'dark' : 'light';
};

const createAppTheme = (mode: 'light' | 'dark'): Theme => {
  return createTheme({
    palette: {
      mode,
      primary: {
        main: '#667eea',
        light: '#8b9ef5',
        dark: '#4c63d2',
      },
      secondary: {
        main: '#764ba2',
        light: '#9a6bc4',
        dark: '#5a3780',
      },
      success: {
        main: '#10b981',
      },
      error: {
        main: '#ef4444',
      },
      warning: {
        main: '#f59e0b',
      },
      info: {
        main: '#3b82f6',
      },
      ...(mode === 'dark'
        ? {
            background: {
              default: '#121212',
              paper: '#1e1e1e',
            },
            text: {
              primary: '#ffffff',
              secondary: 'rgba(255, 255, 255, 0.7)',
            },
          }
        : {
            background: {
              default: '#f8fafc',
              paper: '#ffffff',
            },
            text: {
              primary: 'rgba(0, 0, 0, 0.87)',
              secondary: 'rgba(0, 0, 0, 0.6)',
            },
          }),
    },
    typography: {
      fontFamily: '"Inter", "Roboto", "Helvetica", "Arial", sans-serif',
      h4: {
        fontWeight: 700,
      },
      h5: {
        fontWeight: 600,
      },
      h6: {
        fontWeight: 600,
      },
    },
    shape: {
      borderRadius: 12,
    },
    components: {
      MuiCard: {
        styleOverrides: {
          root: {
            boxShadow:
              mode === 'dark'
                ? '0 4px 6px -1px rgba(0, 0, 0, 0.3), 0 2px 4px -1px rgba(0, 0, 0, 0.2)'
                : '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)',
          },
        },
      },
      MuiButton: {
        styleOverrides: {
          root: {
            textTransform: 'none',
            fontWeight: 500,
          },
        },
      },
    },
  });
};

export const getTheme = () => createAppTheme(getThemeMode());
export const theme = getTheme();
