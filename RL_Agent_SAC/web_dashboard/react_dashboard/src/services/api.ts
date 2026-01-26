import type { StatusResponse, LogResponse, SACTrainingLogResponse, MetricsHistoryItem } from '../types';

const API_BASE = '/api';
const API_TIMEOUT = 30000; // 30 seconds

// Fetch with timeout
async function fetchWithTimeout(url: string, options: RequestInit = {}, timeout = API_TIMEOUT): Promise<Response> {
  const controller = new AbortController();
  const id = setTimeout(() => controller.abort(), timeout);
  
  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    clearTimeout(id);
    return response;
  } catch (error) {
    clearTimeout(id);
    if (error instanceof Error && error.name === 'AbortError') {
      throw new Error('Request timeout. Please try again.');
    }
    throw error;
  }
}

// Error handling wrapper
async function handleResponse<T>(response: Response): Promise<T> {
  if (!response.ok) {
    let errorData;
    try {
      errorData = await response.json();
    } catch {
      errorData = { error: 'Unknown error' };
    }
    throw new Error(errorData.error || errorData.message || `HTTP error! status: ${response.status}`);
  }
  return response.json();
}

export const api = {
  async getStatus(): Promise<StatusResponse> {
    try {
      const response = await fetchWithTimeout(`${API_BASE}/status`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse<StatusResponse>(response);
    } catch (error) {
      console.error('API getStatus error:', error);
      throw error;
    }
  },

  async getLogs(): Promise<LogResponse> {
    try {
      const response = await fetchWithTimeout(`${API_BASE}/logs`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse<LogResponse>(response);
    } catch (error) {
      console.error('API getLogs error:', error);
      throw error;
    }
  },

  async getSACTrainingLogs(): Promise<SACTrainingLogResponse> {
    try {
      const response = await fetchWithTimeout(`${API_BASE}/logs/sac_training`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse<SACTrainingLogResponse>(response);
    } catch (error) {
      console.error('API getSACTrainingLogs error:', error);
      throw error;
    }
  },

  async getMetricsHistory(): Promise<MetricsHistoryItem[]> {
    try {
      const response = await fetchWithTimeout(`${API_BASE}/metrics/history`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse<MetricsHistoryItem[]>(response);
    } catch (error) {
      console.error('API getMetricsHistory error:', error);
      throw error;
    }
  },

  async getCheckpoints() {
    try {
      const response = await fetchWithTimeout(`${API_BASE}/checkpoints`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse(response);
    } catch (error) {
      console.error('API getCheckpoints error:', error);
      throw error;
    }
  },

  async getAutoManageLog() {
    try {
      const response = await fetchWithTimeout(`${API_BASE}/logs/auto_manage`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse(response);
    } catch (error) {
      console.error('API getAutoManageLog error:', error);
      throw error;
    }
  },

  async healthCheck() {
    try {
      const response = await fetchWithTimeout('/health', {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      return handleResponse(response);
    } catch (error) {
      console.error('API healthCheck error:', error);
      throw error;
    }
  },
};
