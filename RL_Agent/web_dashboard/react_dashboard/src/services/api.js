import axios from 'axios'

const API_BASE_URL = '/api'

const api = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
})

export const dashboardAPI = {
  // Get training status
  getStatus: () => api.get('/status'),
  
  // Get checkpoints
  getCheckpoints: () => api.get('/checkpoints'),
  
  // Get metrics history
  getMetricsHistory: () => api.get('/metrics/history'),
  
  // Get auto manage logs
  getAutoManageLogs: () => api.get('/logs/auto_manage'),
  
  // Get tensorboard runs
  getTensorboardRuns: () => api.get('/tensorboard/runs'),
  
  // Get tensorboard view
  getTensorboardView: (runName) => api.get(`/tensorboard/view/${runName}`),
}

export default api

