export interface StatusResponse {
  checkpoint: {
    timestep: number;
    path?: string;
  } | null;
  metrics: {
    current_step: number;
    last_reward?: number;
    episode_reward?: number;
    episode_length?: number;
  };
  status: {
    running: boolean;
  };
  system: {
    cpu: {
      usage: number;
      percent: number;
      used_gb: number;
      total_gb: number;
      free_gb: number;
      temperature?: number | null;
    };
    memory: {
      percent: number;
      used_gb: number;
      total_gb: number;
      free_gb: number;
    };
    disk?: {
      used_percent: number;
      used_gb: number;
      total_gb: number;
      free_gb: number;
    };
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
  progress: {
    current: number;
    started: number;
    target: number;
    remaining: number;
    percentage: number;
  };
  algorithm: string;
  timestamp: string;
}

export interface LogResponse {
  content: string;
  filename: string;
}

export interface SACTrainingLogResponse {
  content: string;
  filename: string;
}

export interface MetricsHistoryItem {
  step: number;
  reward: number;
  timestamp: string;
}

