import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import StatusCard from '../components/Dashboard/StatusCard';
import type { StatusResponse } from '../types';

describe('StatusCard', () => {
  const mockStatus: StatusResponse = {
    checkpoint: {
      timestep: 5000,
      path: 'checkpoints/test.zip',
    },
    metrics: {
      current_step: 5000,
      last_reward: 25.5,
      episode_reward: 30.2,
      episode_length: 10,
    },
    status: {
      running: true,
    },
    system: {
      cpu: {
        usage: 50,
        percent: 50,
        used_gb: 10,
        total_gb: 20,
        free_gb: 10,
      },
      memory: {
        percent: 60,
        used_gb: 30,
        total_gb: 50,
        free_gb: 20,
      },
    },
    progress: {
      current: 5000,
      started: 0,
      target: 500000,
      remaining: 495000,
      percentage: 1.0,
    },
    algorithm: 'SAC',
    timestamp: '2026-01-07 12:00:00',
  };

  it('renders training progress information', () => {
    render(<StatusCard status={mockStatus} />);

    expect(screen.getByText('Training Progress')).toBeInTheDocument();
    expect(screen.getByText('5,000')).toBeInTheDocument(); // Current Step
    expect(screen.getByText('500,000')).toBeInTheDocument(); // Target Steps
    expect(screen.getByText('1.0%')).toBeInTheDocument(); // Progress
  });

  it('displays metrics correctly', () => {
    render(<StatusCard status={mockStatus} />);

    expect(screen.getByText('25.50')).toBeInTheDocument(); // Last Reward
    expect(screen.getByText('30.20')).toBeInTheDocument(); // Episode Reward
    expect(screen.getByText('SAC')).toBeInTheDocument(); // Algorithm
  });

  it('handles missing metrics gracefully', () => {
    const statusWithoutMetrics: StatusResponse = {
      ...mockStatus,
      metrics: {
        current_step: 0,
      },
    };

    render(<StatusCard status={statusWithoutMetrics} />);

    expect(screen.getByText('Training Progress')).toBeInTheDocument();
    expect(screen.getAllByText('-')).toBeTruthy(); // Missing values show as '-'
  });
});

