import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import SystemResourcesCard from '../components/Dashboard/SystemResourcesCard';

describe('SystemResourcesCard', () => {
  const mockSystem = {
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
    disk: {
      used_percent: 70,
      used_gb: 140,
      total_gb: 200,
      free_gb: 60,
    },
  };

  it('renders CPU, Memory, and Disk information', () => {
    render(<SystemResourcesCard system={mockSystem} />);

    expect(screen.getByText('CPU Usage')).toBeInTheDocument();
    expect(screen.getByText('Memory Usage')).toBeInTheDocument();
    expect(screen.getByText('Disk Usage')).toBeInTheDocument();
  });

  it('handles missing disk gracefully', () => {
    const systemWithoutDisk = {
      ...mockSystem,
      disk: undefined,
    };

    render(<SystemResourcesCard system={systemWithoutDisk} />);

    expect(screen.getByText('CPU Usage')).toBeInTheDocument();
    expect(screen.getByText('Memory Usage')).toBeInTheDocument();
    expect(screen.queryByText('Disk Usage')).not.toBeInTheDocument();
  });

  it('displays power information when provided', () => {
    const power = {
      power_draw: 150,
      energy_used: 2.5,
      cost_estimate: 11.25,
    };

    render(<SystemResourcesCard system={mockSystem} power={power} />);

    expect(screen.getByText('Power & Cost')).toBeInTheDocument();
    expect(screen.getByText('150W')).toBeInTheDocument();
    expect(screen.getByText('2.50kWh')).toBeInTheDocument();
    expect(screen.getByText('11.25฿')).toBeInTheDocument();
  });

  it('handles missing power gracefully', () => {
    render(<SystemResourcesCard system={mockSystem} />);

    expect(screen.queryByText('Power & Cost')).not.toBeInTheDocument();
  });

  it('displays GPU information when provided', () => {
    const systemWithGPU = {
      ...mockSystem,
      gpu: [
        {
          name: 'NVIDIA RTX 4090',
          memory_used: 8,
          memory_total: 24,
          utilization: 75,
          temperature: 65,
        },
      ],
    };

    render(<SystemResourcesCard system={systemWithGPU} />);

    expect(screen.getByText('GPU Status')).toBeInTheDocument();
    expect(screen.getByText('NVIDIA RTX 4090')).toBeInTheDocument();
  });
});

