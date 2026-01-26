import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import ProgressCard from '../components/Dashboard/ProgressCard';

describe('ProgressCard', () => {
  const mockProgress = {
    current: 5000,
    started: 0,
    target: 500000,
    remaining: 495000,
    percentage: 1.0,
  };

  it('renders progress information', () => {
    render(<ProgressCard progress={mockProgress} />);

    expect(screen.getByText('Training Overview')).toBeInTheDocument();
    expect(screen.getByText('1.0%')).toBeInTheDocument();
    expect(screen.getByText('Remaining Steps')).toBeInTheDocument();
    expect(screen.getByText('495,000')).toBeInTheDocument();
  });

  it('displays started and target values', () => {
    render(<ProgressCard progress={mockProgress} />);

    expect(screen.getByText('0')).toBeInTheDocument(); // Started
    expect(screen.getByText('500,000')).toBeInTheDocument(); // Target
  });
});

