import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import axios from 'axios';
import Missions from '../../frontend/src/pages/Missions';

jest.mock('axios');

const mockMissions = [
  {
    id: 1,
    name: 'Test Mission',
    waypoints: [
      { latitude: 10, longitude: 20, altitude: 100, action: 'takeoff', order: 1 },
      { latitude: 10.1, longitude: 20.1, altitude: 110, action: 'hover', order: 2 }
    ]
  }
];

describe('Missions Page', () => {
  beforeEach(() => {
    axios.get.mockResolvedValue({ data: mockMissions });
  });

  test('renders mission list and selects a mission', async () => {
    render(<Missions />);
    expect(screen.getByText(/Missions/i)).toBeInTheDocument();

    await waitFor(() => {
      expect(screen.getByText('Test Mission')).toBeInTheDocument();
    });

    fireEvent.click(screen.getByText('Test Mission'));

    await waitFor(() => {
      expect(screen.getByText('Test Mission')).toBeInTheDocument();
      expect(screen.getByText(/Select a mission to view details/i)).not.toBeInTheDocument();
    });
  });
});
