import React from 'react';
import AnalyticsDashboard from '../components/AnalyticsDashboard';

export default function Dashboard() {
  return (
    <div className="p-4">
      <h1 className="text-2xl font-bold mb-4">Dashboard</h1>
      <AnalyticsDashboard />
      {/* Additional dashboard components can be added here */}
    </div>
  );
}
