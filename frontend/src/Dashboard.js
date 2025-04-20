import React from 'react';
import ReactPlayer from 'react-player';

function Dashboard({ user, onLogout }) {
  return (
    <div className="min-h-screen bg-gray-100 dark:bg-gray-900 p-6">
      <header className="flex justify-between items-center mb-6">
        <h1 className="text-3xl font-bold text-gray-900 dark:text-gray-100">Dashboard</h1>
        <button
          onClick={onLogout}
          className="bg-red-600 text-white px-4 py-2 rounded hover:bg-red-700 transition"
        >
          Logout
        </button>
      </header>

      <section className="mb-6">
        <h2 className="text-xl font-semibold text-gray-800 dark:text-gray-200 mb-2">Welcome, {user.username}</h2>
        <p className="text-gray-700 dark:text-gray-300">Real-time video stream:</p>
        <div className="mt-4 aspect-w-16 aspect-h-9">
          <ReactPlayer
            url="http://localhost:8080/hls/live.m3u8"
            playing
            controls
            width="100%"
            height="100%"
          />
        </div>
      </section>

      <section>
        <h2 className="text-xl font-semibold text-gray-800 dark:text-gray-200 mb-2">Alerts & Logs</h2>
        <div className="bg-white dark:bg-gray-800 p-4 rounded shadow h-48 overflow-auto">
          {/* Placeholder for dynamic alerts and logs */}
          <p className="text-gray-600 dark:text-gray-400">No alerts at this time.</p>
        </div>
      </section>
    </div>
  );
}

export default Dashboard;
