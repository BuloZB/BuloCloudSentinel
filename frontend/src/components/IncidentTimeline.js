import React, { useState, useEffect } from 'react';

function IncidentTimeline() {
  const [events, setEvents] = useState([]);
  const [filter, setFilter] = useState('');
  const [search, setSearch] = useState('');

  useEffect(() => {
    async function fetchEvents() {
      let url = '/api/incident-timeline?';
      if (filter) {
        url += `label_filter=${encodeURIComponent(filter)}&`;
      }
      if (search) {
        url += `search=${encodeURIComponent(search)}&`;
      }
      const response = await fetch(url);
      const data = await response.json();
      setEvents(data);
    }
    fetchEvents();
  }, [filter, search]);

  return (
    <div className="p-4">
      <h2 className="text-xl font-bold mb-4">Incident Timeline</h2>
      <div className="mb-4">
        <input
          type="text"
          placeholder="Filter by label"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          className="border p-2 mr-2"
        />
        <input
          type="text"
          placeholder="Search metadata"
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          className="border p-2"
        />
      </div>
      <ul>
        {events.map((event) => (
          <li key={event.id} className="mb-2 flex items-center">
            {event.thumbnail_url && (
              <img src={event.thumbnail_url} alt={event.label} className="w-16 h-16 mr-4" />
            )}
            <div>
              <div className="font-semibold">{event.label}</div>
              <div className="text-sm text-gray-600">{new Date(event.timestamp).toLocaleString()}</div>
              <div className="text-xs text-gray-500">{JSON.stringify(event.metadata)}</div>
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default IncidentTimeline;
