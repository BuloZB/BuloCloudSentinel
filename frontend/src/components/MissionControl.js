import React, { useState, useEffect, useRef } from 'react';

export default function MissionControl({ websocketUrl }) {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const ws = useRef(null);

  useEffect(() => {
    ws.current = new WebSocket(websocketUrl);

    ws.current.onopen = () => {
      console.log('WebSocket connected');
    };

    ws.current.onmessage = (event) => {
      setMessages(prev => [...prev, event.data]);
    };

    ws.current.onclose = () => {
      console.log('WebSocket disconnected');
    };

    return () => {
      ws.current.close();
    };
  }, [websocketUrl]);

  const sendMessage = () => {
    if (ws.current && input.trim() !== '') {
      ws.current.send(input);
      setInput('');
    }
  };

  return (
    <div className="p-4 border rounded shadow">
      <h3 className="text-lg font-bold mb-2">Mission Control</h3>
      <div className="h-40 overflow-y-auto border p-2 mb-2 bg-gray-100">
        {messages.map((msg, idx) => (
          <div key={idx} className="text-sm">{msg}</div>
        ))}
      </div>
      <input
        type="text"
        className="w-full p-2 border rounded mb-2"
        value={input}
        onChange={e => setInput(e.target.value)}
        placeholder="Send command or override"
      />
      <button
        className="w-full bg-blue-600 text-white p-2 rounded hover:bg-blue-700"
        onClick={sendMessage}
      >
        Send
      </button>
    </div>
  );
}
