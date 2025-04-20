import React, { useState, useEffect } from 'react';

function AIToolsPanel() {
  const [status, setStatus] = useState({});
  const [tool, setTool] = useState('chatgpt');
  const [prompt, setPrompt] = useState('');
  const [response, setResponse] = useState('');
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    async function fetchStatus() {
      const res = await fetch('/ai/status');
      const data = await res.json();
      setStatus(data);
    }
    fetchStatus();
  }, []);

  async function handleSubmit() {
    setLoading(true);
    const res = await fetch('/ai/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ tool, prompt }),
    });
    const data = await res.json();
    setResponse(data.response);
    setLoading(false);
  }

  return (
    <div className="p-4 border rounded">
      <h3 className="text-lg font-bold mb-2">AI Tools Panel</h3>
      <div className="mb-2">
        <label htmlFor="tool" className="block mb-1">Select AI Tool:</label>
        <select id="tool" value={tool} onChange={(e) => setTool(e.target.value)} className="border p-1 w-full">
          {Object.keys(status).map((key) => (
            <option key={key} value={key}>
              {key} {status[key].available ? '(Available)' : '(Unavailable)'}
            </option>
          ))}
        </select>
      </div>
      <div className="mb-2">
        <textarea
          rows="4"
          placeholder="Enter your prompt here..."
          value={prompt}
          onChange={(e) => setPrompt(e.target.value)}
          className="border p-2 w-full"
        />
      </div>
      <button
        onClick={handleSubmit}
        disabled={loading || !prompt}
        className="bg-blue-600 text-white px-4 py-2 rounded disabled:opacity-50"
      >
        {loading ? 'Processing...' : 'Send'}
      </button>
      {response && (
        <div className="mt-4 p-2 border rounded bg-gray-100 whitespace-pre-wrap">
          <strong>Response:</strong>
          <div>{response}</div>
        </div>
      )}
    </div>
  );
}

export default AIToolsPanel;
