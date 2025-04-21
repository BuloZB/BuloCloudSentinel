import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import * as d3 from 'd3';

const NetworkTopologyViewer = () => {
  const [topology, setTopology] = useState({ nodes: {}, connections: [] });
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedNode, setSelectedNode] = useState(null);
  const [refreshInterval, setRefreshInterval] = useState(5000);
  const svgRef = useRef(null);
  const intervalRef = useRef(null);

  // Fetch network topology data
  const fetchTopology = async () => {
    try {
      setLoading(true);
      const response = await axios.get('/api/mesh/topology');
      setTopology(response.data);
      setError(null);
    } catch (err) {
      console.error('Error fetching network topology:', err);
      setError('Failed to fetch network topology');
    } finally {
      setLoading(false);
    }
  };

  // Fetch node details when a node is selected
  const fetchNodeDetails = async (nodeId) => {
    try {
      const response = await axios.get(`/api/mesh/nodes/${nodeId}`);
      setSelectedNode(response.data);
    } catch (err) {
      console.error(`Error fetching node details for ${nodeId}:`, err);
      setError(`Failed to fetch details for node ${nodeId}`);
    }
  };

  // Initialize and update the D3 visualization
  const updateVisualization = () => {
    if (!svgRef.current || !topology || Object.keys(topology.nodes).length === 0) {
      return;
    }

    // Clear previous visualization
    d3.select(svgRef.current).selectAll('*').remove();

    // Set up the SVG container
    const width = svgRef.current.clientWidth;
    const height = svgRef.current.clientHeight || 500;
    const svg = d3.select(svgRef.current)
      .attr('width', width)
      .attr('height', height);

    // Create node data array from topology
    const nodes = Object.entries(topology.nodes).map(([id, data]) => ({
      id,
      status: data.status,
      capabilities: data.capabilities,
      x: Math.random() * width,
      y: Math.random() * height
    }));

    // Create link data array from topology
    const links = topology.connections.map(conn => ({
      source: conn.source,
      target: conn.target
    }));

    // Set up the simulation
    const simulation = d3.forceSimulation(nodes)
      .force('link', d3.forceLink(links).id(d => d.id).distance(100))
      .force('charge', d3.forceManyBody().strength(-300))
      .force('center', d3.forceCenter(width / 2, height / 2))
      .force('collision', d3.forceCollide().radius(30));

    // Create links
    const link = svg.append('g')
      .attr('class', 'links')
      .selectAll('line')
      .data(links)
      .enter()
      .append('line')
      .attr('stroke', '#999')
      .attr('stroke-opacity', 0.6)
      .attr('stroke-width', 2);

    // Create nodes
    const node = svg.append('g')
      .attr('class', 'nodes')
      .selectAll('g')
      .data(nodes)
      .enter()
      .append('g')
      .call(d3.drag()
        .on('start', dragstarted)
        .on('drag', dragged)
        .on('end', dragended));

    // Add circles for nodes
    node.append('circle')
      .attr('r', 15)
      .attr('fill', d => {
        switch (d.status) {
          case 'active': return '#4CAF50';
          case 'degraded': return '#FFC107';
          case 'offline': return '#F44336';
          case 'initializing': return '#2196F3';
          default: return '#9E9E9E';
        }
      })
      .attr('stroke', '#fff')
      .attr('stroke-width', 1.5)
      .on('click', (event, d) => {
        fetchNodeDetails(d.id);
      });

    // Add labels for nodes
    node.append('text')
      .attr('dx', 20)
      .attr('dy', 5)
      .text(d => d.id)
      .attr('fill', '#333')
      .style('font-size', '12px');

    // Add capability icons
    node.append('text')
      .attr('dx', 0)
      .attr('dy', 5)
      .attr('text-anchor', 'middle')
      .text(d => {
        if (d.capabilities.includes('gateway')) return '🌐';
        if (d.capabilities.includes('sensor')) return '📡';
        if (d.capabilities.includes('control')) return '🎮';
        if (d.capabilities.includes('ai')) return '🧠';
        return '📦';
      })
      .style('font-size', '12px');

    // Update positions on simulation tick
    simulation.on('tick', () => {
      link
        .attr('x1', d => d.source.x)
        .attr('y1', d => d.source.y)
        .attr('x2', d => d.target.x)
        .attr('y2', d => d.target.y);

      node
        .attr('transform', d => `translate(${d.x},${d.y})`);
    });

    // Drag functions
    function dragstarted(event, d) {
      if (!event.active) simulation.alphaTarget(0.3).restart();
      d.fx = d.x;
      d.fy = d.y;
    }

    function dragged(event, d) {
      d.fx = event.x;
      d.fy = event.y;
    }

    function dragended(event, d) {
      if (!event.active) simulation.alphaTarget(0);
      d.fx = null;
      d.fy = null;
    }
  };

  // Initial fetch and set up interval for refreshing
  useEffect(() => {
    fetchTopology();

    // Set up interval for periodic updates
    intervalRef.current = setInterval(fetchTopology, refreshInterval);

    // Clean up interval on component unmount
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [refreshInterval]);

  // Update visualization when topology changes
  useEffect(() => {
    updateVisualization();
  }, [topology]);

  // Handle refresh interval change
  const handleRefreshIntervalChange = (e) => {
    const newInterval = parseInt(e.target.value, 10);
    setRefreshInterval(newInterval);
    
    // Reset the interval with the new value
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }
    intervalRef.current = setInterval(fetchTopology, newInterval);
  };

  return (
    <div className="p-4">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-2xl font-bold">Network Topology</h2>
        <div className="flex items-center">
          <label htmlFor="refreshInterval" className="mr-2 text-sm">Refresh:</label>
          <select
            id="refreshInterval"
            value={refreshInterval}
            onChange={handleRefreshIntervalChange}
            className="border rounded p-1 text-sm"
          >
            <option value={2000}>2s</option>
            <option value={5000}>5s</option>
            <option value={10000}>10s</option>
            <option value={30000}>30s</option>
          </select>
          <button
            onClick={fetchTopology}
            className="ml-2 bg-blue-500 text-white px-3 py-1 rounded hover:bg-blue-600 text-sm"
          >
            Refresh Now
          </button>
        </div>
      </div>

      {loading && Object.keys(topology.nodes).length === 0 && (
        <div className="text-center py-4">
          <p>Loading network topology...</p>
        </div>
      )}

      {error && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
          <p>{error}</p>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div className="md:col-span-2">
          <div className="bg-white shadow rounded-lg p-4 mb-4">
            <div className="h-96 border rounded">
              <svg ref={svgRef} width="100%" height="100%"></svg>
            </div>
            <div className="mt-2 flex justify-center text-sm">
              <div className="flex items-center mr-4">
                <span className="inline-block w-3 h-3 rounded-full bg-green-500 mr-1"></span>
                <span>Active</span>
              </div>
              <div className="flex items-center mr-4">
                <span className="inline-block w-3 h-3 rounded-full bg-yellow-500 mr-1"></span>
                <span>Degraded</span>
              </div>
              <div className="flex items-center mr-4">
                <span className="inline-block w-3 h-3 rounded-full bg-red-500 mr-1"></span>
                <span>Offline</span>
              </div>
              <div className="flex items-center">
                <span className="inline-block w-3 h-3 rounded-full bg-blue-500 mr-1"></span>
                <span>Initializing</span>
              </div>
            </div>
          </div>

          <div className="bg-white shadow rounded-lg p-4">
            <h3 className="text-lg font-semibold mb-2">Network Statistics</h3>
            <div className="grid grid-cols-2 gap-4">
              <div>
                <p className="text-sm text-gray-500">Total Nodes</p>
                <p className="font-medium">{Object.keys(topology.nodes).length}</p>
              </div>
              <div>
                <p className="text-sm text-gray-500">Total Connections</p>
                <p className="font-medium">{topology.connections.length}</p>
              </div>
              <div>
                <p className="text-sm text-gray-500">Active Nodes</p>
                <p className="font-medium">
                  {Object.values(topology.nodes).filter(node => node.status === 'active').length}
                </p>
              </div>
              <div>
                <p className="text-sm text-gray-500">Degraded/Offline Nodes</p>
                <p className="font-medium">
                  {Object.values(topology.nodes).filter(node => 
                    node.status === 'degraded' || node.status === 'offline'
                  ).length}
                </p>
              </div>
            </div>
          </div>
        </div>

        <div>
          {selectedNode ? (
            <div className="bg-white shadow rounded-lg p-4">
              <div className="flex justify-between items-center mb-4">
                <h3 className="text-lg font-semibold">Node Details</h3>
                <button 
                  onClick={() => setSelectedNode(null)}
                  className="text-gray-500 hover:text-gray-700"
                >
                  ✕
                </button>
              </div>
              
              <div className="mb-4">
                <p className="text-sm text-gray-500">ID</p>
                <p className="font-medium">{selectedNode.node_id}</p>
              </div>
              
              <div className="mb-4">
                <p className="text-sm text-gray-500">Status</p>
                <div className="flex items-center">
                  <span className={`inline-block w-3 h-3 rounded-full mr-2 ${
                    selectedNode.status === 'active' ? 'bg-green-500' :
                    selectedNode.status === 'degraded' ? 'bg-yellow-500' :
                    selectedNode.status === 'offline' ? 'bg-red-500' :
                    'bg-blue-500'
                  }`}></span>
                  <span className="capitalize">{selectedNode.status}</span>
                </div>
              </div>
              
              <div className="mb-4">
                <p className="text-sm text-gray-500">Capabilities</p>
                <div className="flex flex-wrap gap-1 mt-1">
                  {selectedNode.capabilities.map((capability, index) => (
                    <span key={index} className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded">
                      {capability}
                    </span>
                  ))}
                </div>
              </div>
              
              <div className="mb-4">
                <p className="text-sm text-gray-500">Peer Count</p>
                <p className="font-medium">{selectedNode.peer_count}</p>
              </div>
              
              <div>
                <p className="text-sm text-gray-500">Metrics</p>
                <div className="mt-1 text-sm">
                  <div className="grid grid-cols-2 gap-2">
                    <div>
                      <p className="text-gray-500">Messages Sent</p>
                      <p>{selectedNode.metrics.messages_sent}</p>
                    </div>
                    <div>
                      <p className="text-gray-500">Messages Received</p>
                      <p>{selectedNode.metrics.messages_received}</p>
                    </div>
                    <div>
                      <p className="text-gray-500">Messages Forwarded</p>
                      <p>{selectedNode.metrics.messages_forwarded}</p>
                    </div>
                    <div>
                      <p className="text-gray-500">Messages Dropped</p>
                      <p>{selectedNode.metrics.messages_dropped}</p>
                    </div>
                  </div>
                </div>
              </div>
              
              <div className="mt-4 flex justify-between">
                <button 
                  className="bg-blue-500 text-white px-3 py-1 rounded hover:bg-blue-600 text-sm"
                  onClick={() => window.location.href = `/mesh/nodes/${selectedNode.node_id}/messages`}
                >
                  View Messages
                </button>
                <button 
                  className="bg-green-500 text-white px-3 py-1 rounded hover:bg-green-600 text-sm"
                  onClick={() => window.location.href = `/mesh/nodes/${selectedNode.node_id}/peers`}
                >
                  View Peers
                </button>
              </div>
            </div>
          ) : (
            <div className="bg-white shadow rounded-lg p-4">
              <h3 className="text-lg font-semibold mb-2">Node Details</h3>
              <p className="text-gray-500">Select a node to view details</p>
            </div>
          )}

          <div className="bg-white shadow rounded-lg p-4 mt-4">
            <h3 className="text-lg font-semibold mb-2">Network Actions</h3>
            <div className="space-y-2">
              <button 
                className="w-full bg-blue-500 text-white px-4 py-2 rounded hover:bg-blue-600 text-sm"
                onClick={() => window.location.href = '/mesh/nodes/add'}
              >
                Add New Node
              </button>
              <button 
                className="w-full bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600 text-sm"
                onClick={() => window.location.href = '/mesh/connect'}
              >
                Connect Nodes
              </button>
              <button 
                className="w-full bg-purple-500 text-white px-4 py-2 rounded hover:bg-purple-600 text-sm"
                onClick={() => window.location.href = '/mesh/discovery'}
              >
                Run Discovery
              </button>
              <button 
                className="w-full bg-gray-500 text-white px-4 py-2 rounded hover:bg-gray-600 text-sm"
                onClick={() => window.location.href = '/mesh/metrics'}
              >
                View Detailed Metrics
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default NetworkTopologyViewer;
