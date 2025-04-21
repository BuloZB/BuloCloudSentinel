import React, { useState } from 'react';
import NetworkTopologyViewer from '../components/NetworkTopologyViewer';
import axios from 'axios';

export default function MeshNetworking() {
  const [activeTab, setActiveTab] = useState('topology');
  const [formData, setFormData] = useState({
    nodeId: '',
    capabilities: ['routing'],
    node1Id: '',
    node2Id: '',
    senderId: '',
    targetId: '',
    messageContent: '',
    messageType: 'data',
    priority: 'normal',
    ttl: 10
  });
  const [loading, setLoading] = useState(false);
  const [result, setResult] = useState(null);
  const [error, setError] = useState(null);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleCheckboxChange = (e) => {
    const { name, checked } = e.target;
    if (checked) {
      setFormData(prev => ({
        ...prev,
        capabilities: [...prev.capabilities, name]
      }));
    } else {
      setFormData(prev => ({
        ...prev,
        capabilities: prev.capabilities.filter(cap => cap !== name)
      }));
    }
  };

  const handleAddNode = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setResult(null);

    try {
      const response = await axios.post('/api/mesh/nodes', {
        node_id: formData.nodeId,
        capabilities: formData.capabilities
      });
      setResult(response.data);
    } catch (err) {
      setError(err.response?.data?.detail || 'Failed to add node');
    } finally {
      setLoading(false);
    }
  };

  const handleConnectNodes = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setResult(null);

    try {
      const response = await axios.post('/api/mesh/connect', {
        node1_id: formData.node1Id,
        node2_id: formData.node2Id
      });
      setResult(response.data);
    } catch (err) {
      setError(err.response?.data?.detail || 'Failed to connect nodes');
    } finally {
      setLoading(false);
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setResult(null);

    try {
      const response = await axios.post(`/api/mesh/nodes/${formData.senderId}/messages/direct`, {
        sender_id: formData.senderId,
        target_id: formData.targetId,
        content: formData.messageContent,
        message_type: formData.messageType,
        priority: formData.priority
      });
      setResult(response.data);
    } catch (err) {
      setError(err.response?.data?.detail || 'Failed to send message');
    } finally {
      setLoading(false);
    }
  };

  const handleBroadcastMessage = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setResult(null);

    try {
      const response = await axios.post(`/api/mesh/nodes/${formData.senderId}/messages/broadcast`, {
        sender_id: formData.senderId,
        content: formData.messageContent,
        message_type: formData.messageType,
        priority: formData.priority,
        ttl: parseInt(formData.ttl, 10)
      });
      setResult(response.data);
    } catch (err) {
      setError(err.response?.data?.detail || 'Failed to broadcast message');
    } finally {
      setLoading(false);
    }
  };

  const renderTabContent = () => {
    switch (activeTab) {
      case 'topology':
        return <NetworkTopologyViewer />;
      
      case 'addNode':
        return (
          <div className="p-4">
            <h2 className="text-2xl font-bold mb-4">Add New Node</h2>
            <form onSubmit={handleAddNode} className="bg-white shadow rounded-lg p-6">
              <div className="mb-4">
                <label htmlFor="nodeId" className="block text-sm font-medium text-gray-700 mb-1">
                  Node ID
                </label>
                <input
                  type="text"
                  id="nodeId"
                  name="nodeId"
                  value={formData.nodeId}
                  onChange={handleInputChange}
                  className="w-full p-2 border rounded"
                  required
                />
              </div>
              
              <div className="mb-4">
                <p className="block text-sm font-medium text-gray-700 mb-1">
                  Capabilities
                </p>
                <div className="space-y-2">
                  {['routing', 'sensor', 'control', 'storage', 'gateway', 'ai'].map(capability => (
                    <div key={capability} className="flex items-center">
                      <input
                        type="checkbox"
                        id={capability}
                        name={capability}
                        checked={formData.capabilities.includes(capability)}
                        onChange={handleCheckboxChange}
                        className="mr-2"
                      />
                      <label htmlFor={capability} className="text-sm text-gray-700 capitalize">
                        {capability}
                      </label>
                    </div>
                  ))}
                </div>
              </div>
              
              <button
                type="submit"
                className="w-full bg-blue-500 text-white py-2 px-4 rounded hover:bg-blue-600"
                disabled={loading}
              >
                {loading ? 'Adding...' : 'Add Node'}
              </button>
            </form>
            
            {result && (
              <div className="mt-4 p-4 bg-green-100 text-green-800 rounded">
                <p>{result.detail}</p>
              </div>
            )}
            
            {error && (
              <div className="mt-4 p-4 bg-red-100 text-red-800 rounded">
                <p>{error}</p>
              </div>
            )}
          </div>
        );
      
      case 'connectNodes':
        return (
          <div className="p-4">
            <h2 className="text-2xl font-bold mb-4">Connect Nodes</h2>
            <form onSubmit={handleConnectNodes} className="bg-white shadow rounded-lg p-6">
              <div className="mb-4">
                <label htmlFor="node1Id" className="block text-sm font-medium text-gray-700 mb-1">
                  First Node ID
                </label>
                <input
                  type="text"
                  id="node1Id"
                  name="node1Id"
                  value={formData.node1Id}
                  onChange={handleInputChange}
                  className="w-full p-2 border rounded"
                  required
                />
              </div>
              
              <div className="mb-4">
                <label htmlFor="node2Id" className="block text-sm font-medium text-gray-700 mb-1">
                  Second Node ID
                </label>
                <input
                  type="text"
                  id="node2Id"
                  name="node2Id"
                  value={formData.node2Id}
                  onChange={handleInputChange}
                  className="w-full p-2 border rounded"
                  required
                />
              </div>
              
              <button
                type="submit"
                className="w-full bg-blue-500 text-white py-2 px-4 rounded hover:bg-blue-600"
                disabled={loading}
              >
                {loading ? 'Connecting...' : 'Connect Nodes'}
              </button>
            </form>
            
            {result && (
              <div className="mt-4 p-4 bg-green-100 text-green-800 rounded">
                <p>{result.detail}</p>
              </div>
            )}
            
            {error && (
              <div className="mt-4 p-4 bg-red-100 text-red-800 rounded">
                <p>{error}</p>
              </div>
            )}
          </div>
        );
      
      case 'sendMessage':
        return (
          <div className="p-4">
            <h2 className="text-2xl font-bold mb-4">Send Message</h2>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <form onSubmit={handleSendMessage} className="bg-white shadow rounded-lg p-6">
                <h3 className="text-lg font-semibold mb-4">Direct Message</h3>
                <div className="mb-4">
                  <label htmlFor="senderId" className="block text-sm font-medium text-gray-700 mb-1">
                    Sender Node ID
                  </label>
                  <input
                    type="text"
                    id="senderId"
                    name="senderId"
                    value={formData.senderId}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                    required
                  />
                </div>
                
                <div className="mb-4">
                  <label htmlFor="targetId" className="block text-sm font-medium text-gray-700 mb-1">
                    Target Node ID
                  </label>
                  <input
                    type="text"
                    id="targetId"
                    name="targetId"
                    value={formData.targetId}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                    required
                  />
                </div>
                
                <div className="mb-4">
                  <label htmlFor="messageContent" className="block text-sm font-medium text-gray-700 mb-1">
                    Message Content
                  </label>
                  <textarea
                    id="messageContent"
                    name="messageContent"
                    value={formData.messageContent}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                    rows="3"
                    required
                  ></textarea>
                </div>
                
                <div className="mb-4">
                  <label htmlFor="messageType" className="block text-sm font-medium text-gray-700 mb-1">
                    Message Type
                  </label>
                  <select
                    id="messageType"
                    name="messageType"
                    value={formData.messageType}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                  >
                    <option value="data">Data</option>
                    <option value="control">Control</option>
                    <option value="discovery">Discovery</option>
                    <option value="heartbeat">Heartbeat</option>
                    <option value="routing">Routing</option>
                    <option value="ack">Acknowledgment</option>
                  </select>
                </div>
                
                <div className="mb-4">
                  <label htmlFor="priority" className="block text-sm font-medium text-gray-700 mb-1">
                    Priority
                  </label>
                  <select
                    id="priority"
                    name="priority"
                    value={formData.priority}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                  >
                    <option value="low">Low</option>
                    <option value="normal">Normal</option>
                    <option value="high">High</option>
                    <option value="critical">Critical</option>
                  </select>
                </div>
                
                <button
                  type="submit"
                  className="w-full bg-blue-500 text-white py-2 px-4 rounded hover:bg-blue-600"
                  disabled={loading}
                >
                  {loading ? 'Sending...' : 'Send Direct Message'}
                </button>
              </form>
              
              <form onSubmit={handleBroadcastMessage} className="bg-white shadow rounded-lg p-6">
                <h3 className="text-lg font-semibold mb-4">Broadcast Message</h3>
                <div className="mb-4">
                  <label htmlFor="senderId" className="block text-sm font-medium text-gray-700 mb-1">
                    Sender Node ID
                  </label>
                  <input
                    type="text"
                    id="senderId"
                    name="senderId"
                    value={formData.senderId}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                    required
                  />
                </div>
                
                <div className="mb-4">
                  <label htmlFor="messageContent" className="block text-sm font-medium text-gray-700 mb-1">
                    Message Content
                  </label>
                  <textarea
                    id="messageContent"
                    name="messageContent"
                    value={formData.messageContent}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                    rows="3"
                    required
                  ></textarea>
                </div>
                
                <div className="mb-4">
                  <label htmlFor="messageType" className="block text-sm font-medium text-gray-700 mb-1">
                    Message Type
                  </label>
                  <select
                    id="messageType"
                    name="messageType"
                    value={formData.messageType}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                  >
                    <option value="data">Data</option>
                    <option value="control">Control</option>
                    <option value="discovery">Discovery</option>
                    <option value="heartbeat">Heartbeat</option>
                    <option value="routing">Routing</option>
                    <option value="ack">Acknowledgment</option>
                  </select>
                </div>
                
                <div className="mb-4">
                  <label htmlFor="priority" className="block text-sm font-medium text-gray-700 mb-1">
                    Priority
                  </label>
                  <select
                    id="priority"
                    name="priority"
                    value={formData.priority}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                  >
                    <option value="low">Low</option>
                    <option value="normal">Normal</option>
                    <option value="high">High</option>
                    <option value="critical">Critical</option>
                  </select>
                </div>
                
                <div className="mb-4">
                  <label htmlFor="ttl" className="block text-sm font-medium text-gray-700 mb-1">
                    Time-to-Live (TTL)
                  </label>
                  <input
                    type="number"
                    id="ttl"
                    name="ttl"
                    value={formData.ttl}
                    onChange={handleInputChange}
                    className="w-full p-2 border rounded"
                    min="1"
                    max="20"
                    required
                  />
                </div>
                
                <button
                  type="submit"
                  className="w-full bg-blue-500 text-white py-2 px-4 rounded hover:bg-blue-600"
                  disabled={loading}
                >
                  {loading ? 'Broadcasting...' : 'Broadcast Message'}
                </button>
              </form>
            </div>
            
            {result && (
              <div className="mt-4 p-4 bg-green-100 text-green-800 rounded">
                <p>{result.detail}</p>
              </div>
            )}
            
            {error && (
              <div className="mt-4 p-4 bg-red-100 text-red-800 rounded">
                <p>{error}</p>
              </div>
            )}
          </div>
        );
      
      default:
        return <NetworkTopologyViewer />;
    }
  };

  return (
    <div className="h-full flex flex-col">
      <div className="bg-gray-100 border-b">
        <div className="container mx-auto px-4">
          <div className="flex overflow-x-auto">
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'topology'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('topology')}
            >
              Network Topology
            </button>
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'addNode'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('addNode')}
            >
              Add Node
            </button>
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'connectNodes'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('connectNodes')}
            >
              Connect Nodes
            </button>
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'sendMessage'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('sendMessage')}
            >
              Send Messages
            </button>
          </div>
        </div>
      </div>
      
      <div className="flex-1 overflow-auto">
        {renderTabContent()}
      </div>
    </div>
  );
}
