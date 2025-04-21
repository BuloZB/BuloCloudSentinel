import React, { useState, useEffect, useRef } from 'react';
import DOMPurify from 'dompurify';
import './Widget.css';

const VideoWidget = ({ id, title, settings, onRemove }) => {
  const [videoStream, setVideoStream] = useState(null);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [streamSources, setStreamSources] = useState([]);
  const [selectedSource, setSelectedSource] = useState(settings?.defaultSource || null);
  const videoRef = useRef(null);

  // Fetch available video streams
  useEffect(() => {
    const fetchStreamSources = async () => {
      try {
        const response = await fetch('/api/video/streams');
        if (!response.ok) {
          throw new Error(`Failed to fetch video streams: ${response.status}`);
        }
        
        const data = await response.json();
        setStreamSources(data);
        
        // Set default source if available
        if (data.length > 0 && !selectedSource) {
          setSelectedSource(settings?.defaultSource || data[0].id);
        }
        
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching video streams:', error);
        setError('Failed to load video streams');
        setIsLoading(false);
      }
    };

    fetchStreamSources();
  }, [settings?.defaultSource, selectedSource]);

  // Connect to selected video stream
  useEffect(() => {
    if (!selectedSource) return;
    
    const connectToStream = async () => {
      try {
        setIsLoading(true);
        
        // Get stream details
        const response = await fetch(`/api/video/streams/${selectedSource}`);
        if (!response.ok) {
          throw new Error(`Failed to fetch stream details: ${response.status}`);
        }
        
        const streamData = await response.json();
        setVideoStream(streamData);
        
        // If WebRTC stream
        if (streamData.type === 'webrtc' && videoRef.current) {
          // WebRTC connection logic would go here
          // This is simplified for the example
          const pc = new RTCPeerConnection();
          
          // Add event handlers for WebRTC
          pc.ontrack = (event) => {
            if (videoRef.current) {
              videoRef.current.srcObject = event.streams[0];
            }
          };
          
          // Connect to signaling server
          // This is simplified for the example
          
          setIsLoading(false);
        } 
        // If HLS stream
        else if (streamData.type === 'hls' && videoRef.current) {
          // Use HLS.js or native HLS support
          if (videoRef.current.canPlayType('application/vnd.apple.mpegurl')) {
            // Native HLS support
            videoRef.current.src = streamData.url;
          } else {
            // HLS.js would be used here
            // This is simplified for the example
            videoRef.current.src = streamData.url;
          }
          
          setIsLoading(false);
        }
        // If RTSP stream
        else if (streamData.type === 'rtsp') {
          // RTSP streams typically need a server-side proxy
          // This is simplified for the example
          if (videoRef.current) {
            videoRef.current.src = `/api/video/proxy/${selectedSource}`;
          }
          
          setIsLoading(false);
        }
        
      } catch (error) {
        console.error('Error connecting to video stream:', error);
        setError('Failed to connect to video stream');
        setIsLoading(false);
      }
    };

    connectToStream();
    
    // Cleanup function
    return () => {
      if (videoRef.current) {
        videoRef.current.srcObject = null;
        videoRef.current.src = '';
      }
    };
  }, [selectedSource]);

  // Handle source change
  const handleSourceChange = (e) => {
    setSelectedSource(e.target.value);
  };

  return (
    <div className="widget video-widget">
      <div className="widget-header">
        <h3>{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            ✕
          </button>
        )}
      </div>
      
      <div className="widget-content">
        {isLoading ? (
          <div className="widget-loading">Loading video stream...</div>
        ) : error ? (
          <div className="widget-error">{error}</div>
        ) : (
          <>
            <div className="video-container">
              <video 
                ref={videoRef}
                autoPlay 
                playsInline
                muted
                controls
                className="video-player"
              />
            </div>
            
            <div className="video-controls">
              <select 
                value={selectedSource || ''} 
                onChange={handleSourceChange}
                className="video-source-select"
              >
                <option value="" disabled>Select video source</option>
                {streamSources.map(source => (
                  <option key={source.id} value={source.id}>
                    {DOMPurify.sanitize(source.name)}
                  </option>
                ))}
              </select>
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default VideoWidget;
