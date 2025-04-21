import React, { useState, useEffect } from 'react';
import DOMPurify from 'dompurify';
import './Widget.css';

const WeatherWidget = ({ id, title, settings, onRemove }) => {
  const [weatherData, setWeatherData] = useState(null);
  const [locations, setLocations] = useState([]);
  const [selectedLocation, setSelectedLocation] = useState(settings?.defaultLocation || null);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  
  // Fetch available locations
  useEffect(() => {
    const fetchLocations = async () => {
      try {
        const response = await fetch('/api/weather/locations');
        if (!response.ok) {
          throw new Error(`Failed to fetch locations: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Sanitize location data
        const sanitizedLocations = data.map(location => ({
          ...location,
          name: DOMPurify.sanitize(location.name),
          description: location.description ? DOMPurify.sanitize(location.description) : null
        }));
        
        setLocations(sanitizedLocations);
        
        // Set default location if available
        if (sanitizedLocations.length > 0 && !selectedLocation) {
          setSelectedLocation(settings?.defaultLocation || sanitizedLocations[0].id);
        }
        
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching locations:', error);
        setError('Failed to load locations');
        setIsLoading(false);
      }
    };

    fetchLocations();
  }, [settings?.defaultLocation, selectedLocation]);

  // Fetch weather data when location changes
  useEffect(() => {
    if (!selectedLocation) return;
    
    const fetchWeatherData = async () => {
      try {
        setIsLoading(true);
        
        const response = await fetch(`/api/weather/${selectedLocation}`);
        if (!response.ok) {
          throw new Error(`Failed to fetch weather data: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Sanitize weather data
        const sanitizedData = {
          ...data,
          location: {
            ...data.location,
            name: DOMPurify.sanitize(data.location.name)
          },
          current: {
            ...data.current,
            condition: {
              ...data.current.condition,
              text: DOMPurify.sanitize(data.current.condition.text)
            }
          },
          forecast: data.forecast.map(item => ({
            ...item,
            condition: {
              ...item.condition,
              text: DOMPurify.sanitize(item.condition.text)
            }
          })),
          alerts: data.alerts.map(alert => ({
            ...alert,
            title: DOMPurify.sanitize(alert.title),
            description: DOMPurify.sanitize(alert.description)
          }))
        };
        
        setWeatherData(sanitizedData);
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching weather data:', error);
        setError('Failed to load weather data');
        setIsLoading(false);
      }
    };

    fetchWeatherData();
    
    // Set up polling for updates
    const intervalId = setInterval(fetchWeatherData, 300000); // Update every 5 minutes
    
    // Cleanup
    return () => clearInterval(intervalId);
  }, [selectedLocation]);

  // Handle location selection
  const handleLocationChange = (e) => {
    setSelectedLocation(e.target.value);
  };

  // Format date
  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleDateString(undefined, { weekday: 'short', month: 'short', day: 'numeric' });
  };

  // Format time
  const formatTime = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleTimeString(undefined, { hour: '2-digit', minute: '2-digit' });
  };

  // Get weather icon
  const getWeatherIcon = (condition) => {
    const iconMap = {
      'clear': '☀️',
      'sunny': '☀️',
      'partly cloudy': '⛅',
      'cloudy': '☁️',
      'overcast': '☁️',
      'mist': '🌫️',
      'fog': '🌫️',
      'rain': '🌧️',
      'light rain': '🌦️',
      'heavy rain': '🌧️',
      'showers': '🌦️',
      'thunderstorm': '⛈️',
      'snow': '❄️',
      'sleet': '🌨️',
      'hail': '🌨️',
      'windy': '💨'
    };
    
    const conditionLower = condition.toLowerCase();
    
    for (const [key, value] of Object.entries(iconMap)) {
      if (conditionLower.includes(key)) {
        return value;
      }
    }
    
    return '🌡️'; // Default icon
  };

  // Check if conditions are safe for flying
  const isSafeToFly = (weatherData) => {
    if (!weatherData) return false;
    
    // Define safety thresholds
    const MAX_WIND_SPEED = 20; // mph
    const MAX_GUST_SPEED = 25; // mph
    const MIN_VISIBILITY = 3; // miles
    
    // Check conditions
    const windSpeed = weatherData.current.windSpeed;
    const gustSpeed = weatherData.current.windGust;
    const visibility = weatherData.current.visibility;
    const condition = weatherData.current.condition.text.toLowerCase();
    
    // Unsafe conditions
    const unsafeConditions = [
      'thunderstorm', 'lightning', 'storm', 'heavy rain', 'snow', 'sleet', 'hail', 'fog'
    ];
    
    // Check if any unsafe conditions are present
    const hasUnsafeCondition = unsafeConditions.some(c => condition.includes(c));
    
    // Return safety assessment
    return (
      windSpeed <= MAX_WIND_SPEED &&
      gustSpeed <= MAX_GUST_SPEED &&
      visibility >= MIN_VISIBILITY &&
      !hasUnsafeCondition
    );
  };

  return (
    <div className="widget weather-widget">
      <div className="widget-header">
        <h3>{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            ✕
          </button>
        )}
      </div>
      
      <div className="widget-content">
        <div className="weather-controls">
          <select 
            value={selectedLocation || ''} 
            onChange={handleLocationChange}
            className="location-select"
          >
            <option value="" disabled>Select location</option>
            {locations.map(location => (
              <option key={location.id} value={location.id}>
                {location.name}
              </option>
            ))}
          </select>
        </div>
        
        {isLoading ? (
          <div className="widget-loading">Loading weather data...</div>
        ) : error ? (
          <div className="widget-error">{error}</div>
        ) : !weatherData ? (
          <div className="widget-empty">No weather data available</div>
        ) : (
          <div className="weather-display">
            <div className="current-weather">
              <div className="weather-header">
                <div className="weather-location">
                  <h4>{weatherData.location.name}</h4>
                  <span className="weather-updated">
                    Updated: {formatTime(weatherData.current.lastUpdated)}
                  </span>
                </div>
                
                <div className="flight-safety">
                  <span className={`safety-indicator ${isSafeToFly(weatherData) ? 'safe' : 'unsafe'}`}>
                    {isSafeToFly(weatherData) ? 'Safe to fly' : 'Unsafe to fly'}
                  </span>
                </div>
              </div>
              
              <div className="weather-main">
                <div className="weather-condition">
                  <span className="weather-icon">
                    {getWeatherIcon(weatherData.current.condition.text)}
                  </span>
                  <span className="condition-text">
                    {weatherData.current.condition.text}
                  </span>
                </div>
                
                <div className="weather-temp">
                  <span className="temp-value">{Math.round(weatherData.current.tempF)}°F</span>
                  <span className="feels-like">
                    Feels like {Math.round(weatherData.current.feelsLikeF)}°F
                  </span>
                </div>
              </div>
              
              <div className="weather-details">
                <div className="detail-item">
                  <span className="detail-label">Wind:</span>
                  <span className="detail-value">
                    {weatherData.current.windDirection} {weatherData.current.windSpeed} mph
                    {weatherData.current.windGust > 0 && ` (Gusts: ${weatherData.current.windGust} mph)`}
                  </span>
                </div>
                <div className="detail-item">
                  <span className="detail-label">Humidity:</span>
                  <span className="detail-value">{weatherData.current.humidity}%</span>
                </div>
                <div className="detail-item">
                  <span className="detail-label">Visibility:</span>
                  <span className="detail-value">{weatherData.current.visibility} miles</span>
                </div>
                <div className="detail-item">
                  <span className="detail-label">Pressure:</span>
                  <span className="detail-value">{weatherData.current.pressure} inHg</span>
                </div>
                <div className="detail-item">
                  <span className="detail-label">UV Index:</span>
                  <span className="detail-value">{weatherData.current.uvIndex}</span>
                </div>
              </div>
            </div>
            
            <div className="weather-forecast">
              <h5>Forecast</h5>
              <div className="forecast-items">
                {weatherData.forecast.map((item, index) => (
                  <div key={index} className="forecast-item">
                    <div className="forecast-day">{formatDate(item.date)}</div>
                    <div className="forecast-icon">
                      {getWeatherIcon(item.condition.text)}
                    </div>
                    <div className="forecast-temp">
                      <span className="high-temp">{Math.round(item.maxTempF)}°</span>
                      <span className="low-temp">{Math.round(item.minTempF)}°</span>
                    </div>
                    <div className="forecast-wind">
                      {item.maxWindSpeed} mph
                    </div>
                  </div>
                ))}
              </div>
            </div>
            
            {weatherData.alerts.length > 0 && (
              <div className="weather-alerts">
                <h5>Weather Alerts</h5>
                <ul className="alert-list">
                  {weatherData.alerts.map((alert, index) => (
                    <li key={index} className="alert-item">
                      <div className="alert-title">{alert.title}</div>
                      <div className="alert-time">
                        {formatDate(alert.effective)} - {formatDate(alert.expires)}
                      </div>
                      <div className="alert-description">{alert.description}</div>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default WeatherWidget;
