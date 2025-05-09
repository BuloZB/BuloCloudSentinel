import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, Typography, Box, CircularProgress, Chip, Grid, Button } from '@mui/material';
import { WiDaySunny, WiCloudy, WiRain, WiSnow, WiThunderstorm, WiFog, WiWindy } from 'react-icons/wi';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import axios from 'axios';

const weatherIcons = {
  clear: <WiDaySunny size={32} />,
  partly_cloudy: <WiCloudy size={32} />,
  cloudy: <WiCloudy size={32} />,
  overcast: <WiCloudy size={32} />,
  fog: <WiFog size={32} />,
  light_rain: <WiRain size={32} />,
  rain: <WiRain size={32} />,
  heavy_rain: <WiRain size={32} />,
  thunderstorm: <WiThunderstorm size={32} />,
  snow: <WiSnow size={32} />,
  sleet: <WiSnow size={32} />,
  hail: <WiSnow size={32} />,
  windy: <WiWindy size={32} />,
  unknown: <WiDaySunny size={32} />,
};

const severityColors = {
  none: 'success',
  low: 'success',
  medium: 'warning',
  high: 'error',
  extreme: 'error',
};

const WeatherCard = ({ latitude, longitude, apiUrl = '/api/weather' }) => {
  const [currentWeather, setCurrentWeather] = useState(null);
  const [forecast, setForecast] = useState([]);
  const [missionCheck, setMissionCheck] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const fetchWeatherData = async () => {
      try {
        setLoading(true);
        setError(null);

        // Fetch current weather
        const currentResponse = await axios.get(`${apiUrl}/current`, {
          params: { latitude, longitude }
        });
        
        if (currentResponse.data.success) {
          setCurrentWeather(currentResponse.data.data);
        } else {
          throw new Error(currentResponse.data.error || 'Failed to fetch current weather');
        }

        // Fetch forecast
        const forecastResponse = await axios.get(`${apiUrl}/forecast`, {
          params: { latitude, longitude, hours: 24 }
        });
        
        if (forecastResponse.data.success) {
          setForecast(forecastResponse.data.data);
        } else {
          throw new Error(forecastResponse.data.error || 'Failed to fetch forecast');
        }

        // Check mission weather for next 3 hours
        const startTime = new Date();
        startTime.setHours(startTime.getHours() + 1);
        
        const endTime = new Date(startTime);
        endTime.setHours(endTime.getHours() + 2);

        const missionResponse = await axios.get(`${apiUrl}/check-mission`, {
          params: {
            latitude,
            longitude,
            start_time: startTime.toISOString(),
            end_time: endTime.toISOString(),
          }
        });
        
        if (missionResponse.data.success) {
          setMissionCheck(missionResponse.data.data);
        } else {
          throw new Error(missionResponse.data.error || 'Failed to check mission weather');
        }

        setLoading(false);
      } catch (err) {
        console.error('Error fetching weather data:', err);
        setError(err.message);
        setLoading(false);
      }
    };

    fetchWeatherData();
    
    // Refresh every 30 minutes
    const interval = setInterval(fetchWeatherData, 30 * 60 * 1000);
    
    return () => clearInterval(interval);
  }, [latitude, longitude, apiUrl]);

  // Prepare chart data
  const chartData = forecast.map(item => ({
    time: new Date(item.forecast_time).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
    temperature: item.temperature,
    windSpeed: item.wind_speed,
    precipitation: item.precipitation * 10, // Scale up for visibility
  }));

  if (loading) {
    return (
      <Card>
        <CardContent sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: 400 }}>
          <CircularProgress />
        </CardContent>
      </Card>
    );
  }

  if (error) {
    return (
      <Card>
        <CardContent sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: 400 }}>
          <Typography color="error">Error: {error}</Typography>
        </CardContent>
      </Card>
    );
  }

  return (
    <Card>
      <CardHeader 
        title="Weather Conditions" 
        subheader={`${latitude.toFixed(4)}, ${longitude.toFixed(4)}`}
        action={
          missionCheck && (
            <Chip 
              label={missionCheck.is_flyable ? "GO" : "NO-GO"} 
              color={missionCheck.is_flyable ? "success" : "error"}
              sx={{ fontWeight: 'bold', fontSize: '1rem', mr: 2 }}
            />
          )
        }
      />
      <CardContent>
        {currentWeather && (
          <Grid container spacing={2} sx={{ mb: 3 }}>
            <Grid item xs={12} md={6}>
              <Box sx={{ display: 'flex', alignItems: 'center' }}>
                {weatherIcons[currentWeather.condition]}
                <Typography variant="h4" sx={{ ml: 1 }}>
                  {currentWeather.temperature.toFixed(1)}°C
                </Typography>
              </Box>
              <Typography variant="body1">
                {currentWeather.condition.replace('_', ' ')}
              </Typography>
            </Grid>
            <Grid item xs={12} md={6}>
              <Typography variant="body1">
                Wind: {currentWeather.wind_speed.toFixed(1)} m/s
                {currentWeather.wind_direction && ` from ${currentWeather.wind_direction}°`}
              </Typography>
              <Typography variant="body1">
                Precipitation: {currentWeather.precipitation.toFixed(1)} mm/h
              </Typography>
              {currentWeather.humidity && (
                <Typography variant="body1">
                  Humidity: {currentWeather.humidity.toFixed(0)}%
                </Typography>
              )}
            </Grid>
          </Grid>
        )}

        {forecast.length > 0 && (
          <Box sx={{ height: 200, mb: 3 }}>
            <Typography variant="h6" sx={{ mb: 1 }}>24-Hour Forecast</Typography>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={chartData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" />
                <YAxis yAxisId="left" />
                <YAxis yAxisId="right" orientation="right" />
                <Tooltip />
                <Legend />
                <Line yAxisId="left" type="monotone" dataKey="temperature" stroke="#8884d8" name="Temperature (°C)" />
                <Line yAxisId="left" type="monotone" dataKey="windSpeed" stroke="#82ca9d" name="Wind (m/s)" />
                <Line yAxisId="right" type="monotone" dataKey="precipitation" stroke="#ffc658" name="Rain (mm/h × 10)" />
              </LineChart>
            </ResponsiveContainer>
          </Box>
        )}

        {missionCheck && (
          <Box>
            <Typography variant="h6" sx={{ mb: 1 }}>Mission Weather Status</Typography>
            <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
              <Typography variant="body1" sx={{ mr: 1 }}>
                Severity:
              </Typography>
              <Chip 
                label={missionCheck.severity} 
                color={severityColors[missionCheck.severity] || 'default'}
                size="small"
              />
            </Box>
            
            {missionCheck.limitations.length > 0 && (
              <Box sx={{ mb: 2 }}>
                <Typography variant="body2" fontWeight="bold">Limitations:</Typography>
                {missionCheck.limitations.map((limitation, index) => (
                  <Typography key={index} variant="body2">
                    • {limitation.description}
                  </Typography>
                ))}
              </Box>
            )}
            
            {missionCheck.recommendations.length > 0 && (
              <Box>
                <Typography variant="body2" fontWeight="bold">Recommendations:</Typography>
                {missionCheck.recommendations.map((recommendation, index) => (
                  <Typography key={index} variant="body2">
                    • {recommendation}
                  </Typography>
                ))}
              </Box>
            )}
          </Box>
        )}
        
        <Box sx={{ display: 'flex', justifyContent: 'flex-end', mt: 2 }}>
          <Button size="small" onClick={() => window.open('/weather-dashboard', '_blank')}>
            Open Weather Dashboard
          </Button>
        </Box>
      </CardContent>
    </Card>
  );
};

export default WeatherCard;
