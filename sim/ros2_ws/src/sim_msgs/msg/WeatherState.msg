# Weather state message

# Weather type (clear, cloudy, rainy, foggy, stormy)
string weather_type

# Wind parameters
float64 wind_speed        # m/s
float64 wind_direction    # degrees
float64 wind_gust_factor  # multiplier for gusts
float64 wind_turbulence_intensity  # 0-1 scale

# Precipitation parameters
float64 rain_intensity    # 0-1 scale

# Visibility parameters
float64 fog_density       # 0-1 scale

# Time parameters
float64 time_of_day       # hours (0-24)

# GPS effects
float64 gps_multipath_factor  # 0-1 scale
