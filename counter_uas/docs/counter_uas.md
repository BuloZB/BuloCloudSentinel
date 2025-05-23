# Counter-UAS / Intrusion Detection Module

## Overview

The Counter-UAS / Intrusion Detection module provides comprehensive capabilities for detecting, tracking, and reporting unauthorized drone activity in protected airspace. It combines RF direction finding using KerberosSDR with radar-based detection using Acconeer radar sensors to provide accurate and reliable detection of drones and other aerial intruders.

## System Architecture

The Counter-UAS module is designed as a modular, extensible system with the following components:

### Hardware Components

1. **KerberosSDR**
   - 4-channel coherent RTL-SDR array
   - Frequency range: 70MHz-1.7GHz
   - Direction finding using MUSIC algorithm
   - USB 2.0 connectivity

2. **Acconeer Radar**
   - A111 60GHz radar sensors
   - Range and velocity measurement
   - High-resolution distance measurement
   - Low power consumption

### Software Components

1. **Hardware Interfaces**
   - Abstract interfaces for hardware devices
   - Concrete implementations for KerberosSDR and Acconeer radar
   - Hardware manager for device coordination

2. **Signal Processing Pipeline**
   - Direction finding using GNU Radio and MUSIC algorithm
   - Radar processing for range and velocity estimation
   - Sensor fusion using Extended Kalman Filter (EKF)

3. **Event Management**
   - Standardized event schema for intrusion events
   - RabbitMQ integration for event distribution
   - Event classification based on confidence and threat level

4. **API**
   - RESTful API for configuration and monitoring
   - WebSocket for real-time updates
   - Authentication and authorization

5. **User Interface**
   - Heat-map visualization of detection probability
   - Real-time track list with threat classification
   - Alert management and acknowledgment

## Data Flow

The Counter-UAS module processes data in the following flow:

1. **Data Acquisition**
   - KerberosSDR captures RF signals in the specified frequency range
   - Acconeer radar captures range and velocity data
   - Raw data is buffered for processing

2. **Signal Processing**
   - RF signals are processed to estimate direction of arrival (DoA)
   - Radar data is processed to estimate range and velocity
   - Processed data is passed to the sensor fusion component

3. **Sensor Fusion**
   - Extended Kalman Filter combines RF and radar data
   - Unified target tracks are generated with position, heading, and confidence
   - Tracks are filtered based on confidence and consistency

4. **Event Generation**
   - Tracks that meet detection criteria generate intrusion events
   - Events are classified based on confidence and threat level
   - Events are published to the message bus

5. **User Interface**
   - Events are displayed in the user interface
   - Alerts are generated based on event severity
   - Users can acknowledge and resolve alerts

## Hardware Setup

### KerberosSDR Setup

1. **Hardware Requirements**
   - KerberosSDR device
   - USB 2.0 cable
   - Host computer with USB 2.0 port
   - Optional: External antennas for improved reception

2. **Software Requirements**
   - GNU Radio 3.8 or newer
   - gr-kerberos GNU Radio module
   - RTL-SDR driver

3. **Installation**
   ```bash
   # Install dependencies
   sudo apt-get install gnuradio gnuradio-dev librtlsdr-dev
   
   # Install gr-kerberos
   git clone https://github.com/rfjohnso/gr-kerberos
   cd gr-kerberos
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   sudo ldconfig
   ```

4. **Configuration**
   - Connect the KerberosSDR to the host computer
   - Set the center frequency to a common drone frequency (e.g., 915 MHz)
   - Set the sample rate to 2.4 MSPS
   - Set the gain to 30 dB (adjust as needed)
   - Set the reference channel to 0

### Acconeer Radar Setup

1. **Hardware Requirements**
   - Acconeer A111 radar sensor
   - Evaluation board or custom PCB
   - USB cable
   - Host computer with USB port

2. **Software Requirements**
   - Python 3.7 or newer
   - Acconeer Python SDK

3. **Installation**
   ```bash
   # Install dependencies
   pip install acconeer-python-sdk
   
   # Install additional dependencies
   pip install numpy scipy matplotlib
   ```

4. **Configuration**
   - Connect the Acconeer radar to the host computer
   - Set the mode to IQ data
   - Set the range to 0.2-5.0 meters
   - Set the update rate to 10 Hz

## Performance Tuning

### KerberosSDR Tuning

1. **Frequency Selection**
   - Common drone frequencies: 915 MHz, 2.4 GHz, 5.8 GHz
   - Select the frequency based on the expected drone types
   - Use a spectrum analyzer to identify active frequencies

2. **Gain Adjustment**
   - Start with a gain of 30 dB
   - Increase gain if signal is weak
   - Decrease gain if signal is saturated
   - Monitor the signal level to ensure optimal reception

3. **Antenna Placement**
   - Place antennas in a square or circular arrangement
   - Maintain equal spacing between antennas
   - Ensure clear line of sight to the expected detection area
   - Avoid metal objects near the antennas

### Acconeer Radar Tuning

1. **Mode Selection**
   - IQ mode: Best for detecting moving objects
   - Envelope mode: Best for detecting stationary objects
   - Power bins mode: Best for detecting multiple objects
   - Sparse mode: Best for low-power operation

2. **Range Adjustment**
   - Set the start range to the minimum expected distance
   - Set the end range to the maximum expected distance
   - Adjust based on the detection environment
   - Consider the radar's maximum range capability

3. **Update Rate Adjustment**
   - Higher update rate: Better tracking of fast-moving objects
   - Lower update rate: Lower power consumption
   - Balance update rate with processing capabilities
   - Consider the expected drone speed

### Sensor Fusion Tuning

1. **Process Noise Adjustment**
   - Higher process noise: More responsive to changes
   - Lower process noise: More stable tracking
   - Adjust based on the expected drone maneuverability
   - Monitor track stability to ensure optimal performance

2. **Measurement Noise Adjustment**
   - Higher measurement noise: More filtering of noisy measurements
   - Lower measurement noise: More responsive to measurements
   - Adjust based on the sensor accuracy
   - Monitor track accuracy to ensure optimal performance

## Troubleshooting

### KerberosSDR Issues

1. **No Signal**
   - Check USB connection
   - Check antenna connections
   - Verify frequency setting
   - Increase gain
   - Check for interference sources

2. **Poor Direction Finding**
   - Check antenna placement
   - Verify coherent mode is enabled
   - Adjust reference channel
   - Increase signal strength
   - Reduce interference

### Acconeer Radar Issues

1. **No Detection**
   - Check USB connection
   - Verify range settings
   - Adjust mode
   - Increase update rate
   - Check for interference sources

2. **False Detections**
   - Adjust range settings
   - Change mode
   - Decrease update rate
   - Increase minimum confidence
   - Filter out static objects

### Sensor Fusion Issues

1. **Unstable Tracks**
   - Increase process noise
   - Decrease measurement noise
   - Increase minimum detections
   - Decrease maximum age
   - Improve sensor calibration

2. **Missed Detections**
   - Decrease process noise
   - Increase measurement noise
   - Decrease minimum detections
   - Increase maximum age
   - Improve sensor coverage

## Security Considerations

1. **Physical Security**
   - Secure hardware devices to prevent tampering
   - Protect USB connections to prevent unauthorized access
   - Secure the host computer to prevent unauthorized access
   - Implement physical access controls

2. **Network Security**
   - Use encrypted communications for all network traffic
   - Implement authentication and authorization for API access
   - Secure RabbitMQ with authentication and encryption
   - Monitor network traffic for suspicious activity

3. **Data Security**
   - Encrypt sensitive data at rest and in transit
   - Implement access controls for event data
   - Secure configuration files with proper permissions
   - Implement audit logging for security events

4. **Operational Security**
   - Implement secure coding practices
   - Regularly update software dependencies
   - Conduct security assessments
   - Develop and test incident response procedures
