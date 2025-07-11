# Environmental Sensor Dashboard

A comprehensive React dashboard for monitoring environmental sensors in real-time, designed for robotics and IoT applications.

## Overview

The Environmental Sensor Dashboard provides real-time monitoring and visualization of multiple environmental sensors:

- **SHT30 Temperature and Humidity Sensor** - Temperature/Humidity monitoring
- **MQ-4 Methane Gas Detection** - Methane level monitoring  
- **MICS-4514 MEMS CO/NO2 Sensor** - Carbon Monoxide & Nitrogen Dioxide levels
- **DSM501B PM2.5 Dust Sensor** - Air Quality/Dust level monitoring

## Features

### 🌍 Air Quality Overview
- **Air Quality Index (AQI)** calculation and display
- Color-coded air quality levels (Good, Moderate, Unhealthy, etc.)
- Real-time AQI circle indicator with pulsing animations
- Descriptive air quality status messages

### 🌡️ Temperature & Humidity Monitoring
- **SHT30 sensor** data visualization
- Temperature range: -40°C to +125°C
- Humidity range: 0-100% RH
- Color-coded status indicators
- Real-time bar charts

### ⚠️ Gas Detection
- **Methane (MQ-4)**: 0-10,000 ppm range
- **Carbon Monoxide (MICS-4514)**: 0-1000 ppm range  
- **Nitrogen Dioxide (MICS-4514)**: 0-10 ppm range
- Safety threshold indicators
- Critical level warnings with animations

### 🌪️ Particulate Matter Monitoring
- **PM2.5**: Fine particulate matter (≤2.5 μm)
- **PM10**: Coarse particulate matter (≤10 μm)
- **Total Dust Density**: Combined measurement
- EPA-based AQI calculations
- Health impact indicators

### 📊 Environmental Status Summary
- Overall environmental health assessment
- Temperature, humidity, gas safety, and air quality status
- Color-coded status indicators
- Real-time status updates

## Technical Specifications

### Sensor Integration

#### SHT30 Temperature & Humidity Sensor
```
Topics: /environmental/temperature, /environmental/humidity
Message Type: std_msgs/Float32
Update Rate: 2 Hz
Accuracy: ±0.2°C, ±2% RH
```

#### MQ-4 Methane Gas Sensor
```
Topic: /environmental/methane
Message Type: std_msgs/Float32
Update Rate: 2 Hz
Detection Range: 200-10,000 ppm
Response Time: <10s
```

#### MICS-4514 CO/NO2 Sensor
```
Topics: /environmental/carbon_monoxide, /environmental/nitrogen_dioxide
Message Type: std_msgs/Float32
Update Rate: 2 Hz
CO Range: 1-1000 ppm
NO2 Range: 0.05-10 ppm
```

#### DSM501B PM2.5/PM10 Dust Sensor
```
Topics: /environmental/pm25, /environmental/pm10
Message Type: std_msgs/Float32
Update Rate: 2 Hz
Particle Size: 0.5-10 μm
Concentration Range: 0-500 μg/m³
```

### Air Quality Index Calculation

The dashboard implements EPA's AQI calculation based on PM2.5 levels:

- **Good (0-50)**: Air quality is satisfactory
- **Moderate (51-100)**: Air quality is acceptable  
- **Unhealthy for Sensitive Groups (101-150)**: Sensitive groups may experience health effects
- **Unhealthy (151-200)**: Everyone may experience health effects
- **Very Unhealthy (201-300)**: Health alert conditions
- **Hazardous (301+)**: Emergency conditions

## Safety Thresholds

### Gas Safety Limits
- **Methane**: Safe < 50 ppm, Warning > 50 ppm
- **Carbon Monoxide**: Safe < 9 ppm, Warning > 9 ppm
- **Nitrogen Dioxide**: Safe < 0.053 ppm, Warning > 0.053 ppm

### Particulate Matter Limits
- **PM2.5**: Good < 12 μg/m³, Moderate 12-35.4 μg/m³
- **PM10**: Good < 54 μg/m³, Moderate 54-154 μg/m³

### Temperature & Humidity Ranges
- **Temperature**: Optimal 20-25°C, Cold < 10°C, Hot > 35°C
- **Humidity**: Optimal 30-60%, Dry < 30%, Humid > 80%

## Dashboard Features

### Visual Design
- **Futuristic HUD-style interface** with glowing elements
- **Color-coded sensors** for easy identification
- **Animated backgrounds** with particle effects
- **Responsive grid layout** for different screen sizes
- **Real-time data animations** and transitions

### Interactive Elements
- **Hover effects** on sensor cards
- **Test data mode** for development and testing
- **Real-time connection status** indicator
- **Expandable sensor details** with safety information

### Performance Optimizations
- **Efficient React hooks** for state management
- **Optimized re-rendering** with proper dependencies
- **Smooth animations** using CSS transitions
- **Responsive design** for mobile and desktop

## Usage

### Accessing the Dashboard
Navigate to: `http://localhost:3000/environmental-sensors`

### Test Data Mode
The dashboard includes a built-in test data generator:
1. Click "🧪 Use Test Data" button
2. Realistic sensor data will be generated automatically
3. Click "⏹️ Stop Test Data" to return to ROS mode

### ROS Integration
The dashboard automatically connects to ROS topics via rosbridge:
- Ensure rosbridge_server is running
- Topics are subscribed automatically on connection
- Real-time data updates every 2-3 seconds

## Development

### File Structure
```
src/components/
├── EnvironmentalSensorDashboard.tsx    # Main dashboard component
├── EnvironmentalSensorDashboard.css    # Styling and animations
test_publishers/
├── environmental_sensor_test_publisher.py  # Test data publisher
```

### Key Components

#### Data Interface
```typescript
interface SensorData {
  timestamp: number;
  temperature: number;
  humidity: number;
  methaneLevel: number;
  methanePPM: number;
  carbonMonoxide: number;
  nitrogenDioxide: number;
  coPPM: number;
  no2PPM: number;
  pm25: number;
  pm10: number;
  airQualityIndex: number;
  dustDensity: number;
}
```

#### Color Coding Functions
- `getAirQualityLevel()`: AQI level and color determination
- `getTemperatureColor()`: Temperature-based color coding
- `getHumidityColor()`: Humidity-based color coding  
- `getGasLevelColor()`: Gas concentration color coding

### Testing

#### Running Test Publisher
```bash
# Individual environmental sensor publisher
python3 test_publishers/environmental_sensor_test_publisher.py

# All publishers including environmental
python3 test_publishers/run_all_test_publishers.py

# Only environmental sensors
python3 test_publishers/run_all_test_publishers.py --publishers environmental
```

#### Test Data Features
- **Realistic sensor variations** with noise and trends
- **Occasional gas spikes** to test warning systems
- **Weather-based PM variations** for realistic air quality
- **Temperature-humidity correlation** for authentic data
- **AQI calculation** based on PM2.5 levels

## Troubleshooting

### Common Issues

#### No Data Displayed
1. Check ROS bridge connection: `ros2 topic list`
2. Verify environmental topics are published
3. Check browser console for connection errors
4. Use test data mode to verify dashboard functionality

#### Incorrect Sensor Readings
1. Verify sensor calibration
2. Check topic message types match expected format
3. Monitor ROS topics: `ros2 topic echo /environmental/temperature`
4. Validate sensor wiring and power supply

#### Performance Issues
1. Reduce update frequency in test publisher
2. Check browser performance in developer tools
3. Ensure adequate system resources
4. Consider reducing animation complexity

### Debug Mode
Enable debug logging by opening browser developer console:
- Connection status messages
- Received sensor data
- AQI calculations
- Error messages and warnings

## Future Enhancements

### Planned Features
- **Historical data logging** and trend analysis
- **Alert system** with email/SMS notifications
- **Sensor calibration interface** for field adjustments
- **Data export functionality** (CSV, JSON)
- **Multi-location monitoring** for distributed sensors
- **Machine learning predictions** for air quality trends

### Additional Sensors
- **UV Index sensor** for solar radiation monitoring
- **Noise level sensor** for acoustic monitoring
- **Atmospheric pressure sensor** for weather prediction
- **Soil moisture sensors** for agricultural applications
- **Water quality sensors** for environmental monitoring

## Contributing

When adding new environmental sensors:

1. **Update the SensorData interface** with new fields
2. **Add ROS topic subscriptions** in useEffect
3. **Create sensor cards** with appropriate styling
4. **Implement color coding functions** for the new sensor
5. **Update test data generator** with realistic values
6. **Add safety thresholds** and warning systems
7. **Update documentation** with sensor specifications

## License

This environmental sensor dashboard is part of the ROS2 3D Point Cloud Visualization project and follows the same licensing terms.
