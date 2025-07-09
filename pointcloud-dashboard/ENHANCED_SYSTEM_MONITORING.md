# Enhanced Brain System Monitoring Dashboard

## Overview

The Enhanced Brain System Monitoring Dashboard is a modern, real-time monitoring platform designed specifically for tracking the performance and health of Cognition and Perception Brain systems in ROS2 environments. It provides advanced visualizations, real-time charts, and comprehensive system analytics.

## Features

### 🎯 Key Capabilities

- **Real-time Monitoring**: Live data streaming from both Cognition and Perception Brain systems
- **Advanced Charts**: Interactive line charts, bar charts, and doughnut charts using Chart.js
- **Dual Brain Support**: Separate monitoring for Cognition Brain and Perception Brain systems
- **Historical Data**: Time-series data visualization with configurable time ranges
- **Health Assessment**: Intelligent system health evaluation with color-coded indicators
- **Modern UI**: Glassmorphism design with smooth animations and responsive layout

### 📊 Monitoring Metrics

#### System Performance
- **CPU Usage**: Real-time CPU utilization percentage
- **Memory Usage**: RAM consumption monitoring
- **Disk Usage**: Storage utilization tracking
- **Temperature**: System temperature monitoring with thermal alerts

#### Network & Process Information
- **Network I/O**: Upload/download bandwidth monitoring
- **Process Count**: Active process tracking
- **Load Average**: System load metrics (1min, 5min, 15min)
- **Uptime**: System uptime tracking

### 🎨 Visual Components

#### Status Cards
- System Health overview with color-coded status
- Active Brains counter
- Average CPU usage across all systems
- Maximum temperature monitoring

#### Interactive Charts
1. **Real-time CPU Usage**: Line chart showing CPU trends over time
2. **Memory Usage Trends**: Area chart displaying memory consumption
3. **Temperature Monitoring**: Line chart with thermal data
4. **Current System Metrics**: Bar chart comparing current values
5. **CPU Distribution**: Doughnut chart showing CPU usage distribution
6. **Performance Overview**: Visual health indicator

#### Brain Detail Cards
- Individual brain system status
- Comprehensive metrics grid
- Real-time progress bars
- Offline detection and alerts

## Technical Implementation

### ROS2 Integration

The dashboard subscribes to the following ROS topics:
- `/cognitionbrain/system_monitoring` - Cognition Brain system data
- `/perceptionbrain/system_monitoring` - Perception Brain system data

### Message Format

The system expects JSON-formatted messages with the following structure:

```json
{
  "cpu_usage": 45.2,
  "memory_usage": 67.8,
  "disk_usage": 23.1,
  "temperature": 42.5,
  "network_rx": 1024000,
  "network_tx": 512000,
  "process_count": 156,
  "uptime": 86400,
  "load_average": [0.5, 0.7, 0.9]
}
```

### Chart.js Configuration

The dashboard uses Chart.js with the following registered components:
- CategoryScale, LinearScale
- PointElement, LineElement, BarElement
- Title, Tooltip, Legend
- ArcElement, Filler

### Data Management

- **Historical Data**: Maintains last 50 data points (approximately 4 minutes)
- **Update Frequency**: 5-second intervals for real-time updates
- **Offline Detection**: 10-second timeout for brain offline detection

## Usage

### Navigation

Access the Enhanced Brain System Monitoring Dashboard through:
- URL: `/enhanced-system-monitoring`
- Navigation Menu: "Enhanced Brain Monitor" (🧠 icon)

### Controls

#### Time Range Selector
- 1 Minute
- 5 Minutes (default)
- 15 Minutes
- 1 Hour

#### Focus Metric Selector
- CPU Usage
- Memory Usage
- Temperature
- Network Activity

### Health Status Indicators

#### System Health Colors
- 🟢 **Healthy**: All systems operating normally
- 🟡 **Warning**: Some metrics approaching thresholds
- 🔴 **Critical**: One or more systems require attention

#### Threshold Values
- **CPU Usage**: Good ≤50%, Warning ≤80%, Critical >80%
- **Memory Usage**: Good ≤70%, Warning ≤85%, Critical >85%
- **Temperature**: Good ≤60°C, Warning ≤80°C, Critical >80°C
- **Disk Usage**: Good ≤70%, Warning ≤90%, Critical >90%

## Styling and Design

### Modern Glassmorphism UI
- Semi-transparent backgrounds with blur effects
- Gradient borders and hover animations
- Smooth transitions and micro-interactions
- Dark theme optimized for monitoring environments

### Responsive Design
- Mobile-friendly layout
- Adaptive grid systems
- Flexible chart containers
- Touch-friendly controls

### Color Scheme
- **Primary**: #2196F3 (Blue) - Cognition Brain
- **Secondary**: #FF6B6B (Red) - Perception Brain
- **Success**: #00ff41 (Green) - Healthy status
- **Warning**: #ffaa00 (Orange) - Warning status
- **Error**: #ff4444 (Red) - Critical status

## Performance Considerations

### Optimization Features
- Efficient data structure management
- Automatic cleanup of historical data
- Optimized chart rendering
- Minimal re-renders with React hooks

### Memory Management
- Limited historical data retention
- Automatic garbage collection of old data points
- Efficient state updates

## Troubleshooting

### Common Issues

#### No Data Displayed
1. Check ROS bridge connection (ws://localhost:9090)
2. Verify topic names match exactly
3. Ensure message format is correct JSON
4. Check browser console for connection errors

#### Charts Not Updating
1. Verify WebSocket connection is active
2. Check if data is being received in console logs
3. Ensure Chart.js dependencies are loaded
4. Refresh the page to reset connections

#### Performance Issues
1. Reduce historical data retention period
2. Check for memory leaks in browser dev tools
3. Ensure adequate system resources
4. Consider reducing update frequency

### Debug Information

Enable debug logging by checking browser console for:
- "Enhanced System Dashboard: Connected to ROS bridge"
- "Enhanced System Dashboard: Received [brain] data"
- WebSocket connection status messages

## Future Enhancements

### Planned Features
- Export functionality for historical data
- Alert system with email/SMS notifications
- Custom threshold configuration
- Multi-robot support
- Performance benchmarking tools
- System diagnostics integration

### Extensibility
- Plugin architecture for custom metrics
- Configurable chart types
- Custom color themes
- Advanced filtering options

## Dependencies

### Required Packages
- `react-chartjs-2`: Chart.js React wrapper
- `chart.js`: Charting library
- `react-router-dom`: Navigation routing

### Browser Requirements
- Modern browser with WebSocket support
- JavaScript enabled
- Canvas API support for charts

## Contributing

When contributing to the Enhanced System Monitoring Dashboard:

1. Follow the existing code structure and naming conventions
2. Ensure responsive design compatibility
3. Test with both brain systems online and offline
4. Maintain performance optimization standards
5. Update documentation for new features

## License

This component is part of the ROS2 Dashboard project and follows the same licensing terms.
