# Battery Management System Documentation

## Overview

The Battery Management System (BMS) provides comprehensive monitoring and visualization of robot battery status through both a dedicated dashboard page and an integrated widget system. The system connects to ROS2 topics to display real-time battery information with an attractive robotics-focused design.

## Features

### 🔋 Battery Dashboard Page
- **Full-screen battery monitoring interface**
- **Real-time data visualization**
- **Comprehensive battery health metrics**
- **Individual cell voltage monitoring**
- **Temperature monitoring with multiple sensors**
- **System status indicators**
- **Estimated runtime calculations**
- **Attractive robotics-themed design**

### 🔧 Battery Widget
- **Compact battery status widget for the Enhanced PointCloud page**
- **Draggable and resizable interface**
- **Real-time battery data updates**
- **Compact and full display modes**
- **Integration with existing widget system**

## ROS2 Topics

The battery system subscribes to the following ROS2 topics:

```
/battery/remaining_capacity    - Remaining battery capacity (Ah)
/battery/state                - Battery state information
/battery/status               - Battery status string
/battery/pack_voltage         - Total pack voltage (V)
/battery/cycles               - Number of charge cycles
/battery/current              - Current draw/charge (A)
/battery/cell_voltages        - Individual cell voltages array
/battery/temperature          - Temperature sensor readings
/battery/total_capacity       - Total battery capacity (Ah)
/battery/soc                  - State of Charge percentage
```

## Data Structure

The system expects battery data in the following format:

```typescript
interface BatteryData {
  timestamp: number;
  packVoltage: number;          // Total pack voltage in volts
  current: number;              // Current in amperes (+ charging, - discharging)
  soc: number;                  // State of Charge (0-100%)
  remainingCapacity: number;    // Remaining capacity in Ah
  totalCapacity: number;        // Total capacity in Ah
  cycles: number;               // Number of charge cycles
  temperature: number[];        // Array of temperature readings in °C
  cellVoltages: Array<{         // Individual cell voltages
    cellNumber: number;
    voltage: number;
  }>;
  status: string;               // Battery status string
  chargingMos: boolean;         // Charging MOSFET status
  dischargingMos: boolean;      // Discharging MOSFET status
  balancing: boolean;           // Cell balancing status
}
```

## Components

### BatteryDashboard.tsx
Main battery monitoring page component featuring:
- **Visual battery indicator** with animated fill based on SOC
- **Main status cards** showing key metrics
- **Capacity information** with progress bars
- **Temperature monitoring** with color-coded alerts
- **Individual cell voltage display** with health indicators
- **System status indicators** for MOS switches and balancing
- **Runtime estimation** based on current consumption

### BatteryWidget.tsx
Compact widget component for integration with the Enhanced PointCloud page:
- **Compact mode** for minimal space usage
- **Full mode** for detailed information
- **Real-time data updates**
- **Visual battery indicator**
- **Key metrics display**
- **Temperature and current monitoring**

### CSS Styling
- **BatteryDashboard.css** - Full dashboard styling with robotics theme
- **BatteryWidget.css** - Widget-specific styling with responsive design

## Color Coding System

### Battery Health (SOC)
- **Green (#00ff41)**: > 70% - Healthy
- **Orange (#ffaa00)**: 30-70% - Moderate
- **Red (#ff4444)**: < 30% - Low

### Temperature Status
- **Blue (#00aaff)**: < 25°C - Cool
- **Green (#00ff41)**: 25-40°C - Normal
- **Orange (#ffaa00)**: 40-60°C - Warm
- **Red (#ff4444)**: > 60°C - Hot

### Cell Voltage Health
- **Green (#00ff41)**: > 3.3V - Healthy
- **Orange (#ffaa00)**: 3.0-3.3V - Moderate
- **Red (#ff4444)**: < 3.0V - Low

## Navigation Integration

The battery system is integrated into the main navigation:
- **Route**: `/battery`
- **Navigation Icon**: 🔋
- **Menu Item**: "Battery Monitor"

## Widget Integration

The battery widget is integrated into the Enhanced PointCloud page widget system:
- **Widget ID**: `battery-monitor`
- **Default Position**: Bottom-right corner
- **Resizable**: Yes (300-500px width, 400-700px height)
- **Collapsible**: Yes
- **Default Visibility**: Hidden (can be toggled on)

## Usage

### Accessing the Battery Dashboard
1. Navigate to the main dashboard
2. Click on "Battery Monitor" in the navigation menu
3. View comprehensive battery information

### Using the Battery Widget
1. Go to the Enhanced Dashboard page
2. Open the Widget Manager (gear icon)
3. Toggle on "Battery Monitor" widget
4. Drag and resize as needed
5. Monitor battery status while viewing point cloud data

## Technical Implementation

### ROS Connection
- Uses the same ROS bridge connection as other components
- Subscribes to multiple battery topics simultaneously
- Handles connection status and error states
- Automatic topic discovery and fallback

### Data Processing
- Real-time data aggregation from multiple topics
- Data validation and error handling
- Timestamp tracking for data freshness
- Automatic unit conversions and calculations

### Performance Optimizations
- Efficient React state management
- Optimized re-rendering with useCallback and useMemo
- Responsive design for various screen sizes
- Smooth animations and transitions

## Customization

### Adding New Metrics
1. Update the `BatteryData` interface
2. Add new topic subscription in the useEffect
3. Update the UI components to display new data
4. Add appropriate color coding and thresholds

### Styling Modifications
- Modify CSS files for visual changes
- Update color schemes in the component logic
- Adjust responsive breakpoints as needed
- Customize animations and transitions

## Troubleshooting

### Common Issues

1. **No Battery Data Displayed**
   - Check ROS bridge connection
   - Verify battery topics are publishing
   - Check topic names match expected format

2. **Widget Not Appearing**
   - Ensure widget is toggled on in Widget Manager
   - Check for JavaScript console errors
   - Verify component imports are correct

3. **Incorrect Data Values**
   - Verify topic message types match expected format
   - Check data parsing logic in component
   - Validate ROS topic data with `ros2 topic echo`

### Debug Commands

```bash
# Check available battery topics
ros2 topic list | grep battery

# Monitor battery data
ros2 topic echo /battery/soc
ros2 topic echo /battery/pack_voltage

# Check topic types
ros2 topic info /battery/soc
```

## Future Enhancements

### Planned Features
- **Historical data logging** and trend analysis
- **Battery health predictions** based on usage patterns
- **Configurable alerts** for critical battery states
- **Export functionality** for battery data
- **Multiple battery support** for robots with multiple packs
- **Charging station integration** and status monitoring

### Performance Improvements
- **Data caching** for improved responsiveness
- **WebSocket optimization** for real-time updates
- **Progressive loading** for large datasets
- **Mobile optimization** for tablet/phone access

## Dependencies

- React 18+
- TypeScript
- ROS Bridge (rosbridge_suite)
- Three.js (for 3D integration)
- CSS3 (for animations and styling)

## Browser Compatibility

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Security Considerations

- ROS bridge connection security
- Data validation and sanitization
- Error boundary implementation
- Safe rendering of dynamic content
