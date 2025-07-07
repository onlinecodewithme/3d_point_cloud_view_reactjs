import React, { useRef, useEffect, useState } from 'react';
import './BatteryWidget.css';

interface BatteryData {
  timestamp: number;
  packVoltage: number;
  current: number;
  soc: number; // State of Charge
  remainingCapacity: number;
  totalCapacity: number;
  temperature: number[];
  cellVoltages: { cellNumber: number; voltage: number }[];
  status: string;
  chargingMos: boolean;
  dischargingMos: boolean;
}

interface BatteryWidgetProps {
  ros?: any;
  isConnected?: boolean;
  compact?: boolean;
}

const BatteryWidget: React.FC<BatteryWidgetProps> = ({ 
  ros, 
  isConnected = false, 
  compact = false 
}) => {
  const [batteryData, setBatteryData] = useState<BatteryData | null>(null);
  const [connectionStatus, setConnectionStatus] = useState('Connecting...');
  const rosRef = useRef<any>(ros);

  useEffect(() => {
    rosRef.current = ros;
  }, [ros]);

  useEffect(() => {
    if (!isConnected || !rosRef.current) {
      setConnectionStatus('Disconnected');
      return;
    }

    const subscribeToBatteryTopics = (ros: any) => {
      let batteryDataBuffer: Partial<BatteryData> = {};
      let hasReceivedData = false;

      // Helper function to update battery data
      const updateBatteryData = () => {
        if (hasReceivedData) {
          setBatteryData(prev => ({
            ...prev,
            ...batteryDataBuffer,
            timestamp: Date.now()
          } as BatteryData));
          setConnectionStatus('Connected');
        }
      };

      // Subscribe to key battery topics with proper message types
      const socTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/soc',
        messageType: 'std_msgs/Float32'
      });
      socTopic.subscribe((message: any) => {
        console.log('Widget received SOC:', message.data);
        batteryDataBuffer.soc = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      const voltageTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/pack_voltage',
        messageType: 'std_msgs/Float32'
      });
      voltageTopic.subscribe((message: any) => {
        console.log('Widget received pack voltage:', message.data);
        batteryDataBuffer.packVoltage = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      const currentTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/current',
        messageType: 'std_msgs/Float32'
      });
      currentTopic.subscribe((message: any) => {
        console.log('Widget received current:', message.data);
        batteryDataBuffer.current = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      const remainingTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/remaining_capacity',
        messageType: 'std_msgs/Float32'
      });
      remainingTopic.subscribe((message: any) => {
        console.log('Widget received remaining capacity:', message.data);
        batteryDataBuffer.remainingCapacity = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      const totalTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/total_capacity',
        messageType: 'std_msgs/Float32'
      });
      totalTopic.subscribe((message: any) => {
        console.log('Widget received total capacity:', message.data);
        batteryDataBuffer.totalCapacity = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      const tempTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/temperature',
        messageType: 'std_msgs/Float32MultiArray'
      });
      tempTopic.subscribe((message: any) => {
        console.log('Widget received temperature:', message.data);
        batteryDataBuffer.temperature = Array.isArray(message.data) ? message.data : [message.data];
        hasReceivedData = true;
        updateBatteryData();
      });

      const statusTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/status',
        messageType: 'std_msgs/String'
      });
      statusTopic.subscribe((message: any) => {
        console.log('Widget received status:', message.data);
        batteryDataBuffer.status = message.data || 'Unknown';
        hasReceivedData = true;
        updateBatteryData();
      });

      const stateTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/state',
        messageType: 'sensor_msgs/BatteryState'
      });
      stateTopic.subscribe((message: any) => {
        console.log('Widget received battery state:', message);
        
        // Extract comprehensive battery data from BatteryState message
        batteryDataBuffer.packVoltage = message.voltage;
        batteryDataBuffer.current = message.current;
        batteryDataBuffer.remainingCapacity = message.charge;
        batteryDataBuffer.totalCapacity = message.capacity;
        batteryDataBuffer.soc = message.percentage * 100; // Convert from 0-1 to 0-100
        batteryDataBuffer.temperature = [message.temperature];
        
        // Extract individual cell voltages
        if (message.cell_voltage && Array.isArray(message.cell_voltage)) {
          batteryDataBuffer.cellVoltages = message.cell_voltage.map((voltage: number, index: number) => ({
            cellNumber: index + 1,
            voltage: voltage
          }));
        }
        
        // Map power supply status to readable status
        const statusMap: { [key: number]: string } = {
          0: 'Unknown',
          1: 'Charging',
          2: 'Discharging', 
          3: 'Not Charging',
          4: 'Full'
        };
        batteryDataBuffer.status = statusMap[message.power_supply_status] || 'Unknown';
        
        // Map power supply health for MOS status (approximation)
        const isHealthy = message.power_supply_health === 1; // POWER_SUPPLY_HEALTH_GOOD
        batteryDataBuffer.chargingMos = isHealthy;
        batteryDataBuffer.dischargingMos = isHealthy;
        
        hasReceivedData = true;
        updateBatteryData();
      });

      const cellTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/cell_voltages',
        messageType: 'std_msgs/Float32MultiArray'
      });
      cellTopic.subscribe((message: any) => {
        console.log('Widget received cell voltages:', message.data);
        if (Array.isArray(message.data)) {
          batteryDataBuffer.cellVoltages = message.data.map((voltage: number, index: number) => ({
            cellNumber: index + 1,
            voltage: voltage
          }));
        }
        hasReceivedData = true;
        updateBatteryData();
      });
    };

    subscribeToBatteryTopics(rosRef.current);
  }, [isConnected]);

  const getBatteryHealthColor = (soc: number) => {
    if (soc > 70) return '#00ff41';
    if (soc > 30) return '#ffaa00';
    return '#ff4444';
  };

  const getTemperatureColor = (temp: number) => {
    if (temp < 25) return '#00aaff';
    if (temp < 40) return '#00ff41';
    if (temp < 60) return '#ffaa00';
    return '#ff4444';
  };

  const getBatteryIcon = (soc: number) => {
    if (soc > 75) return '🔋';
    if (soc > 50) return '🔋';
    if (soc > 25) return '🪫';
    return '🪫';
  };

  const getStatusIcon = (status: string) => {
    switch (status?.toLowerCase()) {
      case 'charging': return '⚡';
      case 'discharging': return '🔋';
      case 'full': return '✅';
      case 'critical': return '⚠️';
      case 'not charging': return '🔌';
      case 'unknown': return '📊';
      default: return '📊'; // Changed from ❓ to 📊 (chart/status icon)
    }
  };

  if (compact) {
    return (
      <div className="battery-widget compact">
        <div className="battery-widget-header">
          <span className="battery-icon">{getBatteryIcon(batteryData?.soc || 0)}</span>
          <span className="battery-title">Battery</span>
          <span className={`connection-dot ${isConnected ? 'connected' : 'disconnected'}`}>●</span>
        </div>
        
        {batteryData ? (
          <div className="battery-compact-content">
            <div className="battery-main-stat">
              <div 
                className="battery-soc"
                style={{ color: getBatteryHealthColor(batteryData.soc) }}
              >
                {batteryData.soc?.toFixed(1)}%
              </div>
              <div className="battery-voltage">
                {batteryData.packVoltage?.toFixed(1)}V
              </div>
            </div>
            
            <div className="battery-mini-bar">
              <div 
                className="battery-mini-fill"
                style={{
                  width: `${batteryData.soc}%`,
                  backgroundColor: getBatteryHealthColor(batteryData.soc)
                }}
              />
            </div>
            
            <div className="battery-secondary-stats">
              <div className="stat-item">
                <span className="stat-label">Current:</span>
                <span className="stat-value">{batteryData.current?.toFixed(1)}A</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Temp:</span>
                <span 
                  className="stat-value"
                  style={{ color: getTemperatureColor(batteryData.temperature?.[0] || 0) }}
                >
                  {batteryData.temperature?.[0] || 0}°C
                </span>
              </div>
            </div>
          </div>
        ) : (
          <div className="battery-loading-compact">
            <div className="loading-dots">●●●</div>
            <div className="loading-text">Loading...</div>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className="battery-widget">
      <div className="battery-widget-header">
        <div className="battery-widget-title">
          <span className="battery-icon">{getBatteryIcon(batteryData?.soc || 0)}</span>
          <h3>Battery Status</h3>
        </div>
        <div className="battery-connection-status">
          <span className={`connection-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
            ●
          </span>
          <span className="connection-text">{connectionStatus}</span>
        </div>
      </div>

      {batteryData ? (
        <div className="battery-widget-content">
          {/* Main Battery Display */}
          <div className="battery-main-display">
            <div className="battery-visual-widget">
              <div className="battery-outline-widget">
                <div 
                  className="battery-fill-widget"
                  style={{
                    width: `${batteryData.soc}%`,
                    backgroundColor: getBatteryHealthColor(batteryData.soc)
                  }}
                />
                <div className="battery-percentage-widget">
                  {batteryData.soc?.toFixed(1)}%
                </div>
              </div>
              <div className="battery-terminal-widget" />
            </div>
            
            <div className="battery-main-stats">
              <div className="main-stat">
                <div className="stat-label">State of Charge</div>
                <div 
                  className="stat-value primary"
                  style={{ color: getBatteryHealthColor(batteryData.soc) }}
                >
                  {batteryData.soc?.toFixed(1)}%
                </div>
              </div>
              
              <div className="main-stat">
                <div className="stat-label">Pack Voltage</div>
                <div className="stat-value">{batteryData.packVoltage?.toFixed(2)}V</div>
              </div>
              
              <div className="main-stat">
                <div className="stat-label">Current</div>
                <div className="stat-value">{batteryData.current?.toFixed(2)}A</div>
              </div>
            </div>
          </div>

          {/* Secondary Stats */}
          <div className="battery-secondary-stats-widget">
            <div className="secondary-stat">
              <div className="stat-icon">🔋</div>
              <div className="stat-info">
                <div className="stat-label">Capacity</div>
                <div className="stat-value">
                  {batteryData.remainingCapacity?.toFixed(1)} / {batteryData.totalCapacity?.toFixed(1)} Ah
                </div>
              </div>
            </div>
            
            <div className="secondary-stat">
              <div className="stat-icon">🌡️</div>
              <div className="stat-info">
                <div className="stat-label">Temperature</div>
                <div 
                  className="stat-value"
                  style={{ color: getTemperatureColor(batteryData.temperature?.[0] || 0) }}
                >
                  {batteryData.temperature?.[0] || 0}°C
                </div>
              </div>
            </div>
            
            <div className="secondary-stat">
              <div className="stat-icon">{getStatusIcon(batteryData.status)}</div>
              <div className="stat-info">
                <div className="stat-label">Status</div>
                <div className="stat-value">{batteryData.status || 'Unknown'}</div>
              </div>
            </div>
          </div>

          {/* Runtime Estimate */}
          <div className="battery-runtime-widget">
            <div className="runtime-label">Estimated Runtime</div>
            <div className="runtime-value">
              {batteryData.current && batteryData.current > 0 ? (
                `${((batteryData.remainingCapacity || 0) / Math.abs(batteryData.current)).toFixed(1)}h`
              ) : (
                '∞'
              )}
            </div>
          </div>

          {/* Cell Voltage Summary */}
          {batteryData.cellVoltages && batteryData.cellVoltages.length > 0 && (
            <div className="battery-cells-summary">
              <div className="cells-label">Cell Voltages</div>
              <div className="cells-range">
                {Math.min(...batteryData.cellVoltages.map(c => c.voltage)).toFixed(3)}V - 
                {Math.max(...batteryData.cellVoltages.map(c => c.voltage)).toFixed(3)}V
              </div>
              <div className="cells-count">{batteryData.cellVoltages.length} cells</div>
            </div>
          )}

          {/* System Status Indicators */}
          <div className="battery-system-status">
            <div className={`status-item ${batteryData.chargingMos ? 'active' : 'inactive'}`}>
              <span className="status-icon">{batteryData.chargingMos ? '✅' : '❌'}</span>
              <span className="status-text">Charge MOS</span>
            </div>
            <div className={`status-item ${batteryData.dischargingMos ? 'active' : 'inactive'}`}>
              <span className="status-icon">{batteryData.dischargingMos ? '✅' : '❌'}</span>
              <span className="status-text">Discharge MOS</span>
            </div>
          </div>
        </div>
      ) : (
        <div className="battery-widget-loading">
          <div className="loading-spinner-widget"></div>
          <div className="loading-text-widget">Waiting for battery data...</div>
        </div>
      )}
    </div>
  );
};

export default BatteryWidget;
