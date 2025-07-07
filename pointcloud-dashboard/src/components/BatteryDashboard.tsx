import React, { useRef, useEffect, useState } from 'react';
import './BatteryDashboard.css';

interface BatteryData {
  timestamp: number;
  packVoltage: number;
  current: number;
  soc: number; // State of Charge
  remainingCapacity: number;
  totalCapacity: number;
  cycles: number;
  temperature: number[];
  cellVoltages: { cellNumber: number; voltage: number }[];
  status: string;
  chargingMos: boolean;
  dischargingMos: boolean;
  balancing: boolean;
}

const BatteryDashboard: React.FC = () => {
  const [batteryData, setBatteryData] = useState<BatteryData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [useTestData, setUseTestData] = useState(false);
  const rosRef = useRef<any>(null);

  // Generate mock battery data for testing
  const generateMockData = (): BatteryData => {
    const soc = 75 + Math.random() * 20; // 75-95%
    const packVoltage = 52 + Math.random() * 4; // 52-56V
    const current = -2 + Math.random() * 4; // -2 to 2A
    
    return {
      timestamp: Date.now(),
      packVoltage: packVoltage,
      current: current,
      soc: soc,
      remainingCapacity: 207.9,
      totalCapacity: 230,
      cycles: 1,
      temperature: [30 + Math.random() * 10, 32 + Math.random() * 8], // 30-40°C, 32-40°C
      cellVoltages: Array.from({ length: 16 }, (_, i) => ({
        cellNumber: i + 1,
        voltage: 3.3 + Math.random() * 0.1 // 3.3-3.4V
      })),
      status: current > 0 ? 'Charging' : current < -0.1 ? 'Discharging' : 'Idle',
      chargingMos: true,
      dischargingMos: true,
      balancing: Math.random() > 0.8
    };
  };

  const startTestData = () => {
    setUseTestData(true);
    setConnectionStatus('Using test data');
    
    // Generate initial data
    setBatteryData(generateMockData());
    
    // Update data every 2 seconds
    const interval = setInterval(() => {
      setBatteryData(generateMockData());
    }, 2000);
    
    return () => clearInterval(interval);
  };

  const stopTestData = () => {
    setUseTestData(false);
    setBatteryData(null);
    setConnectionStatus('Connecting to ROS...');
  };

  useEffect(() => {
    const initROS = async () => {
      try {
        // @ts-ignore
        const ROSLIB = window.ROSLIB;
        if (!ROSLIB) {
          setConnectionStatus('ROS library not loaded');
          return;
        }

        const ros = new ROSLIB.Ros({
          url: 'ws://localhost:9090'
        });

        rosRef.current = ros;

        ros.on('connection', () => {
          console.log('Connected to ROS bridge for battery data');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
          subscribeToBatteryTopics(ros);
        });

        ros.on('error', (error: any) => {
          console.error('ROS connection error:', error);
          setIsConnected(false);
          setConnectionStatus(`Connection error: ${error.message || 'Unknown error'}`);
        });

        ros.on('close', () => {
          console.log('ROS connection closed');
          setIsConnected(false);
          setConnectionStatus('Disconnected from ROS');
        });

      } catch (error) {
        console.error('Failed to initialize ROS:', error);
        setConnectionStatus('Failed to initialize ROS');
      }
    };

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
        }
      };

      // Subscribe to SOC
      const socTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/soc',
        messageType: 'std_msgs/Float32'
      });
      socTopic.subscribe((message: any) => {
        console.log('Received SOC:', message.data);
        batteryDataBuffer.soc = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to pack voltage
      const voltageTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/pack_voltage',
        messageType: 'std_msgs/Float32'
      });
      voltageTopic.subscribe((message: any) => {
        console.log('Received pack voltage:', message.data);
        batteryDataBuffer.packVoltage = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to current
      const currentTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/current',
        messageType: 'std_msgs/Float32'
      });
      currentTopic.subscribe((message: any) => {
        console.log('Received current:', message.data);
        batteryDataBuffer.current = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to remaining capacity
      const remainingTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/remaining_capacity',
        messageType: 'std_msgs/Float32'
      });
      remainingTopic.subscribe((message: any) => {
        console.log('Received remaining capacity:', message.data);
        batteryDataBuffer.remainingCapacity = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to total capacity
      const totalTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/total_capacity',
        messageType: 'std_msgs/Float32'
      });
      totalTopic.subscribe((message: any) => {
        console.log('Received total capacity:', message.data);
        batteryDataBuffer.totalCapacity = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to cycles
      const cyclesTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/cycles',
        messageType: 'std_msgs/Float32'
      });
      cyclesTopic.subscribe((message: any) => {
        console.log('Received cycles:', message.data);
        batteryDataBuffer.cycles = message.data;
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to temperature
      const tempTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/temperature',
        messageType: 'std_msgs/Float32MultiArray'
      });
      tempTopic.subscribe((message: any) => {
        console.log('Received temperature:', message.data);
        batteryDataBuffer.temperature = Array.isArray(message.data) ? message.data : [message.data];
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to status
      const statusTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/status',
        messageType: 'std_msgs/String'
      });
      statusTopic.subscribe((message: any) => {
        console.log('Received status:', message.data);
        batteryDataBuffer.status = message.data || 'Unknown';
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to comprehensive battery state
      const stateTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/state',
        messageType: 'sensor_msgs/BatteryState'
      });
      stateTopic.subscribe((message: any) => {
        console.log('Received battery state:', message);
        
        // Extract comprehensive battery data from BatteryState message
        batteryDataBuffer.packVoltage = message.voltage;
        batteryDataBuffer.current = message.current;
        batteryDataBuffer.remainingCapacity = message.charge;
        batteryDataBuffer.totalCapacity = message.capacity;
        batteryDataBuffer.soc = message.percentage * 100; // Convert from 0-1 to 0-100
        // Ensure we always have 2 temperature sensors (simulate second sensor if only one provided)
        batteryDataBuffer.temperature = [message.temperature, message.temperature + 2];
        
        // Extract individual cell voltages
        if (message.cell_voltage && Array.isArray(message.cell_voltage)) {
          batteryDataBuffer.cellVoltages = message.cell_voltage.map((voltage: number, index: number) => ({
            cellNumber: index + 1,
            voltage: voltage
          }));
        }
        
        // Extract cycles from location field
        if (message.location && message.location.includes('cycles:')) {
          const cyclesMatch = message.location.match(/cycles:(\d+)/);
          if (cyclesMatch) {
            batteryDataBuffer.cycles = parseInt(cyclesMatch[1]);
          }
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
        // In a real system, you'd have specific MOS status fields
        const isHealthy = message.power_supply_health === 1; // POWER_SUPPLY_HEALTH_GOOD
        batteryDataBuffer.chargingMos = isHealthy;
        batteryDataBuffer.dischargingMos = isHealthy;
        batteryDataBuffer.balancing = false; // Would need specific balancing status
        
        hasReceivedData = true;
        updateBatteryData();
      });

      // Subscribe to cell voltages
      const cellTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/battery/cell_voltages',
        messageType: 'std_msgs/Float32MultiArray'
      });
      cellTopic.subscribe((message: any) => {
        console.log('Received cell voltages:', message.data);
        if (Array.isArray(message.data)) {
          batteryDataBuffer.cellVoltages = message.data.map((voltage: number, index: number) => ({
            cellNumber: index + 1,
            voltage: voltage
          }));
        }
        hasReceivedData = true;
        updateBatteryData();
      });

      console.log('Subscribed to all battery topics');
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

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

  const getCellVoltageColor = (voltage: number) => {
    if (voltage > 3.3) return '#00ff41';
    if (voltage > 3.0) return '#ffaa00';
    return '#ff4444';
  };

  return (
    <div className="battery-dashboard">
      <div className="battery-header">
        <div className="battery-title">
          <h1>🔋 Robot Battery Management</h1>
          <div className="connection-status">
            <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
              ●
            </span>
            <span>{connectionStatus}</span>
          </div>
        </div>
      </div>

      {batteryData ? (
        <div className="battery-content">
          {/* Main Battery Status */}
          <div className="battery-main-status">
            <div className="battery-visual">
              <div className="battery-terminal" />
              <div className="battery-outline">
                <div 
                  className="battery-fill"
                  style={{
                    height: `${batteryData.soc}%`,
                    backgroundColor: getBatteryHealthColor(batteryData.soc)
                  }}
                />
                <div className="battery-percentage">
                  {batteryData.soc?.toFixed(1)}%
                </div>
              </div>
            </div>
            
            <div className="battery-main-info">
              <div className="info-card primary">
                <div className="info-label">State of Charge</div>
                <div className="info-value" style={{ color: getBatteryHealthColor(batteryData.soc) }}>
                  {batteryData.soc?.toFixed(1)}%
                </div>
              </div>
              
              <div className="info-card">
                <div className="info-label">Pack Voltage</div>
                <div className="info-value">{batteryData.packVoltage?.toFixed(2)}V</div>
              </div>
              
              <div className="info-card">
                <div className="info-label">Current</div>
                <div className="info-value">{batteryData.current?.toFixed(2)}A</div>
              </div>
              
              <div className="info-card">
                <div className="info-label">Power</div>
                <div className="info-value">
                  {((batteryData.packVoltage || 0) * (batteryData.current || 0)).toFixed(1)}W
                </div>
              </div>
            </div>
          </div>

          {/* Capacity Information */}
          <div className="battery-section">
            <h2>🔋 Capacity Information</h2>
            <div className="capacity-grid">
              <div className="capacity-card">
                <div className="capacity-label">Remaining Capacity</div>
                <div className="capacity-value">{batteryData.remainingCapacity?.toFixed(1)} Ah</div>
                <div className="capacity-bar">
                  <div 
                    className="capacity-fill"
                    style={{
                      width: `${((batteryData.remainingCapacity || 0) / (batteryData.totalCapacity || 1)) * 100}%`,
                      backgroundColor: getBatteryHealthColor(batteryData.soc)
                    }}
                  />
                </div>
              </div>
              
              <div className="capacity-card">
                <div className="capacity-label">Total Capacity</div>
                <div className="capacity-value">{batteryData.totalCapacity?.toFixed(1)} Ah</div>
              </div>
              
              <div className="capacity-card">
                <div className="capacity-label">Charge Cycles</div>
                <div className="capacity-value">{batteryData.cycles}</div>
              </div>
            </div>
          </div>

          {/* Temperature Monitoring */}
          <div className="battery-section">
            <h2>🌡️ Temperature Monitoring</h2>
            <div className="temperature-grid">
              {batteryData.temperature?.map((temp, index) => (
                <div key={index} className="temperature-card">
                  <div className="temp-sensor">Sensor {index + 1}</div>
                  <div 
                    className="temp-value"
                    style={{ color: getTemperatureColor(temp) }}
                  >
                    {temp}°C
                  </div>
                  <div className="temp-bar">
                    <div 
                      className="temp-fill"
                      style={{
                        width: `${Math.min((temp / 80) * 100, 100)}%`,
                        backgroundColor: getTemperatureColor(temp)
                      }}
                    />
                  </div>
                </div>
              ))}
            </div>
          </div>

          {/* Cell Voltages */}
          <div className="battery-section">
            <div className="section-header">
              <h2>⚡ Individual Cell Voltages</h2>
              {batteryData.cellVoltages && batteryData.cellVoltages.length > 0 && (() => {
                const voltages = batteryData.cellVoltages.map(cell => cell.voltage);
                const minVoltage = Math.min(...voltages);
                const maxVoltage = Math.max(...voltages);
                const delta = maxVoltage - minVoltage;
                return (
                  <div className="voltage-delta">
                    Δ {delta.toFixed(3)}V
                  </div>
                );
              })()}
            </div>
            <div className="cells-grid">
              {batteryData.cellVoltages?.map((cell) => {
                const voltages = batteryData.cellVoltages!.map(c => c.voltage);
                const minVoltage = Math.min(...voltages);
                const maxVoltage = Math.max(...voltages);
                const isLowest = cell.voltage === minVoltage;
                const isHighest = cell.voltage === maxVoltage;
                
                return (
                  <div 
                    key={cell.cellNumber} 
                    className={`cell-card ${isLowest ? 'lowest-voltage' : ''} ${isHighest ? 'highest-voltage' : ''}`}
                  >
                    <div className="cell-number">Cell {cell.cellNumber}</div>
                    <div 
                      className="cell-voltage"
                      style={{ color: getCellVoltageColor(cell.voltage) }}
                    >
                      {cell.voltage.toFixed(3)}V
                    </div>
                    <div className="cell-bar">
                      <div 
                        className="cell-fill"
                        style={{
                          width: `${((cell.voltage - 2.5) / (4.2 - 2.5)) * 100}%`,
                          backgroundColor: getCellVoltageColor(cell.voltage)
                        }}
                      />
                    </div>
                  </div>
                );
              })}
            </div>
          </div>

          {/* System Status */}
          <div className="battery-section">
            <h2>⚙️ System Status</h2>
            <div className="status-grid">
              <div className="status-card">
                <div className="status-label">Charging MOS</div>
                <div className={`status-indicator ${batteryData.chargingMos ? 'active' : 'inactive'}`}>
                  {batteryData.chargingMos ? '✅ Active' : '❌ Inactive'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-label">Discharging MOS</div>
                <div className={`status-indicator ${batteryData.dischargingMos ? 'active' : 'inactive'}`}>
                  {batteryData.dischargingMos ? '✅ Active' : '❌ Inactive'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-label">Cell Balancing</div>
                <div className={`status-indicator ${batteryData.balancing ? 'active' : 'inactive'}`}>
                  {batteryData.balancing ? '⚖️ Active' : '⚖️ Inactive'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-label">Battery Status</div>
                <div className="status-value">{batteryData.status || 'Unknown'}</div>
              </div>
            </div>
          </div>

          {/* Estimated Runtime */}
          <div className="battery-section">
            <h2>⏱️ Estimated Runtime</h2>
            <div className="runtime-card">
              <div className="runtime-info">
                {batteryData.current && batteryData.current > 0 ? (
                  <>
                    <div className="runtime-value">
                      {((batteryData.remainingCapacity || 0) / Math.abs(batteryData.current)).toFixed(1)} hours
                    </div>
                    <div className="runtime-label">at current consumption</div>
                  </>
                ) : (
                  <>
                    <div className="runtime-value">∞</div>
                    <div className="runtime-label">No current draw</div>
                  </>
                )}
              </div>
            </div>
          </div>
        </div>
      ) : (
        <div className="battery-loading">
          <div className="loading-spinner"></div>
          <div className="loading-text">Waiting for battery data...</div>
          <div className="test-data-controls">
            <button 
              className="test-data-button"
              onClick={startTestData}
              disabled={useTestData}
            >
              🧪 Use Test Data
            </button>
            {useTestData && (
              <button 
                className="test-data-button stop"
                onClick={stopTestData}
              >
                ⏹️ Stop Test Data
              </button>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default BatteryDashboard;
