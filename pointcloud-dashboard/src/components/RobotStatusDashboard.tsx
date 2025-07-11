import React, { useRef, useEffect, useState } from 'react';
import './RobotStatusDashboard.css';

interface RobotStatus {
  timestamp: number;
  status: 'idle' | 'ready' | 'running' | 'not_responding' | 'error';
  lastHeartbeat: number;
  uptime: number;
  currentTask: string;
  taskProgress: number;
  errorMessage?: string;
  systemHealth: {
    cpu: number;
    memory: number;
    temperature: number;
    diskSpace: number;
  };
  connectivity: {
    rosConnection: boolean;
    networkLatency: number;
    signalStrength: number;
  };
  subsystems: {
    navigation: 'online' | 'offline' | 'error';
    perception: 'online' | 'offline' | 'error';
    manipulation: 'online' | 'offline' | 'error';
    communication: 'online' | 'offline' | 'error';
  };
  location: {
    x: number;
    y: number;
    z: number;
    orientation: number;
  };
  batteryLevel: number;
  emergencyStop: boolean;
}

interface StatusConfig {
  color: string;
  icon: string;
  description: string;
  priority: number;
}

const RobotStatusDashboard: React.FC = () => {
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [useTestData, setUseTestData] = useState(false);
  const [lastUpdateTime, setLastUpdateTime] = useState<number>(0);
  const rosRef = useRef<any>(null);

  // Status configurations
  const statusConfigs: Record<string, StatusConfig> = {
    idle: {
      color: '#00aaff',
      icon: '🟦',
      description: 'Robot is idle and waiting for commands',
      priority: 1
    },
    ready: {
      color: '#00ff41',
      icon: '🟢',
      description: 'Robot is ready to execute tasks',
      priority: 2
    },
    running: {
      color: '#ffaa00',
      icon: '🟡',
      description: 'Robot is actively executing a task',
      priority: 3
    },
    not_responding: {
      color: '#ff8800',
      icon: '🟠',
      description: 'Robot is not responding to commands',
      priority: 4
    },
    error: {
      color: '#ff4444',
      icon: '🔴',
      description: 'Robot has encountered an error',
      priority: 5
    }
  };

  const getSubsystemColor = (status: string) => {
    switch (status) {
      case 'online': return '#00ff41';
      case 'offline': return '#666666';
      case 'error': return '#ff4444';
      default: return '#666666';
    }
  };

  const getHealthColor = (value: number, type: 'cpu' | 'memory' | 'temperature' | 'disk') => {
    switch (type) {
      case 'cpu':
      case 'memory':
      case 'disk':
        if (value < 50) return '#00ff41';
        if (value < 80) return '#ffaa00';
        return '#ff4444';
      case 'temperature':
        if (value < 60) return '#00ff41';
        if (value < 80) return '#ffaa00';
        return '#ff4444';
      default:
        return '#00ff41';
    }
  };

  const getSignalStrengthBars = (strength: number) => {
    const bars = Math.ceil(strength / 25);
    return '📶'.repeat(Math.max(1, Math.min(4, bars)));
  };

  const formatUptime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  // Generate mock robot status data for testing
  const generateMockData = (): RobotStatus => {
    const statuses: Array<'idle' | 'ready' | 'running' | 'not_responding' | 'error'> = 
      ['idle', 'ready', 'running', 'not_responding', 'error'];
    
    const currentStatus = statuses[Math.floor(Math.random() * statuses.length)];
    const now = Date.now();
    
    // Simulate different scenarios based on status
    let taskProgress = 0;
    let currentTask = 'None';
    let errorMessage = undefined;
    
    switch (currentStatus) {
      case 'running':
        taskProgress = Math.random() * 100;
        currentTask = ['Navigation to waypoint', 'Object detection', 'Mapping area', 'Charging'][Math.floor(Math.random() * 4)];
        break;
      case 'error':
        errorMessage = ['Motor controller fault', 'Sensor calibration failed', 'Navigation path blocked', 'Communication timeout'][Math.floor(Math.random() * 4)];
        break;
      case 'ready':
        currentTask = 'Awaiting instructions';
        break;
      case 'not_responding':
        currentTask = 'Connection lost';
        break;
      default:
        currentTask = 'Standby';
    }

    return {
      timestamp: now,
      status: currentStatus,
      lastHeartbeat: now - Math.random() * 5000,
      uptime: Math.floor(Math.random() * 86400), // Up to 24 hours
      currentTask,
      taskProgress,
      errorMessage,
      systemHealth: {
        cpu: Math.random() * 100,
        memory: Math.random() * 100,
        temperature: 30 + Math.random() * 50, // 30-80°C
        diskSpace: Math.random() * 100
      },
      connectivity: {
        rosConnection: currentStatus !== 'not_responding',
        networkLatency: Math.random() * 100,
        signalStrength: Math.random() * 100
      },
      subsystems: {
        navigation: currentStatus === 'error' && Math.random() > 0.7 ? 'error' : 
                   currentStatus === 'not_responding' ? 'offline' : 'online',
        perception: currentStatus === 'error' && Math.random() > 0.8 ? 'error' : 
                   currentStatus === 'not_responding' ? 'offline' : 'online',
        manipulation: currentStatus === 'error' && Math.random() > 0.9 ? 'error' : 
                     currentStatus === 'not_responding' ? 'offline' : 'online',
        communication: currentStatus === 'not_responding' ? 'offline' : 'online'
      },
      location: {
        x: Math.random() * 20 - 10, // -10 to 10
        y: Math.random() * 20 - 10,
        z: Math.random() * 2,
        orientation: Math.random() * 360
      },
      batteryLevel: Math.random() * 100,
      emergencyStop: currentStatus === 'error' && Math.random() > 0.8
    };
  };

  const startTestData = () => {
    setUseTestData(true);
    setConnectionStatus('Using test data');
    
    // Generate initial data
    setRobotStatus(generateMockData());
    setLastUpdateTime(Date.now());
    
    // Update data every 3 seconds
    const interval = setInterval(() => {
      setRobotStatus(generateMockData());
      setLastUpdateTime(Date.now());
    }, 3000);
    
    return () => clearInterval(interval);
  };

  const stopTestData = () => {
    setUseTestData(false);
    setRobotStatus(null);
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
          console.log('Connected to ROS bridge for robot status data');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
          subscribeToRobotTopics(ros);
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

    const subscribeToRobotTopics = (ros: any) => {
      let statusDataBuffer: Partial<RobotStatus> = {};
      let hasReceivedData = false;

      // Helper function to update robot status
      const updateRobotStatus = () => {
        if (hasReceivedData) {
          setRobotStatus(prev => ({
            ...prev,
            ...statusDataBuffer,
            timestamp: Date.now()
          } as RobotStatus));
          setLastUpdateTime(Date.now());
        }
      };

      // Subscribe to robot status
      const statusTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/robot/status',
        messageType: 'std_msgs/String'
      });
      statusTopic.subscribe((message: any) => {
        console.log('Received robot status:', message.data);
        statusDataBuffer.status = message.data as any;
        hasReceivedData = true;
        updateRobotStatus();
      });

      // Subscribe to robot heartbeat
      const heartbeatTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/robot/heartbeat',
        messageType: 'std_msgs/Header'
      });
      heartbeatTopic.subscribe((message: any) => {
        console.log('Received robot heartbeat');
        statusDataBuffer.lastHeartbeat = Date.now();
        hasReceivedData = true;
        updateRobotStatus();
      });

      // Subscribe to current task
      const taskTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/robot/current_task',
        messageType: 'std_msgs/String'
      });
      taskTopic.subscribe((message: any) => {
        console.log('Received current task:', message.data);
        statusDataBuffer.currentTask = message.data;
        hasReceivedData = true;
        updateRobotStatus();
      });

      // Subscribe to task progress
      const progressTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/robot/task_progress',
        messageType: 'std_msgs/Float32'
      });
      progressTopic.subscribe((message: any) => {
        console.log('Received task progress:', message.data);
        statusDataBuffer.taskProgress = message.data;
        hasReceivedData = true;
        updateRobotStatus();
      });

      // Subscribe to system health
      const healthTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/robot/system_health',
        messageType: 'diagnostic_msgs/DiagnosticArray'
      });
      healthTopic.subscribe((message: any) => {
        console.log('Received system health:', message);
        // Parse diagnostic data
        if (message.status && message.status.length > 0) {
          const healthData = {
            cpu: 0,
            memory: 0,
            temperature: 0,
            diskSpace: 0
          };
          
          message.status.forEach((status: any) => {
            if (status.name.includes('CPU')) {
              healthData.cpu = parseFloat(status.message) || 0;
            } else if (status.name.includes('Memory')) {
              healthData.memory = parseFloat(status.message) || 0;
            } else if (status.name.includes('Temperature')) {
              healthData.temperature = parseFloat(status.message) || 0;
            } else if (status.name.includes('Disk')) {
              healthData.diskSpace = parseFloat(status.message) || 0;
            }
          });
          
          statusDataBuffer.systemHealth = healthData;
          hasReceivedData = true;
          updateRobotStatus();
        }
      });

      console.log('Subscribed to all robot status topics');
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  // Check if robot is responding based on last heartbeat
  const isRobotResponding = robotStatus ? 
    (Date.now() - robotStatus.lastHeartbeat) < 10000 : false;

  return (
    <div className="robot-status-dashboard">
      <div className="robot-status-header">
        <div className="robot-status-title">
          <h1>🤖 Robot Status Monitor</h1>
          <div className="connection-status">
            <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
              ●
            </span>
            <span>{connectionStatus}</span>
          </div>
        </div>
      </div>

      {robotStatus ? (
        <div className="robot-status-content">
          {/* Main Status Display */}
          <div className="main-status-section">
            <div className="status-indicator-large">
              <div 
                className={`status-circle ${robotStatus.status}`}
                style={{ borderColor: statusConfigs[robotStatus.status].color }}
              >
                <div className="status-icon">
                  {statusConfigs[robotStatus.status].icon}
                </div>
                <div 
                  className="status-text"
                  style={{ color: statusConfigs[robotStatus.status].color }}
                >
                  {robotStatus.status.toUpperCase().replace('_', ' ')}
                </div>
              </div>
            </div>
            
            <div className="status-details">
              <div className="status-info-card">
                <div className="info-label">Current Status</div>
                <div 
                  className="info-value"
                  style={{ color: statusConfigs[robotStatus.status].color }}
                >
                  {statusConfigs[robotStatus.status].description}
                </div>
              </div>
              
              <div className="status-info-card">
                <div className="info-label">Current Task</div>
                <div className="info-value">{robotStatus.currentTask}</div>
                {robotStatus.status === 'running' && (
                  <div className="progress-bar">
                    <div 
                      className="progress-fill"
                      style={{ 
                        width: `${robotStatus.taskProgress}%`,
                        backgroundColor: statusConfigs[robotStatus.status].color
                      }}
                    />
                    <div className="progress-text">{robotStatus.taskProgress.toFixed(1)}%</div>
                  </div>
                )}
              </div>
              
              <div className="status-info-card">
                <div className="info-label">Uptime</div>
                <div className="info-value">{formatUptime(robotStatus.uptime)}</div>
              </div>
              
              {robotStatus.errorMessage && (
                <div className="status-info-card error-card">
                  <div className="info-label">Error Message</div>
                  <div className="info-value error-text">{robotStatus.errorMessage}</div>
                </div>
              )}
            </div>
          </div>

          {/* Emergency Stop Alert */}
          {robotStatus.emergencyStop && (
            <div className="emergency-alert">
              <div className="emergency-icon">🚨</div>
              <div className="emergency-text">EMERGENCY STOP ACTIVATED</div>
              <div className="emergency-icon">🚨</div>
            </div>
          )}

          {/* System Health */}
          <div className="status-section">
            <h2>🔧 System Health</h2>
            <div className="health-grid">
              <div className="health-card">
                <div className="health-icon">🖥️</div>
                <div className="health-label">CPU Usage</div>
                <div 
                  className="health-value"
                  style={{ color: getHealthColor(robotStatus.systemHealth.cpu, 'cpu') }}
                >
                  {robotStatus.systemHealth.cpu.toFixed(1)}%
                </div>
                <div className="health-bar">
                  <div 
                    className="health-fill"
                    style={{
                      width: `${robotStatus.systemHealth.cpu}%`,
                      backgroundColor: getHealthColor(robotStatus.systemHealth.cpu, 'cpu')
                    }}
                  />
                </div>
              </div>
              
              <div className="health-card">
                <div className="health-icon">💾</div>
                <div className="health-label">Memory Usage</div>
                <div 
                  className="health-value"
                  style={{ color: getHealthColor(robotStatus.systemHealth.memory, 'memory') }}
                >
                  {robotStatus.systemHealth.memory.toFixed(1)}%
                </div>
                <div className="health-bar">
                  <div 
                    className="health-fill"
                    style={{
                      width: `${robotStatus.systemHealth.memory}%`,
                      backgroundColor: getHealthColor(robotStatus.systemHealth.memory, 'memory')
                    }}
                  />
                </div>
              </div>
              
              <div className="health-card">
                <div className="health-icon">🌡️</div>
                <div className="health-label">Temperature</div>
                <div 
                  className="health-value"
                  style={{ color: getHealthColor(robotStatus.systemHealth.temperature, 'temperature') }}
                >
                  {robotStatus.systemHealth.temperature.toFixed(1)}°C
                </div>
                <div className="health-bar">
                  <div 
                    className="health-fill"
                    style={{
                      width: `${Math.min(robotStatus.systemHealth.temperature, 100)}%`,
                      backgroundColor: getHealthColor(robotStatus.systemHealth.temperature, 'temperature')
                    }}
                  />
                </div>
              </div>
              
              <div className="health-card">
                <div className="health-icon">💿</div>
                <div className="health-label">Disk Usage</div>
                <div 
                  className="health-value"
                  style={{ color: getHealthColor(robotStatus.systemHealth.diskSpace, 'disk') }}
                >
                  {robotStatus.systemHealth.diskSpace.toFixed(1)}%
                </div>
                <div className="health-bar">
                  <div 
                    className="health-fill"
                    style={{
                      width: `${robotStatus.systemHealth.diskSpace}%`,
                      backgroundColor: getHealthColor(robotStatus.systemHealth.diskSpace, 'disk')
                    }}
                  />
                </div>
              </div>
            </div>
          </div>

          {/* Subsystems Status */}
          <div className="status-section">
            <h2>⚙️ Subsystems Status</h2>
            <div className="subsystems-grid">
              <div className="subsystem-card">
                <div className="subsystem-icon">🧭</div>
                <div className="subsystem-label">Navigation</div>
                <div 
                  className="subsystem-status"
                  style={{ color: getSubsystemColor(robotStatus.subsystems.navigation) }}
                >
                  {robotStatus.subsystems.navigation.toUpperCase()}
                </div>
              </div>
              
              <div className="subsystem-card">
                <div className="subsystem-icon">👁️</div>
                <div className="subsystem-label">Perception</div>
                <div 
                  className="subsystem-status"
                  style={{ color: getSubsystemColor(robotStatus.subsystems.perception) }}
                >
                  {robotStatus.subsystems.perception.toUpperCase()}
                </div>
              </div>
              
              <div className="subsystem-card">
                <div className="subsystem-icon">🦾</div>
                <div className="subsystem-label">Manipulation</div>
                <div 
                  className="subsystem-status"
                  style={{ color: getSubsystemColor(robotStatus.subsystems.manipulation) }}
                >
                  {robotStatus.subsystems.manipulation.toUpperCase()}
                </div>
              </div>
              
              <div className="subsystem-card">
                <div className="subsystem-icon">📡</div>
                <div className="subsystem-label">Communication</div>
                <div 
                  className="subsystem-status"
                  style={{ color: getSubsystemColor(robotStatus.subsystems.communication) }}
                >
                  {robotStatus.subsystems.communication.toUpperCase()}
                </div>
              </div>
            </div>
          </div>

          {/* Connectivity & Location */}
          <div className="status-section-row">
            <div className="status-section">
              <h2>📶 Connectivity</h2>
              <div className="connectivity-info">
                <div className="connectivity-item">
                  <div className="connectivity-label">ROS Connection</div>
                  <div 
                    className="connectivity-value"
                    style={{ color: robotStatus.connectivity.rosConnection ? '#00ff41' : '#ff4444' }}
                  >
                    {robotStatus.connectivity.rosConnection ? '✅ Connected' : '❌ Disconnected'}
                  </div>
                </div>
                
                <div className="connectivity-item">
                  <div className="connectivity-label">Network Latency</div>
                  <div className="connectivity-value">
                    {robotStatus.connectivity.networkLatency.toFixed(0)}ms
                  </div>
                </div>
                
                <div className="connectivity-item">
                  <div className="connectivity-label">Signal Strength</div>
                  <div className="connectivity-value">
                    {getSignalStrengthBars(robotStatus.connectivity.signalStrength)} {robotStatus.connectivity.signalStrength.toFixed(0)}%
                  </div>
                </div>
              </div>
            </div>
            
            <div className="status-section">
              <h2>📍 Location & Battery</h2>
              <div className="location-info">
                <div className="location-item">
                  <div className="location-label">Position</div>
                  <div className="location-value">
                    X: {robotStatus.location.x.toFixed(2)}m, Y: {robotStatus.location.y.toFixed(2)}m
                  </div>
                </div>
                
                <div className="location-item">
                  <div className="location-label">Orientation</div>
                  <div className="location-value">
                    {robotStatus.location.orientation.toFixed(1)}°
                  </div>
                </div>
                
                <div className="location-item">
                  <div className="location-label">Battery Level</div>
                  <div 
                    className="location-value"
                    style={{ 
                      color: robotStatus.batteryLevel > 50 ? '#00ff41' : 
                             robotStatus.batteryLevel > 20 ? '#ffaa00' : '#ff4444'
                    }}
                  >
                    🔋 {robotStatus.batteryLevel.toFixed(1)}%
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Last Update Info */}
          <div className="last-update">
            Last updated: {new Date(lastUpdateTime).toLocaleTimeString()}
            {!isRobotResponding && (
              <span className="not-responding-warning">
                ⚠️ Robot not responding (last heartbeat: {new Date(robotStatus.lastHeartbeat).toLocaleTimeString()})
              </span>
            )}
          </div>
        </div>
      ) : (
        <div className="robot-status-loading">
          <div className="loading-spinner"></div>
          <div className="loading-text">Waiting for robot status data...</div>
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

export default RobotStatusDashboard;
