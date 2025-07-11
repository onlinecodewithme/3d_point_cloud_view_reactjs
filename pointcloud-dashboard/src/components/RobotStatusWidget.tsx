import React, { useRef, useEffect, useState } from 'react';
import './RobotStatusWidget.css';

interface RobotStatus {
  timestamp: number;
  status: 'idle' | 'ready' | 'running' | 'not_responding' | 'error';
  lastHeartbeat: number;
  uptime: number;
  currentTask: string;
  taskProgress: number;
  errorMessage?: string;
  batteryLevel: number;
  emergencyStop: boolean;
}

interface StatusConfig {
  color: string;
  icon: string;
  description: string;
}

const RobotStatusWidget: React.FC = () => {
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [useTestData, setUseTestData] = useState(false);
  const rosRef = useRef<any>(null);

  // Status configurations
  const statusConfigs: Record<string, StatusConfig> = {
    idle: {
      color: '#00aaff',
      icon: '🟦',
      description: 'Idle'
    },
    ready: {
      color: '#00ff41',
      icon: '🟢',
      description: 'Ready'
    },
    running: {
      color: '#ffaa00',
      icon: '🟡',
      description: 'Running'
    },
    not_responding: {
      color: '#ff8800',
      icon: '🟠',
      description: 'Not Responding'
    },
    error: {
      color: '#ff4444',
      icon: '🔴',
      description: 'Error'
    }
  };

  const formatUptime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
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
        errorMessage = ['Motor controller fault', 'Sensor calibration failed', 'Navigation path blocked'][Math.floor(Math.random() * 3)];
        currentTask = 'Error state';
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
      batteryLevel: Math.random() * 100,
      emergencyStop: currentStatus === 'error' && Math.random() > 0.8
    };
  };

  const startTestData = () => {
    setUseTestData(true);
    setRobotStatus(generateMockData());
    
    const interval = setInterval(() => {
      setRobotStatus(generateMockData());
    }, 3000);
    
    return () => clearInterval(interval);
  };

  const stopTestData = () => {
    setUseTestData(false);
    setRobotStatus(null);
  };

  useEffect(() => {
    const initROS = async () => {
      try {
        // @ts-ignore
        const ROSLIB = window.ROSLIB;
        if (!ROSLIB) return;

        const ros = new ROSLIB.Ros({
          url: 'ws://localhost:9090'
        });

        rosRef.current = ros;

        ros.on('connection', () => {
          setIsConnected(true);
          subscribeToRobotTopics(ros);
        });

        ros.on('error', () => {
          setIsConnected(false);
        });

        ros.on('close', () => {
          setIsConnected(false);
        });

      } catch (error) {
        console.error('Failed to initialize ROS:', error);
      }
    };

    const subscribeToRobotTopics = (ros: any) => {
      let statusDataBuffer: Partial<RobotStatus> = {};
      let hasReceivedData = false;

      const updateRobotStatus = () => {
        if (hasReceivedData) {
          setRobotStatus(prev => ({
            ...prev,
            ...statusDataBuffer,
            timestamp: Date.now()
          } as RobotStatus));
        }
      };

      // Subscribe to robot status topics
      const topics = [
        { name: '/robot/status', field: 'status', type: 'std_msgs/String' },
        { name: '/robot/current_task', field: 'currentTask', type: 'std_msgs/String' },
        { name: '/robot/task_progress', field: 'taskProgress', type: 'std_msgs/Float32' }
      ];

      topics.forEach(({ name, field, type }) => {
        const topic = new (window as any).ROSLIB.Topic({
          ros: ros,
          name: name,
          messageType: type
        });
        
        topic.subscribe((message: any) => {
          if (type === 'std_msgs/String') {
            (statusDataBuffer as any)[field] = message.data;
          } else {
            (statusDataBuffer as any)[field] = message.data;
          }
          hasReceivedData = true;
          updateRobotStatus();
        });
      });

      // Subscribe to heartbeat
      const heartbeatTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/robot/heartbeat',
        messageType: 'std_msgs/Header'
      });
      heartbeatTopic.subscribe(() => {
        statusDataBuffer.lastHeartbeat = Date.now();
        hasReceivedData = true;
        updateRobotStatus();
      });
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  if (!robotStatus) {
    return (
      <div className="robot-widget">
        <div className="robot-loading">
          <div className="loading-circle"></div>
          <p>Waiting for robot status...</p>
          <button 
            className="test-btn"
            onClick={startTestData}
            disabled={useTestData}
          >
            🧪 Test Data
          </button>
        </div>
      </div>
    );
  }

  const statusConfig = statusConfigs[robotStatus.status];
  const isRobotResponding = (Date.now() - robotStatus.lastHeartbeat) < 10000;

  return (
    <div className="robot-widget">
      {/* Emergency Alert */}
      {robotStatus.emergencyStop && (
        <div className="emergency-alert">
          <span className="emergency-icon">🚨</span>
          <span className="emergency-text">EMERGENCY STOP</span>
        </div>
      )}

      {/* Main Status Display */}
      <div className="status-main">
        <div 
          className={`status-circle ${robotStatus.status}`}
          style={{ borderColor: statusConfig.color }}
        >
          <div className="status-icon">
            {statusConfig.icon}
          </div>
        </div>
        <div className="status-info">
          <div 
            className="status-text"
            style={{ color: statusConfig.color }}
          >
            {statusConfig.description.toUpperCase()}
          </div>
          <div className="status-task">
            {robotStatus.currentTask}
          </div>
        </div>
      </div>

      {/* Progress Bar for Running Status */}
      {robotStatus.status === 'running' && (
        <div className="progress-section">
          <div className="progress-bar">
            <div 
              className="progress-fill"
              style={{ 
                width: `${robotStatus.taskProgress}%`,
                backgroundColor: statusConfig.color
              }}
            />
            <div className="progress-text">{robotStatus.taskProgress.toFixed(0)}%</div>
          </div>
        </div>
      )}

      {/* Error Message */}
      {robotStatus.errorMessage && (
        <div className="error-message">
          ⚠️ {robotStatus.errorMessage}
        </div>
      )}

      {/* Status Info Grid */}
      <div className="info-grid">
        <div className="info-item">
          <div className="info-icon">⏱️</div>
          <div className="info-value">{formatUptime(robotStatus.uptime)}</div>
          <div className="info-label">Uptime</div>
        </div>

        <div className="info-item">
          <div className="info-icon">🔋</div>
          <div 
            className="info-value"
            style={{ 
              color: robotStatus.batteryLevel > 50 ? '#00ff41' : 
                     robotStatus.batteryLevel > 20 ? '#ffaa00' : '#ff4444'
            }}
          >
            {robotStatus.batteryLevel.toFixed(0)}%
          </div>
          <div className="info-label">Battery</div>
        </div>

        <div className="info-item">
          <div className="info-icon">💓</div>
          <div 
            className="info-value"
            style={{ color: isRobotResponding ? '#00ff41' : '#ff4444' }}
          >
            {isRobotResponding ? 'Active' : 'Lost'}
          </div>
          <div className="info-label">Heartbeat</div>
        </div>
      </div>

      {/* Test Data Control */}
      {useTestData && (
        <button className="stop-test-btn" onClick={stopTestData}>
          ⏹️ Stop Test
        </button>
      )}
    </div>
  );
};

export default RobotStatusWidget;
