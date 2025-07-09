import React, { useRef, useEffect, useState } from 'react';
import './SystemMonitoringWidget.css';

interface SystemMetrics {
  timestamp: number;
  cpuUsage: number;
  memoryUsage: number;
  diskUsage: number;
  temperature: number;
  networkRx: number;
  networkTx: number;
  processCount: number;
  uptime: number;
  loadAverage: number[];
}

interface BrainSystemData {
  brainName: string;
  metrics: SystemMetrics;
  isOnline: boolean;
  lastUpdate: number;
}

interface SystemMonitoringWidgetProps {
  ros?: any;
  isConnected?: boolean;
  compact?: boolean;
}

const SystemMonitoringWidget: React.FC<SystemMonitoringWidgetProps> = ({ 
  ros, 
  isConnected = false, 
  compact = false 
}) => {
  const [systemData, setSystemData] = useState<{ [key: string]: BrainSystemData }>({});
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

    const subscribeToSystemTopics = (ros: any) => {
      console.log('System Monitoring Widget: Setting up subscriptions...');
      
      // Helper function to process system data
      const processSystemData = (data: any, brainName: string, brainKey: string) => {
        let metrics;
        
        // Handle different message formats
        if (typeof data === 'string') {
          try {
            const parsedData = JSON.parse(data);
            console.log(`System Widget: Parsed ${brainName} data:`, parsedData);
            
            // Convert string values to numbers for perception brain data
            const parseValue = (value: any) => {
              if (typeof value === 'string') {
                const parsed = parseFloat(value);
                return isNaN(parsed) ? 0 : parsed;
              }
              return typeof value === 'number' ? value : 0;
            };

            metrics = {
              timestamp: Date.now(),
              cpuUsage: parseValue(parsedData.cpu?.usage || parsedData.cpu?.usage_percent || parsedData.cpu_usage),
              memoryUsage: parseValue(parsedData.memory?.usage || parsedData.memory?.usage_percent || parsedData.memory_usage),
              diskUsage: parseValue(parsedData.disk?.usage || parsedData.disk?.usage_percent || parsedData.disk_usage || parsedData.storage?.usage_percent),
              temperature: parseValue(parsedData.thermal?.temperature || parsedData.temperature || parsedData.cpu?.temperature),
              networkRx: parseValue(parsedData.network?.rx_bytes || parsedData.network_rx || parsedData.network?.bytes_received),
              networkTx: parseValue(parsedData.network?.tx_bytes || parsedData.network_tx || parsedData.network?.bytes_sent),
              processCount: parseValue(parsedData.system?.process_count || parsedData.process_count || parsedData.processes?.process_count),
              uptime: parseValue(parsedData.system?.uptime || parsedData.uptime || parsedData.uptime?.seconds),
              loadAverage: parsedData.system?.load_average || parsedData.load_average || parsedData.system_load?.load_1min ? [
                parseValue(parsedData.system_load.load_1min),
                parseValue(parsedData.system_load.load_5min),
                parseValue(parsedData.system_load.load_15min)
              ] : [0, 0, 0]
            };
          } catch (error) {
            console.error(`System Widget: Error parsing ${brainName} JSON data:`, error);
            return;
          }
        } else {
          metrics = {
            timestamp: Date.now(),
            cpuUsage: data.cpu?.usage || data.cpu_usage || data.cpuUsage || 0,
            memoryUsage: data.memory?.usage || data.memory_usage || data.memoryUsage || 0,
            diskUsage: data.disk?.usage || data.disk_usage || data.diskUsage || 0,
            temperature: data.thermal?.temperature || data.temperature || 0,
            networkRx: data.network?.rx_bytes || data.network_rx || data.networkRx || 0,
            networkTx: data.network?.tx_bytes || data.network_tx || data.networkTx || 0,
            processCount: data.system?.process_count || data.process_count || data.processCount || 0,
            uptime: data.system?.uptime || data.uptime || 0,
            loadAverage: data.system?.load_average || data.load_average || data.loadAverage || [0, 0, 0]
          };
        }

        const brainData: BrainSystemData = {
          brainName,
          metrics,
          isOnline: true,
          lastUpdate: Date.now()
        };

        setSystemData(prev => ({
          ...prev,
          [brainKey]: brainData
        }));
        
        setConnectionStatus('Connected');
      };

      // Subscribe to cognition brain system monitoring
      const cognitionTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/cognitionbrain/system_monitoring',
        messageType: 'std_msgs/String'
      });

      cognitionTopic.subscribe((message: any) => {
        console.log('System Widget: Received cognition brain data:', message.data);
        processSystemData(message.data, 'Cognition Brain', 'cognition');
      });

      // Subscribe to perception brain system monitoring
      const perceptionTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/perceptionbrain/system_monitoring',
        messageType: 'std_msgs/String'
      });

      perceptionTopic.subscribe((message: any) => {
        console.log('System Widget: Received perception brain data:', message.data);
        processSystemData(message.data, 'Perception Brain', 'perception');
      });

      console.log('System Widget: Subscribed to system monitoring topics');
    };

    subscribeToSystemTopics(rosRef.current);
  }, [isConnected]);

  const getHealthColor = (value: number, thresholds: { good: number; warning: number }) => {
    if (value <= thresholds.good) return '#00ff41';
    if (value <= thresholds.warning) return '#ffaa00';
    return '#ff4444';
  };

  const formatUptime = (seconds: number) => {
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    
    if (days > 0) return `${days}d ${hours}h`;
    if (hours > 0) return `${hours}h ${minutes}m`;
    return `${minutes}m`;
  };

  const formatBytes = (bytes: number) => {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
  };

  const getSystemIcon = (brainKey: string) => {
    return brainKey === 'cognition' ? '🧠' : '👁️';
  };

  const getBrainThemeColor = (brainKey: string) => {
    return brainKey === 'cognition' ? '#2196F3' : '#FF6B6B';
  };

  const getOverallSystemHealth = () => {
    const onlineBrains = Object.values(systemData).filter(brain => brain.isOnline);
    if (onlineBrains.length === 0) return { status: 'unknown', color: '#888' };

    const avgCpu = onlineBrains.reduce((sum, brain) => sum + brain.metrics.cpuUsage, 0) / onlineBrains.length;
    const avgMemory = onlineBrains.reduce((sum, brain) => sum + brain.metrics.memoryUsage, 0) / onlineBrains.length;
    const maxTemp = Math.max(...onlineBrains.map(brain => brain.metrics.temperature));

    if (avgCpu > 90 || avgMemory > 95 || maxTemp > 85) {
      return { status: 'critical', color: '#ff4444' };
    } else if (avgCpu > 70 || avgMemory > 80 || maxTemp > 70) {
      return { status: 'warning', color: '#ffaa00' };
    } else {
      return { status: 'healthy', color: '#00ff41' };
    }
  };

  const systemHealth = getOverallSystemHealth();
  const onlineBrains = Object.values(systemData).filter(brain => brain.isOnline).length;

  if (compact) {
    return (
      <div className="system-monitoring-widget compact">
        <div className="system-widget-header">
          <span className="system-icon">🧠</span>
          <span className="system-title">System Monitor</span>
          <span className={`connection-dot ${isConnected ? 'connected' : 'disconnected'}`}>●</span>
        </div>
        
        {onlineBrains > 0 ? (
          <div className="system-compact-content">
            <div className="system-health-indicator">
              <div 
                className="health-status"
                style={{ color: systemHealth.color }}
              >
                {systemHealth.status === 'healthy' ? '✅' : systemHealth.status === 'warning' ? '⚠️' : '❌'}
                {systemHealth.status.toUpperCase()}
              </div>
              <div className="brains-count">{onlineBrains}/2 Brains Online</div>
            </div>
            
            <div className="system-mini-stats">
              {Object.entries(systemData).map(([key, brain]) => (
                <div key={key} className="mini-brain-stat">
                  <div className="mini-brain-header">
                    <span className="mini-brain-icon">{getSystemIcon(key)}</span>
                    <span className="mini-brain-name">{key === 'cognition' ? 'Cognition' : 'Perception'}</span>
                  </div>
                  <div className="mini-metrics">
                    <div className="mini-metric">
                      <span className="mini-label">CPU:</span>
                      <span 
                        className="mini-value"
                        style={{ color: getHealthColor(brain.metrics.cpuUsage, { good: 50, warning: 80 }) }}
                      >
                        {brain.metrics.cpuUsage.toFixed(0)}%
                      </span>
                    </div>
                    <div className="mini-metric">
                      <span className="mini-label">RAM:</span>
                      <span 
                        className="mini-value"
                        style={{ color: getHealthColor(brain.metrics.memoryUsage, { good: 70, warning: 85 }) }}
                      >
                        {brain.metrics.memoryUsage.toFixed(0)}%
                      </span>
                    </div>
                    <div className="mini-metric">
                      <span className="mini-label">Temp:</span>
                      <span 
                        className="mini-value"
                        style={{ color: getHealthColor(brain.metrics.temperature, { good: 60, warning: 80 }) }}
                      >
                        {brain.metrics.temperature.toFixed(0)}°C
                      </span>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        ) : (
          <div className="system-loading-compact">
            <div className="loading-dots">●●●</div>
            <div className="loading-text">Loading...</div>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className="system-monitoring-widget">
      <div className="system-widget-header">
        <div className="system-widget-title">
          <span className="system-icon">🧠</span>
          <h3>Brain System Monitor</h3>
        </div>
        <div className="system-connection-status">
          <span className={`connection-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
            ●
          </span>
          <span className="connection-text">{connectionStatus}</span>
        </div>
      </div>

      {onlineBrains > 0 ? (
        <div className="system-widget-content">
          {/* Overall System Health */}
          <div className="system-health-overview">
            <div className="health-indicator">
              <div 
                className="health-icon"
                style={{ color: systemHealth.color }}
              >
                {systemHealth.status === 'healthy' ? '✅' : systemHealth.status === 'warning' ? '⚠️' : '❌'}
              </div>
              <div className="health-info">
                <div className="health-status" style={{ color: systemHealth.color }}>
                  {systemHealth.status.toUpperCase()}
                </div>
                <div className="health-subtitle">
                  {onlineBrains}/2 Brain Systems Online
                </div>
              </div>
            </div>
          </div>

          {/* Brain Systems */}
          <div className="brain-systems-grid">
            {Object.entries(systemData).map(([key, brain]) => (
              <div 
                key={key} 
                className={`brain-system-card ${key}`}
                style={{ borderColor: getBrainThemeColor(key) }}
              >
                <div className="brain-card-header">
                  <div className="brain-info">
                    <span className="brain-icon">{getSystemIcon(key)}</span>
                    <span className="brain-name">{brain.brainName}</span>
                  </div>
                  <span className={`brain-status-badge ${brain.isOnline ? 'online' : 'offline'}`}>
                    {brain.isOnline ? 'ONLINE' : 'OFFLINE'}
                  </span>
                </div>

                {brain.isOnline && (
                  <div className="brain-metrics">
                    <div className="metric-row">
                      <div className="metric-item">
                        <div className="metric-label">CPU</div>
                        <div 
                          className="metric-value"
                          style={{ color: getHealthColor(brain.metrics.cpuUsage, { good: 50, warning: 80 }) }}
                        >
                          {brain.metrics.cpuUsage.toFixed(1)}%
                        </div>
                        <div className="metric-bar">
                          <div 
                            className="metric-fill"
                            style={{
                              width: `${brain.metrics.cpuUsage}%`,
                              backgroundColor: getHealthColor(brain.metrics.cpuUsage, { good: 50, warning: 80 })
                            }}
                          />
                        </div>
                      </div>

                      <div className="metric-item">
                        <div className="metric-label">Memory</div>
                        <div 
                          className="metric-value"
                          style={{ color: getHealthColor(brain.metrics.memoryUsage, { good: 70, warning: 85 }) }}
                        >
                          {brain.metrics.memoryUsage.toFixed(1)}%
                        </div>
                        <div className="metric-bar">
                          <div 
                            className="metric-fill"
                            style={{
                              width: `${brain.metrics.memoryUsage}%`,
                              backgroundColor: getHealthColor(brain.metrics.memoryUsage, { good: 70, warning: 85 })
                            }}
                          />
                        </div>
                      </div>
                    </div>

                    <div className="metric-row">
                      <div className="metric-item">
                        <div className="metric-label">Temperature</div>
                        <div 
                          className="metric-value"
                          style={{ color: getHealthColor(brain.metrics.temperature, { good: 60, warning: 80 }) }}
                        >
                          {brain.metrics.temperature.toFixed(1)}°C
                        </div>
                      </div>

                      <div className="metric-item">
                        <div className="metric-label">Disk</div>
                        <div 
                          className="metric-value"
                          style={{ color: getHealthColor(brain.metrics.diskUsage, { good: 70, warning: 90 }) }}
                        >
                          {brain.metrics.diskUsage.toFixed(1)}%
                        </div>
                      </div>
                    </div>

                    <div className="metric-row">
                      <div className="metric-item">
                        <div className="metric-label">Network I/O</div>
                        <div className="metric-value small">
                          ↓{formatBytes(brain.metrics.networkRx)}/s
                          <br />
                          ↑{formatBytes(brain.metrics.networkTx)}/s
                        </div>
                      </div>

                      <div className="metric-item">
                        <div className="metric-label">Uptime</div>
                        <div className="metric-value small">
                          {formatUptime(brain.metrics.uptime)}
                        </div>
                      </div>
                    </div>

                    <div className="metric-row">
                      <div className="metric-item">
                        <div className="metric-label">Processes</div>
                        <div className="metric-value">{brain.metrics.processCount}</div>
                      </div>

                      <div className="metric-item">
                        <div className="metric-label">Load Avg</div>
                        <div className="metric-value small">
                          {brain.metrics.loadAverage.map(load => load.toFixed(2)).join(' ')}
                        </div>
                      </div>
                    </div>
                  </div>
                )}

                {!brain.isOnline && (
                  <div className="brain-offline-message">
                    <div className="offline-icon">⚠️</div>
                    <div className="offline-text">Brain system is offline</div>
                    <div className="offline-time">
                      Last seen: {new Date(brain.lastUpdate).toLocaleTimeString()}
                    </div>
                  </div>
                )}
              </div>
            ))}
          </div>

          {/* Quick Stats Summary */}
          <div className="system-quick-stats">
            <div className="quick-stat">
              <div className="quick-stat-icon">⚡</div>
              <div className="quick-stat-info">
                <div className="quick-stat-label">Avg CPU</div>
                <div className="quick-stat-value">
                  {onlineBrains > 0 ? 
                    (Object.values(systemData).reduce((sum, brain) => sum + brain.metrics.cpuUsage, 0) / onlineBrains).toFixed(1) + '%'
                    : '--'
                  }
                </div>
              </div>
            </div>

            <div className="quick-stat">
              <div className="quick-stat-icon">💾</div>
              <div className="quick-stat-info">
                <div className="quick-stat-label">Avg Memory</div>
                <div className="quick-stat-value">
                  {onlineBrains > 0 ? 
                    (Object.values(systemData).reduce((sum, brain) => sum + brain.metrics.memoryUsage, 0) / onlineBrains).toFixed(1) + '%'
                    : '--'
                  }
                </div>
              </div>
            </div>

            <div className="quick-stat">
              <div className="quick-stat-icon">🌡️</div>
              <div className="quick-stat-info">
                <div className="quick-stat-label">Max Temp</div>
                <div className="quick-stat-value">
                  {onlineBrains > 0 ? 
                    Math.max(...Object.values(systemData).map(brain => brain.metrics.temperature)).toFixed(1) + '°C'
                    : '--'
                  }
                </div>
              </div>
            </div>

            <div className="quick-stat">
              <div className="quick-stat-icon">🔄</div>
              <div className="quick-stat-info">
                <div className="quick-stat-label">Total Processes</div>
                <div className="quick-stat-value">
                  {onlineBrains > 0 ? 
                    Object.values(systemData).reduce((sum, brain) => sum + brain.metrics.processCount, 0)
                    : '--'
                  }
                </div>
              </div>
            </div>
          </div>
        </div>
      ) : (
        <div className="system-widget-loading">
          <div className="loading-spinner-widget"></div>
          <div className="loading-text-widget">Waiting for brain system data...</div>
        </div>
      )}
    </div>
  );
};

export default SystemMonitoringWidget;
