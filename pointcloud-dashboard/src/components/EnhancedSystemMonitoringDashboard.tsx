import React, { useRef, useEffect, useState } from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  Title,
  Tooltip,
  Legend,
  ArcElement,
  Filler
} from 'chart.js';
import { Line, Bar, Doughnut } from 'react-chartjs-2';
import './EnhancedSystemMonitoringDashboard.css';

// Register Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  Title,
  Tooltip,
  Legend,
  ArcElement,
  Filler
);

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

interface HistoricalData {
  timestamp: number;
  time: string;
  cognitionCpu: number;
  perceptionCpu: number;
  cognitionMemory: number;
  perceptionMemory: number;
  cognitionTemp: number;
  perceptionTemp: number;
  cognitionNetwork: number;
  perceptionNetwork: number;
}

const EnhancedSystemMonitoringDashboard: React.FC = () => {
  const [systemData, setSystemData] = useState<{ [key: string]: BrainSystemData }>({});
  const [historicalData, setHistoricalData] = useState<HistoricalData[]>([]);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [selectedTimeRange, setSelectedTimeRange] = useState('1s');
  const [selectedMetric, setSelectedMetric] = useState('cpu');
  const rosRef = useRef<any>(null);
  const cpuChartRef = useRef<any>(null);
  const memoryChartRef = useRef<any>(null);
  const tempChartRef = useRef<any>(null);
  const metricsChartRef = useRef<any>(null);
  const doughnutChartRef = useRef<any>(null);

  useEffect(() => {

    console.log('ookk')
    // Initialize ROS connection
    const initROS = async () => {
      try {
        // @ts-ignore
        const ROSLIB = window.ROSLIB;
        if (!ROSLIB) {
          setConnectionStatus('ROS library not loaded');
          return;
        }

        // Create ROS connection
        const ros = new ROSLIB.Ros({
          url: 'ws://localhost:9090'
        });

        rosRef.current = ros;

        ros.on('connection', () => {
          console.log('Enhanced System Dashboard: Connected to ROS bridge');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
          
          // Subscribe to system monitoring topics after connection
          subscribeToSystemTopics(ros);
        });

        ros.on('error', (error: any) => {
          console.error('Enhanced System Dashboard: ROS connection error:', error);
          setIsConnected(false);
          setConnectionStatus(`Connection error: ${error.message || 'Unknown error'}`);
        });

        ros.on('close', () => {
          console.log('Enhanced System Dashboard: ROS connection closed');
          setIsConnected(false);
          setConnectionStatus('Disconnected from ROS');
        });

        console.log('Enhanced System Dashboard: ROS initialization complete');
      } catch (error) {
        console.error('Enhanced System Dashboard: Failed to initialize ROS:', error);
        setConnectionStatus('Failed to initialize ROS');
      }
    };

    const subscribeToSystemTopics = (ros: any) => {
      console.log('Enhanced System Dashboard: Setting up system monitoring subscriptions...');
      
      // Helper function to process system data
      const processSystemData = (data: any, brainName: string, brainKey: string) => {
        let metrics;
        console.log('data', data);
        // Handle different message formats
        if (typeof data === 'string') {
          try {
            const parsedData = JSON.parse(data);
            console.log(`Enhanced System Dashboard: Parsed ${brainName} data:`, parsedData);
            
            // Handle different JSON structures from brain systems
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
            console.error(`Enhanced System Dashboard: Error parsing ${brainName} JSON data:`, error);
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
      };

      // Subscribe to cognition brain system monitoring (using std_msgs/String)
      const cognitionTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/cognitionbrain/system_monitoring',
        messageType: 'std_msgs/String'
      });

      cognitionTopic.subscribe((message: any) => {
        console.log('Enhanced System Dashboard: Received cognition brain data:', message.data);
        processSystemData(message.data, 'Cognition Brain', 'cognition');
      });

      // Subscribe to perception brain system monitoring (using std_msgs/String)
      const perceptionTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/perceptionbrain/system_monitoring',
        messageType: 'std_msgs/String'
      });

      perceptionTopic.subscribe((message: any) => {
        console.log('Enhanced System Dashboard: Received perception brain data:', message.data);
        processSystemData(message.data, 'Perception Brain', 'perception');
      });

      // Add error handling for perception topic
      perceptionTopic.on('error', (error: any) => {
        console.error('Enhanced System Dashboard: Perception brain topic error:', error);
      });

      console.log('Enhanced System Dashboard: Subscribed to system monitoring topics');
    };

    initROS();

    // Cleanup
    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  // Store historical data for charts - collect data immediately when systemData changes
  useEffect(() => {
    const cognitionData = systemData['cognition'];
    const perceptionData = systemData['perception'];
    
    if (cognitionData || perceptionData) {
      const now = Date.now();
      const timeString = new Date(now).toLocaleTimeString();
      
      const newDataPoint: HistoricalData = {
        timestamp: now,
        time: timeString,
        cognitionCpu: cognitionData?.metrics.cpuUsage || 0,
        perceptionCpu: perceptionData?.metrics.cpuUsage || 0,
        cognitionMemory: cognitionData?.metrics.memoryUsage || 0,
        perceptionMemory: perceptionData?.metrics.memoryUsage || 0,
        cognitionTemp: cognitionData?.metrics.temperature || 0,
        perceptionTemp: perceptionData?.metrics.temperature || 0,
        cognitionNetwork: (cognitionData?.metrics.networkRx || 0) + (cognitionData?.metrics.networkTx || 0),
        perceptionNetwork: (perceptionData?.metrics.networkRx || 0) + (perceptionData?.metrics.networkTx || 0)
      };

      setHistoricalData(prev => {
        // Calculate how many data points to keep based on time range
        const getMaxDataPoints = (timeRange: string) => {
          switch (timeRange) {
            case '1s': return 60;   // 1 minute of 1-second data
            case '5s': return 60;   // 5 minutes of 5-second data
            case '10s': return 60;  // 10 minutes of 10-second data
            case '30s': return 60;  // 30 minutes of 30-second data
            case '1m': return 60;   // 1 hour of 1-minute data
            case '5m': return 60;   // 5 hours of 5-minute data
            case '15m': return 60;  // 15 hours of 15-minute data
            case '1h': return 48;   // 2 days of 1-hour data
            default: return 60;
          }
        };

        const maxDataPoints = getMaxDataPoints(selectedTimeRange);
        
        // Avoid adding duplicate data points (within 500ms)
        const lastDataPoint = prev[prev.length - 1];
        if (lastDataPoint && (now - lastDataPoint.timestamp) < 500) {
          return prev;
        }
        
        const updated = [...prev, newDataPoint];
        const newData = updated.slice(-maxDataPoints);
        
        console.log(`Historical data updated: ${newData.length} points, latest CPU: C=${newDataPoint.cognitionCpu}%, P=${newDataPoint.perceptionCpu}%`);
        
        return newData;
      });
    }
  }, [systemData, selectedTimeRange]);

  // Smoothly update charts when historical data changes
  useEffect(() => {
    if (historicalData.length > 0) {
      // Update all chart instances smoothly without re-creating them
      const updateChart = (chartRef: any) => {
        if (chartRef.current) {
          const chart = chartRef.current;
          if (chart.update) {
            chart.update('none'); // Update without animation for real-time feel
          }
        }
      };

      // Update all charts
      updateChart(cpuChartRef);
      updateChart(memoryChartRef);
      updateChart(tempChartRef);
      updateChart(metricsChartRef);
      updateChart(doughnutChartRef);
    }
  }, [historicalData]);

  const getHealthColor = (value: number, thresholds: { good: number; warning: number }) => {
    if (value <= thresholds.good) return '#00ff41';
    if (value <= thresholds.warning) return '#ffaa00';
    return '#ff4444';
  };

  const formatBytes = (bytes: number) => {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
  };

  const formatUptime = (seconds: number) => {
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    
    if (days > 0) return `${days}d ${hours}h ${minutes}m`;
    if (hours > 0) return `${hours}h ${minutes}m`;
    return `${minutes}m`;
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
  const totalBrains = Object.keys(systemData).length;
  const onlineBrains = Object.values(systemData).filter(brain => brain.isOnline).length;

  // Chart configurations
  const chartOptions = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top' as const,
        labels: {
          color: '#ccc',
          font: {
            size: 12
          }
        }
      },
      tooltip: {
        backgroundColor: 'rgba(0, 0, 0, 0.8)',
        titleColor: '#fff',
        bodyColor: '#ccc',
        borderColor: '#333',
        borderWidth: 1
      }
    },
    scales: {
      x: {
        ticks: {
          color: '#ccc',
          font: {
            size: 10
          }
        },
        grid: {
          color: '#333'
        }
      },
      y: {
        ticks: {
          color: '#ccc',
          font: {
            size: 10
          }
        },
        grid: {
          color: '#333'
        }
      }
    }
  };

  // CPU Usage Line Chart Data
  const cpuChartData = {
    labels: historicalData.map(d => d.time),
    datasets: [
      {
        label: 'Cognition Brain CPU',
        data: historicalData.map(d => d.cognitionCpu),
        borderColor: '#2196F3',
        backgroundColor: 'rgba(33, 150, 243, 0.1)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      },
      {
        label: 'Perception Brain CPU',
        data: historicalData.map(d => d.perceptionCpu),
        borderColor: '#FF6B6B',
        backgroundColor: 'rgba(255, 107, 107, 0.1)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      }
    ]
  };

  // Memory Usage Line Chart Data
  const memoryChartData = {
    labels: historicalData.map(d => d.time),
    datasets: [
      {
        label: 'Cognition Brain Memory',
        data: historicalData.map(d => d.cognitionMemory),
        borderColor: '#2196F3',
        backgroundColor: 'rgba(33, 150, 243, 0.2)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      },
      {
        label: 'Perception Brain Memory',
        data: historicalData.map(d => d.perceptionMemory),
        borderColor: '#FF6B6B',
        backgroundColor: 'rgba(255, 107, 107, 0.2)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      }
    ]
  };

  // Temperature Line Chart Data
  const temperatureChartData = {
    labels: historicalData.map(d => d.time),
    datasets: [
      {
        label: 'Cognition Brain Temp',
        data: historicalData.map(d => d.cognitionTemp),
        borderColor: '#00ff41',
        backgroundColor: 'rgba(0, 255, 65, 0.1)',
        borderWidth: 3,
        fill: false,
        tension: 0.4
      },
      {
        label: 'Perception Brain Temp',
        data: historicalData.map(d => d.perceptionTemp),
        borderColor: '#ffaa00',
        backgroundColor: 'rgba(255, 170, 0, 0.1)',
        borderWidth: 3,
        fill: false,
        tension: 0.4
      }
    ]
  };

  // Current Metrics Bar Chart Data
  const currentMetricsData = {
    labels: Object.values(systemData).map(brain => brain.brainName),
    datasets: [
      {
        label: 'CPU %',
        data: Object.values(systemData).map(brain => brain.metrics.cpuUsage),
        backgroundColor: '#2196F3',
        borderColor: '#2196F3',
        borderWidth: 1
      },
      {
        label: 'Memory %',
        data: Object.values(systemData).map(brain => brain.metrics.memoryUsage),
        backgroundColor: '#FF6B6B',
        borderColor: '#FF6B6B',
        borderWidth: 1
      },
      {
        label: 'Temperature °C',
        data: Object.values(systemData).map(brain => brain.metrics.temperature),
        backgroundColor: '#00ff41',
        borderColor: '#00ff41',
        borderWidth: 1
      }
    ]
  };

  // CPU Distribution Doughnut Chart Data
  const cpuDistributionData = {
    labels: Object.values(systemData).map(brain => brain.brainName),
    datasets: [
      {
        data: Object.values(systemData).map(brain => brain.metrics.cpuUsage),
        backgroundColor: ['#2196F3', '#FF6B6B'],
        borderColor: ['#2196F3', '#FF6B6B'],
        borderWidth: 2
      }
    ]
  };

  return (
    <div className="enhanced-system-monitoring-dashboard">
      {/* Header */}
      <div className="enhanced-dashboard-header">
        <div className="header-title">
          <h1>🧠 Enhanced Brain System Monitoring</h1>
          <p>Real-time monitoring with advanced analytics for Cognition and Perception Brain systems</p>
        </div>
        
        <div className="header-controls">
          <div className="connection-status">
            <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>●</span>
            <span className="status-text">{connectionStatus}</span>
          </div>
          
          <div className="time-range-selector">
            <label>Time Range:</label>
            <select 
              value={selectedTimeRange} 
              onChange={(e) => setSelectedTimeRange(e.target.value)}
              className="time-select"
            >
              <option value="1s">1 Second</option>
              <option value="5s">5 Seconds</option>
              <option value="10s">10 Seconds</option>
              <option value="30s">30 Seconds</option>
              <option value="1m">1 Minute</option>
              <option value="5m">5 Minutes</option>
              <option value="15m">15 Minutes</option>
              <option value="1h">1 Hour</option>
            </select>
          </div>

          <div className="metric-selector">
            <label>Focus Metric:</label>
            <select 
              value={selectedMetric} 
              onChange={(e) => setSelectedMetric(e.target.value)}
              className="metric-select"
            >
              <option value="cpu">CPU Usage</option>
              <option value="memory">Memory Usage</option>
              <option value="temperature">Temperature</option>
              <option value="network">Network Activity</option>
            </select>
          </div>
        </div>
      </div>

      {/* Separate Brain Status Sections */}
      <div className="brain-status-sections">
        {/* Cognition Brain Section */}
        <div className="brain-section cognition-section">
          <div className="brain-section-header">
            <div className="brain-title">
              <span className="brain-icon">🧠</span>
              <h2>Cognition Brain</h2>
              <span className={`brain-status-badge ${systemData['cognition']?.isOnline ? 'online' : 'offline'}`}>
                {systemData['cognition']?.isOnline ? 'ONLINE' : 'OFFLINE'}
              </span>
            </div>
          </div>
          
          <div className="brain-metrics-cards">
            <div className="metric-card">
              <div className="metric-icon">⚡</div>
              <div className="metric-info">
                <div className="metric-label">CPU Usage</div>
                <div className="metric-value" style={{ 
                  color: systemData['cognition'] ? getHealthColor(systemData['cognition'].metrics.cpuUsage, { good: 50, warning: 80 }) : '#888'
                }}>
                  {systemData['cognition'] ? `${systemData['cognition'].metrics.cpuUsage.toFixed(1)}%` : '--'}
                </div>
              </div>
            </div>

            <div className="metric-card">
              <div className="metric-icon">💾</div>
              <div className="metric-info">
                <div className="metric-label">Memory Usage</div>
                <div className="metric-value" style={{ 
                  color: systemData['cognition'] ? getHealthColor(systemData['cognition'].metrics.memoryUsage, { good: 70, warning: 85 }) : '#888'
                }}>
                  {systemData['cognition'] ? `${systemData['cognition'].metrics.memoryUsage.toFixed(1)}%` : '--'}
                </div>
              </div>
            </div>

            <div className="metric-card">
              <div className="metric-icon">🌡️</div>
              <div className="metric-info">
                <div className="metric-label">Temperature</div>
                <div className="metric-value" style={{ 
                  color: systemData['cognition'] ? getHealthColor(systemData['cognition'].metrics.temperature, { good: 60, warning: 80 }) : '#888'
                }}>
                  {systemData['cognition'] ? `${systemData['cognition'].metrics.temperature.toFixed(1)}°C` : '--'}
                </div>
              </div>
            </div>

            <div className="metric-card">
              <div className="metric-icon">💽</div>
              <div className="metric-info">
                <div className="metric-label">Disk Usage</div>
                <div className="metric-value" style={{ 
                  color: systemData['cognition'] ? getHealthColor(systemData['cognition'].metrics.diskUsage, { good: 70, warning: 90 }) : '#888'
                }}>
                  {systemData['cognition'] ? `${systemData['cognition'].metrics.diskUsage.toFixed(1)}%` : '--'}
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Perception Brain Section */}
        <div className="brain-section perception-section">
          <div className="brain-section-header">
            <div className="brain-title">
              <span className="brain-icon">👁️</span>
              <h2>Perception Brain</h2>
              <span className={`brain-status-badge ${systemData['perception']?.isOnline ? 'online' : 'offline'}`}>
                {systemData['perception']?.isOnline ? 'ONLINE' : 'OFFLINE'}
              </span>
            </div>
          </div>
          
          <div className="brain-metrics-cards">
            <div className="metric-card">
              <div className="metric-icon">⚡</div>
              <div className="metric-info">
                <div className="metric-label">CPU Usage</div>
                <div className="metric-value" style={{ 
                  color: systemData['perception'] ? getHealthColor(systemData['perception'].metrics.cpuUsage, { good: 50, warning: 80 }) : '#888'
                }}>
                  {systemData['perception'] ? `${systemData['perception'].metrics.cpuUsage.toFixed(1)}%` : '--'}
                </div>
              </div>
            </div>

            <div className="metric-card">
              <div className="metric-icon">💾</div>
              <div className="metric-info">
                <div className="metric-label">Memory Usage</div>
                <div className="metric-value" style={{ 
                  color: systemData['perception'] ? getHealthColor(systemData['perception'].metrics.memoryUsage, { good: 70, warning: 85 }) : '#888'
                }}>
                  {systemData['perception'] ? `${systemData['perception'].metrics.memoryUsage.toFixed(1)}%` : '--'}
                </div>
              </div>
            </div>

            <div className="metric-card">
              <div className="metric-icon">🌡️</div>
              <div className="metric-info">
                <div className="metric-label">Temperature</div>
                <div className="metric-value" style={{ 
                  color: systemData['perception'] ? getHealthColor(systemData['perception'].metrics.temperature, { good: 60, warning: 80 }) : '#888'
                }}>
                  {systemData['perception'] ? `${systemData['perception'].metrics.temperature.toFixed(1)}°C` : '--'}
                </div>
              </div>
            </div>

            <div className="metric-card">
              <div className="metric-icon">💽</div>
              <div className="metric-info">
                <div className="metric-label">Disk Usage</div>
                <div className="metric-value" style={{ 
                  color: systemData['perception'] ? getHealthColor(systemData['perception'].metrics.diskUsage, { good: 70, warning: 90 }) : '#888'
                }}>
                  {systemData['perception'] ? `${systemData['perception'].metrics.diskUsage.toFixed(1)}%` : '--'}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Main Charts Grid */}
      <div className="charts-grid">
        {/* Real-time CPU Chart */}
        <div className="chart-container large">
          <div className="chart-header">
            <h3>📈 Real-time CPU Usage</h3>
            <div className="chart-legend">
              <span className="legend-item cognition">● Cognition Brain</span>
              <span className="legend-item perception">● Perception Brain</span>
            </div>
          </div>
          <div style={{ height: '300px' }}>
            <Line 
              ref={cpuChartRef}
              data={cpuChartData} 
              options={chartOptions} 
            />
          </div>
        </div>

        {/* Memory Usage Chart */}
        <div className="chart-container medium">
          <div className="chart-header">
            <h3>💾 Memory Usage Trends</h3>
          </div>
          <div style={{ height: '250px' }}>
            <Line 
              ref={memoryChartRef}
              data={memoryChartData} 
              options={chartOptions} 
            />
          </div>
        </div>

        {/* Temperature Monitoring */}
        <div className="chart-container medium">
          <div className="chart-header">
            <h3>🌡️ Temperature Monitoring</h3>
          </div>
          <div style={{ height: '250px' }}>
            <Line 
              ref={tempChartRef}
              data={temperatureChartData} 
              options={chartOptions} 
            />
          </div>
        </div>

        {/* Current Metrics Bar Chart */}
        <div className="chart-container medium">
          <div className="chart-header">
            <h3>📊 Current System Metrics</h3>
          </div>
          <div style={{ height: '250px' }}>
            <Bar 
              ref={metricsChartRef}
              data={currentMetricsData} 
              options={chartOptions} 
            />
          </div>
        </div>

        {/* CPU Usage Distribution */}
        <div className="chart-container small">
          <div className="chart-header">
            <h3>🥧 CPU Distribution</h3>
          </div>
          <div style={{ height: '200px' }}>
            <Doughnut 
              ref={doughnutChartRef}
              data={cpuDistributionData} 
              options={{
              ...chartOptions,
              plugins: {
                ...chartOptions.plugins,
                legend: {
                  position: 'bottom' as const,
                  labels: {
                    color: '#ccc',
                    font: {
                      size: 10
                    }
                  }
                }
              }
            }} />
          </div>
        </div>

        {/* Performance Overview */}
        <div className="chart-container small">
          <div className="chart-header">
            <h3>🎯 Performance Overview</h3>
          </div>
          <div style={{ height: '200px', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <div style={{ textAlign: 'center', color: '#ccc' }}>
              <div style={{ fontSize: '48px', marginBottom: '10px' }}>
                {systemHealth.status === 'healthy' ? '✅' : systemHealth.status === 'warning' ? '⚠️' : '❌'}
              </div>
              <div style={{ fontSize: '18px', fontWeight: 'bold', color: systemHealth.color }}>
                {systemHealth.status.toUpperCase()}
              </div>
              <div style={{ fontSize: '14px', marginTop: '5px' }}>
                System Performance
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Brain Details Section */}
      <div className="brain-details-section">
        <h2>🧠 Brain System Details</h2>
        <div className="brain-cards-grid">
          {Object.entries(systemData).map(([key, brainData]) => (
            <div key={key} className={`brain-detail-card ${brainData.isOnline ? 'online' : 'offline'}`}>
              <div className="brain-card-header">
                <div className="brain-info">
                  <h3>{brainData.brainName}</h3>
                  <span className={`status-badge ${brainData.isOnline ? 'online' : 'offline'}`}>
                    {brainData.isOnline ? 'ONLINE' : 'OFFLINE'}
                  </span>
                </div>
                <div className="brain-icon-large">
                  {key === 'cognition' ? '🧠' : '👁️'}
                </div>
              </div>

              {brainData.isOnline && (
                <div className="brain-metrics-grid">
                  <div className="metric-box">
                    <div className="metric-label">CPU Usage</div>
                    <div className="metric-value" style={{ color: getHealthColor(brainData.metrics.cpuUsage, { good: 50, warning: 80 }) }}>
                      {brainData.metrics.cpuUsage.toFixed(1)}%
                    </div>
                    <div className="metric-bar">
                      <div 
                        className="metric-fill"
                        style={{
                          width: `${brainData.metrics.cpuUsage}%`,
                          backgroundColor: getHealthColor(brainData.metrics.cpuUsage, { good: 50, warning: 80 })
                        }}
                      />
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Memory Usage</div>
                    <div className="metric-value" style={{ color: getHealthColor(brainData.metrics.memoryUsage, { good: 70, warning: 85 }) }}>
                      {brainData.metrics.memoryUsage.toFixed(1)}%
                    </div>
                    <div className="metric-bar">
                      <div 
                        className="metric-fill"
                        style={{
                          width: `${brainData.metrics.memoryUsage}%`,
                          backgroundColor: getHealthColor(brainData.metrics.memoryUsage, { good: 70, warning: 85 })
                        }}
                      />
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Temperature</div>
                    <div className="metric-value" style={{ color: getHealthColor(brainData.metrics.temperature, { good: 60, warning: 80 }) }}>
                      {brainData.metrics.temperature.toFixed(1)}°C
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Disk Usage</div>
                    <div className="metric-value" style={{ color: getHealthColor(brainData.metrics.diskUsage, { good: 70, warning: 90 }) }}>
                      {brainData.metrics.diskUsage.toFixed(1)}%
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Network I/O</div>
                    <div className="metric-value">
                      ↓{formatBytes(brainData.metrics.networkRx)}/s
                      <br />
                      ↑{formatBytes(brainData.metrics.networkTx)}/s
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Uptime</div>
                    <div className="metric-value">
                      {formatUptime(brainData.metrics.uptime)}
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Processes</div>
                    <div className="metric-value">
                      {brainData.metrics.processCount}
                    </div>
                  </div>

                  <div className="metric-box">
                    <div className="metric-label">Load Average</div>
                    <div className="metric-value">
                      {brainData.metrics.loadAverage.map(load => load.toFixed(2)).join(' ')}
                    </div>
                  </div>
                </div>
              )}

              {!brainData.isOnline && (
                <div className="brain-offline-message">
                  <div className="offline-icon">⚠️</div>
                  <div className="offline-text">Brain system is offline</div>
                  <div className="offline-time">
                    Last seen: {new Date(brainData.lastUpdate).toLocaleString()}
                  </div>
                </div>
              )}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

export default EnhancedSystemMonitoringDashboard;
