import React, { useRef, useEffect, useState } from 'react';
import './EnvironmentalSensorDashboard.css';

interface SensorData {
  timestamp: number;
  // SHT30 Temperature and Humidity Sensor
  temperature: number;
  humidity: number;
  // MQ-4 Methane Gas Detection
  methaneLevel: number;
  methanePPM: number;
  // MICS-4514 CO/NO2 Sensor
  carbonMonoxide: number;
  nitrogenDioxide: number;
  coPPM: number;
  no2PPM: number;
  // DSM501B PM2.5 Dust Sensor
  pm25: number;
  pm10: number;
  airQualityIndex: number;
  dustDensity: number;
}

interface AirQualityLevel {
  level: string;
  color: string;
  description: string;
}

const EnvironmentalSensorDashboard: React.FC = () => {
  const [sensorData, setSensorData] = useState<SensorData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [useTestData, setUseTestData] = useState(false);
  const rosRef = useRef<any>(null);

  // Air Quality Index calculation and levels
  const getAirQualityLevel = (aqi: number): AirQualityLevel => {
    if (aqi <= 50) return { level: 'Good', color: '#00ff41', description: 'Air quality is satisfactory' };
    if (aqi <= 100) return { level: 'Moderate', color: '#ffaa00', description: 'Air quality is acceptable' };
    if (aqi <= 150) return { level: 'Unhealthy for Sensitive', color: '#ff8800', description: 'Sensitive groups may experience health effects' };
    if (aqi <= 200) return { level: 'Unhealthy', color: '#ff4444', description: 'Everyone may experience health effects' };
    if (aqi <= 300) return { level: 'Very Unhealthy', color: '#8b00ff', description: 'Health alert: everyone may experience serious health effects' };
    return { level: 'Hazardous', color: '#7e0023', description: 'Health warnings of emergency conditions' };
  };

  const getTemperatureColor = (temp: number) => {
    if (temp < 10) return '#00aaff';
    if (temp < 25) return '#00ff41';
    if (temp < 35) return '#ffaa00';
    return '#ff4444';
  };

  const getHumidityColor = (humidity: number) => {
    if (humidity < 30) return '#ff8800';
    if (humidity < 60) return '#00ff41';
    if (humidity < 80) return '#ffaa00';
    return '#ff4444';
  };

  const getGasLevelColor = (level: number, maxSafe: number) => {
    const percentage = (level / maxSafe) * 100;
    if (percentage < 25) return '#00ff41';
    if (percentage < 50) return '#ffaa00';
    if (percentage < 75) return '#ff8800';
    return '#ff4444';
  };

  // Generate mock sensor data for testing
  const generateMockData = (): SensorData => {
    const baseTemp = 22 + Math.random() * 8; // 22-30°C
    const baseHumidity = 45 + Math.random() * 25; // 45-70%
    const baseMethane = Math.random() * 50; // 0-50 ppm
    const baseCO = Math.random() * 10; // 0-10 ppm
    const baseNO2 = Math.random() * 0.2; // 0-0.2 ppm
    const basePM25 = Math.random() * 35; // 0-35 μg/m³
    const basePM10 = basePM25 + Math.random() * 15; // PM10 > PM2.5
    
    // Calculate AQI based on PM2.5 (simplified)
    let aqi = 0;
    if (basePM25 <= 12) aqi = (basePM25 / 12) * 50;
    else if (basePM25 <= 35.4) aqi = 50 + ((basePM25 - 12) / (35.4 - 12)) * 50;
    else if (basePM25 <= 55.4) aqi = 100 + ((basePM25 - 35.4) / (55.4 - 35.4)) * 50;
    else aqi = 150 + Math.min(((basePM25 - 55.4) / 100) * 150, 150);

    return {
      timestamp: Date.now(),
      temperature: baseTemp,
      humidity: baseHumidity,
      methaneLevel: baseMethane,
      methanePPM: baseMethane,
      carbonMonoxide: baseCO,
      nitrogenDioxide: baseNO2,
      coPPM: baseCO,
      no2PPM: baseNO2,
      pm25: basePM25,
      pm10: basePM10,
      airQualityIndex: aqi,
      dustDensity: basePM25 + basePM10
    };
  };

  const startTestData = () => {
    setUseTestData(true);
    setConnectionStatus('Using test data');
    
    // Generate initial data
    setSensorData(generateMockData());
    
    // Update data every 3 seconds
    const interval = setInterval(() => {
      setSensorData(generateMockData());
    }, 3000);
    
    return () => clearInterval(interval);
  };

  const stopTestData = () => {
    setUseTestData(false);
    setSensorData(null);
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
          console.log('Connected to ROS bridge for environmental sensor data');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
          subscribeToSensorTopics(ros);
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

    const subscribeToSensorTopics = (ros: any) => {
      let sensorDataBuffer: Partial<SensorData> = {};
      let hasReceivedData = false;

      // Helper function to update sensor data
      const updateSensorData = () => {
        if (hasReceivedData) {
          setSensorData(prev => ({
            ...prev,
            ...sensorDataBuffer,
            timestamp: Date.now()
          } as SensorData));
        }
      };

      // Subscribe to SHT30 Temperature
      const tempTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/temperature',
        messageType: 'std_msgs/Float32'
      });
      tempTopic.subscribe((message: any) => {
        console.log('Received temperature:', message.data);
        sensorDataBuffer.temperature = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to SHT30 Humidity
      const humidityTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/humidity',
        messageType: 'std_msgs/Float32'
      });
      humidityTopic.subscribe((message: any) => {
        console.log('Received humidity:', message.data);
        sensorDataBuffer.humidity = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to MQ-4 Methane
      const methaneTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/methane',
        messageType: 'std_msgs/Float32'
      });
      methaneTopic.subscribe((message: any) => {
        console.log('Received methane level:', message.data);
        sensorDataBuffer.methaneLevel = message.data;
        sensorDataBuffer.methanePPM = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to MICS-4514 CO
      const coTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/carbon_monoxide',
        messageType: 'std_msgs/Float32'
      });
      coTopic.subscribe((message: any) => {
        console.log('Received CO level:', message.data);
        sensorDataBuffer.carbonMonoxide = message.data;
        sensorDataBuffer.coPPM = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to MICS-4514 NO2
      const no2Topic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/nitrogen_dioxide',
        messageType: 'std_msgs/Float32'
      });
      no2Topic.subscribe((message: any) => {
        console.log('Received NO2 level:', message.data);
        sensorDataBuffer.nitrogenDioxide = message.data;
        sensorDataBuffer.no2PPM = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to DSM501B PM2.5
      const pm25Topic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/pm25',
        messageType: 'std_msgs/Float32'
      });
      pm25Topic.subscribe((message: any) => {
        console.log('Received PM2.5 level:', message.data);
        sensorDataBuffer.pm25 = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to DSM501B PM10
      const pm10Topic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/pm10',
        messageType: 'std_msgs/Float32'
      });
      pm10Topic.subscribe((message: any) => {
        console.log('Received PM10 level:', message.data);
        sensorDataBuffer.pm10 = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      // Subscribe to Air Quality Index
      const aqiTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/environmental/air_quality_index',
        messageType: 'std_msgs/Float32'
      });
      aqiTopic.subscribe((message: any) => {
        console.log('Received AQI:', message.data);
        sensorDataBuffer.airQualityIndex = message.data;
        hasReceivedData = true;
        updateSensorData();
      });

      console.log('Subscribed to all environmental sensor topics');
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  return (
    <div className="environmental-dashboard">
      <div className="environmental-header">
        <div className="environmental-title">
          <h1>🌍 Environmental Sensor Monitoring</h1>
          <div className="connection-status">
            <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
              ●
            </span>
            <span>{connectionStatus}</span>
          </div>
        </div>
      </div>

      {sensorData ? (
        <div className="environmental-content">
          {/* Air Quality Overview */}
          <div className="air-quality-overview">
            <div className="aqi-display">
              <div className="aqi-circle" style={{ borderColor: getAirQualityLevel(sensorData.airQualityIndex).color }}>
                <div className="aqi-value" style={{ color: getAirQualityLevel(sensorData.airQualityIndex).color }}>
                  {Math.round(sensorData.airQualityIndex)}
                </div>
                <div className="aqi-label">AQI</div>
              </div>
              <div className="aqi-info">
                <div className="aqi-level" style={{ color: getAirQualityLevel(sensorData.airQualityIndex).color }}>
                  {getAirQualityLevel(sensorData.airQualityIndex).level}
                </div>
                <div className="aqi-description">
                  {getAirQualityLevel(sensorData.airQualityIndex).description}
                </div>
              </div>
            </div>
          </div>

          {/* Temperature & Humidity Section */}
          <div className="sensor-section">
            <h2>🌡️ SHT30 Temperature & Humidity</h2>
            <div className="temp-humidity-grid">
              <div className="sensor-card temperature-card">
                <div className="sensor-icon">🌡️</div>
                <div className="sensor-value" style={{ color: getTemperatureColor(sensorData.temperature || 0) }}>
                  {(sensorData.temperature || 0).toFixed(1)}°C
                </div>
                <div className="sensor-label">Temperature</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.temperature || 0) / 50) * 100, 100)}%`,
                      backgroundColor: getTemperatureColor(sensorData.temperature || 0)
                    }}
                  />
                </div>
              </div>
              
              <div className="sensor-card humidity-card">
                <div className="sensor-icon">💧</div>
                <div className="sensor-value" style={{ color: getHumidityColor(sensorData.humidity || 0) }}>
                  {(sensorData.humidity || 0).toFixed(1)}%
                </div>
                <div className="sensor-label">Humidity</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${sensorData.humidity || 0}%`,
                      backgroundColor: getHumidityColor(sensorData.humidity || 0)
                    }}
                  />
                </div>
              </div>
            </div>
          </div>

          {/* Gas Detection Section */}
          <div className="sensor-section">
            <h2>⚠️ Gas Detection Sensors</h2>
            <div className="gas-sensors-grid">
              <div className="sensor-card methane-card">
                <div className="sensor-icon">🔥</div>
                <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.methanePPM || 0, 100) }}>
                  {(sensorData.methanePPM || 0).toFixed(1)}
                </div>
                <div className="sensor-unit">ppm</div>
                <div className="sensor-label">Methane (MQ-4)</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.methanePPM || 0) / 100) * 100, 100)}%`,
                      backgroundColor: getGasLevelColor(sensorData.methanePPM || 0, 100)
                    }}
                  />
                </div>
                <div className="safety-threshold">Safe: &lt; 50 ppm</div>
              </div>
              
              <div className="sensor-card co-card">
                <div className="sensor-icon">☠️</div>
                <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.coPPM || 0, 35) }}>
                  {(sensorData.coPPM || 0).toFixed(2)}
                </div>
                <div className="sensor-unit">ppm</div>
                <div className="sensor-label">Carbon Monoxide</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.coPPM || 0) / 35) * 100, 100)}%`,
                      backgroundColor: getGasLevelColor(sensorData.coPPM || 0, 35)
                    }}
                  />
                </div>
                <div className="safety-threshold">Safe: &lt; 9 ppm</div>
              </div>
              
              <div className="sensor-card no2-card">
                <div className="sensor-icon">🌫️</div>
                <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.no2PPM || 0, 1) }}>
                  {(sensorData.no2PPM || 0).toFixed(3)}
                </div>
                <div className="sensor-unit">ppm</div>
                <div className="sensor-label">Nitrogen Dioxide</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.no2PPM || 0) / 1) * 100, 100)}%`,
                      backgroundColor: getGasLevelColor(sensorData.no2PPM || 0, 1)
                    }}
                  />
                </div>
                <div className="safety-threshold">Safe: &lt; 0.053 ppm</div>
              </div>
            </div>
          </div>

          {/* Particulate Matter Section */}
          <div className="sensor-section">
            <h2>🌪️ DSM501B Particulate Matter</h2>
            <div className="particulate-grid">
              <div className="sensor-card pm25-card">
                <div className="sensor-icon">🌫️</div>
                <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.pm25 || 0, 35) }}>
                  {(sensorData.pm25 || 0).toFixed(1)}
                </div>
                <div className="sensor-unit">μg/m³</div>
                <div className="sensor-label">PM2.5</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.pm25 || 0) / 35) * 100, 100)}%`,
                      backgroundColor: getGasLevelColor(sensorData.pm25 || 0, 35)
                    }}
                  />
                </div>
                <div className="safety-threshold">Good: &lt; 12 μg/m³</div>
              </div>
              
              <div className="sensor-card pm10-card">
                <div className="sensor-icon">💨</div>
                <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.pm10 || 0, 150) }}>
                  {(sensorData.pm10 || 0).toFixed(1)}
                </div>
                <div className="sensor-unit">μg/m³</div>
                <div className="sensor-label">PM10</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.pm10 || 0) / 150) * 100, 100)}%`,
                      backgroundColor: getGasLevelColor(sensorData.pm10 || 0, 150)
                    }}
                  />
                </div>
                <div className="safety-threshold">Good: &lt; 54 μg/m³</div>
              </div>
              
              <div className="sensor-card dust-density-card">
                <div className="sensor-icon">🌪️</div>
                <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.dustDensity || 0, 200) }}>
                  {(sensorData.dustDensity || 0).toFixed(1)}
                </div>
                <div className="sensor-unit">μg/m³</div>
                <div className="sensor-label">Total Dust Density</div>
                <div className="sensor-bar">
                  <div 
                    className="sensor-fill"
                    style={{
                      width: `${Math.min(((sensorData.dustDensity || 0) / 200) * 100, 100)}%`,
                      backgroundColor: getGasLevelColor(sensorData.dustDensity || 0, 200)
                    }}
                  />
                </div>
                <div className="safety-threshold">Combined PM2.5 + PM10</div>
              </div>
            </div>
          </div>

          {/* Environmental Status Summary */}
          <div className="sensor-section">
            <h2>📊 Environmental Status Summary</h2>
            <div className="status-summary-grid">
              <div className="status-card">
                <div className="status-icon">🌡️</div>
                <div className="status-label">Temperature Status</div>
                <div className="status-value" style={{ color: getTemperatureColor(sensorData.temperature || 0) }}>
                  {(sensorData.temperature || 0) < 10 ? 'Cold' : 
                   (sensorData.temperature || 0) < 25 ? 'Optimal' : 
                   (sensorData.temperature || 0) < 35 ? 'Warm' : 'Hot'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-icon">💧</div>
                <div className="status-label">Humidity Status</div>
                <div className="status-value" style={{ color: getHumidityColor(sensorData.humidity || 0) }}>
                  {(sensorData.humidity || 0) < 30 ? 'Dry' : 
                   (sensorData.humidity || 0) < 60 ? 'Optimal' : 
                   (sensorData.humidity || 0) < 80 ? 'Humid' : 'Very Humid'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-icon">⚠️</div>
                <div className="status-label">Gas Safety</div>
                <div className="status-value" style={{ 
                  color: ((sensorData.methanePPM || 0) > 50 || (sensorData.coPPM || 0) > 9 || (sensorData.no2PPM || 0) > 0.053) ? '#ff4444' : '#00ff41'
                }}>
                  {((sensorData.methanePPM || 0) > 50 || (sensorData.coPPM || 0) > 9 || (sensorData.no2PPM || 0) > 0.053) ? 'Warning' : 'Safe'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-icon">🌪️</div>
                <div className="status-label">Air Quality</div>
                <div className="status-value" style={{ color: getAirQualityLevel(sensorData.airQualityIndex || 0).color }}>
                  {getAirQualityLevel(sensorData.airQualityIndex || 0).level}
                </div>
              </div>
            </div>
          </div>
        </div>
      ) : (
        <div className="environmental-loading">
          <div className="loading-spinner"></div>
          <div className="loading-text">Waiting for environmental sensor data...</div>
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

export default EnvironmentalSensorDashboard;
