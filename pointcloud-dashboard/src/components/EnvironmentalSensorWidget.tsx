import React, { useRef, useEffect, useState } from 'react';
import './EnvironmentalSensorWidget.css';

interface SensorData {
  timestamp: number;
  temperature: number;
  humidity: number;
  methanePPM: number;
  coPPM: number;
  no2PPM: number;
  pm25: number;
  pm10: number;
  airQualityIndex: number;
}

const EnvironmentalSensorWidget: React.FC = () => {
  const [sensorData, setSensorData] = useState<SensorData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [useTestData, setUseTestData] = useState(false);
  const rosRef = useRef<any>(null);

  const getAirQualityLevel = (aqi: number) => {
    if (aqi <= 50) return { level: 'Good', color: '#00ff41' };
    if (aqi <= 100) return { level: 'Moderate', color: '#ffaa00' };
    if (aqi <= 150) return { level: 'Unhealthy for Sensitive', color: '#ff8800' };
    if (aqi <= 200) return { level: 'Unhealthy', color: '#ff4444' };
    if (aqi <= 300) return { level: 'Very Unhealthy', color: '#8b00ff' };
    return { level: 'Hazardous', color: '#7e0023' };
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

  const getGasLevelColor = (value: number, type: 'methane' | 'co' | 'no2') => {
    const thresholds = {
      methane: { safe: 50, warning: 100 },
      co: { safe: 9, warning: 35 },
      no2: { safe: 0.053, warning: 0.2 }
    };
    
    const threshold = thresholds[type];
    if (value < threshold.safe) return '#00ff41';
    if (value < threshold.warning) return '#ffaa00';
    return '#ff4444';
  };

  // Generate mock sensor data for testing
  const generateMockData = (): SensorData => {
    const temp = 15 + Math.random() * 25; // 15-40°C
    const humidity = 30 + Math.random() * 50; // 30-80%
    const pm25 = Math.random() * 50; // 0-50 μg/m³
    const pm10 = pm25 + Math.random() * 20; // PM10 > PM2.5
    
    // Calculate AQI based on PM2.5
    let aqi = 0;
    if (pm25 <= 12) aqi = (pm25 / 12) * 50;
    else if (pm25 <= 35.4) aqi = 50 + ((pm25 - 12) / (35.4 - 12)) * 50;
    else if (pm25 <= 55.4) aqi = 100 + ((pm25 - 35.4) / (55.4 - 35.4)) * 50;
    else if (pm25 <= 150.4) aqi = 150 + ((pm25 - 55.4) / (150.4 - 55.4)) * 50;
    else if (pm25 <= 250.4) aqi = 200 + ((pm25 - 150.4) / (250.4 - 150.4)) * 100;
    else aqi = 300 + ((pm25 - 250.4) / (350.4 - 250.4)) * 100;

    return {
      timestamp: Date.now(),
      temperature: temp,
      humidity: humidity,
      methanePPM: Math.random() * 200,
      coPPM: Math.random() * 50,
      no2PPM: Math.random() * 0.5,
      pm25: pm25,
      pm10: pm10,
      airQualityIndex: Math.round(aqi)
    };
  };

  const startTestData = () => {
    setUseTestData(true);
    setSensorData(generateMockData());
    
    const interval = setInterval(() => {
      setSensorData(generateMockData());
    }, 3000);
    
    return () => clearInterval(interval);
  };

  const stopTestData = () => {
    setUseTestData(false);
    setSensorData(null);
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
          subscribeToEnvironmentalTopics(ros);
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

    const subscribeToEnvironmentalTopics = (ros: any) => {
      let dataBuffer: Partial<SensorData> = {};
      let hasReceivedData = false;

      const updateSensorData = () => {
        if (hasReceivedData) {
          setSensorData(prev => ({
            ...prev,
            ...dataBuffer,
            timestamp: Date.now()
          } as SensorData));
        }
      };

      // Subscribe to environmental topics
      const topics = [
        { name: '/environmental/temperature', field: 'temperature' },
        { name: '/environmental/humidity', field: 'humidity' },
        { name: '/environmental/methane', field: 'methanePPM' },
        { name: '/environmental/carbon_monoxide', field: 'coPPM' },
        { name: '/environmental/nitrogen_dioxide', field: 'no2PPM' },
        { name: '/environmental/pm25', field: 'pm25' },
        { name: '/environmental/pm10', field: 'pm10' },
        { name: '/environmental/air_quality_index', field: 'airQualityIndex' }
      ];

      topics.forEach(({ name, field }) => {
        const topic = new (window as any).ROSLIB.Topic({
          ros: ros,
          name: name,
          messageType: 'std_msgs/Float32'
        });
        
        topic.subscribe((message: any) => {
          (dataBuffer as any)[field] = message.data;
          hasReceivedData = true;
          updateSensorData();
        });
      });
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  if (!sensorData) {
    return (
      <div className="env-widget">
        <div className="env-loading">
          <div className="loading-circle"></div>
          <p>Waiting for sensor data...</p>
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

  const aqiInfo = getAirQualityLevel(sensorData.airQualityIndex);

  return (
    <div className="env-widget">
      {/* Main AQI Display */}
      <div className="aqi-main">
        <div 
          className="aqi-circle"
          style={{ borderColor: aqiInfo.color }}
        >
          <div className="aqi-number" style={{ color: aqiInfo.color }}>
            {sensorData.airQualityIndex}
          </div>
          <div className="aqi-text">AQI</div>
        </div>
        <div className="aqi-status">
          <div className="aqi-level" style={{ color: aqiInfo.color }}>
            {aqiInfo.level}
          </div>
        </div>
      </div>

      {/* Sensor Values Grid */}
      <div className="sensors-grid">
        <div className="sensor-item">
          <div className="sensor-icon">🌡️</div>
          <div className="sensor-value" style={{ color: getTemperatureColor(sensorData.temperature) }}>
            {sensorData.temperature.toFixed(1)}°C
          </div>
          <div className="sensor-name">Temperature</div>
        </div>

        <div className="sensor-item">
          <div className="sensor-icon">💧</div>
          <div className="sensor-value" style={{ color: getHumidityColor(sensorData.humidity) }}>
            {sensorData.humidity.toFixed(1)}%
          </div>
          <div className="sensor-name">Humidity</div>
        </div>

        <div className="sensor-item">
          <div className="sensor-icon">🌪️</div>
          <div className="sensor-value" style={{ color: sensorData.pm25 > 35 ? '#ff4444' : '#00ff41' }}>
            {sensorData.pm25.toFixed(1)}
          </div>
          <div className="sensor-name">PM2.5 μg/m³</div>
        </div>

        <div className="sensor-item">
          <div className="sensor-icon">⚠️</div>
          <div className="sensor-value" style={{ color: getGasLevelColor(sensorData.methanePPM, 'methane') }}>
            {sensorData.methanePPM.toFixed(0)}
          </div>
          <div className="sensor-name">Methane ppm</div>
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

export default EnvironmentalSensorWidget;
