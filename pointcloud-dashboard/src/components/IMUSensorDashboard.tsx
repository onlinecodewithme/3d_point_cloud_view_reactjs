import React, { useRef, useEffect, useState } from 'react';
import './IMUSensorDashboard.css';

interface IMUData {
  timestamp: number;
  orientation: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
  angularVelocity: {
    x: number;
    y: number;
    z: number;
  };
  linearAcceleration: {
    x: number;
    y: number;
    z: number;
  };
  magnetometer: {
    x: number;
    y: number;
    z: number;
  };
  gravity: {
    x: number;
    y: number;
    z: number;
  };
  orientationCovariance: number[];
  angularVelocityCovariance: number[];
  linearAccelerationCovariance: number[];
  valid: boolean;
}

interface CompassData {
  magneticHeading: number;
  tiltCompensatedHeading: number;
  magneticDeclination: number;
  trueHeading: number;
  magneticFieldStrength: number;
  tiltAngle: number;
}

interface EulerAngles {
  roll: number;
  pitch: number;
  yaw: number;
}

interface ThresholdSettings {
  roll: { warning: number; critical: number };
  pitch: { warning: number; critical: number };
  yaw: { warning: number; critical: number };
  acceleration: { warning: number; critical: number };
  angularVelocity: { warning: number; critical: number };
}

const IMUSensorDashboard: React.FC = () => {
  const [imuData, setImuData] = useState<IMUData | null>(null);
  const [eulerAngles, setEulerAngles] = useState<EulerAngles | null>(null);
  const [compassData, setCompassData] = useState<CompassData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [useTestData, setUseTestData] = useState(false);
  const [dataHistory, setDataHistory] = useState<IMUData[]>([]);
  const [showThresholdSettings, setShowThresholdSettings] = useState(false);
  const [forceUpdate, setForceUpdate] = useState(0); // Force re-render counter
  const [thresholds, setThresholds] = useState<ThresholdSettings>({
    roll: { warning: 30, critical: 60 },
    pitch: { warning: 30, critical: 60 },
    yaw: { warning: 90, critical: 180 },
    acceleration: { warning: 5, critical: 10 },
    angularVelocity: { warning: 1, critical: 2 }
  });
  const rosRef = useRef<any>(null);
  const testDataIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const maxHistoryLength = 100;

  // Convert quaternion to Euler angles
  const quaternionToEuler = (q: { x: number; y: number; z: number; w: number }): EulerAngles => {
    const { x, y, z, w } = q;
    
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return {
      roll: roll * 180 / Math.PI,
      pitch: pitch * 180 / Math.PI,
      yaw: yaw * 180 / Math.PI
    };
  };

  // Calculate magnetometer-based compass heading with tilt compensation
  const calculateCompassHeading = (magnetometer: { x: number; y: number; z: number }, 
                                   gravity: { x: number; y: number; z: number }): CompassData => {
    const { x: mx, y: my, z: mz } = magnetometer;
    const { x: gx, y: gy, z: gz } = gravity;
    
    // Normalize gravity vector
    const gMag = Math.sqrt(gx * gx + gy * gy + gz * gz);
    const gxNorm = gx / gMag;
    const gyNorm = gy / gMag;
    const gzNorm = gz / gMag;
    
    // Calculate roll and pitch from gravity
    const roll = Math.atan2(gyNorm, gzNorm);
    const pitch = Math.atan2(-gxNorm, Math.sqrt(gyNorm * gyNorm + gzNorm * gzNorm));
    
    // Tilt compensation for magnetometer
    const cosRoll = Math.cos(roll);
    const sinRoll = Math.sin(roll);
    const cosPitch = Math.cos(pitch);
    const sinPitch = Math.sin(pitch);
    
    // Apply tilt compensation to magnetometer readings
    const mxComp = mx * cosPitch + mz * sinPitch;
    const myComp = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
    
    // Calculate magnetic heading (0° = North, 90° = East, 180° = South, 270° = West)
    let magneticHeading = Math.atan2(myComp, mxComp) * 180 / Math.PI;
    
    // Normalize to 0-360 degrees
    if (magneticHeading < 0) {
      magneticHeading += 360;
    }
    
    // Calculate tilt-compensated heading (same as magnetic for now)
    const tiltCompensatedHeading = magneticHeading;
    
    // Magnetic declination (adjust based on location - this is a placeholder)
    const magneticDeclination = 0; // Set to your local magnetic declination
    
    // True heading = magnetic heading + declination
    let trueHeading = magneticHeading + magneticDeclination;
    if (trueHeading >= 360) trueHeading -= 360;
    if (trueHeading < 0) trueHeading += 360;
    
    // Calculate magnetic field strength
    const magneticFieldStrength = Math.sqrt(mx * mx + my * my + mz * mz);
    
    // Calculate tilt angle
    const tiltAngle = Math.acos(Math.abs(gzNorm)) * 180 / Math.PI;
    
    return {
      magneticHeading,
      tiltCompensatedHeading,
      magneticDeclination,
      trueHeading,
      magneticFieldStrength,
      tiltAngle
    };
  };

  // Generate mock IMU data for testing
  const generateMockData = (): IMUData => {
    const time = Date.now() / 1000;
    
    // Simulate some realistic IMU motion
    const roll = Math.sin(time * 0.1) * 15; // ±15 degrees
    const pitch = Math.cos(time * 0.15) * 10; // ±10 degrees
    const yaw = time * 5 % 360; // Continuous rotation
    
    // Convert Euler to quaternion for realistic data
    const cr = Math.cos(roll * Math.PI / 360);
    const sr = Math.sin(roll * Math.PI / 360);
    const cp = Math.cos(pitch * Math.PI / 360);
    const sp = Math.sin(pitch * Math.PI / 360);
    const cy = Math.cos(yaw * Math.PI / 360);
    const sy = Math.sin(yaw * Math.PI / 360);

    const orientation = {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };

    return {
      timestamp: Date.now(),
      orientation,
      angularVelocity: {
        x: (Math.random() - 0.5) * 0.2, // ±0.1 rad/s
        y: (Math.random() - 0.5) * 0.2,
        z: (Math.random() - 0.5) * 0.4
      },
      linearAcceleration: {
        x: (Math.random() - 0.5) * 2, // ±1 m/s²
        y: (Math.random() - 0.5) * 2,
        z: 9.81 + (Math.random() - 0.5) * 0.5 // Gravity ± noise
      },
      magnetometer: {
        x: -1.25 + (Math.random() - 0.5) * 0.1, // Mock magnetometer data
        y: 0.0625 + (Math.random() - 0.5) * 0.05,
        z: -5 + (Math.random() - 0.5) * 0.2
      },
      gravity: {
        x: 0 + (Math.random() - 0.5) * 0.1,
        y: 0 + (Math.random() - 0.5) * 0.1,
        z: 9.81 + (Math.random() - 0.5) * 0.1
      },
      orientationCovariance: Array(9).fill(0).map(() => Math.random() * 0.01),
      angularVelocityCovariance: Array(9).fill(0).map(() => Math.random() * 0.001),
      linearAccelerationCovariance: Array(9).fill(0).map(() => Math.random() * 0.1),
      valid: true
    };
  };

  const startTestData = () => {
    setUseTestData(true);
    setConnectionStatus('Using test data');
    
    // Clear any existing interval
    if (testDataIntervalRef.current) {
      clearInterval(testDataIntervalRef.current);
    }
    
    // Generate initial data
    const initialData = generateMockData();
    setImuData(initialData);
    setEulerAngles(quaternionToEuler(initialData.orientation));
    setCompassData(calculateCompassHeading(initialData.magnetometer, initialData.gravity));
    
    // Update data every 50ms for smooth visualization
    testDataIntervalRef.current = setInterval(() => {
      const newData = generateMockData();
      const compass = calculateCompassHeading(newData.magnetometer, newData.gravity);
      
      setImuData(newData);
      setEulerAngles(quaternionToEuler(newData.orientation));
      setCompassData(compass);
      setForceUpdate(prev => prev + 1);
      
      // Update history
      setDataHistory(prev => {
        const newHistory = [...prev, newData];
        return newHistory.slice(-maxHistoryLength);
      });
    }, 50);
  };

  const stopTestData = () => {
    setUseTestData(false);
    
    // Clear the test data interval
    if (testDataIntervalRef.current) {
      clearInterval(testDataIntervalRef.current);
      testDataIntervalRef.current = null;
    }
    
    setImuData(null);
    setEulerAngles(null);
    setDataHistory([]);
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
          console.log('Connected to ROS bridge for IMU data');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
          subscribeToIMUTopic(ros);
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

    const subscribeToIMUTopic = (ros: any) => {
      // Subscribe to IMU JSON data
      const imuTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/perception_brain/imu/json',
        messageType: 'std_msgs/String'
      });

      imuTopic.subscribe((message: any) => {
        console.log('🔍 [IMU] Received raw message:', message);
        
        try {
          // Parse the JSON data from the string message
          const jsonData = JSON.parse(message.data);
          console.log('📊 [IMU] Parsed JSON structure:', JSON.stringify(jsonData, null, 2));
          
          // Log available fields
          console.log('🔑 [IMU] Available fields:', Object.keys(jsonData));
          
          // Extract IMU data from the JSON structure
          const newData: IMUData = {
            timestamp: jsonData.timestamp * 1000 || Date.now(), // Convert to milliseconds
            orientation: {
              x: jsonData.orientation?.x || 0,
              y: jsonData.orientation?.y || 0,
              z: jsonData.orientation?.z || 0,
              w: jsonData.orientation?.w || 1
            },
            angularVelocity: {
              x: jsonData.gyroscope?.x || 0,
              y: jsonData.gyroscope?.y || 0,
              z: jsonData.gyroscope?.z || 0
            },
            linearAcceleration: {
              x: jsonData.accelerometer?.x || 0,
              y: jsonData.accelerometer?.y || 0,
              z: jsonData.accelerometer?.z || 0
            },
            magnetometer: {
              x: jsonData.magnetometer?.x || 0,
              y: jsonData.magnetometer?.y || 0,
              z: jsonData.magnetometer?.z || 0
            },
            gravity: {
              x: jsonData.gravity?.x || 0,
              y: jsonData.gravity?.y || 0,
              z: jsonData.gravity?.z || 9.81
            },
            orientationCovariance: [],
            angularVelocityCovariance: [],
            linearAccelerationCovariance: [],
            valid: jsonData.valid !== false // Default to true unless explicitly false
          };

          console.log('✅ [IMU] Processed data:', {
            timestamp: newData.timestamp,
            orientation: newData.orientation,
            angularVelocity: newData.angularVelocity,
            linearAcceleration: newData.linearAcceleration
          });

          // Calculate compass data from magnetometer and gravity
          const compass = calculateCompassHeading(newData.magnetometer, newData.gravity);
          
          // Force re-render by using functional updates and force update counter
          setImuData(() => newData);
          setEulerAngles(() => quaternionToEuler(newData.orientation));
          setCompassData(() => compass);
          setForceUpdate(prev => prev + 1);
          
          // Update history
          setDataHistory(prev => {
            const newHistory = [...prev, newData];
            return newHistory.slice(-maxHistoryLength);
          });
        } catch (error) {
          console.error('❌ [IMU] Error parsing JSON data:', error);
          console.error('❌ [IMU] Raw message data:', message.data);
        }
      });

      console.log('Subscribed to IMU JSON topic: /perception_brain/imu/json');
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
      // Clean up test data interval on unmount
      if (testDataIntervalRef.current) {
        clearInterval(testDataIntervalRef.current);
      }
    };
  }, []);

  // Additional cleanup effect for test data
  useEffect(() => {
    return () => {
      if (testDataIntervalRef.current) {
        clearInterval(testDataIntervalRef.current);
      }
    };
  }, [useTestData]);

  const getAccelerationMagnitude = (acc: { x: number; y: number; z: number }) => {
    return Math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
  };

  const getAngularVelocityMagnitude = (vel: { x: number; y: number; z: number }) => {
    return Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
  };

  const getValueColor = (value: number, type: 'roll' | 'pitch' | 'yaw' | 'acceleration' | 'angularVelocity') => {
    const absValue = Math.abs(value);
    const threshold = thresholds[type];
    
    if (absValue < threshold.warning) return '#00ff41'; // Green
    if (absValue < threshold.critical) return '#ffaa00'; // Orange
    return '#ff4444'; // Red
  };

  const updateThreshold = (type: keyof ThresholdSettings, level: 'warning' | 'critical', value: number) => {
    setThresholds(prev => ({
      ...prev,
      [type]: {
        ...prev[type],
        [level]: value
      }
    }));
  };

  const renderThresholdSettings = () => {
    if (!showThresholdSettings) return null;

    return (
      <div className="threshold-settings-overlay">
        <div className="threshold-settings-panel">
          <div className="threshold-header">
            <h3>⚙️ Threshold Settings</h3>
            <button 
              className="close-button"
              onClick={() => setShowThresholdSettings(false)}
            >
              ✕
            </button>
          </div>
          
          <div className="threshold-groups">
            {Object.entries(thresholds).map(([key, values]) => (
              <div key={key} className="threshold-group">
                <h4>{key.charAt(0).toUpperCase() + key.slice(1)}</h4>
                <div className="threshold-inputs">
                  <div className="threshold-input">
                    <label>Warning:</label>
                    <input
                      type="number"
                      value={values.warning}
                      onChange={(e) => updateThreshold(key as keyof ThresholdSettings, 'warning', parseFloat(e.target.value))}
                      step="0.1"
                      min="0"
                    />
                    <span className="unit">
                      {key.includes('acceleration') ? 'm/s²' : 
                       key.includes('angularVelocity') ? 'rad/s' : '°'}
                    </span>
                  </div>
                  <div className="threshold-input">
                    <label>Critical:</label>
                    <input
                      type="number"
                      value={values.critical}
                      onChange={(e) => updateThreshold(key as keyof ThresholdSettings, 'critical', parseFloat(e.target.value))}
                      step="0.1"
                      min="0"
                    />
                    <span className="unit">
                      {key.includes('acceleration') ? 'm/s²' : 
                       key.includes('angularVelocity') ? 'rad/s' : '°'}
                    </span>
                  </div>
                </div>
              </div>
            ))}
          </div>
          
          <div className="threshold-actions">
            <button 
              className="reset-button"
              onClick={() => setThresholds({
                roll: { warning: 30, critical: 60 },
                pitch: { warning: 30, critical: 60 },
                yaw: { warning: 90, critical: 180 },
                acceleration: { warning: 5, critical: 10 },
                angularVelocity: { warning: 1, critical: 2 }
              })}
            >
              Reset to Defaults
            </button>
          </div>
        </div>
      </div>
    );
  };

  const renderOrientationVisualizer = () => {
    if (!eulerAngles) return null;

    const { roll, pitch, yaw } = eulerAngles;
    
    return (
      <div className="orientation-visualizer">
        <div className="attitude-indicator">
          <div className="horizon-container">
            <div 
              className="horizon"
              style={{
                transform: `rotate(${-roll}deg) translateY(${-pitch * 2}px)`
              }}
            >
              <div className="horizon-line"></div>
              <div className="sky"></div>
              <div className="ground"></div>
            </div>
            <div className="aircraft-symbol">
              <div className="aircraft-center"></div>
              <div className="aircraft-wings"></div>
            </div>
            <div className="pitch-scale">
              {[-30, -20, -10, 0, 10, 20, 30].map(angle => (
                <div 
                  key={angle}
                  className="pitch-line"
                  style={{
                    transform: `translateY(${angle * 2}px) rotate(${-roll}deg)`
                  }}
                >
                  <span className="pitch-label">{angle}°</span>
                </div>
              ))}
            </div>
          </div>
          <div className="roll-scale">
            <div 
              className="roll-pointer"
              style={{ transform: `rotate(${roll}deg)` }}
            ></div>
            {[-60, -45, -30, -15, 0, 15, 30, 45, 60].map(angle => (
              <div 
                key={angle}
                className="roll-tick"
                style={{ transform: `rotate(${angle}deg)` }}
              >
                <div className="roll-mark"></div>
                {angle % 30 === 0 && (
                  <span className="roll-label">{Math.abs(angle)}</span>
                )}
              </div>
            ))}
          </div>
        </div>
        
        <div className="compass">
          <div 
            className="compass-rose"
            style={{ 
              transform: `rotate(${compassData && compassData.magneticFieldStrength > 0.1 ? 
                -compassData.magneticHeading : 
                -(yaw < 0 ? yaw + 360 : yaw)}deg)` 
            }}
          >
            <div className="compass-direction north">N</div>
            <div className="compass-direction east">E</div>
            <div className="compass-direction south">S</div>
            <div className="compass-direction west">W</div>
            {[0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330].map(angle => (
              <div 
                key={angle}
                className="compass-tick"
                style={{ transform: `rotate(${angle}deg)` }}
              ></div>
            ))}
          </div>
          <div className="compass-pointer"></div>
          <div className="compass-heading">
            {compassData && compassData.magneticFieldStrength > 0.1 ? 
              Math.round(compassData.magneticHeading) : 
              Math.round(yaw < 0 ? yaw + 360 : yaw)}°
          </div>
          <div className="compass-label">
            {compassData && compassData.magneticFieldStrength > 0.1 ? 
              `Magnetic (${compassData.magneticFieldStrength.toFixed(2)}μT)` : 
              'Quaternion Yaw'}
          </div>
          {/* Debug Info */}
          {compassData && (
            <div style={{
              position: 'absolute',
              bottom: '-80px',
              left: '50%',
              transform: 'translateX(-50%)',
              fontSize: '10px',
              color: '#888',
              textAlign: 'center',
              background: 'rgba(0,0,0,0.7)',
              padding: '4px',
              borderRadius: '4px',
              whiteSpace: 'nowrap'
            }}>
              Mag: {compassData.magneticHeading.toFixed(1)}° | 
              Quat: {(yaw < 0 ? yaw + 360 : yaw).toFixed(1)}° | 
              Tilt: {compassData.tiltAngle.toFixed(1)}°
            </div>
          )}
        </div>
        
        {/* Axis Reference Display */}
        <div className="axis-reference">
          <h4>📐 ROS/REP-103 Standard Coordinate System</h4>
          <div className="axis-info">
            <div className="axis-item">
              <span className="axis-label">Roll (X-axis):</span>
              <span className="axis-desc">Rotation around forward axis (banking left/right)</span>
            </div>
            <div className="axis-item">
              <span className="axis-label">Pitch (Y-axis):</span>
              <span className="axis-desc">Rotation around right axis (nose up/down)</span>
            </div>
            <div className="axis-item">
              <span className="axis-label">Yaw (Z-axis):</span>
              <span className="axis-desc">Rotation around up axis (heading/direction)</span>
            </div>
          </div>
        </div>
      </div>
    );
  };

  const renderDataChart = () => {
    if (dataHistory.length < 2) return null;

    const chartWidth = 800;
    const chartHeight = 200;
    const maxPoints = 50;
    const recentData = dataHistory.slice(-maxPoints);

    const createPath = (data: number[], color: string, scale: number = 1, offset: number = 0) => {
      if (data.length < 2) return null;
      
      // Find min/max for better scaling
      const minVal = Math.min(...data);
      const maxVal = Math.max(...data);
      const range = Math.max(Math.abs(maxVal), Math.abs(minVal));
      const adaptiveScale = range > 0 ? (chartHeight * 0.3) / range : scale;
      
      const points = data.map((value, index) => {
        const x = (index / (data.length - 1)) * chartWidth;
        const y = chartHeight / 2 - ((value + offset) * adaptiveScale);
        return `${x},${Math.max(5, Math.min(chartHeight - 5, y))}`;
      }).join(' ');

      return (
        <polyline
          points={points}
          fill="none"
          stroke={color}
          strokeWidth="2"
          opacity="0.9"
        />
      );
    };

    return (
      <div className="data-charts">
        <div className="chart-container">
          <h4>Linear Acceleration (m/s²)</h4>
          <svg width={chartWidth} height={chartHeight} className="data-chart">
            <defs>
              <linearGradient id="chartGradient" x1="0%" y1="0%" x2="0%" y2="100%">
                <stop offset="0%" stopColor="rgba(0,255,65,0.1)" />
                <stop offset="100%" stopColor="rgba(0,255,65,0)" />
              </linearGradient>
            </defs>
            <rect width={chartWidth} height={chartHeight} fill="url(#chartGradient)" />
            {/* X-axis (bottom line) */}
            <line x1="0" y1={chartHeight-1} x2={chartWidth} y2={chartHeight-1} stroke="rgba(255,255,255,0.5)" strokeWidth="1" />
            {/* Center line (Y=0) */}
            <line x1="0" y1={chartHeight/2} x2={chartWidth} y2={chartHeight/2} stroke="rgba(255,255,255,0.3)" strokeWidth="1" />
            {/* Quarter lines */}
            <line x1="0" y1={chartHeight*0.25} x2={chartWidth} y2={chartHeight*0.25} stroke="rgba(255,255,255,0.1)" strokeWidth="1" strokeDasharray="2,2" />
            <line x1="0" y1={chartHeight*0.75} x2={chartWidth} y2={chartHeight*0.75} stroke="rgba(255,255,255,0.1)" strokeWidth="1" strokeDasharray="2,2" />
            {createPath(recentData.map(d => d.linearAcceleration.x), '#ff6b6b')}
            {createPath(recentData.map(d => d.linearAcceleration.y), '#4ecdc4')}
            {createPath(recentData.map(d => d.linearAcceleration.z), '#45b7d1', 1, -9.81)}
          </svg>
          <div className="chart-legend">
            <span style={{color: '#ff6b6b'}}>● X</span>
            <span style={{color: '#4ecdc4'}}>● Y</span>
            <span style={{color: '#45b7d1'}}>● Z</span>
          </div>
        </div>

        <div className="chart-container">
          <h4>Angular Velocity (rad/s)</h4>
          <svg width={chartWidth} height={chartHeight} className="data-chart">
            <rect width={chartWidth} height={chartHeight} fill="url(#chartGradient)" />
            {/* X-axis (bottom line) */}
            <line x1="0" y1={chartHeight-1} x2={chartWidth} y2={chartHeight-1} stroke="rgba(255,255,255,0.5)" strokeWidth="1" />
            {/* Center line (Y=0) */}
            <line x1="0" y1={chartHeight/2} x2={chartWidth} y2={chartHeight/2} stroke="rgba(255,255,255,0.3)" strokeWidth="1" />
            {/* Quarter lines */}
            <line x1="0" y1={chartHeight*0.25} x2={chartWidth} y2={chartHeight*0.25} stroke="rgba(255,255,255,0.1)" strokeWidth="1" strokeDasharray="2,2" />
            <line x1="0" y1={chartHeight*0.75} x2={chartWidth} y2={chartHeight*0.75} stroke="rgba(255,255,255,0.1)" strokeWidth="1" strokeDasharray="2,2" />
            {createPath(recentData.map(d => d.angularVelocity.x), '#ff6b6b')}
            {createPath(recentData.map(d => d.angularVelocity.y), '#4ecdc4')}
            {createPath(recentData.map(d => d.angularVelocity.z), '#45b7d1')}
          </svg>
          <div className="chart-legend">
            <span style={{color: '#ff6b6b'}}>● X</span>
            <span style={{color: '#4ecdc4'}}>● Y</span>
            <span style={{color: '#45b7d1'}}>● Z</span>
          </div>
        </div>
      </div>
    );
  };

  return (
    <div className="imu-dashboard" key={`imu-${forceUpdate}`}>
      <div className="imu-header">
        <div className="imu-title">
          <h1>🧭 IMU Sensor Monitoring</h1>
          <div className="connection-status">
            <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
              ●
            </span>
            <span>{connectionStatus}</span>
          </div>
        </div>
        <div className="header-controls">
          <button 
            className="threshold-settings-button"
            onClick={() => setShowThresholdSettings(true)}
          >
            ⚙️ Thresholds
          </button>
        </div>
      </div>

      {renderThresholdSettings()}

      {imuData && eulerAngles ? (
        <div className="imu-content">
          {/* Orientation Visualization */}
          <div className="imu-section orientation-section">
            <h2>🎯 Orientation Visualization</h2>
            {renderOrientationVisualizer()}
          </div>

          {/* Euler Angles */}
          <div className="imu-section">
            <h2>📐 Euler Angles</h2>
            <div className="euler-grid">
              <div className="euler-card">
                <div className="euler-label">Roll</div>
                <div 
                  className="euler-value"
                  style={{ color: getValueColor(eulerAngles.roll, 'roll') }}
                >
                  {eulerAngles.roll.toFixed(1)}°
                </div>
                <div className="euler-bar">
                  <div 
                    className="euler-fill"
                    style={{
                      width: `${Math.min(Math.abs(eulerAngles.roll) / 90 * 100, 100)}%`,
                      backgroundColor: getValueColor(eulerAngles.roll, 'roll')
                    }}
                  />
                </div>
              </div>

              <div className="euler-card">
                <div className="euler-label">Pitch</div>
                <div 
                  className="euler-value"
                  style={{ color: getValueColor(eulerAngles.pitch, 'pitch') }}
                >
                  {eulerAngles.pitch.toFixed(1)}°
                </div>
                <div className="euler-bar">
                  <div 
                    className="euler-fill"
                    style={{
                      width: `${Math.min(Math.abs(eulerAngles.pitch) / 90 * 100, 100)}%`,
                      backgroundColor: getValueColor(eulerAngles.pitch, 'pitch')
                    }}
                  />
                </div>
              </div>

              <div className="euler-card">
                <div className="euler-label">Yaw</div>
                <div 
                  className="euler-value"
                  style={{ color: getValueColor(eulerAngles.yaw, 'yaw') }}
                >
                  {eulerAngles.yaw.toFixed(1)}°
                </div>
                <div className="euler-bar">
                  <div 
                    className="euler-fill"
                    style={{
                      width: `${Math.abs(eulerAngles.yaw) / 180 * 100}%`,
                      backgroundColor: getValueColor(eulerAngles.yaw, 'yaw')
                    }}
                  />
                </div>
              </div>
            </div>
          </div>

          {/* Raw Sensor Data */}
          <div className="imu-section">
            <h2>📊 Raw Sensor Data</h2>
            <div className="sensor-data-grid">
              {/* Quaternion */}
              <div className="sensor-group">
                <h3>Quaternion</h3>
                <div className="quaternion-grid">
                  <div className="quat-value">
                    <span className="quat-label">W:</span>
                    <span className="quat-number">{Math.abs(imuData.orientation.w) > 1000 ? imuData.orientation.w.toExponential(2) : imuData.orientation.w.toFixed(3)}</span>
                  </div>
                  <div className="quat-value">
                    <span className="quat-label">X:</span>
                    <span className="quat-number">{Math.abs(imuData.orientation.x) > 1000 ? imuData.orientation.x.toExponential(2) : imuData.orientation.x.toFixed(3)}</span>
                  </div>
                  <div className="quat-value">
                    <span className="quat-label">Y:</span>
                    <span className="quat-number">{Math.abs(imuData.orientation.y) > 1000 ? imuData.orientation.y.toExponential(2) : imuData.orientation.y.toFixed(3)}</span>
                  </div>
                  <div className="quat-value">
                    <span className="quat-label">Z:</span>
                    <span className="quat-number">{Math.abs(imuData.orientation.z) > 1000 ? imuData.orientation.z.toExponential(2) : imuData.orientation.z.toFixed(3)}</span>
                  </div>
                </div>
              </div>

              {/* Linear Acceleration */}
              <div className="sensor-group">
                <h3>Linear Acceleration (m/s²)</h3>
                <div className="vector-grid">
                  <div className="vector-value">
                    <span className="vector-label">X:</span>
                    <span 
                      className="vector-number"
                      style={{ color: getValueColor(imuData.linearAcceleration.x, 'acceleration') }}
                    >
                      {imuData.linearAcceleration.x >= 0 ? '+' : ''}{imuData.linearAcceleration.x.toFixed(2)}
                    </span>
                  </div>
                  <div className="vector-value">
                    <span className="vector-label">Y:</span>
                    <span 
                      className="vector-number"
                      style={{ color: getValueColor(imuData.linearAcceleration.y, 'acceleration') }}
                    >
                      {imuData.linearAcceleration.y >= 0 ? '+' : ''}{imuData.linearAcceleration.y.toFixed(2)}
                    </span>
                  </div>
                  <div className="vector-value">
                    <span className="vector-label">Z:</span>
                    <span 
                      className="vector-number"
                      style={{ color: getValueColor(imuData.linearAcceleration.z - 9.81, 'acceleration') }}
                    >
                      {imuData.linearAcceleration.z >= 0 ? '+' : ''}{imuData.linearAcceleration.z.toFixed(2)}
                    </span>
                  </div>
                  <div className="vector-magnitude">
                    Magnitude: {getAccelerationMagnitude(imuData.linearAcceleration).toFixed(2)} m/s²
                  </div>
                </div>
              </div>

              {/* Angular Velocity */}
              <div className="sensor-group">
                <h3>Angular Velocity (rad/s)</h3>
                <div className="vector-grid">
                  <div className="vector-value">
                    <span className="vector-label">X:</span>
                    <span 
                      className="vector-number"
                      style={{ color: getValueColor(imuData.angularVelocity.x, 'angularVelocity') }}
                    >
                      {imuData.angularVelocity.x >= 0 ? '+' : ''}{imuData.angularVelocity.x.toFixed(3)}
                    </span>
                  </div>
                  <div className="vector-value">
                    <span className="vector-label">Y:</span>
                    <span 
                      className="vector-number"
                      style={{ color: getValueColor(imuData.angularVelocity.y, 'angularVelocity') }}
                    >
                      {imuData.angularVelocity.y >= 0 ? '+' : ''}{imuData.angularVelocity.y.toFixed(3)}
                    </span>
                  </div>
                  <div className="vector-value">
                    <span className="vector-label">Z:</span>
                    <span 
                      className="vector-number"
                      style={{ color: getValueColor(imuData.angularVelocity.z, 'angularVelocity') }}
                    >
                      {imuData.angularVelocity.z >= 0 ? '+' : ''}{imuData.angularVelocity.z.toFixed(3)}
                    </span>
                  </div>
                  <div className="vector-magnitude">
                    Magnitude: {getAngularVelocityMagnitude(imuData.angularVelocity).toFixed(3)} rad/s
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Data Visualization Charts */}
          <div className="imu-section">
            <h2>📈 Real-time Data Visualization</h2>
            {renderDataChart()}
          </div>

          {/* System Information */}
          <div className="imu-section">
            <h2>ℹ️ System Information</h2>
            <div className="system-info-grid">
              <div className="info-card">
                <div className="info-label">Data Rate</div>
                <div className="info-value">{dataHistory.length > 1 ? '20 Hz' : 'N/A'}</div>
              </div>
              <div className="info-card">
                <div className="info-label">Last Update</div>
                <div className="info-value">
                  {new Date(imuData.timestamp).toLocaleTimeString()}
                </div>
              </div>
              <div className="info-card">
                <div className="info-label">Topic</div>
                <div className="info-value">/perception_brain/imu/json</div>
              </div>
              <div className="info-card">
                <div className="info-label">Message Type</div>
                <div className="info-value">std_msgs/String</div>
              </div>
            </div>
          </div>
        </div>
      ) : (
        <div className="imu-loading">
          <div className="loading-spinner"></div>
          <div className="loading-text">Waiting for IMU data...</div>
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

export default IMUSensorDashboard;
