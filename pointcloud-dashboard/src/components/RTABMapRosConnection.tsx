import React, { useRef, useEffect, useState } from 'react';

interface RTABMapData {
  occupancyGrid: {
    data: number[];
    width: number;
    height: number;
    resolution: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  robotPose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  trajectory: Array<{
    position: { x: number; y: number; z: number };
    timestamp: number;
  }>;
  mapCloud: {
    points: Float32Array;
    colors?: Float32Array;
  };
  loopClosures: Array<{
    from: number;
    to: number;
    confidence: number;
  }>;
}

interface RTABMapRosConnectionProps {
  onRTABMapUpdate: (data: RTABMapData) => void;
  onRTABMapStatusChange: (active: boolean) => void;
  onRTABMapStatsUpdate: (stats: {
    totalNodes: number;
    loopClosures: number;
    mapSize: string;
    processingTime: number;
    memoryUsage: number;
  }) => void;
  isRTABMapActive: boolean;
  mapQuality: string;
  loopClosureThreshold: number;
}

const RTABMapRosConnection: React.FC<RTABMapRosConnectionProps> = ({
  onRTABMapUpdate,
  onRTABMapStatusChange,
  onRTABMapStatsUpdate,
  isRTABMapActive,
  mapQuality,
  loopClosureThreshold
}) => {
  const wsRef = useRef<WebSocket | null>(null);
  const [rosUrl] = useState('ws://localhost:9090');
  const trajectoryRef = useRef<Array<{ position: { x: number; y: number; z: number }; timestamp: number }>>([]);
  const loopClosuresRef = useRef<Array<{ from: number; to: number; confidence: number }>>([]);
  const statsRef = useRef({
    totalNodes: 0,
    loopClosures: 0,
    mapSize: '0 MB',
    processingTime: 0,
    memoryUsage: 0
  });

  // Parse occupancy grid message
  const parseOccupancyGrid = (message: any) => {
    try {
      const { data, info } = message;
      
      // Handle base64 encoded data
      let gridData: number[];
      if (typeof data === 'string') {
        const binaryString = atob(data);
        gridData = Array.from(binaryString).map(char => char.charCodeAt(0));
      } else {
        gridData = data;
      }

      return {
        data: gridData,
        width: info.width,
        height: info.height,
        resolution: info.resolution,
        origin: {
          position: info.origin.position,
          orientation: info.origin.orientation
        }
      };
    } catch (error) {
      console.error('Error parsing occupancy grid:', error);
      return null;
    }
  };

  // Parse robot pose message
  const parseRobotPose = (message: any) => {
    try {
      const { pose } = message;
      return {
        position: pose.position,
        orientation: pose.orientation
      };
    } catch (error) {
      console.error('Error parsing robot pose:', error);
      return null;
    }
  };

  // Parse RTAB-Map point cloud
  const parseMapCloud = (message: any) => {
    try {
      const { data, fields, point_step, width, height } = message;
      
      // Find field offsets
      const xField = fields.find((f: any) => f.name === 'x');
      const yField = fields.find((f: any) => f.name === 'y');
      const zField = fields.find((f: any) => f.name === 'z');
      const rgbField = fields.find((f: any) => f.name === 'rgb');

      if (!xField || !yField || !zField) {
        console.error('Missing required fields in RTAB-Map cloud');
        return null;
      }

      const totalPoints = width * height;
      
      // Handle base64 encoded data
      let binaryData: Uint8Array;
      if (typeof data === 'string') {
        const binaryString = atob(data);
        binaryData = new Uint8Array(binaryString.length);
        for (let i = 0; i < binaryString.length; i++) {
          binaryData[i] = binaryString.charCodeAt(i);
        }
      } else {
        binaryData = new Uint8Array(data);
      }
      
      const buffer = binaryData.buffer.slice(binaryData.byteOffset, binaryData.byteOffset + binaryData.byteLength);
      const view = new DataView(buffer);

      const validPointsData: number[] = [];
      const validColorsData: number[] = [];

      for (let i = 0; i < totalPoints; i++) {
        const pointOffset = i * point_step;
        
        const x = view.getFloat32(pointOffset + xField.offset, true);
        const y = view.getFloat32(pointOffset + yField.offset, true);
        const z = view.getFloat32(pointOffset + zField.offset, true);

        if (isFinite(x) && isFinite(y) && isFinite(z)) {
          validPointsData.push(x, y, z);

          if (rgbField) {
            const rgbValue = view.getUint32(pointOffset + rgbField.offset, true);
            const r = ((rgbValue >> 16) & 0xFF) / 255.0;
            const g = ((rgbValue >> 8) & 0xFF) / 255.0;
            const b = (rgbValue & 0xFF) / 255.0;
            validColorsData.push(r, g, b);
          } else {
            // Default coloring for RTAB-Map points
            validColorsData.push(0.8, 0.8, 0.9); // Light blue-gray
          }
        }
      }

      return {
        points: new Float32Array(validPointsData),
        colors: new Float32Array(validColorsData)
      };
    } catch (error) {
      console.error('Error parsing RTAB-Map cloud:', error);
      return null;
    }
  };

  // Parse loop closure info
  const parseLoopClosures = (message: any) => {
    try {
      const { loop_closures } = message;
      return loop_closures.map((closure: any) => ({
        from: closure.from_id,
        to: closure.to_id,
        confidence: closure.confidence
      }));
    } catch (error) {
      console.error('Error parsing loop closures:', error);
      return [];
    }
  };

  // Update trajectory
  const updateTrajectory = (pose: any) => {
    const newPoint = {
      position: pose.position,
      timestamp: Date.now()
    };
    
    trajectoryRef.current.push(newPoint);
    
    // Keep only last 1000 trajectory points for performance
    if (trajectoryRef.current.length > 1000) {
      trajectoryRef.current = trajectoryRef.current.slice(-1000);
    }
  };

  // Update statistics
  const updateStats = () => {
    statsRef.current = {
      totalNodes: trajectoryRef.current.length,
      loopClosures: loopClosuresRef.current.length,
      mapSize: `${(trajectoryRef.current.length * 0.001).toFixed(1)} MB`,
      processingTime: Math.random() * 50 + 10, // Simulated processing time
      memoryUsage: trajectoryRef.current.length * 0.01 + Math.random() * 10
    };
    
    onRTABMapStatsUpdate(statsRef.current);
  };

  // Connect to ROS
  const connect = () => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      return;
    }

    try {
      const ws = new WebSocket(rosUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('Connected to ROS bridge for RTAB-Map');
        onRTABMapStatusChange(true);

        // Subscribe to RTAB-Map topics
        const subscriptions = [
          {
            op: 'subscribe',
            topic: '/map',
            type: 'nav_msgs/OccupancyGrid'
          },
          {
            op: 'subscribe',
            topic: '/robot_pose',
            type: 'geometry_msgs/PoseStamped'
          },
          {
            op: 'subscribe',
            topic: '/rtabmap/cloud_map',
            type: 'sensor_msgs/PointCloud2'
          },
          {
            op: 'subscribe',
            topic: '/rtabmap/info',
            type: 'rtabmap_ros/Info'
          }
        ];

        subscriptions.forEach(sub => {
          ws.send(JSON.stringify(sub));
        });

        console.log('Subscribed to RTAB-Map topics');
      };

      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          
          if (!message.topic || !message.msg) return;

          let rtabMapData: Partial<RTABMapData> = {};
          let shouldUpdate = false;

          switch (message.topic) {
            case '/map':
              const occupancyGrid = parseOccupancyGrid(message.msg);
              if (occupancyGrid) {
                rtabMapData.occupancyGrid = occupancyGrid;
                shouldUpdate = true;
              }
              break;

            case '/robot_pose':
              const robotPose = parseRobotPose(message.msg);
              if (robotPose) {
                rtabMapData.robotPose = robotPose;
                updateTrajectory(robotPose);
                rtabMapData.trajectory = [...trajectoryRef.current];
                shouldUpdate = true;
              }
              break;

            case '/rtabmap/cloud_map':
              const mapCloud = parseMapCloud(message.msg);
              if (mapCloud) {
                rtabMapData.mapCloud = mapCloud;
                shouldUpdate = true;
              }
              break;

            case '/rtabmap/info':
              const loopClosures = parseLoopClosures(message.msg);
              if (loopClosures.length > 0) {
                loopClosuresRef.current = loopClosures;
                rtabMapData.loopClosures = loopClosures;
                shouldUpdate = true;
              }
              break;
          }

          if (shouldUpdate) {
            // Merge with existing data
            const completeData: RTABMapData = {
              occupancyGrid: rtabMapData.occupancyGrid || {
                data: [],
                width: 0,
                height: 0,
                resolution: 0.05,
                origin: {
                  position: { x: 0, y: 0, z: 0 },
                  orientation: { x: 0, y: 0, z: 0, w: 1 }
                }
              },
              robotPose: rtabMapData.robotPose || {
                position: { x: 0, y: 0, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
              },
              trajectory: rtabMapData.trajectory || trajectoryRef.current,
              mapCloud: rtabMapData.mapCloud || {
                points: new Float32Array(0),
                colors: new Float32Array(0)
              },
              loopClosures: rtabMapData.loopClosures || loopClosuresRef.current
            };

            onRTABMapUpdate(completeData);
            updateStats();
          }

        } catch (error) {
          console.error('Error processing RTAB-Map message:', error);
        }
      };

      ws.onclose = () => {
        console.log('Disconnected from ROS bridge (RTAB-Map)');
        onRTABMapStatusChange(false);
      };

      ws.onerror = (error) => {
        console.error('RTAB-Map WebSocket error:', error);
        onRTABMapStatusChange(false);
      };

    } catch (error) {
      console.error('Failed to create RTAB-Map WebSocket connection:', error);
      onRTABMapStatusChange(false);
    }
  };

  // Disconnect from ROS
  const disconnect = () => {
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    onRTABMapStatusChange(false);
  };

  // Auto-connect when RTAB-Map is activated
  useEffect(() => {
    if (isRTABMapActive) {
      connect();
    } else {
      disconnect();
    }

    return () => {
      disconnect();
    };
  }, [isRTABMapActive]);

  // Generate demo RTAB-Map data when not connected
  useEffect(() => {
    let demoInterval: NodeJS.Timeout;

    if (!isRTABMapActive) {
      demoInterval = setInterval(() => {
        // Generate demo RTAB-Map data
        const demoData: RTABMapData = {
          occupancyGrid: {
            data: Array(100 * 100).fill(0).map(() => Math.random() > 0.8 ? 100 : 0),
            width: 100,
            height: 100,
            resolution: 0.05,
            origin: {
              position: { x: -2.5, y: -2.5, z: 0 },
              orientation: { x: 0, y: 0, z: 0, w: 1 }
            }
          },
          robotPose: {
            position: { 
              x: Math.sin(Date.now() * 0.001) * 2, 
              y: Math.cos(Date.now() * 0.001) * 2, 
              z: 0 
            },
            orientation: { x: 0, y: 0, z: Math.sin(Date.now() * 0.0005), w: Math.cos(Date.now() * 0.0005) }
          },
          trajectory: trajectoryRef.current,
          mapCloud: {
            points: new Float32Array(300).map(() => (Math.random() - 0.5) * 10),
            colors: new Float32Array(300).map(() => 0.7 + Math.random() * 0.3)
          },
          loopClosures: []
        };

        onRTABMapUpdate(demoData);
        updateStats();
      }, 200); // Update every 200ms for demo
    }

    return () => {
      if (demoInterval) {
        clearInterval(demoInterval);
      }
    };
  }, [isRTABMapActive]);

  return null; // This component doesn't render anything
};

export default RTABMapRosConnection;
