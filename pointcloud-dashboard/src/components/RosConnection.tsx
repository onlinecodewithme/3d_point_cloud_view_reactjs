import React, { useEffect, useRef, useState } from 'react';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

interface RosConnectionProps {
  onPointCloudUpdate: (data: PointCloudData) => void;
  onConnectionStatusChange: (connected: boolean) => void;
  onStatusMessageChange: (message: string) => void;
}

interface PointCloud2Message {
  header: {
    stamp: {
      sec: number;
      nanosec: number;
    };
    frame_id: string;
  };
  height: number;
  width: number;
  fields: Array<{
    name: string;
    offset: number;
    datatype: number;
    count: number;
  }>;
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: number[];
  is_dense: boolean;
}

const RosConnection: React.FC<RosConnectionProps> = ({
  onPointCloudUpdate,
  onConnectionStatusChange,
  onStatusMessageChange,
}) => {
  const [rosUrl, setRosUrl] = useState('ws://localhost:9090');
  const [isConnecting, setIsConnecting] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const parsePointCloud2 = (message: PointCloud2Message): PointCloudData | null => {
    try {
      const { data, fields, point_step, width, height } = message;
      
      // Find field offsets
      const xField = fields.find(f => f.name === 'x');
      const yField = fields.find(f => f.name === 'y');
      const zField = fields.find(f => f.name === 'z');
      const rgbField = fields.find(f => f.name === 'rgb');

      if (!xField || !yField || !zField) {
        console.error('Missing required fields (x, y, z) in point cloud');
        return null;
      }

      const totalPoints = width * height;
      
      // Check if data is base64 encoded (common with rosbridge)
      let binaryData: Uint8Array;
      
      if (typeof data === 'string') {
        // Data is base64 encoded string
        const binaryString = atob(data);
        binaryData = new Uint8Array(binaryString.length);
        for (let i = 0; i < binaryString.length; i++) {
          binaryData[i] = binaryString.charCodeAt(i);
        }
      } else if (Array.isArray(data)) {
        // Data is array of bytes
        binaryData = new Uint8Array(data);
      } else {
        console.error('Unknown data format:', typeof data);
        return null;
      }
      
      // Create DataView for proper parsing
      const buffer = binaryData.buffer.slice(binaryData.byteOffset, binaryData.byteOffset + binaryData.byteLength);
      const view = new DataView(buffer);

      // First pass: collect valid points
      const validPointsData: number[] = [];
      const validColorsData: number[] = [];

      for (let i = 0; i < totalPoints; i++) {
        const pointOffset = i * point_step;
        
        // Extract XYZ coordinates
        const x = view.getFloat32(pointOffset + xField.offset, true);
        const y = view.getFloat32(pointOffset + yField.offset, true);
        const z = view.getFloat32(pointOffset + zField.offset, true);

        // Filter out only NaN and infinity values, keep all other points including zeros
        if (isFinite(x) && isFinite(y) && isFinite(z)) {
          
          validPointsData.push(x, y, z);

          // Extract RGB if available
          if (rgbField) {
            const rgbValue = view.getUint32(pointOffset + rgbField.offset, true);
            let r = ((rgbValue >> 16) & 0xFF) / 255.0;
            let g = ((rgbValue >> 8) & 0xFF) / 255.0;
            let b = (rgbValue & 0xFF) / 255.0;
            
            
            // Apply moderate brightness enhancement for more vivid colors like RViz
            const brightness = 1.1; // Moderate brightness increase
            r = Math.min(1.0, r * brightness);
            g = Math.min(1.0, g * brightness);
            b = Math.min(1.0, b * brightness);
            
            // Ensure we have valid colors (fallback if all are black)
            if (r === 0 && g === 0 && b === 0) {
              // Use height-based coloring as fallback
              const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
              r = normalizedZ;
              g = 1 - normalizedZ;
              b = 1 - normalizedZ;
            }
            
            validColorsData.push(r, g, b);
          } else {
            // Default color based on height with bright colors
            const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
            validColorsData.push(normalizedZ, 1 - normalizedZ, 1 - normalizedZ);
          }
        }
      }

      // Create final arrays with only valid points
      const points = new Float32Array(validPointsData);
      const colors = new Float32Array(validColorsData);
      const numPoints = validPointsData.length / 3;

      const timestamp = message.header.stamp.sec * 1000 + message.header.stamp.nanosec / 1000000;


      return {
        points,
        colors: rgbField ? colors : undefined,
        timestamp,
      };
    } catch (error) {
      console.error('Error parsing point cloud data:', error);
      return null;
    }
  };

  const connect = () => {
    if (isConnecting || (wsRef.current && wsRef.current.readyState === WebSocket.OPEN)) {
      return;
    }

    setIsConnecting(true);
    onStatusMessageChange('Connecting...');

    try {
      const ws = new WebSocket(rosUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('Connected to ROS bridge');
        setIsConnecting(false);
        onConnectionStatusChange(true);
        onStatusMessageChange('Connected');

        // Subscribe to point cloud topic
        const subscribeMessage = {
          op: 'subscribe',
          topic: '/zed/zed_node/point_cloud/cloud_registered',
          type: 'sensor_msgs/PointCloud2',
        };

        ws.send(JSON.stringify(subscribeMessage));
        console.log('Subscribed to point cloud topic');
      };

      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          
          if (message.topic === '/zed/zed_node/point_cloud/cloud_registered' && message.msg) {
            const pointCloudData = parsePointCloud2(message.msg);
            if (pointCloudData) {
              onPointCloudUpdate(pointCloudData);
            }
          }
        } catch (error) {
          console.error('Error processing message:', error);
        }
      };

      ws.onclose = () => {
        console.log('Disconnected from ROS bridge');
        setIsConnecting(false);
        onConnectionStatusChange(false);
        onStatusMessageChange('Disconnected');
        
        // Attempt to reconnect after 3 seconds
        reconnectTimeoutRef.current = setTimeout(() => {
          if (!wsRef.current || wsRef.current.readyState === WebSocket.CLOSED) {
            onStatusMessageChange('Reconnecting...');
            connect();
          }
        }, 3000);
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setIsConnecting(false);
        onConnectionStatusChange(false);
        onStatusMessageChange('Connection Error');
      };

    } catch (error) {
      console.error('Failed to create WebSocket connection:', error);
      setIsConnecting(false);
      onConnectionStatusChange(false);
      onStatusMessageChange('Connection Failed');
    }
  };

  const disconnect = () => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    
    setIsConnecting(false);
    onConnectionStatusChange(false);
    onStatusMessageChange('Disconnected');
  };

  // Auto-connect on component mount
  useEffect(() => {
    connect();

    return () => {
      disconnect();
    };
  }, []);

  // Generate demo data for testing when not connected
  useEffect(() => {
    let demoInterval: NodeJS.Timeout;

    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      demoInterval = setInterval(() => {
        // Generate demo point cloud data
        const numPoints = 1000;
        const points = new Float32Array(numPoints * 3);
        const colors = new Float32Array(numPoints * 3);

        for (let i = 0; i < numPoints; i++) {
          // Create a simple sphere pattern
          const theta = Math.random() * Math.PI * 2;
          const phi = Math.random() * Math.PI;
          const radius = 1 + Math.random() * 2;

          points[i * 3] = radius * Math.sin(phi) * Math.cos(theta);
          points[i * 3 + 1] = radius * Math.sin(phi) * Math.sin(theta);
          points[i * 3 + 2] = radius * Math.cos(phi);

          // Color based on position
          colors[i * 3] = (points[i * 3] + 3) / 6;
          colors[i * 3 + 1] = (points[i * 3 + 1] + 3) / 6;
          colors[i * 3 + 2] = (points[i * 3 + 2] + 3) / 6;
        }

        onPointCloudUpdate({
          points,
          colors,
          timestamp: Date.now(),
        });
      }, 100); // Update every 100ms for demo
    }

    return () => {
      if (demoInterval) {
        clearInterval(demoInterval);
      }
    };
  }, [wsRef.current?.readyState]);

  return (
    <div className="connection-controls">
      <input
        type="text"
        value={rosUrl}
        onChange={(e) => setRosUrl(e.target.value)}
        placeholder="ROS Bridge URL (ws://localhost:9090)"
        className="connection-input"
        disabled={isConnecting}
      />
      <button
        onClick={wsRef.current?.readyState === WebSocket.OPEN ? disconnect : connect}
        disabled={isConnecting}
        className="connect-button"
      >
        {isConnecting ? 'Connecting...' : 
         wsRef.current?.readyState === WebSocket.OPEN ? 'Disconnect' : 'Connect'}
      </button>
    </div>
  );
};

export default RosConnection;
