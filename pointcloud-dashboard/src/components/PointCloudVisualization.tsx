import React, { useRef, useEffect, useMemo, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Stats } from '@react-three/drei';
import * as THREE from 'three';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

// Point Cloud Renderer Component
const PointCloudRenderer: React.FC<{ data: PointCloudData | null }> = ({ data }) => {
  const pointsRef = useRef<THREE.Points>(null);
  const geometryRef = useRef<THREE.BufferGeometry>(null);

  // Create geometry and material with enhanced settings for better colors
  const geometry = useMemo(() => new THREE.BufferGeometry(), []);
  const material = useMemo(() => new THREE.PointsMaterial({ 
    size: 0.04, // Maximum density - tiny points for ultra-detailed mapping
    vertexColors: true,
    sizeAttenuation: true,
    transparent: false,
    opacity: 1.0,
    toneMapped: false, // Disable tone mapping for brighter colors
    fog: false, // Disable fog for clearer colors
  }), []);

  // Update point cloud data
  useEffect(() => {
    if (!data || !data.points || data.points.length === 0) {
      return;
    }

    // Set position attribute
    geometry.setAttribute('position', new THREE.BufferAttribute(data.points, 3));

    // Set color attribute
    if (data.colors && data.colors.length > 0) {
      geometry.setAttribute('color', new THREE.BufferAttribute(data.colors, 3));
    } else {
      // Generate default colors based on height (Z coordinate)
      const colors = new Float32Array(data.points.length);
      for (let i = 0; i < data.points.length; i += 3) {
        const z = data.points[i + 2];
        // Color gradient from blue (low) to red (high)
        const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4)); // Normalize Z to 0-1
        colors[i] = normalizedZ; // Red
        colors[i + 1] = 1 - normalizedZ; // Green
        colors[i + 2] = 1 - normalizedZ; // Blue
      }
      geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    }

    // Update geometry
    geometry.attributes.position.needsUpdate = true;
    geometry.attributes.color.needsUpdate = true;
    geometry.computeBoundingSphere();

  }, [data, geometry]);

  // Animation frame
  useFrame(() => {
    if (pointsRef.current) {
      // Optional: Add subtle rotation or animation
      // pointsRef.current.rotation.y += 0.001;
    }
  });

  if (!data || !data.points || data.points.length === 0) {
    return (
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[0.1, 0.1, 0.1]} />
        <meshBasicMaterial color="orange" />
      </mesh>
    );
  }

  return (
    <points 
      ref={pointsRef} 
      geometry={geometry} 
      material={material}
      rotation={[-Math.PI / 2, 0, 0]} // Rotate -90 degrees around X-axis (anti-clockwise)
    />
  );
};

// Main Point Cloud Visualization Component
const PointCloudVisualization: React.FC = () => {
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [pointCount, setPointCount] = useState(0);
  const rosRef = useRef<any>(null);

  useEffect(() => {
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
          console.log('Connected to ROS bridge');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
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

        // Subscribe to RTAB-Map cloud_map topic
        const cloudMapTopic = new ROSLIB.Topic({
          ros: ros,
          name: '/cloud_map',
          messageType: 'sensor_msgs/PointCloud2'
        });

        cloudMapTopic.subscribe((message: any) => {
          try {
            console.log('Received cloud_map data:', message);
            
            // Parse PointCloud2 message
            const result = parsePointCloud2(message);
            if (result && result.points.length > 0) {
              const pointCloudData: PointCloudData = {
                points: result.points,
                colors: result.colors,
                timestamp: Date.now()
              };
              
              setPointCloudData(pointCloudData);
              setPointCount(result.points.length / 3);
              setConnectionStatus(`Connected - ${(result.points.length / 3).toLocaleString()} points`);
            }
          } catch (error) {
            console.error('Error processing point cloud data:', error);
            setConnectionStatus('Error processing point cloud data');
          }
        });

        console.log('ROS initialization complete');
      } catch (error) {
        console.error('Failed to initialize ROS:', error);
        setConnectionStatus('Failed to initialize ROS');
      }
    };

    initROS();

    // Cleanup
    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  // Parse PointCloud2 message to Float32Array with RGB colors
  const parsePointCloud2 = (message: any): { points: Float32Array; colors: Float32Array } | null => {
    try {
      if (!message.data || message.data.length === 0) {
        return null;
      }

      // PointCloud2 parsing logic
      const width = message.width;
      const height = message.height;
      const pointStep = message.point_step;
      const rowStep = message.row_step;
      
      // Find field offsets for x, y, z, and rgb
      let xOffset = -1, yOffset = -1, zOffset = -1, rgbOffset = -1;
      for (const field of message.fields) {
        if (field.name === 'x') xOffset = field.offset;
        if (field.name === 'y') yOffset = field.offset;
        if (field.name === 'z') zOffset = field.offset;
        if (field.name === 'rgb' || field.name === 'rgba') rgbOffset = field.offset;
      }

      if (xOffset === -1 || yOffset === -1 || zOffset === -1) {
        console.error('Could not find x, y, z fields in point cloud');
        return null;
      }

      // Convert base64 data to binary
      const binaryData = atob(message.data);
      const dataView = new DataView(new ArrayBuffer(binaryData.length));
      for (let i = 0; i < binaryData.length; i++) {
        dataView.setUint8(i, binaryData.charCodeAt(i));
      }

      // Extract points and colors
      const numPoints = width * height;
      const points = new Float32Array(numPoints * 3);
      const colors = new Float32Array(numPoints * 3);
      let pointIndex = 0;

      for (let i = 0; i < numPoints; i++) {
        const pointOffset = i * pointStep;
        
        const x = dataView.getFloat32(pointOffset + xOffset, true); // little endian
        const y = dataView.getFloat32(pointOffset + yOffset, true);
        const z = dataView.getFloat32(pointOffset + zOffset, true);

        // Filter out invalid points
        if (!isNaN(x) && !isNaN(y) && !isNaN(z) && isFinite(x) && isFinite(y) && isFinite(z)) {
          points[pointIndex * 3] = x;
          points[pointIndex * 3 + 1] = y;
          points[pointIndex * 3 + 2] = z;

          // Extract RGB color if available
          if (rgbOffset !== -1) {
            // RGB is typically packed as a 32-bit float or uint32
            const rgbValue = dataView.getUint32(pointOffset + rgbOffset, true);
            
            // Extract RGB components (assuming RGB packed format)
            const r = ((rgbValue >> 16) & 0xFF) / 255.0;
            const g = ((rgbValue >> 8) & 0xFF) / 255.0;
            const b = (rgbValue & 0xFF) / 255.0;
            
            colors[pointIndex * 3] = r;
            colors[pointIndex * 3 + 1] = g;
            colors[pointIndex * 3 + 2] = b;
          } else {
            // Fallback to height-based coloring if no RGB data
            const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
            colors[pointIndex * 3] = normalizedZ; // Red
            colors[pointIndex * 3 + 1] = 1 - normalizedZ; // Green
            colors[pointIndex * 3 + 2] = 1 - normalizedZ; // Blue
          }
          
          pointIndex++;
        }
      }

      // Return trimmed arrays with only valid points
      return {
        points: points.slice(0, pointIndex * 3),
        colors: colors.slice(0, pointIndex * 3)
      };
    } catch (error) {
      console.error('Error parsing PointCloud2:', error);
      return null;
    }
  };

  return (
    <div style={{ width: '100%', height: '100vh', position: 'relative' }}>
      {/* Status overlay */}
      <div style={{
        position: 'absolute',
        top: '10px',
        left: '10px',
        background: 'rgba(0, 0, 0, 0.8)',
        color: 'white',
        padding: '10px',
        borderRadius: '5px',
        zIndex: 1000,
        fontFamily: 'monospace',
        fontSize: '12px'
      }}>
        <div>Status: {connectionStatus}</div>
        <div>Points: {pointCount.toLocaleString()}</div>
        <div>Topic: /cloud_map</div>
        <div style={{ color: isConnected ? '#00ff00' : '#ff0000' }}>
          ● {isConnected ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      {/* 3D Canvas */}
      <Canvas
        camera={{ position: [5, 5, 5] as [number, number, number], fov: 75 }}
        style={{ background: '#000' }}
        gl={{ 
          antialias: true, 
          alpha: false,
          powerPreference: "high-performance"
        }}
      >
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10] as [number, number, number]} />
        
        <PointCloudRenderer data={pointCloudData} />
        
        <OrbitControls 
          enablePan={true}
          enableZoom={true}
          enableRotate={true}
          maxDistance={50}
          minDistance={0.1}
        />
        
        <Stats />
        
        {/* Grid helper for reference */}
        <gridHelper args={[20, 20]} />
        
        {/* Axes helper for orientation */}
        <axesHelper args={[5]} />
        
        {/* Additional lighting */}
        <directionalLight 
          position={[10, 10, 5] as [number, number, number]} 
          intensity={0.3}
          castShadow
        />
        <hemisphereLight 
          args={["#ffffff", "#444444", 0.2] as [string, string, number]}
        />
      </Canvas>
    </div>
  );
};

export default PointCloudVisualization;
