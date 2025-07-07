import React, { useRef, useEffect, useMemo, useState, useCallback } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Stats } from '@react-three/drei';
import * as THREE from 'three';
import Dashboard from './Dashboard';
import OccupancyMapViewer from './OccupancyMapViewer';
import NavigationControlPanel from './NavigationControlPanel';
import MappingWidget from './MappingWidget';
import RTABMapWidget from './RTABMapWidget';
import BatteryWidget from './BatteryWidget';
import WidgetManager from './WidgetManager';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

interface OccupancyMapData {
  width: number;
  height: number;
  resolution: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  data: number[];
  timestamp: number;
}

interface Waypoint {
  id: string;
  x: number;
  y: number;
  z?: number;
  name: string;
  timestamp: number;
}

// Point Cloud Renderer Component
const PointCloudRenderer: React.FC<{ data: PointCloudData | null; showPointCloud: boolean }> = ({ data, showPointCloud }) => {
  const pointsRef = useRef<THREE.Points>(null);
  const geometryRef = useRef<THREE.BufferGeometry>(null);

  // Create geometry and material with enhanced settings for better colors
  const geometry = useMemo(() => new THREE.BufferGeometry(), []);
  const material = useMemo(() => new THREE.PointsMaterial({ 
    size: 0.045, // Balanced size for stable high-density mapping
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

  if (!data || !data.points || data.points.length === 0 || !showPointCloud) {
    return null;
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

// Occupancy Grid Renderer for 3D view
const OccupancyGridRenderer: React.FC<{ 
  mapData: OccupancyMapData | null; 
  opacity: number;
  showGrid: boolean;
}> = ({ mapData, opacity, showGrid }) => {
  const meshRef = useRef<THREE.InstancedMesh>(null);

  useEffect(() => {
    if (!mapData || !meshRef.current || !showGrid) return;

    const { width, height, data, resolution } = mapData;
    const mesh = meshRef.current;
    
    // Create instances for all cells (occupied and free)
    const allCells: { x: number; y: number; value: number }[] = [];
    
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const index = y * width + x;
        const value = data[index];
        
        // Render all known cells (occupied: value > 50, free: value >= 0 && value <= 50)
        // Skip unknown cells (value < 0)
        if (value >= 0) {
          allCells.push({ x, y, value });
        }
      }
    }

    // Update instance count
    mesh.count = allCells.length;
    
    // Set up instance matrices and colors
    const matrix = new THREE.Matrix4();
    const color = new THREE.Color();
    
    // Create quaternion from map origin orientation
    const quaternion = new THREE.Quaternion(
      mapData.origin.orientation.x,
      mapData.origin.orientation.y,
      mapData.origin.orientation.z,
      mapData.origin.orientation.w
    );
    
    allCells.forEach((cell, i) => {
      // Position in local map coordinates (ROS occupancy grid: origin is at bottom-left corner)
      const localX = cell.x * resolution;
      const localY = cell.y * resolution;
      const localZ = 0.01; // Slightly above ground
      
      // Create local position vector
      const localPosition = new THREE.Vector3(localX, localY, localZ);
      
      // Apply map orientation to local position
      localPosition.applyQuaternion(quaternion);
      
      // Add map origin position to get world coordinates
      const worldX = mapData.origin.position.x + localPosition.x;
      const worldY = mapData.origin.position.y + localPosition.y;
      const worldZ = mapData.origin.position.z + localPosition.z;
      
      // Set instance matrix
      matrix.setPosition(worldX, worldY, worldZ);
      mesh.setMatrixAt(i, matrix);
      
      // Set instance color based on occupancy value
      if (cell.value > 50) {
        // Occupied cells - Red
        color.setRGB(1.0, 0.0, 0.0);
      } else {
        // Free cells - White
        color.setRGB(1.0, 1.0, 1.0);
      }
      mesh.setColorAt(i, color);
    });
    
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) {
      mesh.instanceColor.needsUpdate = true;
    }
    
  }, [mapData, showGrid]);

  if (!mapData || !showGrid) return null;

  const cellSize = mapData.resolution;

  return (
    <instancedMesh
      ref={meshRef}
      args={[undefined, undefined, 100000]}
      rotation={[-Math.PI / 2, 0, 0]} // Rotate -90 degrees around X-axis (anticlockwise)
    >
      <boxGeometry args={[cellSize, cellSize, cellSize * 0.05]} />
      <meshLambertMaterial transparent opacity={opacity} />
    </instancedMesh>
  );
};

// Main Enhanced Point Cloud Visualization Component
const EnhancedPointCloudVisualization: React.FC = () => {
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);
  const [occupancyMapData, setOccupancyMapData] = useState<OccupancyMapData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting to ROS...');
  const [pointCount, setPointCount] = useState(0);
  const [timestamp, setTimestamp] = useState<number>();
  
  // Dashboard state
  const [enableMapping, setEnableMapping] = useState(false);
  const [mapPointCount, setMapPointCount] = useState(0);
  const [maxMapPoints] = useState(1000000);
  const [voxelSize, setVoxelSize] = useState(0.05);
  const [isProcessing, setIsProcessing] = useState(false);
  
  // RTAB-Map state
  const [isRTABMapActive, setIsRTABMapActive] = useState(false);
  const [showOccupancyGrid, setShowOccupancyGrid] = useState(true);
  const [showPointCloud, setShowPointCloud] = useState(true);
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [showLoopClosures, setShowLoopClosures] = useState(false);
  const [gridOpacity, setGridOpacity] = useState(0.7);
  const [mapQuality, setMapQuality] = useState('high');
  const [loopClosureThreshold, setLoopClosureThreshold] = useState(0.11);
  const [rtabMapStats] = useState({
    totalNodes: 0,
    loopClosures: 0,
    mapSize: '0 MB',
    processingTime: 0,
    memoryUsage: 0
  });

  // Occupancy map viewer state
  const [isMapExpanded, setIsMapExpanded] = useState(false);

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

        // Subscribe to point cloud topic
        const subscribeToPointCloud = (topicName: string) => {
          const cloudMapTopic = new ROSLIB.Topic({
            ros: ros,
            name: topicName,
            messageType: 'sensor_msgs/PointCloud2'
          });

          cloudMapTopic.subscribe((message: any) => {
            try {
              console.log('Received cloud_map data:', message);
              console.log('Point cloud frame_id:', message.header.frame_id);
              
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
                setMapPointCount(result.points.length / 3);
                setTimestamp(Date.now());
                setConnectionStatus(`Connected - ${(result.points.length / 3).toLocaleString()} points`);
              }
            } catch (error) {
              console.error('Error processing point cloud data:', error);
              setConnectionStatus('Error processing point cloud data');
            }
          });
        };

        // Subscribe to occupancy map
        const mapTopic = new ROSLIB.Topic({
          ros: ros,
          name: '/map',
          messageType: 'nav_msgs/OccupancyGrid'
        });

        mapTopic.subscribe((message: any) => {
          try {
            const mapData: OccupancyMapData = {
              width: message.info.width,
              height: message.info.height,
              resolution: message.info.resolution,
              origin: message.info.origin,
              data: message.data,
              timestamp: Date.now()
            };

            setOccupancyMapData(mapData);
            console.log('Received occupancy map:', mapData);
          } catch (error) {
            console.error('Error processing occupancy map:', error);
          }
        });

        // Auto-detect point cloud topic
        const topicsClient = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/topics',
          serviceType: 'rosapi/Topics'
        });

        const request = new ROSLIB.ServiceRequest({});
        
        topicsClient.callService(request, (result: any) => {
          const topics = result.topics || [];
          console.log('Available topics for point cloud:', topics);
          
          // Look for point cloud topics in order of preference
          const possibleTopics = [
            '/cloud_map',
            '/rtabmap/cloud_map',
            '/zed/zed_node/point_cloud/cloud_registered',
            '/zed2i/zed_node/point_cloud/cloud_registered',
            '/zed2/zed_node/point_cloud/cloud_registered'
          ];
          
          let selectedTopic = null;
          for (const topic of possibleTopics) {
            if (topics.includes(topic)) {
              selectedTopic = topic;
              break;
            }
          }
          
          // If no exact match, look for any point cloud topic
          if (!selectedTopic) {
            selectedTopic = topics.find((topic: string) => 
              topic.includes('cloud') || topic.includes('point_cloud')
            );
          }
          
          if (selectedTopic) {
            console.log('Found point cloud topic:', selectedTopic);
            subscribeToPointCloud(selectedTopic);
          } else {
            console.warn('No point cloud topic found, using default /cloud_map');
            subscribeToPointCloud('/cloud_map');
          }
        }, (error: any) => {
          // Fallback to default topic
          subscribeToPointCloud('/cloud_map');
          console.error('Failed to get topics list:', error);
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

  // Dashboard handlers
  const handleToggleMapping = (enabled: boolean) => {
    setEnableMapping(enabled);
    setIsProcessing(enabled);
  };

  const handleClearMap = () => {
    setPointCloudData(null);
    setMapPointCount(0);
    setPointCount(0);
  };

  const handleSaveMap = () => {
    console.log('Saving map...');
    // Implement map saving logic
  };

  const handleLoadMap = () => {
    console.log('Loading map...');
    // Implement map loading logic
  };

  const handleToggleRTABMap = (active: boolean) => {
    setIsRTABMapActive(active);
  };

  const handleResetRTABMap = () => {
    console.log('Resetting RTAB-Map...');
    // Implement RTAB-Map reset logic
  };

  const handleSaveRTABMap = () => {
    console.log('Saving RTAB-Map...');
    // Implement RTAB-Map save logic
  };

  const handleLoadRTABMap = () => {
    console.log('Loading RTAB-Map...');
    // Implement RTAB-Map load logic
  };

  // Widget configuration
  const [widgets, setWidgets] = useState<any[]>([
    {
      id: 'mapping-controls',
      title: 'Mapping & CUDA Acceleration',
      icon: '🚀',
      component: MappingWidget,
      props: {
        enableMapping,
        onToggleMapping: handleToggleMapping,
        onClearMap: handleClearMap,
        onSaveMap: handleSaveMap,
        onLoadMap: handleLoadMap,
        mapPointCount,
        maxMapPoints,
        voxelSize,
        onVoxelSizeChange: setVoxelSize,
        isProcessing
      },
      defaultPosition: { x: 20, y: 20 },
      defaultSize: { width: 380, height: 450 },
      minSize: { width: 300, height: 350 },
      maxSize: { width: 500, height: 600 },
      resizable: true,
      collapsible: true,
      visible: false
    },
    {
      id: 'rtabmap-controls',
      title: 'RTAB-Map SLAM For Navigation',
      icon: '🎯',
      component: RTABMapWidget,
      props: {
        isRTABMapActive,
        onToggleRTABMap: handleToggleRTABMap,
        onResetMap: handleResetRTABMap,
        onSaveMap: handleSaveRTABMap,
        onLoadMap: handleLoadRTABMap,
        showOccupancyGrid,
        onToggleOccupancyGrid: setShowOccupancyGrid,
        showPointCloud,
        onTogglePointCloud: setShowPointCloud,
        showTrajectory,
        onToggleTrajectory: setShowTrajectory,
        showLoopClosures,
        onToggleLoopClosures: setShowLoopClosures,
        gridOpacity,
        onGridOpacityChange: setGridOpacity,
        mapQuality,
        onMapQualityChange: setMapQuality,
        loopClosureThreshold,
        onLoopClosureThresholdChange: setLoopClosureThreshold,
        rtabMapStats
      },
      defaultPosition: { x: 20, y: window.innerHeight - 520 },
      defaultSize: { width: 380, height: 500 },
      minSize: { width: 300, height: 400 },
      maxSize: { width: 500, height: 700 },
      resizable: true,
      collapsible: true,
      visible: false
    },
    {
      id: 'navigation-panel',
      title: 'Navigation Control',
      icon: '🧭',
      component: NavigationControlPanel,
      props: {
        ros: rosRef.current,
        isConnected,
        mapData: occupancyMapData,
        onWaypointAdded: (waypoint: any) => console.log('Waypoint added:', waypoint),
        onNavigationStarted: (waypoint: any) => console.log('Navigation started to:', waypoint)
      },
      defaultPosition: { x: window.innerWidth - 420, y: 20 },
      defaultSize: { width: 400, height: 850 },
      minSize: { width: 350, height: 700 },
      maxSize: { width: 600, height: 1000 },
      resizable: true,
      collapsible: true,
      visible: false
    },
    {
      id: 'battery-monitor',
      title: 'Battery Monitor',
      icon: '🔋',
      component: BatteryWidget,
      props: {
        ros: rosRef.current,
        isConnected,
        compact: false
      },
      defaultPosition: { x: 20, y: window.innerHeight - 520 },
      defaultSize: { width: 400, height: 500 },
      minSize: { width: 300, height: 400 },
      maxSize: { width: 500, height: 700 },
      resizable: true,
      collapsible: true,
      visible: true
    }
  ]);

  const handleWidgetToggle = useCallback((widgetId: string, visible: boolean) => {
    setWidgets(prev => prev.map(widget => 
      widget.id === widgetId ? { ...widget, visible } : widget
    ));
  }, []);

  // Update widget props when state changes
  useEffect(() => {
    setWidgets(prev => prev.map(widget => {
      if (widget.id === 'mapping-controls') {
        return {
          ...widget,
          props: {
            ...widget.props,
            enableMapping,
            mapPointCount,
            voxelSize,
            isProcessing
          }
        };
      } else if (widget.id === 'rtabmap-controls') {
        return {
          ...widget,
          props: {
            ...widget.props,
            isRTABMapActive,
            showOccupancyGrid,
            showPointCloud,
            showTrajectory,
            showLoopClosures,
            gridOpacity,
            mapQuality,
            loopClosureThreshold,
            rtabMapStats
          }
        };
      } else if (widget.id === 'navigation-panel') {
        return {
          ...widget,
          props: {
            ...widget.props,
            ros: rosRef.current,
            isConnected,
            mapData: occupancyMapData
          }
        };
      } else if (widget.id === 'battery-monitor') {
        return {
          ...widget,
          props: {
            ...widget.props,
            ros: rosRef.current,
            isConnected
          }
        };
      }
      return widget;
    }));
  }, [
    enableMapping, mapPointCount, voxelSize, isProcessing,
    isRTABMapActive, showOccupancyGrid, showPointCloud, showTrajectory, showLoopClosures,
    gridOpacity, mapQuality, loopClosureThreshold, rtabMapStats,
    isConnected, occupancyMapData
  ]);

  return (
    <div style={{ width: '100%', height: '100vh', position: 'relative' }}>
      {/* Full-screen 3D Canvas */}
      <Canvas
        camera={{ position: [5, 5, 5] as [number, number, number], fov: 75 }}
        style={{ 
          background: '#000',
          position: 'absolute',
          top: 0,
          left: 0,
          width: '100%',
          height: '100%',
          zIndex: 1
        }}
        gl={{ 
          antialias: true, 
          alpha: false,
          powerPreference: "high-performance"
        }}
      >
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10] as [number, number, number]} />
        
        <PointCloudRenderer data={pointCloudData} showPointCloud={showPointCloud} />
        
        <OccupancyGridRenderer 
          mapData={occupancyMapData}
          opacity={gridOpacity}
          showGrid={showOccupancyGrid}
        />
        
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
        fontSize: '12px',
        pointerEvents: 'none'
      }}>
        <div>Status: {connectionStatus}</div>
        <div>Points: {pointCount.toLocaleString()}</div>
        <div>3D + 2D View: {showOccupancyGrid ? 'Enabled' : 'Disabled'}</div>
        <div style={{ color: isConnected ? '#00ff00' : '#ff0000' }}>
          ● {isConnected ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      {/* Widget Manager */}
      <WidgetManager 
        widgets={widgets}
        onWidgetToggle={handleWidgetToggle}
      />
    </div>
  );
};

export default EnhancedPointCloudVisualization;
