import React, { useEffect, useRef, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';

interface OccupancyMapViewerProps {
  ros: any;
  isConnected: boolean;
  isExpanded: boolean;
  onExpandToggle: () => void;
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

// 3D Grid Renderer Component
const MapGridRenderer: React.FC<{ mapData: OccupancyMapData | null; isExpanded: boolean }> = ({ 
  mapData, 
  isExpanded 
}) => {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const [hoveredCell, setHoveredCell] = useState<{ x: number; y: number } | null>(null);

  useEffect(() => {
    if (!mapData || !meshRef.current) return;

    const { width, height, data, resolution } = mapData;
    const mesh = meshRef.current;
    
    // Create instances for occupied cells
    const occupiedCells: { x: number; y: number; value: number }[] = [];
    
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const index = y * width + x;
        const value = data[index];
        
        // Only render occupied cells (value > 50) and unknown cells (value = -1)
        if (value > 50 || value === -1) {
          occupiedCells.push({ x, y, value });
        }
      }
    }

    // Update instance count
    mesh.count = occupiedCells.length;
    
    // Set up instance matrices and colors
    const matrix = new THREE.Matrix4();
    const color = new THREE.Color();
    
    occupiedCells.forEach((cell, i) => {
      // Position in world coordinates
      const worldX = (cell.x - width / 2) * resolution;
      const worldY = (cell.y - height / 2) * resolution;
      const worldZ = 0;
      
      // Set instance matrix
      matrix.setPosition(worldX, worldY, worldZ);
      mesh.setMatrixAt(i, matrix);
      
      // Set instance color based on occupancy value
      if (cell.value === -1) {
        color.setHex(0x888888); // Gray for unknown
      } else if (cell.value > 80) {
        color.setHex(0x000000); // Black for highly occupied
      } else {
        color.setHex(0x444444); // Dark gray for occupied
      }
      
      mesh.setColorAt(i, color);
    });
    
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) {
      mesh.instanceColor.needsUpdate = true;
    }
    
  }, [mapData]);

  useFrame(() => {
    if (meshRef.current && !isExpanded) {
      // Subtle rotation when not expanded
      meshRef.current.rotation.z += 0.001;
    }
  });

  if (!mapData) return null;

  const cellSize = mapData.resolution;

  return (
    <instancedMesh
      ref={meshRef}
      args={[undefined, undefined, 10000]} // Max instances
    >
      <boxGeometry args={[cellSize, cellSize, cellSize * 0.1]} />
      <meshLambertMaterial />
    </instancedMesh>
  );
};

const OccupancyMapViewer: React.FC<OccupancyMapViewerProps> = ({
  ros,
  isConnected,
  isExpanded,
  onExpandToggle
}) => {
  const [mapData, setMapData] = useState<OccupancyMapData | null>(null);
  const [mapStats, setMapStats] = useState({ occupied: 0, free: 0, unknown: 0 });
  const topicRef = useRef<any>(null);

  useEffect(() => {
    if (!ros || !isConnected) {
      setMapData(null);
      return;
    }

    try {
      // Subscribe to occupancy grid map topic
      const mapTopic = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
      });

      topicRef.current = mapTopic;

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

          setMapData(mapData);
          
          // Calculate statistics
          const stats = { occupied: 0, free: 0, unknown: 0 };
          message.data.forEach((value: number) => {
            if (value === -1) stats.unknown++;
            else if (value > 50) stats.occupied++;
            else stats.free++;
          });
          setMapStats(stats);

          console.log('Received occupancy map:', {
            width: mapData.width,
            height: mapData.height,
            resolution: mapData.resolution,
            totalCells: mapData.data.length,
            stats
          });
        } catch (error) {
          console.error('Error processing occupancy map:', error);
        }
      });

      console.log('Subscribed to occupancy map topic');
    } catch (error) {
      console.error('Failed to subscribe to occupancy map:', error);
    }

    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
      }
    };
  }, [ros, isConnected]);

  return (
    <div className={`occupancy-map-viewer ${isExpanded ? 'expanded' : 'compact'}`}>
      <div className="map-header">
        <h3>🗺️ Occupancy Map</h3>
        <div className="map-controls">
          <button 
            className="expand-btn"
            onClick={onExpandToggle}
            title={isExpanded ? 'Minimize' : 'Expand'}
          >
            {isExpanded ? '🔽' : '🔼'}
          </button>
        </div>
      </div>

      {mapData ? (
        <>
          <div className="map-canvas-container">
            <Canvas
              camera={{ 
                position: (isExpanded ? [0, 0, 10] : [0, 0, 5]) as [number, number, number],
                fov: 60 
              }}
              style={{ background: '#f0f0f0' }}
            >
              <ambientLight intensity={0.6} />
              <directionalLight 
                position={[10, 10, 5] as [number, number, number]} 
                intensity={0.8}
                castShadow
              />
              
              <MapGridRenderer mapData={mapData} isExpanded={isExpanded} />
              
              {isExpanded && (
                <OrbitControls 
                  enablePan={true}
                  enableZoom={true}
                  enableRotate={true}
                  maxDistance={50}
                  minDistance={1}
                />
              )}
              
            </Canvas>
          </div>

          <div className="map-info">
            <div className="map-stats">
              <div className="stat-item">
                <span className="stat-label">Size:</span>
                <span className="stat-value">{mapData.width}×{mapData.height}</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Resolution:</span>
                <span className="stat-value">{mapData.resolution.toFixed(3)}m</span>
              </div>
              {isExpanded && (
                <>
                  <div className="stat-item">
                    <span className="stat-label">Occupied:</span>
                    <span className="stat-value occupied">{mapStats.occupied}</span>
                  </div>
                  <div className="stat-item">
                    <span className="stat-label">Free:</span>
                    <span className="stat-value free">{mapStats.free}</span>
                  </div>
                  <div className="stat-item">
                    <span className="stat-label">Unknown:</span>
                    <span className="stat-value unknown">{mapStats.unknown}</span>
                  </div>
                </>
              )}
            </div>

            {isExpanded && (
              <div className="map-legend">
                <h4>Legend</h4>
                <div className="legend-items">
                  <div className="legend-item">
                    <div className="legend-color occupied"></div>
                    <span>Occupied</span>
                  </div>
                  <div className="legend-item">
                    <div className="legend-color free"></div>
                    <span>Free</span>
                  </div>
                  <div className="legend-item">
                    <div className="legend-color unknown"></div>
                    <span>Unknown</span>
                  </div>
                </div>
              </div>
            )}
          </div>
        </>
      ) : (
        <div className="no-map-overlay">
          <div className="no-map-message">
            <h4>🗺️ No Map Data</h4>
            <p>
              {!isConnected 
                ? 'Waiting for ROS connection...' 
                : 'Waiting for occupancy map from /map topic'
              }
            </p>
            <div className="loading-spinner"></div>
          </div>
        </div>
      )}

      <div className="connection-status">
        <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
          ● {isConnected ? 'Connected' : 'Disconnected'}
        </span>
      </div>
    </div>
  );
};

export default OccupancyMapViewer;
