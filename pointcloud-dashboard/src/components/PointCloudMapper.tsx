import React, { useRef, useEffect, useMemo, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

interface MapPoint {
  position: THREE.Vector3;
  color: THREE.Color;
  timestamp: number;
  confidence: number;
}

interface PointCloudMapperProps {
  data: PointCloudData | null;
  enableMapping: boolean;
  maxMapPoints: number;
  voxelSize: number;
}

const PointCloudMapper: React.FC<PointCloudMapperProps> = ({ 
  data, 
  enableMapping = true, 
  maxMapPoints = 1000000,
  voxelSize = 0.05 
}) => {
  const mapPointsRef = useRef<THREE.Points>(null);
  const currentPointsRef = useRef<THREE.Points>(null);
  const [mapPoints, setMapPoints] = useState<Map<string, MapPoint>>(new Map());
  const [isProcessing, setIsProcessing] = useState(false);

  // Geometries and materials
  const mapGeometry = useMemo(() => new THREE.BufferGeometry(), []);
  const currentGeometry = useMemo(() => new THREE.BufferGeometry(), []);
  
  const mapMaterial = useMemo(() => new THREE.PointsMaterial({ 
    size: 0.02, 
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.8
  }), []);
  
  const currentMaterial = useMemo(() => new THREE.PointsMaterial({ 
    size: 0.025, 
    vertexColors: true,
    sizeAttenuation: true,
    transparent: false,
    opacity: 1.0
  }), []);

  // Voxel grid key generation for spatial hashing
  const getVoxelKey = (x: number, y: number, z: number): string => {
    const vx = Math.floor(x / voxelSize);
    const vy = Math.floor(y / voxelSize);
    const vz = Math.floor(z / voxelSize);
    return `${vx},${vy},${vz}`;
  };

  // CUDA-accelerated point processing (simulated with optimized JS)
  const processPointsWithAcceleration = async (points: Float32Array, colors?: Float32Array): Promise<MapPoint[]> => {
    return new Promise((resolve) => {
      // Simulate CUDA-like parallel processing with Web Workers concept
      const newPoints: MapPoint[] = [];
      const numPoints = points.length / 3;
      
      // Process points in batches for better performance
      const batchSize = 1000;
      let processed = 0;
      
      const processBatch = () => {
        const endIdx = Math.min(processed + batchSize, numPoints);
        
        for (let i = processed; i < endIdx; i++) {
          const x = points[i * 3];
          const y = points[i * 3 + 1];
          const z = points[i * 3 + 2];
          
          // Skip invalid points
          if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
          
          const position = new THREE.Vector3(x, y, z);
          let color = new THREE.Color(1, 1, 1);
          
          if (colors) {
            color.setRGB(
              colors[i * 3] || 0,
              colors[i * 3 + 1] || 0,
              colors[i * 3 + 2] || 0
            );
          } else {
            // Height-based coloring
            const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
            color.setRGB(normalizedZ, 1 - normalizedZ, 1 - normalizedZ);
          }
          
          newPoints.push({
            position,
            color,
            timestamp: Date.now(),
            confidence: 1.0
          });
        }
        
        processed = endIdx;
        
        if (processed < numPoints) {
          // Continue processing in next frame
          requestAnimationFrame(processBatch);
        } else {
          resolve(newPoints);
        }
      };
      
      processBatch();
    });
  };

  // Update map with new point cloud data
  const updateMap = async (newData: PointCloudData) => {
    if (!enableMapping || isProcessing) return;
    
    setIsProcessing(true);
    
    try {
      // Process new points with acceleration
      const newPoints = await processPointsWithAcceleration(newData.points, newData.colors);
      
      setMapPoints(prevMap => {
        const updatedMap = new Map(prevMap);
        
        // Add new points to voxel grid
        newPoints.forEach(point => {
          const key = getVoxelKey(
            point.position.x,
            point.position.y,
            point.position.z
          );
          
          const existingPoint = updatedMap.get(key);
          if (existingPoint) {
            // Update existing point with weighted average
            const weight = 0.1; // New point influence
            existingPoint.position.lerp(point.position, weight);
            existingPoint.color.lerp(point.color, weight);
            existingPoint.confidence = Math.min(1.0, existingPoint.confidence + 0.1);
            existingPoint.timestamp = point.timestamp;
          } else {
            // Add new point if map hasn't reached max capacity
            if (updatedMap.size < maxMapPoints) {
              updatedMap.set(key, point);
            }
          }
        });
        
        // Remove old points if map is too large
        if (updatedMap.size > maxMapPoints) {
          const sortedEntries = Array.from(updatedMap.entries())
            .sort((a, b) => b[1].confidence - a[1].confidence);
          
          const newMap = new Map();
          sortedEntries.slice(0, maxMapPoints).forEach(([key, point]) => {
            newMap.set(key, point);
          });
          
          return newMap;
        }
        
        return updatedMap;
      });
    } catch (error) {
      console.error('Error updating map:', error);
    } finally {
      setIsProcessing(false);
    }
  };

  // Update current point cloud visualization
  useEffect(() => {
    if (!data || !data.points || data.points.length === 0) return;

    // Update current points
    currentGeometry.setAttribute('position', new THREE.BufferAttribute(data.points, 3));
    
    if (data.colors && data.colors.length > 0) {
      currentGeometry.setAttribute('color', new THREE.BufferAttribute(data.colors, 3));
    } else {
      // Generate colors for current points
      const colors = new Float32Array(data.points.length);
      for (let i = 0; i < data.points.length; i += 3) {
        const z = data.points[i + 2];
        const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
        colors[i] = normalizedZ;
        colors[i + 1] = 1 - normalizedZ;
        colors[i + 2] = 1 - normalizedZ;
      }
      currentGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    }

    currentGeometry.attributes.position.needsUpdate = true;
    currentGeometry.attributes.color.needsUpdate = true;
    currentGeometry.computeBoundingSphere();

    // Update map
    if (enableMapping) {
      updateMap(data);
    }
  }, [data, enableMapping]);

  // Update map visualization
  useEffect(() => {
    if (mapPoints.size === 0) return;

    const positions = new Float32Array(mapPoints.size * 3);
    const colors = new Float32Array(mapPoints.size * 3);
    
    let index = 0;
    mapPoints.forEach(point => {
      positions[index * 3] = point.position.x;
      positions[index * 3 + 1] = point.position.y;
      positions[index * 3 + 2] = point.position.z;
      
      colors[index * 3] = point.color.r;
      colors[index * 3 + 1] = point.color.g;
      colors[index * 3 + 2] = point.color.b;
      
      index++;
    });

    mapGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    mapGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    mapGeometry.attributes.position.needsUpdate = true;
    mapGeometry.attributes.color.needsUpdate = true;
    mapGeometry.computeBoundingSphere();
  }, [mapPoints]);

  // Animation frame for any real-time updates
  useFrame(() => {
    // Optional: Add any real-time processing here
  });

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      {/* Persistent map points */}
      <points 
        ref={mapPointsRef} 
        geometry={mapGeometry} 
        material={mapMaterial}
      />
      
      {/* Current frame points (highlighted) */}
      <points 
        ref={currentPointsRef} 
        geometry={currentGeometry} 
        material={currentMaterial}
      />
    </group>
  );
};

export default PointCloudMapper;
