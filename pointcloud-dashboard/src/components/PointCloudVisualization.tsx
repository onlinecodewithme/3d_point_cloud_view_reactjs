import React, { useRef, useEffect, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

interface PointCloudVisualizationProps {
  data: PointCloudData | null;
}

const PointCloudVisualization: React.FC<PointCloudVisualizationProps> = ({ data }) => {
  const pointsRef = useRef<THREE.Points>(null);
  const geometryRef = useRef<THREE.BufferGeometry>(null);

  // Create geometry and material with enhanced settings for better colors
  const geometry = useMemo(() => new THREE.BufferGeometry(), []);
  const material = useMemo(() => new THREE.PointsMaterial({ 
    size: 0.025, // Slightly larger points for better visibility
    vertexColors: true,
    sizeAttenuation: true,
    transparent: false,
    opacity: 1.0,
    // Enhanced color settings for more vivid appearance like RViz
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
        <meshBasicMaterial color="red" />
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

export default PointCloudVisualization;
