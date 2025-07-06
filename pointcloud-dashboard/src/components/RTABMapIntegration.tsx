import React, { useRef, useEffect, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

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

interface RTABMapIntegrationProps {
  rtabMapData: RTABMapData | null;
  showOccupancyGrid: boolean;
  showTrajectory: boolean;
  showLoopClosures: boolean;
  gridOpacity: number;
}

const RTABMapIntegration: React.FC<RTABMapIntegrationProps> = ({
  rtabMapData,
  showOccupancyGrid = true,
  showTrajectory = true,
  showLoopClosures = true,
  gridOpacity = 0.8
}) => {
  const occupancyGridRef = useRef<THREE.Mesh>(null);
  const trajectoryRef = useRef<THREE.Line>(null);
  const robotPoseRef = useRef<THREE.Group>(null);
  const mapCloudRef = useRef<THREE.Points>(null);
  const loopClosuresRef = useRef<THREE.Group>(null);

  // Geometries and materials
  const gridGeometry = useRef<THREE.PlaneGeometry>(new THREE.PlaneGeometry());
  const gridTexture = useRef<THREE.DataTexture | null>(null);
  const gridMaterial = useRef<THREE.MeshBasicMaterial>(
    new THREE.MeshBasicMaterial({
      transparent: true,
      opacity: gridOpacity,
      side: THREE.DoubleSide
    })
  );

  const trajectoryGeometry = useRef<THREE.BufferGeometry>(new THREE.BufferGeometry());
  const trajectoryMaterial = useRef<THREE.LineBasicMaterial>(
    new THREE.LineBasicMaterial({
      color: 0x00ff00,
      linewidth: 3
    })
  );

  // Robot pose visualization (arrow/robot model)
  const robotGeometry = useRef<THREE.ConeGeometry>(new THREE.ConeGeometry(0.1, 0.3, 8));
  const robotMaterial = useRef<THREE.MeshBasicMaterial>(
    new THREE.MeshBasicMaterial({ color: 0xff0000 })
  );

  // Update occupancy grid visualization
  useEffect(() => {
    if (!rtabMapData?.occupancyGrid || !showOccupancyGrid) return;

    const { data, width, height, resolution, origin } = rtabMapData.occupancyGrid;

    // Create texture from occupancy grid data
    const textureData = new Uint8Array(width * height * 4); // RGBA

    for (let i = 0; i < data.length; i++) {
      const value = data[i];
      const idx = i * 4;

      if (value === -1) {
        // Unknown space - gray
        textureData[idx] = 128;     // R
        textureData[idx + 1] = 128; // G
        textureData[idx + 2] = 128; // B
        textureData[idx + 3] = 100; // A (semi-transparent)
      } else if (value === 0) {
        // Free space - white
        textureData[idx] = 255;     // R
        textureData[idx + 1] = 255; // G
        textureData[idx + 2] = 255; // B
        textureData[idx + 3] = 50;  // A (very transparent)
      } else {
        // Occupied space - black
        textureData[idx] = 0;       // R
        textureData[idx + 1] = 0;   // G
        textureData[idx + 2] = 0;   // B
        textureData[idx + 3] = 255; // A (opaque)
      }
    }

    // Create and update texture
    if (gridTexture.current) {
      gridTexture.current.dispose();
    }

    gridTexture.current = new THREE.DataTexture(
      textureData,
      width,
      height,
      THREE.RGBAFormat
    );
    gridTexture.current.needsUpdate = true;
    gridTexture.current.flipY = true;

    // Update material
    gridMaterial.current.map = gridTexture.current;
    gridMaterial.current.needsUpdate = true;

    // Update geometry size
    const worldWidth = width * resolution;
    const worldHeight = height * resolution;
    
    gridGeometry.current.dispose();
    gridGeometry.current = new THREE.PlaneGeometry(worldWidth, worldHeight);

    // Position the grid according to origin
    if (occupancyGridRef.current) {
      occupancyGridRef.current.position.set(
        origin.position.x + worldWidth / 2,
        origin.position.y + worldHeight / 2,
        origin.position.z
      );
      occupancyGridRef.current.rotation.x = -Math.PI / 2; // Lay flat
    }

  }, [rtabMapData?.occupancyGrid, showOccupancyGrid, gridOpacity]);

  // Update robot trajectory
  useEffect(() => {
    if (!rtabMapData?.trajectory || !showTrajectory) return;

    const points = rtabMapData.trajectory.map(point => 
      new THREE.Vector3(point.position.x, point.position.y, point.position.z)
    );

    trajectoryGeometry.current.setFromPoints(points);
    trajectoryGeometry.current.attributes.position.needsUpdate = true;

  }, [rtabMapData?.trajectory, showTrajectory]);

  // Update robot pose
  useEffect(() => {
    if (!rtabMapData?.robotPose || !robotPoseRef.current) return;

    const { position, orientation } = rtabMapData.robotPose;
    
    // Set position
    robotPoseRef.current.position.set(position.x, position.y, position.z + 0.15);
    
    // Set orientation from quaternion
    const quaternion = new THREE.Quaternion(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w
    );
    robotPoseRef.current.setRotationFromQuaternion(quaternion);

  }, [rtabMapData?.robotPose]);

  // Update RTAB-Map point cloud
  useEffect(() => {
    if (!rtabMapData?.mapCloud || !mapCloudRef.current) return;

    const { points, colors } = rtabMapData.mapCloud;
    
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    
    if (colors) {
      geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    }

    if (mapCloudRef.current) {
      mapCloudRef.current.geometry.dispose();
      mapCloudRef.current.geometry = geometry;
    }

  }, [rtabMapData?.mapCloud]);

  // Update loop closures visualization
  useEffect(() => {
    if (!rtabMapData?.loopClosures || !showLoopClosures || !loopClosuresRef.current) return;

    // Clear previous loop closure lines
    while (loopClosuresRef.current.children.length > 0) {
      const child = loopClosuresRef.current.children[0];
      loopClosuresRef.current.remove(child);
      if (child instanceof THREE.Line) {
        child.geometry.dispose();
        child.material.dispose();
      }
    }

    // Add new loop closure lines
    rtabMapData.loopClosures.forEach((closure, index) => {
      if (!rtabMapData.trajectory) return;

      const fromPoint = rtabMapData.trajectory[closure.from];
      const toPoint = rtabMapData.trajectory[closure.to];

      if (fromPoint && toPoint) {
        const points = [
          new THREE.Vector3(fromPoint.position.x, fromPoint.position.y, fromPoint.position.z),
          new THREE.Vector3(toPoint.position.x, toPoint.position.y, toPoint.position.z)
        ];

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
          color: 0xffff00, // Yellow for loop closures
          opacity: closure.confidence,
          transparent: true
        });

        const line = new THREE.Line(geometry, material);
        loopClosuresRef.current?.add(line);
      }
    });

  }, [rtabMapData?.loopClosures, showLoopClosures]);

  // Animation frame
  useFrame(() => {
    // Optional: Add any real-time animations
  });

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      {/* Occupancy Grid */}
      {showOccupancyGrid && rtabMapData?.occupancyGrid && (
        <mesh
          ref={occupancyGridRef}
          geometry={gridGeometry.current}
          material={gridMaterial.current}
        />
      )}

      {/* Robot Trajectory */}
      {showTrajectory && rtabMapData?.trajectory && (
        <primitive
          object={new THREE.Line(trajectoryGeometry.current, trajectoryMaterial.current)}
        />
      )}

      {/* Current Robot Pose */}
      {rtabMapData?.robotPose && (
        <group ref={robotPoseRef}>
          <mesh geometry={robotGeometry.current} material={robotMaterial.current} />
          {/* Robot orientation indicator */}
          <arrowHelper args={[new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.5, 0xff0000]} />
        </group>
      )}

      {/* RTAB-Map Point Cloud */}
      {rtabMapData?.mapCloud && (
        <points ref={mapCloudRef}>
          <pointsMaterial
            size={0.02}
            vertexColors={!!rtabMapData.mapCloud.colors}
            sizeAttenuation={true}
          />
        </points>
      )}

      {/* Loop Closures */}
      {showLoopClosures && (
        <group ref={loopClosuresRef} />
      )}
    </group>
  );
};

export default RTABMapIntegration;
