import React, { useRef, useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stats } from '@react-three/drei';
import * as THREE from 'three';
import PointCloudMapper from './components/PointCloudMapper';
import RosConnection from './components/RosConnection';
import Dashboard from './components/Dashboard';
import RTABMapIntegration from './components/RTABMapIntegration';
import RTABMapRosConnection from './components/RTABMapRosConnection';
import './App.css';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

const App: React.FC = () => {
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [pointCount, setPointCount] = useState(0);
  
  // 3D Mapping state
  const [enableMapping, setEnableMapping] = useState(true);
  const [mapPointCount, setMapPointCount] = useState(0);
  const [maxMapPoints] = useState(1000000); // 1M points max
  const [voxelSize, setVoxelSize] = useState(0.05); // 5cm voxels
  const [isProcessing, setIsProcessing] = useState(false);
  const mapperRef = useRef<any>(null);

  // RTAB-Map state
  const [isRTABMapActive, setIsRTABMapActive] = useState(false);
  const [rtabMapData, setRTABMapData] = useState<any>(null);
  const [showOccupancyGrid, setShowOccupancyGrid] = useState(true);
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [showLoopClosures, setShowLoopClosures] = useState(true);
  const [gridOpacity, setGridOpacity] = useState(0.8);
  const [mapQuality, setMapQuality] = useState('balanced');
  const [loopClosureThreshold, setLoopClosureThreshold] = useState(0.5);
  const [rtabMapStats, setRTABMapStats] = useState({
    totalNodes: 0,
    loopClosures: 0,
    mapSize: '0 MB',
    processingTime: 0,
    memoryUsage: 0
  });

  const handlePointCloudUpdate = (data: PointCloudData) => {
    setPointCloudData(data);
    setPointCount(data.points.length / 3); // Each point has x, y, z coordinates
  };

  // Mapping control handlers
  const handleToggleMapping = (enabled: boolean) => {
    setEnableMapping(enabled);
  };

  const handleClearMap = () => {
    // This would be implemented in the PointCloudMapper component
    setMapPointCount(0);
    console.log('Clearing 3D map...');
  };

  const handleSaveMap = () => {
    // Save map to local storage or file
    console.log('Saving 3D map...');
    // Implementation would export the map data
  };

  const handleLoadMap = () => {
    // Load map from file
    console.log('Loading 3D map...');
    // Implementation would import map data
  };

  const handleVoxelSizeChange = (size: number) => {
    setVoxelSize(size);
  };

  // RTAB-Map control handlers
  const handleToggleRTABMap = (active: boolean) => {
    setIsRTABMapActive(active);
  };

  const handleResetRTABMap = () => {
    console.log('Resetting RTAB-Map...');
    setRTABMapData(null);
    setRTABMapStats({
      totalNodes: 0,
      loopClosures: 0,
      mapSize: '0 MB',
      processingTime: 0,
      memoryUsage: 0
    });
  };

  const handleSaveRTABMap = () => {
    console.log('Saving RTAB-Map...');
    // Implementation would save RTAB-Map database
  };

  const handleLoadRTABMap = () => {
    console.log('Loading RTAB-Map...');
    // Implementation would load RTAB-Map database
  };

  const handleRTABMapUpdate = (data: any) => {
    setRTABMapData(data);
  };

  const handleRTABMapStatusChange = (active: boolean) => {
    // This is handled by the connection component
  };

  const handleRTABMapStatsUpdate = (stats: any) => {
    setRTABMapStats(stats);
  };

  // Simulate map point count updates (in real implementation, this would come from PointCloudMapper)
  useEffect(() => {
    if (enableMapping && pointCloudData) {
      // Simulate gradual map building
      setMapPointCount(prev => Math.min(prev + Math.floor(pointCount * 0.1), maxMapPoints));
    }
  }, [pointCloudData, enableMapping, pointCount, maxMapPoints]);

  return (
    <div className="app">
      <div className="dashboard-panel">
        <Dashboard 
          isConnected={isConnected}
          connectionStatus={connectionStatus}
          pointCount={pointCount}
          timestamp={pointCloudData?.timestamp}
          enableMapping={enableMapping}
          onToggleMapping={handleToggleMapping}
          onClearMap={handleClearMap}
          onSaveMap={handleSaveMap}
          onLoadMap={handleLoadMap}
          mapPointCount={mapPointCount}
          maxMapPoints={maxMapPoints}
          voxelSize={voxelSize}
          onVoxelSizeChange={handleVoxelSizeChange}
          isProcessing={isProcessing}
          isRTABMapActive={isRTABMapActive}
          onToggleRTABMap={handleToggleRTABMap}
          onResetRTABMap={handleResetRTABMap}
          onSaveRTABMap={handleSaveRTABMap}
          onLoadRTABMap={handleLoadRTABMap}
          showOccupancyGrid={showOccupancyGrid}
          onToggleOccupancyGrid={setShowOccupancyGrid}
          showTrajectory={showTrajectory}
          onToggleTrajectory={setShowTrajectory}
          showLoopClosures={showLoopClosures}
          onToggleLoopClosures={setShowLoopClosures}
          gridOpacity={gridOpacity}
          onGridOpacityChange={setGridOpacity}
          mapQuality={mapQuality}
          onMapQualityChange={setMapQuality}
          loopClosureThreshold={loopClosureThreshold}
          onLoopClosureThresholdChange={setLoopClosureThreshold}
          rtabMapStats={rtabMapStats}
        />
      </div>
      
      <div className="visualization-container">
        <Canvas
          camera={{ position: [5, 5, 5] as [number, number, number], fov: 75 }}
          style={{ background: '#000' }}
          gl={{ 
            antialias: true, 
            alpha: false,
            powerPreference: "high-performance" // Request high-performance GPU
          }}
        >
          <ambientLight intensity={0.5} />
          <pointLight position={[10, 10, 10] as [number, number, number]} />
          
          {/* 3D Mapping with CUDA-ready acceleration */}
          <PointCloudMapper 
            data={pointCloudData}
            enableMapping={enableMapping}
            maxMapPoints={maxMapPoints}
            voxelSize={voxelSize}
          />

          {/* RTAB-Map SLAM Integration */}
          <RTABMapIntegration
            rtabMapData={rtabMapData}
            showOccupancyGrid={showOccupancyGrid}
            showTrajectory={showTrajectory}
            showLoopClosures={showLoopClosures}
            gridOpacity={gridOpacity}
          />
          
          <OrbitControls 
            enablePan={true}
            enableZoom={true}
            enableRotate={true}
            maxDistance={50}
            minDistance={0.1}
          />
          
          <Stats />
          
          {/* Enhanced grid helper for mapping reference */}
          <gridHelper args={[20, 20]} />
          
          {/* Axes helper for orientation */}
          <axesHelper args={[5]} />
          
          {/* Additional lighting for better visualization */}
          <directionalLight 
            position={[10, 10, 5] as [number, number, number]} 
            intensity={0.3}
            castShadow
          />
          <hemisphereLight 
            args={["#ffffff", "#444444", 0.2]}
          />
        </Canvas>
      </div>

      <RosConnection
        onPointCloudUpdate={handlePointCloudUpdate}
        onConnectionStatusChange={setIsConnected}
        onStatusMessageChange={setConnectionStatus}
      />

      {/* RTAB-Map ROS Connection */}
      <RTABMapRosConnection
        onRTABMapUpdate={handleRTABMapUpdate}
        onRTABMapStatusChange={handleRTABMapStatusChange}
        onRTABMapStatsUpdate={handleRTABMapStatsUpdate}
        isRTABMapActive={isRTABMapActive}
        mapQuality={mapQuality}
        loopClosureThreshold={loopClosureThreshold}
      />

      {/* Performance monitoring overlay */}
      <div className="performance-overlay">
        <div className="perf-item">
          <span>🎯 Current: {pointCount.toLocaleString()} pts</span>
        </div>
        <div className="perf-item">
          <span>🗺️ Map: {mapPointCount.toLocaleString()} pts</span>
        </div>
        <div className="perf-item">
          <span>⚡ {enableMapping ? 'MAPPING' : 'STOPPED'}</span>
        </div>
        <div className="perf-item">
          <span>🚀 WebGL + CUDA Ready</span>
        </div>
      </div>

      <style>{`
        .performance-overlay {
          position: absolute;
          top: 10px;
          right: 10px;
          background: rgba(0, 0, 0, 0.8);
          color: white;
          padding: 8px;
          border-radius: 4px;
          font-size: 11px;
          font-family: monospace;
          z-index: 1000;
          border: 1px solid #333;
        }

        .perf-item {
          margin-bottom: 2px;
        }

        .perf-item:last-child {
          margin-bottom: 0;
        }
      `}</style>
    </div>
  );
};

export default App;
