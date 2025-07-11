import React, { useState } from 'react';
import MappingControls from './MappingControls';
import RTABMapControls from './RTABMapControls';
import './Dashboard.css';

interface DashboardProps {
  isConnected: boolean;
  connectionStatus: string;
  pointCount: number;
  timestamp?: number;
  enableMapping: boolean;
  onToggleMapping: (enabled: boolean) => void;
  onClearMap: () => void;
  onSaveMap: () => void;
  onLoadMap: () => void;
  mapPointCount: number;
  maxMapPoints: number;
  voxelSize: number;
  onVoxelSizeChange: (size: number) => void;
  isProcessing: boolean;
  // RTAB-Map props
  isRTABMapActive: boolean;
  onToggleRTABMap: (active: boolean) => void;
  onResetRTABMap: () => void;
  onSaveRTABMap: () => void;
  onLoadRTABMap: () => void;
  showOccupancyGrid: boolean;
  onToggleOccupancyGrid: (show: boolean) => void;
  showPointCloud: boolean;
  onTogglePointCloud: (show: boolean) => void;
  showTrajectory: boolean;
  onToggleTrajectory: (show: boolean) => void;
  showLoopClosures: boolean;
  onToggleLoopClosures: (show: boolean) => void;
  gridOpacity: number;
  onGridOpacityChange: (opacity: number) => void;
  mapQuality: string;
  onMapQualityChange: (quality: string) => void;
  loopClosureThreshold: number;
  onLoopClosureThresholdChange: (threshold: number) => void;
  rtabMapStats: {
    totalNodes: number;
    loopClosures: number;
    mapSize: string;
    processingTime: number;
    memoryUsage: number;
  };
}

const Dashboard: React.FC<DashboardProps> = ({ 
  isConnected, 
  connectionStatus, 
  pointCount, 
  timestamp,
  enableMapping,
  onToggleMapping,
  onClearMap,
  onSaveMap,
  onLoadMap,
  mapPointCount,
  maxMapPoints,
  voxelSize,
  onVoxelSizeChange,
  isProcessing,
  isRTABMapActive,
  onToggleRTABMap,
  onResetRTABMap,
  onSaveRTABMap,
  onLoadRTABMap,
  showOccupancyGrid,
  onToggleOccupancyGrid,
  showPointCloud,
  onTogglePointCloud,
  showTrajectory,
  onToggleTrajectory,
  showLoopClosures,
  onToggleLoopClosures,
  gridOpacity,
  onGridOpacityChange,
  mapQuality,
  onMapQualityChange,
  loopClosureThreshold,
  onLoopClosureThresholdChange,
  rtabMapStats
}) => {
  const [showMapping, setShowMapping] = useState(true);
  const [showRTABMap, setShowRTABMap] = useState(true);

  const formatTimestamp = (ts?: number) => {
    if (!ts) return 'N/A';
    return new Date(ts).toLocaleTimeString();
  };

  const formatPointCount = (count: number) => {
    if (count > 1000000) {
      return `${(count / 1000000).toFixed(1)}M`;
    } else if (count > 1000) {
      return `${(count / 1000).toFixed(1)}K`;
    }
    return count.toString();
  };

  return (
    <div className="dashboard-page">
      <div className="dashboard-background">
        <div className="circuit-pattern"></div>
        <div className="floating-particles"></div>
      </div>
      
      <div className="dashboard-container">
        <div className="dashboard-header">
          <h1 className="dashboard-title">🤖 Xavier Control Dashboard</h1>
          <p className="dashboard-subtitle">Advanced Point Cloud & SLAM Management System</p>
        </div>

        {/* Status Cards */}
        <div className="dashboard-status-bar">
          <div className={`status-indicator-card ${isConnected ? 'connected' : 'disconnected'}`}>
            <div className="status-card-header">
              <span className={`status-dot ${isConnected ? 'connected' : 'disconnected'}`}></span>
              <span className="status-card-icon">🔗</span>
              <span className="status-card-title">Connection</span>
            </div>
            <div className={`status-card-value ${isConnected ? '' : 'disconnected'}`}>
              {connectionStatus}
            </div>
            <div className="status-card-subtitle">ROS Bridge Status</div>
          </div>

          <div className="status-indicator-card">
            <div className="status-card-header">
              <span className="status-card-icon">📊</span>
              <span className="status-card-title">Point Count</span>
            </div>
            <div className="status-card-value">{formatPointCount(pointCount)}</div>
            <div className="status-card-subtitle">Active Points</div>
          </div>

          <div className="status-indicator-card">
            <div className="status-card-header">
              <span className="status-card-icon">⏰</span>
              <span className="status-card-title">Last Update</span>
            </div>
            <div className="status-card-value">{formatTimestamp(timestamp)}</div>
            <div className="status-card-subtitle">Data Timestamp</div>
          </div>
        </div>

        {/* Point Cloud Information */}
        <div className="point-cloud-info">
          <h3>
            <span className="status-card-icon">☁️</span>
            Point Cloud Information
          </h3>
          <div className="info-grid">
            <div className="info-row">
              <span className="info-label">Topic:</span>
              <span className="info-value">/zed/zed_node/point_cloud/cloud_registered</span>
            </div>
            <div className="info-row">
              <span className="info-label">Frame Rate:</span>
              <span className="info-value">{isConnected ? 'Real-time' : 'N/A'}</span>
            </div>
            <div className="info-row">
              <span className="info-label">Data Type:</span>
              <span className="info-value">PointCloud2</span>
            </div>
            <div className="info-row">
              <span className="info-label">Processing:</span>
              <span className="info-value">{isProcessing ? 'Active' : 'Idle'}</span>
            </div>
          </div>
        </div>

        {/* Dashboard Sections */}
        <div className="dashboard-grid">
          {/* 3D Mapping Controls */}
          <div className="dashboard-section">
            <div 
              className="section-header"
              onClick={() => setShowMapping(!showMapping)}
            >
              <div className="section-header-content">
                <span className="section-icon">🗺️</span>
                <h3 className="section-title">3D Mapping & CUDA Acceleration</h3>
              </div>
              <span className={`section-toggle-icon ${showMapping ? 'expanded' : ''}`}>▼</span>
            </div>
            
            {showMapping && (
              <div className="section-content">
                <MappingControls
                  enableMapping={enableMapping}
                  onToggleMapping={onToggleMapping}
                  onClearMap={onClearMap}
                  onSaveMap={onSaveMap}
                  onLoadMap={onLoadMap}
                  mapPointCount={mapPointCount}
                  maxMapPoints={maxMapPoints}
                  voxelSize={voxelSize}
                  onVoxelSizeChange={onVoxelSizeChange}
                  isProcessing={isProcessing}
                />
              </div>
            )}
          </div>

          {/* RTAB-Map SLAM Controls */}
          <div className="dashboard-section">
            <div 
              className="section-header rtabmap-header"
              onClick={() => setShowRTABMap(!showRTABMap)}
            >
              <div className="section-header-content">
                <span className="section-icon">🧭</span>
                <h3 className="section-title">RTAB-Map SLAM Navigation</h3>
              </div>
              <span className={`section-toggle-icon ${showRTABMap ? 'expanded' : ''}`}>▼</span>
            </div>
            
            {showRTABMap && (
              <div className="section-content">
                <RTABMapControls
                  isRTABMapActive={isRTABMapActive}
                  onToggleRTABMap={onToggleRTABMap}
                  onResetMap={onResetRTABMap}
                  onSaveMap={onSaveRTABMap}
                  onLoadMap={onLoadRTABMap}
                  showOccupancyGrid={showOccupancyGrid}
                  onToggleOccupancyGrid={onToggleOccupancyGrid}
                  showPointCloud={showPointCloud}
                  onTogglePointCloud={onTogglePointCloud}
                  showTrajectory={showTrajectory}
                  onToggleTrajectory={onToggleTrajectory}
                  showLoopClosures={showLoopClosures}
                  onToggleLoopClosures={onToggleLoopClosures}
                  gridOpacity={gridOpacity}
                  onGridOpacityChange={onGridOpacityChange}
                  mapQuality={mapQuality}
                  onMapQualityChange={onMapQualityChange}
                  loopClosureThreshold={loopClosureThreshold}
                  onLoopClosureThresholdChange={onLoopClosureThresholdChange}
                  rtabMapStats={rtabMapStats}
                />
              </div>
            )}
          </div>
        </div>

        {/* Controls Information */}
        <div className="controls-info">
          <h4>
            <span className="status-card-icon">🎮</span>
            Visualization Controls
          </h4>
          <ul className="controls-list">
            <li>
              <span className="control-icon">🖱️</span>
              <span>Left click + drag: Rotate view</span>
            </li>
            <li>
              <span className="control-icon">🖱️</span>
              <span>Right click + drag: Pan view</span>
            </li>
            <li>
              <span className="control-icon">🔄</span>
              <span>Mouse wheel: Zoom in/out</span>
            </li>
            <li>
              <span className="control-icon">⌨️</span>
              <span>Keyboard shortcuts available in controls</span>
            </li>
          </ul>
        </div>
      </div>
    </div>
  );
};

export default Dashboard;
