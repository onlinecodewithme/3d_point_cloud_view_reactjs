import React, { useState } from 'react';
import MappingControls from './MappingControls';
import RTABMapControls from './RTABMapControls';

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
    <div>
      <h2 style={{ margin: '0 0 20px 0', color: '#007bff' }}>Point Cloud Dashboard</h2>
      
      <div className={`dashboard-item ${isConnected ? '' : 'disconnected'}`}>
        <h3>Connection Status</h3>
        <div className="value">
          <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}></span>
          {connectionStatus}
        </div>
      </div>

      <div className="dashboard-item">
        <h3>Point Count</h3>
        <div className="value">{formatPointCount(pointCount)}</div>
      </div>

      <div className="dashboard-item">
        <h3>Last Update</h3>
        <div className="value">{formatTimestamp(timestamp)}</div>
      </div>

      <div className="point-cloud-info">
        <h3>Point Cloud Info</h3>
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
      </div>

      {/* 3D Mapping Controls */}
      <div className="mapping-section">
        <div className="section-header">
          <button 
            className="section-toggle"
            onClick={() => setShowMapping(!showMapping)}
          >
            {showMapping ? '▼' : '▶'} 3D Mapping & CUDA Acceleration
          </button>
        </div>
        
        {showMapping && (
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
        )}
      </div>

      {/* RTAB-Map SLAM Controls */}
      <div className="rtabmap-section">
        <div className="section-header">
          <button 
            className="section-toggle rtabmap-toggle"
            onClick={() => setShowRTABMap(!showRTABMap)}
          >
            {showRTABMap ? '▼' : '▶'} RTAB-Map SLAM for Navigation
          </button>
        </div>
        
        {showRTABMap && (
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
        )}
      </div>

      <div style={{ marginTop: '20px', fontSize: '12px', color: '#888' }}>
        <p>Controls:</p>
        <ul style={{ margin: 0, paddingLeft: '20px' }}>
          <li>Left click + drag: Rotate</li>
          <li>Right click + drag: Pan</li>
          <li>Scroll: Zoom</li>
        </ul>
      </div>

      <style>{`
        .mapping-section, .rtabmap-section {
          margin: 16px 0;
          border: 1px solid #444;
          border-radius: 8px;
          background: rgba(20, 20, 20, 0.8);
        }

        .section-header {
          padding: 0;
        }

        .section-toggle {
          width: 100%;
          background: linear-gradient(135deg, #2196F3, #1976D2);
          color: white;
          border: none;
          padding: 12px 16px;
          border-radius: 8px 8px 0 0;
          cursor: pointer;
          font-size: 14px;
          font-weight: bold;
          text-align: left;
          transition: all 0.3s ease;
        }

        .section-toggle.rtabmap-toggle {
          background: linear-gradient(135deg, #FF6B35, #E55A2B);
        }

        .section-toggle:hover {
          background: linear-gradient(135deg, #1976D2, #1565C0);
          transform: translateY(-1px);
        }

        .section-toggle.rtabmap-toggle:hover {
          background: linear-gradient(135deg, #E55A2B, #D14A1F);
        }

        .section-toggle:active {
          transform: translateY(0);
        }
      `}</style>
    </div>
  );
};

export default Dashboard;
