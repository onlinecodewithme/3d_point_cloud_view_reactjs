import React, { useState } from 'react';

interface RTABMapControlsProps {
  isRTABMapActive: boolean;
  onToggleRTABMap: (active: boolean) => void;
  onResetMap: () => void;
  onSaveMap: () => void;
  onLoadMap: () => void;
  showOccupancyGrid: boolean;
  onToggleOccupancyGrid: (show: boolean) => void;
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

const RTABMapControls: React.FC<RTABMapControlsProps> = ({
  isRTABMapActive,
  onToggleRTABMap,
  onResetMap,
  onSaveMap,
  onLoadMap,
  showOccupancyGrid,
  onToggleOccupancyGrid,
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
  const [showAdvanced, setShowAdvanced] = useState(false);
  const [showStats, setShowStats] = useState(true);

  return (
    <div className="rtabmap-controls">
      <div className="rtabmap-header">
        <h3>🤖 RTAB-Map SLAM</h3>
        <div className="rtabmap-status">
          <span className={`status-indicator ${isRTABMapActive ? 'active' : 'inactive'}`}>
            {isRTABMapActive ? '🟢 ACTIVE' : '🔴 STOPPED'}
          </span>
        </div>
      </div>

      {/* Main Controls */}
      <div className="rtabmap-main-controls">
        <button 
          className={`control-btn ${isRTABMapActive ? 'stop' : 'start'}`}
          onClick={() => onToggleRTABMap(!isRTABMapActive)}
        >
          {isRTABMapActive ? '⏹️ Stop SLAM' : '▶️ Start SLAM'}
        </button>

        <button 
          className="control-btn reset"
          onClick={onResetMap}
          disabled={!isRTABMapActive}
        >
          🔄 Reset Map
        </button>

        <button 
          className="control-btn save"
          onClick={onSaveMap}
          disabled={!isRTABMapActive}
        >
          💾 Save Map
        </button>

        <button 
          className="control-btn load"
          onClick={onLoadMap}
        >
          📁 Load Map
        </button>
      </div>

      {/* Visualization Controls */}
      <div className="visualization-controls">
        <h4>📊 Visualization</h4>
        
        <div className="control-row">
          <label className="checkbox-label">
            <input
              type="checkbox"
              checked={showOccupancyGrid}
              onChange={(e) => onToggleOccupancyGrid(e.target.checked)}
            />
            <span>Occupancy Grid</span>
          </label>
        </div>

        <div className="control-row">
          <label className="checkbox-label">
            <input
              type="checkbox"
              checked={showTrajectory}
              onChange={(e) => onToggleTrajectory(e.target.checked)}
            />
            <span>Robot Trajectory</span>
          </label>
        </div>

        <div className="control-row">
          <label className="checkbox-label">
            <input
              type="checkbox"
              checked={showLoopClosures}
              onChange={(e) => onToggleLoopClosures(e.target.checked)}
            />
            <span>Loop Closures</span>
          </label>
        </div>

        {showOccupancyGrid && (
          <div className="control-row">
            <label>Grid Opacity:</label>
            <input
              type="range"
              min="0.1"
              max="1.0"
              step="0.1"
              value={gridOpacity}
              onChange={(e) => onGridOpacityChange(parseFloat(e.target.value))}
            />
            <span>{Math.round(gridOpacity * 100)}%</span>
          </div>
        )}
      </div>

      {/* Advanced Settings */}
      <div className="advanced-section">
        <button 
          className="advanced-toggle"
          onClick={() => setShowAdvanced(!showAdvanced)}
        >
          ⚙️ Advanced SLAM Settings {showAdvanced ? '▼' : '▶'}
        </button>

        {showAdvanced && (
          <div className="advanced-panel">
            <div className="control-row">
              <label>Map Quality:</label>
              <select 
                value={mapQuality} 
                onChange={(e) => onMapQualityChange(e.target.value)}
              >
                <option value="fast">Fast (Low Quality)</option>
                <option value="balanced">Balanced</option>
                <option value="accurate">Accurate (High Quality)</option>
                <option value="ultra">Ultra (Slow)</option>
              </select>
            </div>

            <div className="control-row">
              <label>Loop Closure Threshold:</label>
              <input
                type="range"
                min="0.1"
                max="1.0"
                step="0.05"
                value={loopClosureThreshold}
                onChange={(e) => onLoopClosureThresholdChange(parseFloat(e.target.value))}
              />
              <span>{loopClosureThreshold.toFixed(2)}</span>
            </div>

            <div className="rtabmap-params">
              <h5>🔧 RTAB-Map Parameters</h5>
              <div className="param-info">
                <span>• Memory Management: Enabled</span>
                <span>• Loop Closure Detection: Active</span>
                <span>• RGB-D SLAM: ZED Camera</span>
                <span>• Graph Optimization: Real-time</span>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Statistics */}
      <div className="stats-section">
        <button 
          className="stats-toggle"
          onClick={() => setShowStats(!showStats)}
        >
          📈 SLAM Statistics {showStats ? '▼' : '▶'}
        </button>

        {showStats && (
          <div className="stats-panel">
            <div className="stat-row">
              <span className="stat-label">Total Nodes:</span>
              <span className="stat-value">{rtabMapStats.totalNodes.toLocaleString()}</span>
            </div>
            <div className="stat-row">
              <span className="stat-label">Loop Closures:</span>
              <span className="stat-value">{rtabMapStats.loopClosures}</span>
            </div>
            <div className="stat-row">
              <span className="stat-label">Map Size:</span>
              <span className="stat-value">{rtabMapStats.mapSize}</span>
            </div>
            <div className="stat-row">
              <span className="stat-label">Processing Time:</span>
              <span className="stat-value">{rtabMapStats.processingTime.toFixed(1)}ms</span>
            </div>
            <div className="stat-row">
              <span className="stat-label">Memory Usage:</span>
              <span className="stat-value">{rtabMapStats.memoryUsage.toFixed(1)}MB</span>
            </div>
          </div>
        )}
      </div>

      <style>{`
        .rtabmap-controls {
          background: rgba(25, 25, 25, 0.95);
          border: 1px solid #555;
          border-radius: 8px;
          padding: 16px;
          margin: 8px 0;
          color: white;
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }

        .rtabmap-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 16px;
          border-bottom: 1px solid #444;
          padding-bottom: 8px;
        }

        .rtabmap-header h3 {
          margin: 0;
          color: #FF6B35;
          font-size: 16px;
        }

        .rtabmap-status {
          display: flex;
          align-items: center;
        }

        .status-indicator {
          font-size: 12px;
          font-weight: bold;
        }

        .status-indicator.active {
          color: #4CAF50;
        }

        .status-indicator.inactive {
          color: #f44336;
        }

        .rtabmap-main-controls {
          display: grid;
          grid-template-columns: 1fr 1fr;
          gap: 8px;
          margin-bottom: 16px;
        }

        .control-btn {
          padding: 8px 12px;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          font-size: 12px;
          font-weight: bold;
          transition: all 0.2s ease;
        }

        .control-btn:disabled {
          opacity: 0.5;
          cursor: not-allowed;
        }

        .control-btn.start {
          background: #4CAF50;
          color: white;
        }

        .control-btn.stop {
          background: #f44336;
          color: white;
        }

        .control-btn.reset {
          background: #ff9800;
          color: white;
        }

        .control-btn.save {
          background: #2196F3;
          color: white;
        }

        .control-btn.load {
          background: #9C27B0;
          color: white;
        }

        .control-btn:hover:not(:disabled) {
          transform: translateY(-1px);
          box-shadow: 0 2px 4px rgba(0,0,0,0.3);
        }

        .visualization-controls {
          margin-bottom: 16px;
          border: 1px solid #444;
          border-radius: 4px;
          padding: 12px;
        }

        .visualization-controls h4 {
          margin: 0 0 12px 0;
          color: #2196F3;
          font-size: 14px;
        }

        .control-row {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 8px;
          font-size: 12px;
        }

        .control-row:last-child {
          margin-bottom: 0;
        }

        .checkbox-label {
          display: flex;
          align-items: center;
          cursor: pointer;
        }

        .checkbox-label input {
          margin-right: 8px;
        }

        .control-row input[type="range"] {
          width: 60px;
          margin: 0 8px;
        }

        .advanced-toggle, .stats-toggle {
          width: 100%;
          background: #333;
          color: white;
          border: 1px solid #555;
          padding: 8px;
          border-radius: 4px;
          cursor: pointer;
          font-size: 12px;
          margin-bottom: 8px;
        }

        .advanced-panel, .stats-panel {
          background: rgba(0, 0, 0, 0.3);
          padding: 12px;
          border-radius: 4px;
          border: 1px solid #555;
        }

        .control-row select {
          background: #333;
          color: white;
          border: 1px solid #555;
          border-radius: 4px;
          padding: 4px;
          font-size: 11px;
        }

        .rtabmap-params {
          margin-top: 12px;
          padding-top: 12px;
          border-top: 1px solid #444;
        }

        .rtabmap-params h5 {
          margin: 0 0 8px 0;
          color: #FF6B35;
          font-size: 12px;
        }

        .param-info {
          display: flex;
          flex-direction: column;
          gap: 4px;
        }

        .param-info span {
          font-size: 10px;
          color: #ccc;
        }

        .stats-section {
          margin-top: 16px;
        }

        .stat-row {
          display: flex;
          justify-content: space-between;
          margin-bottom: 4px;
          font-size: 11px;
        }

        .stat-label {
          color: #ccc;
        }

        .stat-value {
          color: white;
          font-weight: bold;
        }
      `}</style>
    </div>
  );
};

export default RTABMapControls;
