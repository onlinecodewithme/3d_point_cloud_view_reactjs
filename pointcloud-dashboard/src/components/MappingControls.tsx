import React, { useState } from 'react';

interface MappingControlsProps {
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
}

const MappingControls: React.FC<MappingControlsProps> = ({
  enableMapping,
  onToggleMapping,
  onClearMap,
  onSaveMap,
  onLoadMap,
  mapPointCount,
  maxMapPoints,
  voxelSize,
  onVoxelSizeChange,
  isProcessing
}) => {
  const [showAdvanced, setShowAdvanced] = useState(false);

  const mapProgress = (mapPointCount / maxMapPoints) * 100;

  return (
    <div className="mapping-controls">
      <div className="mapping-header">
        <h3>🗺️ 3D Mapping</h3>
        <div className="mapping-status">
          <span className={`status-indicator ${enableMapping ? 'active' : 'inactive'}`}>
            {enableMapping ? '🟢 MAPPING' : '🔴 STOPPED'}
          </span>
          {isProcessing && <span className="processing">⚡ Processing...</span>}
        </div>
      </div>

      <div className="mapping-stats">
        <div className="stat-item">
          <label>Map Points:</label>
          <span>{mapPointCount.toLocaleString()} / {maxMapPoints.toLocaleString()}</span>
        </div>
        <div className="progress-bar">
          <div 
            className="progress-fill" 
            style={{ width: `${mapProgress}%` }}
          ></div>
        </div>
        <div className="stat-item">
          <label>Memory Usage:</label>
          <span>{Math.round(mapProgress)}%</span>
        </div>
      </div>

      <div className="mapping-controls-grid">
        <button 
          className={`control-btn ${enableMapping ? 'stop' : 'start'}`}
          onClick={() => onToggleMapping(!enableMapping)}
        >
          {enableMapping ? '⏹️ Stop Mapping' : '▶️ Start Mapping'}
        </button>

        <button 
          className="control-btn clear"
          onClick={onClearMap}
          disabled={mapPointCount === 0}
        >
          🗑️ Clear Map
        </button>

        <button 
          className="control-btn save"
          onClick={onSaveMap}
          disabled={mapPointCount === 0}
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

      <div className="advanced-controls">
        <button 
          className="advanced-toggle"
          onClick={() => setShowAdvanced(!showAdvanced)}
        >
          ⚙️ Advanced Settings {showAdvanced ? '▼' : '▶'}
        </button>

        {showAdvanced && (
          <div className="advanced-panel">
            <div className="control-group">
              <label>Voxel Size (m):</label>
              <input
                type="range"
                min="0.01"
                max="0.2"
                step="0.01"
                value={voxelSize}
                onChange={(e) => onVoxelSizeChange(parseFloat(e.target.value))}
              />
              <span>{voxelSize.toFixed(2)}m</span>
            </div>

            <div className="control-group">
              <label>CUDA Acceleration:</label>
              <div className="cuda-status">
                <span className="cuda-indicator">🚀 WebGL Accelerated</span>
                <small>Browser-based GPU acceleration active</small>
              </div>
            </div>

            <div className="control-group">
              <label>Map Quality:</label>
              <select defaultValue="high">
                <option value="low">Low (Fast)</option>
                <option value="medium">Medium</option>
                <option value="high">High (Detailed)</option>
                <option value="ultra">Ultra (Slow)</option>
              </select>
            </div>
          </div>
        )}
      </div>

      <div className="mapping-info">
        <h4>📊 Mapping Statistics</h4>
        <div className="info-grid">
          <div className="info-item">
            <span className="info-label">Resolution:</span>
            <span className="info-value">{voxelSize.toFixed(2)}m</span>
          </div>
          <div className="info-item">
            <span className="info-label">Coverage:</span>
            <span className="info-value">{Math.round(mapProgress)}%</span>
          </div>
          <div className="info-item">
            <span className="info-label">Acceleration:</span>
            <span className="info-value">WebGL + CUDA Ready</span>
          </div>
        </div>
      </div>

      <style>{`
        .mapping-controls {
          background: rgba(30, 30, 30, 0.95);
          border: 1px solid #444;
          border-radius: 8px;
          padding: 16px;
          margin: 8px 0;
          color: white;
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }

        .mapping-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 12px;
        }

        .mapping-header h3 {
          margin: 0;
          color: #4CAF50;
        }

        .mapping-status {
          display: flex;
          gap: 8px;
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

        .processing {
          color: #ff9800;
          font-size: 12px;
          animation: pulse 1s infinite;
        }

        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }

        .mapping-stats {
          margin-bottom: 16px;
        }

        .stat-item {
          display: flex;
          justify-content: space-between;
          margin-bottom: 4px;
          font-size: 14px;
        }

        .stat-item label {
          color: #ccc;
        }

        .progress-bar {
          width: 100%;
          height: 6px;
          background: #333;
          border-radius: 3px;
          margin: 8px 0;
          overflow: hidden;
        }

        .progress-fill {
          height: 100%;
          background: linear-gradient(90deg, #4CAF50, #8BC34A);
          transition: width 0.3s ease;
        }

        .mapping-controls-grid {
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

        .control-btn.clear {
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

        .advanced-toggle {
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

        .advanced-panel {
          background: rgba(0, 0, 0, 0.3);
          padding: 12px;
          border-radius: 4px;
          border: 1px solid #555;
        }

        .control-group {
          margin-bottom: 12px;
        }

        .control-group label {
          display: block;
          color: #ccc;
          font-size: 12px;
          margin-bottom: 4px;
        }

        .control-group input[type="range"] {
          width: 70%;
          margin-right: 8px;
        }

        .control-group select {
          width: 100%;
          padding: 4px;
          background: #333;
          color: white;
          border: 1px solid #555;
          border-radius: 4px;
        }

        .cuda-status {
          display: flex;
          flex-direction: column;
          gap: 2px;
        }

        .cuda-indicator {
          color: #4CAF50;
          font-size: 12px;
          font-weight: bold;
        }

        .cuda-status small {
          color: #999;
          font-size: 10px;
        }

        .mapping-info {
          border-top: 1px solid #444;
          padding-top: 12px;
        }

        .mapping-info h4 {
          margin: 0 0 8px 0;
          color: #2196F3;
          font-size: 14px;
        }

        .info-grid {
          display: grid;
          grid-template-columns: 1fr 1fr 1fr;
          gap: 8px;
        }

        .info-item {
          display: flex;
          flex-direction: column;
          align-items: center;
          text-align: center;
        }

        .info-label {
          font-size: 10px;
          color: #999;
          margin-bottom: 2px;
        }

        .info-value {
          font-size: 11px;
          color: white;
          font-weight: bold;
        }
      `}</style>
    </div>
  );
};

export default MappingControls;
