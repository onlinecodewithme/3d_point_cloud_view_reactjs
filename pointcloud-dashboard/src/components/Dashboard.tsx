import React from 'react';

interface DashboardProps {
  isConnected: boolean;
  connectionStatus: string;
  pointCount: number;
  timestamp?: number;
}

const Dashboard: React.FC<DashboardProps> = ({ 
  isConnected, 
  connectionStatus, 
  pointCount, 
  timestamp 
}) => {
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

      <div style={{ marginTop: '20px', fontSize: '12px', color: '#888' }}>
        <p>Controls:</p>
        <ul style={{ margin: 0, paddingLeft: '20px' }}>
          <li>Left click + drag: Rotate</li>
          <li>Right click + drag: Pan</li>
          <li>Scroll: Zoom</li>
        </ul>
      </div>
    </div>
  );
};

export default Dashboard;
