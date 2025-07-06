import React from 'react';

interface RobotStatusOverlayProps {
  connectionStatus: string;
  isConnected: boolean;
  cameraFeedActive: boolean;
}

const RobotStatusOverlay: React.FC<RobotStatusOverlayProps> = ({
  connectionStatus,
  isConnected,
  cameraFeedActive
}) => {
  return (
    <div className="robot-status-overlay">
      <div className="status-header">
        <h3>🤖 Robot Status</h3>
      </div>
      
      <div className="status-items">
        <div className="status-item">
          <span className="status-label">ROS Connection:</span>
          <span className={`status-value ${isConnected ? 'connected' : 'disconnected'}`}>
            <span className="status-dot">●</span>
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
        
        <div className="status-item">
          <span className="status-label">Camera Feed:</span>
          <span className={`status-value ${cameraFeedActive ? 'active' : 'inactive'}`}>
            <span className="status-dot">●</span>
            {cameraFeedActive ? 'Active' : 'Inactive'}
          </span>
        </div>
        
        <div className="status-item">
          <span className="status-label">Bridge URL:</span>
          <span className="status-value">ws://localhost:9090</span>
        </div>
        
        <div className="status-item">
          <span className="status-label">Topics:</span>
          <div className="topic-list">
            <div className="topic-item">
              <span className="topic-name">/cmd_vel</span>
              <span className="topic-type">geometry_msgs/Twist</span>
            </div>
            <div className="topic-item">
              <span className="topic-name">/zed/zed_node/rgb/image_rect_color</span>
              <span className="topic-type">sensor_msgs/Image</span>
            </div>
            <div className="topic-item">
              <span className="topic-name">/map</span>
              <span className="topic-type">nav_msgs/OccupancyGrid</span>
            </div>
          </div>
        </div>
      </div>
      
      <div className="status-footer">
        <div className="timestamp">
          Last Update: {new Date().toLocaleTimeString()}
        </div>
      </div>
    </div>
  );
};

export default RobotStatusOverlay;
