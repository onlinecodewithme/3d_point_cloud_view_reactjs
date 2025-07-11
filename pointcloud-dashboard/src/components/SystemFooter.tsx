import React, { useState, useEffect } from 'react';
import './SystemFooter.css';

interface SystemStats {
  cpu: number;
  memory: number;
  network: {
    upload: number;
    download: number;
  };
  timestamp: number;
}

interface SystemFooterProps {
  isDrawerOpen?: boolean;
}

const SystemFooter: React.FC<SystemFooterProps> = ({ isDrawerOpen = true }) => {
  const [systemStats, setSystemStats] = useState<SystemStats>({
    cpu: 0,
    memory: 0,
    network: { upload: 0, download: 0 },
    timestamp: Date.now()
  });
  const [isVisible, setIsVisible] = useState(true);

  // Generate mock system data
  const generateSystemStats = (): SystemStats => {
    return {
      cpu: Math.random() * 100,
      memory: 60 + Math.random() * 30, // 60-90% usage
      network: {
        upload: Math.random() * 1024, // KB/s
        download: Math.random() * 2048 // KB/s
      },
      timestamp: Date.now()
    };
  };

  useEffect(() => {
    // Update system stats every 2 seconds
    const interval = setInterval(() => {
      setSystemStats(generateSystemStats());
    }, 2000);

    // Initial load
    setSystemStats(generateSystemStats());

    return () => clearInterval(interval);
  }, []);

  const formatBytes = (bytes: number): string => {
    if (bytes < 1024) return `${bytes.toFixed(0)} B/s`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB/s`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB/s`;
  };

  const getStatusColor = (value: number, type: 'cpu' | 'memory'): string => {
    if (type === 'cpu') {
      if (value < 50) return '#00ff41';
      if (value < 80) return '#ffaa00';
      return '#ff4444';
    } else {
      if (value < 70) return '#00ff41';
      if (value < 85) return '#ffaa00';
      return '#ff4444';
    }
  };

  const getCurrentTime = (): string => {
    return new Date().toLocaleTimeString();
  };

  if (!isVisible) return null;

  return (
    <div className={`system-footer ${!isDrawerOpen ? 'drawer-closed' : ''}`}>
      <div className="footer-content">
        <div className="system-info">
          <div className="info-group">
            <div className="info-icon">🖥️</div>
            <div className="info-details">
              <div className="info-label">CPU</div>
              <div className="info-value" style={{ color: getStatusColor(systemStats.cpu, 'cpu') }}>
                {systemStats.cpu.toFixed(1)}%
              </div>
            </div>
            <div className="info-bar">
              <div 
                className="info-fill"
                style={{
                  width: `${systemStats.cpu}%`,
                  backgroundColor: getStatusColor(systemStats.cpu, 'cpu')
                }}
              />
            </div>
          </div>

          <div className="info-group">
            <div className="info-icon">💾</div>
            <div className="info-details">
              <div className="info-label">Memory</div>
              <div className="info-value" style={{ color: getStatusColor(systemStats.memory, 'memory') }}>
                {systemStats.memory.toFixed(1)}%
              </div>
            </div>
            <div className="info-bar">
              <div 
                className="info-fill"
                style={{
                  width: `${systemStats.memory}%`,
                  backgroundColor: getStatusColor(systemStats.memory, 'memory')
                }}
              />
            </div>
          </div>

          <div className="info-group network">
            <div className="info-icon">📡</div>
            <div className="info-details">
              <div className="info-label">Network</div>
              <div className="network-stats">
                <div className="network-item">
                  <span className="network-direction">↑</span>
                  <span className="network-value">{formatBytes(systemStats.network.upload)}</span>
                </div>
                <div className="network-item">
                  <span className="network-direction">↓</span>
                  <span className="network-value">{formatBytes(systemStats.network.download)}</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        <div className="footer-center">
          <div className="system-status">
            <div className="status-indicator online"></div>
            <span className="status-text">System Online</span>
          </div>
          <div className="current-time">
            {getCurrentTime()}
          </div>
        </div>

        <div className="footer-actions">
          <button 
            className="footer-button"
            onClick={() => setIsVisible(false)}
            title="Hide footer"
          >
            <span className="button-icon">👁️</span>
          </button>
          <button 
            className="footer-button"
            onClick={() => setSystemStats(generateSystemStats())}
            title="Refresh stats"
          >
            <span className="button-icon">🔄</span>
          </button>
        </div>
      </div>

      {!isVisible && (
        <button 
          className="show-footer-button"
          onClick={() => setIsVisible(true)}
          title="Show system footer"
        >
          <span className="button-icon">📊</span>
        </button>
      )}
    </div>
  );
};

export default SystemFooter;
