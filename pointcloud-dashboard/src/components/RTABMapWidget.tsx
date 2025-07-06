import React from 'react';
import RTABMapControls from './RTABMapControls';

interface RTABMapWidgetProps {
  isRTABMapActive: boolean;
  onToggleRTABMap: (active: boolean) => void;
  onResetMap: () => void;
  onSaveMap: () => void;
  onLoadMap: () => void;
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

const RTABMapWidget: React.FC<RTABMapWidgetProps> = (props) => {
  return (
    <div style={{ padding: '16px', height: '100%' }}>
      <RTABMapControls {...props} />
    </div>
  );
};

export default RTABMapWidget;
