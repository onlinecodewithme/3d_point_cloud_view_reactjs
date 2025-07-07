import React from 'react';
import MappingControls from './MappingControls';

interface MappingWidgetProps {
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

const MappingWidget: React.FC<MappingWidgetProps> = (props) => {
  return (
    <div style={{ padding: '16px', height: '100%' }}>
      <MappingControls {...props} />
    </div>
  );
};

export default MappingWidget;
