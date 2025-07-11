import React, { useState, useRef, useEffect, ReactNode } from 'react';

interface DraggableWidgetProps {
  id: string;
  title: string;
  children: ReactNode;
  initialPosition: { x: number; y: number };
  initialSize?: { width: number; height: number };
  minSize?: { width: number; height: number };
  maxSize?: { width: number; height: number };
  resizable?: boolean;
  collapsible?: boolean;
  onPositionChange?: (id: string, position: { x: number; y: number }) => void;
  onSizeChange?: (id: string, size: { width: number; height: number }) => void;
  onCollapse?: (id: string, collapsed: boolean) => void;
  zIndex?: number;
}

const DraggableWidget: React.FC<DraggableWidgetProps> = ({
  id,
  title,
  children,
  initialPosition,
  initialSize = { width: 350, height: 400 },
  minSize = { width: 250, height: 200 },
  maxSize = { width: 800, height: 800 },
  resizable = true,
  collapsible = true,
  onPositionChange,
  onSizeChange,
  onCollapse,
  zIndex = 1000
}) => {
  const [position, setPosition] = useState(initialPosition);
  const [size, setSize] = useState(initialSize);
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });
  const [resizeStart, setResizeStart] = useState({ x: 0, y: 0, width: 0, height: 0 });
  const [isFocused, setIsFocused] = useState(false);

  const widgetRef = useRef<HTMLDivElement>(null);
  const headerRef = useRef<HTMLDivElement>(null);

  // Load saved position and size from localStorage
  useEffect(() => {
    const savedState = localStorage.getItem(`widget_${id}`);
    if (savedState) {
      const state = JSON.parse(savedState);
      setPosition(state.position || initialPosition);
      setSize(state.size || initialSize);
      setIsCollapsed(state.collapsed || false);
    }
  }, [id, initialPosition, initialSize]);

  // Save state to localStorage
  const saveState = () => {
    const state = {
      position,
      size,
      collapsed: isCollapsed
    };
    localStorage.setItem(`widget_${id}`, JSON.stringify(state));
  };

  useEffect(() => {
    saveState();
  }, [position, size, isCollapsed]);

  // Handle mouse down on header for dragging
  const handleMouseDown = (e: React.MouseEvent) => {
    if (e.target === headerRef.current || headerRef.current?.contains(e.target as Node)) {
      setIsDragging(true);
      setIsFocused(true);
      const rect = widgetRef.current?.getBoundingClientRect();
      if (rect) {
        setDragOffset({
          x: e.clientX - rect.left,
          y: e.clientY - rect.top
        });
      }
      e.preventDefault();
    }
  };

  // Handle mouse down on resize handle
  const handleResizeMouseDown = (e: React.MouseEvent) => {
    if (!resizable) return;
    setIsResizing(true);
    setIsFocused(true);
    setResizeStart({
      x: e.clientX,
      y: e.clientY,
      width: size.width,
      height: size.height
    });
    e.preventDefault();
    e.stopPropagation();
  };

  // Handle mouse move for dragging and resizing
  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (isDragging) {
        // Account for drawer width (320px) and header height (60px)
        const drawerWidth = 320;
        const headerHeight = 60;
        const footerHeight = 60;
        
        const newPosition = {
          x: Math.max(drawerWidth, Math.min(window.innerWidth - size.width, e.clientX - dragOffset.x)),
          y: Math.max(headerHeight, Math.min(window.innerHeight - footerHeight - 50, e.clientY - dragOffset.y))
        };
        setPosition(newPosition);
        onPositionChange?.(id, newPosition);
      } else if (isResizing) {
        const deltaX = e.clientX - resizeStart.x;
        const deltaY = e.clientY - resizeStart.y;
        
        const newSize = {
          width: Math.max(minSize.width, Math.min(maxSize.width, resizeStart.width + deltaX)),
          height: Math.max(minSize.height, Math.min(maxSize.height, resizeStart.height + deltaY))
        };
        
        setSize(newSize);
        onSizeChange?.(id, newSize);
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
      setIsResizing(false);
    };

    if (isDragging || isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      document.body.style.userSelect = 'none';
      document.body.style.cursor = isDragging ? 'grabbing' : 'nw-resize';
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
      document.body.style.userSelect = '';
      document.body.style.cursor = '';
    };
  }, [isDragging, isResizing, dragOffset, resizeStart, size, minSize, maxSize, id, onPositionChange, onSizeChange]);

  // Handle collapse toggle
  const handleCollapse = () => {
    const newCollapsed = !isCollapsed;
    setIsCollapsed(newCollapsed);
    onCollapse?.(id, newCollapsed);
  };

  // Handle widget focus
  const handleWidgetClick = () => {
    setIsFocused(true);
  };

  // Handle click outside to remove focus
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (widgetRef.current && !widgetRef.current.contains(e.target as Node)) {
        setIsFocused(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  return (
    <div
      ref={widgetRef}
      className={`draggable-widget ${isFocused ? 'focused' : ''} ${isCollapsed ? 'collapsed' : ''}`}
      style={{
        position: 'fixed',
        left: position.x,
        top: position.y,
        width: size.width,
        height: isCollapsed ? 'auto' : size.height,
        zIndex: isFocused ? zIndex + 1000 : zIndex,
        pointerEvents: 'auto'
      }}
      onClick={handleWidgetClick}
    >
      {/* Widget Header */}
      <div
        ref={headerRef}
        className="widget-header"
        onMouseDown={handleMouseDown}
        style={{ cursor: isDragging ? 'grabbing' : 'grab' }}
      >
        <div className="widget-title">
          <span className="widget-icon">📊</span>
          <span>{title}</span>
        </div>
        <div className="widget-controls">
          {collapsible && (
            <button
              className="widget-control-btn"
              onClick={handleCollapse}
              title={isCollapsed ? 'Expand' : 'Collapse'}
            >
              {isCollapsed ? '🔼' : '🔽'}
            </button>
          )}
          <button
            className="widget-control-btn"
            onClick={() => {
              // Reset to initial position
              setPosition(initialPosition);
              setSize(initialSize);
              onPositionChange?.(id, initialPosition);
              onSizeChange?.(id, initialSize);
            }}
            title="Reset Position"
          >
            🏠
          </button>
        </div>
      </div>

      {/* Widget Content */}
      {!isCollapsed && (
        <div className="widget-content">
          {children}
        </div>
      )}

      {/* Resize Handle */}
      {resizable && !isCollapsed && (
        <div
          className="resize-handle"
          onMouseDown={handleResizeMouseDown}
          style={{ cursor: 'nw-resize' }}
        />
      )}

      <style>{`
        .draggable-widget {
          background: rgba(20, 20, 20, 0.95);
          border: 1px solid #444;
          border-radius: 8px;
          box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
          backdrop-filter: blur(10px);
          transition: all 0.2s ease;
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
          overflow: hidden;
        }

        .draggable-widget.focused {
          border-color: #2196F3;
          box-shadow: 0 4px 20px rgba(33, 150, 243, 0.3);
        }

        .draggable-widget.collapsed {
          height: auto !important;
        }

        .widget-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 8px 12px;
          background: linear-gradient(135deg, #2196F3, #1976D2);
          color: white;
          border-radius: 8px 8px 0 0;
          user-select: none;
          min-height: 32px;
        }

        .widget-title {
          display: flex;
          align-items: center;
          gap: 8px;
          font-weight: bold;
          font-size: 14px;
        }

        .widget-icon {
          font-size: 16px;
        }

        .widget-controls {
          display: flex;
          gap: 4px;
        }

        .widget-control-btn {
          background: rgba(255, 255, 255, 0.2);
          border: none;
          border-radius: 4px;
          color: white;
          cursor: pointer;
          padding: 4px 6px;
          font-size: 12px;
          transition: background 0.2s ease;
        }

        .widget-control-btn:hover {
          background: rgba(255, 255, 255, 0.3);
        }

        .widget-content {
          padding: 0;
          height: calc(100% - 48px);
          overflow-y: auto;
          overflow-x: hidden;
        }

        .widget-content::-webkit-scrollbar {
          width: 6px;
        }

        .widget-content::-webkit-scrollbar-track {
          background: rgba(255, 255, 255, 0.1);
        }

        .widget-content::-webkit-scrollbar-thumb {
          background: rgba(255, 255, 255, 0.3);
          border-radius: 3px;
        }

        .widget-content::-webkit-scrollbar-thumb:hover {
          background: rgba(255, 255, 255, 0.5);
        }

        .resize-handle {
          position: absolute;
          bottom: 0;
          right: 0;
          width: 16px;
          height: 16px;
          background: linear-gradient(-45deg, transparent 30%, #666 30%, #666 70%, transparent 70%);
          cursor: nw-resize;
        }

        .resize-handle:hover {
          background: linear-gradient(-45deg, transparent 30%, #888 30%, #888 70%, transparent 70%);
        }

        /* Animation for collapse/expand */
        .draggable-widget {
          transition: height 0.3s ease;
        }
      `}</style>
    </div>
  );
};

export default DraggableWidget;
