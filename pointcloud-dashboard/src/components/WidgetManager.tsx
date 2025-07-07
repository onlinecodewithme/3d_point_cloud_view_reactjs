import React, { useState, useCallback } from 'react';
import DraggableWidget from './DraggableWidget';

interface Widget {
  id: string;
  title: string;
  icon: string;
  component: React.ComponentType<any>;
  props?: any;
  defaultPosition: { x: number; y: number };
  defaultSize: { width: number; height: number };
  minSize?: { width: number; height: number };
  maxSize?: { width: number; height: number };
  resizable?: boolean;
  collapsible?: boolean;
  visible: boolean;
}

interface WidgetManagerProps {
  widgets: Widget[];
  onWidgetToggle?: (widgetId: string, visible: boolean) => void;
}

const WidgetManager: React.FC<WidgetManagerProps> = ({ widgets, onWidgetToggle }) => {
  const [widgetStates, setWidgetStates] = useState<{ [key: string]: any }>({});

  const handlePositionChange = useCallback((id: string, position: { x: number; y: number }) => {
    setWidgetStates(prev => ({
      ...prev,
      [id]: { ...prev[id], position }
    }));
  }, []);

  const handleSizeChange = useCallback((id: string, size: { width: number; height: number }) => {
    setWidgetStates(prev => ({
      ...prev,
      [id]: { ...prev[id], size }
    }));
  }, []);

  const handleCollapse = useCallback((id: string, collapsed: boolean) => {
    setWidgetStates(prev => ({
      ...prev,
      [id]: { ...prev[id], collapsed }
    }));
  }, []);

  const toggleWidget = useCallback((widgetId: string) => {
    const widget = widgets.find(w => w.id === widgetId);
    if (widget) {
      onWidgetToggle?.(widgetId, !widget.visible);
    }
  }, [widgets, onWidgetToggle]);

  return (
    <>
      {/* Widget Toggle Panel */}
      <div className="widget-toggle-panel">
        <div className="widget-toggle-header">
          <span className="toggle-icon">🎛️</span>
          <span>Widgets</span>
        </div>
        <div className="widget-toggle-list">
          {widgets.map(widget => (
            <button
              key={widget.id}
              className={`widget-toggle-btn ${widget.visible ? 'active' : ''}`}
              onClick={() => toggleWidget(widget.id)}
              title={`Toggle ${widget.title}`}
            >
              <span className="widget-btn-icon">{widget.icon}</span>
              <span className="widget-btn-text">{widget.title}</span>
            </button>
          ))}
        </div>
        
        <div className="widget-actions">
          <button
            className="widget-action-btn"
            onClick={() => {
              // Reset all widgets to default positions
              widgets.forEach(widget => {
                if (widget.visible) {
                  handlePositionChange(widget.id, widget.defaultPosition);
                  handleSizeChange(widget.id, widget.defaultSize);
                }
              });
            }}
            title="Reset All Positions"
          >
            🏠 Reset All
          </button>
          <button
            className="widget-action-btn"
            onClick={() => {
              // Hide all widgets
              widgets.forEach(widget => {
                if (widget.visible) {
                  toggleWidget(widget.id);
                }
              });
            }}
            title="Hide All Widgets"
          >
            👁️ Hide All
          </button>
        </div>
      </div>

      {/* Render Visible Widgets */}
      {widgets.map(widget => {
        if (!widget.visible) return null;

        const WidgetComponent = widget.component;
        const state = widgetStates[widget.id] || {};

        return (
          <DraggableWidget
            key={widget.id}
            id={widget.id}
            title={widget.title}
            initialPosition={state.position || widget.defaultPosition}
            initialSize={state.size || widget.defaultSize}
            minSize={widget.minSize}
            maxSize={widget.maxSize}
            resizable={widget.resizable}
            collapsible={widget.collapsible}
            onPositionChange={handlePositionChange}
            onSizeChange={handleSizeChange}
            onCollapse={handleCollapse}
            zIndex={1000}
          >
            <WidgetComponent {...widget.props} />
          </DraggableWidget>
        );
      })}

      <style>{`
        .widget-toggle-panel {
          position: fixed;
          top: 90px;
          right: 20px;
          background: rgba(20, 20, 20, 0.95);
          border: 1px solid #444;
          border-radius: 8px;
          padding: 12px;
          backdrop-filter: blur(10px);
          box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
          z-index: 2000;
          min-width: 200px;
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }

        .widget-toggle-header {
          display: flex;
          align-items: center;
          gap: 8px;
          color: white;
          font-weight: bold;
          font-size: 14px;
          margin-bottom: 12px;
          padding-bottom: 8px;
          border-bottom: 1px solid #444;
        }

        .toggle-icon {
          font-size: 16px;
        }

        .widget-toggle-list {
          display: flex;
          flex-direction: column;
          gap: 6px;
          margin-bottom: 12px;
        }

        .widget-toggle-btn {
          display: flex;
          align-items: center;
          gap: 8px;
          padding: 8px 12px;
          background: rgba(255, 255, 255, 0.1);
          border: 1px solid #555;
          border-radius: 6px;
          color: white;
          cursor: pointer;
          transition: all 0.2s ease;
          font-size: 12px;
        }

        .widget-toggle-btn:hover {
          background: rgba(255, 255, 255, 0.2);
          border-color: #666;
        }

        .widget-toggle-btn.active {
          background: rgba(33, 150, 243, 0.3);
          border-color: #2196F3;
          color: #2196F3;
        }

        .widget-btn-icon {
          font-size: 14px;
          min-width: 16px;
        }

        .widget-btn-text {
          flex: 1;
          text-align: left;
        }

        .widget-actions {
          display: flex;
          flex-direction: column;
          gap: 6px;
          border-top: 1px solid #444;
          padding-top: 12px;
        }

        .widget-action-btn {
          padding: 6px 12px;
          background: rgba(255, 255, 255, 0.1);
          border: 1px solid #555;
          border-radius: 4px;
          color: white;
          cursor: pointer;
          transition: all 0.2s ease;
          font-size: 11px;
        }

        .widget-action-btn:hover {
          background: rgba(255, 255, 255, 0.2);
          border-color: #666;
        }
      `}</style>
    </>
  );
};

export default WidgetManager;
