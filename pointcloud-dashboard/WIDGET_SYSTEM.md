# Draggable Widget System

## Overview

The enhanced point cloud dashboard now features a flexible draggable widget system that allows users to position control panels anywhere on the screen. This provides a customizable and intuitive interface for managing 3D point cloud visualization, 2D occupancy mapping, and navigation controls.

## Features

### 🎯 Draggable Widgets
- **Drag & Drop**: Click and drag widget headers to reposition anywhere on screen
- **Resizable**: Drag the resize handle in the bottom-right corner to adjust size
- **Collapsible**: Click the collapse button to minimize widgets to header only
- **Persistent State**: Widget positions, sizes, and states are saved to browser localStorage

### 🎛️ Widget Manager
- **Toggle Panel**: Located in the top-right corner for easy access
- **Show/Hide Widgets**: Toggle individual widgets on/off
- **Reset Positions**: Restore all widgets to default positions
- **Hide All**: Quickly hide all widgets for unobstructed view

### 📊 Available Widgets

#### 1. Control Dashboard Widget
- **Icon**: 🎛️
- **Default Position**: Top-left (20, 80)
- **Default Size**: 400×600px
- **Features**:
  - 3D Mapping controls (enable/disable, save/load)
  - RTAB-Map SLAM configuration
  - Point cloud statistics
  - Voxel size adjustment
  - Processing status

#### 2. 2D Occupancy Map Widget
- **Icon**: 🗺️
- **Default Position**: Center-left (440, 80)
- **Default Size**: 350×400px
- **Features**:
  - Real-time 2D occupancy grid visualization
  - Interactive 3D map view
  - Map statistics and legend
  - Expandable view mode

#### 3. Navigation Control Widget
- **Icon**: 🧭
- **Default Position**: Top-right (screen width - 420, 80)
- **Default Size**: 400×700px
- **Features**:
  - Waypoint management (add, delete, save sets)
  - Navigation controls (start, stop, monitor)
  - Map management (save/load maps)
  - Real-time path visualization
  - Interactive 3D waypoint placement

## Usage Instructions

### Basic Widget Operations

1. **Moving Widgets**:
   - Click and hold the widget header (blue bar with title)
   - Drag to desired position
   - Release to place

2. **Resizing Widgets**:
   - Hover over bottom-right corner until resize cursor appears
   - Click and drag to adjust size
   - Minimum and maximum sizes are enforced

3. **Collapsing Widgets**:
   - Click the 🔽 button in widget header to collapse
   - Click 🔼 to expand again
   - Collapsed widgets show only the header

4. **Focusing Widgets**:
   - Click anywhere on a widget to bring it to front
   - Focused widgets have blue border and higher z-index

### Widget Manager Operations

1. **Toggle Widget Visibility**:
   - Open widget manager panel (top-right)
   - Click widget buttons to show/hide
   - Active widgets are highlighted in blue

2. **Reset All Positions**:
   - Click "🏠 Reset All" to restore default layout
   - All widgets return to original positions and sizes

3. **Hide All Widgets**:
   - Click "👁️ Hide All" for unobstructed 3D view
   - Use individual toggles to show specific widgets

### Keyboard Shortcuts

- **ESC**: Deselect focused widget
- **Click outside**: Remove focus from all widgets

## Technical Implementation

### Widget Architecture

```typescript
interface Widget {
  id: string;                    // Unique identifier
  title: string;                 // Display name
  icon: string;                  // Emoji icon
  component: React.ComponentType; // React component
  props: any;                    // Component props
  defaultPosition: {x, y};       // Initial position
  defaultSize: {width, height};  // Initial size
  minSize?: {width, height};     // Minimum dimensions
  maxSize?: {width, height};     // Maximum dimensions
  resizable: boolean;            // Can be resized
  collapsible: boolean;          // Can be collapsed
  visible: boolean;              // Currently visible
}
```

### State Management

- **Position & Size**: Stored in component state and localStorage
- **Visibility**: Managed by WidgetManager component
- **Focus**: Handled by individual DraggableWidget components
- **Persistence**: All states automatically saved to browser storage

### Performance Optimizations

- **Efficient Rendering**: Only visible widgets are rendered
- **Smooth Animations**: CSS transitions for collapse/expand
- **Memory Management**: Proper cleanup of event listeners
- **Boundary Checking**: Widgets constrained to viewport

## Customization

### Adding New Widgets

1. Create your React component
2. Add widget configuration to the widgets array:

```typescript
{
  id: 'my-widget',
  title: 'My Custom Widget',
  icon: '⚡',
  component: MyComponent,
  props: { /* component props */ },
  defaultPosition: { x: 100, y: 100 },
  defaultSize: { width: 300, height: 400 },
  resizable: true,
  collapsible: true,
  visible: true
}
```

### Styling Widgets

Widgets inherit the dark theme with:
- **Background**: `rgba(20, 20, 20, 0.95)`
- **Border**: `#444` with blue focus state
- **Header**: Blue gradient background
- **Content**: Scrollable with custom scrollbars

### Widget Constraints

- **Minimum Size**: 250×200px (configurable per widget)
- **Maximum Size**: 800×800px (configurable per widget)
- **Boundary**: Widgets cannot be dragged outside viewport
- **Z-Index**: Focused widgets appear above others

## Browser Compatibility

- **Chrome**: Full support with hardware acceleration
- **Firefox**: Full support
- **Safari**: Full support
- **Edge**: Full support
- **Mobile**: Touch-friendly drag operations

## Performance Considerations

- **Large Maps**: Widget rendering optimized for large occupancy grids
- **Real-time Updates**: Efficient prop updates without re-rendering
- **Memory Usage**: Automatic cleanup of unused widgets
- **GPU Acceleration**: WebGL-accelerated 3D rendering unaffected by widgets

## Troubleshooting

### Common Issues

1. **Widget Not Dragging**:
   - Ensure clicking on header area (blue bar)
   - Check for conflicting event handlers

2. **Position Not Saving**:
   - Verify localStorage is enabled
   - Check browser storage limits

3. **Performance Issues**:
   - Reduce number of visible widgets
   - Close unused widgets to free memory

4. **Resize Handle Not Working**:
   - Hover over bottom-right corner
   - Ensure widget is resizable (check configuration)

### Debug Mode

Enable debug logging by adding to browser console:
```javascript
localStorage.setItem('widget-debug', 'true');
```

## Future Enhancements

### Planned Features
- **Widget Snapping**: Snap to grid or other widgets
- **Widget Grouping**: Group related widgets together
- **Custom Layouts**: Save and load widget arrangements
- **Widget Docking**: Dock widgets to screen edges
- **Multi-Monitor**: Support for multiple displays

### Advanced Customization
- **Theme Support**: Light/dark theme switching
- **Widget Templates**: Pre-configured widget sets
- **Export/Import**: Share widget configurations
- **Widget Store**: Community-contributed widgets

## API Reference

### DraggableWidget Props
```typescript
interface DraggableWidgetProps {
  id: string;
  title: string;
  children: ReactNode;
  initialPosition: {x: number, y: number};
  initialSize?: {width: number, height: number};
  minSize?: {width: number, height: number};
  maxSize?: {width: number, height: number};
  resizable?: boolean;
  collapsible?: boolean;
  onPositionChange?: (id: string, position: {x, y}) => void;
  onSizeChange?: (id: string, size: {width, height}) => void;
  onCollapse?: (id: string, collapsed: boolean) => void;
  zIndex?: number;
}
```

### WidgetManager Props
```typescript
interface WidgetManagerProps {
  widgets: Widget[];
  onWidgetToggle?: (widgetId: string, visible: boolean) => void;
}
```

The draggable widget system transforms the static dashboard into a dynamic, user-customizable interface that adapts to individual workflow preferences while maintaining the powerful 3D visualization and navigation capabilities.
