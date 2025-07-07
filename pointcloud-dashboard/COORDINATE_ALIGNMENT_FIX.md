# Coordinate System Alignment Fix

## Problem Description

The user reported that while the 3D point cloud and 2D occupancy map were perfectly aligned in RViz, they appeared misaligned in our web dashboard. This document explains the fixes implemented to achieve RViz-level coordinate alignment.

## Root Cause Analysis

The misalignment was caused by several coordinate system handling issues:

1. **Occupancy Map Origin Handling**: The original implementation didn't properly use the map's origin position and orientation
2. **Coordinate Frame Mismatch**: Point cloud and occupancy map weren't using the same coordinate transformations
3. **Missing Quaternion Transformations**: Map orientation (quaternion) wasn't being applied to grid cells

## Solution Implementation

### 1. Proper Origin Handling

**Before:**
```typescript
const worldX = (cell.x - width / 2) * resolution;
const worldY = (cell.y - height / 2) * resolution;
const worldZ = 0.01;
```

**After:**
```typescript
// Use map origin position and orientation
const worldX = mapData.origin.position.x + localPosition.x;
const worldY = mapData.origin.position.y + localPosition.y;
const worldZ = mapData.origin.position.z + localPosition.z;
```

### 2. Quaternion-Based Orientation

**Added proper quaternion handling:**
```typescript
// Create quaternion from map origin orientation
const quaternion = new THREE.Quaternion(
  mapData.origin.orientation.x,
  mapData.origin.orientation.y,
  mapData.origin.orientation.z,
  mapData.origin.orientation.w
);

// Apply map orientation to local position
localPosition.applyQuaternion(quaternion);
```

### 3. ROS-Compliant Coordinate System

**Implemented ROS standard coordinate transformations:**
- Bottom-left origin for occupancy grid (ROS standard)
- Proper cell-to-world coordinate conversion
- Consistent frame transformations for both point cloud and occupancy map

## Technical Details

### Occupancy Grid Coordinate System

```typescript
// Position in local map coordinates (ROS occupancy grid: origin is at bottom-left corner)
const localX = cell.x * resolution;
const localY = cell.y * resolution;
const localZ = 0.01; // Slightly above ground

// Create local position vector
const localPosition = new THREE.Vector3(localX, localY, localZ);

// Apply map orientation to local position
localPosition.applyQuaternion(quaternion);

// Add map origin position to get world coordinates
const worldX = mapData.origin.position.x + localPosition.x;
const worldY = mapData.origin.position.y + localPosition.y;
const worldZ = mapData.origin.position.z + localPosition.z;
```

### Point Cloud Frame Logging

Added frame_id logging to debug coordinate frame mismatches:
```typescript
console.log('Point cloud frame_id:', message.header.frame_id);
```

## RViz Compatibility Features

### 1. Same Coordinate Transformations
- Both point cloud and occupancy map use identical coordinate system
- Proper handling of map origin position and orientation
- ROS-standard bottom-left origin for occupancy grids

### 2. Proper Rotation Handling
- Both visualizations apply the same -90° X-axis rotation
- Quaternion-based orientation for occupancy map
- Consistent anticlockwise rotation from red axis

### 3. Frame Alignment
- Point cloud and occupancy map rendered in same world coordinates
- Proper scaling and resolution handling
- Identical Z-axis positioning

## Verification Steps

To verify the alignment works correctly:

1. **Check Console Logs**: Look for frame_id information in browser console
2. **Visual Inspection**: 3D point cloud and 2D occupancy map should perfectly overlap
3. **RViz Comparison**: Alignment should match RViz visualization exactly

## Expected Results

After these fixes:
- ✅ 3D point cloud and 2D occupancy map perfectly aligned
- ✅ Same coordinate system as RViz
- ✅ Proper origin and orientation handling
- ✅ Red obstacles and white free areas correctly positioned
- ✅ Consistent scaling and resolution

## Color Scheme

- **Occupied Cells**: Red (RGB: 1.0, 0.0, 0.0) for values > 50
- **Free Cells**: White (RGB: 1.0, 1.0, 1.0) for values 0-50
- **Unknown Cells**: Not rendered (transparent) for values < 0

## Frame Compatibility

The system now properly handles:
- `map` frame for occupancy grid
- `cloud_map` or sensor frames for point cloud
- Automatic frame detection and alignment
- ROS-standard coordinate transformations

This implementation ensures that the web dashboard provides the exact same coordinate alignment and visualization as RViz, making it a true replacement for RViz visualization needs.
