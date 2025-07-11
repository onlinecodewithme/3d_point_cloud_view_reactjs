import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';

interface Waypoint {
  id: string;
  x: number;
  y: number;
  z?: number;
  name: string;
  timestamp: number;
}

interface NavigationPath {
  points: THREE.Vector3[];
  timestamp: number;
}

interface NavigationControlPanelProps {
  ros: any;
  isConnected: boolean;
  mapData: any;
  onWaypointAdded?: (waypoint: Waypoint) => void;
  onNavigationStarted?: (targetWaypoint: Waypoint) => void;
}

// Waypoint Renderer Component
const WaypointRenderer: React.FC<{ 
  waypoints: Waypoint[]; 
  selectedWaypoint: string | null;
  onWaypointClick: (waypoint: Waypoint) => void;
}> = ({ waypoints, selectedWaypoint, onWaypointClick }) => {
  const { camera, raycaster, mouse } = useThree();
  const waypointRefs = useRef<{ [key: string]: THREE.Mesh }>({});

  const handleClick = useCallback((event: any) => {
    event.stopPropagation();
    
    // Get canvas element for proper coordinate calculation
    const canvas = event.target.closest('canvas');
    if (!canvas) return;
    
    const rect = canvas.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    // Raycast to find clicked waypoint
    raycaster.setFromCamera(mouse, camera);
    const meshes = Object.values(waypointRefs.current);
    const intersects = raycaster.intersectObjects(meshes);

    if (intersects.length > 0) {
      const clickedMesh = intersects[0].object as THREE.Mesh;
      const waypoint = waypoints.find(w => waypointRefs.current[w.id] === clickedMesh);
      if (waypoint) {
        onWaypointClick(waypoint);
      }
    }
  }, [waypoints, onWaypointClick, camera, raycaster, mouse]);

  return (
    <group onClick={handleClick}>
      {waypoints.map((waypoint) => (
        <mesh
          key={waypoint.id}
          ref={(ref: THREE.Mesh) => {
            if (ref) waypointRefs.current[waypoint.id] = ref;
          }}
          position={[waypoint.x, waypoint.y, (waypoint.z || 0) + 0.5]}
        >
          <sphereGeometry args={[0.2, 16, 16]} />
          <meshLambertMaterial 
            color={new THREE.Color(selectedWaypoint === waypoint.id ? '#ff4444' : '#44ff44')}
            emissive={new THREE.Color(selectedWaypoint === waypoint.id ? '#440000' : '#004400')}
          />
          {/* Waypoint label */}
          <mesh position={[0, 0, 0.5]}>
            <planeGeometry args={[1, 0.3]} />
            <meshBasicMaterial color={new THREE.Color("#000000")} transparent opacity={0.7} />
          </mesh>
        </mesh>
      ))}
    </group>
  );
};

// Navigation Path Renderer
const NavigationPathRenderer: React.FC<{ path: NavigationPath | null }> = ({ path }) => {
  const meshRef = useRef<THREE.Mesh>(null);

  useEffect(() => {
    if (!path || !meshRef.current || path.points.length < 2) return;

    // Create a tube geometry for the path
    const curve = new THREE.CatmullRomCurve3(path.points);
    const geometry = new THREE.TubeGeometry(curve, path.points.length * 2, 0.05, 8, false);
    meshRef.current.geometry = geometry;
  }, [path]);

  if (!path || path.points.length < 2) return null;

  return (
    <mesh ref={meshRef}>
      <tubeGeometry args={[new THREE.CatmullRomCurve3(path.points), path.points.length * 2, 0.05, 8, false]} />
      <meshBasicMaterial color={new THREE.Color("#ffff00")} />
    </mesh>
  );
};

// Map Click Handler Component
const MapClickHandler: React.FC<{
  onMapClick: (x: number, y: number) => void;
  isWaypointMode: boolean;
}> = ({ onMapClick, isWaypointMode }) => {
  const { camera, raycaster, mouse } = useThree();

  const handleClick = useCallback((event: any) => {
    if (!isWaypointMode) return;

    event.stopPropagation();
    
    // Update mouse position
    const rect = event.target.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    // Raycast to find clicked position on the ground plane
    raycaster.setFromCamera(mouse, camera);
    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const intersectionPoint = new THREE.Vector3();
    
    if (raycaster.ray.intersectPlane(groundPlane, intersectionPoint)) {
      onMapClick(intersectionPoint.x, intersectionPoint.y);
    }
  }, [onMapClick, isWaypointMode, camera, raycaster, mouse]);

  return (
    <mesh onClick={handleClick} visible={false}>
      <planeGeometry args={[100, 100]} />
      <meshBasicMaterial transparent opacity={0} />
    </mesh>
  );
};

const NavigationControlPanel: React.FC<NavigationControlPanelProps> = ({
  ros,
  isConnected,
  mapData,
  onWaypointAdded,
  onNavigationStarted
}) => {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedWaypoint, setSelectedWaypoint] = useState<string | null>(null);
  const [isWaypointMode, setIsWaypointMode] = useState(false);
  const [navigationPath, setNavigationPath] = useState<NavigationPath | null>(null);
  const [isNavigating, setIsNavigating] = useState(false);
  const [savedMaps, setSavedMaps] = useState<string[]>([]);
  const [currentMapName, setCurrentMapName] = useState('');
  const [waypointSetName, setWaypointSetName] = useState('');
  const [savedWaypointSets, setSavedWaypointSets] = useState<string[]>([]);

  // ROS Publishers and Subscribers
  const moveBaseGoalPublisher = useRef<any>(null);
  const pathSubscriber = useRef<any>(null);
  const navigationStatusSubscriber = useRef<any>(null);

  // Initialize ROS publishers and subscribers
  useEffect(() => {
    if (!ros || !isConnected) return;

    try {
      // Move Base Goal Publisher for Nav2
      moveBaseGoalPublisher.current = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/move_base_simple/goal',
        messageType: 'geometry_msgs/PoseStamped'
      });

      // Navigation Path Subscriber
      pathSubscriber.current = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/plan',
        messageType: 'nav_msgs/Path'
      });

      pathSubscriber.current.subscribe((message: any) => {
        const pathPoints = message.poses.map((pose: any) => 
          new THREE.Vector3(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z || 0
          )
        );
        
        setNavigationPath({
          points: pathPoints,
          timestamp: Date.now()
        });
      });

      // Navigation Status Subscriber
      navigationStatusSubscriber.current = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/move_base/status',
        messageType: 'actionlib_msgs/GoalStatusArray'
      });

      navigationStatusSubscriber.current.subscribe((message: any) => {
        const hasActiveGoal = message.status_list.some((status: any) => 
          status.status === 1 // ACTIVE
        );
        setIsNavigating(hasActiveGoal);
      });

    } catch (error) {
      console.error('Failed to initialize navigation ROS topics:', error);
    }

    return () => {
      if (pathSubscriber.current) pathSubscriber.current.unsubscribe();
      if (navigationStatusSubscriber.current) navigationStatusSubscriber.current.unsubscribe();
    };
  }, [ros, isConnected]);

  // Load saved data from localStorage
  useEffect(() => {
    const savedMapsData = localStorage.getItem('savedMaps');
    if (savedMapsData) {
      setSavedMaps(JSON.parse(savedMapsData));
    }

    const savedWaypointsData = localStorage.getItem('savedWaypointSets');
    if (savedWaypointsData) {
      setSavedWaypointSets(JSON.parse(savedWaypointsData));
    }

    const currentWaypoints = localStorage.getItem('currentWaypoints');
    if (currentWaypoints) {
      setWaypoints(JSON.parse(currentWaypoints));
    }
  }, []);

  // Save waypoints to localStorage
  const saveWaypoints = useCallback(() => {
    localStorage.setItem('currentWaypoints', JSON.stringify(waypoints));
  }, [waypoints]);

  useEffect(() => {
    saveWaypoints();
  }, [waypoints, saveWaypoints]);

  // Handle map click for waypoint placement
  const handleMapClick = useCallback((x: number, y: number) => {
    if (!isWaypointMode) return;

    const newWaypoint: Waypoint = {
      id: `waypoint_${Date.now()}`,
      x,
      y,
      z: 0,
      name: `Waypoint ${waypoints.length + 1}`,
      timestamp: Date.now()
    };

    setWaypoints(prev => [...prev, newWaypoint]);
    onWaypointAdded?.(newWaypoint);
    setIsWaypointMode(false);
  }, [isWaypointMode, waypoints.length, onWaypointAdded]);

  // Handle waypoint click
  const handleWaypointClick = useCallback((waypoint: Waypoint) => {
    setSelectedWaypoint(waypoint.id);
  }, []);

  // Navigate to waypoint
  const navigateToWaypoint = useCallback((waypoint: Waypoint) => {
    if (!moveBaseGoalPublisher.current) return;

    const goal = {
      header: {
        frame_id: 'map',
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nanosec: (Date.now() % 1000) * 1000000
        }
      },
      pose: {
        position: {
          x: waypoint.x,
          y: waypoint.y,
          z: waypoint.z || 0
        },
        orientation: {
          x: 0,
          y: 0,
          z: 0,
          w: 1
        }
      }
    };

    moveBaseGoalPublisher.current.publish(goal);
    onNavigationStarted?.(waypoint);
    setIsNavigating(true);
  }, [onNavigationStarted]);

  // Save map
  const saveMap = useCallback(() => {
    if (!currentMapName.trim()) return;

    // Call ROS service to save map
    if (ros && isConnected) {
      const saveMapService = new (window as any).ROSLIB.Service({
        ros: ros,
        name: '/save_map',
        serviceType: 'nav2_msgs/SaveMap'
      });

      const request = new (window as any).ROSLIB.ServiceRequest({
        map_topic: '/map',
        map_url: `/tmp/${currentMapName}`,
        image_format: 'pgm',
        map_mode: 'trinary',
        free_thresh: 0.25,
        occupied_thresh: 0.65
      });

      saveMapService.callService(request, (result: any) => {
        if (result.result === 0) { // SUCCESS
          setSavedMaps(prev => {
            const updated = [...prev, currentMapName];
            localStorage.setItem('savedMaps', JSON.stringify(updated));
            return updated;
          });
          setCurrentMapName('');
          console.log('Map saved successfully');
        }
      });
    }
  }, [currentMapName, ros, isConnected]);

  // Load map
  const loadMap = useCallback((mapName: string) => {
    if (ros && isConnected) {
      // This would typically involve loading a saved map file
      // Implementation depends on your specific ROS setup
      console.log('Loading map:', mapName);
    }
  }, [ros, isConnected]);

  // Save waypoint set
  const saveWaypointSet = useCallback(() => {
    if (!waypointSetName.trim() || waypoints.length === 0) return;

    const waypointData = {
      name: waypointSetName,
      waypoints: waypoints,
      timestamp: Date.now()
    };

    localStorage.setItem(`waypoints_${waypointSetName}`, JSON.stringify(waypointData));
    setSavedWaypointSets(prev => {
      const updated = [...prev, waypointSetName];
      localStorage.setItem('savedWaypointSets', JSON.stringify(updated));
      return updated;
    });
    setWaypointSetName('');
  }, [waypointSetName, waypoints]);

  // Load waypoint set
  const loadWaypointSet = useCallback((setName: string) => {
    const waypointData = localStorage.getItem(`waypoints_${setName}`);
    if (waypointData) {
      const parsed = JSON.parse(waypointData);
      setWaypoints(parsed.waypoints);
    }
  }, []);

  // Clear waypoints
  const clearWaypoints = useCallback(() => {
    setWaypoints([]);
    setSelectedWaypoint(null);
  }, []);

  // Delete selected waypoint
  const deleteSelectedWaypoint = useCallback(() => {
    if (!selectedWaypoint) return;
    setWaypoints(prev => prev.filter(w => w.id !== selectedWaypoint));
    setSelectedWaypoint(null);
  }, [selectedWaypoint]);

  // Cancel navigation
  const cancelNavigation = useCallback(() => {
    if (ros && isConnected) {
      const cancelGoalPublisher = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/move_base/cancel',
        messageType: 'actionlib_msgs/GoalID'
      });

      cancelGoalPublisher.publish({});
      setIsNavigating(false);
    }
  }, [ros, isConnected]);

  return (
    <div className="navigation-control-panel">
      <div className="panel-header">
        <h3>🧭 Navigation Control Panel</h3>
        <div className="connection-status">
          <span className={`status-dot ${isConnected ? 'connected' : 'disconnected'}`}>●</span>
          <span>{isConnected ? 'Connected' : 'Disconnected'}</span>
        </div>
      </div>

      {/* Map Management */}
      <div className="control-section">
        <h4>🗺️ Map Management</h4>
        <div className="control-group">
          <div className="input-group">
            <input
              type="text"
              placeholder="Map name"
              value={currentMapName}
              onChange={(e) => setCurrentMapName(e.target.value)}
            />
            <button onClick={saveMap} disabled={!currentMapName.trim()}>
              💾 Save Map
            </button>
          </div>
          
          {savedMaps.length > 0 && (
            <div className="saved-items">
              <label>Saved Maps:</label>
              <select onChange={(e) => loadMap(e.target.value)} defaultValue="">
                <option value="">Select map to load</option>
                {savedMaps.map(mapName => (
                  <option key={mapName} value={mapName}>{mapName}</option>
                ))}
              </select>
            </div>
          )}
        </div>
      </div>

      {/* Waypoint Management */}
      <div className="control-section">
        <h4>📍 Waypoint Management</h4>
        <div className="control-group">
          <div className="button-row">
            <button 
              className={`waypoint-mode-btn ${isWaypointMode ? 'active' : ''}`}
              onClick={() => setIsWaypointMode(!isWaypointMode)}
            >
              {isWaypointMode ? '✋ Cancel' : '📍 Add Waypoint'}
            </button>
            <button onClick={clearWaypoints} disabled={waypoints.length === 0}>
              🗑️ Clear All
            </button>
            <button 
              onClick={deleteSelectedWaypoint} 
              disabled={!selectedWaypoint}
            >
              ❌ Delete Selected
            </button>
          </div>

          <div className="input-group">
            <input
              type="text"
              placeholder="Waypoint set name"
              value={waypointSetName}
              onChange={(e) => setWaypointSetName(e.target.value)}
            />
            <button 
              onClick={saveWaypointSet} 
              disabled={!waypointSetName.trim() || waypoints.length === 0}
            >
              💾 Save Waypoints
            </button>
          </div>

          {savedWaypointSets.length > 0 && (
            <div className="saved-items">
              <label>Saved Waypoint Sets:</label>
              <select onChange={(e) => loadWaypointSet(e.target.value)} defaultValue="">
                <option value="">Select waypoint set to load</option>
                {savedWaypointSets.map(setName => (
                  <option key={setName} value={setName}>{setName}</option>
                ))}
              </select>
            </div>
          )}
        </div>
      </div>

      {/* Navigation Controls */}
      <div className="control-section">
        <h4>🚀 Navigation Controls</h4>
        <div className="control-group">
          <div className="navigation-status">
            <span className={`nav-status ${isNavigating ? 'active' : 'idle'}`}>
              {isNavigating ? '🟢 Navigating' : '🔴 Idle'}
            </span>
            {isNavigating && (
              <button className="cancel-nav-btn" onClick={cancelNavigation}>
                ⏹️ Cancel Navigation
              </button>
            )}
          </div>

          {selectedWaypoint && (
            <div className="selected-waypoint">
              <span>Selected: {waypoints.find(w => w.id === selectedWaypoint)?.name}</span>
              <button 
                onClick={() => {
                  const waypoint = waypoints.find(w => w.id === selectedWaypoint);
                  if (waypoint) navigateToWaypoint(waypoint);
                }}
                disabled={isNavigating}
              >
                🎯 Navigate Here
              </button>
            </div>
          )}
        </div>
      </div>

      {/* Waypoint List */}
      {waypoints.length > 0 && (
        <div className="control-section">
          <h4>📋 Waypoints ({waypoints.length})</h4>
          <div className="waypoint-list">
            {waypoints.map((waypoint) => (
              <div 
                key={waypoint.id}
                className={`waypoint-item ${selectedWaypoint === waypoint.id ? 'selected' : ''}`}
                onClick={() => setSelectedWaypoint(waypoint.id)}
              >
                <span className="waypoint-name">{waypoint.name}</span>
                <span className="waypoint-coords">
                  ({waypoint.x.toFixed(2)}, {waypoint.y.toFixed(2)})
                </span>
                <button 
                  onClick={(e) => {
                    e.stopPropagation();
                    navigateToWaypoint(waypoint);
                  }}
                  disabled={isNavigating}
                  className="nav-btn"
                >
                  🎯
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Instructions */}
      <div className="control-section instructions">
        <h4>ℹ️ Instructions</h4>
        <ul>
          <li>Click "Add Waypoint" then click on the map to place waypoints</li>
          <li>Click on waypoints to select them</li>
          <li>Use "Navigate Here" to send the robot to selected waypoint</li>
          <li>Save waypoint sets for reuse</li>
          <li>Path planning avoids real-time obstacles automatically</li>
        </ul>
      </div>

      {/* 3D Visualization */}
      <div className="map-visualization">
        <Canvas
          camera={{ position: [0, 0, 10], fov: 60 }}
          style={{ height: '300px', background: '#f0f0f0' }}
        >
          <ambientLight intensity={0.6} />
          <directionalLight position={[10, 10, 5] as [number, number, number]} intensity={0.8} />
          
          <WaypointRenderer 
            waypoints={waypoints}
            selectedWaypoint={selectedWaypoint}
            onWaypointClick={handleWaypointClick}
          />
          
          <NavigationPathRenderer path={navigationPath} />
          
          <MapClickHandler 
            onMapClick={handleMapClick}
            isWaypointMode={isWaypointMode}
          />
          
          <OrbitControls 
            enablePan={true}
            enableZoom={true}
            enableRotate={true}
          />
          
          <gridHelper args={[20, 20]} />
          <axesHelper args={[2] as [number]} />
        </Canvas>
        
        {isWaypointMode && (
          <div className="waypoint-mode-overlay">
            <p>📍 Click on the map to place a waypoint</p>
          </div>
        )}
      </div>

      <style>{`
        .navigation-control-panel {
          background: rgba(30, 30, 30, 0.95);
          border: 1px solid #444;
          border-radius: 8px;
          padding: 16px;
          margin: 8px 0;
          color: white;
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
          max-height: 80vh;
          overflow-y: auto;
        }

        .panel-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 16px;
          border-bottom: 1px solid #444;
          padding-bottom: 8px;
        }

        .panel-header h3 {
          margin: 0;
          color: #4CAF50;
        }

        .connection-status {
          display: none;
          align-items: center;
          gap: 8px;
          font-size: 12px;
        }

        .status-dot {
          width: 8px;
          height: 8px;
          border-radius: 50%;
        }

        .status-dot.connected {
          background: #4CAF50;
        }

        .status-dot.disconnected {
          background: #f44336;
        }

        .control-section {
          margin-bottom: 16px;
          border: 1px solid #555;
          border-radius: 6px;
          padding: 12px;
          background: rgba(0, 0, 0, 0.3);
        }

        .control-section h4 {
          margin: 0 0 12px 0;
          color: #2196F3;
          font-size: 14px;
        }

        .control-group {
          display: flex;
          flex-direction: column;
          gap: 8px;
        }

        .input-group {
          display: flex;
          gap: 8px;
        }

        .input-group input {
          flex: 1;
          padding: 6px 8px;
          background: #333;
          border: 1px solid #555;
          border-radius: 4px;
          color: white;
          font-size: 12px;
        }

        .input-group button {
          padding: 6px 12px;
          background: #2196F3;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          font-size: 12px;
          font-weight: bold;
        }

        .input-group button:disabled {
          background: #666;
          cursor: not-allowed;
        }

        .button-row {
          display: flex;
          gap: 8px;
          flex-wrap: wrap;
        }

        .button-row button {
          padding: 6px 12px;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          font-size: 12px;
          font-weight: bold;
        }

        .waypoint-mode-btn {
          background: #4CAF50;
          color: white;
        }

        .waypoint-mode-btn.active {
          background: #f44336;
        }

        .saved-items {
          display: flex;
          flex-direction: column;
          gap: 4px;
        }

        .saved-items label {
          font-size: 12px;
          color: #ccc;
        }

        .saved-items select {
          padding: 6px;
          background: #333;
          border: 1px solid #555;
          border-radius: 4px;
          color: white;
          font-size: 12px;
        }

        .navigation-status {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 8px;
          background: rgba(0, 0, 0, 0.5);
          border-radius: 4px;
        }

        .nav-status {
          font-weight: bold;
          font-size: 12px;
        }

        .nav-status.active {
          color: #4CAF50;
        }

        .nav-status.idle {
          color: #999;
        }

        .cancel-nav-btn {
          background: #f44336;
          color: white;
          border: none;
          padding: 4px 8px;
          border-radius: 4px;
          cursor: pointer;
          font-size: 11px;
        }

        .selected-waypoint {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 8px;
          background: rgba(33, 150, 243, 0.2);
          border-radius: 4px;
          font-size: 12px;
        }

        .selected-waypoint button {
          background: #FF9800;
          color: white;
          border: none;
          padding: 4px 8px;
          border-radius: 4px;
          cursor: pointer;
          font-size: 11px;
        }

        .waypoint-list {
          max-height: 150px;
          overflow-y: auto;
          border: 1px solid #555;
          border-radius: 4px;
        }

        .waypoint-item {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 8px;
          border-bottom: 1px solid #555;
          cursor: pointer;
          font-size: 12px;
        }

        .waypoint-item:hover {
          background: rgba(255, 255, 255, 0.1);
        }

        .waypoint-item.selected {
          background: rgba(33, 150, 243, 0.3);
        }

        .waypoint-name {
          font-weight: bold;
        }

        .waypoint-coords {
          color: #999;
          font-size: 11px;
        }

        .nav-btn {
          background: #FF9800;
          color: white;
          border: none;
          padding: 2px 6px;
          border-radius: 3px;
          cursor: pointer;
          font-size: 10px;
        }

        .nav-btn:disabled {
          background: #666;
          cursor: not-allowed;
        }

        .instructions {
          background: rgba(76, 175, 80, 0.1);
          border-color: #4CAF50;
        }

        .instructions ul {
          margin: 0;
          padding-left: 16px;
          font-size: 11px;
          color: #ccc;
        }

        .instructions li {
          margin-bottom: 4px;
        }

        .map-visualization {
          position: relative;
          border: 1px solid #555;
          border-radius: 4px;
          overflow: hidden;
        }

        .waypoint-mode-overlay {
          position: absolute;
          top: 10px;
          left: 10px;
          right: 10px;
          background: rgba(76, 175, 80, 0.9);
          color: white;
          padding: 8px;
          border-radius: 4px;
          text-align: center;
          font-size: 12px;
          font-weight: bold;
          z-index: 1000;
        }
      `}</style>
    </div>
  );
};

export default NavigationControlPanel;
