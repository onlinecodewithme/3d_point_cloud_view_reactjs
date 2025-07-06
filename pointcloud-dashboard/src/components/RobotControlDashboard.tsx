import React, { useState, useEffect, useRef } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stats } from '@react-three/drei';
import CameraFeedViewer from './CameraFeedViewer';
import JoystickController from './JoystickController';
import OccupancyMapViewer from './OccupancyMapViewer';
import RobotStatusOverlay from './RobotStatusOverlay';
import './RobotControlDashboard.css';

interface RobotControlDashboardProps {}

const RobotControlDashboard: React.FC<RobotControlDashboardProps> = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Connecting...');
  const [cameraFeedActive, setCameraFeedActive] = useState(false);
  const [mapExpanded, setMapExpanded] = useState(false);
  const rosRef = useRef<any>(null);

  // Initialize ROS connection
  useEffect(() => {
    const initROS = async () => {
      try {
        // @ts-ignore
        const ROSLIB = window.ROSLIB;
        if (!ROSLIB) {
          setConnectionStatus('ROS library not loaded');
          return;
        }

        const ros = new ROSLIB.Ros({
          url: 'ws://localhost:9090'
        });

        rosRef.current = ros;

        ros.on('connection', () => {
          console.log('Connected to ROS bridge');
          setIsConnected(true);
          setConnectionStatus('Connected to ROS');
        });

        ros.on('error', (error: any) => {
          console.error('ROS connection error:', error);
          setIsConnected(false);
          setConnectionStatus(`Connection error: ${error.message || 'Unknown error'}`);
        });

        ros.on('close', () => {
          console.log('ROS connection closed');
          setIsConnected(false);
          setConnectionStatus('Disconnected from ROS');
        });

      } catch (error) {
        console.error('Failed to initialize ROS:', error);
        setConnectionStatus('Failed to initialize ROS');
      }
    };

    initROS();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  return (
    <div className="robot-control-dashboard">
      {/* Main Camera Feed Background */}
      <div className="camera-feed-container">
        <CameraFeedViewer 
          ros={rosRef.current}
          isConnected={isConnected}
          onFeedStatusChange={setCameraFeedActive}
        />
      </div>

      {/* Status Overlay - Top Left */}
      <div className="status-overlay">
        <RobotStatusOverlay 
          connectionStatus={connectionStatus}
          isConnected={isConnected}
          cameraFeedActive={cameraFeedActive}
        />
      </div>

      {/* Joystick Controller - Right Side */}
      <div className="joystick-overlay">
        <JoystickController 
          ros={rosRef.current}
          isConnected={isConnected}
        />
      </div>

      {/* Occupancy Map - Bottom Left */}
      <div className={`map-overlay ${mapExpanded ? 'expanded' : ''}`}>
        <OccupancyMapViewer 
          ros={rosRef.current}
          isConnected={isConnected}
          isExpanded={mapExpanded}
          onExpandToggle={() => setMapExpanded(!mapExpanded)}
        />
      </div>

    </div>
  );
};

export default RobotControlDashboard;
