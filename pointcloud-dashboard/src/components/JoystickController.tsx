import React, { useState, useRef, useEffect, useCallback } from 'react';

interface JoystickControllerProps {
  ros: any;
  isConnected: boolean;
}

interface JoystickState {
  x: number;
  y: number;
  active: boolean;
}

const JoystickController: React.FC<JoystickControllerProps> = ({
  ros,
  isConnected
}) => {
  const [linearJoystick, setLinearJoystick] = useState<JoystickState>({ x: 0, y: 0, active: false });
  const [angularJoystick, setAngularJoystick] = useState<JoystickState>({ x: 0, y: 0, active: false });
  const [maxLinearSpeed, setMaxLinearSpeed] = useState(1.0);
  const [maxAngularSpeed, setMaxAngularSpeed] = useState(1.0);
  const [publishRate, setPublishRate] = useState(10); // Hz
  
  const cmdVelTopicRef = useRef<any>(null);
  const publishIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const linearJoystickRef = useRef<HTMLDivElement>(null);
  const angularJoystickRef = useRef<HTMLDivElement>(null);

  // Initialize cmd_vel publisher
  useEffect(() => {
    if (ros && isConnected) {
      cmdVelTopicRef.current = new (window as any).ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
      });
    }
  }, [ros, isConnected]);

  // Publish velocity commands at regular intervals
  useEffect(() => {
    if (publishIntervalRef.current) {
      clearInterval(publishIntervalRef.current);
    }

    if (cmdVelTopicRef.current && (linearJoystick.active || angularJoystick.active)) {
      publishIntervalRef.current = setInterval(() => {
        publishVelocityCommand();
      }, 1000 / publishRate);
    }

    return () => {
      if (publishIntervalRef.current) {
        clearInterval(publishIntervalRef.current);
      }
    };
  }, [linearJoystick, angularJoystick, maxLinearSpeed, maxAngularSpeed, publishRate]);

  const publishVelocityCommand = useCallback(() => {
    if (!cmdVelTopicRef.current) return;

    // Calculate linear velocities (forward/backward, left/right strafe)
    const linearX = linearJoystick.active ? (linearJoystick.y * maxLinearSpeed) : 0;
    const linearY = linearJoystick.active ? (-linearJoystick.x * maxLinearSpeed) : 0;
    
    // Calculate angular velocity (rotation)
    const angularZ = angularJoystick.active ? (-angularJoystick.x * maxAngularSpeed) : 0;

    const twistMessage = new (window as any).ROSLIB.Message({
      linear: {
        x: linearX,
        y: linearY,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angularZ
      }
    });

    cmdVelTopicRef.current.publish(twistMessage);
  }, [linearJoystick, angularJoystick, maxLinearSpeed, maxAngularSpeed]);

  const handleJoystickMove = (
    event: React.MouseEvent | React.TouchEvent,
    joystickType: 'linear' | 'angular',
    joystickElement: HTMLDivElement
  ) => {
    const rect = joystickElement.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;
    const radius = Math.min(centerX, centerY) - 20; // Account for knob size

    let clientX: number, clientY: number;
    
    if ('touches' in event) {
      if (event.touches.length === 0) return;
      clientX = event.touches[0].clientX;
      clientY = event.touches[0].clientY;
    } else {
      clientX = event.clientX;
      clientY = event.clientY;
    }

    const x = clientX - rect.left - centerX;
    const y = clientY - rect.top - centerY;
    
    const distance = Math.sqrt(x * x + y * y);
    const normalizedX = distance > radius ? (x / distance) * radius : x;
    const normalizedY = distance > radius ? (y / distance) * radius : y;
    
    const joystickX = normalizedX / radius;
    const joystickY = normalizedY / radius;

    if (joystickType === 'linear') {
      setLinearJoystick({ x: joystickX, y: joystickY, active: true });
    } else {
      setAngularJoystick({ x: joystickX, y: joystickY, active: true });
    }
  };

  const handleJoystickEnd = (joystickType: 'linear' | 'angular') => {
    if (joystickType === 'linear') {
      setLinearJoystick({ x: 0, y: 0, active: false });
    } else {
      setAngularJoystick({ x: 0, y: 0, active: false });
    }
  };

  const emergencyStop = () => {
    setLinearJoystick({ x: 0, y: 0, active: false });
    setAngularJoystick({ x: 0, y: 0, active: false });
    
    if (cmdVelTopicRef.current) {
      const stopMessage = new (window as any).ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      });
      cmdVelTopicRef.current.publish(stopMessage);
    }
  };

  return (
    <div className="joystick-controller">
      <div className="joystick-header">
        <h3>🎮 Robot Control</h3>
        <div className="connection-status">
          <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
            ● {isConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </div>

      {/* Control Settings */}
      <div className="control-settings">
        <div className="setting-group">
          <label>Max Linear Speed: {maxLinearSpeed.toFixed(1)} m/s</label>
          <input
            type="range"
            min="0.1"
            max="3.0"
            step="0.1"
            value={maxLinearSpeed}
            onChange={(e) => setMaxLinearSpeed(parseFloat(e.target.value))}
          />
        </div>
        <div className="setting-group">
          <label>Max Angular Speed: {maxAngularSpeed.toFixed(1)} rad/s</label>
          <input
            type="range"
            min="0.1"
            max="3.0"
            step="0.1"
            value={maxAngularSpeed}
            onChange={(e) => setMaxAngularSpeed(parseFloat(e.target.value))}
          />
        </div>
      </div>

      {/* Linear Movement Joystick */}
      <div className="joystick-section">
        <h4>Linear Movement</h4>
        <div
          ref={linearJoystickRef}
          className="joystick-container"
          onMouseDown={(e) => {
            if (linearJoystickRef.current) {
              handleJoystickMove(e, 'linear', linearJoystickRef.current);
            }
          }}
          onMouseMove={(e) => {
            if (linearJoystick.active && linearJoystickRef.current) {
              handleJoystickMove(e, 'linear', linearJoystickRef.current);
            }
          }}
          onMouseUp={() => handleJoystickEnd('linear')}
          onMouseLeave={() => handleJoystickEnd('linear')}
          onTouchStart={(e) => {
            if (linearJoystickRef.current) {
              handleJoystickMove(e, 'linear', linearJoystickRef.current);
            }
          }}
          onTouchMove={(e) => {
            if (linearJoystick.active && linearJoystickRef.current) {
              handleJoystickMove(e, 'linear', linearJoystickRef.current);
            }
          }}
          onTouchEnd={() => handleJoystickEnd('linear')}
        >
          <div className="joystick-base">
            <div className="joystick-directions">
              <span className="direction-label top">Forward</span>
              <span className="direction-label bottom">Backward</span>
              <span className="direction-label left">Left</span>
              <span className="direction-label right">Right</span>
            </div>
            <div
              className="joystick-knob"
              style={{
                transform: `translate(${linearJoystick.x * 60}px, ${linearJoystick.y * 60}px)`,
                backgroundColor: linearJoystick.active ? '#00ff00' : '#ffffff'
              }}
            />
          </div>
        </div>
        <div className="joystick-values">
          <span>X: {linearJoystick.x.toFixed(2)}</span>
          <span>Y: {linearJoystick.y.toFixed(2)}</span>
        </div>
      </div>

      {/* Angular Movement Joystick */}
      <div className="joystick-section">
        <h4>Angular Movement</h4>
        <div
          ref={angularJoystickRef}
          className="joystick-container"
          onMouseDown={(e) => {
            if (angularJoystickRef.current) {
              handleJoystickMove(e, 'angular', angularJoystickRef.current);
            }
          }}
          onMouseMove={(e) => {
            if (angularJoystick.active && angularJoystickRef.current) {
              handleJoystickMove(e, 'angular', angularJoystickRef.current);
            }
          }}
          onMouseUp={() => handleJoystickEnd('angular')}
          onMouseLeave={() => handleJoystickEnd('angular')}
          onTouchStart={(e) => {
            if (angularJoystickRef.current) {
              handleJoystickMove(e, 'angular', angularJoystickRef.current);
            }
          }}
          onTouchMove={(e) => {
            if (angularJoystick.active && angularJoystickRef.current) {
              handleJoystickMove(e, 'angular', angularJoystickRef.current);
            }
          }}
          onTouchEnd={() => handleJoystickEnd('angular')}
        >
          <div className="joystick-base">
            <div className="joystick-directions">
              <span className="direction-label left">↺ CCW</span>
              <span className="direction-label right">CW ↻</span>
            </div>
            <div
              className="joystick-knob"
              style={{
                transform: `translate(${angularJoystick.x * 60}px, ${angularJoystick.y * 60}px)`,
                backgroundColor: angularJoystick.active ? '#ff6600' : '#ffffff'
              }}
            />
          </div>
        </div>
        <div className="joystick-values">
          <span>Rotation: {angularJoystick.x.toFixed(2)}</span>
        </div>
      </div>

      {/* Control Buttons */}
      <div className="control-buttons">
        <button
          className="stop-btn"
          onClick={emergencyStop}
          disabled={!isConnected}
        >
          ⏹️ STOP
        </button>
        <button
          className="emergency-stop-btn"
          onClick={emergencyStop}
          disabled={!isConnected}
        >
          🛑 E-STOP
        </button>
      </div>

      {/* Velocity Display */}
      <div className="velocity-display">
        <h4>Current Velocities</h4>
        <div className="velocity-values">
          <div>Linear X: {(linearJoystick.y * maxLinearSpeed).toFixed(2)} m/s</div>
          <div>Linear Y: {(-linearJoystick.x * maxLinearSpeed).toFixed(2)} m/s</div>
          <div>Angular Z: {(-angularJoystick.x * maxAngularSpeed).toFixed(2)} rad/s</div>
        </div>
      </div>
    </div>
  );
};

export default JoystickController;
