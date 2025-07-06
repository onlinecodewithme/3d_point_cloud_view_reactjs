import React, { useEffect, useRef, useState } from 'react';

interface CameraFeedViewerProps {
  ros: any;
  isConnected: boolean;
  onFeedStatusChange: (active: boolean) => void;
}

const CameraFeedViewer: React.FC<CameraFeedViewerProps> = ({
  ros,
  isConnected,
  onFeedStatusChange
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement>(null);
  const [feedActive, setFeedActive] = useState(false);
  const [lastFrameTime, setLastFrameTime] = useState<number>(0);
  const [streamMethod, setStreamMethod] = useState<'web_video_server' | 'rosbridge'>('web_video_server');
  const topicRef = useRef<any>(null);

  useEffect(() => {
    if (!isConnected) {
      setFeedActive(false);
      onFeedStatusChange(false);
      return;
    }

    console.log(`Setting up camera feed via ${streamMethod}...`);

    let cleanup: (() => void) | undefined;

    if (streamMethod === 'web_video_server') {
      cleanup = setupWebVideoServerStream();
    } else if (ros) {
      setupROSBridgeStream();
    }

    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
      if (cleanup) {
        cleanup();
      }
    };
  }, [ros, isConnected, streamMethod, onFeedStatusChange]);

  const setupWebVideoServerStream = () => {
    console.log('Setting up Web Video Server stream...');
    
    const img = imgRef.current;
    if (!img) {
      console.error('Image element not available');
      return;
    }

    // Web Video Server stream URL - stable URL without cache busting to avoid reconnections
    const streamUrl = 'http://localhost:8080/stream?topic=/zed/zed_node/rgb/image_rect_color&type=mjpeg';
    
    console.log('Connecting to Web Video Server:', streamUrl);

    // Set up image handlers
    img.onload = () => {
      console.log('✅ Web Video Server stream connected successfully');
      console.log('Setting feedActive to true...');
      setFeedActive(true);
      onFeedStatusChange(true);
      setLastFrameTime(Date.now());
    };

    // For MJPEG streams, we need to continuously update the frame time
    // since onload only fires once
    const frameUpdateInterval = setInterval(() => {
      if (img.complete && img.naturalWidth > 0) {
        setLastFrameTime(Date.now());
      }
    }, 1000); // Update every second

    img.onerror = (error) => {
      console.error('❌ Web Video Server stream failed:', error);
      console.log('Setting feedActive to false due to error...');
      setFeedActive(false);
      onFeedStatusChange(false);
      
      // Only fallback if we haven't already tried ROSBridge
      if (streamMethod === 'web_video_server') {
        console.log('Falling back to ROSBridge method...');
        setTimeout(() => {
          setStreamMethod('rosbridge');
        }, 1000);
      }
    };

    // Set the source to start the stream
    img.src = streamUrl;
    
    // Monitor stream health - check if image is still loading
    const healthCheckInterval = setInterval(() => {
      if (img.complete && img.naturalWidth === 0) {
        console.warn('⚠️ Web Video Server stream appears broken, refreshing...');
        // Force reload by adding timestamp
        img.src = `${streamUrl}&_t=${Date.now()}`;
      }
    }, 10000); // Check every 10 seconds

    return () => {
      clearInterval(healthCheckInterval);
      clearInterval(frameUpdateInterval);
      img.onload = null;
      img.onerror = null;
    };
  };

  const setupROSBridgeStream = () => {
    if (!ros) {
      console.error('ROS not available for ROSBridge stream');
      return;
    }

    console.log('Setting up ROSBridge stream...');
    console.log('ROS object:', ros);
    console.log('ROS connection status:', ros.isConnected);

    try {
      // First, let's check what topics are available
      console.log('Checking available topics...');
      
      // Get list of topics
      const topicsClient = new (window as any).ROSLIB.Service({
        ros: ros,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics'
      });

      const request = new (window as any).ROSLIB.ServiceRequest({});
      
      topicsClient.callService(request, (result: any) => {
        console.log('Available topics:', result.topics);
        const hasZedTopic = result.topics && result.topics.includes('/zed/zed_node/rgb/image_rect_color');
        console.log('ZED camera topic available:', hasZedTopic);
        
        if (hasZedTopic) {
          subscribeToCamera();
        } else {
          console.error('ZED camera topic not found in available topics');
          console.log('Available topics:', result.topics);
        }
      }, (error: any) => {
        console.error('Failed to get topics list:', error);
        console.log('Trying direct subscription anyway...');
        subscribeToCamera();
      });

      const subscribeToCamera = () => {
        console.log('Subscribing to camera topic...');
        
        // Direct subscription to the known working topic
        const imageTopic = new (window as any).ROSLIB.Topic({
          ros: ros,
          name: '/zed/zed_node/rgb/image_rect_color',
          messageType: 'sensor_msgs/Image'
        });

        topicRef.current = imageTopic;

        imageTopic.subscribe((message: any) => {
          console.log('🎉 Received camera message via ROSBridge!');
          console.log('Message details:', {
            width: message.width,
            height: message.height,
            encoding: message.encoding,
            dataType: typeof message.data,
            dataLength: message.data ? message.data.length : 'undefined',
            timestamp: new Date().toLocaleTimeString()
          });
          
          try {
            drawImageToCanvas(message);
            setFeedActive(true);
            onFeedStatusChange(true);
            setLastFrameTime(Date.now());
          } catch (error) {
            console.error('Error processing camera image:', error);
            setFeedActive(false);
            onFeedStatusChange(false);
          }
        });

        console.log('✅ Subscribed to ZED camera feed via ROSBridge: /zed/zed_node/rgb/image_rect_color');
        
        // Set a timeout to check if we receive any messages
        setTimeout(() => {
          if (!feedActive) {
            console.warn('⚠️ No camera messages received after 10 seconds');
            console.log('This could mean:');
            console.log('1. ZED camera is not publishing');
            console.log('2. Topic name is incorrect');
            console.log('3. ROSBridge is not forwarding the topic');
            console.log('4. Camera node is not running');
          }
        }, 10000);
      };

    } catch (error) {
      console.error('Failed to subscribe to camera feed via ROSBridge:', error);
      setFeedActive(false);
      onFeedStatusChange(false);
    }
  };

  const drawImageToCanvas = (imageMessage: any) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    try {
      const { width, height, encoding, data } = imageMessage;
      
      console.log('Received image:', { width, height, encoding, dataLength: data.length });
      
      // Set canvas dimensions
      canvas.width = width;
      canvas.height = height;

      // Handle different data formats
      let bytes: Uint8Array;
      
      if (typeof data === 'string') {
        // Base64 encoded data
        try {
          const binaryData = atob(data);
          bytes = new Uint8Array(binaryData.length);
          for (let i = 0; i < binaryData.length; i++) {
            bytes[i] = binaryData.charCodeAt(i);
          }
        } catch (e) {
          console.error('Failed to decode base64 data:', e);
          return;
        }
      } else if (Array.isArray(data)) {
        // Array of bytes
        bytes = new Uint8Array(data);
      } else if (data instanceof Uint8Array) {
        // Already Uint8Array
        bytes = data;
      } else {
        console.error('Unknown data format:', typeof data);
        return;
      }

      console.log('Processed bytes length:', bytes.length);

      // Create ImageData object
      const imageData = ctx.createImageData(width, height);
      
      // Process based on encoding
      if (encoding === 'rgb8') {
        // RGB8 format: 3 bytes per pixel (R, G, B)
        console.log('Processing RGB8 format');
        for (let i = 0; i < width * height; i++) {
          const srcIndex = i * 3;
          const dstIndex = i * 4;
          
          if (srcIndex + 2 < bytes.length) {
            imageData.data[dstIndex] = bytes[srcIndex];     // R
            imageData.data[dstIndex + 1] = bytes[srcIndex + 1]; // G
            imageData.data[dstIndex + 2] = bytes[srcIndex + 2]; // B
            imageData.data[dstIndex + 3] = 255; // A (alpha)
          }
        }
      } else if (encoding === 'bgr8') {
        // BGR8 format: 3 bytes per pixel (B, G, R)
        console.log('Processing BGR8 format');
        for (let i = 0; i < width * height; i++) {
          const srcIndex = i * 3;
          const dstIndex = i * 4;
          
          if (srcIndex + 2 < bytes.length) {
            imageData.data[dstIndex] = bytes[srcIndex + 2];     // R (from B)
            imageData.data[dstIndex + 1] = bytes[srcIndex + 1]; // G
            imageData.data[dstIndex + 2] = bytes[srcIndex];     // B (from R)
            imageData.data[dstIndex + 3] = 255; // A (alpha)
          }
        }
      } else if (encoding === 'rgba8') {
        // RGBA8 format: 4 bytes per pixel
        console.log('Processing RGBA8 format');
        for (let i = 0; i < bytes.length && i < imageData.data.length; i++) {
          imageData.data[i] = bytes[i];
        }
      } else if (encoding === 'mono8') {
        // Grayscale format: 1 byte per pixel
        console.log('Processing MONO8 format');
        for (let i = 0; i < width * height; i++) {
          const srcIndex = i;
          const dstIndex = i * 4;
          
          if (srcIndex < bytes.length) {
            const gray = bytes[srcIndex];
            imageData.data[dstIndex] = gray;     // R
            imageData.data[dstIndex + 1] = gray; // G
            imageData.data[dstIndex + 2] = gray; // B
            imageData.data[dstIndex + 3] = 255;  // A (alpha)
          }
        }
      } else {
        console.warn(`Unsupported image encoding: ${encoding}`);
        // Try to display as RGB8 anyway
        console.log('Attempting to display as RGB8...');
        for (let i = 0; i < width * height && i * 3 + 2 < bytes.length; i++) {
          const srcIndex = i * 3;
          const dstIndex = i * 4;
          
          imageData.data[dstIndex] = bytes[srcIndex];     // R
          imageData.data[dstIndex + 1] = bytes[srcIndex + 1]; // G
          imageData.data[dstIndex + 2] = bytes[srcIndex + 2]; // B
          imageData.data[dstIndex + 3] = 255; // A (alpha)
        }
      }

      // Draw the image data to canvas
      ctx.putImageData(imageData, 0, 0);
      console.log('Image drawn to canvas successfully');
      
    } catch (error) {
      console.error('Error drawing image to canvas:', error);
    }
  };

  // Check for feed timeout
  useEffect(() => {
    const checkFeedTimeout = setInterval(() => {
      if (feedActive && Date.now() - lastFrameTime > 5000) {
        console.warn('🚨 TIMEOUT: Setting feedActive to false due to timeout');
        console.log('Current time:', Date.now());
        console.log('Last frame time:', lastFrameTime);
        console.log('Time difference:', Date.now() - lastFrameTime);
        setFeedActive(false);
        onFeedStatusChange(false);
      }
    }, 1000);

    return () => clearInterval(checkFeedTimeout);
  }, [feedActive, lastFrameTime, onFeedStatusChange]);

  return (
    <div className="camera-feed-viewer">
      {/* Web Video Server Image Stream */}
      {streamMethod === 'web_video_server' && (
        <img
          ref={imgRef}
          className="camera-image"
          style={{
            width: '100%',
            height: '100%',
            objectFit: 'cover',
            backgroundColor: '#000',
            display: feedActive ? 'block' : 'none'
          }}
          alt="ZED Camera Feed"
        />
      )}
      
      {/* ROSBridge Canvas Stream */}
      {streamMethod === 'rosbridge' && (
        <canvas
          ref={canvasRef}
          className="camera-canvas"
          style={{
            width: '100%',
            height: '100%',
            objectFit: 'cover',
            backgroundColor: '#000',
            display: feedActive ? 'block' : 'none'
          }}
        />
      )}
      
      {!feedActive && (
        <div className="no-feed-overlay">
          <div className="no-feed-message">
            <h3>📹 Camera Feed</h3>
            <p>
              {!isConnected 
                ? 'Waiting for ROS connection...' 
                : `Trying ${streamMethod === 'web_video_server' ? 'Web Video Server' : 'ROSBridge'} method...`
              }
            </p>
            <div className="loading-spinner"></div>
          </div>
        </div>
      )}

      {feedActive && (
        <div className="feed-info-overlay">
          <span className="feed-status">🔴 LIVE</span>
          <span className="feed-topic">/zed/zed_node/rgb/image_rect_color</span>
          <span className="feed-method">via {streamMethod === 'web_video_server' ? 'Web Video Server' : 'ROSBridge'}</span>
        </div>
      )}
    </div>
  );
};

export default CameraFeedViewer;
