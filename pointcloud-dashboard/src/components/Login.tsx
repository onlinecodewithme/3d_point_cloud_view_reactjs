import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import './Login.css';

interface LoginProps {
  onLogin: (user: any) => void;
}

const Login: React.FC<LoginProps> = ({ onLogin }) => {
  const [formData, setFormData] = useState({
    username: '',
    password: ''
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const navigate = useNavigate();

  // Dummy user data
  const dummyUsers = [
    {
      id: 1,
      username: 'admin',
      password: 'admin123',
      email: 'admin@robotics.com',
      role: 'Administrator',
      avatar: '🤖',
      joinDate: '2024-01-15',
      lastLogin: new Date().toISOString(),
      permissions: ['all']
    },
    {
      id: 2,
      username: 'operator',
      password: 'op123',
      email: 'operator@robotics.com',
      role: 'Robot Operator',
      avatar: '👨‍🔧',
      joinDate: '2024-02-20',
      lastLogin: new Date().toISOString(),
      permissions: ['control', 'monitor']
    },
    {
      id: 3,
      username: 'engineer',
      password: 'eng123',
      email: 'engineer@robotics.com',
      role: 'Systems Engineer',
      avatar: '👩‍💻',
      joinDate: '2024-03-10',
      lastLogin: new Date().toISOString(),
      permissions: ['control', 'monitor', 'configure']
    }
  ];

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
    setError('');
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    // Simulate API call delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    const user = dummyUsers.find(
      u => u.username === formData.username && u.password === formData.password
    );

    if (user) {
      const userWithSession = {
        ...user,
        lastLogin: new Date().toISOString()
      };
      localStorage.setItem('currentUser', JSON.stringify(userWithSession));
      onLogin(userWithSession);
      navigate('/');
    } else {
      setError('Invalid username or password');
    }

    setIsLoading(false);
  };

  const handleDemoLogin = (username: string) => {
    const user = dummyUsers.find(u => u.username === username);
    if (user) {
      setFormData({ username: user.username, password: user.password });
    }
  };

  return (
    <div className="login-page">
      <div className="login-container">
      <div className="login-background">
        <div className="circuit-pattern"></div>
        <div className="floating-particles"></div>
      </div>
      
      <div className="login-content">
        <div className="robot-showcase">
          <div className="robot-logo-container">
            <div className="logo-glow-effect"></div>
            <img 
              src="/logo2.png" 
              alt="Xavier Robot" 
              className="robot-logo-image"
            />
            <div className="logo-overlay-effects">
              <div className="scanning-line"></div>
              <div className="pulse-ring"></div>
            </div>
          </div>
          
          <div className="robot-info">
            <h2>🤖 Xavier Control Dashboard</h2>
            <p>Advanced Robotics Management System</p>
            <div className="system-status">
              <div className="status-item">
                <span className="status-dot online"></span>
                <span>System Online</span>
              </div>
              <div className="status-item">
                <span className="status-dot online"></span>
                <span>Secure Connection</span>
              </div>
            </div>
          </div>
        </div>

        <div className="login-form-container">
          <div className="login-header">
            <h1>Welcome Back</h1>
            <p>Sign in to access the robot control dashboard</p>
          </div>

          <form onSubmit={handleSubmit} className="login-form">
            <div className="form-group">
              <label htmlFor="username">Username</label>
              <div className="input-container">
                <span className="input-icon">👤</span>
                <input
                  type="text"
                  id="username"
                  name="username"
                  value={formData.username}
                  onChange={handleInputChange}
                  placeholder="Enter your username"
                  required
                />
              </div>
            </div>

            <div className="form-group">
              <label htmlFor="password">Password</label>
              <div className="input-container">
                <span className="input-icon">🔒</span>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  placeholder="Enter your password"
                  required
                />
              </div>
            </div>

            {error && (
              <div className="error-message">
                <span className="error-icon">⚠️</span>
                {error}
              </div>
            )}

            <button 
              type="submit" 
              className={`login-button ${isLoading ? 'loading' : ''}`}
              disabled={isLoading}
            >
              {isLoading ? (
                <>
                  <div className="loading-spinner"></div>
                  Authenticating...
                </>
              ) : (
                <>
                  <span className="button-icon">🚀</span>
                  Sign In
                </>
              )}
            </button>
          </form>

          <div className="demo-accounts">
            <p>Demo Accounts:</p>
            <div className="demo-buttons">
              <button 
                className="demo-button admin"
                onClick={() => handleDemoLogin('admin')}
              >
                🤖 Admin
              </button>
              <button 
                className="demo-button operator"
                onClick={() => handleDemoLogin('operator')}
              >
                👨‍🔧 Operator
              </button>
              <button 
                className="demo-button engineer"
                onClick={() => handleDemoLogin('engineer')}
              >
                👩‍💻 Engineer
              </button>
            </div>
          </div>

          <div className="login-footer">
            <p>Don't have an account? <a href="/register">Create one here</a></p>
          </div>
        </div>
      </div>
    </div>
    </div>
  );
};

export default Login;
