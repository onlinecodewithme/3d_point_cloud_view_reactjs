import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import './Register.css';

const Register: React.FC = () => {
  const [formData, setFormData] = useState({
    username: '',
    email: '',
    password: '',
    confirmPassword: '',
    role: 'operator'
  });
  const [isLoading, setIsLoading] = useState(false);
  const [errors, setErrors] = useState<{[key: string]: string}>({});
  const [success, setSuccess] = useState(false);
  const navigate = useNavigate();

  const roles = [
    { value: 'operator', label: 'Robot Operator', icon: '👨‍🔧', description: 'Control and monitor robots' },
    { value: 'engineer', label: 'Systems Engineer', icon: '👩‍💻', description: 'Configure and maintain systems' },
    { value: 'supervisor', label: 'Supervisor', icon: '👨‍💼', description: 'Oversee operations and teams' }
  ];

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
    // Clear error for this field
    if (errors[e.target.name]) {
      setErrors({
        ...errors,
        [e.target.name]: ''
      });
    }
  };

  const validateForm = () => {
    const newErrors: {[key: string]: string} = {};

    if (!formData.username.trim()) {
      newErrors.username = 'Username is required';
    } else if (formData.username.length < 3) {
      newErrors.username = 'Username must be at least 3 characters';
    }

    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
      newErrors.email = 'Email is invalid';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 6) {
      newErrors.password = 'Password must be at least 6 characters';
    }

    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    // Simulate API call delay
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Simulate successful registration
    setSuccess(true);
    setIsLoading(false);

    // Redirect to login after 2 seconds
    setTimeout(() => {
      navigate('/login');
    }, 2000);
  };

  if (success) {
    return (
      <div className="register-container">
        <div className="register-background">
          <div className="circuit-pattern"></div>
          <div className="floating-particles"></div>
        </div>
        
        <div className="success-container">
          <div className="success-robot">
            <div className="robot-celebration">
              <div className="robot-head success">
                <div className="robot-eyes">
                  <div className="eye left-eye happy"></div>
                  <div className="eye right-eye happy"></div>
                </div>
                <div className="robot-smile"></div>
              </div>
            </div>
          </div>
          
          <div className="success-message">
            <h1>🎉 Registration Successful!</h1>
            <p>Welcome to the Xavier Control family!</p>
            <div className="success-details">
              <div className="detail-item">
                <span className="detail-icon">✅</span>
                <span>Account created successfully</span>
              </div>
              <div className="detail-item">
                <span className="detail-icon">🔐</span>
                <span>Security protocols activated</span>
              </div>
              <div className="detail-item">
                <span className="detail-icon">🚀</span>
                <span>Redirecting to login...</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="register-container">
      <div className="register-background">
        <div className="circuit-pattern"></div>
        <div className="floating-particles"></div>
      </div>
      
      <div className="register-content">
        <div className="robot-showcase">
          <div className="assembly-robot">
            <div className="robot-arm left-arm">
              <div className="arm-segment"></div>
              <div className="arm-joint"></div>
              <div className="arm-segment"></div>
              <div className="arm-end"></div>
            </div>
            
            <div className="robot-base">
              <div className="base-platform">
                <div className="platform-lights">
                  <div className="light active"></div>
                  <div className="light"></div>
                  <div className="light active"></div>
                </div>
              </div>
              <div className="base-column">
                <div className="column-rings">
                  <div className="ring"></div>
                  <div className="ring"></div>
                  <div className="ring"></div>
                </div>
              </div>
            </div>
            
            <div className="robot-arm right-arm">
              <div className="arm-segment"></div>
              <div className="arm-joint"></div>
              <div className="arm-segment"></div>
              <div className="arm-end"></div>
            </div>
          </div>
          
          <div className="robot-info">
            <h2>🔧 Join Our Team</h2>
            <p>Register for Advanced Robotics Access</p>
            <div className="features-list">
              <div className="feature-item">
                <span className="feature-icon">🤖</span>
                <span>Robot Control Access</span>
              </div>
              <div className="feature-item">
                <span className="feature-icon">📊</span>
                <span>Real-time Monitoring</span>
              </div>
              <div className="feature-item">
                <span className="feature-icon">🔧</span>
                <span>System Configuration</span>
              </div>
            </div>
          </div>
        </div>

        <div className="register-form-container">
          <div className="register-header">
            <h1>Create Account</h1>
            <p>Join the future of robotics control</p>
          </div>

          <form onSubmit={handleSubmit} className="register-form">
            <div className="form-row">
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
                    placeholder="Choose a username"
                    className={errors.username ? 'error' : ''}
                  />
                </div>
                {errors.username && (
                  <span className="field-error">{errors.username}</span>
                )}
              </div>

              <div className="form-group">
                <label htmlFor="email">Email</label>
                <div className="input-container">
                  <span className="input-icon">📧</span>
                  <input
                    type="email"
                    id="email"
                    name="email"
                    value={formData.email}
                    onChange={handleInputChange}
                    placeholder="Enter your email"
                    className={errors.email ? 'error' : ''}
                  />
                </div>
                {errors.email && (
                  <span className="field-error">{errors.email}</span>
                )}
              </div>
            </div>

            <div className="form-row">
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
                    placeholder="Create a password"
                    className={errors.password ? 'error' : ''}
                  />
                </div>
                {errors.password && (
                  <span className="field-error">{errors.password}</span>
                )}
              </div>

              <div className="form-group">
                <label htmlFor="confirmPassword">Confirm Password</label>
                <div className="input-container">
                  <span className="input-icon">🔐</span>
                  <input
                    type="password"
                    id="confirmPassword"
                    name="confirmPassword"
                    value={formData.confirmPassword}
                    onChange={handleInputChange}
                    placeholder="Confirm your password"
                    className={errors.confirmPassword ? 'error' : ''}
                  />
                </div>
                {errors.confirmPassword && (
                  <span className="field-error">{errors.confirmPassword}</span>
                )}
              </div>
            </div>

            <div className="form-group">
              <label htmlFor="role">Role</label>
              <div className="role-selection">
                {roles.map((role) => (
                  <div
                    key={role.value}
                    className={`role-option ${formData.role === role.value ? 'selected' : ''}`}
                    onClick={() => setFormData({ ...formData, role: role.value })}
                  >
                    <div className="role-icon">{role.icon}</div>
                    <div className="role-info">
                      <div className="role-title">{role.label}</div>
                      <div className="role-description">{role.description}</div>
                    </div>
                    <div className="role-radio">
                      <input
                        type="radio"
                        name="role"
                        value={role.value}
                        checked={formData.role === role.value}
                        onChange={handleInputChange}
                      />
                    </div>
                  </div>
                ))}
              </div>
            </div>

            <button 
              type="submit" 
              className={`register-button ${isLoading ? 'loading' : ''}`}
              disabled={isLoading}
            >
              {isLoading ? (
                <>
                  <div className="loading-spinner"></div>
                  Creating Account...
                </>
              ) : (
                <>
                  <span className="button-icon">🚀</span>
                  Create Account
                </>
              )}
            </button>
          </form>

          <div className="register-footer">
            <p>Already have an account? <a href="/login">Sign in here</a></p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Register;
