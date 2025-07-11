import React, { useState, useEffect } from 'react';
import './Profile.css';

interface User {
  id: number;
  username: string;
  email: string;
  role: string;
  avatar: string;
  joinDate: string;
  lastLogin: string;
  permissions: string[];
  preferences: {
    theme: string;
    notifications: boolean;
    autoSave: boolean;
    language: string;
  };
  stats: {
    sessionsCount: number;
    totalUptime: number;
    robotsControlled: number;
    tasksCompleted: number;
  };
}

const Profile: React.FC = () => {
  const [user, setUser] = useState<User | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [editForm, setEditForm] = useState({
    username: '',
    email: '',
    currentPassword: '',
    newPassword: '',
    confirmPassword: ''
  });
  const [activeTab, setActiveTab] = useState('overview');

  useEffect(() => {
    // Load user data from localStorage
    const currentUser = localStorage.getItem('currentUser');
    if (currentUser) {
      const userData = JSON.parse(currentUser);
      // Add additional profile data
      const enhancedUser: User = {
        ...userData,
        preferences: {
          theme: 'dark',
          notifications: true,
          autoSave: true,
          language: 'en'
        },
        stats: {
          sessionsCount: Math.floor(Math.random() * 100) + 50,
          totalUptime: Math.floor(Math.random() * 1000) + 500,
          robotsControlled: Math.floor(Math.random() * 20) + 5,
          tasksCompleted: Math.floor(Math.random() * 500) + 200
        }
      };
      setUser(enhancedUser);
      setEditForm({
        username: enhancedUser.username,
        email: enhancedUser.email,
        currentPassword: '',
        newPassword: '',
        confirmPassword: ''
      });
    }
  }, []);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setEditForm({
      ...editForm,
      [e.target.name]: e.target.value
    });
  };

  const handleSave = () => {
    if (user) {
      const updatedUser = {
        ...user,
        username: editForm.username,
        email: editForm.email
      };
      setUser(updatedUser);
      localStorage.setItem('currentUser', JSON.stringify(updatedUser));
      setIsEditing(false);
    }
  };

  const formatUptime = (hours: number): string => {
    const days = Math.floor(hours / 24);
    const remainingHours = hours % 24;
    return `${days}d ${remainingHours}h`;
  };

  const getRoleColor = (role: string): string => {
    switch (role.toLowerCase()) {
      case 'administrator': return '#ff4444';
      case 'robot operator': return '#00aaff';
      case 'systems engineer': return '#ffaa00';
      default: return '#00ff41';
    }
  };

  const getPermissionIcon = (permission: string): string => {
    switch (permission) {
      case 'all': return '🔑';
      case 'control': return '🎮';
      case 'monitor': return '📊';
      case 'configure': return '⚙️';
      default: return '📋';
    }
  };

  if (!user) {
    return (
      <div className="profile-container">
        <div className="profile-loading">
          <div className="loading-spinner"></div>
          <p>Loading profile...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="profile-container">
      <div className="profile-header">
        <div className="profile-banner">
          <div className="banner-pattern"></div>
        </div>
        
        <div className="profile-info">
          <div className="avatar-section">
            <div className="avatar-container">
              <div className="avatar">{user.avatar}</div>
              <div className="avatar-status online"></div>
            </div>
            <button className="change-avatar-btn">
              📷 Change Avatar
            </button>
          </div>
          
          <div className="user-details">
            <h1 className="username">{user.username}</h1>
            <div className="user-role" style={{ color: getRoleColor(user.role) }}>
              {user.role}
            </div>
            <div className="user-meta">
              <div className="meta-item">
                <span className="meta-icon">📧</span>
                <span>{user.email}</span>
              </div>
              <div className="meta-item">
                <span className="meta-icon">📅</span>
                <span>Joined {new Date(user.joinDate).toLocaleDateString()}</span>
              </div>
              <div className="meta-item">
                <span className="meta-icon">🕒</span>
                <span>Last login {new Date(user.lastLogin).toLocaleString()}</span>
              </div>
            </div>
          </div>
          
          <div className="profile-actions">
            <button 
              className={`edit-btn ${isEditing ? 'active' : ''}`}
              onClick={() => setIsEditing(!isEditing)}
            >
              {isEditing ? '❌ Cancel' : '✏️ Edit Profile'}
            </button>
          </div>
        </div>
      </div>

      <div className="profile-content">
        <div className="profile-tabs">
          <button 
            className={`tab-btn ${activeTab === 'overview' ? 'active' : ''}`}
            onClick={() => setActiveTab('overview')}
          >
            📊 Overview
          </button>
          <button 
            className={`tab-btn ${activeTab === 'stats' ? 'active' : ''}`}
            onClick={() => setActiveTab('stats')}
          >
            📈 Statistics
          </button>
          <button 
            className={`tab-btn ${activeTab === 'permissions' ? 'active' : ''}`}
            onClick={() => setActiveTab('permissions')}
          >
            🔐 Permissions
          </button>
          <button 
            className={`tab-btn ${activeTab === 'settings' ? 'active' : ''}`}
            onClick={() => setActiveTab('settings')}
          >
            ⚙️ Settings
          </button>
        </div>

        <div className="tab-content">
          {activeTab === 'overview' && (
            <div className="overview-tab">
              {isEditing ? (
                <div className="edit-form">
                  <h3>Edit Profile Information</h3>
                  <div className="form-grid">
                    <div className="form-group">
                      <label>Username</label>
                      <input
                        type="text"
                        name="username"
                        value={editForm.username}
                        onChange={handleInputChange}
                        placeholder="Enter username"
                      />
                    </div>
                    <div className="form-group">
                      <label>Email</label>
                      <input
                        type="email"
                        name="email"
                        value={editForm.email}
                        onChange={handleInputChange}
                        placeholder="Enter email"
                      />
                    </div>
                    <div className="form-group">
                      <label>Current Password</label>
                      <input
                        type="password"
                        name="currentPassword"
                        value={editForm.currentPassword}
                        onChange={handleInputChange}
                        placeholder="Enter current password"
                      />
                    </div>
                    <div className="form-group">
                      <label>New Password</label>
                      <input
                        type="password"
                        name="newPassword"
                        value={editForm.newPassword}
                        onChange={handleInputChange}
                        placeholder="Enter new password"
                      />
                    </div>
                    <div className="form-group">
                      <label>Confirm New Password</label>
                      <input
                        type="password"
                        name="confirmPassword"
                        value={editForm.confirmPassword}
                        onChange={handleInputChange}
                        placeholder="Confirm new password"
                      />
                    </div>
                  </div>
                  <div className="form-actions">
                    <button className="save-btn" onClick={handleSave}>
                      💾 Save Changes
                    </button>
                    <button className="cancel-btn" onClick={() => setIsEditing(false)}>
                      ❌ Cancel
                    </button>
                  </div>
                </div>
              ) : (
                <div className="overview-content">
                  <div className="info-cards">
                    <div className="info-card">
                      <div className="card-icon">👤</div>
                      <div className="card-content">
                        <h4>Account Information</h4>
                        <div className="card-details">
                          <div className="detail-row">
                            <span className="detail-label">Username:</span>
                            <span className="detail-value">{user.username}</span>
                          </div>
                          <div className="detail-row">
                            <span className="detail-label">Email:</span>
                            <span className="detail-value">{user.email}</span>
                          </div>
                          <div className="detail-row">
                            <span className="detail-label">Role:</span>
                            <span className="detail-value" style={{ color: getRoleColor(user.role) }}>
                              {user.role}
                            </span>
                          </div>
                        </div>
                      </div>
                    </div>

                    <div className="info-card">
                      <div className="card-icon">📊</div>
                      <div className="card-content">
                        <h4>Quick Stats</h4>
                        <div className="stats-grid">
                          <div className="stat-item">
                            <div className="stat-value">{user.stats.sessionsCount}</div>
                            <div className="stat-label">Sessions</div>
                          </div>
                          <div className="stat-item">
                            <div className="stat-value">{formatUptime(user.stats.totalUptime)}</div>
                            <div className="stat-label">Total Uptime</div>
                          </div>
                          <div className="stat-item">
                            <div className="stat-value">{user.stats.robotsControlled}</div>
                            <div className="stat-label">Robots Controlled</div>
                          </div>
                          <div className="stat-item">
                            <div className="stat-value">{user.stats.tasksCompleted}</div>
                            <div className="stat-label">Tasks Completed</div>
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              )}
            </div>
          )}

          {activeTab === 'stats' && (
            <div className="stats-tab">
              <div className="stats-header">
                <h3>📈 Detailed Statistics</h3>
                <p>Your activity and performance metrics</p>
              </div>
              
              <div className="stats-cards">
                <div className="stat-card">
                  <div className="stat-card-header">
                    <div className="stat-card-icon">🎮</div>
                    <h4>Control Sessions</h4>
                  </div>
                  <div className="stat-card-value">{user.stats.sessionsCount}</div>
                  <div className="stat-card-trend">+12% from last month</div>
                </div>

                <div className="stat-card">
                  <div className="stat-card-header">
                    <div className="stat-card-icon">⏱️</div>
                    <h4>Total Uptime</h4>
                  </div>
                  <div className="stat-card-value">{formatUptime(user.stats.totalUptime)}</div>
                  <div className="stat-card-trend">+8% from last month</div>
                </div>

                <div className="stat-card">
                  <div className="stat-card-header">
                    <div className="stat-card-icon">🤖</div>
                    <h4>Robots Managed</h4>
                  </div>
                  <div className="stat-card-value">{user.stats.robotsControlled}</div>
                  <div className="stat-card-trend">+3 new this month</div>
                </div>

                <div className="stat-card">
                  <div className="stat-card-header">
                    <div className="stat-card-icon">✅</div>
                    <h4>Tasks Completed</h4>
                  </div>
                  <div className="stat-card-value">{user.stats.tasksCompleted}</div>
                  <div className="stat-card-trend">+25% success rate</div>
                </div>
              </div>
            </div>
          )}

          {activeTab === 'permissions' && (
            <div className="permissions-tab">
              <div className="permissions-header">
                <h3>🔐 Access Permissions</h3>
                <p>Your current system access levels</p>
              </div>
              
              <div className="permissions-list">
                {user.permissions.map((permission, index) => (
                  <div key={index} className="permission-item">
                    <div className="permission-icon">
                      {getPermissionIcon(permission)}
                    </div>
                    <div className="permission-details">
                      <div className="permission-name">
                        {permission === 'all' ? 'Full System Access' : 
                         permission.charAt(0).toUpperCase() + permission.slice(1)}
                      </div>
                      <div className="permission-description">
                        {permission === 'all' ? 'Complete administrative access to all systems' :
                         permission === 'control' ? 'Robot control and operation capabilities' :
                         permission === 'monitor' ? 'System monitoring and data viewing' :
                         permission === 'configure' ? 'System configuration and settings' :
                         'Standard user permissions'}
                      </div>
                    </div>
                    <div className="permission-status granted">
                      ✅ Granted
                    </div>
                  </div>
                ))}
              </div>
            </div>
          )}

          {activeTab === 'settings' && (
            <div className="settings-tab">
              <div className="settings-header">
                <h3>⚙️ User Preferences</h3>
                <p>Customize your dashboard experience</p>
              </div>
              
              <div className="settings-sections">
                <div className="settings-section">
                  <h4>🎨 Appearance</h4>
                  <div className="setting-item">
                    <div className="setting-info">
                      <div className="setting-name">Theme</div>
                      <div className="setting-description">Choose your preferred color scheme</div>
                    </div>
                    <select className="setting-control" value={user.preferences.theme}>
                      <option value="dark">Dark</option>
                      <option value="light">Light</option>
                      <option value="auto">Auto</option>
                    </select>
                  </div>
                  
                  <div className="setting-item">
                    <div className="setting-info">
                      <div className="setting-name">Language</div>
                      <div className="setting-description">Select your preferred language</div>
                    </div>
                    <select className="setting-control" value={user.preferences.language}>
                      <option value="en">English</option>
                      <option value="es">Spanish</option>
                      <option value="fr">French</option>
                      <option value="de">German</option>
                    </select>
                  </div>
                </div>

                <div className="settings-section">
                  <h4>🔔 Notifications</h4>
                  <div className="setting-item">
                    <div className="setting-info">
                      <div className="setting-name">Enable Notifications</div>
                      <div className="setting-description">Receive system alerts and updates</div>
                    </div>
                    <label className="toggle-switch">
                      <input 
                        type="checkbox" 
                        checked={user.preferences.notifications}
                        onChange={() => {}}
                      />
                      <span className="toggle-slider"></span>
                    </label>
                  </div>
                </div>

                <div className="settings-section">
                  <h4>💾 Data</h4>
                  <div className="setting-item">
                    <div className="setting-info">
                      <div className="setting-name">Auto Save</div>
                      <div className="setting-description">Automatically save your work</div>
                    </div>
                    <label className="toggle-switch">
                      <input 
                        type="checkbox" 
                        checked={user.preferences.autoSave}
                        onChange={() => {}}
                      />
                      <span className="toggle-slider"></span>
                    </label>
                  </div>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default Profile;
