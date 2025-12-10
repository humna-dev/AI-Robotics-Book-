import React, { useState, useEffect } from 'react';

const PersonalizationSettings = ({ onClose }) => {
  const [settings, setSettings] = useState({
    fontSize: 'medium',
    theme: 'light',
    readingSpeed: 'normal',
    showAnimations: true,
    showProgress: true
  });

  // Load saved settings from localStorage on component mount
  useEffect(() => {
    const savedSettings = localStorage.getItem('personalizationSettings');
    if (savedSettings) {
      try {
        const parsedSettings = JSON.parse(savedSettings);
        setSettings(parsedSettings);

        // Apply theme to document
        document.documentElement.setAttribute('data-theme', parsedSettings.theme);
      } catch (e) {
        console.error('Error loading settings:', e);
      }
    }
  }, []);

  // Save settings to localStorage when they change
  useEffect(() => {
    localStorage.setItem('personalizationSettings', JSON.stringify(settings));

    // Apply theme to document
    document.documentElement.setAttribute('data-theme', settings.theme);

    // Apply font size class to body
    document.body.classList.remove('font-size-small', 'font-size-medium', 'font-size-large');
    document.body.classList.add(`font-size-${settings.fontSize}`);
  }, [settings]);

  const handleSettingChange = (settingName, value) => {
    setSettings(prev => ({
      ...prev,
      [settingName]: value
    }));
  };

  const handleReset = () => {
    const defaultSettings = {
      fontSize: 'medium',
      theme: 'light',
      readingSpeed: 'normal',
      showAnimations: true,
      showProgress: true
    };
    setSettings(defaultSettings);
    localStorage.removeItem('personalizationSettings');
  };

  return (
    <div className="personalization-settings-overlay" style={{
      position: 'fixed',
      top: 0,
      left: 0,
      width: '100%',
      height: '100%',
      backgroundColor: 'rgba(0, 0, 0, 0.5)',
      zIndex: 1000,
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center'
    }}>
      <div className="personalization-settings-panel" style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '20px',
        width: '90%',
        maxWidth: '500px',
        maxHeight: '80vh',
        overflowY: 'auto',
        position: 'relative'
      }}>
        <div style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          marginBottom: '20px'
        }}>
          <h2 style={{ margin: 0 }}>Personalization Settings</h2>
          <button
            onClick={onClose}
            style={{
              background: 'none',
              border: 'none',
              fontSize: '1.5rem',
              cursor: 'pointer',
              padding: '5px'
            }}
          >
            ×
          </button>
        </div>

        <div className="settings-form">
          {/* Font Size Setting */}
          <div className="setting-item" style={{ marginBottom: '20px' }}>
            <label style={{ display: 'block', marginBottom: '8px', fontWeight: 'bold' }}>
              Font Size
            </label>
            <select
              value={settings.fontSize}
              onChange={(e) => handleSettingChange('fontSize', e.target.value)}
              style={{
                width: '100%',
                padding: '8px',
                borderRadius: '4px',
                border: '1px solid #ccc'
              }}
            >
              <option value="small">Small</option>
              <option value="medium">Medium</option>
              <option value="large">Large</option>
            </select>
          </div>

          {/* Theme Setting */}
          <div className="setting-item" style={{ marginBottom: '20px' }}>
            <label style={{ display: 'block', marginBottom: '8px', fontWeight: 'bold' }}>
              Theme
            </label>
            <select
              value={settings.theme}
              onChange={(e) => handleSettingChange('theme', e.target.value)}
              style={{
                width: '100%',
                padding: '8px',
                borderRadius: '4px',
                border: '1px solid #ccc'
              }}
            >
              <option value="light">Light</option>
              <option value="dark">Dark</option>
              <option value="auto">System</option>
            </select>
          </div>

          {/* Reading Speed Setting */}
          <div className="setting-item" style={{ marginBottom: '20px' }}>
            <label style={{ display: 'block', marginBottom: '8px', fontWeight: 'bold' }}>
              Reading Speed
            </label>
            <select
              value={settings.readingSpeed}
              onChange={(e) => handleSettingChange('readingSpeed', e.target.value)}
              style={{
                width: '100%',
                padding: '8px',
                borderRadius: '4px',
                border: '1px solid #ccc'
              }}
            >
              <option value="slow">Slow</option>
              <option value="normal">Normal</option>
              <option value="fast">Fast</option>
            </select>
          </div>

          {/* Animations Setting */}
          <div className="setting-item" style={{ marginBottom: '20px' }}>
            <label style={{ display: 'flex', alignItems: 'center' }}>
              <input
                type="checkbox"
                checked={settings.showAnimations}
                onChange={(e) => handleSettingChange('showAnimations', e.target.checked)}
                style={{ marginRight: '8px' }}
              />
              <span>Show Animations</span>
            </label>
          </div>

          {/* Progress Tracking Setting */}
          <div className="setting-item" style={{ marginBottom: '20px' }}>
            <label style={{ display: 'flex', alignItems: 'center' }}>
              <input
                type="checkbox"
                checked={settings.showProgress}
                onChange={(e) => handleSettingChange('showProgress', e.target.checked)}
                style={{ marginRight: '8px' }}
              />
              <span>Show Progress Tracking</span>
            </label>
          </div>
        </div>

        <div style={{
          display: 'flex',
          justifyContent: 'space-between',
          marginTop: '20px'
        }}>
          <button
            onClick={handleReset}
            style={{
              padding: '8px 16px',
              backgroundColor: '#6c757d',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Reset to Default
          </button>
          <button
            onClick={onClose}
            style={{
              padding: '8px 16px',
              backgroundColor: '#25c2a0',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Save & Close
          </button>
        </div>
      </div>
    </div>
  );
};

export default PersonalizationSettings;