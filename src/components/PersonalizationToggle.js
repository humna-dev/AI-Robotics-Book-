import React, { useState } from 'react';
import PersonalizationSettings from './PersonalizationSettings';

const PersonalizationToggle = () => {
  const [showSettings, setShowSettings] = useState(false);

  return (
    <>
      <button
        onClick={() => setShowSettings(true)}
        style={{
          position: 'fixed',
        bottom: '20px',
        right: '90px', // Position to the left of the chatbot
        width: '40px',
        height: '40px',
        borderRadius: '50%',
        backgroundColor: '#6c757d',
        color: 'white',
        border: 'none',
        fontSize: '16px',
        cursor: 'pointer',
        zIndex: 997,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        boxShadow: '0 2px 8px rgba(0,0,0,0.15)'
      }}
        title="Personalization Settings"
      >
        ⚙️
      </button>

      {showSettings && (
        <PersonalizationSettings onClose={() => setShowSettings(false)} />
      )}
    </>
  );
};

export default PersonalizationToggle;