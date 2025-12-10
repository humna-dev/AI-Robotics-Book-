import React, { useState, useEffect } from 'react';

const ProgressTracker = () => {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const calculateProgress = () => {
      const scrollTop = window.scrollY;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const newProgress = docHeight > 0 ? (scrollTop / docHeight) * 100 : 0;
      setProgress(newProgress);
    };

    // Initial calculation
    calculateProgress();

    // Add scroll event listener
    window.addEventListener('scroll', calculateProgress);

    // Cleanup
    return () => {
      window.removeEventListener('scroll', calculateProgress);
    };
  }, []);

  return (
    <div className="progress-tracker-container" style={{
      position: 'fixed',
      top: '0',
      left: '0',
      width: '100%',
      height: '4px',
      backgroundColor: '#e0e0e0',
      zIndex: 1000,
    }}>
      <div
        className="progress-bar"
        style={{
          height: '100%',
          width: `${progress}%`,
          backgroundColor: '#25c2a0',
          transition: 'width 0.2s ease-out',
        }}
      />
    </div>
  );
};

export default ProgressTracker;