import React from 'react';
import clsx from 'clsx';
import useColorMode from '@theme/hooks/useColorMode';

const ColorModeToggle = ({className}) => {
  const {colorMode, setColorMode} = useColorMode();

  return (
    <div className={clsx('color-mode-toggle', className)}>
      <button
        onClick={() =>
          setColorMode(colorMode === 'light' ? 'dark' : 'light')
        }
        aria-label={`Switch to ${colorMode === 'light' ? 'dark' : 'light'} mode`}
        className="button button--secondary"
      >
        {colorMode === 'light' ? '🌙' : '☀️'}
      </button>
    </div>
  );
};

export default ColorModeToggle;