import React, { useState, useEffect } from 'react';

const TextSelector = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showChat, setShowChat] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleSelection = () => {
      const selectedTextObj = window.getSelection();
      const text = selectedTextObj.toString().trim();

      if (text) {
        // Get the bounding rectangle of the selection
        const range = selectedTextObj.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(text);
        setPosition({ x: rect.right, y: rect.top });
        setShowChat(true);
      } else {
        setShowChat(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleAskQuestion = () => {
    if (selectedText && onTextSelected) {
      onTextSelected(selectedText);
      setShowChat(false);
      setSelectedText('');
    }
  };

  if (!showChat || !selectedText) {
    return null;
  }

  return (
    <div
      className="text-selector-popup"
      style={{
        position: 'fixed',
        left: position.x + 10,
        top: position.y - 30,
        backgroundColor: '#25c2a0',
        color: 'white',
        padding: '8px 12px',
        borderRadius: '4px',
        zIndex: 1000,
        boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
        fontSize: '14px',
        cursor: 'pointer'
      }}
      onClick={handleAskQuestion}
    >
      Ask about this
    </div>
  );
};

export default TextSelector;