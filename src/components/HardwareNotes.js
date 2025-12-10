import React from 'react';

const HardwareNotes = ({ notes, title = "Hardware & Deployment Notes" }) => {
  if (!notes || notes.length === 0) {
    return null;
  }

  return (
    <div className="hardware-notes" style={{
      backgroundColor: '#f8f9fa',
      border: '1px solid #e9ecef',
      borderRadius: '8px',
      padding: '16px',
      margin: '20px 0',
      borderLeft: '4px solid #25c2a0'
    }}>
      <h3 style={{
        margin: '0 0 12px 0',
        color: '#25c2a0',
        fontSize: '1.2em'
      }}>
        {title}
      </h3>
      <ul style={{
        margin: 0,
        padding: '0 0 0 20px'
      }}>
        {notes.map((note, index) => (
          <li key={index} style={{
            marginBottom: '8px',
            lineHeight: '1.5'
          }}>
            {note}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default HardwareNotes;