import React from 'react';
import Link from '@docusaurus/Link';

const ModuleCard = ({ moduleNumber, title, description, link, icon }) => {
  return (
    <Link
      to={link}
      style={{ textDecoration: 'none', color: 'inherit' }}
      className="module-card-link"
    >
      <div
        className="module-card"
        style={{
          border: '1px solid #e0e0e0',
          borderRadius: '8px',
          padding: '20px',
          backgroundColor: 'white',
          transition: 'transform 0.2s, box-shadow 0.2s',
          height: '100%',
          display: 'flex',
          flexDirection: 'column'
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'translateY(-5px)';
          e.currentTarget.style.boxShadow = '0 6px 16px rgba(0,0,0,0.15)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'translateY(0)';
          e.currentTarget.style.boxShadow = '0 2px 8px rgba(0,0,0,0.1)';
        }}
      >
        <div
          style={{
            fontSize: '2rem',
            marginBottom: '15px',
            textAlign: 'center'
          }}
        >
          {icon || `📚`}
        </div>
        <h3
          style={{
            margin: '0 0 10px 0',
            fontSize: '1.1rem',
            color: '#25c2a0'
          }}
        >
          Module {moduleNumber}: {title}
        </h3>
        <p
          style={{
            margin: '0 0 15px 0',
            color: '#666',
            flex: 1,
            fontSize: '0.9rem'
          }}
        >
          {description}
        </p>
        <div
          style={{
            textAlign: 'right',
            fontSize: '0.85rem',
            color: '#25c2a0',
            fontWeight: '500'
          }}
        >
          Explore →
        </div>
      </div>
    </Link>
  );
};

export default ModuleCard;