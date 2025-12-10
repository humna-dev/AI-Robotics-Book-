import React, { useState, useEffect } from 'react';
import { getLanguagePreference, setLanguagePreference, translateText } from '../../plugins/docusaurus-plugin-urdu-translation/src/translation-utils';

const TranslationButton = () => {
  const [currentLanguage, setCurrentLanguage] = useState('en');
  const [isTranslating, setIsTranslating] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  // Initialize language preference
  useEffect(() => {
    const savedLanguage = getLanguagePreference();
    setCurrentLanguage(savedLanguage);
  }, []);

  const toggleLanguage = async () => {
    const newLanguage = currentLanguage === 'en' ? 'ur' : 'en';
    setIsTranslating(true);

    try {
      // Save preference
      setLanguagePreference(newLanguage);
      setCurrentLanguage(newLanguage);

      // In a real implementation, we would translate the page content
      // For now, we'll just show a message and simulate the process
      console.log(`Language switched to: ${newLanguage}`);

      // Simulate translation process
      await new Promise(resolve => setTimeout(resolve, 500));
    } catch (error) {
      console.error('Translation error:', error);
    } finally {
      setIsTranslating(false);
    }
  };

  const getButtonText = () => {
    if (isTranslating) return 'Translating...';
    return currentLanguage === 'en' ? '.Translate to Urdu' : 'English';
  };

  return (
    <div className="translation-button-container" style={{ position: 'relative', display: 'inline-block' }}>
      <button
        onClick={toggleLanguage}
        disabled={isTranslating}
        style={{
          padding: '8px 12px',
          backgroundColor: currentLanguage === 'ur' ? '#28a745' : '#6c757d',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isTranslating ? 'wait' : 'pointer',
          fontSize: '14px',
          display: 'flex',
          alignItems: 'center',
          gap: '5px'
        }}
      >
        {isTranslating ? (
          <>
            <span>🔄</span> {getButtonText()}
          </>
        ) : (
          <>
            <span>{currentLanguage === 'en' ? '🇺🇸' : '🇵🇰'}</span> {getButtonText()}
          </>
        )}
      </button>
    </div>
  );
};

export default TranslationButton;