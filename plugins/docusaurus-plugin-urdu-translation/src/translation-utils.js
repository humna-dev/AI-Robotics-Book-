// Translation utilities for Urdu translation functionality

// Mock translation function - in a real implementation, this would call a translation API
export const translateText = async (text, targetLanguage = 'ur') => {
  if (targetLanguage !== 'ur') {
    return text; // Only support Urdu for now
  }

  // In a real implementation, this would call an API like:
  // const response = await fetch('/api/translate', {
  //   method: 'POST',
  //   headers: { 'Content-Type': 'application/json' },
  //   body: JSON.stringify({ text, targetLanguage })
  // });
  // return await response.json();

  // For this example, we'll return a mock translation
  // This would be replaced with actual translation logic
  const mockTranslations = {
    "Welcome to": "خوش آمدید",
    "Physical AI & Humanoid Robotics": "فزیکل ای آئی اور ہیومنوائڈ روبوٹکس",
    "An AI-Native Textbook for the Next Generation of Robotics": "روبوٹکس کی اگلی نسل کے لیے ایک ای آئی نیٹو ٹیکسٹ بک",
    "Read the Textbook": "متن کتاب پڑھیں",
    "Course Modules": "کورس کے موڈیولز",
    "Explore the 5 comprehensive modules of Physical AI & Humanoid Robotics": "فزیکل ای آئی اور ہیومنوائڈ روبوٹکس کے 5 جامع موڈیولز کا جائزہ لیں",
    "Start Reading the Textbook": "متن کتاب پڑھنا شروع کریں",
    "Module": "موڈیول",
    "The Robotic Nervous System (ROS 2)": "روبوٹک نروس سسٹم (ROS 2)",
    "The Digital Twin (Gazebo & Unity)": "ڈیجیٹل ٹوئن (گیزیبو اور یونیٹی)",
    "The AI-Robot Brain (NVIDIA Isaac)": "ای آئی - روبوٹ براہین (این وی ڈی ای ایسیک)",
    "Vision-Language-Action (VLA)": "وژن - لینگویج - ایکشن (وی ایل اے)",
    "Capstone Autonomous Humanoid Project": "کیپسٹون خودکار ہیومنوائڈ پروجیکٹ"
  };

  // Simple word-by-word translation for demo purposes
  let translated = text;
  for (const [english, urdu] of Object.entries(mockTranslations)) {
    translated = translated.replace(new RegExp(english, 'gi'), urdu);
  }

  // If no specific translation found, return original text with a note
  if (translated === text) {
    console.warn(`No translation found for: "${text}"`);
    // In a real implementation, we would call the translation API
    return `[URDU: ${text}]`; // Placeholder for untranslated text
  }

  return translated;
};

// This is a simplified version since we can't use React hooks directly here
// The actual hook would be in a separate component file
export const getLanguagePreference = () => {
  if (typeof window !== 'undefined') {
    return localStorage.getItem('preferredLanguage') || 'en';
  }
  return 'en';
};

export const setLanguagePreference = (language) => {
  if (typeof window !== 'undefined') {
    localStorage.setItem('preferredLanguage', language);
  }
};