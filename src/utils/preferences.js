// Preferences utility functions for user personalization

class PreferencesManager {
  constructor() {
    this.storageKey = 'personalizationSettings';
    this.defaultSettings = {
      fontSize: 'medium',
      theme: 'light',
      readingSpeed: 'normal',
      showAnimations: true,
      showProgress: true,
      language: 'en',
      lastUpdated: null
    };
  }

  // Get all preferences
  getPreferences() {
    try {
      const saved = localStorage.getItem(this.storageKey);
      if (saved) {
        const parsed = JSON.parse(saved);
        return { ...this.defaultSettings, ...parsed };
      }
      return this.defaultSettings;
    } catch (error) {
      console.error('Error loading preferences:', error);
      return this.defaultSettings;
    }
  }

  // Get a specific preference
  getPreference(key) {
    const preferences = this.getPreferences();
    return preferences[key] !== undefined ? preferences[key] : this.defaultSettings[key];
  }

  // Set a specific preference
  setPreference(key, value) {
    try {
      const preferences = this.getPreferences();
      preferences[key] = value;
      preferences.lastUpdated = new Date().toISOString();
      localStorage.setItem(this.storageKey, JSON.stringify(preferences));
      return true;
    } catch (error) {
      console.error('Error saving preference:', error);
      return false;
    }
  }

  // Set multiple preferences at once
  setPreferences(newPreferences) {
    try {
      const currentPreferences = this.getPreferences();
      const updatedPreferences = {
        ...currentPreferences,
        ...newPreferences,
        lastUpdated: new Date().toISOString()
      };
      localStorage.setItem(this.storageKey, JSON.stringify(updatedPreferences));
      return true;
    } catch (error) {
      console.error('Error saving preferences:', error);
      return false;
    }
  }

  // Reset to default preferences
  resetPreferences() {
    try {
      localStorage.removeItem(this.storageKey);
      return true;
    } catch (error) {
      console.error('Error resetting preferences:', error);
      return false;
    }
  }

  // Apply theme to document
  applyTheme(theme) {
    if (theme === 'auto') {
      const systemTheme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
      document.documentElement.setAttribute('data-theme', systemTheme);
    } else {
      document.documentElement.setAttribute('data-theme', theme);
    }
  }

  // Apply font size to document
  applyFontSize(fontSize) {
    document.body.classList.remove('font-size-small', 'font-size-medium', 'font-size-large');
    document.body.classList.add(`font-size-${fontSize}`);
  }

  // Apply all settings to the UI
  applyAllSettings() {
    const preferences = this.getPreferences();

    // Apply theme
    this.applyTheme(preferences.theme);

    // Apply font size
    this.applyFontSize(preferences.fontSize);
  }

  // Initialize preferences on app load
  initialize() {
    this.applyAllSettings();
  }
}

// Create a singleton instance
const preferencesManager = new PreferencesManager();

// Export the class and instance
export { PreferencesManager };
export default preferencesManager;