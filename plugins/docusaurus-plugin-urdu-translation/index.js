const path = require('path');

module.exports = function (context, options) {
  const { siteDir } = context;

  return {
    name: 'docusaurus-plugin-urdu-translation',

    getClientModules() {
      return [path.resolve(__dirname, './src/translation-client-module')];
    },

    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@translation': path.resolve(__dirname, './src'),
          },
        },
      };
    },

    // Add plugin translations to the site
    getTranslationFiles() {
      return [
        {
          path: path.resolve(__dirname, 'translations.json'),
          load: () => require('./translations.json'),
        },
      ];
    },
  };
};