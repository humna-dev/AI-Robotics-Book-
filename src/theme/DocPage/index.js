import React from 'react';
import DocPage from '@theme-original/DocPage';
import Chatbot from '@site/src/components/Chatbot';
import TextSelector from '@site/src/components/TextSelector';
import PersonalizationToggle from '@site/src/components/PersonalizationToggle';

export default function DocPageWrapper(props) {
  const [selectedText, setSelectedText] = React.useState(null);

  const handleTextSelected = (text) => {
    setSelectedText(text);
  };

  return (
    <>
      <DocPage {...props} />
      <TextSelector onTextSelected={handleTextSelected} />
      <Chatbot selectedText={selectedText} />
      <PersonalizationToggle />
    </>
  );
}