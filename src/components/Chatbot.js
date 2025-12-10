import React, { useState, useEffect, useRef } from 'react';

const Chatbot = ({ selectedText = null }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(() => {
    return localStorage.getItem('chat_session_id') || Date.now().toString();
  });
  const messagesEndRef = useRef(null);

  // Initialize with selected text if provided
  useEffect(() => {
    if (selectedText) {
      setMessages([{
        id: Date.now(),
        role: 'user',
        content: selectedText,
        timestamp: new Date()
      }]);
      setIsOpen(true);
    }
  }, [selectedText]);

  // Save session ID to localStorage
  useEffect(() => {
    localStorage.setItem('chat_session_id', sessionId);
  }, [sessionId]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();

    if (!inputMessage.trim()) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputMessage,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      // In a real implementation, this would call the backend API
      // For now, we'll simulate the API call
      const response = await simulateAPIResponse(inputMessage, selectedText);

      const botMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: response,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Simulate API response - in real implementation, call the backend
  const simulateAPIResponse = async (message, selectedText) => {
    // Simulate network delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    // In a real implementation, you would call:
    /*
    const response = await fetch('/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        session_id: sessionId,
        message: message,
        selected_text: selectedText
      })
    });

    const data = await response.json();
    return data.response;
    */

    // For this example, return a mock response
    return `I understand you're asking about: "${message}". ${
      selectedText ? `The selected text "${selectedText.substring(0, 50)}..." provides context for your question.` : ''
    } This is a simulated response from the RAG chatbot. In a real implementation, this would be generated using the textbook content and AI.`;
  };

  return (
    <div className="chatbot-container">
      {/* Chatbot toggle button */}
      <button
        className="chatbot-toggle"
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#25c2a0',
          color: 'white',
          border: 'none',
          fontSize: '24px',
          cursor: 'pointer',
          zIndex: 999,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
        }}
      >
        💬
      </button>

      {/* Chatbot window */}
      {isOpen && (
        <div
          className="chatbot-window"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '350px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '8px',
            boxShadow: '0 4px 20px rgba(0,0,0,0.15)',
            zIndex: 998,
            display: 'flex',
            flexDirection: 'column',
            fontFamily: 'sans-serif'
          }}
        >
          {/* Chat header */}
          <div
            className="chat-header"
            style={{
              backgroundColor: '#25c2a0',
              color: 'white',
              padding: '15px',
              borderTopLeftRadius: '8px',
              borderTopRightRadius: '8px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>Textbook Assistant</h3>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '18px',
                cursor: 'pointer'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages container */}
          <div
            className="chat-messages"
            style={{
              flex: 1,
              padding: '15px',
              overflowY: 'auto',
              backgroundColor: '#f9f9f9'
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role}`}
                style={{
                  marginBottom: '15px',
                  textAlign: message.role === 'user' ? 'right' : 'left'
                }}
              >
                <div
                  style={{
                    display: 'inline-block',
                    padding: '10px 15px',
                    borderRadius: '18px',
                    backgroundColor: message.role === 'user' ? '#25c2a0' : '#e9ecef',
                    color: message.role === 'user' ? 'white' : '#333',
                    maxWidth: '80%'
                  }}
                >
                  {message.content}
                </div>
                <div
                  style={{
                    fontSize: '12px',
                    color: '#6c757d',
                    marginTop: '4px',
                    textAlign: 'right'
                  }}
                >
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}
            {isLoading && (
              <div
                className="message assistant"
                style={{
                  marginBottom: '15px',
                  textAlign: 'left'
                }}
              >
                <div
                  style={{
                    display: 'inline-block',
                    padding: '10px 15px',
                    borderRadius: '18px',
                    backgroundColor: '#e9ecef',
                    color: '#333',
                    maxWidth: '80%'
                  }}
                >
                  Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <form
            className="chat-input"
            onSubmit={handleSendMessage}
            style={{
              padding: '15px',
              borderTop: '1px solid #dee2e6',
              backgroundColor: 'white'
            }}
          >
            <div style={{ display: 'flex' }}>
              <input
                type="text"
                value={inputMessage}
                onChange={(e) => setInputMessage(e.target.value)}
                placeholder="Ask about the textbook..."
                style={{
                  flex: 1,
                  padding: '10px',
                  border: '1px solid #ced4da',
                  borderRadius: '20px',
                  outline: 'none'
                }}
              />
              <button
                type="submit"
                disabled={isLoading || !inputMessage.trim()}
                style={{
                  marginLeft: '10px',
                  padding: '10px 15px',
                  backgroundColor: '#25c2a0',
                  color: 'white',
                  border: 'none',
                  borderRadius: '20px',
                  cursor: 'pointer',
                  opacity: isLoading || !inputMessage.trim() ? 0.6 : 1
                }}
              >
                Send
              </button>
            </div>
          </form>
        </div>
      )}
    </div>
  );
};

export default Chatbot;