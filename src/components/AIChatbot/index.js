import React, { useState, useEffect } from 'react';

const AIChatbot = () => {
  const [colorMode, setColorMode] = useState('light'); // Default to light mode
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Get color mode from document class or default to light
  useEffect(() => {
    const updateColorMode = () => {
      const isDarkMode = document.documentElement.classList.contains('dark') ||
        (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches);
      setColorMode(isDarkMode ? 'dark' : 'light');
    };

    // Initial check
    updateColorMode();

    // Listen for changes to dark mode preference
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    const handleChange = () => updateColorMode();

    mediaQuery.addEventListener('change', handleChange);

    // Also check for Docusaurus theme changes by monitoring class changes
    const observer = new MutationObserver(updateColorMode);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['class']
    });

    return () => {
      mediaQuery.removeEventListener('change', handleChange);
      observer.disconnect();
    };
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim()) return;

    const userMessage = { role: 'user', content: inputValue };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/api/v1/agent/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          selectedText: '', // No selected text in this basic version
          topK: 5
        })
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Create bot response
      const botResponse = {
        role: 'assistant',
        content: data.response
      };

      setMessages(prev => [...prev, botResponse]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        role: 'assistant',
        content: `Sorry, I encountered an error processing your request: ${error.message}. Please try again.`
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      sendMessage();
    }
  };

  return (
    <div className={`chatbot-container ${isOpen ? 'open' : ''}`}>
      {!isOpen ? (
        <button
          className="chatbot-toggle-button"
          onClick={toggleChat}
          aria-label="Open AI Chatbot"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.32L2 22L7.68 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM8 15V13H10V15H8ZM12 15V13H14V15H12ZM16 15V13H18V15H16Z" fill="currentColor"/>
          </svg>
        </button>
      ) : (
        <div className={`chatbot-window ${colorMode}`} onClick={(e) => e.stopPropagation()}>
          <div className="chatbot-header">
            <h3>AI Assistant</h3>
            <button className="close-button" onClick={toggleChat}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M19 6.41L17.59 5L12 10.59L6.41 5L5 6.41L10.59 12L5 17.59L6.41 19L12 13.41L17.59 19L19 17.59L13.41 12L19 6.41Z" fill="currentColor"/>
              </svg>
            </button>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.</p>
                <p>Ask me anything about the content, and I'll search the textbook to provide accurate answers.</p>
              </div>
            ) : (
              messages.map((msg, index) => (
                <div key={index} className={`message ${msg.role}`}>
                  <div className="message-content">{msg.content}</div>
                </div>
              ))
            )}

            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <span className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </span>
                </div>
              </div>
            )}
          </div>

          <div className="chatbot-input">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about Physical AI or Humanoid Robotics..."
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={!inputValue.trim() || isLoading}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M2.01 21L23 12L2.01 3L2 10L17 12L2 14L2.01 21Z" fill="currentColor"/>
              </svg>
            </button>
          </div>
        </div>
      )}

      <style jsx>{`
        .chatbot-container {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 1000;
        }

        .chatbot-toggle-button {
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background-color: rgba(37, 193, 189, 0.6);
          color: white;
          border: none;
          cursor: pointer;
          display: flex;
          align-items: center;
          justify-content: center;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
          transition: all 0.3s ease;
        }

        .chatbot-toggle-button:hover {
          transform: scale(1.05);
          box-shadow: 0 6px 16px rgba(0, 0, 0, 0.2);
        }

        .chatbot-window {
          width: 350px;
          height: 500px;
          background-color: ${colorMode === 'dark' ? '#1a1a1a' : 'white'};
          border-radius: 12px;
          display: flex;
          flex-direction: column;
          box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
          overflow: hidden;
          border: 1px solid ${colorMode === 'dark' ? '#555' : '#ddd'};
        }

        .chatbot-header {
          background-color: rgba(37, 193, 189, 0.6);
          color: white;
          padding: 15px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .chatbot-header h3 {
          margin: 0;
          font-size: 16px;
        }

        .close-button {
          background: none;
          border: none;
          color: white;
          cursor: pointer;
          padding: 4px;
        }

        .chatbot-messages {
          flex: 1;
          padding: 15px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 12px;
          background-color: ${colorMode === 'dark' ? '#121212' : '#fafafa'};
        }

        .welcome-message {
          color: ${colorMode === 'dark' ? '#cccccc' : '#666'};
          font-style: italic;
        }

        .message {
          max-width: 80%;
          padding: 10px 14px;
          border-radius: 18px;
          line-height: 1.4;
        }

        .message.user {
          align-self: flex-end;
          background-color: rgba(37, 193, 189, 0.6);
          color: white;
          border-bottom-right-radius: 4px;
        }

        .message.assistant {
          align-self: flex-start;
          background-color: ${colorMode === 'dark' ? '#2a2a2a' : '#e9ecef'};
          color: ${colorMode === 'dark' ? '#ffffff' : '#333'};
          border-bottom-left-radius: 4px;
        }

        .typing-indicator {
          display: inline-flex;
          align-items: center;
        }

        .typing-indicator span {
          width: 8px;
          height: 8px;
          background-color: ${colorMode === 'dark' ? '#cccccc' : '#666'};
          border-radius: 50%;
          margin: 0 2px;
          animation: typing 1.4s infinite ease-in-out;
        }

        .typing-indicator span:nth-child(1) {
          animation-delay: 0s;
        }

        .typing-indicator span:nth-child(2) {
          animation-delay: 0.2s;
        }

        .typing-indicator span:nth-child(3) {
          animation-delay: 0.4s;
        }

        @keyframes typing {
          0%, 60%, 100% {
            transform: translateY(0);
          }
          30% {
            transform: translateY(-5px);
          }
        }

        .chatbot-input {
          padding: 15px;
          background-color: ${colorMode === 'dark' ? '#1a1a1a' : 'white'};
          border-top: 1px solid ${colorMode === 'dark' ? '#555' : '#ddd'};
          display: flex;
          gap: 8px;
        }

        .chatbot-input input {
          flex: 1;
          padding: 12px 15px;
          border: 1px solid ${colorMode === 'dark' ? '#555' : '#ddd'};
          border-radius: 24px;
          outline: none;
          background-color: ${colorMode === 'dark' ? '#2a2a2a' : 'white'};
          color: ${colorMode === 'dark' ? '#ffffff' : 'black'};
        }

        .chatbot-input input:focus {
          border-color: var(--ifm-color-primary);
        }

        .chatbot-input button {
          width: 44px;
          height: 44px;
          border-radius: 50%;
          border: none;
          background-color: var(--ifm-color-primary);
          color: white;
          cursor: pointer;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .chatbot-input button:disabled {
          background-color: #cccccc;
          cursor: not-allowed;
        }
      `}</style>
    </div>
  );
};

export default AIChatbot;