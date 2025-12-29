import React, { useState, useEffect, useRef } from 'react';
import { apiRequest, API_CONFIG } from '../../config/api';
import './styles.css';

const RAGChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [sessionId] = useState(() => `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`);
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle text selection on the page
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text && text.length > 0) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => document.removeEventListener('mouseup', handleTextSelection);
  }, []);

  // Initialize with welcome message
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([{
        role: 'assistant',
        content: '👋 Hello! I\'m your Physical AI & Humanoid Robotics assistant. I can help you understand concepts from the textbook.\n\n💡 **Tips:**\n- Ask me questions about any topic in the book\n- Select text on the page and click "Ask about selection" to get detailed explanations\n- I use the actual textbook content to provide accurate, grounded answers',
        timestamp: new Date(),
      }]);
    }
  }, []);

  const handleSendMessage = async (messageText = null, isSelectedTextQuery = false) => {
    const queryText = messageText || input.trim();

    if (!queryText) return;

    // Add user message
    const userMessage = {
      role: 'user',
      content: isSelectedTextQuery ? `Explain this: "${queryText}"` : queryText,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Determine which endpoint to use
      const endpoint = isSelectedTextQuery
        ? API_CONFIG.ENDPOINTS.ASK_SELECTED
        : API_CONFIG.ENDPOINTS.ASK;

      const payload = isSelectedTextQuery
        ? {
            query: 'Explain this selection',
            selected_text: queryText,
            selection_metadata: {
              source: 'Physical AI textbook',
              session_id: sessionId,
            },
          }
        : {
            query: queryText,
            session_id: sessionId,
          };

      const response = await apiRequest(endpoint, {
        method: 'POST',
        body: JSON.stringify(payload),
      });

      // Add assistant response
      const assistantMessage = {
        role: 'assistant',
        content: response.answer || response.explanation || 'I received your question but couldn\'t generate a response.',
        sources: response.sources || [],
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selection after use

    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        role: 'assistant',
        content: '❌ Sorry, I encountered an error. Please make sure the backend is running and the Google API key is configured.\n\nError: ' + error.message,
        timestamp: new Date(),
        isError: true,
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleAskAboutSelection = () => {
    if (selectedText) {
      handleSendMessage(selectedText, true);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const clearChat = () => {
    setMessages([{
      role: 'assistant',
      content: '👋 Chat cleared! How can I help you with Physical AI & Humanoid Robotics?',
      timestamp: new Date(),
    }]);
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`rag-chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={toggleChat}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Selected Text Action Button */}
      {selectedText && !isOpen && (
        <div className="rag-chatbot-selection-prompt">
          <button
            className="rag-chatbot-selection-btn"
            onClick={() => {
              setIsOpen(true);
              setTimeout(() => handleAskAboutSelection(), 300);
            }}
          >
            💡 Ask about selection
          </button>
        </div>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className="rag-chatbot-container">
          <div className="rag-chatbot-header">
            <div className="rag-chatbot-header-title">
              <span className="rag-chatbot-icon">🤖</span>
              <div>
                <h3>Physical AI Assistant</h3>
                <span className="rag-chatbot-status">Powered by RAG</span>
              </div>
            </div>
            <div className="rag-chatbot-header-actions">
              <button
                className="rag-chatbot-header-btn"
                onClick={clearChat}
                title="Clear chat"
              >
                🗑️
              </button>
              <button
                className="rag-chatbot-header-btn"
                onClick={toggleChat}
                title="Close chat"
              >
                ✕
              </button>
            </div>
          </div>

          <div className="rag-chatbot-messages">
            {messages.map((message, index) => (
              <div
                key={index}
                className={`rag-chatbot-message ${message.role} ${message.isError ? 'error' : ''}`}
              >
                <div className="rag-chatbot-message-content">
                  {message.content}
                </div>
                {message.sources && message.sources.length > 0 && (
                  <div className="rag-chatbot-sources">
                    <strong>📚 Sources:</strong>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx}>
                          {source.chapter_title || source.section_title || `Source ${idx + 1}`}
                          {source.similarity_score && (
                            <span className="similarity-score">
                              ({(source.similarity_score * 100).toFixed(0)}% relevant)
                            </span>
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
                <div className="rag-chatbot-message-time">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="rag-chatbot-message assistant">
                <div className="rag-chatbot-message-content">
                  <div className="rag-chatbot-typing">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className="rag-chatbot-selection-bar">
              <span className="rag-chatbot-selection-text">
                "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
              </span>
              <button
                className="rag-chatbot-selection-bar-btn"
                onClick={handleAskAboutSelection}
                disabled={isLoading}
              >
                Ask about this
              </button>
            </div>
          )}

          <div className="rag-chatbot-input-container">
            <textarea
              ref={textareaRef}
              className="rag-chatbot-input"
              placeholder="Ask a question about Physical AI..."
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
              rows={1}
            />
            <button
              className="rag-chatbot-send-btn"
              onClick={() => handleSendMessage()}
              disabled={isLoading || !input.trim()}
            >
              {isLoading ? '⏳' : '📤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default RAGChatbot;
