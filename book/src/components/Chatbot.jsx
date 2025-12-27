import React, { useState, useRef, useEffect } from 'react';
import styles from './Chatbot.module.css';

/**
 * Chatbot Component for Physical AI Textbook
 *
 * Features:
 * - Normal mode: Ask questions with RAG retrieval
 * - Selected text mode: Ask about specific selections
 * - Session-based conversation history
 * - Source attribution with links
 * - Dark mode support
 * - Responsive design
 */
const Chatbot = ({ apiBaseUrl = 'http://localhost:8000/api/v1' }) => {
  // State management
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [mode, setMode] = useState('normal'); // 'normal' or 'selected'
  const [selectedText, setSelectedText] = useState('');
  const [sessionId] = useState(() => `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`);
  const [error, setError] = useState(null);

  // Refs
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Detect text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 10 && text.length < 5000) {
        setSelectedText(text);
        setMode('selected');
        setIsOpen(true);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Focus input when opened
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Send message to API
  const sendMessage = async (question) => {
    if (!question.trim()) return;

    // Add user message
    const userMessage = {
      role: 'user',
      content: question,
      timestamp: new Date().toISOString(),
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      let response;

      if (mode === 'selected' && selectedText) {
        // Use /ask/selected endpoint
        response = await fetch(`${apiBaseUrl}/ask/selected`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            query: question,
            selected_text: selectedText,
            selection_metadata: {
              timestamp: new Date().toISOString(),
            },
          }),
        });
      } else {
        // Use /ask endpoint
        response = await fetch(`${apiBaseUrl}/ask/`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            query: question,
            top_k: 5,
            session_id: sessionId,
            include_history: true,
          }),
        });
      }

      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources || [],
        metadata: {
          model: data.model,
          tokens_used: data.tokens_used,
          processing_time: data.processing_time_seconds,
          chunks_retrieved: data.chunks_retrieved,
        },
        timestamp: new Date().toISOString(),
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Clear selected text after question
      if (mode === 'selected') {
        setSelectedText('');
        setMode('normal');
      }
    } catch (err) {
      console.error('Chatbot error:', err);
      setError(err.message);

      // Add error message
      const errorMessage = {
        role: 'assistant',
        content: `Sorry, I encountered an error: ${err.message}. Please try again or check if the backend is running.`,
        isError: true,
        timestamp: new Date().toISOString(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle form submit
  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage(inputValue);
  };

  // Clear conversation
  const clearConversation = () => {
    setMessages([]);
    setError(null);
  };

  // Toggle chatbot
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Floating button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.active : ''}`}
        onClick={toggleChat}
        aria-label="Toggle chatbot"
        title="Ask questions about this textbook"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <h3>🤖 Physical AI Assistant</h3>
              <div className={styles.modeIndicator}>
                {mode === 'selected' && (
                  <span className={styles.badge}>
                    📝 Selected Text Mode
                  </span>
                )}
              </div>
            </div>
            <div className={styles.headerActions}>
              <button
                onClick={clearConversation}
                className={styles.clearButton}
                title="Clear conversation"
              >
                🗑️
              </button>
              <button
                onClick={toggleChat}
                className={styles.closeButton}
                title="Close"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 <strong>Welcome!</strong></p>
                <p>Ask me anything about the Physical AI & Humanoid Robotics textbook.</p>
                <div className={styles.tips}>
                  <p><strong>💡 Tips:</strong></p>
                  <ul>
                    <li>Select text on the page to ask about specific sections</li>
                    <li>I only answer based on the textbook content</li>
                    <li>Try: "What is ROS 2?" or "Explain bipedal locomotion"</li>
                  </ul>
                </div>
              </div>
            )}

            {messages.map((msg, index) => (
              <div
                key={index}
                className={`${styles.message} ${styles[msg.role]} ${msg.isError ? styles.error : ''}`}
              >
                <div className={styles.messageContent}>
                  {msg.role === 'assistant' && <span className={styles.avatar}>🤖</span>}
                  {msg.role === 'user' && <span className={styles.avatar}>👤</span>}

                  <div className={styles.messageBody}>
                    <div className={styles.messageText}>
                      {msg.content}
                    </div>

                    {/* Sources */}
                    {msg.sources && msg.sources.length > 0 && (
                      <div className={styles.sources}>
                        <strong>📚 Sources:</strong>
                        <ul>
                          {msg.sources.map((source, idx) => (
                            <li key={idx}>
                              <a
                                href={`#${source.chunk_id}`}
                                className={styles.sourceLink}
                              >
                                {source.heading}
                              </a>
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}

                    {/* Metadata */}
                    {msg.metadata && (
                      <div className={styles.metadata}>
                        <small>
                          {msg.metadata.chunks_retrieved && (
                            <span>📊 {msg.metadata.chunks_retrieved} chunks retrieved • </span>
                          )}
                          <span>⏱️ {msg.metadata.processing_time?.toFixed(2)}s</span>
                        </small>
                      </div>
                    )}
                  </div>
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <span className={styles.avatar}>🤖</span>
                  <div className={styles.loadingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Selected text preview */}
          {mode === 'selected' && selectedText && (
            <div className={styles.selectedTextPreview}>
              <div className={styles.previewHeader}>
                <strong>📝 Selected text:</strong>
                <button
                  onClick={() => {
                    setSelectedText('');
                    setMode('normal');
                  }}
                  className={styles.clearSelection}
                >
                  ✕
                </button>
              </div>
              <div className={styles.previewContent}>
                {selectedText.substring(0, 150)}
                {selectedText.length > 150 && '...'}
              </div>
            </div>
          )}

          {/* Input form */}
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={
                mode === 'selected'
                  ? 'Ask about the selected text...'
                  : 'Ask a question...'
              }
              disabled={isLoading}
              className={styles.input}
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              className={styles.sendButton}
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>

          {/* Error banner */}
          {error && (
            <div className={styles.errorBanner}>
              ⚠️ Backend connection failed. Make sure the API is running at {apiBaseUrl}
            </div>
          )}
        </div>
      )}
    </>
  );
};

export default Chatbot;
