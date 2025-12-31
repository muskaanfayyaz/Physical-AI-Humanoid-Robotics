# Chatbot Architecture & Implementation

**Document**: RAG Chatbot Technical Architecture
**Project**: Physical AI & Humanoid Robotics Textbook
**Last Updated**: January 2026

---

## Table of Contents

1. [Overview](#overview)
2. [Frontend Architecture](#frontend-architecture)
3. [Component Design](#component-design)
4. [State Management](#state-management)
5. [API Integration](#api-integration)
6. [User Experience](#user-experience)
7. [Styling & Themes](#styling--themes)
8. [Performance Optimization](#performance-optimization)
9. [Error Handling](#error-handling)
10. [Accessibility](#accessibility)

---

## Overview

### Chatbot Purpose

The RAG chatbot provides an interactive Q&A interface for students using the Physical AI & Humanoid Robotics textbook.

**Key Features**:
- ✅ Context-aware answers from textbook content
- ✅ Conversation history tracking
- ✅ Source citations
- ✅ Real-time response
- ✅ Expandable UI (doesn't obstruct reading)
- ✅ Mobile-responsive design

### Technology Stack

**Frontend**:
- React 18.2+
- JavaScript (ES6+)
- CSS Modules
- Docusaurus 3.1+ integration

**Backend Integration**:
- REST API (FastAPI)
- JSON communication
- CORS-enabled

---

## Frontend Architecture

### High-Level Structure

```
┌──────────────────────────────────────────────────────────┐
│                    Docusaurus App                         │
│  ┌────────────────────────────────────────────────────┐  │
│  │                   Layout                            │  │
│  │  ┌──────────┐  ┌─────────────┐  ┌──────────────┐  │  │
│  │  │  Navbar  │  │   Content   │  │   Sidebar    │  │  │
│  │  └──────────┘  │  (Chapters) │  └──────────────┘  │  │
│  │                └─────────────┘                     │  │
│  │                                                     │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │          Chatbot Component                    │  │  │
│  │  │  (Floating, position: fixed)                 │  │  │
│  │  │                                               │  │  │
│  │  │  ┌─────────────────────────────────────────┐ │  │  │
│  │  │  │  ChatbotWrapper (Root Theme)           │ │  │  │
│  │  │  │    └── Chatbot                         │ │  │  │
│  │  │  │          ├── ChatbotToggle             │ │  │  │
│  │  │  │          └── ChatbotWindow             │ │  │  │
│  │  │  │                ├── MessageList         │ │  │  │
│  │  │  │                ├── InputBox            │ │  │  │
│  │  │  │                └── SourceReferences    │ │  │  │
│  │  │  └─────────────────────────────────────────┘ │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

### File Structure

```
book/
├── src/
│   ├── components/
│   │   ├── Chatbot.jsx                  # Main chatbot component
│   │   ├── Chatbot.module.css           # Chatbot styles
│   │   └── ChatbotWrapper.jsx           # Wrapper for Root integration
│   ├── theme/
│   │   └── Root.js                      # Theme Root (injects chatbot)
│   └── config/
│       └── api.js                       # API configuration
└── docusaurus.config.ts                 # Docusaurus config
```

---

## Component Design

### 1. Chatbot.jsx (Main Component)

**Responsibility**: Core chatbot logic and UI

**Component Structure**:
```jsx
import React, { useState, useEffect, useRef } from 'react';
import styles from './Chatbot.module.css';

export default function Chatbot() {
  // State management
  const [messages, setMessages] = useState([]);
  const [conversationId, setConversationId] = useState(null);
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [inputValue, setInputValue] = useState('');

  // Refs
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Effects
  useEffect(() => {
    // Auto-scroll to bottom on new messages
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  useEffect(() => {
    // Focus input when opened
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Event handlers
  const handleToggle = () => setIsOpen(!isOpen);
  const handleSendMessage = async () => { /* ... */ };
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className={styles.chatbot}>
      {/* Floating button */}
      {!isOpen && (
        <button
          className={styles.toggleButton}
          onClick={handleToggle}
          aria-label="Open chatbot"
        >
          💬 Ask AI
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <h3>AI Assistant</h3>
            <button onClick={handleToggle}>✕</button>
          </div>

          {/* Messages */}
          <div className={styles.messageList}>
            {messages.map((msg, idx) => (
              <Message key={idx} message={msg} />
            ))}
            {isLoading && <TypingIndicator />}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputBox}>
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
            />
            <button onClick={handleSendMessage}>Send</button>
          </div>

          {/* Error display */}
          {error && (
            <div className={styles.error}>{error}</div>
          )}
        </div>
      )}
    </div>
  );
}
```

---

### 2. Message Component

**Sub-component for rendering messages**:

```jsx
function Message({ message }) {
  const isUser = message.role === 'user';

  return (
    <div className={isUser ? styles.userMessage : styles.botMessage}>
      <div className={styles.messageContent}>
        {message.content}
      </div>

      {/* Show sources for bot messages */}
      {!isUser && message.sources && (
        <div className={styles.sources}>
          <strong>Sources:</strong>
          {message.sources.map((source, idx) => (
            <div key={idx} className={styles.source}>
              📖 {source.chapter}
            </div>
          ))}
        </div>
      )}

      <div className={styles.timestamp}>
        {formatTime(message.timestamp)}
      </div>
    </div>
  );
}
```

---

### 3. TypingIndicator Component

**Shows bot is "thinking"**:

```jsx
function TypingIndicator() {
  return (
    <div className={styles.typingIndicator}>
      <span></span>
      <span></span>
      <span></span>
    </div>
  );
}
```

**CSS Animation**:
```css
.typingIndicator span {
  animation: blink 1.4s infinite;
}

.typingIndicator span:nth-child(2) {
  animation-delay: 0.2s;
}

.typingIndicator span:nth-child(3) {
  animation-delay: 0.4s;
}

@keyframes blink {
  0%, 60%, 100% { opacity: 0; }
  30% { opacity: 1; }
}
```

---

### 4. ChatbotWrapper Component

**Integrates chatbot into Docusaurus Root**:

```jsx
// src/theme/Root.js
import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
```

---

## State Management

### State Variables

```jsx
// Message history
const [messages, setMessages] = useState([]);
// Format: [
//   { role: 'user', content: '...', timestamp: Date },
//   { role: 'assistant', content: '...', sources: [...], timestamp: Date }
// ]

// Conversation tracking
const [conversationId, setConversationId] = useState(null);
// Unique ID for this conversation session

// UI state
const [isOpen, setIsOpen] = useState(false);
const [isLoading, setIsLoading] = useState(false);
const [error, setError] = useState(null);

// Input state
const [inputValue, setInputValue] = useState('');
```

### State Updates

**Adding a message**:
```jsx
const addMessage = (role, content, sources = null) => {
  const newMessage = {
    role,
    content,
    sources,
    timestamp: new Date()
  };

  setMessages(prev => [...prev, newMessage]);
};
```

**Handling errors**:
```jsx
const setErrorMessage = (message) => {
  setError(message);
  setTimeout(() => setError(null), 5000); // Auto-clear after 5s
};
```

---

## API Integration

### API Configuration

**File**: `src/config/api.js`

```javascript
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://physical-ai-humanoid-robotics.onrender.com'
  : 'http://localhost:8000';

export const API_ENDPOINTS = {
  ask: `${API_URL}/api/v1/ask`,
  search: `${API_URL}/api/v1/search`,
  conversation: `${API_URL}/api/v1/conversations`
};
```

### API Call Implementation

**Ask Endpoint**:
```jsx
const handleSendMessage = async () => {
  if (!inputValue.trim()) return;

  const userQuestion = inputValue.trim();

  // Add user message to UI immediately
  addMessage('user', userQuestion);
  setInputValue('');
  setIsLoading(true);
  setError(null);

  try {
    // Call API
    const response = await fetch(API_ENDPOINTS.ask, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question: userQuestion,
        conversation_id: conversationId,
        top_k: 5,
        include_sources: true
      })
    });

    if (!response.ok) {
      throw new Error(`API error: ${response.status}`);
    }

    const data = await response.json();

    // Add bot response to UI
    addMessage('assistant', data.answer, data.sources);

    // Store conversation ID for follow-ups
    if (data.conversation_id && !conversationId) {
      setConversationId(data.conversation_id);
    }

  } catch (error) {
    console.error('Chatbot error:', error);
    setErrorMessage('Failed to get response. Please try again.');
  } finally {
    setIsLoading(false);
  }
};
```

### Error Handling

**Network Errors**:
```jsx
try {
  const response = await fetch(url, options);
  // ...
} catch (error) {
  if (error.name === 'TypeError') {
    // Network error
    setErrorMessage('Network error. Check your connection.');
  } else {
    setErrorMessage('An unexpected error occurred.');
  }
}
```

**API Errors**:
```jsx
if (!response.ok) {
  if (response.status === 429) {
    setErrorMessage('Too many requests. Please wait a moment.');
  } else if (response.status === 500) {
    setErrorMessage('Server error. Please try again later.');
  } else {
    setErrorMessage(`Error: ${response.statusText}`);
  }
  return;
}
```

---

## User Experience

### Conversation Flow

```
1. User clicks "Ask AI" button
   ↓
2. Chat window slides up from bottom
   ↓
3. User types question and presses Enter
   ↓
4. User message appears immediately (optimistic UI)
   ↓
5. Typing indicator shows bot is "thinking"
   ↓
6. Bot response appears with sources
   ↓
7. User can ask follow-up (conversation context maintained)
   ↓
8. User clicks X to close (conversation persists)
```

### UX Principles

1. **Immediate Feedback**
   - User message appears instantly
   - Typing indicator during processing
   - Error messages are clear and actionable

2. **Non-Intrusive**
   - Floating button in corner
   - Doesn't block content
   - Easy to dismiss

3. **Contextual**
   - Maintains conversation history
   - Shows source citations
   - Clear timestamping

4. **Accessible**
   - Keyboard navigation
   - ARIA labels
   - Focus management

---

## Styling & Themes

### CSS Modules

**File**: `Chatbot.module.css`

**Key Styles**:

```css
/* Floating button */
.toggleButton {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;

  padding: 12px 20px;
  border-radius: 25px;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border: none;
  box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);

  cursor: pointer;
  transition: transform 0.2s, box-shadow 0.2s;
}

.toggleButton:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(0, 0, 0, 0.3);
}

/* Chat window */
.chatWindow {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;

  width: 400px;
  max-width: calc(100vw - 40px);
  height: 600px;
  max-height: calc(100vh - 100px);

  background: white;
  border-radius: 12px;
  box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2);

  display: flex;
  flex-direction: column;

  animation: slideUp 0.3s ease-out;
}

@keyframes slideUp {
  from {
    transform: translateY(100%);
    opacity: 0;
  }
  to {
    transform: translateY(0);
    opacity: 1;
  }
}

/* Message list */
.messageList {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  background: #f9fafb;
}

/* Messages */
.userMessage {
  display: flex;
  justify-content: flex-end;
  margin-bottom: 12px;
}

.userMessage .messageContent {
  background: #667eea;
  color: white;
  padding: 10px 14px;
  border-radius: 18px 18px 4px 18px;
  max-width: 80%;
}

.botMessage {
  display: flex;
  justify-content: flex-start;
  margin-bottom: 12px;
}

.botMessage .messageContent {
  background: white;
  color: #1f2937;
  padding: 10px 14px;
  border-radius: 18px 18px 18px 4px;
  max-width: 80%;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.1);
}

/* Sources */
.sources {
  margin-top: 8px;
  padding: 8px;
  background: #f3f4f6;
  border-radius: 8px;
  font-size: 0.85em;
}

.source {
  margin-top: 4px;
  color: #6b7280;
}

/* Input box */
.inputBox {
  display: flex;
  padding: 12px;
  background: white;
  border-top: 1px solid #e5e7eb;
  border-radius: 0 0 12px 12px;
}

.inputBox textarea {
  flex: 1;
  padding: 8px 12px;
  border: 1px solid #d1d5db;
  border-radius: 20px;
  resize: none;
  font-family: inherit;
  font-size: 14px;
}

.inputBox button {
  margin-left: 8px;
  padding: 8px 16px;
  background: #667eea;
  color: white;
  border: none;
  border-radius: 20px;
  cursor: pointer;
  transition: background 0.2s;
}

.inputBox button:hover {
  background: #5568d3;
}
```

### Dark Mode Support

```css
[data-theme='dark'] .chatWindow {
  background: #1f2937;
  color: #f9fafb;
}

[data-theme='dark'] .messageList {
  background: #111827;
}

[data-theme='dark'] .botMessage .messageContent {
  background: #374151;
  color: #f9fafb;
}
```

### Mobile Responsive

```css
@media (max-width: 768px) {
  .chatWindow {
    width: calc(100vw - 20px);
    height: calc(100vh - 80px);
    bottom: 10px;
    right: 10px;
  }

  .toggleButton {
    bottom: 10px;
    right: 10px;
  }
}
```

---

## Performance Optimization

### Lazy Loading

```jsx
// Only load chatbot when user interacts
const [chatbotLoaded, setChatbotLoaded] = useState(false);

const handleFirstOpen = () => {
  if (!chatbotLoaded) {
    setChatbotLoaded(true);
  }
  setIsOpen(true);
};
```

### Memoization

```jsx
import { useMemo } from 'react';

const Message = React.memo(({ message }) => {
  // Component only re-renders if message changes
  return <div>{message.content}</div>;
});
```

### Debouncing (future)

```jsx
// Debounce typing indicator for search-as-you-type
const debouncedSearch = useMemo(
  () => debounce((query) => {
    // Perform search
  }, 300),
  []
);
```

---

## Error Handling

### User-Friendly Messages

```jsx
const ERROR_MESSAGES = {
  network: 'Network error. Please check your connection and try again.',
  timeout: 'Request timed out. Please try again.',
  rateLimit: 'Too many requests. Please wait a moment.',
  server: 'Server error. Our team has been notified.',
  unknown: 'Something went wrong. Please try again.'
};
```

### Retry Logic

```jsx
const fetchWithRetry = async (url, options, retries = 2) => {
  try {
    return await fetch(url, options);
  } catch (error) {
    if (retries > 0) {
      await new Promise(resolve => setTimeout(resolve, 1000));
      return fetchWithRetry(url, options, retries - 1);
    }
    throw error;
  }
};
```

---

## Accessibility

### ARIA Labels

```jsx
<button
  className={styles.toggleButton}
  onClick={handleToggle}
  aria-label="Open AI chatbot assistant"
  aria-expanded={isOpen}
>
  💬 Ask AI
</button>
```

### Keyboard Navigation

```jsx
// Enter to send, Shift+Enter for new line
const handleKeyPress = (e) => {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    handleSendMessage();
  }
};

// Escape to close
useEffect(() => {
  const handleEscape = (e) => {
    if (e.key === 'Escape' && isOpen) {
      setIsOpen(false);
    }
  };

  document.addEventListener('keydown', handleEscape);
  return () => document.removeEventListener('keydown', handleEscape);
}, [isOpen]);
```

### Focus Management

```jsx
useEffect(() => {
  // Focus input when opened
  if (isOpen && inputRef.current) {
    inputRef.current.focus();
  }
}, [isOpen]);
```

---

## Future Enhancements

### Planned Features

1. **Voice Input**
   - Speech-to-text for questions
   - Text-to-speech for answers

2. **Rich Media**
   - Display images from textbook
   - Render code snippets with syntax highlighting
   - Show diagrams and charts

3. **Conversation Export**
   - Download chat history as PDF
   - Share conversations

4. **Search History**
   - Show recent questions
   - Quick re-ask feature

5. **Feedback System**
   - Rate answers (👍/👎)
   - Report issues
   - Suggest improvements

---

## Metrics

### Performance Targets

- **Initial Load**: <100ms
- **Message Send**: <50ms (optimistic UI)
- **API Response**: <2s (backend RAG)
- **Total UX**: <2.5s from question to answer

### Usage Tracking

```jsx
// Track chatbot usage (future)
const trackChatbotEvent = (event, data) => {
  // Analytics implementation
  console.log('Chatbot event:', event, data);
};

// Track opens
trackChatbotEvent('open', { timestamp: new Date() });

// Track questions
trackChatbotEvent('question', {
  length: question.length,
  timestamp: new Date()
});
```

---

## Conclusion

The chatbot provides a seamless, intuitive interface for students to interact with the textbook content through natural language questions.

**Key Strengths**:
- ✅ Clean, modern UI
- ✅ Fast, responsive
- ✅ Mobile-friendly
- ✅ Accessible
- ✅ Well-integrated with Docusaurus

**Future Improvements**:
- Voice interaction
- Rich media support
- Enhanced analytics
- Personalization

---

**Document Status**: ✅ Complete
**Last Updated**: January 2026
**Next Review**: February 2026
