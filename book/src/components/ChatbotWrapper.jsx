import React from 'react';
import Chatbot from './Chatbot';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * ChatbotWrapper Component
 *
 * A wrapper that automatically configures the chatbot based on
 * Docusaurus site configuration.
 *
 * Usage in MDX files:
 * ```mdx
 * import ChatbotWrapper from '@site/src/components/ChatbotWrapper';
 *
 * <ChatbotWrapper />
 * ```
 */
const ChatbotWrapper = () => {
  const { siteConfig } = useDocusaurusContext();

  // Get API URL from customFields in docusaurus.config.js
  const apiBaseUrl = siteConfig.customFields?.chatbotApiUrl || 'http://localhost:8000/api/v1';

  return <Chatbot apiBaseUrl={apiBaseUrl} />;
};

export default ChatbotWrapper;
