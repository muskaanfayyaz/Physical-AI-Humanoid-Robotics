import React from 'react';
import RAGChatbot from '../components/RAGChatbot';

// Wraps the entire application to add global components
export default function Root({ children }) {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
