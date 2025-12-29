/**
 * API Configuration for Physical AI RAG Backend
 */

// Backend URL - Update this based on environment
export const API_CONFIG = {
  // Production (Render deployment)
  BACKEND_URL: process.env.REACT_APP_BACKEND_URL || 'https://physical-ai-humanoid-robotics-kafl.onrender.com',

  // API prefix
  API_PREFIX: '/api/v1',

  // Full API endpoints
  ENDPOINTS: {
    HEALTH: '/health',
    SEARCH: '/api/v1/search',
    ASK: '/api/v1/ask',
    ASK_SELECTED: '/api/v1/ask/selected',
    DOCS: '/docs',
  },

  // Request timeout (milliseconds)
  TIMEOUT: 30000, // 30 seconds

  // Retry configuration
  RETRY: {
    attempts: 3,
    delay: 1000, // 1 second
  },
};

// Helper function to build full URL
export function getApiUrl(endpoint) {
  return `${API_CONFIG.BACKEND_URL}${endpoint}`;
}

// Helper function to make API requests
export async function apiRequest(endpoint, options = {}) {
  const url = getApiUrl(endpoint);

  const defaultOptions = {
    headers: {
      'Content-Type': 'application/json',
      ...options.headers,
    },
    timeout: API_CONFIG.TIMEOUT,
  };

  const response = await fetch(url, { ...defaultOptions, ...options });

  if (!response.ok) {
    throw new Error(`API request failed: ${response.statusText}`);
  }

  return response.json();
}

// Example usage:
// import { apiRequest, API_CONFIG } from './config/api';
// const data = await apiRequest(API_CONFIG.ENDPOINTS.ASK, {
//   method: 'POST',
//   body: JSON.stringify({ query: 'What is ROS 2?' })
// });
