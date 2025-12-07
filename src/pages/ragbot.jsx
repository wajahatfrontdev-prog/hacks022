import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../../ragbot-ui/ChatInterface';

export default function RagbotPage() {
  // Default API URL (relative) for normal deployments / local dev
  // Guard access to `process` so browsers where `process` is not defined
  // won't throw a runtime error.
  const defaultApi = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL)
    ? process.env.REACT_APP_API_URL
    : '/api';

  // If running inside GitHub Codespaces / preview where the host encodes the port
  // (e.g. "<name>-3000.app.github.dev"), detect and rewrite to the backend port
  // (8000) so the frontend can call the API through the forwarded port.
  let apiUrl = defaultApi;
  if (typeof window !== 'undefined') {
    try {
      const host = window.location.host; // includes port-subdomain like name-3000.app.github.dev
      // Detect the common pattern for GitHub Codespaces preview ("-3000.") and
      // replace it with "-8000." to target the backend port forwarding.
      // Append "/api" so the frontend points at the API prefix (e.g. https://...:8000/api)
      if (host.includes('-3000.')) {
        apiUrl = `${window.location.protocol}//${host.replace('-3000.', '-8000.')}/api`;
      }
    } catch (e) {
      // fallback to defaultApi
      apiUrl = defaultApi;
    }
  }

  return (
    <Layout
      title="Ask the Book"
      description="RAG Chatbot for Physical AI Humanoid Robotics Book"
    >
      <div style={{ height: 'calc(100vh - 60px)', overflow: 'hidden' }}>
        <ChatInterface apiUrl={apiUrl} />
      </div>
    </Layout>
  );
}
