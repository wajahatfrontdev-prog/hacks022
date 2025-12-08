import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../../ragbot-ui/ChatInterface';

export default function RagbotPage() {
  const apiUrl = 'https://hacks022-backend.vercel.app'

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
