import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../../ragbot-ui/ChatInterface';
import { AuthProvider } from '../components/AuthProvider';
import { ProtectedRoute } from '../components/ProtectedRoute';
import { useAuth } from '../components/AuthProvider';

function RagbotContent() {
  const { user, signOut } = useAuth();
  const apiUrl = 'https://hacks022-backend.vercel.app';

  return (
    <Layout
      title="Ask the Book"
      description="RAG Chatbot for Physical AI Humanoid Robotics Book"
    >
      <div style={{ 
        position: 'absolute',
        top: '10px',
        right: '20px',
        zIndex: 1000,
        background: 'rgba(255,255,255,0.9)',
        padding: '8px 16px',
        borderRadius: '20px',
        fontSize: '14px',
        fontWeight: '600',
        color: '#667eea'
      }}>
        👋 {user?.name || user?.email} 
        <button 
          onClick={signOut}
          style={{
            marginLeft: '10px',
            background: '#ff6b6b',
            color: 'white',
            border: 'none',
            padding: '4px 12px',
            borderRadius: '12px',
            cursor: 'pointer',
            fontSize: '12px'
          }}
        >
          Sign Out
        </button>
      </div>
      <div style={{ height: 'calc(100vh - 60px)', overflow: 'hidden' }}>
        <ChatInterface apiUrl={apiUrl} />
      </div>
    </Layout>
  );
}

export default function RagbotPage() {
  return (
    <AuthProvider>
      <ProtectedRoute>
        <RagbotContent />
      </ProtectedRoute>
    </AuthProvider>
  );
}
