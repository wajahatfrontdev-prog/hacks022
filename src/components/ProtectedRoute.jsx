import React from 'react';
import { useAuth } from './AuthProvider';

export function ProtectedRoute({ children }) {
  const { user, loading } = useAuth();

  if (loading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        height: '100vh',
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        color: 'white',
        fontSize: '18px',
        fontWeight: '600'
      }}>
        🔄 Loading...
      </div>
    );
  }

  if (!user) {
    window.location.href = '/auth/signin';
    return null;
  }

  return children;
}