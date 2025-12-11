import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './auth.module.css';

export default function DemoAuth() {
  const [loading, setLoading] = useState(false);

  const handleDemoLogin = () => {
    setLoading(true);
    // Simulate login and redirect
    setTimeout(() => {
      localStorage.setItem('demo_user', JSON.stringify({
        name: 'Demo User',
        email: 'demo@example.com'
      }));
      window.location.href = '/ragbot';
    }, 1000);
  };

  return (
    <Layout title="Demo Access">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>🚀 Quick Demo</h1>
          <p className={styles.authSubtitle}>Try the RAG Chatbot instantly</p>

          <button 
            onClick={handleDemoLogin}
            disabled={loading}
            className={styles.authButton}
            style={{ marginTop: '20px' }}
          >
            {loading ? '⏳ Loading...' : '🎯 Try Demo Now'}
          </button>

          <div className={styles.authLinks}>
            <p>Want full features? <a href="/auth/signup">Create Account</a></p>
            <p>Already have account? <a href="/auth/signin">Sign In</a></p>
          </div>
        </div>
      </div>
    </Layout>
  );
}