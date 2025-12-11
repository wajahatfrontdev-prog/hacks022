import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './auth.module.css';

export default function SignIn() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const users = JSON.parse(localStorage.getItem('users') || '[]');
      const user = users.find(u => u.email === email && u.password === password);
      
      if (user) {
        localStorage.setItem('current_user', JSON.stringify(user));
        window.location.href = '/ragbot';
      } else {
        setError('Invalid email or password');
      }
    } catch (err) {
      setError('Sign in failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign In">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>🔐 Sign In</h1>
          <p className={styles.authSubtitle}>Access your RAG Chatbot</p>

          {error && <div className={styles.error}>{error}</div>}

          <form onSubmit={handleSubmit} className={styles.authForm}>
            <div className={styles.inputGroup}>
              <label>Email</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                className={styles.input}
                placeholder="your@email.com"
              />
            </div>

            <div className={styles.inputGroup}>
              <label>Password</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                className={styles.input}
                placeholder="••••••••"
              />
            </div>

            <button 
              type="submit" 
              disabled={loading}
              className={styles.authButton}
            >
              {loading ? '⏳ Signing In...' : '🚀 Sign In'}
            </button>
          </form>

          <div className={styles.authLinks}>
            <p>Don't have an account? <a href="/auth/signup">Sign Up</a></p>
          </div>

          <div className={styles.socialAuth}>
            <p>Or continue with</p>
            <div className={styles.socialButtons}>
              <button className={styles.socialButton}>
                🐙 GitHub
              </button>
              <button className={styles.socialButton}>
                🌐 Google
              </button>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}