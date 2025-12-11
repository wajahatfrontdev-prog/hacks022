import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './auth.module.css';

export default function SignUp() {
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    confirmPassword: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      setLoading(false);
      return;
    }

    try {
      // Simple local storage based auth for demo
      const users = JSON.parse(localStorage.getItem('users') || '[]');
      
      if (users.find(u => u.email === formData.email)) {
        setError('Email already exists');
        return;
      }
      
      const newUser = {
        id: Date.now(),
        name: formData.name,
        email: formData.email,
        password: formData.password
      };
      
      users.push(newUser);
      localStorage.setItem('users', JSON.stringify(users));
      localStorage.setItem('current_user', JSON.stringify(newUser));
      
      window.location.href = '/ragbot';
    } catch (err) {
      setError('Sign up failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>🚀 Sign Up</h1>
          <p className={styles.authSubtitle}>Create your RAG Chatbot account</p>

          {error && <div className={styles.error}>{error}</div>}

          <form onSubmit={handleSubmit} className={styles.authForm}>
            <div className={styles.inputGroup}>
              <label>Full Name</label>
              <input
                type="text"
                name="name"
                value={formData.name}
                onChange={handleChange}
                required
                className={styles.input}
                placeholder="Your Name"
              />
            </div>

            <div className={styles.inputGroup}>
              <label>Email</label>
              <input
                type="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                required
                className={styles.input}
                placeholder="your@email.com"
              />
            </div>

            <div className={styles.inputGroup}>
              <label>Password</label>
              <input
                type="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                required
                className={styles.input}
                placeholder="••••••••"
                minLength="6"
              />
            </div>

            <div className={styles.inputGroup}>
              <label>Confirm Password</label>
              <input
                type="password"
                name="confirmPassword"
                value={formData.confirmPassword}
                onChange={handleChange}
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
              {loading ? '⏳ Creating Account...' : '✨ Create Account'}
            </button>
          </form>

          <div className={styles.authLinks}>
            <p>Already have an account? <a href="/auth/signin">Sign In</a></p>
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