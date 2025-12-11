import React, { useState, useEffect } from 'react';
import ChatInterface from '../../ragbot-ui/ChatInterface';
import styles from './FloatingChatbot.module.css';

export default function FloatingChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [showAuth, setShowAuth] = useState(false);
  const [authMode, setAuthMode] = useState('signin');
  const [formData, setFormData] = useState({ name: '', email: '', password: '' });

  useEffect(() => {
    const currentUser = localStorage.getItem('current_user');
    setIsAuthenticated(!!currentUser);
  }, []);

  const handleChatClick = () => {
    if (isAuthenticated) {
      setIsOpen(!isOpen);
    } else {
      setShowAuth(true);
    }
  };

  const handleAuth = (e) => {
    e.preventDefault();
    const users = JSON.parse(localStorage.getItem('users') || '[]');
    
    if (authMode === 'signup') {
      if (users.find(u => u.email === formData.email)) {
        alert('Email already exists');
        return;
      }
      const newUser = { id: Date.now(), ...formData };
      users.push(newUser);
      localStorage.setItem('users', JSON.stringify(users));
      localStorage.setItem('current_user', JSON.stringify(newUser));
    } else {
      const user = users.find(u => u.email === formData.email && u.password === formData.password);
      if (!user) {
        alert('Invalid credentials');
        return;
      }
      localStorage.setItem('current_user', JSON.stringify(user));
    }
    
    setIsAuthenticated(true);
    setShowAuth(false);
    setIsOpen(true);
  };

  return (
    <>
      <button 
        className={styles.floatingButton}
        onClick={handleChatClick}
        aria-label="Open Chat"
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {showAuth && (
        <div className={styles.authOverlay}>
          <div className={styles.authModal}>
            <button className={styles.closeAuth} onClick={() => setShowAuth(false)}>✕</button>
            <h3>{authMode === 'signin' ? 'Sign In' : 'Sign Up'}</h3>
            <form onSubmit={handleAuth}>
              {authMode === 'signup' && (
                <input
                  type="text"
                  placeholder="Name"
                  value={formData.name}
                  onChange={(e) => setFormData({...formData, name: e.target.value})}
                  required
                />
              )}
              <input
                type="email"
                placeholder="Email"
                value={formData.email}
                onChange={(e) => setFormData({...formData, email: e.target.value})}
                required
              />
              <input
                type="password"
                placeholder="Password"
                value={formData.password}
                onChange={(e) => setFormData({...formData, password: e.target.value})}
                required
              />
              <button type="submit">{authMode === 'signin' ? 'Sign In' : 'Sign Up'}</button>
            </form>
            <p onClick={() => setAuthMode(authMode === 'signin' ? 'signup' : 'signin')}>
              {authMode === 'signin' ? 'Need account? Sign Up' : 'Have account? Sign In'}
            </p>
          </div>
        </div>
      )}

      {isOpen && isAuthenticated && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>🤖 AI Assistant</h3>
            <button onClick={() => setIsOpen(false)} className={styles.closeBtn}>✕</button>
          </div>
          <ChatInterface apiUrl="https://hacks022-backend.vercel.app" />
        </div>
      )}
    </>
  );
}
