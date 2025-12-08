import React, { useState } from 'react';
import ChatInterface from '../../ragbot-ui/ChatInterface';
import styles from './FloatingChatbot.module.css';

export default function FloatingChatbot() {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {/* Floating Chat Button */}
      <button 
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open Chat"
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chat Window */}
      {isOpen && (
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
