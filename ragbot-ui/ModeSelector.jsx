import React from 'react';
import styles from './ChatInterface.module.css';

export function ModeSelector({ mode, onChange }) {
  const [isExpanded, setIsExpanded] = React.useState(false);

  return (
    <div className={styles.modeSelector}>
      <label onClick={() => setIsExpanded(!isExpanded)}>
        <span>Chat Mode: {mode === 'fullbook' ? '📚 Full Book' : '✂️ Selected Text'}</span>
        <span className={`${styles.toggleIcon} ${!isExpanded ? styles.collapsed : ''}`}>▼</span>
      </label>
      {isExpanded && (
        <>
          <div className={styles.modeButtons}>
            <button
              className={`${styles.modeButton} ${mode === 'fullbook' ? styles.active : ''}`}
              onClick={() => onChange('fullbook')}
            >
              📚 Full Book RAG
            </button>
            <button
              className={`${styles.modeButton} ${mode === 'selected' ? styles.active : ''}`}
              onClick={() => onChange('selected')}
            >
              ✂️ Selected Text Only
            </button>
          </div>
          <p className={styles.modeDescription}>
            {mode === 'fullbook'
              ? 'Search and answer from the entire book using RAG'
              : 'Answer only based on your selected text'}
          </p>
        </>
      )}
    </div>
  );
}

export default ModeSelector;
