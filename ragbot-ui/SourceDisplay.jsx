import React, { useState } from 'react';
import styles from './ChatInterface.module.css';

export function SourceDisplay({ sources }) {
  const [expanded, setExpanded] = useState(false);
  const [copied, setCopied] = useState(false);

  if (!sources || sources.length === 0) {
    return null;
  }

  const copySourcesText = () => {
    const sourcesText = sources.map(
      (s, idx) => `${idx + 1}. ${s.filename}\n   Module: ${s.module}\n   Content: ${s.content.substring(0, 200)}\n`
    ).join('\n');
    
    navigator.clipboard.writeText(sourcesText).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  };

  return (
    <div className={styles.sourcesSection}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: '10px' }}>
        <button
          className={styles.sourcesToggle}
          onClick={() => setExpanded(!expanded)}
          style={{ flex: 1 }}
        >
          📑 Sources ({sources.length}) {expanded ? '▼' : '▶'}
        </button>
        <button
          className={styles.selectButton}
          onClick={copySourcesText}
          title="Copy sources to clipboard"
        >
          {copied ? '✓ Copied!' : 'Copy'}
        </button>
      </div>

      {expanded && (
        <div className={styles.sourcesList}>
          {sources.map((source, idx) => (
            <div key={idx} className={styles.sourceItem}>
              <div className={styles.sourceHeader}>
                <strong>📄 {idx + 1}. {source.filename}</strong>
              </div>
              {source.module && (
                <div style={{ marginBottom: '4px' }}>
                  <span className={styles.module}>📁 {source.module}</span>
                </div>
              )}
              {source.section && (
                <div className={styles.sourceSection}>
                  <strong>📌 Section:</strong> {source.section}
                </div>
              )}
              <div className={styles.sourceContent}>
                <strong>📝 Content:</strong> {source.content.substring(0, 200)}{source.content.length > 200 ? '...' : ''}
              </div>
              {source.score && (
                <div className={styles.sourceScore}>
                  ⭐ Relevance: {(source.score * 100).toFixed(1)}%
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

export default SourceDisplay;
