import React from 'react';
import styles from './ChatInterface.module.css';

export function SelectedTextBox({ value, onChange }) {
  const handleSelectFromPage = () => {
    const selectedText = window.getSelection().toString();
    if (selectedText) {
      onChange(selectedText);
    } else {
      alert('Please select text on the page first');
    }
  };

  return (
    <div className={styles.selectedTextBox}>
      <div className={styles.selectedTextHeader}>
        <label>📝 Selected Text:</label>
        <button
          type="button"
          className={styles.selectButton}
          onClick={handleSelectFromPage}
        >
          Select from Page
        </button>
      </div>
      <textarea
        value={value}
        onChange={(e) => onChange(e.target.value)}
        placeholder="Paste or select text from the book here..."
        className={styles.selectedTextArea}
      />
      <p className={styles.selectedTextInfo}>
        {value.length} characters selected
      </p>
    </div>
  );
}

export default SelectedTextBox;
