import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatInterface.module.css';
import { ModeSelector } from './ModeSelector';
import { SelectedTextBox } from './SelectedTextBox';
import { SourceDisplay } from './SourceDisplay';

// Component to format assistant response with better styling
function FormattedResponse({ content }) {
  // Split by newlines and format paragraphs
  const lines = content.split('\n');
  const paragraphs = [];
  let currentParagraph = [];

  lines.forEach(line => {
    if (line.trim() === '') {
      if (currentParagraph.length > 0) {
        paragraphs.push(currentParagraph.join('\n'));
        currentParagraph = [];
      }
    } else {
      currentParagraph.push(line);
    }
  });

  if (currentParagraph.length > 0) {
    paragraphs.push(currentParagraph.join('\n'));
  }

  return (
    <div style={{ whiteSpace: 'pre-wrap', wordWrap: 'break-word' }}>
      {paragraphs.map((para, idx) => (
        <p key={idx} style={{ marginBottom: '12px', lineHeight: '1.6' }}>
          {para.split('\n').map((line, lineIdx) => (
            <React.Fragment key={lineIdx}>
              {line}
              {lineIdx < para.split('\n').length - 1 && <br />}
            </React.Fragment>
          ))}
        </p>
      ))}
    </div>
  );
}

export function ChatInterface({ sessionId: initialSessionId, apiUrl = '/api' }) {
  const [messages, setMessages] = useState([]);
  const [query, setQuery] = useState('');
  const [mode, setMode] = useState('fullbook');
  const [selectedText, setSelectedText] = useState('');
  const [loading, setLoading] = useState(false);
  const [sessionId, setSessionId] = useState(initialSessionId || null);
  const [sources, setSources] = useState([]);
  const [error, setError] = useState('');
  const [copiedIdx, setCopiedIdx] = useState(-1);
  const [feedback, setFeedback] = useState({});
  const messagesEndRef = useRef(null);

  // Scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const copyToClipboard = (text, idx) => {
    navigator.clipboard.writeText(text).then(() => {
      setCopiedIdx(idx);
      setTimeout(() => setCopiedIdx(-1), 2000);
    });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Initialize session
  useEffect(() => {
    if (!sessionId) {
      const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      setSessionId(newSessionId);
      loadChatHistory(newSessionId);
    }
  }, []);

  const loadChatHistory = async (sid) => {
    try {
      const response = await fetch(`${apiUrl}/chat/history/${sid}`);
      if (response.ok) {
        const data = await response.json();
        const loadedMessages = data.messages.map(msg => ({
          role: 'user',
          content: msg.query
        })).flatMap((msg, idx) => [
          msg,
          {
            role: 'assistant',
            content: data.messages[idx]?.response,
            sources: data.messages[idx]?.sources || [],
            mode: data.messages[idx]?.mode
          }
        ]);
        setMessages(loadedMessages);
      }
    } catch (err) {
      console.error('Error loading chat history:', err);
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();

    if (!query.trim()) return;
    if (mode === 'selected' && !selectedText.trim()) {
      setError('Please select text for "selected" mode');
      return;
    }

    setError('');
    const userMessage = { role: 'user', content: query };
    setMessages(prev => [...prev, userMessage]);
    setQuery('');
    setLoading(true);

    try {
      const requestBody = {
        query: query,
        mode: mode,
        session_id: sessionId || 'default',
        user_id: 'user_' + (sessionId || 'default')
      };
      
      if (mode === 'selected' && selectedText) {
        requestBody.selected_text = selectedText;
      }

      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Chat failed' }));
        const errorMsg = typeof errorData.detail === 'string' ? errorData.detail : JSON.stringify(errorData.detail || errorData);
        throw new Error(errorMsg);
      }

      const data = await response.json();

      setSources(data.sources || []);
      const assistantMessage = {
        role: 'assistant',
        content: typeof data.response === 'string' ? data.response : JSON.stringify(data.response),
        sources: data.sources || [],
        mode: data.mode
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      const isNetworkFailure = (err && (err.message === 'Failed to fetch' || err.name === 'TypeError'));

      // If backend unreachable, provide a helpful local fallback for greetings
      const isGreeting = (text) => {
        if (!text) return false;
        const s = text.toLowerCase().trim();
        return /^(hi|hello|hey|salam|assalam|as-salamu|hallo)\b/.test(s);
      };

      if (isNetworkFailure) {
        const fallbackResponse = isGreeting(query)
          ? `Hi! I can't reach the backend right now, but 👋 hello! To enable the full chatbot please make sure the backend is running and port 8000 is accessible from the preview (or run the API locally).` 
          : `I can't reach the backend right now. To use the full chatbot, ensure the backend is running and port 8000 is public (Codespaces Ports panel) or run the API locally.`;

        const assistantMessage = {
          role: 'assistant',
          content: fallbackResponse,
          sources: [],
          mode: mode
        };

        setMessages(prev => [...prev, assistantMessage]);
        setError('Backend unreachable. Using a local fallback reply for now.');
      } else {
        setError(err.message || 'Failed to send message');
      }

      console.error('Chat error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h1>📖 Ask the Book 🤖</h1>
        <p>Physical AI Humanoid Robotics RAG Chatbot</p>
      </div>

      <div className={styles.controlsPanel}>
        <ModeSelector mode={mode} onChange={setMode} />
        {mode === 'selected' && (
          <SelectedTextBox value={selectedText} onChange={setSelectedText} />
        )}
      </div>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.emptyState}>
            <p>👋 Welcome! Ask any question about the book.</p>
            <p>
              {mode === 'fullbook'
                ? 'I\'ll search the entire book for relevant information.'
                : 'I\'ll answer based on your selected text only.'}
            </p>
            <div className={styles.quickReplies}>
              <button onClick={() => setQuery('What is ROS2?')} className={styles.quickReply}>What is ROS2?</button>
              <button onClick={() => setQuery('Explain digital twins')} className={styles.quickReply}>Explain digital twins</button>
              <button onClick={() => setQuery('Tell me about NVIDIA Isaac')} className={styles.quickReply}>Tell me about NVIDIA Isaac</button>
            </div>
          </div>
        )}

        {messages.map((message, idx) => (
          <div key={idx} className={`${styles.message} ${styles[message.role]}`}>
            <div className={styles.messageContent}>
              {message.role === 'assistant' ? (
                <FormattedResponse content={message.content} />
              ) : (
                message.content
              )}
            </div>
            <div className={styles.messageFooter}>
              <span className={styles.timestamp}>
                {new Date().toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' })}
              </span>
              {message.role === 'assistant' && (
                <div className={styles.messageActions}>
                  <button
                    onClick={() => copyToClipboard(message.content, idx)}
                    className={styles.actionBtn}
                    title="Copy"
                  >
                    {copiedIdx === idx ? '✓' : '📋'}
                  </button>
                  <button
                    onClick={() => setFeedback({...feedback, [idx]: 'up'})}
                    className={`${styles.actionBtn} ${feedback[idx] === 'up' ? styles.active : ''}`}
                    title="Good response"
                  >
                    👍
                  </button>
                  <button
                    onClick={() => setFeedback({...feedback, [idx]: 'down'})}
                    className={`${styles.actionBtn} ${feedback[idx] === 'down' ? styles.active : ''}`}
                    title="Bad response"
                  >
                    👎
                  </button>
                </div>
              )}
            </div>
            {message.role === 'assistant' && message.sources && message.sources.length > 0 && (
              <SourceDisplay sources={message.sources} />
            )}
          </div>
        ))}

        {loading && (
          <div className={styles.message + ' ' + styles.assistant}>
            <div className={styles.loading}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSendMessage} className={styles.inputForm}>
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder={mode === 'fullbook' ? "Ask a question..." : "Ask about the selected text..."}
          className={styles.input}
          disabled={loading}
        />
        <button type="submit" disabled={loading || !query.trim()} className={styles.button}>
          {loading ? '⏳' : '📤'}
        </button>
      </form>
    </div>
  );
}

export default ChatInterface;
