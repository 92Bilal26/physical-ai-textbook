import React, { useState } from 'react';
import styles from './AuthModal.module.css';
import { useAuth, User } from '@site/src/contexts/AuthContext';

interface AuthModalProps {
  isOpen?: boolean;
  onClose?: () => void;
  onAuthSuccess?: (user: User) => void;
}

// Use window location for API URL (works in browser)
const getApiUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:3001';
  return window.location.hostname === 'localhost'
    ? 'http://localhost:3001'
    : `https://${window.location.hostname}:3001`;
};

export default function AuthModal({ isOpen: propIsOpen, onClose: propOnClose, onAuthSuccess }: AuthModalProps) {
  const { user, setUser, isAuthModalOpen, closeAuthModal } = useAuth();

  // Support both controlled (props) and context-based usage
  const isOpen = propIsOpen !== undefined ? propIsOpen : isAuthModalOpen;
  const onClose = propOnClose || closeAuthModal;

  const [mode, setMode] = useState<'signin' | 'signup'>('signin');
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [messageType, setMessageType] = useState<'success' | 'error' | 'info'>('info');

  // Form states
  const [email, setEmail] = useState('test@example.com');
  const [password, setPassword] = useState('Test123!@');
  const [name, setName] = useState('Test User');

  const API_URL = getApiUrl();

  const showMessage = (msg: string, type: 'success' | 'error' | 'info') => {
    setMessage(msg);
    setMessageType(type);
  };

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/auth/sign-up`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ email, password, name }),
      });

      const data = await response.json();

      if (response.ok) {
        showMessage('✅ Account created! Signing you in...', 'success');
        setUser(data.user);
        setTimeout(() => {
          onAuthSuccess?.(data.user);
          onClose();
        }, 1000);
      } else {
        showMessage(`❌ ${data.message || 'Sign up failed'}`, 'error');
      }
    } catch (error) {
      showMessage(`❌ Error: ${error instanceof Error ? error.message : 'Unknown error'}`, 'error');
    } finally {
      setLoading(false);
    }
  };

  const handleSignIn = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/auth/sign-in`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (response.ok) {
        showMessage('✅ Signed in successfully!', 'success');
        setUser(data.user);
        setTimeout(() => {
          onAuthSuccess?.(data.user);
          onClose();
        }, 1000);
      } else {
        showMessage(`❌ ${data.message || 'Sign in failed'}`, 'error');
      }
    } catch (error) {
      showMessage(`❌ Error: ${error instanceof Error ? error.message : 'Unknown error'}`, 'error');
    } finally {
      setLoading(false);
    }
  };

  const handleSignOut = async () => {
    try {
      await fetch(`${API_URL}/api/auth/sign-out`, {
        method: 'POST',
        credentials: 'include',
      });
      setUser(null);
      showMessage('✅ Signed out', 'success');
    } catch (error) {
      showMessage('❌ Sign out failed', 'error');
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modal}>
      <div className={styles.overlay} onClick={onClose} />
      <div className={styles.content}>
        <button className={styles.closeBtn} onClick={onClose}>×</button>

        {user ? (
          // Logged In View
          <div className={styles.container}>
            <h2>Welcome, {user.name || user.email}! 👋</h2>
            <div className={styles.userInfo}>
              <p><strong>Email:</strong> {user.email}</p>
              <p><strong>ID:</strong> {user.id}</p>
            </div>
            <button
              className={styles.signOutBtn}
              onClick={handleSignOut}
            >
              Sign Out
            </button>
            <p className={styles.hint}>You can now access personalized content!</p>
          </div>
        ) : (
          // Sign In / Sign Up View
          <div className={styles.container}>
            <h2>{mode === 'signin' ? '🔐 Sign In' : '✨ Create Account'}</h2>

            {message && (
              <div className={`${styles.message} ${styles[messageType]}`}>
                {message}
              </div>
            )}

            <form onSubmit={mode === 'signin' ? handleSignIn : handleSignUp}>
              <div className={styles.formGroup}>
                <label>Email</label>
                <input
                  type="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  placeholder="you@example.com"
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label>Password</label>
                <input
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="Enter password"
                  required
                />
              </div>

              {mode === 'signup' && (
                <div className={styles.formGroup}>
                  <label>Name</label>
                  <input
                    type="text"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    placeholder="Your name"
                    required
                  />
                </div>
              )}

              <button
                type="submit"
                className={styles.submitBtn}
                disabled={loading}
              >
                {loading ? '⏳ Loading...' : (mode === 'signin' ? 'Sign In' : 'Create Account')}
              </button>
            </form>

            <div className={styles.toggle}>
              <p>
                {mode === 'signin' ? "Don't have an account? " : 'Already have an account? '}
                <button
                  type="button"
                  onClick={() => {
                    setMode(mode === 'signin' ? 'signup' : 'signin');
                    setMessage('');
                  }}
                  className={styles.toggleBtn}
                >
                  {mode === 'signin' ? 'Sign Up' : 'Sign In'}
                </button>
              </p>
            </div>

            <p className={styles.hint}>
              {mode === 'signin'
                ? 'Test: test@example.com / Test123!@'
                : 'Create a new account to personalize your learning'}
            </p>
          </div>
        )}
      </div>
    </div>
  );
}
