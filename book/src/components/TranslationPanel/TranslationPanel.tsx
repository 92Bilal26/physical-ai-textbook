import React, { useState, useEffect } from 'react';
import { useTranslation } from '@site/src/contexts/TranslationContext';
import { useAuth } from '@site/src/contexts/AuthContext';
import styles from './TranslationPanel.module.css';

interface TranslationPanelProps {
  pageId: string;
  pagePath: string;
  pageTitle: string;
}

export default function TranslationPanel({ pageId, pagePath, pageTitle }: TranslationPanelProps) {
  const { user, openAuthModal } = useAuth();
  const {
    isTranslationPanelOpen,
    closeTranslationPanel,
    currentPageTranslation,
    saveTranslation,
    isLoading,
  } = useTranslation();

  const [urduText, setUrduText] = useState('');
  const [message, setMessage] = useState('');
  const [messageType, setMessageType] = useState<'success' | 'error' | 'info'>('info');
  const [isSaving, setIsSaving] = useState(false);

  // Load existing translation when panel opens
  useEffect(() => {
    if (isTranslationPanelOpen && currentPageTranslation) {
      setUrduText(currentPageTranslation.urduTranslation);
    } else if (isTranslationPanelOpen) {
      setUrduText('');
    }
  }, [isTranslationPanelOpen, currentPageTranslation]);

  const showMessage = (msg: string, type: 'success' | 'error' | 'info') => {
    setMessage(msg);
    setMessageType(type);
    setTimeout(() => setMessage(''), 5000);
  };

  const handleSave = async () => {
    if (!user) {
      showMessage('Please sign in to save translations', 'error');
      openAuthModal();
      return;
    }

    if (!urduText.trim()) {
      showMessage('Translation cannot be empty', 'error');
      return;
    }

    setIsSaving(true);
    const success = await saveTranslation(pageId, pagePath, pageTitle, urduText);
    setIsSaving(false);

    if (success) {
      showMessage('✅ Translation saved successfully!', 'success');
      setTimeout(() => {
        closeTranslationPanel();
      }, 1500);
    } else {
      showMessage('❌ Failed to save translation', 'error');
    }
  };

  const handleCancel = () => {
    if (urduText !== (currentPageTranslation?.urduTranslation || '')) {
      if (confirm('You have unsaved changes. Are you sure you want to close?')) {
        closeTranslationPanel();
      }
    } else {
      closeTranslationPanel();
    }
  };

  if (!isTranslationPanelOpen) return null;

  return (
    <div className={styles.overlay}>
      <div className={styles.panel}>
        <div className={styles.header}>
          <div>
            <h2>🌍 Add Urdu Translation</h2>
            <p className={styles.subtitle}>{pageTitle}</p>
          </div>
          <button
            className={styles.closeBtn}
            onClick={handleCancel}
            aria-label="Close"
          >
            ×
          </button>
        </div>

        <div className={styles.content}>
          {!user && (
            <div className={styles.authWarning}>
              <p>📝 You must be signed in to add translations</p>
              <button onClick={openAuthModal} className={styles.signInBtn}>
                Sign In
              </button>
            </div>
          )}

          {message && (
            <div className={`${styles.message} ${styles[messageType]}`}>
              {message}
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="urduTranslation">
              Urdu Translation (اردو ترجمہ)
              {currentPageTranslation && (
                <span className={styles.lastEdited}>
                  Last edited by {currentPageTranslation.userName}
                </span>
              )}
            </label>
            <textarea
              id="urduTranslation"
              className={styles.textarea}
              value={urduText}
              onChange={(e) => setUrduText(e.target.value)}
              placeholder="اردو میں ترجمہ لکھیں..."
              rows={20}
              dir="rtl"
              lang="ur"
              disabled={!user}
            />
            <div className={styles.charCount}>
              {urduText.length} characters
            </div>
          </div>

          <div className={styles.actions}>
            <button
              className={styles.cancelBtn}
              onClick={handleCancel}
              disabled={isSaving}
            >
              Cancel
            </button>
            <button
              className={styles.saveBtn}
              onClick={handleSave}
              disabled={!user || isSaving || isLoading}
            >
              {isSaving ? '💾 Saving...' : '💾 Save Translation'}
            </button>
          </div>

          <div className={styles.tips}>
            <p><strong>Tips for translating:</strong></p>
            <ul>
              <li>Keep technical terms in English where appropriate</li>
              <li>Maintain formatting (headings, code blocks, lists)</li>
              <li>Preserve code examples unchanged</li>
              <li>Focus on clarity and accuracy</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
}
