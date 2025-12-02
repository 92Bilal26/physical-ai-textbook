import React from 'react';
import { useTranslation } from '@site/src/contexts/TranslationContext';
import { useAuth } from '@site/src/contexts/AuthContext';
import styles from './LanguageToggle.module.css';

export default function LanguageToggle() {
  const { currentLanguage, setLanguage, currentPageTranslation, openTranslationPanel } = useTranslation();
  const { user, openAuthModal } = useAuth();

  const hasTranslation = currentPageTranslation !== null;

  const handleTranslate = () => {
    if (!user) {
      openAuthModal();
      return;
    }
    openTranslationPanel();
  };

  return (
    <div className={styles.container}>
      {/* Language Toggle - only show if translation exists */}
      {hasTranslation && (
        <div className={styles.toggleGroup}>
          <button
            className={`${styles.langBtn} ${currentLanguage === 'en' ? styles.active : ''}`}
            onClick={() => setLanguage('en')}
            title="Switch to English"
          >
            EN
          </button>
          <button
            className={`${styles.langBtn} ${currentLanguage === 'ur' ? styles.active : ''}`}
            onClick={() => setLanguage('ur')}
            title="اردو میں دیکھیں"
          >
            UR
          </button>
        </div>
      )}

      {/* Translate Button - always show for logged-in users */}
      {user && (
        <button
          className={styles.translateBtn}
          onClick={handleTranslate}
          title={hasTranslation ? 'Edit Urdu Translation' : 'Add Urdu Translation'}
        >
          {hasTranslation ? '✏️ Edit Translation' : '🌍 Translate to Urdu'}
        </button>
      )}

      {/* Sign in prompt for non-logged in users */}
      {!user && (
        <button className={styles.signInPrompt} onClick={openAuthModal}>
          🔐 Sign in to translate
        </button>
      )}
    </div>
  );
}
