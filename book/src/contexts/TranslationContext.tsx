import React, { createContext, useContext, useState, ReactNode } from 'react';
import { useAuth } from './AuthContext';

export interface Translation {
  id: string;
  pageId: string;
  pagePath: string;
  pageTitle: string;
  originalContent: string;
  urduTranslation: string;
  userId: string;
  userName: string;
  createdAt: string;
  updatedAt: string;
}

interface TranslationContextType {
  currentLanguage: 'en' | 'ur';
  setLanguage: (lang: 'en' | 'ur') => void;
  isTranslationPanelOpen: boolean;
  openTranslationPanel: () => void;
  closeTranslationPanel: () => void;
  currentPageTranslation: Translation | null;
  setCurrentPageTranslation: (translation: Translation | null) => void;
  loadTranslation: (pageId: string) => Promise<void>;
  saveTranslation: (pageId: string, pagePath: string, pageTitle: string, urduText: string) => Promise<boolean>;
  isLoading: boolean;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

// Backend API URL
const getApiUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:3001';
  return window.location.hostname === 'localhost'
    ? 'http://localhost:3001'
    : 'https://physical-ai-auth.onrender.com';
};

export function TranslationProvider({ children }: { children: ReactNode }) {
  const { user } = useAuth();
  const [currentLanguage, setCurrentLanguage] = useState<'en' | 'ur'>('en');
  const [isTranslationPanelOpen, setIsTranslationPanelOpen] = useState(false);
  const [currentPageTranslation, setCurrentPageTranslation] = useState<Translation | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const setLanguage = (lang: 'en' | 'ur') => {
    setCurrentLanguage(lang);
    // Save language preference to localStorage
    if (typeof window !== 'undefined') {
      localStorage.setItem('preferredLanguage', lang);
    }
  };

  // Load language preference on mount
  React.useEffect(() => {
    if (typeof window !== 'undefined') {
      const savedLang = localStorage.getItem('preferredLanguage');
      if (savedLang === 'ur' || savedLang === 'en') {
        setCurrentLanguage(savedLang);
      }
    }
  }, []);

  const openTranslationPanel = () => setIsTranslationPanelOpen(true);
  const closeTranslationPanel = () => setIsTranslationPanelOpen(false);

  const loadTranslation = async (pageId: string) => {
    setIsLoading(true);
    try {
      const response = await fetch(`${getApiUrl()}/api/translations/${encodeURIComponent(pageId)}`, {
        credentials: 'include',
      });

      if (response.ok) {
        const data = await response.json();
        setCurrentPageTranslation(data.translation || null);
      } else {
        setCurrentPageTranslation(null);
      }
    } catch (error) {
      console.error('Failed to load translation:', error);
      setCurrentPageTranslation(null);
    } finally {
      setIsLoading(false);
    }
  };

  const saveTranslation = async (
    pageId: string,
    pagePath: string,
    pageTitle: string,
    urduText: string
  ): Promise<boolean> => {
    if (!user) {
      console.error('User must be logged in to save translations');
      return false;
    }

    setIsLoading(true);
    try {
      const response = await fetch(`${getApiUrl()}/api/translations`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          pageId,
          pagePath,
          pageTitle,
          urduTranslation: urduText,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setCurrentPageTranslation(data.translation);
        return true;
      } else {
        const error = await response.json();
        console.error('Failed to save translation:', error);
        return false;
      }
    } catch (error) {
      console.error('Failed to save translation:', error);
      return false;
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <TranslationContext.Provider
      value={{
        currentLanguage,
        setLanguage,
        isTranslationPanelOpen,
        openTranslationPanel,
        closeTranslationPanel,
        currentPageTranslation,
        setCurrentPageTranslation,
        loadTranslation,
        saveTranslation,
        isLoading,
      }}
    >
      {children}
    </TranslationContext.Provider>
  );
}

export function useTranslation() {
  const context = useContext(TranslationContext);
  if (context === undefined) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  return context;
}
