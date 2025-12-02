import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import { TranslationProvider } from '@site/src/contexts/TranslationContext';
import AuthModal from '@site/src/components/AuthModal';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <TranslationProvider>
        {children}
        <ChatWidget />
        <AuthModal />
      </TranslationProvider>
    </AuthProvider>
  );
}
