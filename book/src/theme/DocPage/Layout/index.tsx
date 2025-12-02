import React, { useEffect } from 'react';
import Layout from '@theme-original/DocPage/Layout';
import type LayoutType from '@theme/DocPage/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import { useDoc } from '@docusaurus/theme-common/internal';
import { useTranslation } from '@site/src/contexts/TranslationContext';
import LanguageToggle from '@site/src/components/LanguageToggle';
import TranslationPanel from '@site/src/components/TranslationPanel';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  const location = useLocation();
  const doc = useDoc();
  const { loadTranslation, currentLanguage, currentPageTranslation } = useTranslation();

  // Generate page ID from path
  const pageId = location.pathname.replace(/^\/|\/$/g, '').replace(/\//g, '-') || 'home';

  // Load translation when page changes
  useEffect(() => {
    loadTranslation(pageId);
  }, [pageId]);

  return (
    <>
      <Layout {...props} />
      {/* Only show translation features on doc pages, not on homepage */}
      {location.pathname !== '/' && location.pathname !== '/docs/' && (
        <>
          <div style={{
            position: 'fixed',
            bottom: '80px',
            right: '20px',
            zIndex: 999,
            background: 'var(--ifm-background-color)',
            borderRadius: '12px',
            boxShadow: '0 4px 20px rgba(0, 0, 0, 0.15)',
            padding: '8px',
          }}>
            <LanguageToggle />
          </div>
          <TranslationPanel
            pageId={pageId}
            pagePath={location.pathname}
            pageTitle={doc.metadata.title || 'Untitled Page'}
          />
        </>
      )}

      {/* Render Urdu translation if active */}
      {currentLanguage === 'ur' && currentPageTranslation && location.pathname !== '/' && (
        <div style={{
          position: 'fixed',
          top: '60px',
          left: 0,
          right: 0,
          bottom: 0,
          background: 'var(--ifm-background-color)',
          zIndex: 998,
          overflow: 'auto',
          padding: '20px',
        }}>
          <div style={{
            maxWidth: '900px',
            margin: '0 auto',
            direction: 'rtl',
            fontFamily: "'Noto Nastaliq Urdu', 'Arial', sans-serif",
            fontSize: '18px',
            lineHeight: '1.8',
            color: 'var(--ifm-font-color-base)',
          }}>
            <h1>{doc.metadata.title}</h1>
            <div style={{whiteSpace: 'pre-wrap'}}>
              {currentPageTranslation.urduTranslation}
            </div>
            <div style={{
              marginTop: '40px',
              padding: '16px',
              background: 'var(--ifm-color-emphasis-100)',
              borderRadius: '8px',
              fontSize: '14px',
            }}>
              <p style={{margin: 0}}>
                ترجمہ کار: {currentPageTranslation.userName} |
                تاریخ: {new Date(currentPageTranslation.updatedAt).toLocaleDateString('ur-PK')}
              </p>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
