---
slug: /
---

import HomepageHero from '@site/src/components/HomepageHero';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

<HomepageHero />

<HomepageFeatures />

<div style={{padding: '80px 40px', background: 'linear-gradient(135deg, #1a2a4f 0%, #0f3a6f 100%)', textAlign: 'center'}}>
  <div style={{maxWidth: '800px', margin: '0 auto'}}>
    <h2 style={{fontFamily: "'Sohne', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif", fontSize: '42px', fontWeight: 700, margin: '0 0 16px 0', color: '#f1f5f9'}}>Ready to Learn Robotics?</h2>
    <p style={{fontSize: '16px', color: '#cbd5e1', margin: '0 0 32px 0'}}>
      Start with Module 1: ROS 2 Fundamentals. Learn at your own pace with hands-on code examples and practical exercises.
    </p>
    <a href="/docs/module-1/" style={{display: 'inline-flex', alignItems: 'center', gap: '8px', padding: '12px 24px', fontSize: '14px', fontWeight: 600, borderRadius: '12px', border: 'none', cursor: 'pointer', transition: 'all 0.3s ease', textDecoration: 'none', background: 'linear-gradient(135deg, #06b6d4 0%, #3b82f6 100%)', color: '#0f172a', boxShadow: '0 8px 24px rgba(6, 182, 212, 0.3)'}} onMouseOver={(e) => {e.target.style.transform = 'translateY(-2px)'; e.target.style.boxShadow = '0 12px 32px rgba(6, 182, 212, 0.4)'}} onMouseOut={(e) => {e.target.style.transform = 'translateY(0)'; e.target.style.boxShadow = '0 8px 24px rgba(6, 182, 212, 0.3)'}}>
      ▶ Start Module 1
    </a>
  </div>
</div>
