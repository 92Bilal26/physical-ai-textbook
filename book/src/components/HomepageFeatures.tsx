import React from 'react';
import '../css/homepage.css';

interface Feature {
  icon: string;
  title: string;
  description: string;
}

const features: Feature[] = [
  {
    icon: '📚',
    title: 'Comprehensive Learning',
    description: '3 chapters covering ROS 2 fundamentals, URDF robot description, and Python integration.',
  },
  {
    icon: '💻',
    title: 'Hands-On Code Examples',
    description: '8+ production-ready ROS 2 Python packages with detailed comments and full source code.',
  },
  {
    icon: '🎯',
    title: '9 Progressive Exercises',
    description: 'Beginner to advanced exercises with complete solutions and success criteria.',
  },
  {
    icon: '🤖',
    title: 'Simulation Ready',
    description: 'All examples work with Gazebo simulator. Control robots without physical hardware.',
  },
  {
    icon: '🔬',
    title: 'Pedagogically Sound',
    description: 'Built on Bloom\'s taxonomy and mental models for deeper, lasting understanding.',
  },
  {
    icon: '🆓',
    title: 'Free & Open Source',
    description: 'MIT License. No paywalls, no restrictions. Community contributions welcome.',
  },
];

export default function HomepageFeatures() {
  return (
    <section className="features">
      <div className="features-grid">
        {features.map((feature, idx) => (
          <div key={idx} className="feature-card">
            <div className="feature-icon">{feature.icon}</div>
            <h3>{feature.title}</h3>
            <p>{feature.description}</p>
          </div>
        ))}
      </div>
    </section>
  );
}
