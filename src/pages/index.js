import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import lottie from 'lottie-web';
import FloatingChatbot from '../components/FloatingChatbot';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container heroContent">
        {/* bubbles removed for cleaner hero */}
        <div className={styles.heroText}>
          <Heading as="h1" className={clsx('hero__title', styles.heroTitle)}>
            {siteConfig.title}
          </Heading>
          <p className={clsx('hero__subtitle', styles.heroSubtitle)}>{siteConfig.tagline}</p>
          <div className={styles.heroLead}>
            <p>
              Practical, hands-on labs and ready-to-run examples: install ROS 2,
              launch a simulation, and follow step-by-step lab guides to build
              real robot features.
            </p>
          </div>
          <div className={styles.heroCTAs}>
            <Link className="button button--primary button--lg primaryCta" to="/docs/intro">Get Started</Link>
            <a className="button button--secondary button--lg secondaryCta" href="https://github.com/WajahatAli3218664">Contribute</a>
          </div>
          <div className={styles.heroMeta}>
            <span className="pill">4 Modules</span>
            <span className="pill">Hands-on Labs</span>
            <span className="pill">Project-driven</span>
          </div>
        </div>

        <div className={styles.heroRight}>
          <div className={styles.heroIllustration} aria-hidden>
            {/* Amazing Alien SVG */}
            <svg viewBox="0 0 360 240" width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">
              <defs>
                <linearGradient id="alienGlow" x1="0" x2="1" y1="0" y2="1">
                  <stop offset="0%" stopColor="#8e2de2" />
                  <stop offset="50%" stopColor="#4a00e0" />
                  <stop offset="100%" stopColor="#2575fc" />
                </linearGradient>
                <filter id="glow">
                  <feGaussianBlur stdDeviation="3" result="coloredBlur"/>
                  <feMerge> 
                    <feMergeNode in="coloredBlur"/>
                    <feMergeNode in="SourceGraphic"/> 
                  </feMerge>
                </filter>
              </defs>
              
              {/* Alien Body */}
              <ellipse cx="180" cy="160" rx="45" ry="60" fill="url(#alienGlow)" filter="url(#glow)" opacity="0.9">
                <animateTransform attributeName="transform" type="rotate" values="0 180 160;5 180 160;0 180 160;-5 180 160;0 180 160" dur="4s" repeatCount="indefinite"/>
              </ellipse>
              
              {/* Alien Head */}
              <ellipse cx="180" cy="80" rx="55" ry="45" fill="url(#alienGlow)" filter="url(#glow)" opacity="0.95">
                <animateTransform attributeName="transform" type="scale" values="1;1.05;1;0.98;1" dur="3s" repeatCount="indefinite"/>
              </ellipse>
              
              {/* Eyes */}
              <ellipse cx="165" cy="75" rx="12" ry="18" fill="#00ff88" opacity="0.9">
                <animate attributeName="opacity" values="0.9;0.3;0.9" dur="2s" repeatCount="indefinite"/>
              </ellipse>
              <ellipse cx="195" cy="75" rx="12" ry="18" fill="#00ff88" opacity="0.9">
                <animate attributeName="opacity" values="0.9;0.3;0.9" dur="2s" repeatCount="indefinite"/>
              </ellipse>
              
              {/* Eye pupils */}
              <circle cx="165" cy="75" r="4" fill="#000">
                <animateTransform attributeName="transform" type="translate" values="0 0;2 -1;0 0;-2 1;0 0" dur="5s" repeatCount="indefinite"/>
              </circle>
              <circle cx="195" cy="75" r="4" fill="#000">
                <animateTransform attributeName="transform" type="translate" values="0 0;2 -1;0 0;-2 1;0 0" dur="5s" repeatCount="indefinite"/>
              </circle>
              
              {/* Arms */}
              <ellipse cx="130" cy="140" rx="8" ry="25" fill="url(#alienGlow)" opacity="0.8" transform="rotate(-20 130 140)">
                <animateTransform attributeName="transform" type="rotate" values="-20 130 140;-10 130 140;-20 130 140" dur="3s" repeatCount="indefinite"/>
              </ellipse>
              <ellipse cx="230" cy="140" rx="8" ry="25" fill="url(#alienGlow)" opacity="0.8" transform="rotate(20 230 140)">
                <animateTransform attributeName="transform" type="rotate" values="20 230 140;10 230 140;20 230 140" dur="3s" repeatCount="indefinite"/>
              </ellipse>
              
              {/* Floating particles */}
              <circle cx="120" cy="60" r="2" fill="#00ff88" opacity="0.7">
                <animateTransform attributeName="transform" type="translate" values="0 0;10 -15;0 0" dur="4s" repeatCount="indefinite"/>
                <animate attributeName="opacity" values="0.7;0.2;0.7" dur="4s" repeatCount="indefinite"/>
              </circle>
              <circle cx="250" cy="90" r="1.5" fill="#8e2de2" opacity="0.6">
                <animateTransform attributeName="transform" type="translate" values="0 0;-8 -12;0 0" dur="3.5s" repeatCount="indefinite"/>
                <animate attributeName="opacity" values="0.6;0.1;0.6" dur="3.5s" repeatCount="indefinite"/>
              </circle>
              <circle cx="280" cy="120" r="2.5" fill="#4a00e0" opacity="0.5">
                <animateTransform attributeName="transform" type="translate" values="0 0;-15 10;0 0" dur="5s" repeatCount="indefinite"/>
                <animate attributeName="opacity" values="0.5;0.1;0.5" dur="5s" repeatCount="indefinite"/>
              </circle>
            </svg>
          </div>
          <div className={styles.statsStrip}>
            {/* Replace simple stats with actionable stat cards */}
            <div className={styles.statCard}>
              <h4>Quickstart Guide</h4>
              <p>One-command setup to run a simulation locally.</p>
              <Link className={styles.statAction} to="/docs/intro">Open Guide</Link>
            </div>
            <div className={styles.statCard}>
              <h4>Simulation Templates</h4>
              <p>Prebuilt Gazebo & Unity scenes to jumpstart experiments.</p>
              <Link className={styles.statAction} to="/docs/module02-digital-twin/intro-digital-twin">View Templates</Link>
            </div>
            <div className={styles.statCard}>
              <h4>Capstone Recipes</h4>
              <p>End-to-end demos combining vision, language and action.</p>
              <Link className={styles.statAction} to="/docs/module04-vla-humanoid/intro-vla">See Recipes</Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

const modules = [
  {
    id: 'module01',
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description:
      'Build the communication backbone of your humanoid using ROS 2 nodes, topics, services, and actions.',
    to: '/docs/module01-ros2/intro-ros2',
  },
  {
    id: 'module02',
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description:
      'Create a precise digital twin of your humanoid and its world using Gazebo and Unity for simulation.',
    to: '/docs/module02-digital-twin/intro-digital-twin',
  },
  {
    id: 'module03',
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    description:
      'Use NVIDIA Isaac to give your humanoid high-fidelity perception, navigation, and manipulation capabilities.',
    to: '/docs/module03-isaac-brain/intro-isaac-brain',
  },
  {
    id: 'module04',
    title: 'Module 4: Vision-Language-Action + Capstone',
    description:
      'Wire language models and vision into your robot to build a full VLA-driven autonomous humanoid.',
    to: '/docs/module04-vla-humanoid/intro-vla',
  },
];

function ModulesGrid() {
  const animRefs = React.useRef({});

  React.useEffect(() => {
    const players = [];
    modules.forEach((m) => {
      const container = animRefs.current[m.id];
      if (container) {
        try {
          const player = lottie.loadAnimation({
            container,
            renderer: 'svg',
            loop: true,
            autoplay: true,
            path: `/animations/${m.id}.json`,
          });
          players.push(player);
        } catch (e) {
          // fall back silently
          // console.warn('lottie failed', e);
        }
      }
    });
    return () => players.forEach((p) => p.destroy && p.destroy());
  }, []);

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Explore the Book Modules
        </Heading>
        <div className={styles.modulesGrid}>
          {modules.map((module) => (
            <Link
              key={module.id}
              to={module.to}
              className={clsx('card padding--lg', styles.moduleCard, styles.fadeIn)}>
              <div ref={(el) => (animRefs.current[module.id] = el)} className={styles.moduleIcon} aria-hidden />
              <div className={styles.moduleBody}>
                <h3>{module.title}</h3>
                <p>{module.description}</p>
                <span className={styles.moduleLink}>Read this module →</span>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const features = [
    {
      title: 'Hands-on Labs',
      desc: 'Practical exercises with ROS 2, Gazebo, Isaac Sim and more.',
    },
    {
      title: 'Visualizations',
      desc: 'High-fidelity digital twin examples and step-by-step guides.',
    },
    {
      title: 'VLA Integrations',
      desc: 'Language and vision fused with action pipelines for humanoids.',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          What You’ll Build
        </Heading>
        <div className={styles.featuresGrid}>
          {features.map((f) => (
            <div key={f.title} className={clsx('card padding--md', styles.featureCard, styles.popIn)}>
              <h3>{f.title}</h3>
              <p>{f.desc}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Page-level footer removed — using the global theme footer instead

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Physical AI & Humanoid Robotics – a four-module journey from ROS 2 to full VLA-driven humanoids.">
      <HomepageHeader />
      <main>
        <ModulesGrid />
        <FeaturesSection />
        <section className={styles.ctaSection}>
          <div className="container">
            <h2>Ready to build?</h2>
            <p>Start the practical journey — follow module examples and build your first humanoid demo.</p>
            <div className={styles.ctaRow}>
              <Link className="button button--primary" to="/docs/intro">Get Started</Link>
              <a className="button button--secondary" href="https://github.com/WajahatAli3218664">Contribute</a>
            </div>
          </div>
        </section>
      </main>
      <FloatingChatbot />
    </Layout>
  );
}
