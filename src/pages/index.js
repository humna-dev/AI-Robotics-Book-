import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ModuleCard from '@site/src/components/ModuleCard';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroBanner)}>
      <div className="container">
        <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>

        <div className={styles.buttons}>
          <Link
            className={clsx('button button--secondary button--lg')}
            to="/docs/intro">
            Read the Textbook
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModulePreviewSection() {
  const modules = [
    {
      number: 1,
      title: "The Robotic Nervous System (ROS 2)",
      description: "Learn about ROS 2 fundamentals, nodes, topics, and Python agent integration.",
      link: "/docs/module-1",
      icon: "🤖"
    },
    {
      number: 2,
      title: "The Digital Twin (Gazebo & Unity)",
      description: "Explore physics simulation and sensor integration in virtual environments.",
      link: "/docs/module-2",
      icon: "🏗️"
    },
    {
      number: 3,
      title: "The AI-Robot Brain (NVIDIA Isaac)",
      description: "Discover Isaac ROS perception and path planning for intelligent navigation.",
      link: "/docs/module-3",
      icon: "🧠"
    },
    {
      number: 4,
      title: "Vision-Language-Action (VLA)",
      description: "Understand voice-to-action systems and cognitive planning with LLMs.",
      link: "/docs/module-4",
      icon: "🗣️"
    },
    {
      number: 5,
      title: "Capstone Autonomous Humanoid Project",
      description: "Integrate all concepts in a comprehensive humanoid robot project.",
      link: "/docs/module-5",
      icon: "🦾"
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">

        <div className="text--center padding-bottom--lg">
          <h2>Course Modules</h2>
          <p>Explore all 5 comprehensive modules of Physical AI & Humanoid Robotics</p>
        </div>

        {/* 🎯 GRID FIXED */}
        <div className={styles.modulesGrid}>
          {modules.map((m, i) => (
            <ModuleCard
              key={i}
              moduleNumber={m.number}
              title={m.title}
              description={m.description}
              link={m.link}
              icon={m.icon}
            />
          ))}
        </div>

        <div className="text--center padding-top--lg">
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Reading the Textbook
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-Native Textbook for Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <ModulePreviewSection />
      </main>
    </Layout>
  );
}
