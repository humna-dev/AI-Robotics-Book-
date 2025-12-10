import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: '5 Comprehensive Modules',
    description: (
      <>
        Learn Physical AI & Humanoid Robotics through 5 carefully designed modules,
        each containing 2 focused topics that build upon each other.
      </>
    ),
  },
  {
    title: 'Interactive Learning',
    description: (
      <>
        Engage with the content through our integrated RAG chatbot that answers
        questions about the textbook content in real-time.
      </>
    ),
  },
  {
    title: 'Modern Technology Stack',
    description: (
      <>
        Built with Docusaurus for optimal documentation experience, featuring
        responsive design, dark mode, and progress tracking.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {/* <Svg className={styles.featureSvg} role="img" /> */}
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}