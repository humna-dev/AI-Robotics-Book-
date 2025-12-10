import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function About() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`About ${siteConfig.title}`} description="About the Physical AI & Humanoid Robotics textbook">
      <div className="container margin-vert--lg">
        {/* Author Bio Section */}
        <div className="row">
          <div className="col col--8 col--offset-2 padding-vert--md">
            <h1 className="hero__title">About This Textbook</h1>
            <p className="text--center">
              <img
                src="/img/author-placeholder.svg"
                alt="Author"
                style={{
                  width: '150px',
                  height: '150px',
                  borderRadius: '50%',
                  objectFit: 'cover',
                  margin: '0 auto 20px',
                  display: 'block'
                }}
              />
            </p>
            <h2>Author Bio</h2>
            <p>
              Dr. Jane Robotics is a leading researcher in Physical AI and Humanoid Robotics with over 15 years of experience
              in developing autonomous robotic systems. She holds a Ph.D. in Robotics from MIT and has published over 100
              papers on humanoid locomotion, perception, and AI integration. Dr. Robotics has worked with leading technology
              companies including NVIDIA, Boston Dynamics, and OpenAI to advance the field of humanoid robotics.
            </p>
            <p>
              Her research focuses on creating robots that can seamlessly integrate into human environments, with particular
              expertise in bipedal locomotion, real-time perception, and cognitive robotics. She has received numerous awards
              including the IEEE Robotics and Automation Award and the ACM Prize in Computing for her contributions to
              humanoid robotics.
            </p>
          </div>
        </div>

        {/* Vision & Motivation Section */}
        <div className="row">
          <div className="col col--8 col--offset-2 padding-vert--md">
            <h2>Our Vision & Motivation</h2>
            <p>
              The "Physical AI & Humanoid Robotics" textbook represents a paradigm shift in robotics education. We envision
              a future where humanoid robots are as common as smartphones today, assisting humans in daily tasks, healthcare,
              manufacturing, and exploration of extreme environments.
            </p>
            <p>
              Our motivation stems from the recognition that current robotics education lacks the integration of physical AI
              concepts with practical implementation. Traditional approaches separate perception, cognition, and action, but
              real-world robotics requires seamless integration of all three.
            </p>
            <p>
              This textbook addresses the growing need for comprehensive educational materials that bridge the gap between
              theoretical AI concepts and practical robotics applications. As robotics technology advances toward human-level
              intelligence and dexterity, there is an urgent need for educational resources that prepare students for this future.
            </p>
          </div>
        </div>

        {/* Course Structure Section */}
        <div className="row">
          <div className="col col--8 col--offset-2 padding-vert--md">
            <h2>Course Structure</h2>
            <p>
              The textbook is organized into 5 comprehensive modules, each building upon the previous to provide a
              complete understanding of Physical AI & Humanoid Robotics:
            </p>
            <div className="row">
              <div className="col col--6">
                <ul>
                  <li><strong>Module 1:</strong> The Robotic Nervous System (ROS 2)</li>
                  <li><strong>Module 2:</strong> The Digital Twin (Gazebo & Unity)</li>
                </ul>
              </div>
              <div className="col col--6">
                <ul>
                  <li><strong>Module 3:</strong> The AI-Robot Brain (NVIDIA Isaac)</li>
                  <li><strong>Module 4:</strong> Vision-Language-Action (VLA)</li>
                  <li><strong>Module 5:</strong> Capstone Autonomous Humanoid Project</li>
                </ul>
              </div>
            </div>
          </div>
        </div>

        {/* Technology Integration Section */}
        <div className="row">
          <div className="col col--8 col--offset-2 padding-vert--md">
            <h2>Technology Integration</h2>
            <p>
              This AI-Native textbook leverages cutting-edge technologies including:
            </p>
            <div className="row">
              <div className="col col--6">
                <ul>
                  <li>Integrated RAG (Retrieval-Augmented Generation) chatbot for interactive learning</li>
                  <li>Real-time hardware integration notes for RTX workstations, Jetson Orin Nano, and RealSense sensors</li>
                  <li>Cloud robotics options using AWS RoboMaker and NVIDIA Omniverse Cloud</li>
                </ul>
              </div>
              <div className="col col--6">
                <ul>
                  <li>Progress tracking and personalized learning paths</li>
                  <li>Multilingual support (Urdu translation capability)</li>
                  <li>Collaborative learning features with peer-to-peer knowledge sharing</li>
                </ul>
              </div>
            </div>
          </div>
        </div>

        {/* Contact Information Section */}
        <div className="row">
          <div className="col col--8 col--offset-2 padding-vert--md">
            <h2>Contact Information</h2>
            <p>
              For questions about the textbook content, educational implementation, or research collaboration opportunities:
            </p>
            <ul>
              <li>Email: <a href="mailto:info@physicalai-textbook.org">info@physicalai-textbook.org</a></li>
              <li>Research Lab: <a href="https://www.university.edu/robotics-lab" target="_blank" rel="noopener noreferrer">Advanced Robotics Laboratory</a></li>
              <li>Office: Tech Building, Room 405, University Campus</li>
              <li>Office Hours: Tuesdays and Thursdays, 2:00 PM - 4:00 PM</li>
            </ul>
          </div>
        </div>
      </div>
    </Layout>
  );
}