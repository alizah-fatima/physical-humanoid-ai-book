import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className="hero__title">{siteConfig.title}</h1>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive Educational Resource on Advanced AI and Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.modulesSection}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h2 className={styles.sectionTitle}>Explore Our Learning Modules</h2>
                <p className={styles.sectionSubtitle}>Four comprehensive modules covering the complete spectrum of Physical AI and Humanoid Robotics</p>
              </div>
            </div>

            <div className="row" style={{marginTop: '2rem', gap: '1.5rem'}}>
              <div className="col col--6">
                <div className="card">
                  <div className="card__header">
                    <h3>
                      <span className="module-badge module-badge--ros2">Module 1</span>
                      <br />
                      The Robotic Nervous System (ROS 2)
                    </h3>
                  </div>
                  <div className="card__body">
                    <p>Master ROS 2 architecture, Python package development, and URDF modeling for humanoid robots.</p>
                    <ul>
                      <li>ROS 2 Architecture fundamentals</li>
                      <li>Node, topic, service, and action patterns</li>
                      <li>Python package development with rclpy</li>
                      <li>URDF for humanoid robot modeling</li>
                    </ul>
                  </div>
                  <div className="card__footer">
                    <Link className="button button--primary button--block" to="/docs/module1-ros2/chapter1-architecture">
                      Start Module 1
                    </Link>
                  </div>
                </div>
              </div>

              <div className="col col--6">
                <div className="card">
                  <div className="card__header">
                    <h3>
                      <span className="module-badge module-badge--gazebo">Module 2</span>
                      <br />
                      The Digital Twin (Gazebo & Unity)
                    </h3>
                  </div>
                  <div className="card__body">
                    <p>Master Gazebo simulation, robot description formats, and Unity integration for high-fidelity rendering.</p>
                    <ul>
                      <li>Gazebo physics simulation</li>
                      <li>URDF and SDF formats</li>
                      <li>Sensor simulation techniques</li>
                      <li>Unity integration for visualization</li>
                    </ul>
                  </div>
                  <div className="card__footer">
                    <Link className="button button--primary button--block" to="/docs/module2-digital-twin/chapter1-gazebo-intro">
                      Start Module 2
                    </Link>
                  </div>
                </div>
              </div>
            </div>

            <div className="row" style={{marginTop: '1.5rem', gap: '1.5rem'}}>
              <div className="col col--6">
                <div className="card">
                  <div className="card__header">
                    <h3>
                      <span className="module-badge module-badge--isaac">Module 3</span>
                      <br />
                      The AI-Robot Brain (NVIDIA Isaac™)
                    </h3>
                  </div>
                  <div className="card__body">
                    <p>Explore photorealistic simulation, hardware-accelerated perception, and sim-to-real transfer techniques.</p>
                    <ul>
                      <li>NVIDIA Isaac Sim platform</li>
                      <li>Isaac ROS hardware acceleration</li>
                      <li>Visual SLAM and navigation</li>
                      <li>Sim-to-real transfer methodologies</li>
                    </ul>
                  </div>
                  <div className="card__footer">
                    <Link className="button button--primary button--block" to="/docs/module3-nvidia-isaac/chapter1-isaac-sim-intro">
                      Start Module 3
                    </Link>
                  </div>
                </div>
              </div>

              <div className="col col--6">
                <div className="card">
                  <div className="card__header">
                    <h3>
                      <span className="module-badge module-badge--vla">Module 4</span>
                      <br />
                      Vision-Language-Action (VLA)
                    </h3>
                  </div>
                  <div className="card__body">
                    <p>Understand VLA models, voice-to-action systems, and cognitive planning for embodied intelligence.</p>
                    <ul>
                      <li>Vision-Language-Action models</li>
                      <li>Speech recognition with Whisper</li>
                      <li>Natural language to ROS 2 actions</li>
                      <li>Cognitive planning systems</li>
                    </ul>
                  </div>
                  <div className="card__footer">
                    <Link className="button button--primary button--block" to="/docs/module4-vla/chapter1-vla-intro">
                      Start Module 4
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.featuresSection}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h2 className={styles.sectionTitle}>Why Learn With Us?</h2>
              </div>
            </div>
            <div className="row" style={{marginTop: '2rem', gap: '1.5rem'}}>
              <div className="col col--3">
                <div className="card text--center">
                  <div className="card__body">
                    <h3>📚 Comprehensive</h3>
                    <p>Complete curriculum from fundamentals to advanced applications</p>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card text--center">
                  <div className="card__body">
                    <h3>🛠️ Practical</h3>
                    <p>Hands-on examples and real-world applications</p>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card text--center">
                  <div className="card__body">
                    <h3>🤖 Cutting-edge</h3>
                    <p>Latest technologies in AI and robotics</p>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card text--center">
                  <div className="card__body">
                    <h3>🎓 Educational</h3>
                    <p>Designed for students and professionals</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}