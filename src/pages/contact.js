import React, {useState} from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Contact() {
  const {siteConfig} = useDocusaurusContext();
  const [email, setEmail] = useState('');
  const [submitted, setSubmitted] = useState(false);
  const [error, setError] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();

    // Simple email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      setError('Please enter a valid email address');
      return;
    }

    // Check for duplicate email in localStorage
    const waitlist = JSON.parse(localStorage.getItem('waitlist') || '[]');
    const emailExists = waitlist.some(item => item.email === email);

    if (emailExists) {
      setError('This email is already on the waitlist');
      return;
    }

    // Add to waitlist
    const newEntry = {
      email: email,
      timestamp: new Date().toISOString()
    };

    waitlist.push(newEntry);
    localStorage.setItem('waitlist', JSON.stringify(waitlist));

    setSubmitted(true);
    setError('');
  };

  return (
    <Layout title={`Contact ${siteConfig.title}`} description="Join the waitlist for Physical AI & Humanoid Robotics textbook updates">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="hero__title">Join Our Waitlist</h1>
            <p>
              Get notified when new content is available or when the course launches.
              We'll send you updates about the "Physical AI & Humanoid Robotics" textbook.
            </p>

            {submitted ? (
              <div className="alert alert--success margin-vert--md" role="alert">
                <h3>Thank You!</h3>
                <p>You've been added to our waitlist. We'll contact you soon with updates.</p>
              </div>
            ) : (
              <form onSubmit={handleSubmit} className="margin-vert--lg">
                <div className="form-group margin-bottom--md">
                  <label htmlFor="email">Email address</label>
                  <input
                    type="email"
                    id="email"
                    className={`form-control ${error ? 'form-control--error' : ''}`}
                    placeholder="your@email.com"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                  />
                  {error && (
                    <div className="alert alert--danger margin-top--sm" role="alert">
                      {error}
                    </div>
                  )}
                </div>
                <button type="submit" className="button button--primary button--lg">
                  Join Waitlist
                </button>
              </form>
            )}

            <h2 className="margin-top--xl">About the Course</h2>
            <p>
              The "Physical AI & Humanoid Robotics" course is designed to prepare students
              for the future of robotics, where AI and physical systems converge to create
              truly autonomous humanoid robots.
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}