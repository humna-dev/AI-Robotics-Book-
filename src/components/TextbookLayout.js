import React from 'react';
import Layout from '@theme/Layout';

export default function TextbookLayout(props) {
  const {children, title, description} = props;

  return (
    <Layout title={title} description={description}>
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            {children}
          </div>
        </div>
      </div>
    </Layout>
  );
}