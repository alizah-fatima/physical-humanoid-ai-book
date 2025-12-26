import React from 'react';
import Layout from '@theme-original/Layout';
import AIChatbot from '../components/AIChatbot';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <AIChatbot />
    </>
  );
}