import React from 'react';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import AIChatbot from '../components/AIChatbot';

export default function Layout({ children }) {
  const { metadata } = useDoc();

  // Only show the AI chatbot on documentation pages
  const showChatbot = metadata && metadata.id;

  return (
    <>
      {children}
      {showChatbot && <AIChatbot />}
    </>
  );
}