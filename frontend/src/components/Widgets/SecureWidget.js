import React from 'react';
import { sanitizeString, sanitizeHtml } from '../../utils/security';
import './Widget.css';

/**
 * A secure widget component that sanitizes all content to prevent XSS attacks.
 * 
 * @param {Object} props - The component props.
 * @param {string} props.id - The widget ID.
 * @param {string} props.title - The widget title.
 * @param {Object} props.settings - The widget settings.
 * @param {Function} props.onRemove - The function to call when the widget is removed.
 * @param {React.ReactNode} props.children - The widget content.
 * @returns {React.ReactElement} The secure widget component.
 */
const SecureWidget = ({ id, title, settings, onRemove, children, className = '' }) => {
  // Sanitize title
  const sanitizedTitle = sanitizeString(title);
  
  return (
    <div className={`widget ${className}`}>
      <div className="widget-header">
        <h3 className="widget-title">{sanitizedTitle}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            &times;
          </button>
        )}
      </div>
      
      <div className="widget-content">
        {children}
      </div>
    </div>
  );
};

/**
 * A secure widget content component that sanitizes HTML content to prevent XSS attacks.
 * 
 * @param {Object} props - The component props.
 * @param {string} props.html - The HTML content to sanitize.
 * @param {string} props.className - The CSS class name.
 * @returns {React.ReactElement} The secure widget content component.
 */
export const SecureWidgetContent = ({ html, className = '' }) => {
  // Sanitize HTML
  const sanitizedHtml = sanitizeHtml(html);
  
  return (
    <div 
      className={`secure-widget-content ${className}`}
      dangerouslySetInnerHTML={{ __html: sanitizedHtml }}
    />
  );
};

/**
 * A secure widget text component that sanitizes text content to prevent XSS attacks.
 * 
 * @param {Object} props - The component props.
 * @param {string} props.text - The text content to sanitize.
 * @param {string} props.className - The CSS class name.
 * @returns {React.ReactElement} The secure widget text component.
 */
export const SecureWidgetText = ({ text, className = '' }) => {
  // Sanitize text
  const sanitizedText = sanitizeString(text);
  
  return (
    <div className={`secure-widget-text ${className}`}>
      {sanitizedText}
    </div>
  );
};

export default SecureWidget;
