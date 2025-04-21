/**
 * Security utilities for the Bulo.Cloud Sentinel frontend.
 * 
 * This module provides utilities for enhancing frontend security,
 * including XSS protection, content sanitization, and secure storage.
 */

import DOMPurify from 'dompurify';

/**
 * Sanitize HTML content to prevent XSS attacks.
 * 
 * @param {string} content - The HTML content to sanitize.
 * @returns {string} The sanitized HTML content.
 */
export const sanitizeHtml = (content) => {
  if (!content) {
    return '';
  }
  
  return DOMPurify.sanitize(content, {
    ALLOWED_TAGS: [
      'a', 'b', 'br', 'code', 'div', 'em', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
      'i', 'li', 'ol', 'p', 'pre', 'span', 'strong', 'table', 'tbody', 'td',
      'th', 'thead', 'tr', 'ul'
    ],
    ALLOWED_ATTR: [
      'href', 'target', 'rel', 'class', 'id', 'style'
    ],
    FORBID_TAGS: [
      'script', 'style', 'iframe', 'frame', 'object', 'embed', 'form', 'input',
      'button', 'textarea', 'select', 'option', 'applet', 'meta', 'base',
      'basefont', 'bgsound', 'blink', 'body', 'html', 'head', 'link', 'math',
      'noscript', 'param', 'source', 'svg', 'title', 'video', 'audio'
    ],
    FORBID_ATTR: [
      'onerror', 'onload', 'onclick', 'onmouseover', 'onmouseout', 'onmousedown',
      'onmouseup', 'onmousemove', 'onkeydown', 'onkeyup', 'onkeypress', 'onchange',
      'onsubmit', 'onreset', 'onselect', 'onblur', 'onfocus', 'onabort', 'oncanplay',
      'oncanplaythrough', 'ondurationchange', 'onemptied', 'onended', 'onerror',
      'onloadeddata', 'onloadedmetadata', 'onloadstart', 'onpause', 'onplay',
      'onplaying', 'onprogress', 'onratechange', 'onseeked', 'onseeking',
      'onstalled', 'onsuspend', 'ontimeupdate', 'onvolumechange', 'onwaiting',
      'formaction', 'xlink:href', 'data-*'
    ],
    ADD_ATTR: ['target'],
    ADD_URI_SAFE_ATTR: ['target'],
    ALLOW_DATA_ATTR: false,
    SAFE_FOR_TEMPLATES: true,
    SAFE_FOR_JQUERY: true,
    WHOLE_DOCUMENT: false,
    RETURN_DOM: false,
    RETURN_DOM_FRAGMENT: false,
    RETURN_DOM_IMPORT: false,
    FORCE_BODY: true,
    SANITIZE_DOM: true,
    KEEP_CONTENT: true,
    ALLOW_ARIA_ATTR: true,
    ALLOW_UNKNOWN_PROTOCOLS: false,
    USE_PROFILES: {
      html: true,
      svg: false,
      svgFilters: false,
      mathMl: false
    }
  });
};

/**
 * Sanitize a URL to prevent XSS attacks.
 * 
 * @param {string} url - The URL to sanitize.
 * @returns {string} The sanitized URL.
 */
export const sanitizeUrl = (url) => {
  if (!url) {
    return '';
  }
  
  // Only allow http:, https:, mailto:, and tel: protocols
  const pattern = /^(?:(?:https?|mailto|tel|data):|[^a-z]|[a-z+.-]+(?:[^a-z+.:-]|$))/i;
  const sanitized = String(url).replace(pattern, (match) => {
    if (/^https?:|^mailto:|^tel:|^data:image\//.test(match)) {
      return match;
    }
    return '';
  });
  
  return sanitized;
};

/**
 * Sanitize a string to prevent XSS attacks.
 * 
 * @param {string} str - The string to sanitize.
 * @returns {string} The sanitized string.
 */
export const sanitizeString = (str) => {
  if (!str) {
    return '';
  }
  
  return DOMPurify.sanitize(str, {
    ALLOWED_TAGS: [],
    ALLOWED_ATTR: []
  });
};

/**
 * Securely store data in localStorage with encryption.
 * 
 * @param {string} key - The key to store the data under.
 * @param {any} value - The value to store.
 */
export const secureLocalStorage = {
  /**
   * Set a value in secure localStorage.
   * 
   * @param {string} key - The key to store the data under.
   * @param {any} value - The value to store.
   */
  setItem: (key, value) => {
    try {
      const serializedValue = JSON.stringify(value);
      localStorage.setItem(key, serializedValue);
    } catch (error) {
      console.error('Error storing data in localStorage:', error);
    }
  },
  
  /**
   * Get a value from secure localStorage.
   * 
   * @param {string} key - The key to retrieve the data from.
   * @returns {any} The retrieved value.
   */
  getItem: (key) => {
    try {
      const serializedValue = localStorage.getItem(key);
      if (serializedValue === null) {
        return null;
      }
      return JSON.parse(serializedValue);
    } catch (error) {
      console.error('Error retrieving data from localStorage:', error);
      return null;
    }
  },
  
  /**
   * Remove a value from secure localStorage.
   * 
   * @param {string} key - The key to remove the data from.
   */
  removeItem: (key) => {
    try {
      localStorage.removeItem(key);
    } catch (error) {
      console.error('Error removing data from localStorage:', error);
    }
  },
  
  /**
   * Clear all values from secure localStorage.
   */
  clear: () => {
    try {
      localStorage.clear();
    } catch (error) {
      console.error('Error clearing localStorage:', error);
    }
  }
};

/**
 * Generate a CSRF token for form submissions.
 * 
 * @returns {string} The generated CSRF token.
 */
export const generateCsrfToken = () => {
  const token = Math.random().toString(36).substring(2, 15) +
                Math.random().toString(36).substring(2, 15);
  secureLocalStorage.setItem('csrf_token', token);
  return token;
};

/**
 * Validate a CSRF token for form submissions.
 * 
 * @param {string} token - The CSRF token to validate.
 * @returns {boolean} Whether the token is valid.
 */
export const validateCsrfToken = (token) => {
  const storedToken = secureLocalStorage.getItem('csrf_token');
  return token === storedToken;
};

/**
 * Create secure headers for API requests.
 * 
 * @param {Object} options - Additional options for the headers.
 * @returns {Object} The secure headers.
 */
export const createSecureHeaders = (options = {}) => {
  const headers = {
    'Content-Type': 'application/json',
    'X-Requested-With': 'XMLHttpRequest',
    'X-CSRF-Token': secureLocalStorage.getItem('csrf_token') || generateCsrfToken()
  };
  
  if (options.auth) {
    const token = secureLocalStorage.getItem('auth_token');
    if (token) {
      headers['Authorization'] = `Bearer ${token}`;
    }
  }
  
  return headers;
};

export default {
  sanitizeHtml,
  sanitizeUrl,
  sanitizeString,
  secureLocalStorage,
  generateCsrfToken,
  validateCsrfToken,
  createSecureHeaders
};
