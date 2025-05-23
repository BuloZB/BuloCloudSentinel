FROM opendronemap/nodeodm:latest

# Expose port
EXPOSE 3000

# Set environment variables
ENV PORT=3000
ENV NODE_ENV=production

# Run the application
CMD ["node", "/var/www/index.js"]
