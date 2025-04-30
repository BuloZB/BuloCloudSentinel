# Security Fundamentals Quiz

This quiz tests your understanding of the security fundamentals covered in the [Security Fundamentals](../security_fundamentals.md) module.

## Instructions

1. Answer each question to the best of your ability.
2. For multiple-choice questions, select the best answer.
3. For open-ended questions, provide a concise but complete answer.
4. Submit your answers to your security champion or team lead for review.

## Questions

### Security Principles

1. Which security principle states that users and processes should have only the minimum privileges necessary to perform their functions?
   - A) Defense in Depth
   - B) Principle of Least Privilege
   - C) Secure by Default
   - D) Fail Securely

2. In the context of Bulo.Cloud Sentinel, provide an example of how the "Defense in Depth" principle is implemented.

3. Why is the "Fail Securely" principle important? Give an example of how it is implemented in Bulo.Cloud Sentinel.

### Threat Modeling

4. What does the "I" in STRIDE stand for?
   - A) Integrity
   - B) Injection
   - C) Information Disclosure
   - D) Interception

5. Match each threat type with its corresponding example:
   - Spoofing: _____
   - Tampering: _____
   - Repudiation: _____
   - Information Disclosure: _____
   - Denial of Service: _____
   - Elevation of Privilege: _____

   Examples:
   - An attacker modifies data in a database
   - A user denies having performed a sensitive operation
   - An attacker overwhelms a system with requests
   - An attacker gains administrative access
   - Sensitive data is leaked through error messages
   - An attacker impersonates a legitimate user

6. Describe the steps of the threat modeling process in the correct order.

### Security Controls

7. Which type of security control aims to identify when an attack is occurring or has occurred?
   - A) Preventive Control
   - B) Detective Control
   - C) Corrective Control
   - D) Compensating Control

8. Provide an example of each type of security control (preventive, detective, and corrective) used in Bulo.Cloud Sentinel.

9. Why is it important to have multiple types of security controls in a system?

### Authentication and Authorization

10. What is the difference between authentication and authorization?

11. Which of the following is NOT a secure practice for session management?
    - A) Using JWT tokens with appropriate expiration
    - B) Setting cookies with HttpOnly and Secure flags
    - C) Storing session IDs in localStorage
    - D) Implementing session timeout for inactive users

12. Explain how role-based access control (RBAC) is implemented in Bulo.Cloud Sentinel.

### Data Protection

13. Which encryption algorithm is used for data at rest in Bulo.Cloud Sentinel?
    - A) MD5
    - B) SHA-256
    - C) AES-256
    - D) RSA-2048

14. What is the purpose of data classification? How is it implemented in Bulo.Cloud Sentinel?

15. List three practices that help ensure data privacy in Bulo.Cloud Sentinel.

### Secure Communications

16. Why is it important to use TLS for all communications in Bulo.Cloud Sentinel?

17. Which of the following is NOT a secure practice for API security?
    - A) Authentication for all API endpoints
    - B) Input validation for all parameters
    - C) Detailed error messages with stack traces
    - D) Rate limiting to prevent abuse

18. Explain how WebSocket connections are secured in Bulo.Cloud Sentinel.

### Security in the Software Development Lifecycle

19. At which phase of the software development lifecycle should threat modeling be performed?
    - A) Requirements Phase
    - B) Design Phase
    - C) Implementation Phase
    - D) Testing Phase

20. For each phase of the software development lifecycle, provide one security activity that should be performed:
    - Requirements Phase: _____
    - Design Phase: _____
    - Implementation Phase: _____
    - Testing Phase: _____
    - Deployment Phase: _____
    - Maintenance Phase: _____

## Answers

The answers to this quiz will be provided by your security champion or team lead after submission.

## Scoring

- 90-100%: Excellent understanding of security fundamentals
- 80-89%: Good understanding of security fundamentals
- 70-79%: Satisfactory understanding of security fundamentals
- Below 70%: Additional review of security fundamentals is recommended
