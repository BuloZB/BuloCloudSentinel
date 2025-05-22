# Security Incident Response Tabletop Exercise

## Exercise Overview

- **Date:** [Insert Date]
- **Time:** [Insert Time]
- **Location:** [Virtual/Physical Location]
- **Duration:** 2 hours
- **Facilitator:** [Insert Name]

## Objectives

1. Test the effectiveness of the Bulo.Cloud Sentinel Security Incident Response Plan
2. Identify gaps in the incident response process
3. Improve team coordination during security incidents
4. Familiarize team members with their roles and responsibilities
5. Practice decision-making under pressure

## Participants and Roles

- **Security Lead:** [Insert Name]
- **Technical Lead:** [Insert Name]
- **Communications Lead:** [Insert Name]
- **Legal Advisor:** [Insert Name]
- **Executive Sponsor:** [Insert Name]
- **Development Team Representatives:** [Insert Names]
- **Operations Team Representatives:** [Insert Names]
- **Observers:** [Insert Names]

## Exercise Format

1. **Introduction (15 minutes)**
   - Welcome and objectives
   - Review of the Security Incident Response Plan
   - Explanation of the exercise format and rules

2. **Scenario Presentation (15 minutes)**
   - Presentation of the incident scenario
   - Initial information provided
   - Questions for clarification

3. **Response Simulation (60 minutes)**
   - Team works through the incident response process
   - Facilitator introduces injects (new information) at key points
   - Team makes decisions and takes actions based on the available information

4. **Debrief (30 minutes)**
   - Discussion of what went well
   - Identification of areas for improvement
   - Action items for updating the incident response plan

## Scenario: Data Breach via Compromised Credentials

### Initial Information

*[To be revealed at the start of the exercise]*

The Security Operations Center has detected unusual API access patterns from an IP address not associated with any known customer. The activity appears to be accessing sensitive drone telemetry data across multiple customer accounts. The activity has been ongoing for approximately 12 hours.

### Inject 1 (15 minutes into response)

*[To be revealed during the exercise]*

Further investigation reveals that the suspicious API calls are being authenticated with valid API keys belonging to a system administrator account. The account belongs to a developer who recently left the company two weeks ago.

### Inject 2 (30 minutes into response)

*[To be revealed during the exercise]*

Log analysis shows that the administrator account was accessed from an unusual location 3 days ago. The access came from a public IP address associated with a coffee shop in a different country than where the former employee was based.

### Inject 3 (45 minutes into response)

*[To be revealed during the exercise]*

A customer has contacted support reporting that they've received an email from an unknown party claiming to have access to their drone flight data and demanding payment to prevent public disclosure.

## Expected Actions

The following actions are expected from the team during the exercise:

### Detection and Reporting

- Identify the security incident
- Escalate to the appropriate team members
- Create an incident ticket
- Establish a communication channel

### Assessment and Triage

- Confirm the incident
- Determine the severity level
- Assemble the Incident Response Team
- Assign roles and responsibilities

### Containment

- Disable the compromised account
- Revoke and rotate affected API keys
- Block the suspicious IP address
- Preserve evidence for investigation

### Investigation

- Analyze logs to determine the scope of the breach
- Identify the attack vector
- Determine what data was accessed
- Create a timeline of events

### Eradication

- Remove any persistent access mechanisms
- Reset credentials for all potentially affected accounts
- Patch any vulnerabilities that were exploited
- Verify that all unauthorized access has been removed

### Recovery

- Restore normal operations
- Implement additional monitoring
- Verify system integrity
- Communicate with affected customers

### Communication

- Internal communication to stakeholders
- External communication to affected customers
- Potential regulatory notifications
- Public relations response if necessary

## Evaluation Criteria

The exercise will be evaluated based on the following criteria:

1. **Time to Detection**
   - How quickly was the incident detected?
   - Were the right monitoring systems in place?

2. **Time to Response**
   - How quickly was the incident response team assembled?
   - Were the right people involved?

3. **Containment Effectiveness**
   - How effectively was the incident contained?
   - Were appropriate containment measures taken?

4. **Investigation Thoroughness**
   - Was the investigation thorough?
   - Were all affected systems identified?

5. **Communication Effectiveness**
   - Was communication clear and timely?
   - Were all stakeholders informed appropriately?

6. **Decision-Making Process**
   - Were decisions made in a timely manner?
   - Were decisions based on available information?

7. **Documentation**
   - Was the incident properly documented?
   - Were all actions recorded?

## Post-Exercise Activities

1. **Debrief Meeting**
   - Review the exercise
   - Discuss what went well and what could be improved
   - Identify gaps in the incident response plan

2. **Action Items**
   - Update the incident response plan
   - Address identified gaps
   - Schedule follow-up training

3. **Documentation**
   - Document the exercise
   - Document lessons learned
   - Update relevant documentation

## Appendix A: Exercise Rules

1. Treat the exercise as a real incident
2. Use the actual tools and processes that would be used in a real incident
3. Document all actions taken
4. Communicate as you would in a real incident
5. The facilitator may introduce new information during the exercise
6. The facilitator may call a timeout to provide guidance or clarification
7. If you're unsure about something, ask the facilitator

## Appendix B: Reference Materials

- [Bulo.Cloud Sentinel Security Incident Response Plan](security_incident_response_plan.md)
- [Contact List](internal-only-contact-list.md)
- [System Architecture Diagram](internal-only-architecture.md)
- [Data Classification Policy](internal-only-data-classification.md)

## Appendix C: Facilitator Notes

*[For facilitator use only - not to be shared with participants]*

- Observe how well the team follows the incident response plan
- Note any deviations from the plan
- Be prepared to provide additional information if the team gets stuck
- Adjust the difficulty based on the team's performance
- Ensure all participants have an opportunity to contribute
- Keep the exercise moving forward
- Document observations for the debrief
