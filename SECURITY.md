# Security Policy

## Supported Versions

Please check the table below to see which versions of Exact2D CAD are currently receiving security updates.

| Version | Supported          |
| ------- | ------------------ |
| v0.1.x  | :white_check_mark: |
| < 0.1.0 | :x:                |

*(Note: As the project is currently in the `v0.1.0` release stage, only the `main` branch and the `0.1.x` series are actively supported.)*

## Reporting a Vulnerability

We take the security of Exact2D CAD seriously. Because the project involves a complex exact algebraic geometry kernel and parses external formats (like DXF, SVG, and native `.e2d` files through the `exact2d_io` crate), discovering a vulnerability is possible. 

If you discover a security flaw, please do **not** report it through public GitHub issues.

Instead, please report it privately by:

1. **Emailing:** fcoltro@proton.me
2. **Subject Line:** Please use the prefix `[SECURITY]` in your email subject.

**What to include:**
* A description of the vulnerability and its potential impact.
* Steps to reproduce the issue (e.g., attaching a specific `.e2d` or `.dxf` file that triggers the bug).
* Any potential mitigation or solutions if you have them.

**What to expect:**
We will acknowledge receipt of your vulnerability report within 48 hours and strive to send you regular updates about our progress. If the issue is confirmed, a patch will be released as soon as possible, and you will be credited for the discovery (unless you prefer to remain anonymous).
