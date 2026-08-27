# Contributing to NexusOS & Contributor License Agreement (CLA)

Thank you for your interest in contributing to NexusOS! We welcome community contributions, bug fixes, hardware plugin extensions, and feature pull requests.

---

### Contributor License Agreement (CLA)

By submitting code, documentation, or hardware drivers to the NexusOS repository (via Pull Request, Commit, Issue, or Patch), you explicitly agree to the following terms:

#### 1. Grant of Copyright License
You grant to the NexusOS Maintainers and Project Copyright Holders a perpetual, worldwide, non-exclusive, no-charge, royalty-free, irrevocable copyright license to reproduce, prepare derivative works of, publicly display, publicly perform, sublicense, and distribute your Contributions.

#### 2. Dual-Licensing Authorization
You explicitly acknowledge and agree that NexusOS is distributed under a **Dual-Licensing Model (NexusOS Community & Commercial License - NCL v1.0)**. You grant the NexusOS Maintainers the sole right to relicense and distribute your Contributions under both:
- (a) **Community Tier (AGPLv3 / Open Source)**; and
- (b) **Commercial Tier (Proprietary / Paid Commercial Partner Agreements)**.

#### 3. Original Work Representation
You represent that each Contribution is your original creation and that you have full legal right and authority to grant the licenses set forth herein. If your employer has rights to intellectual property created by you, you represent that you have received permission to make Contributions on behalf of that employer.

---

### Contribution Workflow

1. **Fork the Repository**: Create your feature branch (`git checkout -b feature/amazing-plugin`).
2. **Coding Standards**:
   - Ensure all code is written in Python 3.10+ with modern type hints (`typing`, `dataclasses`).
   - Run type and syntax validation (`python -m py_compile ...`).
   - Include docstrings detailing hardware driver specifications.
3. **Submit a Pull Request**: Include a clear description of your changes and reference any related issues.
