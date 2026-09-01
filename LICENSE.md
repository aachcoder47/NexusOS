# NexusOS Community & Commercial License (NCL-1.0)

## Copyright Notice

Copyright (c) 2024–2026 NexusOS Contributors and Maintainers. All rights reserved.

---

## Preamble

The NexusOS Community & Commercial License ("NCL-1.0") governs all use, reproduction, modification, and distribution of the NexusOS software, including its Hardware Abstraction Layer, Agent Context Engine, Fleet Control Middleware, and associated tooling (collectively, the "Software").

This license is designed to sustain an open-core development model: non-commercial use, research, education, and open-source projects receive the full rights of a permissive free license, while commercial deployments generating revenue above the thresholds defined herein are required to obtain a commercial license and contribute a revenue share.

---

## Definitions

- **"Software"**: The NexusOS source code, compiled binaries, documentation, datasets, and any derivative works.
- **"Contributor"**: Any individual or legal entity that submits modifications to the Software.
- **"Commercial Use"**: Any deployment, integration, or distribution of the Software—or any product or service substantially derived from or dependent upon it—in exchange for monetary compensation, including SaaS subscriptions, API fees, hardware product sales, or professional services.
- **"Annual Recurring Revenue (ARR)"**: The annualized value of recurring revenue attributable to products or services that incorporate, depend upon, or are substantially enabled by the Software.
- **"Derivative Work"**: Any work that is based on or derived from the Software, including modifications, extensions, ports, and compilations.

---

## Part I — Community License (Free, Open-Source, Non-Commercial)

### 1.1 Grant of Rights

Subject to the terms of this License, each Contributor hereby grants You a worldwide, royalty-free, non-exclusive, irrevocable license to:

1. Use, copy, and modify the Software for **any non-commercial purpose**, including personal projects, academic research, education, and open-source development.
2. Distribute verbatim or modified copies of the Software under the terms of this NCL-1.0 License.
3. Deploy the Software in production environments **provided your Annual Recurring Revenue attributable to such deployment does not exceed USD $100,000 (one hundred thousand US dollars)** per twelve-month calendar period.

### 1.2 Open-Source Commercial Exemption

Organizations using the Software in an open-source product (published under OSI-approved licenses, with full source code publicly available) are exempt from the revenue thresholds in §1.1 and §2.1, regardless of revenue.

### 1.3 Attribution Requirement

All copies and substantial uses of the Software, whether modified or unmodified, must retain:
- This License text in full.
- The NexusOS copyright notice.
- A prominent notice in the documentation or user interface: "Powered by NexusOS (nexusos.io)"

### 1.4 Restrictions

Under the Community License, You may NOT:
- Remove, alter, or obscure the NexusOS copyright notices or attribution requirements.
- Use NexusOS trademarks or branding in a manner that implies endorsement by the NexusOS Project.
- Sublicense the Software under different terms.

---

## Part II — Commercial License

### 2.1 Commercial License Requirement

If Your deployment of the Software generates Annual Recurring Revenue exceeding **USD $100,000**, You **must** obtain a Commercial License from the NexusOS Project before the earlier of:
- The date on which Your ARR first exceeds the Community tier threshold, or
- 90 days from initial deployment.

### 2.2 Commercial License Tiers

| Tier | ARR Range | License Fee Model |
|------|-----------|------------------|
| **Commercial** | $100K – $10M ARR | 3% of ARR attributable to NexusOS-powered products + annual base fee |
| **Enterprise** | > $10M ARR | Negotiated — contact licensing@nexusos.io |
| **OEM / Embedded** | Any revenue, closed hardware | Per-unit license or flat annual fee — contact licensing@nexusos.io |

### 2.3 Commercial License Rights

A valid Commercial License grants the following additional rights beyond the Community License:

1. Closed-source commercial deployment without source disclosure.
2. Sublicensing to end-users of Your product.
3. Removal of Community attribution requirements (upon Enterprise tier).
4. Priority access to commercial support, SLA guarantees, and hardware certification programs.

### 2.4 License Enforcement & Grace Mode

If the Software detects deployment without a valid Commercial License where one is required:

1. **The Software will NOT stop operating or render robots inoperative.** Safety is paramount.
2. Instead, the Software will engage **Grace Mode**:
   - Maximum robot speed is capped at 0.5 m/s.
   - Telemetry streams are watermarked with a compliance notice.
   - A licensing alert is logged and surfaced in all monitoring dashboards.
3. Grace Mode provides a **30-day operational runway** during which the operator must obtain a valid Commercial License.
4. After 30 days in Grace Mode without a valid license, the speed cap is reduced to 0.2 m/s. Emergency Stop functionality is ALWAYS available.

---

## Part III — Contributor License Agreement

### 3.1 Inbound CLA

By submitting a Contribution (pull request, patch, or other code submission) to the NexusOS Project, You agree that:

1. You grant the NexusOS Project a perpetual, irrevocable, worldwide, royalty-free license to use, reproduce, modify, and distribute Your Contribution under the NCL-1.0 or any future version thereof.
2. You represent that You have legal authority to grant the above license.
3. Your Contribution is Your original work, or You have obtained necessary rights to contribute it.

---

## Part IV — Disclaimer of Warranties & Limitation of Liability

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, OR SAFETY FOR ROBOTIC DEPLOYMENT.

**ROBOTIC AND AUTONOMOUS SYSTEMS SAFETY NOTICE:** NexusOS is middleware software. Safety of physical robotic deployments is the sole responsibility of the system integrator. The NexusOS Project explicitly disclaims responsibility for any physical harm, property damage, or regulatory violations arising from robotic deployments using this Software.

IN NO EVENT SHALL THE AUTHORS, COPYRIGHT HOLDERS, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES ARISING FROM USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

---

## Part V — Governing Law & Dispute Resolution

This License shall be governed by and construed under the laws of the jurisdiction of the NexusOS Project's primary maintainer, without regard to conflict of law principles. Any dispute not resolved by good-faith negotiation shall be subject to binding arbitration under the UNCITRAL Arbitration Rules.

---

## Contact & Commercial Licensing

To obtain a Commercial or Enterprise license, or to report a compliance concern:

- **Email**: licensing@nexusos.io
- **Web**: https://nexusos.io/licensing
- **GitHub**: https://github.com/nexusos-robotics/NexusOS/issues

---

*NexusOS Community & Commercial License version 1.0 — Effective January 1, 2024*
