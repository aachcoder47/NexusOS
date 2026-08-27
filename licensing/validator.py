"""
NexusOS Licensing Framework - Runtime License Validator & Hardware Safety Hook
Enforces license compliance at runtime while guaranteeing safety-critical robotics fallback.
"""

import os
import logging
import time
from typing import Optional

from licensing.base import LicenseTier, LicensePayload, ValidationResult
from licensing.key_gen import LicenseKeyGenerator

logger = logging.getLogger("NexusOS.Licensing")


class LicenseValidator:
    """
    Runtime License Validator for NexusOS Edge Agents and Cloud Backend.
    Guarantees that missing or expired keys degrade gracefully without bricking robotics hardware.
    """

    def __init__(self, license_key: Optional[str] = None):
        self.raw_key = license_key or os.getenv("NEXUSOS_LICENSE_KEY")
        self.key_generator = LicenseKeyGenerator()
        self._cached_result: Optional[ValidationResult] = None

    def validate(self) -> ValidationResult:
        """
        Validate active license key.
        Returns a ValidationResult indicating tier and sandbox fallback state.
        """
        if not self.raw_key:
            # Fallback to Community / Open-Source Tier
            logger.info("NEXUSOS LICENSE NOTICE: No commercial key detected (NEXUSOS_LICENSE_KEY env variable unset). Operating under Community AGPL-3.0 Tier.")
            return ValidationResult(
                is_valid=True,
                tier=LicenseTier.COMMUNITY,
                message="Operating under Community AGPL-3.0 Open-Source License.",
                in_sandbox_mode=False,
                payload=LicensePayload(
                    license_id="community-oss-default",
                    org_name="Open-Source Community User",
                    tier=LicenseTier.COMMUNITY,
                    max_robots=5
                )
            )

        # Attempt to decode and verify JWT signature
        payload = self.key_generator.verify_and_decode(self.raw_key)

        if not payload:
            logger.warning("NEXUSOS LICENSE WARNING: Invalid or tampered commercial license signature! Falling back to Evaluation Sandbox Mode.")
            return ValidationResult(
                is_valid=False,
                tier=LicenseTier.INVALID,
                message="License signature invalid or corrupt. Operating in Evaluation Sandbox.",
                in_sandbox_mode=True,
                payload=None
            )

        if payload.is_expired():
            logger.warning(f"NEXUSOS LICENSE WARNING: Commercial License for '{payload.org_name}' EXPIRED on {time.ctime(payload.expires_at)}! Falling back to Evaluation Sandbox.")
            return ValidationResult(
                is_valid=False,
                tier=LicenseTier.EXPIRED,
                message=f"Commercial license expired for org '{payload.org_name}'.",
                in_sandbox_mode=True,
                payload=payload
            )

        logger.info(f"NEXUSOS LICENSE VALIDATED: Org '{payload.org_name}' | Tier: {payload.tier.value} | Max Robots: {payload.max_robots}")
        return ValidationResult(
            is_valid=True,
            tier=payload.tier,
            message=f"Valid Commercial License verified for '{payload.org_name}'.",
            in_sandbox_mode=False,
            payload=payload
        )

    def enforce_safety_sandbox(self, result: ValidationResult) -> None:
        """
        Safety Fallback Mechanism:
        Logs licensing notices without throwing unhandled exceptions that could crash motor drivers.
        """
        if result.in_sandbox_mode:
            logger.warning("==========================================================================")
            logger.warning("NEXUSOS SAFETY NOTICE: Running in EVALUATION SANDBOX MODE.")
            logger.warning("Hardware control loops and E-Stop remain 100% FUNCTIONAL for safety.")
            logger.warning("Commercial deployments exceeding $100K USD annual revenue require a paid NCL license.")
            logger.warning("Visit https://nexusos.io/commercial to obtain a license key.")
            logger.warning("==========================================================================")
