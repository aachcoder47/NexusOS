"""
NexusOS Licensing Framework - Key Generator & JWT Signer
Generates cryptographically signed JWT license tokens for NexusOS commercial partners.
"""

import base64
import hashlib
import hmac
import json
import time
from typing import Dict, Any, Optional

from licensing.base import LicensePayload, LicenseTier


# Standard secret key for local HMAC verification (commercial server holds private secret)
DEFAULT_SIGNING_SECRET = "nexusos-commercial-master-signing-key-2026-secret"


class LicenseKeyGenerator:
    """
    Utility class to issue and sign JWT-compatible NexusOS license keys.
    """

    def __init__(self, signing_secret: str = DEFAULT_SIGNING_SECRET):
        self.signing_secret = signing_secret

    def generate_token(
        self,
        license_id: str,
        org_name: str,
        tier: LicenseTier = LicenseTier.COMMERCIAL,
        max_robots: int = 100,
        valid_days: int = 365,
        features: Optional[list] = None
    ) -> str:
        """Create a base64-encoded signed JWT token."""
        now = time.time()
        expires_at = now + (valid_days * 86400) if valid_days > 0 else None

        header = {"alg": "HS256", "typ": "JWT"}
        payload = {
            "license_id": license_id,
            "org_name": org_name,
            "tier": tier.value,
            "max_robots": max_robots,
            "issued_at": now,
            "expires_at": expires_at,
            "features": features or ["cloud_fleet_sync", "advanced_telemetry", "unlimited_hal"]
        }

        # Encode Header & Payload
        header_b64 = self._b64_encode(json.dumps(header))
        payload_b64 = self._b64_encode(json.dumps(payload))

        signing_input = f"{header_b64}.{payload_b64}"
        signature = self._compute_signature(signing_input)

        return f"{signing_input}.{signature}"

    def verify_and_decode(self, token: str) -> Optional[LicensePayload]:
        """Verify HMAC signature and decode license token payload."""
        try:
            parts = token.split(".")
            if len(parts) != 3:
                return None

            header_b64, payload_b64, signature = parts
            signing_input = f"{header_b64}.{payload_b64}"
            expected_signature = self._compute_signature(signing_input)

            if not hmac.compare_digest(signature, expected_signature):
                return None

            payload_json = json.loads(self._b64_decode(payload_b64))
            return LicensePayload(
                license_id=payload_json.get("license_id", "unknown"),
                org_name=payload_json.get("org_name", "Unknown Org"),
                tier=LicenseTier(payload_json.get("tier", "COMMUNITY")),
                max_robots=payload_json.get("max_robots", 5),
                issued_at=payload_json.get("issued_at", time.time()),
                expires_at=payload_json.get("expires_at"),
                features=payload_json.get("features", [])
            )
        except Exception:
            return None

    def _compute_signature(self, data: str) -> str:
        sig = hmac.new(
            self.signing_secret.encode("utf-8"),
            data.encode("utf-8"),
            hashlib.sha256
        ).digest()
        return base64.urlsafe_b64encode(sig).decode("utf-8").rstrip("=")

    @staticmethod
    def _b64_encode(data_str: str) -> str:
        return base64.urlsafe_b64encode(data_str.encode("utf-8")).decode("utf-8").rstrip("=")

    @staticmethod
    def _b64_decode(b64_str: str) -> str:
        padding = "=" * (-len(b64_str) % 4)
        return base64.urlsafe_b64decode((b64_str + padding).encode("utf-8")).decode("utf-8")
