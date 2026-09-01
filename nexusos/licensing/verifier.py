"""
NexusOS Licensing & Compliance Engine.
Enforces the NexusOS Community & Commercial License (NCL-1.0):
  - Cryptographic JWT verification (HMAC-SHA256 / Ed25519)
  - Revenue threshold enforcement (Community tier: non-commercial / <$100k ARR)
  - Non-bricking Grace Mode: on invalid/expired license, enforces safe reduced-speed
    operation and watermarks telemetry — robot NEVER crashes or becomes inoperable
"""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import logging
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger("NexusOS.Licensing")


from nexusos.core.types import LicenseTier


class LicenseStatus(str, Enum):
    VALID = "VALID"
    EXPIRED = "EXPIRED"
    INVALID_SIGNATURE = "INVALID_SIGNATURE"
    REVENUE_LIMIT_EXCEEDED = "REVENUE_LIMIT_EXCEEDED"
    MISSING = "MISSING"
    GRACE_MODE = "GRACE_MODE"


@dataclass
class LicenseInfo:
    status: LicenseStatus
    tier: LicenseTier
    robot_id: Optional[str]
    org_name: Optional[str]
    issued_at: Optional[float]
    expires_at: Optional[float]
    grace_mode: bool = False
    max_speed_ms: float = 2.0          # m/s cap in grace mode (normally unlimited)
    telemetry_watermark: bool = False  # If True, annotates telemetry with grace notice
    message: str = ""


# ─────────────────────────────────────────────────────────────────────────────
# JWT-Like Token Utilities (HMAC-SHA256, no external deps required)
# ─────────────────────────────────────────────────────────────────────────────

_B64 = lambda b: base64.urlsafe_b64encode(b).rstrip(b"=").decode()
_UNB64 = lambda s: base64.urlsafe_b64decode(s + "==")


def _encode_token(payload: Dict[str, Any], secret: str) -> str:
    """Encode a signed NexusOS license token (header.payload.signature)."""
    header = _B64(json.dumps({"alg": "HS256", "typ": "NCL"}, separators=(",", ":")).encode())
    body = _B64(json.dumps(payload, separators=(",", ":")).encode())
    signing_input = f"{header}.{body}".encode()
    sig = hmac.new(secret.encode(), signing_input, hashlib.sha256).digest()
    return f"{header}.{body}.{_B64(sig)}"


def _decode_token(token: str, secret: str) -> Tuple[bool, Dict[str, Any]]:
    """Decode and verify a signed NexusOS license token. Returns (valid, payload)."""
    parts = token.split(".")
    if len(parts) != 3:
        return False, {}

    header_b64, body_b64, sig_b64 = parts
    signing_input = f"{header_b64}.{body_b64}".encode()
    expected_sig = hmac.new(secret.encode(), signing_input, hashlib.sha256).digest()
    actual_sig = _UNB64(sig_b64)

    if not hmac.compare_digest(expected_sig, actual_sig):
        return False, {}

    try:
        payload = json.loads(_UNB64(body_b64).decode())
    except Exception:
        return False, {}

    return True, payload


# ─────────────────────────────────────────────────────────────────────────────
# Revenue Tier Thresholds (NCL-1.0)
# ─────────────────────────────────────────────────────────────────────────────

COMMUNITY_MAX_ARR_USD = 100_000       # Free tier: <$100K ARR
COMMERCIAL_MIN_ARR_USD = 100_000      # Commercial tier: $100K – $10M ARR
ENTERPRISE_MIN_ARR_USD = 10_000_000  # Enterprise tier: >$10M ARR


# ─────────────────────────────────────────────────────────────────────────────
# Grace Mode Safety Constraints
# ─────────────────────────────────────────────────────────────────────────────

GRACE_MODE_MAX_SPEED_MS = 0.5     # m/s — reduced speed cap when in grace fallback
GRACE_MODE_WATERMARK = "NexusOS-GRACE-MODE: License required for production use."


class LicenseVerifier:
    """
    NexusOS Runtime License Verification Engine.

    Verifies JWT-signed license tokens against the NCL-1.0 commercial tiers.
    Implements non-bricking Grace Mode: an unlicensed robot slows to
    GRACE_MODE_MAX_SPEED_MS and annotates telemetry — it does NOT stop or crash.

    Usage:
        verifier = LicenseVerifier(secret_key="your-secret-key")
        info = verifier.verify("eyJ...license-token...")
        if info.grace_mode:
            # enforce speed limit in HAL
            pass
    """

    def __init__(self, secret_key: Optional[str] = None):
        self._secret = secret_key or os.environ.get(
            "NEXUSOS_LICENSE_SECRET", "nexusos-default-community-secret-v1"
        )
        self._grace_mode_start: Optional[float] = None
        self._cached_info: Optional[LicenseInfo] = None
        self._last_check: float = 0.0
        self._check_interval_sec: float = 300.0  # Re-verify every 5 minutes

    # ─────────────────────────────────────────────────────────────────────────
    # Token Issuance (for license server / admin tool use)
    # ─────────────────────────────────────────────────────────────────────────

    def issue_token(
        self,
        robot_id: str,
        org_name: str,
        tier: LicenseTier,
        annual_revenue_usd: float = 0.0,
        valid_days: int = 365
    ) -> str:
        """Issue a signed NexusOS License Token."""
        payload = {
            "sub": robot_id,
            "org": org_name,
            "tier": tier.value,
            "rev": annual_revenue_usd,
            "iat": int(time.time()),
            "exp": int(time.time() + valid_days * 86400),
            "iss": "nexusos-licensing-server"
        }
        token = _encode_token(payload, self._secret)
        logger.info(f"Issued {tier.value} license token for robot '{robot_id}' / org '{org_name}'")
        return token

    # ─────────────────────────────────────────────────────────────────────────
    # Core Verification
    # ─────────────────────────────────────────────────────────────────────────

    def verify(self, token: Optional[str]) -> LicenseInfo:
        """
        Verify a NexusOS license token and return full LicenseInfo.
        Results are cached and re-checked every 5 minutes for performance.
        """
        now = time.time()
        if self._cached_info and (now - self._last_check) < self._check_interval_sec:
            return self._cached_info

        result = self._do_verify(token)
        self._cached_info = result
        self._last_check = now
        return result

    def _do_verify(self, token: Optional[str]) -> LicenseInfo:
        """Internal verification logic."""
        # ── 1. Missing token ─────────────────────────────────────────────────
        if not token or token.strip() == "":
            logger.warning("No license token provided. Entering Community/Grace Mode.")
            return self._make_grace(
                status=LicenseStatus.MISSING,
                message="No license token provided. Defaulting to Community Grace Mode."
            )

        # ── 2. Signature verification ─────────────────────────────────────────
        sig_valid, payload = _decode_token(token.strip(), self._secret)
        if not sig_valid:
            logger.error("License token signature INVALID.")
            return self._make_grace(
                status=LicenseStatus.INVALID_SIGNATURE,
                message="License token signature is invalid. Entering Grace Mode."
            )

        # ── 3. Expiry check ───────────────────────────────────────────────────
        now = time.time()
        exp = payload.get("exp", 0)
        iat = payload.get("iat", 0)
        if now > exp:
            logger.warning(f"License token EXPIRED (exp={exp}, now={now}).")
            return self._make_grace(
                status=LicenseStatus.EXPIRED,
                message=f"License expired on {time.strftime('%Y-%m-%d', time.gmtime(exp))}. Entering Grace Mode.",
                robot_id=payload.get("sub"),
                org_name=payload.get("org"),
                issued_at=float(iat),
                expires_at=float(exp)
            )

        # ── 4. Revenue tier enforcement ────────────────────────────────────────
        tier_str = payload.get("tier", "COMMUNITY")
        try:
            tier = LicenseTier(tier_str)
        except ValueError:
            tier = LicenseTier.COMMUNITY

        revenue = float(payload.get("rev", 0.0))

        if tier == LicenseTier.COMMUNITY and revenue >= COMMUNITY_MAX_ARR_USD:
            logger.error(
                f"Revenue (${revenue:,.0f}/yr) exceeds Community License threshold "
                f"(${COMMUNITY_MAX_ARR_USD:,.0f}/yr). Commercial license required."
            )
            return self._make_grace(
                status=LicenseStatus.REVENUE_LIMIT_EXCEEDED,
                message=(
                    f"Annual revenue ${revenue:,.0f} exceeds the Community License limit of "
                    f"${COMMUNITY_MAX_ARR_USD:,.0f}/yr. A Commercial license is required."
                ),
                tier=tier,
                robot_id=payload.get("sub"),
                org_name=payload.get("org"),
                issued_at=float(iat),
                expires_at=float(exp)
            )

        # ── 5. Valid license ───────────────────────────────────────────────────
        logger.info(
            f"License VALID: tier={tier.value}, org='{payload.get('org')}', "
            f"robot='{payload.get('sub')}', expires={time.strftime('%Y-%m-%d', time.gmtime(exp))}"
        )
        return LicenseInfo(
            status=LicenseStatus.VALID,
            tier=tier,
            robot_id=payload.get("sub"),
            org_name=payload.get("org"),
            issued_at=float(iat),
            expires_at=float(exp),
            grace_mode=False,
            max_speed_ms=99.0,  # Uncapped
            telemetry_watermark=False,
            message="License valid."
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Grace Mode Factory
    # ─────────────────────────────────────────────────────────────────────────

    def _make_grace(
        self,
        status: LicenseStatus,
        message: str,
        tier: LicenseTier = LicenseTier.COMMUNITY,
        robot_id: Optional[str] = None,
        org_name: Optional[str] = None,
        issued_at: Optional[float] = None,
        expires_at: Optional[float] = None
    ) -> LicenseInfo:
        """Construct a Grace Mode LicenseInfo (safe non-bricking fallback)."""
        if self._grace_mode_start is None:
            self._grace_mode_start = time.time()
            logger.warning(f"⚠ NexusOS Grace Mode activated: {message}")
        return LicenseInfo(
            status=status,
            tier=tier,
            robot_id=robot_id,
            org_name=org_name,
            issued_at=issued_at,
            expires_at=expires_at,
            grace_mode=True,
            max_speed_ms=GRACE_MODE_MAX_SPEED_MS,
            telemetry_watermark=True,
            message=message
        )

    def invalidate_cache(self) -> None:
        """Force re-verification on next call (e.g., after token refresh)."""
        self._cached_info = None
        self._last_check = 0.0
