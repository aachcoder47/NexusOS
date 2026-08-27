"""
NexusOS Commercial Onboarding & Stripe Licensing Webhook API
Provides endpoints for license verification, usage reporting, and automated Stripe subscription processing.
"""

import hmac
import hashlib
import json
import logging
import time
from typing import Dict, Any, Optional

from fastapi import APIRouter, HTTPException, Header, Request, status
from pydantic import BaseModel

from licensing.base import LicenseTier, LicensePayload
from licensing.key_gen import LicenseKeyGenerator
from licensing.validator import LicenseValidator

logger = logging.getLogger("NexusOS.LicensingAPI")

router = APIRouter(prefix="/api/v1/license", tags=["Licensing & Monetization"])
generator = LicenseKeyGenerator()


# Pydantic Schemas

class LicenseVerifyRequest(BaseModel):
    license_key: str
    robot_id: Optional[str] = None


class UsageReportRequest(BaseModel):
    license_key: str
    org_id: str
    active_robot_units: int
    gross_revenue_quarter_usd: float
    reported_timestamp: float = time.time()


class AdminGenerateKeyRequest(BaseModel):
    org_name: str
    tier: str = "COMMERCIAL"
    max_robots: int = 100
    valid_days: int = 365
    admin_secret: str


# --- Endpoints ---

@router.post("/verify")
async def verify_license_key(req: LicenseVerifyRequest):
    """
    Validate a commercial JWT license key submitted by an Edge Agent or Fleet Server.
    """
    validator = LicenseValidator(license_key=req.license_key)
    res = validator.validate()

    return {
        "is_valid": res.is_valid,
        "tier": res.tier.value,
        "message": res.message,
        "in_sandbox_mode": res.in_sandbox_mode,
        "payload": res.payload.to_dict() if res.payload else None
    }


@router.post("/report_usage")
async def report_commercial_usage(req: UsageReportRequest):
    """
    Automated usage and revenue-share reporting endpoint for commercial partners.
    """
    validator = LicenseValidator(license_key=req.license_key)
    res = validator.validate()

    if not res.is_valid or res.tier != LicenseTier.COMMERCIAL:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Usage reporting requires a valid Commercial License key."
        )

    # Calculate 1.5% royalty commission if over $100K threshold
    revenue = req.gross_revenue_quarter_usd
    commission_due = 0.0
    if revenue > 25000.0:  # $25K quarterly = $100K annual
        commission_due = round(revenue * 0.015, 2)

    logger.info(f"Licensing API: Usage reported by '{req.org_id}'. Active Robots: {req.active_robot_units}, Revenue: ${revenue:.2f}, Royalty Due: ${commission_due:.2f}")

    return {
        "status": "ACCEPTED",
        "org_id": req.org_id,
        "active_robot_units": req.active_robot_units,
        "quarterly_gross_usd": revenue,
        "calculated_commission_usd": commission_due,
        "next_payment_due_timestamp": time.time() + (30 * 86400)
    }


@router.post("/generate_key")
async def generate_commercial_key(req: AdminGenerateKeyRequest):
    """
    Admin endpoint to issue cryptographically signed commercial JWT license tokens.
    """
    if req.admin_secret != "nexusos-admin-secret-key-2026":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid admin secret authorization key."
        )

    try:
        tier_enum = LicenseTier(req.tier)
    except ValueError:
        tier_enum = LicenseTier.COMMERCIAL

    lic_id = f"NCL-{int(time.time())}"
    token = generator.generate_token(
        license_id=lic_id,
        org_name=req.org_name,
        tier=tier_enum,
        max_robots=req.max_robots,
        valid_days=req.valid_days
    )

    return {
        "status": "ISSUED",
        "license_id": lic_id,
        "org_name": req.org_name,
        "tier": tier_enum.value,
        "license_key": token
    }


@router.post("/webhooks/stripe")
async def stripe_webhook_handler(request: Request, stripe_signature: Optional[str] = Header(None)):
    """
    FastAPI Webhook receiver for Stripe subscription events.
    Automatically generates and delivers signed JWT commercial license keys upon payment.
    """
    body_bytes = await request.body()
    try:
        event = json.loads(body_bytes.decode("utf-8"))
    except Exception:
        raise HTTPException(status_code=400, detail="Invalid JSON webhook payload.")

    event_type = event.get("type")
    logger.info(f"Received Stripe Webhook event: {event_type}")

    if event_type in ["checkout.session.completed", "invoice.payment_succeeded"]:
        session_data = event.get("data", {}).get("object", {})
        customer_email = session_data.get("customer_email") or session_data.get("billing_details", {}).get("email", "partner@enterprise.com")
        org_name = session_data.get("customer_details", {}).get("name", customer_email)

        # Generate 1-Year Commercial License Key
        lic_id = f"STRIPE-{event.get('id', int(time.time()))}"
        license_key = generator.generate_token(
            license_id=lic_id,
            org_name=org_name,
            tier=LicenseTier.COMMERCIAL,
            max_robots=50,
            valid_days=365
        )

        logger.info(f"Stripe Webhook SUCCESS: Automated Commercial License generated for '{org_name}' ({customer_email})")

        return {
            "status": "PROCESSED",
            "event_type": event_type,
            "org_name": org_name,
            "license_id": lic_id,
            "license_key": license_key
        }

    return {"status": "IGNORED", "event_type": event_type}
