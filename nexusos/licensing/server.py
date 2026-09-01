"""
NexusOS License Server Mock.
Lightweight FastAPI server for issuing and verifying NexusOS license tokens in development.
Production: replace with HSM-backed license management service.
Usage: python -m nexusos.licensing.server
"""

from __future__ import annotations
import logging
import os
import time
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from nexusos.licensing.verifier import LicenseVerifier, LicenseTier

logging.basicConfig(level="INFO", format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("NexusOS.LicenseServer")

app = FastAPI(title="NexusOS License Server", version="1.0.0", docs_url="/docs")
verifier = LicenseVerifier()


class IssueRequest(BaseModel):
    robot_id: str
    org_name: str
    tier: str = "COMMUNITY"
    annual_revenue_usd: float = 0.0
    valid_days: int = 365


class VerifyRequest(BaseModel):
    token: str


@app.get("/health")
def health():
    return {"status": "OK", "timestamp": time.time()}


@app.post("/issue")
def issue(req: IssueRequest):
    try:
        tier = LicenseTier(req.tier.upper())
    except ValueError:
        return JSONResponse(status_code=400, content={"error": f"Invalid tier: {req.tier}"})
    token = verifier.issue_token(
        robot_id=req.robot_id,
        org_name=req.org_name,
        tier=tier,
        annual_revenue_usd=req.annual_revenue_usd,
        valid_days=req.valid_days
    )
    return {"token": token, "tier": tier.value, "robot_id": req.robot_id}


@app.post("/verify")
def verify(req: VerifyRequest):
    info = verifier.verify(req.token)
    return {
        "status": info.status.value,
        "tier": info.tier.value,
        "grace_mode": info.grace_mode,
        "message": info.message,
        "robot_id": info.robot_id,
        "org_name": info.org_name
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("nexusos.licensing.server:app", host="0.0.0.0", port=9000, reload=False)
