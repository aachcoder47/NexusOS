"""
NexusOS Licensing Framework - Base Types and Dataclasses
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Any
import time


class LicenseTier(str, Enum):
    COMMUNITY = "COMMUNITY"          # Free open-source / copyleft tier
    ACADEMIC = "ACADEMIC"            # Free academic / research tier
    EVALUATION = "EVALUATION"        # Temporary trial tier
    COMMERCIAL = "COMMERCIAL"        # Paid commercial partner tier
    EXPIRED = "EXPIRED"              # Expired license
    INVALID = "INVALID"              # Malformed or unverified signature


@dataclass
class LicensePayload:
    license_id: str
    org_name: str
    tier: LicenseTier
    max_robots: int = 5
    issued_at: float = field(default_factory=time.time)
    expires_at: Optional[float] = None
    features: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def is_expired(self) -> bool:
        if self.expires_at is None:
            return False
        return time.time() > self.expires_at

    def to_dict(self) -> Dict[str, Any]:
        return {
            "license_id": self.license_id,
            "org_name": self.org_name,
            "tier": self.tier.value,
            "max_robots": self.max_robots,
            "issued_at": self.issued_at,
            "expires_at": self.expires_at,
            "features": self.features,
            "metadata": self.metadata
        }


@dataclass
class ValidationResult:
    is_valid: bool
    tier: LicenseTier
    message: str
    in_sandbox_mode: bool = False
    payload: Optional[LicensePayload] = None
