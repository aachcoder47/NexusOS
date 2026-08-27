"""
NexusOS Licensing Framework Unit & Integration Tests
"""

import sys
from pathlib import Path

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))

from licensing.base import LicenseTier, LicensePayload
from licensing.key_gen import LicenseKeyGenerator
from licensing.validator import LicenseValidator


def test_license_generation_and_validation():
    print("Running test_license_generation_and_validation...")
    gen = LicenseKeyGenerator()

    # 1. Generate valid commercial token
    token = gen.generate_token(
        license_id="TEST-001",
        org_name="RoboCorp Inc.",
        tier=LicenseTier.COMMERCIAL,
        max_robots=25,
        valid_days=30
    )
    assert token is not None, "Token generation failed!"
    print(f"  Generated Token: {token[:30]}...")

    # 2. Validate token using LicenseValidator
    validator = LicenseValidator(license_key=token)
    res = validator.validate()
    assert res.is_valid is True, f"Validation failed: {res.message}"
    assert res.tier == LicenseTier.COMMERCIAL
    assert res.payload.org_name == "RoboCorp Inc."
    assert res.in_sandbox_mode is False
    print("  Valid Commercial License verified successfully!")

    # 3. Test missing key fallback
    empty_validator = LicenseValidator(license_key=None)
    fallback_res = empty_validator.validate()
    assert fallback_res.is_valid is True
    assert fallback_res.tier == LicenseTier.COMMUNITY
    print("  Missing key gracefully fell back to Community Tier!")

    # 4. Test invalid signature tampering
    tampered_token = token[:-5] + "XXXXX"
    tampered_validator = LicenseValidator(license_key=tampered_token)
    sandbox_res = tampered_validator.validate()
    assert sandbox_res.is_valid is False
    assert sandbox_res.in_sandbox_mode is True
    print("  Tampered token correctly triggered Evaluation Sandbox Mode!")

    print("ALL LICENSING TESTS PASSED SUCCESSFULLY!")


if __name__ == "__main__":
    test_license_generation_and_validation()
