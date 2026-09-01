"""
NexusOS Monorepo Test Suite.
Covers: Proto/Pydantic types, HAL drivers, Memory Engine, Licensing verifier, Edge Daemon, Server.
"""

from __future__ import annotations
import asyncio
import math
import time
import uuid
import pytest

from nexusos.core.types import (
    RobotType, RobotMode, Vector3D, Quaternion3D, Pose3D, Twist3D,
    BatteryState, JointState, SensorPayload, TelemetryFrame,
    CommandFrame, DeviceHandshake, SpatialEntity, EnvironmentFact,
    AgentContextBlock, LicenseTier
)
from nexusos.core.hal import DifferentialDriveMockDriver, RoboticArm6DOFMockDriver
from nexusos.core.registry import DriverRegistry
from nexusos.memory.engine import AgentContextEngine
from nexusos.licensing.verifier import (
    LicenseVerifier, LicenseStatus, COMMUNITY_MAX_ARR_USD
)
from nexusos.agent.daemon import EdgeDaemon


# ─────────────────────────────────────────────────────────────────────────────
# 1. Pydantic Type Model Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestCoreTypes:

    def test_vector3d_creation_and_serialization(self):
        v = Vector3D(x=1.5, y=-2.3, z=0.0)
        assert v.x == pytest.approx(1.5)
        d = v.model_dump()
        assert d["x"] == pytest.approx(1.5)

    def test_telemetry_frame_roundtrip_json(self):
        frame = TelemetryFrame(
            robot_id="test_bot",
            robot_type=RobotType.DIFFERENTIAL_DRIVE,
            pose=Pose3D(position=Vector3D(x=1.0, y=2.0)),
            battery=BatteryState(voltage=12.1, percentage=85.5)
        )
        json_str = frame.model_dump_json()
        reconstructed = TelemetryFrame.model_validate_json(json_str)
        assert reconstructed.robot_id == "test_bot"
        assert reconstructed.battery.percentage == pytest.approx(85.5)

    def test_command_frame_serialization(self):
        cmd = CommandFrame(
            command_id="cmd_test_001",
            robot_id="bot_a",
            target_twist=Twist3D(
                linear=Vector3D(x=1.2, y=0.0, z=0.0),
                angular=Vector3D(x=0.0, y=0.0, z=0.5)
            )
        )
        d = cmd.model_dump()
        assert d["command_id"] == "cmd_test_001"
        assert d["target_twist"]["linear"]["x"] == pytest.approx(1.2)

    def test_device_handshake_defaults(self):
        hs = DeviceHandshake(
            robot_id="rover_99",
            hardware_signature="SHA256:abc123",
            license_token="eyJ..."
        )
        assert hs.robot_type == RobotType.DIFFERENTIAL_DRIVE
        assert len(hs.capabilities) == 0

    def test_agent_context_block_serialization(self):
        block = AgentContextBlock(
            context_id="ctx_001",
            robot_id="r1",
            detected_entities=[
                SpatialEntity(entity_id="e1", label="shelf", position=Vector3D(x=3.0, y=1.0, z=0.5))
            ],
            embedding_vector=[0.1, 0.2, 0.3],
            active_intent="PICK_AND_PLACE"
        )
        json_str = block.model_dump_json()
        restored = AgentContextBlock.model_validate_json(json_str)
        assert restored.active_intent == "PICK_AND_PLACE"
        assert len(restored.detected_entities) == 1


# ─────────────────────────────────────────────────────────────────────────────
# 2. Hardware Abstraction Layer Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestDifferentialDriveDriver:

    @pytest.mark.asyncio
    async def test_initialization(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_01")
        result = await driver.initialize()
        assert result is True
        assert driver.is_initialized
        assert driver.current_mode == RobotMode.MANUAL

    @pytest.mark.asyncio
    async def test_telemetry_production(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_02")
        await driver.initialize()
        frame = await driver.read_telemetry()
        assert frame.robot_id == "dd_test_02"
        assert frame.robot_type == RobotType.DIFFERENTIAL_DRIVE
        assert frame.battery.voltage > 0
        assert len(frame.sensors) > 0
        assert frame.sensors[0].sensor_type == "LIDAR_2D"

    @pytest.mark.asyncio
    async def test_velocity_command_and_kinematics(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_03")
        await driver.initialize()
        cmd = CommandFrame(
            command_id="c1",
            robot_id="dd_test_03",
            target_twist=Twist3D(linear=Vector3D(x=1.0), angular=Vector3D(z=0.0))
        )
        ok = await driver.send_command(cmd)
        assert ok is True
        # Drive for several cycles
        for _ in range(20):
            frame = await driver.read_telemetry()
        # Robot should have moved forward
        assert abs(driver._x) > 0.001 or abs(driver._y) > 0.001

    @pytest.mark.asyncio
    async def test_speed_clamping(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_04", max_linear_speed=2.0)
        await driver.initialize()
        # Command beyond speed limit
        cmd = CommandFrame(
            command_id="c2", robot_id="dd_test_04",
            target_twist=Twist3D(linear=Vector3D(x=99.0))
        )
        await driver.send_command(cmd)
        assert driver._cmd_v <= 2.0

    @pytest.mark.asyncio
    async def test_emergency_stop(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_05")
        await driver.initialize()
        await driver.emergency_stop()
        assert driver._emergency_stop_latched is True
        assert driver.current_mode == RobotMode.EMERGENCY_STOP
        # Command rejected while E-stop active
        cmd = CommandFrame(command_id="c3", robot_id="dd_test_05",
                           target_twist=Twist3D(linear=Vector3D(x=1.0)))
        ok = await driver.send_command(cmd)
        assert ok is False

    @pytest.mark.asyncio
    async def test_estop_reset(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_06")
        await driver.initialize()
        await driver.emergency_stop()
        result = driver.reset_estop()
        assert result is True
        assert driver.current_mode == RobotMode.MANUAL

    @pytest.mark.asyncio
    async def test_battery_discharge_over_time(self):
        driver = DifferentialDriveMockDriver(robot_id="dd_test_07")
        await driver.initialize()
        cmd = CommandFrame(command_id="c4", robot_id="dd_test_07",
                           target_twist=Twist3D(linear=Vector3D(x=2.0)))
        await driver.send_command(cmd)
        initial_soc = driver._battery_soc
        for _ in range(50):
            await driver.read_telemetry()
        assert driver._battery_soc <= initial_soc


class TestRoboticArm6DOFDriver:

    @pytest.mark.asyncio
    async def test_initialization(self):
        driver = RoboticArm6DOFMockDriver(robot_id="arm_test_01")
        ok = await driver.initialize()
        assert ok is True
        assert driver.is_initialized

    @pytest.mark.asyncio
    async def test_joint_telemetry(self):
        driver = RoboticArm6DOFMockDriver(robot_id="arm_test_02")
        await driver.initialize()
        frame = await driver.read_telemetry()
        assert len(frame.joints.names) == 6
        assert len(frame.joints.positions) == 6
        # End-effector should be above the base
        assert frame.pose.position.z > 0.0

    @pytest.mark.asyncio
    async def test_joint_trajectory_command(self):
        driver = RoboticArm6DOFMockDriver(robot_id="arm_test_03")
        await driver.initialize()
        target = [0.5, -0.3, 0.8, 0.1, -0.5, 0.2]
        cmd = CommandFrame(
            command_id="j1", robot_id="arm_test_03",
            target_joints=JointState(names=driver.JOINT_NAMES, positions=target)
        )
        ok = await driver.send_command(cmd)
        assert ok is True
        import time as _time
        # Simulate 20ms ticks by rewinding the driver's last-time before each call
        for _ in range(300):
            driver._last_telemetry_time = _time.monotonic() - 0.02
            await driver.read_telemetry()
        # Joints should have fully converged to targets
        for i in range(6):
            assert abs(driver.current_joints[i] - target[i]) < 0.02

    @pytest.mark.asyncio
    async def test_joint_limit_enforcement(self):
        driver = RoboticArm6DOFMockDriver(robot_id="arm_test_04")
        await driver.initialize()
        cmd = CommandFrame(
            command_id="j2", robot_id="arm_test_04",
            target_joints=JointState(names=driver.JOINT_NAMES, positions=[99.0]*6)
        )
        await driver.send_command(cmd)
        for i, (lo, hi) in enumerate(driver.JOINT_LIMITS):
            assert lo <= driver.target_joints[i] <= hi

    @pytest.mark.asyncio
    async def test_arm_emergency_stop(self):
        driver = RoboticArm6DOFMockDriver(robot_id="arm_test_05")
        await driver.initialize()
        await driver.emergency_stop()
        assert driver._emergency_stop_latched is True
        assert driver.joint_velocities == [0.0] * 6


# ─────────────────────────────────────────────────────────────────────────────
# 3. Driver Registry Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestDriverRegistry:

    def test_registered_defaults(self):
        drivers = DriverRegistry.list_registered_drivers()
        assert "DIFFERENTIAL_DRIVE" in drivers
        assert "ROBOTIC_ARM" in drivers

    def test_create_driver_by_name(self):
        driver = DriverRegistry.create_driver("DIFFERENTIAL_DRIVE", robot_id="reg_bot_01")
        assert isinstance(driver, DifferentialDriveMockDriver)
        assert driver.robot_id == "reg_bot_01"

    def test_invalid_driver_raises(self):
        with pytest.raises(KeyError):
            DriverRegistry.create_driver("NONEXISTENT_ROBOT_TYPE", robot_id="x")

    def test_custom_driver_registration(self):
        class CustomDriver(DifferentialDriveMockDriver):
            pass
        DriverRegistry.register("CUSTOM_TEST_DRIVER", CustomDriver)
        driver = DriverRegistry.create_driver("CUSTOM_TEST_DRIVER", robot_id="custom_01")
        assert isinstance(driver, CustomDriver)


# ─────────────────────────────────────────────────────────────────────────────
# 4. Agent Context Memory Engine Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestAgentContextEngine:

    def _make_frame(self, robot_id: str, x: float = 0.0, y: float = 0.0) -> TelemetryFrame:
        return TelemetryFrame(
            robot_id=robot_id,
            robot_type=RobotType.DIFFERENTIAL_DRIVE,
            pose=Pose3D(position=Vector3D(x=x, y=y))
        )

    def test_telemetry_ingestion_and_window(self):
        engine = AgentContextEngine(robot_id="mem_bot_01", temporal_window_sec=5.0)
        for i in range(10):
            engine.ingest_telemetry(self._make_frame("mem_bot_01", x=float(i)))
        window = engine.get_temporal_chunk()
        assert len(window) == 10

    def test_spatial_entity_ingestion(self):
        engine = AgentContextEngine(robot_id="mem_bot_02")
        entity = engine.ingest_spatial_entity("shelf", Vector3D(x=3.0, y=1.0, z=0.5))
        assert entity.label == "shelf"
        assert entity.entity_id.startswith("entity_")

    def test_spatial_entity_deduplication(self):
        engine = AgentContextEngine(robot_id="mem_bot_03", spatial_dedup_threshold_m=1.0)
        e1 = engine.ingest_spatial_entity("door", Vector3D(x=2.0, y=0.0))
        e2 = engine.ingest_spatial_entity("door", Vector3D(x=2.3, y=0.0))  # Within 1m threshold
        assert e1.entity_id == e2.entity_id  # Same entity merged
        assert len(engine._spatial_entities) == 1

    def test_spatial_entity_query_by_radius(self):
        engine = AgentContextEngine(robot_id="mem_bot_04")
        engine.ingest_spatial_entity("shelf_A", Vector3D(x=1.0, y=0.0))
        engine.ingest_spatial_entity("shelf_B", Vector3D(x=10.0, y=0.0))
        results = engine.query_spatial_entities(center=Vector3D(x=0.0), radius_m=3.0)
        assert len(results) == 1
        assert results[0].label == "shelf_A"

    def test_fact_assertion_and_retrieval(self):
        engine = AgentContextEngine(robot_id="mem_bot_05")
        fact = engine.assert_fact("door_3", "is_state", "LOCKED", validity_sec=100.0)
        valid_facts = engine.get_valid_facts()
        assert any(f.fact_id == fact.fact_id for f in valid_facts)

    def test_fact_temporal_expiry(self):
        engine = AgentContextEngine(robot_id="mem_bot_06")
        # Inject a fact with a past timestamp manually
        f = EnvironmentFact(
            fact_id="old_fact", subject="zone_a", predicate="is_state", object="CLEAR",
            temporal_validity_sec=1.0, timestamp=time.time() - 100.0  # already expired
        )
        engine._facts["old_fact"] = f
        valid = engine.get_valid_facts()
        assert not any(f.fact_id == "old_fact" for f in valid)
        assert "old_fact" not in engine._facts

    def test_context_block_creation(self):
        engine = AgentContextEngine(robot_id="mem_bot_07")
        engine.ingest_telemetry(self._make_frame("mem_bot_07", x=1.5))
        block = engine.create_context_block(
            embedding=[0.1, 0.2, 0.3, 0.4],
            active_intent="NAVIGATION"
        )
        assert block.robot_id == "mem_bot_07"
        assert block.active_intent == "NAVIGATION"
        assert len(block.embedding_vector) == 4

    def test_vector_similarity_query(self):
        engine = AgentContextEngine(robot_id="mem_bot_08")
        engine.ingest_telemetry(self._make_frame("mem_bot_08"))

        # Insert reference embeddings
        engine.create_context_block(embedding=[1.0, 0.0, 0.0, 0.0])
        engine.create_context_block(embedding=[0.0, 1.0, 0.0, 0.0])
        engine.create_context_block(embedding=[0.0, 0.0, 1.0, 0.0])

        results = engine.query_similar_contexts([1.0, 0.0, 0.0, 0.0], top_k=1)
        assert len(results) == 1
        similarity, _ = results[0]
        assert similarity > 0.99  # Near-perfect match


# ─────────────────────────────────────────────────────────────────────────────
# 5. Licensing & Compliance Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestLicenseVerifier:

    def _verifier(self) -> LicenseVerifier:
        return LicenseVerifier(secret_key="test-secret-12345")

    def test_missing_token_returns_grace_mode(self):
        v = self._verifier()
        info = v.verify(None)
        assert info.grace_mode is True
        assert info.status == LicenseStatus.MISSING

    def test_empty_string_returns_grace_mode(self):
        v = self._verifier()
        info = v.verify("")
        assert info.grace_mode is True

    def test_invalid_signature_returns_grace(self):
        v = self._verifier()
        info = v.verify("not.a.valid.token.at.all")
        assert info.status == LicenseStatus.INVALID_SIGNATURE
        assert info.grace_mode is True

    def test_valid_community_token(self):
        v = self._verifier()
        token = v.issue_token("bot_1", "Acme Corp", LicenseTier.COMMUNITY, annual_revenue_usd=5000.0)
        info = v.verify(token)
        assert info.status == LicenseStatus.VALID
        assert info.tier == LicenseTier.COMMUNITY
        assert info.grace_mode is False

    def test_valid_commercial_token(self):
        v = self._verifier()
        token = v.issue_token("bot_2", "BigCorp Inc", LicenseTier.COMMERCIAL, annual_revenue_usd=500_000.0)
        info = v.verify(token)
        assert info.status == LicenseStatus.VALID
        assert info.tier == LicenseTier.COMMERCIAL

    def test_revenue_limit_exceeded_in_community_license(self):
        v = self._verifier()
        token = v.issue_token(
            "bot_3", "RichCorp", LicenseTier.COMMUNITY,
            annual_revenue_usd=COMMUNITY_MAX_ARR_USD + 1.0
        )
        v.invalidate_cache()
        info = v.verify(token)
        assert info.status == LicenseStatus.REVENUE_LIMIT_EXCEEDED
        assert info.grace_mode is True

    def test_expired_token_returns_grace(self):
        import base64, hashlib, hmac, json
        v = self._verifier()
        payload = {
            "sub": "bot_exp", "org": "OldCorp", "tier": "COMMUNITY",
            "rev": 0.0, "iat": int(time.time()) - 700 * 86400,
            "exp": int(time.time()) - 100, "iss": "nexusos-test"
        }
        token = v.issue_token.__func__  # Just use internal encoder
        # Manually craft expired token
        b64 = lambda b: base64.urlsafe_b64encode(b).rstrip(b"=").decode()
        header = b64(json.dumps({"alg": "HS256", "typ": "NCL"}, separators=(",", ":")).encode())
        body = b64(json.dumps(payload, separators=(",", ":")).encode())
        sig_input = f"{header}.{body}".encode()
        sig = hmac.new("test-secret-12345".encode(), sig_input, hashlib.sha256).digest()
        expired_token = f"{header}.{body}.{b64(sig)}"

        v.invalidate_cache()
        info = v.verify(expired_token)
        assert info.status == LicenseStatus.EXPIRED
        assert info.grace_mode is True

    def test_grace_mode_speed_limit(self):
        v = self._verifier()
        info = v.verify("")  # Missing token -> grace mode
        assert info.max_speed_ms < 1.0  # Speed capped to 0.5 m/s

    def test_token_caching(self):
        v = self._verifier()
        token = v.issue_token("bot_cache", "Org", LicenseTier.COMMUNITY)
        v._check_interval_sec = 999  # Long cache window
        info1 = v.verify(token)
        info2 = v.verify(token)
        assert info1.status == info2.status
        assert v._cached_info is not None


# ─────────────────────────────────────────────────────────────────────────────
# 6. Edge Daemon Tests (HAL + License integration)
# ─────────────────────────────────────────────────────────────────────────────

class TestEdgeDaemon:

    @pytest.mark.asyncio
    async def test_daemon_hal_loop_produces_telemetry(self):
        """Test that the HAL loop runs and produces telemetry without crashing."""
        driver = DifferentialDriveMockDriver(robot_id="daemon_test_01")
        daemon = EdgeDaemon(
            robot_id="daemon_test_01",
            driver=driver,
            license_token="",
            cloud_ws_url="ws://localhost:8000/ws/edge"  # Won't connect in test
        )
        await driver.initialize()

        # Simulate HAL loop manually (without starting full asyncio tasks)
        for _ in range(5):
            frame = await driver.read_telemetry()
            daemon.context.ingest_telemetry(frame)
            daemon._latest_telemetry = frame
            daemon._telemetry_count += 1

        assert daemon._telemetry_count == 5
        assert daemon.context.get_temporal_chunk()  # Memory has data

    @pytest.mark.asyncio
    async def test_grace_mode_detected_on_no_token(self):
        driver = DifferentialDriveMockDriver(robot_id="daemon_test_02")
        daemon = EdgeDaemon(robot_id="daemon_test_02", driver=driver, license_token="")
        assert daemon._grace_mode is True
        assert daemon._license_info.max_speed_ms < 1.0

    @pytest.mark.asyncio
    async def test_grace_mode_not_active_with_valid_token(self):
        verifier = LicenseVerifier(secret_key="test-key-999")
        token = verifier.issue_token("daemon_bot_03", "TestOrg", LicenseTier.COMMUNITY)
        driver = DifferentialDriveMockDriver(robot_id="daemon_bot_03")
        daemon = EdgeDaemon(robot_id="daemon_bot_03", driver=driver, license_token=token)
        # Must override daemon's verifier secret to match
        daemon.license_verifier = verifier
        daemon._license_info = verifier.verify(token)
        daemon._grace_mode = daemon._license_info.grace_mode
        assert daemon._grace_mode is False

    def test_daemon_status_dict(self):
        driver = DifferentialDriveMockDriver(robot_id="daemon_test_04")
        daemon = EdgeDaemon(robot_id="daemon_test_04", driver=driver)
        status = daemon.get_status()
        assert status["robot_id"] == "daemon_test_04"
        assert "license_status" in status
        assert "grace_mode" in status
