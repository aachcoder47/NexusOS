"""
NexusOS Agent Context & Temporal Memory Engine.
Real-time spatial memory ingestion pipeline, temporal state chunking,
landmark deduplication, and semantic context retrieval for AI agent control loops.
"""

from __future__ import annotations
import time
import math
import uuid
import logging
from collections import deque
from typing import Dict, List, Optional, Any, Tuple
import numpy as np

from nexusos.core.types import (
    Vector3D,
    Pose3D,
    TelemetryFrame,
    SpatialEntity,
    EnvironmentFact,
    AgentContextBlock
)

logger = logging.getLogger("NexusOS.Memory")


class AgentContextEngine:
    """
    High-performance real-time spatial memory and temporal context engine.
    Maintains:
      1. Short-term episodic telemetry buffer (high-frequency temporal chunking)
      2. Spatial Topological Scene Graph (deduplicated persistent landmark entities)
      3. Semantic Knowledge Base (decaying environment facts)
      4. Vector embedding indexing for cosine similarity agent queries
    """

    def __init__(
        self,
        robot_id: str,
        temporal_window_sec: float = 60.0,
        max_entities: int = 500,
        spatial_dedup_threshold_m: float = 0.35
    ):
        self.robot_id = robot_id
        self.temporal_window_sec = float(temporal_window_sec)
        self.max_entities = int(max_entities)
        self.spatial_dedup_threshold = float(spatial_dedup_threshold_m)

        # 1. Temporal rolling buffer of telemetry frames
        self._telemetry_history: deque[TelemetryFrame] = deque(maxlen=2000)

        # 2. Spatial Entities (entity_id -> SpatialEntity)
        self._spatial_entities: Dict[str, SpatialEntity] = {}

        # 3. Environment Facts (fact_id -> EnvironmentFact)
        self._facts: Dict[str, EnvironmentFact] = {}

        # 4. Context embeddings history: list of (embedding_np, AgentContextBlock)
        self._embedding_index: List[Tuple[np.ndarray, AgentContextBlock]] = []

        # Current conversational and intent state
        self._conversational_state: str = "IDLE"
        self._active_intent: str = "EXPLORATION"

    # ─────────────────────────────────────────────────────────────────────────
    # Telemetry Ingestion & Temporal Chunking
    # ─────────────────────────────────────────────────────────────────────────

    def ingest_telemetry(self, frame: TelemetryFrame) -> None:
        """Ingest high-frequency telemetry frame and prune stale temporal records."""
        self._telemetry_history.append(frame)
        self._prune_stale_history()

    def get_temporal_chunk(self, duration_sec: Optional[float] = None) -> List[TelemetryFrame]:
        """Retrieve recent window of telemetry frames for trajectory analysis."""
        window = duration_sec or self.temporal_window_sec
        cutoff = time.time() - window
        return [f for f in self._telemetry_history if f.timestamp_sec >= cutoff]

    def _prune_stale_history(self) -> None:
        cutoff = time.time() - self.temporal_window_sec
        while self._telemetry_history and self._telemetry_history[0].timestamp_sec < cutoff:
            self._telemetry_history.popleft()

    # ─────────────────────────────────────────────────────────────────────────
    # Spatial Entity Ingestion & Deduplication
    # ─────────────────────────────────────────────────────────────────────────

    def ingest_spatial_entity(
        self,
        label: str,
        position: Vector3D,
        bounding_box: Optional[Vector3D] = None,
        confidence: float = 1.0,
        attributes: Optional[Dict[str, Any]] = None
    ) -> SpatialEntity:
        """
        Ingest a detected physical object or landmark.
        Performs spatial deduplication: merges with existing entity if within distance threshold.
        """
        now = int(time.time())
        bbox = bounding_box or Vector3D(x=0.5, y=0.5, z=0.5)
        attrs = attributes or {}

        # Check for proximity merge
        matched_id = None
        for eid, existing in self._spatial_entities.items():
            if existing.label == label:
                dx = existing.position.x - position.x
                dy = existing.position.y - position.y
                dz = existing.position.z - position.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist <= self.spatial_dedup_threshold:
                    matched_id = eid
                    break

        if matched_id:
            # Update existing entity with moving average position and refreshed timestamp
            prev = self._spatial_entities[matched_id]
            alpha = 0.3
            new_pos = Vector3D(
                x=round(prev.position.x * (1 - alpha) + position.x * alpha, 4),
                y=round(prev.position.y * (1 - alpha) + position.y * alpha, 4),
                z=round(prev.position.z * (1 - alpha) + position.z * alpha, 4)
            )
            merged_attrs = {**prev.attributes, **attrs}
            updated = SpatialEntity(
                entity_id=prev.entity_id,
                label=label,
                position=new_pos,
                bounding_box=bbox,
                confidence=max(prev.confidence, confidence),
                first_seen_utc=prev.first_seen_utc,
                last_seen_utc=now,
                attributes=merged_attrs
            )
            self._spatial_entities[matched_id] = updated
            return updated
        else:
            # Create new entity
            eid = f"entity_{uuid.uuid4().hex[:8]}"
            entity = SpatialEntity(
                entity_id=eid,
                label=label,
                position=position,
                bounding_box=bbox,
                confidence=confidence,
                first_seen_utc=now,
                last_seen_utc=now,
                attributes=attrs
            )
            if len(self._spatial_entities) >= self.max_entities:
                # Evict oldest entity
                oldest_id = min(self._spatial_entities.keys(), key=lambda k: self._spatial_entities[k].last_seen_utc)
                del self._spatial_entities[oldest_id]

            self._spatial_entities[eid] = entity
            return entity

    def query_spatial_entities(
        self,
        center: Vector3D,
        radius_m: float,
        label_filter: Optional[str] = None
    ) -> List[SpatialEntity]:
        """Query all entities within spherical radius of a center point."""
        results = []
        for entity in self._spatial_entities.values():
            if label_filter and entity.label.lower() != label_filter.lower():
                continue
            dx = entity.position.x - center.x
            dy = entity.position.y - center.y
            dz = entity.position.z - center.z
            if math.sqrt(dx * dx + dy * dy + dz * dz) <= radius_m:
                results.append(entity)
        return results

    # ─────────────────────────────────────────────────────────────────────────
    # Environment Facts & Semantic Memory
    # ─────────────────────────────────────────────────────────────────────────

    def assert_fact(self, subject: str, predicate: str, obj: str, validity_sec: float = 3600.0) -> EnvironmentFact:
        """Register a temporal semantic fact (e.g., 'door_3', 'is_state', 'LOCKED')."""
        fid = f"fact_{uuid.uuid4().hex[:8]}"
        fact = EnvironmentFact(
            fact_id=fid,
            subject=subject,
            predicate=predicate,
            object=obj,
            temporal_validity_sec=validity_sec,
            timestamp=time.time()
        )
        self._facts[fid] = fact
        return fact

    def get_valid_facts(self) -> List[EnvironmentFact]:
        """Return all non-expired environment facts."""
        now = time.time()
        valid = []
        expired = []
        for fid, f in self._facts.items():
            if now - f.timestamp <= f.temporal_validity_sec:
                valid.append(f)
            else:
                expired.append(fid)
        for fid in expired:
            del self._facts[fid]
        return valid

    # ─────────────────────────────────────────────────────────────────────────
    # Context Snapshot & Embedding Indexing
    # ─────────────────────────────────────────────────────────────────────────

    def create_context_block(
        self,
        embedding: Optional[List[float]] = None,
        conversational_state: Optional[str] = None,
        active_intent: Optional[str] = None
    ) -> AgentContextBlock:
        """Snapshot current robot state, entities, and active facts into an immutable Context Block."""
        latest_pose = Pose3D()
        if self._telemetry_history:
            latest_pose = self._telemetry_history[-1].pose

        if conversational_state:
            self._conversational_state = conversational_state
        if active_intent:
            self._active_intent = active_intent

        block = AgentContextBlock(
            context_id=f"ctx_{uuid.uuid4().hex[:12]}",
            robot_id=self.robot_id,
            timestamp_sec=time.time(),
            current_pose=latest_pose,
            detected_entities=list(self._spatial_entities.values()),
            facts=self.get_valid_facts(),
            embedding_vector=embedding or [],
            conversational_state=self._conversational_state,
            active_intent=self._active_intent
        )

        if embedding and len(embedding) > 0:
            arr = np.array(embedding, dtype=np.float32)
            norm = np.linalg.norm(arr)
            if norm > 1e-6:
                arr = arr / norm
            self._embedding_index.append((arr, block))
            if len(self._embedding_index) > 500:
                self._embedding_index.pop(0)

        return block

    def query_similar_contexts(self, query_embedding: List[float], top_k: int = 3) -> List[Tuple[float, AgentContextBlock]]:
        """Retrieve top_k most similar historical contexts using vector cosine similarity."""
        if not self._embedding_index or not query_embedding:
            return []

        q = np.array(query_embedding, dtype=np.float32)
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-6:
            q = q / q_norm
        else:
            return []

        scores = []
        for emb, block in self._embedding_index:
            sim = float(np.dot(q, emb))
            scores.append((sim, block))

        scores.sort(key=lambda x: x[0], reverse=True)
        return scores[:top_k]
