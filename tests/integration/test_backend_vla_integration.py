"""
Backend → VLA Server → MongoDB Full Flow Integration Tests

Tests the complete flow from Backend API → VLA Server → MongoDB Episode storage.
This is the core integration test for the VLA Arena MVP.

Prerequisites:
- Backend service running on http://localhost:8000
- Mock VLA service running on http://localhost:8001
- Octo-Small VLA service running on http://localhost:8002 (optional)
- PostgreSQL database running
- MongoDB running

Run with: pytest tests/integration/ --run-integration -v
"""

import time

import httpx
import pytest
from motor.motor_asyncio import AsyncIOMotorClient
from vlaarena_shared.config import settings

# Service endpoints
BACKEND_URL = "http://localhost:8000"
MOCK_VLA_URL = "http://localhost:8001"
OCTO_SMALL_VLA_URL = "http://localhost:8002"

# Mark all tests in this file as integration tests
pytestmark = pytest.mark.integration


@pytest.fixture
async def mongodb_client():
    """MongoDB client for verifying episode storage"""
    client = AsyncIOMotorClient(settings.MONGODB_URI)
    yield client
    client.close()


@pytest.fixture
async def mongodb_episodes_collection(mongodb_client):
    """Get episodes collection"""
    db = mongodb_client[settings.MONGO_DATABASE]
    return db.episodes


@pytest.mark.asyncio
class TestBackendVLAIntegration:
    """Test Backend → VLA Server → MongoDB full flow"""

    async def test_session_creation_via_backend(self):
        """Test session creation via Backend API"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(
                f"{BACKEND_URL}/api/sessions/init",
                json={"robot_id": "franka_panda", "scene_id": "table_pick_place"},
            )

            assert response.status_code == 201
            data = response.json()
            assert "session_id" in data
            assert "battle_id" in data
            assert data["session_id"].startswith("sess_")
            assert data["battle_id"].startswith("battle_")

    async def test_turn_creation_with_mock_vla(self, mongodb_episodes_collection):
        """
        Test full flow: Backend → Mock VLA Server → MongoDB

        Flow:
        1. Create session via Backend
        2. Create turn (user provides instruction)
        3. Backend calls Mock VLA Server (both left and right models)
        4. Episodes saved to MongoDB
        5. Verify episode data in MongoDB
        """
        async with httpx.AsyncClient(timeout=60.0) as client:
            # Step 1: Create session
            session_response = await client.post(
                f"{BACKEND_URL}/api/sessions/init",
                json={"robot_id": "franka_panda", "scene_id": "table_pick_place"},
            )
            assert session_response.status_code == 201
            battle_id = session_response.json()["battle_id"]

            # Step 2: Create turn with instruction
            turn_response = await client.post(
                f"{BACKEND_URL}/api/battles/{battle_id}/turns",
                json={"instruction": "Pick up the red cube"},
            )

            # Step 3: Verify turn creation success
            assert turn_response.status_code == 201
            turn_data = turn_response.json()
            assert "turn_id" in turn_data
            assert "left_episode_id" in turn_data
            assert "right_episode_id" in turn_data
            assert turn_data["status"] == "completed"

            left_episode_id = turn_data["left_episode_id"]
            right_episode_id = turn_data["right_episode_id"]

            # Step 4: Verify episodes exist in MongoDB
            left_episode = await mongodb_episodes_collection.find_one(
                {"episode_id": left_episode_id}
            )
            right_episode = await mongodb_episodes_collection.find_one(
                {"episode_id": right_episode_id}
            )

            # Step 5: Verify episode data structure
            assert left_episode is not None, "Left episode not found in MongoDB"
            assert right_episode is not None, "Right episode not found in MongoDB"

            # Verify episode fields
            for episode in [left_episode, right_episode]:
                assert "episode_id" in episode
                assert "battle_id" in episode
                assert "turn_id" in episode
                assert "side" in episode
                assert "model_id" in episode
                assert "actions" in episode
                assert "states" in episode
                assert "duration_ms" in episode

                # Verify actions structure
                assert isinstance(episode["actions"], list)
                assert len(episode["actions"]) > 0
                for action in episode["actions"]:
                    assert isinstance(action, list)
                    assert len(action) == 8  # 7 joints + gripper

                # Verify states structure
                assert isinstance(episode["states"], list)
                assert len(episode["states"]) == len(episode["actions"])
                for state in episode["states"]:
                    assert "qpos" in state
                    assert "qvel" in state
                    assert "time" in state

            # Verify left and right episodes are different
            assert left_episode["side"] != right_episode["side"]
            assert left_episode["model_id"] in ["mock-vla", "octo-small"]
            assert right_episode["model_id"] in ["mock-vla", "octo-small"]

    async def test_episode_generation_performance(self, mongodb_episodes_collection):
        """
        Test episode generation performance: < 60s

        Performance requirement from ROADMAP.md:
        - Episode generation should complete in < 60 seconds
        """
        async with httpx.AsyncClient(timeout=120.0) as client:
            # Create session
            session_response = await client.post(
                f"{BACKEND_URL}/api/sessions/init",
                json={"robot_id": "franka_panda", "scene_id": "table_pick_place"},
            )
            assert session_response.status_code == 201
            battle_id = session_response.json()["battle_id"]

            # Measure turn creation time (includes 2 episode generations)
            start_time = time.time()

            turn_response = await client.post(
                f"{BACKEND_URL}/api/battles/{battle_id}/turns",
                json={"instruction": "Pick up the red cube and place it in the bin"},
            )

            duration = time.time() - start_time

            # Verify success
            assert turn_response.status_code == 201
            turn_data = turn_response.json()
            assert turn_data["status"] == "completed"

            # Verify performance (< 60s for 2 episodes)
            assert duration < 60.0, f"Episode generation took {duration:.2f}s (> 60s)"

            # Verify episodes in MongoDB
            left_episode_id = turn_data["left_episode_id"]
            right_episode_id = turn_data["right_episode_id"]

            left_episode = await mongodb_episodes_collection.find_one(
                {"episode_id": left_episode_id}
            )
            right_episode = await mongodb_episodes_collection.find_one(
                {"episode_id": right_episode_id}
            )

            assert left_episode is not None
            assert right_episode is not None

            # Log performance metrics
            print("\n[Performance Metrics]")
            print(f"Total duration: {duration:.2f}s")
            print(f"Left episode steps: {len(left_episode['actions'])}")
            print(f"Right episode steps: {len(right_episode['actions'])}")
            print(f"Left episode duration: {left_episode['duration_ms']}ms")
            print(f"Right episode duration: {right_episode['duration_ms']}ms")

    async def test_multi_turn_battle_flow(self, mongodb_episodes_collection):
        """
        Test multi-turn battle: multiple instructions in same session

        Flow:
        1. Create session
        2. Create turn 1
        3. Create turn 2
        4. Verify both turns have different episodes
        5. Verify all episodes stored in MongoDB
        """
        async with httpx.AsyncClient(timeout=120.0) as client:
            # Step 1: Create session
            session_response = await client.post(
                f"{BACKEND_URL}/api/sessions/init",
                json={"robot_id": "franka_panda", "scene_id": "table_pick_place"},
            )
            assert session_response.status_code == 201
            battle_id = session_response.json()["battle_id"]

            # Step 2: Create turn 1
            turn1_response = await client.post(
                f"{BACKEND_URL}/api/battles/{battle_id}/turns",
                json={"instruction": "Pick up the red cube"},
            )
            assert turn1_response.status_code == 201
            turn1_data = turn1_response.json()

            # Step 3: Create turn 2
            turn2_response = await client.post(
                f"{BACKEND_URL}/api/battles/{battle_id}/turns",
                json={"instruction": "Place the cube in the bin"},
            )
            assert turn2_response.status_code == 201
            turn2_data = turn2_response.json()

            # Step 4: Verify different turn_ids and episode_ids
            assert turn1_data["turn_id"] != turn2_data["turn_id"]
            assert turn1_data["left_episode_id"] != turn2_data["left_episode_id"]
            assert turn1_data["right_episode_id"] != turn2_data["right_episode_id"]

            # Step 5: Verify all 4 episodes in MongoDB
            episode_ids = [
                turn1_data["left_episode_id"],
                turn1_data["right_episode_id"],
                turn2_data["left_episode_id"],
                turn2_data["right_episode_id"],
            ]

            for episode_id in episode_ids:
                episode = await mongodb_episodes_collection.find_one({"episode_id": episode_id})
                assert episode is not None, f"Episode {episode_id} not found in MongoDB"
                assert episode["battle_id"] == battle_id

    async def test_error_handling_vla_server_down(self):
        """
        Test error handling when VLA server is unavailable

        Scenario:
        1. Backend tries to create turn
        2. VLA server is down (or non-existent endpoint)
        3. Backend should return appropriate error
        """
        async with httpx.AsyncClient(timeout=30.0) as client:
            # Create session (should succeed)
            session_response = await client.post(
                f"{BACKEND_URL}/api/sessions/init",
                json={"robot_id": "franka_panda", "scene_id": "table_pick_place"},
            )

            # If session creation succeeds, battle_id is available
            if session_response.status_code == 201:
                battle_id = session_response.json()["battle_id"]

                # Try to create turn (may fail if VLA server down)
                # This test verifies graceful error handling
                turn_response = await client.post(
                    f"{BACKEND_URL}/api/battles/{battle_id}/turns",
                    json={"instruction": "Pick up the red cube"},
                )

                # If VLA server is down, should get 500 or 503
                # If VLA server is up, should get 201
                # Either is acceptable for this test - we're just verifying no crash
                assert turn_response.status_code in [201, 500, 503]


@pytest.mark.asyncio
class TestVLAServerDirectAccess:
    """Test VLA Server endpoints directly (for debugging)"""

    async def test_mock_vla_health(self):
        """Test Mock VLA Server health check"""
        async with httpx.AsyncClient(timeout=10.0) as client:
            try:
                response = await client.get(f"{MOCK_VLA_URL}/health")
                assert response.status_code == 200
                data = response.json()
                assert data["status"] == "healthy"
                assert data["model"] == "mock-vla"
            except httpx.ConnectError:
                pytest.skip("Mock VLA Server not running")

    async def test_mock_vla_predict(self):
        """Test Mock VLA Server predict endpoint"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            try:
                # Simple XML for testing
                test_xml = (
                    "<mujoco><worldbody>"
                    "<body name='test' pos='0 0 0'><geom type='sphere' size='0.1'/></body>"
                    "</worldbody></mujoco>"
                )

                response = await client.post(
                    f"{MOCK_VLA_URL}/predict",
                    json={"instruction": "Pick up the red cube", "xml_string": test_xml},
                )
                assert response.status_code == 200
                data = response.json()
                assert "action" in data
                assert isinstance(data["action"], list)
                assert len(data["action"]) == 8
            except httpx.ConnectError:
                pytest.skip("Mock VLA Server not running")


# Run with:
# pytest tests/integration/test_backend_vla_integration.py --run-integration -v -s
