"""
Unit tests for FastAPI app factory

Tests the create_vla_app function and its endpoints.
"""

import numpy as np
from fastapi.testclient import TestClient
from vla_server_base.adapters.base import VLAModelAdapter
from vla_server_base.app_factory import create_vla_app


class MockVLAAdapter(VLAModelAdapter):
    """Mock VLA adapter for testing"""

    def __init__(self, should_fail=False):
        super().__init__()
        self.model_id = "mock-test"
        self.device = "cpu"
        self.model_loaded = True
        self.should_fail = should_fail

    def load_model(self, model_id: str, device: str = "cpu", cache_dir: str = "/tmp"):
        """Mock load - already loaded"""
        pass

    def preprocess_observation(self, obs: dict) -> dict:
        """Pass through observation"""
        if self.should_fail:
            raise ValueError("Preprocessing failed")
        return obs

    def preprocess_instruction(self, instruction: str) -> str:
        """Pass through instruction"""
        return instruction

    def predict(self, obs: dict, instruction: str) -> np.ndarray:
        """Return random 8-dim action"""
        if self.should_fail:
            raise RuntimeError("Prediction failed")
        return np.random.randn(8).astype(np.float32)

    def postprocess_action(self, action: np.ndarray) -> list[float]:
        """Convert to list"""
        return action.tolist()


# Simple 8-DOF robot XML for testing
TEST_XML = """
<mujoco model="test_robot">
  <compiler angle="radian"/>
  <option timestep="0.002"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" pos="0 0 0" size="0 0 0.05" type="plane"/>

    <body name="base" pos="0 0 0">
      <inertial pos="0 0 0" mass="4" diaginertia="0.4 0.4 0.4"/>
      <geom type="box" size="0.1 0.1 0.1"/>

      <body name="link1" pos="0 0 0.2">
        <joint name="joint1" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <geom type="cylinder" size="0.05 0.1"/>

        <body name="link2" pos="0 0 0.2">
          <joint name="joint2" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="cylinder" size="0.04 0.1"/>

          <body name="link3" pos="0 0 0.2">
            <joint name="joint3" type="hinge" axis="0 0 1"/>
            <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
            <geom type="box" size="0.02 0.02 0.02"/>

            <body name="finger1" pos="0.015 0 0">
              <joint name="joint4" type="slide" axis="1 0 0"/>
              <inertial pos="0 0 0" mass="0.1" diaginertia="0.01 0.01 0.01"/>
              <geom type="box" size="0.005 0.01 0.03"/>
            </body>

            <body name="finger2" pos="-0.015 0 0">
              <joint name="joint5" type="slide" axis="-1 0 0"/>
              <inertial pos="0 0 0" mass="0.1" diaginertia="0.01 0.01 0.01"/>
              <geom type="box" size="0.005 0.01 0.03"/>
            </body>

            <body name="dummy6">
              <joint name="joint6" type="slide" axis="0 0 1" range="-1 1"/>
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>
              <geom type="sphere" size="0.001"/>
            </body>

            <body name="dummy7">
              <joint name="joint7" type="slide" axis="0 0 1" range="-1 1"/>
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>
              <geom type="sphere" size="0.001"/>
            </body>

            <body name="dummy8">
              <joint name="joint8" type="slide" axis="0 0 1" range="-1 1"/>
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>
              <geom type="sphere" size="0.001"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="motor1" joint="joint1"/>
    <motor name="motor2" joint="joint2"/>
    <motor name="motor3" joint="joint3"/>
    <motor name="motor4" joint="joint4"/>
    <motor name="motor5" joint="joint5"/>
    <motor name="motor6" joint="joint6"/>
    <motor name="motor7" joint="joint7"/>
    <motor name="motor8" joint="joint8"/>
  </actuator>
</mujoco>
"""


class TestCreateVLAApp:
    """Test create_vla_app function"""

    def test_creates_fastapi_app(self):
        """Test create_vla_app returns FastAPI instance"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", "1.0.0", max_steps=5)

        assert app is not None
        assert app.title == "Test Service"
        assert app.version == "1.0.0"

    def test_health_endpoint_with_loaded_model(self):
        """Test /health returns healthy when model is loaded"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", max_steps=5)
        client = TestClient(app)

        response = client.get("/health")

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert data["model"] == "mock-test"
        assert data["model_status"] == "loaded"

    def test_health_endpoint_with_unloaded_model(self):
        """Test /health returns unhealthy when model is not loaded"""
        adapter = MockVLAAdapter()
        adapter.model_loaded = False
        adapter.model_id = None

        app = create_vla_app(adapter, "Test Service", max_steps=5)
        client = TestClient(app)

        response = client.get("/health")

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "unhealthy"
        assert data["model"] == "unknown"
        assert data["model_status"] == "not_loaded"

    def test_info_endpoint(self):
        """Test /info returns correct service information"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", "1.2.3", max_steps=50)
        client = TestClient(app)

        response = client.get("/info")

        assert response.status_code == 200
        data = response.json()
        assert data["service_name"] == "Test Service"
        assert data["service_version"] == "1.2.3"
        assert data["model_id"] == "mock-test"
        assert data["device"] == "cpu"
        assert data["loaded"] is True
        assert data["max_steps"] == 50

    def test_execute_endpoint_success(self):
        """Test /execute with valid request"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", max_steps=3)
        client = TestClient(app)

        payload = {"instruction": "Pick up the cube", "xml_string": TEST_XML}

        response = client.post("/execute", json=payload)

        assert response.status_code == 200
        data = response.json()
        assert "actions" in data
        assert "states" in data
        assert "duration_ms" in data
        assert len(data["actions"]) == 3
        assert len(data["states"]) == 3

    def test_execute_endpoint_validates_request_schema(self):
        """Test /execute validates request schema"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", max_steps=5)
        client = TestClient(app)

        # Missing instruction
        payload = {"xml_string": TEST_XML}
        response = client.post("/execute", json=payload)
        assert response.status_code == 422

        # Missing xml_string
        payload = {"instruction": "Pick up the cube"}
        response = client.post("/execute", json=payload)
        assert response.status_code == 422

    def test_execute_endpoint_returns_states_with_correct_format(self):
        """Test /execute returns states with qpos, qvel, time"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", max_steps=2)
        client = TestClient(app)

        payload = {"instruction": "Pick up the cube", "xml_string": TEST_XML}

        response = client.post("/execute", json=payload)

        assert response.status_code == 200
        data = response.json()

        for state in data["states"]:
            assert "qpos" in state
            assert "qvel" in state
            assert "time" in state
            assert isinstance(state["qpos"], list)
            assert isinstance(state["qvel"], list)
            assert isinstance(state["time"], float)

    def test_execute_endpoint_handles_invalid_xml(self):
        """Test /execute returns 400 for invalid XML (ValueError)"""
        adapter = MockVLAAdapter()
        app = create_vla_app(adapter, "Test Service", max_steps=5)
        client = TestClient(app)

        payload = {
            "instruction": "Pick up the cube",
            "xml_string": "<invalid>xml</invalid>",
        }

        response = client.post("/execute", json=payload)

        # Invalid XML raises ValueError, which returns 400
        assert response.status_code == 400
        assert "detail" in response.json()

    def test_execute_endpoint_handles_adapter_errors(self):
        """Test /execute returns 400 for ValueError, 500 for RuntimeError"""
        # Test ValueError (preprocessing failed) -> 400
        adapter_value_error = MockVLAAdapter(should_fail=True)
        app = create_vla_app(adapter_value_error, "Test Service", max_steps=5)
        client = TestClient(app)

        payload = {"instruction": "Pick up the cube", "xml_string": TEST_XML}

        response = client.post("/execute", json=payload)

        # Preprocessing raises ValueError -> 400
        assert response.status_code == 400
        assert "detail" in response.json()

    def test_execute_endpoint_respects_max_steps(self):
        """Test /execute respects max_steps parameter"""
        adapter = MockVLAAdapter()
        max_steps = 7
        app = create_vla_app(adapter, "Test Service", max_steps=max_steps)
        client = TestClient(app)

        payload = {"instruction": "Pick up the cube", "xml_string": TEST_XML}

        response = client.post("/execute", json=payload)

        assert response.status_code == 200
        data = response.json()
        assert len(data["actions"]) == max_steps
        assert len(data["states"]) == max_steps
