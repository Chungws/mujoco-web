"""
VLA Service HTTP Client

Communicates with VLA inference services (mock, octo-small, smolvla)
via HTTP to generate episodes.

Architecture:
- Backend (this client) → HTTP POST /predict → VLA Service
- VLA Service runs MuJoCo simulation + VLA inference
- Returns episode data (actions, states, duration_ms)
"""

import logging
from typing import Any

import httpx
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)


class VLAServiceClient:
    """
    HTTP client for VLA inference services

    Handles communication with VLA servers running as separate microservices.
    Each VLA model (mock, octo-small, smolvla) runs on its own port.
    """

    # Timeout configuration
    DEFAULT_TIMEOUT = 120.0  # 2 minutes for episode generation
    CONNECT_TIMEOUT = 10.0  # 10 seconds to establish connection

    def __init__(self, model_endpoints: dict[str, str]):
        """
        Initialize VLA service client

        Args:
            model_endpoints: Mapping of model_id -> base_url
                Example: {"mock-vla": "http://localhost:8001",
                          "octo-small": "http://localhost:8002"}
        """
        self.model_endpoints = model_endpoints
        logger.info(f"Initialized VLAServiceClient with {len(model_endpoints)} endpoints")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10),
        reraise=True,
    )
    async def generate_episode(
        self,
        model_id: str,
        instruction: str,
        robot_id: str,
        scene_id: str,
    ) -> dict[str, Any]:
        """
        Generate episode by calling VLA service via HTTP

        Args:
            model_id: VLA model identifier (e.g., "octo-small")
            instruction: Natural language instruction
            robot_id: Robot identifier (e.g., "franka")
            scene_id: Scene identifier (e.g., "table")

        Returns:
            Dictionary containing:
            - actions: List of 8-dim action vectors
            - states: List of state dicts (qpos, qvel, time)
            - duration_ms: Execution duration in milliseconds

        Raises:
            ValueError: If model_id not found in endpoints
            httpx.HTTPError: If HTTP request fails
            httpx.TimeoutException: If request times out
        """
        # Get endpoint for this model
        base_url = self.model_endpoints.get(model_id)
        if not base_url:
            raise ValueError(
                f"Model endpoint not found: {model_id}. "
                f"Available: {list(self.model_endpoints.keys())}"
            )

        endpoint = f"{base_url}/predict"

        # Prepare request payload
        # VLA services expect: instruction + xml_string
        # We'll compose xml_string from robot_id + scene_id
        xml_string = self._compose_xml_string(robot_id, scene_id)

        payload = {
            "instruction": instruction,
            "xml_string": xml_string,
        }

        logger.info(
            f"Calling VLA service: model={model_id}, endpoint={endpoint}, "
            f"instruction={instruction[:50]}..."
        )

        # Make HTTP request
        timeout = httpx.Timeout(
            connect=self.CONNECT_TIMEOUT,
            read=self.DEFAULT_TIMEOUT,
            write=self.DEFAULT_TIMEOUT,
            pool=self.DEFAULT_TIMEOUT,
        )

        async with httpx.AsyncClient(timeout=timeout) as client:
            try:
                response = await client.post(endpoint, json=payload)
                response.raise_for_status()

                data = response.json()

                logger.info(
                    f"VLA service response received: model={model_id}, "
                    f"status={response.status_code}"
                )

                # Transform response to match expected format
                # VLA services return: {"action": [8-dim], ...}
                # We need to aggregate into episode format for compatibility
                return self._transform_response(data)

            except httpx.TimeoutException as e:
                logger.error(f"VLA service timeout: {endpoint}, error={e}")
                raise

            except httpx.HTTPStatusError as e:
                logger.error(
                    f"VLA service HTTP error: {endpoint}, "
                    f"status={e.response.status_code}, error={e}"
                )
                raise

            except Exception as e:
                logger.error(f"VLA service error: {endpoint}, error={e}")
                raise

    def _compose_xml_string(self, robot_id: str, scene_id: str) -> str:
        """
        Compose MuJoCo XML string from robot and scene

        This is a temporary implementation. In production, this should:
        1. Load template.xml from config/mujoco/
        2. Load robot XML from config/mujoco/robots/{robot_id}.xml
        3. Load scene XML from config/mujoco/scenes/{scene_id}.xml
        4. Compose final XML using vla-server-base logic

        For now, return a minimal test XML.

        Args:
            robot_id: Robot identifier
            scene_id: Scene identifier

        Returns:
            Complete MuJoCo XML string
        """
        # TODO: Implement proper XML composition from config files
        # This is temporary for initial integration
        return (
            "<mujoco>"
            "<worldbody>"
            f"<body name='test_{robot_id}_{scene_id}' pos='0 0 0'>"
            "<geom type='sphere' size='0.1'/>"
            "</body>"
            "</worldbody>"
            "</mujoco>"
        )

    def _transform_response(self, vla_response: dict[str, Any]) -> dict[str, Any]:
        """
        Transform VLA service response to episode format

        VLA services currently return single predictions:
        {"action": [8-dim floats], ...}

        But TurnService expects episode format:
        {"actions": [[...], [...]], "states": [...], "duration_ms": 123}

        This is a temporary adapter. Once we implement full episode execution
        in VLA services (PR upcoming), this transformation won't be needed.

        Args:
            vla_response: Response from VLA service

        Returns:
            Episode format dictionary
        """
        # For now, create a single-step episode from the prediction
        # TODO: Remove this once VLA services return full episodes
        action = vla_response.get("action", [0.0] * 8)

        # Mock episode with single step
        return {
            "actions": [action],  # Single action as list
            "states": [
                {
                    "qpos": [0.0] * 7,
                    "qvel": [0.0] * 7,
                    "time": 0.0,
                }
            ],
            "duration_ms": 100,  # Mock duration
        }
