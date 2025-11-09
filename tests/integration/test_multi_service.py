"""
Integration tests for multiple VLA services running together

Tests the mock and octo-small services running in parallel,
service communication patterns, error scenarios, and performance.

Prerequisites:
- Mock service running on http://localhost:8001
- Octo-Small service running on http://localhost:8002

Run with: pytest tests/integration/ --run-integration -v
"""

import asyncio
import time

import httpx
import pytest

# Service endpoints
MOCK_SERVICE_URL = "http://localhost:8001"
OCTO_SMALL_SERVICE_URL = "http://localhost:8002"

# Test data
SAMPLE_XML = "<mujoco><worldbody><body name='test' pos='0 0 0'><geom type='sphere' size='0.1'/></body></worldbody></mujoco>"
SAMPLE_INSTRUCTION = "Pick up the red cube"


# Mark all tests in this file as integration tests
pytestmark = pytest.mark.integration


@pytest.mark.asyncio
class TestMultiServiceHealthChecks:
    """Test health checks for all services"""

    async def test_mock_service_health(self):
        """Test mock service health endpoint"""
        async with httpx.AsyncClient() as client:
            response = await client.get(f"{MOCK_SERVICE_URL}/health")
            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "healthy"
            assert data["model"] == "mock-vla"

    async def test_octo_small_service_health(self):
        """Test octo-small service health endpoint"""
        async with httpx.AsyncClient() as client:
            response = await client.get(f"{OCTO_SMALL_SERVICE_URL}/health")
            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "healthy"
            assert data["model"] == "octo-small"
            assert data["model_status"] == "loaded"

    async def test_all_services_healthy_concurrent(self):
        """Test all services health endpoints concurrently"""
        async with httpx.AsyncClient() as client:
            tasks = [
                client.get(f"{MOCK_SERVICE_URL}/health"),
                client.get(f"{OCTO_SMALL_SERVICE_URL}/health"),
            ]
            responses = await asyncio.gather(*tasks)

            # All should return 200
            for response in responses:
                assert response.status_code == 200
                data = response.json()
                assert data["status"] == "healthy"


@pytest.mark.asyncio
class TestMultiServiceInfo:
    """Test service info endpoints"""

    async def test_mock_service_info(self):
        """Test mock service info endpoint"""
        async with httpx.AsyncClient() as client:
            response = await client.get(f"{MOCK_SERVICE_URL}/info")
            assert response.status_code == 200
            data = response.json()
            assert data["model_id"] == "mock-vla"
            assert data["device"] == "cpu"
            assert data["loaded"] is True
            assert data["type"] == "mock"

    async def test_octo_small_service_info(self):
        """Test octo-small service info endpoint"""
        async with httpx.AsyncClient() as client:
            response = await client.get(f"{OCTO_SMALL_SERVICE_URL}/info")
            assert response.status_code == 200
            data = response.json()
            assert data["model_id"] == "octo-small"
            assert data["device"] == "cpu"
            assert data["loaded"] is True
            assert data["type"] == "octo-small"
            assert data["parameters"] == "27M"
            assert data["framework"] == "JAX"

    async def test_all_services_info_concurrent(self):
        """Test all services info endpoints concurrently"""
        async with httpx.AsyncClient() as client:
            tasks = [
                client.get(f"{MOCK_SERVICE_URL}/info"),
                client.get(f"{OCTO_SMALL_SERVICE_URL}/info"),
            ]
            responses = await asyncio.gather(*tasks)

            # All should return 200
            for response in responses:
                assert response.status_code == 200
                data = response.json()
                assert "model_id" in data
                assert "loaded" in data
                assert data["loaded"] is True


@pytest.mark.asyncio
class TestMultiServicePrediction:
    """Test prediction endpoints for all services"""

    async def test_mock_service_prediction(self):
        """Test mock service prediction endpoint"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}
            response = await client.post(f"{MOCK_SERVICE_URL}/predict", json=payload)
            assert response.status_code == 200
            data = response.json()
            assert "action" in data
            assert isinstance(data["action"], list)
            assert len(data["action"]) == 8  # 7 joints + gripper

    async def test_octo_small_service_prediction(self):
        """Test octo-small service prediction endpoint"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}
            response = await client.post(f"{OCTO_SMALL_SERVICE_URL}/predict", json=payload)
            assert response.status_code == 200
            data = response.json()
            assert "action" in data
            assert isinstance(data["action"], list)
            assert len(data["action"]) == 8  # 7 joints + gripper

    async def test_all_services_prediction_concurrent(self):
        """Test all services prediction endpoints concurrently"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}
            tasks = [
                client.post(f"{MOCK_SERVICE_URL}/predict", json=payload),
                client.post(f"{OCTO_SMALL_SERVICE_URL}/predict", json=payload),
            ]
            responses = await asyncio.gather(*tasks)

            # All should return 200
            for response in responses:
                assert response.status_code == 200
                data = response.json()
                assert "action" in data
                assert isinstance(data["action"], list)
                assert len(data["action"]) == 8

    async def test_different_models_different_actions(self):
        """Test that different models produce different actions"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}

            # Get predictions from both services
            mock_response = await client.post(f"{MOCK_SERVICE_URL}/predict", json=payload)
            octo_response = await client.post(f"{OCTO_SMALL_SERVICE_URL}/predict", json=payload)

            assert mock_response.status_code == 200
            assert octo_response.status_code == 200

            mock_action = mock_response.json()["action"]
            octo_action = octo_response.json()["action"]

            # Actions should be different (different models)
            assert mock_action != octo_action


@pytest.mark.asyncio
class TestMultiServiceErrorScenarios:
    """Test error scenarios"""

    async def test_invalid_request_missing_instruction(self):
        """Test prediction with missing instruction field"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"xml_string": SAMPLE_XML}

            # Test both services
            for url in [MOCK_SERVICE_URL, OCTO_SMALL_SERVICE_URL]:
                response = await client.post(f"{url}/predict", json=payload)
                assert response.status_code == 422  # Validation error

    async def test_invalid_request_missing_xml(self):
        """Test prediction with missing xml_string field"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION}

            # Test both services
            for url in [MOCK_SERVICE_URL, OCTO_SMALL_SERVICE_URL]:
                response = await client.post(f"{url}/predict", json=payload)
                assert response.status_code == 422  # Validation error

    async def test_invalid_xml_format(self):
        """Test prediction with invalid XML"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": "invalid xml <>"}

            # Test both services - should handle error gracefully
            for url in [MOCK_SERVICE_URL, OCTO_SMALL_SERVICE_URL]:
                response = await client.post(f"{url}/predict", json=payload)
                # Should either return 400 or 500 depending on error handling
                assert response.status_code in [400, 500]


@pytest.mark.asyncio
class TestMultiServicePerformance:
    """Test performance characteristics"""

    async def test_mock_service_response_time(self):
        """Test mock service response time"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}

            start = time.time()
            response = await client.post(f"{MOCK_SERVICE_URL}/predict", json=payload)
            duration = time.time() - start

            assert response.status_code == 200
            # Mock should be very fast (< 1 second)
            assert duration < 1.0

    async def test_octo_small_service_response_time(self):
        """Test octo-small service response time (baseline)"""
        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}

            start = time.time()
            response = await client.post(f"{OCTO_SMALL_SERVICE_URL}/predict", json=payload)
            duration = time.time() - start

            assert response.status_code == 200
            # Octo-Small should be reasonably fast (< 5 seconds for single prediction)
            assert duration < 5.0

    async def test_concurrent_predictions_performance(self):
        """Test performance of concurrent predictions"""
        async with httpx.AsyncClient(timeout=60.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}

            # Run 5 predictions concurrently on each service
            tasks = []
            for _ in range(5):
                tasks.append(client.post(f"{MOCK_SERVICE_URL}/predict", json=payload))
                tasks.append(client.post(f"{OCTO_SMALL_SERVICE_URL}/predict", json=payload))

            start = time.time()
            responses = await asyncio.gather(*tasks)
            duration = time.time() - start

            # All should succeed
            for response in responses:
                assert response.status_code == 200

            # Should complete in reasonable time (< 30 seconds for 10 total predictions)
            assert duration < 30.0


@pytest.mark.asyncio
class TestServiceCommunicationPatterns:
    """Test service-to-service communication patterns"""

    async def test_round_robin_load_distribution(self):
        """Test distributing requests in round-robin fashion"""
        services = [MOCK_SERVICE_URL, OCTO_SMALL_SERVICE_URL]

        async with httpx.AsyncClient(timeout=30.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}

            # Send 10 requests in round-robin
            results = []
            for i in range(10):
                service_url = services[i % len(services)]
                response = await client.post(f"{service_url}/predict", json=payload)
                assert response.status_code == 200
                results.append({"service": service_url, "action": response.json()["action"]})

            # Should have used both services
            mock_count = sum(1 for r in results if r["service"] == MOCK_SERVICE_URL)
            octo_count = sum(1 for r in results if r["service"] == OCTO_SMALL_SERVICE_URL)
            assert mock_count == 5
            assert octo_count == 5

    async def test_failover_scenario(self):
        """Test failover when one service is slow/unavailable"""
        async with httpx.AsyncClient(timeout=2.0) as client:
            payload = {"instruction": SAMPLE_INSTRUCTION, "xml_string": SAMPLE_XML}

            # Try to call a non-existent service (should fail)
            try:
                await client.post("http://localhost:9999/predict", json=payload)
                raise AssertionError("Should have raised exception")
            except (httpx.ConnectError, httpx.TimeoutException):
                pass  # Expected

            # But other services should still work
            response = await client.post(f"{MOCK_SERVICE_URL}/predict", json=payload)
            assert response.status_code == 200


# Run with: pytest tests/integration/test_multi_service.py -v -s
