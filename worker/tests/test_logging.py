"""
Tests for logging configuration
"""

import logging

from vlaarena_worker.logging_config import setup_logging


class TestLoggingConfiguration:
    """Test logging setup"""

    def test_setup_logging_creates_logger(self):
        """Test that setup_logging creates a logger"""
        # Act
        logger = setup_logging("test_logger_1")

        # Assert
        assert isinstance(logger, logging.Logger)
        assert logger.name == "test_logger_1"

    def test_setup_logging_sets_log_level(self):
        """Test that setup_logging sets INFO level by default"""
        # Act
        logger = setup_logging("test_logger_2")

        # Assert
        assert logger.level == logging.INFO

    def test_setup_logging_adds_stream_handler(self):
        """Test that setup_logging adds StreamHandler (stdout)"""
        # Act
        logger = setup_logging("test_logger_3")

        # Assert
        # Should have at least one handler
        assert len(logger.handlers) > 0

        # Should have StreamHandler
        stream_handlers = [h for h in logger.handlers if isinstance(h, logging.StreamHandler)]
        assert len(stream_handlers) > 0

    def test_setup_logging_uses_correct_format(self):
        """Test that log messages use correct format"""
        # Act
        logger = setup_logging("test_logger_4")

        # Assert
        # Get the first StreamHandler
        stream_handler = next(
            (h for h in logger.handlers if isinstance(h, logging.StreamHandler)), None
        )
        assert stream_handler is not None

        # Check format includes timestamp, level, and message
        formatter = stream_handler.formatter
        assert formatter is not None
        # Format should include asctime, levelname, message
        assert "asctime" in formatter._fmt or "%(asctime)" in formatter._fmt
        assert "levelname" in formatter._fmt or "%(levelname)" in formatter._fmt
        assert "message" in formatter._fmt or "%(message)" in formatter._fmt

    def test_setup_logging_custom_level(self, monkeypatch):
        """Test that setup_logging respects LOG_LEVEL environment variable"""
        # Arrange
        monkeypatch.setenv("LOG_LEVEL", "DEBUG")

        # Act
        logger = setup_logging("test_logger_5")

        # Assert
        assert logger.level == logging.DEBUG
