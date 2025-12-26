"""
Retry handler for external API calls
"""
import time
import random
from typing import Callable, Any, Optional, Tuple
from ..utils.logging import get_logger

logger = get_logger()


class RetryHandler:
    """Handler for implementing retry logic for external API calls"""

    def __init__(self, max_retries: int = 3, base_delay: float = 1.0, max_delay: float = 60.0, backoff_factor: float = 2.0):
        """
        Initialize the retry handler.

        Args:
            max_retries: Maximum number of retry attempts
            base_delay: Initial delay between retries in seconds
            max_delay: Maximum delay between retries in seconds
            backoff_factor: Multiplier for exponential backoff
        """
        self.max_retries = max_retries
        self.base_delay = base_delay
        self.max_delay = max_delay
        self.backoff_factor = backoff_factor

    def execute_with_retry(self, func: Callable, *args, **kwargs) -> Tuple[bool, Any, Optional[Exception]]:
        """
        Execute a function with retry logic.

        Args:
            func: The function to execute
            *args: Arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function

        Returns:
            Tuple of (success: bool, result: Any, exception: Optional[Exception])
        """
        last_exception = None

        for attempt in range(self.max_retries + 1):
            try:
                result = func(*args, **kwargs)
                return True, result, None
            except Exception as e:
                last_exception = e
                if attempt == self.max_retries:
                    # Final attempt failed
                    logger.error(f"Function failed after {self.max_retries} retries: {str(e)}")
                    break

                # Calculate delay with exponential backoff and jitter
                delay = min(self.base_delay * (self.backoff_factor ** attempt), self.max_delay)
                jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                actual_delay = delay + jitter

                logger.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {actual_delay:.2f}s...")

                time.sleep(actual_delay)

        return False, None, last_exception

    @staticmethod
    def is_retryable_exception(exception: Exception) -> bool:
        """
        Determine if an exception is retryable.

        Args:
            exception: The exception to check

        Returns:
            True if the exception is retryable, False otherwise
        """
        # Common retryable exceptions (network issues, server errors, etc.)
        retryable_exceptions = (
            ConnectionError,
            TimeoutError,
            # Add more specific exceptions as needed
        )

        # HTTP status codes that indicate retryable errors
        if hasattr(exception, 'status_code'):
            status_code = getattr(exception, 'status_code', None)
            if status_code in [429, 500, 502, 503, 504]:
                return True

        return isinstance(exception, retryable_exceptions)

    def execute_with_retry_conditional(self, func: Callable, *args, **kwargs) -> Tuple[bool, Any, Optional[Exception]]:
        """
        Execute a function with retry logic that only retries on specific exceptions.

        Args:
            func: The function to execute
            *args: Arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function

        Returns:
            Tuple of (success: bool, result: Any, exception: Optional[Exception])
        """
        last_exception = None

        for attempt in range(self.max_retries + 1):
            try:
                result = func(*args, **kwargs)
                return True, result, None
            except Exception as e:
                last_exception = e

                # Check if this exception is retryable
                if not self.is_retryable_exception(e):
                    logger.error(f"Non-retryable exception occurred: {str(e)}")
                    break

                if attempt == self.max_retries:
                    # Final attempt failed
                    logger.error(f"Function failed after {self.max_retries} retries: {str(e)}")
                    break

                # Calculate delay with exponential backoff and jitter
                delay = min(self.base_delay * (self.backoff_factor ** attempt), self.max_delay)
                jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                actual_delay = delay + jitter

                logger.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {actual_delay:.2f}s...")

                time.sleep(actual_delay)

        return False, None, last_exception


# Default retry handler instance
default_retry_handler = RetryHandler()