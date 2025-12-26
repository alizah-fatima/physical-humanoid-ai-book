"""
Security headers and best practices for RAG Agent API
"""
from fastapi import FastAPI
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response
import secrets
import hashlib
import time
from typing import Optional, Dict, Any


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Middleware to add security headers to responses"""

    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)

        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["Content-Security-Policy"] = "default-src 'self'; frame-ancestors 'none';"

        # Remove server header to avoid revealing server information
        if "Server" in response.headers:
            del response.headers["Server"]

        return response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Middleware to implement rate limiting"""

    def __init__(self, app: FastAPI, requests_per_minute: int = 60, window_size: int = 60):
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.window_size = window_size
        self.requests: Dict[str, list] = {}  # Store request times per IP

    async def dispatch(self, request: Request, call_next):
        # Get client IP
        client_ip = request.client.host
        current_time = time.time()

        # Initialize request list for this IP if needed
        if client_ip not in self.requests:
            self.requests[client_ip] = []

        # Remove requests outside the time window
        self.requests[client_ip] = [
            req_time for req_time in self.requests[client_ip]
            if current_time - req_time < self.window_size
        ]

        # Check if rate limit exceeded
        if len(self.requests[client_ip]) >= self.requests_per_minute:
            response = Response(
                content="Rate limit exceeded: Too many requests",
                status_code=429
            )
            return response

        # Add current request time
        self.requests[client_ip].append(current_time)

        response = await call_next(request)
        return response


class AuthenticationMiddleware(BaseHTTPMiddleware):
    """Middleware to handle API key authentication"""

    def __init__(self, app: FastAPI, api_keys: Optional[list] = None):
        super().__init__(app)
        self.api_keys = api_keys or []

    async def dispatch(self, request: Request, call_next):
        # Extract API key from header
        auth_header = request.headers.get("Authorization")
        api_key = None

        if auth_header and auth_header.startswith("Bearer "):
            api_key = auth_header[7:]  # Remove "Bearer " prefix
        elif auth_header and auth_header.startswith("API-Key "):
            api_key = auth_header[8:]  # Remove "API-Key " prefix
        elif "api_key" in request.query_params:
            api_key = request.query_params["api_key"]

        # If API keys are configured, validate the provided key
        if self.api_keys and api_key not in self.api_keys:
            response = Response(
                content="Unauthorized: Invalid API key",
                status_code=401
            )
            return response

        response = await call_next(request)
        return response


def add_security_headers(app: FastAPI):
    """Add security headers and best practices to the FastAPI app"""
    # Add security headers middleware
    app.add_middleware(SecurityHeadersMiddleware)

    # Note: In a production environment, you would also add:
    # - Rate limiting middleware (with appropriate limits)
    # - Authentication middleware (with proper API key validation)
    # - Input validation and sanitization
    # - Proper CORS configuration


def validate_input(input_text: str, max_length: int = 1000) -> bool:
    """
    Validate input to prevent common attacks and ensure reasonable length.

    Args:
        input_text: The input text to validate
        max_length: Maximum allowed length for the input

    Returns:
        True if input is valid, False otherwise
    """
    if not input_text:
        return False

    if len(input_text) > max_length:
        return False

    # Check for common attack patterns
    dangerous_patterns = [
        "<script", "javascript:", "vbscript:", "onerror", "onload",
        "eval(", "expression(", "javascript:", "data:", "vbscript:",
        "<iframe", "<object", "<embed", "<form", "<link", "<meta"
    ]

    lower_input = input_text.lower()
    for pattern in dangerous_patterns:
        if pattern in lower_input:
            return False

    return True


def hash_api_key(api_key: str) -> str:
    """
    Hash an API key for secure storage.

    Args:
        api_key: The API key to hash

    Returns:
        The hashed API key
    """
    return hashlib.sha256(api_key.encode()).hexdigest()


def generate_api_key() -> str:
    """
    Generate a secure API key.

    Returns:
        A new API key
    """
    return secrets.token_urlsafe(32)


def sanitize_input(input_text: str) -> str:
    """
    Sanitize input to remove potentially dangerous content.

    Args:
        input_text: The input text to sanitize

    Returns:
        Sanitized input text
    """
    # Remove potentially dangerous characters/sequences
    sanitized = input_text.replace("<script", "&lt;script")
    sanitized = sanitized.replace("javascript:", "javascript&#58;")
    sanitized = sanitized.replace("vbscript:", "vbscript&#58;")

    return sanitized