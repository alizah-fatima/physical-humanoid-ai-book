"""
Middleware for RAG Agent API
"""
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Callable, Any
import time
import logging
from ..utils.logging import get_logger
from ..utils.errors import RAGAgentError, handle_error

logger = get_logger()


class RequestLoggingMiddleware:
    """Middleware to log incoming requests and responses"""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        request = Request(scope)
        start_time = time.time()

        # Log the incoming request
        logger.info(f"Request: {request.method} {request.url.path}")

        async def send_with_logging(message):
            if message["type"] == "http.response.start":
                process_time = time.time() - start_time
                response_status = message["status"]
                logger.info(f"Response: {response_status} in {process_time:.2f}ms - {request.method} {request.url.path}")
            await send(message)

        await self.app(scope, receive, send_with_logging)


class ErrorHandlingMiddleware:
    """Middleware to handle errors globally"""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        try:
            await self.app(scope, receive, send)
        except RAGAgentError as e:
            logger.error(f"RAG Agent error: {str(e)}")
            # The error is already formatted properly
            response = JSONResponse(
                status_code=e.status_code,
                content=e.to_dict()
            )
            # Create a new scope for the error response
            async def send_error_response(message):
                if message["type"] == "http.response.start":
                    message["status"] = e.status_code
                elif message["type"] == "http.response.body":
                    if hasattr(response, "body_iterator"):
                        body = b""
                        async for chunk in response.body_iterator:
                            body += chunk
                        message["body"] = body
                        message["more_body"] = False
                await send(message)

            await send_error_response({
                "type": "http.response.start",
                "status": e.status_code,
                "headers": [[b"content-type", b"application/json"]]
            })
            await send_error_response({
                "type": "http.response.body",
                "body": response.body,
                "more_body": False
            })
        except HTTPException as e:
            logger.error(f"HTTP error: {e.detail}")
            # Handle FastAPI HTTP exceptions
            handled_error = handle_error(e, e.status_code)
            response = JSONResponse(
                status_code=e.status_code,
                content=handled_error.to_dict()
            )
            async def send_error_response(message):
                if message["type"] == "http.response.start":
                    message["status"] = e.status_code
                elif message["type"] == "http.response.body":
                    message["body"] = response.body
                    message["more_body"] = False
                await send(message)

            await send_error_response({
                "type": "http.response.start",
                "status": e.status_code,
                "headers": [[b"content-type", b"application/json"]]
            })
            await send_error_response({
                "type": "http.response.body",
                "body": response.body,
                "more_body": False
            })
        except Exception as e:
            logger.error(f"Unexpected error: {str(e)}")
            handled_error = handle_error(e)
            response = JSONResponse(
                status_code=handled_error.status_code,
                content=handled_error.to_dict()
            )
            async def send_error_response(message):
                if message["type"] == "http.response.start":
                    message["status"] = handled_error.status_code
                elif message["type"] == "http.response.body":
                    message["body"] = response.body
                    message["more_body"] = False
                await send(message)

            await send_error_response({
                "type": "http.response.start",
                "status": handled_error.status_code,
                "headers": [[b"content-type", b"application/json"]]
            })
            await send_error_response({
                "type": "http.response.body",
                "body": response.body,
                "more_body": False
            })