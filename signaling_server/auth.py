import os


def validate_token(token: str) -> bool:
    if not token:
        return False
    # environment variable AUTH_TOKENS can contain comma separated tokens
    allowed = os.environ.get("AUTH_TOKENS", "demo_token_123")
    return token in [t.strip() for t in allowed.split(",")]
