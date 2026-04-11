"""Formatting utilities for TUI display."""

_PREFERRED_TOOL_ARG_KEYS = (
    "path",
    "query",
    "offset",
    "limit",
    "old_string",
    "new_string",
    "replace_all",
)


def format_cost(cost: float) -> str:
    """Format cost as currency string.

    Args:
        cost: Cost in USD

    Returns:
        Formatted string like "$0.0234"
    """
    if cost <= 0:
        return "$0.0000"
    if cost < 0.0001:
        return "<$0.0001"
    elif cost < 0.01:
        return f"${cost:.6f}"
    elif cost < 1.0:
        return f"${cost:.4f}"
    else:
        return f"${cost:.2f}"


def format_tokens(tokens: int) -> str:
    """Format token count with K/M suffix.

    Args:
        tokens: Token count

    Returns:
        Formatted string like "12.5K" or "1.2M"
    """
    if tokens < 1000:
        return str(tokens)
    elif tokens < 1_000_000:
        return f"{tokens / 1000:.1f}K"
    else:
        return f"{tokens / 1_000_000:.1f}M"


def format_duration(seconds: float) -> str:
    """Format duration as human-readable string.

    Args:
        seconds: Duration in seconds

    Returns:
        Formatted string like "2m 15s" or "1h 5m"
    """
    if seconds < 1:
        return f"{seconds * 1000:.0f}ms"
    elif seconds < 60:
        return f"{seconds:.1f}s"

    minutes, secs = divmod(int(seconds), 60)
    if minutes < 60:
        return f"{minutes}m {secs}s"

    hours, mins = divmod(minutes, 60)
    return f"{hours}h {mins}m"


def format_timestamp(seconds: float) -> str:
    """Format elapsed time as MM:SS timestamp.

    Args:
        seconds: Elapsed seconds from start

    Returns:
        Formatted string like "02:15" or "1:23:45"
    """
    if seconds < 3600:  # Less than 1 hour
        minutes, secs = divmod(int(seconds), 60)
        return f"{minutes:02d}:{secs:02d}"
    else:  # 1 hour or more
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours}:{minutes:02d}:{secs:02d}"


def truncate_text(text: str, max_length: int, suffix: str = "...") -> str:
    """Truncate text to max_length with suffix.

    Args:
        text: Text to truncate
        max_length: Maximum length including suffix
        suffix: Suffix to add if truncated

    Returns:
        Truncated text with suffix if needed
    """
    if len(text) <= max_length:
        return text
    return text[: max_length - len(suffix)] + suffix


def format_tool_args(args, max_items: int | None = None) -> list[str]:
    """Format tool arguments for display.

    Args:
        args: Tool arguments dictionary or string
        max_items: Maximum number of items to show, or None for all

    Returns:
        List of formatted argument strings like ["file: foo.py", "line: 42"]
    """
    if isinstance(args, str):
        try:
            import json

            args = json.loads(args)
        except (json.JSONDecodeError, TypeError):
            return [truncate_text(args, 60)] if args else []

    if not isinstance(args, dict):
        return [str(args)] if args else []

    preferred_order = {key: index for index, key in enumerate(_PREFERRED_TOOL_ARG_KEYS)}
    ordered_items = sorted(
        args.items(),
        key=lambda item: (preferred_order.get(item[0], len(preferred_order)), item[0]),
    )

    formatted = []
    for i, (key, value) in enumerate(ordered_items):
        if max_items is not None and i >= max_items:
            formatted.append(f"... ({len(args) - max_items} more)")
            break

        # Format value based on type
        if isinstance(value, str):
            # Truncate long strings
            if len(value) > 50:
                value = truncate_text(value, 50)
            formatted.append(f"{key}: {value}")
        elif isinstance(value, (int, float, bool)):
            formatted.append(f"{key}: {value}")
        elif isinstance(value, (list, dict)):
            formatted.append(f"{key}: {type(value).__name__}({len(value)})")
        else:
            formatted.append(f"{key}: {type(value).__name__}")

    return formatted
