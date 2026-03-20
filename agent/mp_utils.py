from __future__ import annotations

import multiprocessing as mp
import os

_MP_START_METHOD_ENV = "ARTICRAFT_MP_START_METHOD"
_DEFAULT_START_METHODS = ("forkserver", "spawn")


def resolve_mp_start_method() -> str:
    available = tuple(mp.get_all_start_methods())
    if not available:
        raise RuntimeError("No multiprocessing start methods are available on this platform.")

    raw_override = os.environ.get(_MP_START_METHOD_ENV)
    if raw_override is not None and raw_override.strip():
        start_method = raw_override.strip().lower()
        if start_method not in available:
            supported = ", ".join(available)
            raise ValueError(
                f"Unsupported multiprocessing start method {raw_override!r} from "
                f"{_MP_START_METHOD_ENV}. Supported values: {supported}."
            )
        return start_method

    # Prefer forkserver over spawn on Unix because it avoids repeated interpreter
    # bootstrap while still avoiding the hazards of plain fork in a live process.
    for start_method in _DEFAULT_START_METHODS:
        if start_method in available:
            return start_method
    return available[0]


def get_mp_context() -> mp.context.BaseContext:
    return mp.get_context(resolve_mp_start_method())
