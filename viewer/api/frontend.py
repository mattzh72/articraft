from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse


def install_frontend_routes(app: FastAPI, *, dist_dir: Path | None = None) -> None:
    frontend_dist = dist_dir or Path(__file__).resolve().parents[1] / "web" / "dist"
    if frontend_dist.exists():
        resolved_dist_dir = frontend_dist.resolve()

        @app.get("/", include_in_schema=False)
        async def frontend_index() -> FileResponse:
            return FileResponse(
                resolved_dist_dir / "index.html",
                headers={"Cache-Control": "no-cache"},
            )

        @app.get("/{full_path:path}", include_in_schema=False)
        async def frontend_fallback(full_path: str) -> FileResponse:
            if (
                full_path == "api"
                or full_path.startswith("api/")
                or full_path in {"health", "docs", "redoc", "openapi.json"}
            ):
                raise HTTPException(status_code=404)
            requested_path = Path(full_path)
            if requested_path.is_absolute() or ".." in requested_path.parts:
                raise HTTPException(status_code=404)
            target = (resolved_dist_dir / requested_path).resolve()
            try:
                target.relative_to(resolved_dist_dir)
            except ValueError as exc:
                raise HTTPException(status_code=404) from exc
            if target.exists() and target.is_file():
                cache_control = (
                    "public, max-age=31536000, immutable"
                    if target.parts[-2:-1] == ("assets",)
                    else "no-cache"
                )
                return FileResponse(target, headers={"Cache-Control": cache_control})
            return FileResponse(
                resolved_dist_dir / "index.html",
                headers={"Cache-Control": "no-cache"},
            )

        return

    @app.get("/", include_in_schema=False)
    async def frontend_missing() -> dict[str, str]:
        return {
            "message": "Frontend build not found. Run `npm --prefix viewer/web install && npm --prefix viewer/web run build`."
        }
