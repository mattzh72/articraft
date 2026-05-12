from __future__ import annotations

from typing import Annotated, cast

from fastapi import Depends, Request

from viewer.api.file_resolver import ViewerFileResolver
from viewer.api.store import ViewerStore


def get_viewer_store(request: Request) -> ViewerStore:
    return cast(ViewerStore, request.app.state.viewer_store)


def get_file_resolver(request: Request) -> ViewerFileResolver:
    return cast(ViewerFileResolver, request.app.state.file_resolver)


ViewerStoreDep = Annotated[ViewerStore, Depends(get_viewer_store)]
FileResolverDep = Annotated[ViewerFileResolver, Depends(get_file_resolver)]
