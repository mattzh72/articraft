from __future__ import annotations

from sdk_no_testing import ArticulatedObject


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model")
    return model


object_model = build_object_model()
