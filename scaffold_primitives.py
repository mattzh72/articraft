from __future__ import annotations

from sdk_primitives import ArticulatedObject, TestContext, TestReport


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model")
    return model


def run_tests() -> TestReport:
    return TestContext(object_model).report()


object_model = build_object_model()
