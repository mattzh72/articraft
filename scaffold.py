from __future__ import annotations

from sdk import ArticulatedObject, TestContext, TestReport

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model")
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    # For bounded REVOLUTE/PRISMATIC joints, add exact lower/upper motion-limit
    # checks for prompt-critical contacts, clearances, and motion direction. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.expect_contact(lid, body, elem_a="hinge_leaf", elem_b="body_leaf")

    return ctx.report()


object_model = build_object_model()
