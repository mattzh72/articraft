from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_light")

    housing = model.part("housing")
    # Housing is 1.2m long (X), 0.3m wide at bottom (Y), 0.2m wide at top, 0.08m tall (Z)
    housing_cq = (
        cq.Workplane("YZ")
        .moveTo(-0.15, -0.04)
        .lineTo(-0.10, 0.04)
        .lineTo(0.10, 0.04)
        .lineTo(0.15, -0.04)
        .close()
        .extrude(0.6, both=True)
        .faces("<Z")
        .shell(-0.002)
    )
    housing.visual(
        mesh_from_cadquery(housing_cq, "housing_shell"),
        name="housing_shell",
    )

    diffuser = model.part("diffuser")
    # Diffuser panel is 1.2m long, 0.3m wide, 0.005m thick
    diffuser.visual(
        Box((1.2, 0.3, 0.005)),
        # Local origin is at the hinge line (Y=-0.15 in housing, which means Y=0 in diffuser frame)
        origin=Origin(xyz=(0.0, 0.15, -0.0025)),
        name="diffuser_panel",
    )

    model.articulation(
        "housing_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=diffuser,
        origin=Origin(xyz=(0.0, -0.15, -0.04)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    diffuser = object_model.get_part("diffuser")
    hinge = object_model.get_articulation("housing_to_diffuser")

    # The diffuser sits flush against the housing at rest
    ctx.expect_overlap(diffuser, housing, axes="xy", min_overlap=0.1)

    with ctx.pose({hinge: 1.57}):
        # When fully opened, the diffuser should hang downwards
        diff_aabb = ctx.part_world_aabb(diffuser)
        if diff_aabb:
            ctx.check(
                "diffuser_opens_downward",
                diff_aabb[0][2] < -0.3,
                f"diffuser aabb min {diff_aabb[0]} should be below -0.3",
            )

    return ctx.report()


object_model = build_object_model()
