from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_rotary_fixture")

    satin_steel = model.material("satin_steel", color=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_steel", color=(0.18, 0.20, 0.22, 1.0))
    blue_anodized = model.material("blue_anodized", color=(0.05, 0.22, 0.70, 1.0))
    amber_anodized = model.material("amber_anodized", color=(0.90, 0.48, 0.10, 1.0))
    black_rubber = model.material("black_rubber", color=(0.025, 0.024, 0.022, 1.0))

    fixture = model.part("fixture")
    fixture.visual(
        Cylinder(radius=0.22, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_plate",
    )
    fixture.visual(
        Cylinder(radius=0.030, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=satin_steel,
        name="mast",
    )
    fixture.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_steel,
        name="base_collar",
    )

    # Lower branch yoke: two supported plates carried radially from the mast.
    fixture.visual(
        Box((0.150, 0.060, 0.018)),
        origin=Origin(xyz=(0.105, 0.0, 0.344)),
        material=satin_steel,
        name="lower_bottom_plate",
    )
    fixture.visual(
        Box((0.150, 0.060, 0.018)),
        origin=Origin(xyz=(0.105, 0.0, 0.416)),
        material=satin_steel,
        name="lower_top_plate",
    )
    fixture.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.180, 0.0, 0.331)),
        material=dark_steel,
        name="lower_axis_cap",
    )
    fixture.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.180, 0.0, 0.429)),
        material=dark_steel,
        name="lower_axis_head",
    )

    # Upper branch yoke is staggered both in height and around the mast.
    fixture.visual(
        Box((0.060, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.594)),
        material=satin_steel,
        name="upper_bottom_plate",
    )
    fixture.visual(
        Box((0.060, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.666)),
        material=satin_steel,
        name="upper_top_plate",
    )
    fixture.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.0, 0.180, 0.581)),
        material=dark_steel,
        name="upper_axis_cap",
    )
    fixture.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.0, 0.180, 0.679)),
        material=dark_steel,
        name="upper_axis_head",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.026, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_anodized,
        name="lower_sleeve",
    )
    lower_arm.visual(
        Box((0.260, 0.034, 0.030)),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=blue_anodized,
        name="lower_beam",
    )
    lower_arm.visual(
        Box((0.080, 0.115, 0.022)),
        origin=Origin(xyz=(0.295, 0.0, -0.001)),
        material=black_rubber,
        name="lower_pad",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.026, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=amber_anodized,
        name="upper_sleeve",
    )
    upper_arm.visual(
        Box((0.240, 0.032, 0.030)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=amber_anodized,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.045, 0.115, 0.030)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=amber_anodized,
        name="upper_fork_bridge",
    )
    upper_arm.visual(
        Box((0.115, 0.026, 0.026)),
        origin=Origin(xyz=(0.315, 0.044, 0.0)),
        material=amber_anodized,
        name="upper_fork_tine_0",
    )
    upper_arm.visual(
        Box((0.115, 0.026, 0.026)),
        origin=Origin(xyz=(0.315, -0.044, 0.0)),
        material=amber_anodized,
        name="upper_fork_tine_1",
    )

    lower_joint = model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=lower_arm,
        origin=Origin(xyz=(0.180, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    upper_joint = model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.180, 0.630), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.10, upper=1.10),
    )
    lower_joint.meta["branch"] = "independent"
    upper_joint.meta["branch"] = "independent"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture = object_model.get_part("fixture")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_pivot = object_model.get_articulation("lower_pivot")
    upper_pivot = object_model.get_articulation("upper_pivot")

    ctx.check(
        "two independent revolute branches",
        getattr(lower_pivot.parent, "name", lower_pivot.parent) == "fixture"
        and getattr(upper_pivot.parent, "name", upper_pivot.parent) == "fixture"
        and getattr(lower_pivot.child, "name", lower_pivot.child) == "lower_arm"
        and getattr(upper_pivot.child, "name", upper_pivot.child) == "upper_arm"
        and getattr(lower_pivot, "mimic", None) is None
        and getattr(upper_pivot, "mimic", None) is None,
        details="Both arms must be separate revolute children of the fixed mast with no mimic coupling.",
    )

    ctx.expect_gap(
        fixture,
        lower_arm,
        axis="z",
        positive_elem="lower_top_plate",
        negative_elem="lower_sleeve",
        max_gap=0.001,
        max_penetration=0.000001,
        name="lower sleeve seats under top support plate",
    )
    ctx.expect_gap(
        lower_arm,
        fixture,
        axis="z",
        positive_elem="lower_sleeve",
        negative_elem="lower_bottom_plate",
        max_gap=0.001,
        max_penetration=0.000001,
        name="lower sleeve seats over bottom support plate",
    )
    ctx.expect_gap(
        fixture,
        upper_arm,
        axis="z",
        positive_elem="upper_top_plate",
        negative_elem="upper_sleeve",
        max_gap=0.001,
        max_penetration=0.000001,
        name="upper sleeve seats under top support plate",
    )
    ctx.expect_gap(
        upper_arm,
        fixture,
        axis="z",
        positive_elem="upper_sleeve",
        negative_elem="upper_bottom_plate",
        max_gap=0.001,
        max_penetration=0.000001,
        name="upper sleeve seats over bottom support plate",
    )

    def _elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    lower_rest = _elem_center(lower_arm, "lower_pad")
    upper_rest = _elem_center(upper_arm, "upper_fork_bridge")
    with ctx.pose({lower_pivot: 0.75}):
        lower_swung = _elem_center(lower_arm, "lower_pad")
        upper_after_lower = _elem_center(upper_arm, "upper_fork_bridge")
    with ctx.pose({upper_pivot: -0.70}):
        lower_after_upper = _elem_center(lower_arm, "lower_pad")
        upper_swung = _elem_center(upper_arm, "upper_fork_bridge")

    ctx.check(
        "lower pivot swings only lower branch",
        lower_rest is not None
        and lower_swung is not None
        and upper_rest is not None
        and upper_after_lower is not None
        and abs(lower_swung[1] - lower_rest[1]) > 0.12
        and max(abs(upper_after_lower[i] - upper_rest[i]) for i in range(3)) < 1e-6,
        details=f"lower_rest={lower_rest}, lower_swung={lower_swung}, upper_rest={upper_rest}, upper_after_lower={upper_after_lower}",
    )
    ctx.check(
        "upper pivot swings only upper branch",
        upper_rest is not None
        and upper_swung is not None
        and lower_rest is not None
        and lower_after_upper is not None
        and abs(upper_swung[0] - upper_rest[0]) > 0.08
        and max(abs(lower_after_upper[i] - lower_rest[i]) for i in range(3)) < 1e-6,
        details=f"upper_rest={upper_rest}, upper_swung={upper_swung}, lower_rest={lower_rest}, lower_after_upper={lower_after_upper}",
    )

    return ctx.report()


object_model = build_object_model()
