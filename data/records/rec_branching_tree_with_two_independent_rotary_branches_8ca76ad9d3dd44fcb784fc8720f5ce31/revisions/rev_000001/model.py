from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_rotary_fixture")

    painted_steel = Material("satin_black_painted_steel", color=(0.02, 0.025, 0.028, 1.0))
    machined_steel = Material("machined_steel", color=(0.62, 0.64, 0.62, 1.0))
    pad_blue = Material("blue_anodized_mounting_pads", color=(0.05, 0.18, 0.55, 1.0))
    rubber = Material("dark_rubber_feet_and_bolt_heads", color=(0.005, 0.005, 0.006, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.32, 0.23, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=painted_steel,
        name="floor_plate",
    )
    spine.visual(
        Box((0.10, 0.08, 0.68)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=painted_steel,
        name="vertical_spine",
    )
    spine.visual(
        Box((0.13, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        material=machined_steel,
        name="top_cap",
    )

    def add_side_support(prefix: str, side: float, z: float) -> None:
        # A welded fork from the central post supports each rotating hub from the side.
        x_bridge = side * 0.077
        spine.visual(
            Box((0.112, 0.14, 0.042)),
            origin=Origin(xyz=(x_bridge, 0.0, z), rpy=(0.0, 0.0, 0.0)),
            material=painted_steel,
            name=f"{prefix}_support_web",
        )
        x_cheek = side * 0.103
        for y in (-0.058, 0.058):
            spine.visual(
                Box((0.095, 0.025, 0.14)),
                origin=Origin(xyz=(x_cheek, y, z)),
                material=painted_steel,
                name=f"{prefix}_cheek_{'neg' if y < 0 else 'pos'}",
            )
        x_hub = side * 0.18
        if prefix == "lower":
            bearings = (
                (0.057, "lower_upper_bearing_face", "lower_upper_bearing_bridge"),
                (-0.057, "lower_lower_bearing_face", "lower_lower_bearing_bridge"),
            )
        else:
            bearings = (
                (0.057, "upper_upper_bearing_face", "upper_upper_bearing_bridge"),
                (-0.057, "upper_lower_bearing_face", "upper_lower_bearing_bridge"),
            )
        for dz, face_name, bridge_name in bearings:
            spine.visual(
                Cylinder(radius=0.052, length=0.014),
                origin=Origin(xyz=(x_hub, 0.0, z + dz)),
                material=machined_steel,
                name=face_name,
            )
            spine.visual(
                Box((0.205, 0.026, 0.014)),
                origin=Origin(xyz=(side * 0.101, 0.0, z + dz)),
                material=machined_steel,
                name=bridge_name,
            )

    add_side_support("lower", 1.0, 0.23)
    add_side_support("upper", -1.0, 0.50)

    def add_branch(name: str, outward: float) -> object:
        branch = model.part(name)
        branch.visual(
            Cylinder(radius=0.044, length=0.10),
            origin=Origin(),
            material=machined_steel,
            name="hub_barrel",
        )
        branch.visual(
            Box((0.38, 0.052, 0.036)),
            origin=Origin(xyz=(outward * 0.23, 0.0, 0.0)),
            material=machined_steel,
            name="branch_arm",
        )
        branch.visual(
            Box((0.16, 0.115, 0.026)),
            origin=Origin(xyz=(outward * 0.48, 0.0, 0.0)),
            material=pad_blue,
            name="mounting_pad",
        )
        branch.visual(
            Box((0.06, 0.075, 0.018)),
            origin=Origin(xyz=(outward * 0.395, 0.0, 0.0)),
            material=machined_steel,
            name="pad_neck",
        )
        for ix, xoff in enumerate((-0.035, 0.035)):
            for iy, yoff in enumerate((-0.032, 0.032)):
                branch.visual(
                    Cylinder(radius=0.010, length=0.006),
                    origin=Origin(xyz=(outward * (0.48 + xoff), yoff, 0.016)),
                    material=rubber,
                    name=f"bolt_head_{ix}_{iy}",
                )
        return branch

    lower_branch = add_branch("lower_branch", 1.0)
    upper_branch = add_branch("upper_branch", -1.0)

    travel = math.radians(120.0)
    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=(0.18, 0.0, 0.23)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-travel / 2.0, upper=travel / 2.0),
    )
    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=(-0.18, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-travel / 2.0, upper=travel / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_branch")
    upper = object_model.get_part("upper_branch")
    lower_pivot = object_model.get_articulation("lower_pivot")
    upper_pivot = object_model.get_articulation("upper_pivot")

    travel = math.radians(120.0)
    for joint in (lower_pivot, upper_pivot):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a limited revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs((limits.upper - limits.lower) - travel) < 1e-6,
            details=f"type={joint.articulation_type}, limits={limits}",
        )
        ctx.check(
            f"{joint.name} axis is vertical",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )

    ctx.check(
        "branch pivot axes are parallel",
        tuple(round(v, 6) for v in lower_pivot.axis) == tuple(round(v, 6) for v in upper_pivot.axis),
        details=f"lower={lower_pivot.axis}, upper={upper_pivot.axis}",
    )

    ctx.expect_contact(
        lower,
        "spine",
        elem_a="hub_barrel",
        elem_b="lower_upper_bearing_face",
        contact_tol=0.001,
        name="lower hub is captured by the upper bearing face",
    )
    ctx.expect_contact(
        upper,
        "spine",
        elem_a="hub_barrel",
        elem_b="upper_lower_bearing_face",
        contact_tol=0.001,
        name="upper hub is captured by the lower bearing face",
    )

    def pad_center_y(part_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="mounting_pad")
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    lower_rest_y = pad_center_y("lower_branch")
    upper_rest_y = pad_center_y("upper_branch")
    with ctx.pose({lower_pivot: lower_pivot.motion_limits.upper}):
        lower_swept_y = pad_center_y("lower_branch")
        upper_still_y = pad_center_y("upper_branch")
    with ctx.pose({upper_pivot: upper_pivot.motion_limits.lower}):
        upper_swept_y = pad_center_y("upper_branch")

    ctx.check(
        "lower branch sweeps about its hub",
        lower_rest_y is not None and lower_swept_y is not None and lower_swept_y > lower_rest_y + 0.25,
        details=f"rest_y={lower_rest_y}, swept_y={lower_swept_y}",
    )
    ctx.check(
        "upper branch moves independently",
        upper_rest_y is not None
        and upper_still_y is not None
        and upper_swept_y is not None
        and abs(upper_still_y - upper_rest_y) < 0.005
        and upper_swept_y > upper_rest_y + 0.25,
        details=f"rest_y={upper_rest_y}, still_y={upper_still_y}, swept_y={upper_swept_y}",
    )

    return ctx.report()


object_model = build_object_model()
