from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, radians, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_powder = model.material("dark_powder", rgba=(0.18, 0.19, 0.21, 1.0))
    nylon = model.material("nylon", rgba=(0.89, 0.90, 0.86, 1.0))
    concrete = model.material("concrete", rgba=(0.61, 0.61, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    base_frame = model.part("base_frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((2.70, 0.22, 2.30)),
        mass=180.0,
        origin=Origin(xyz=(-0.35, 0.0, 1.15)),
    )
    base_frame.visual(
        Box((0.12, 0.14, 2.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=dark_powder,
        name="support_post",
    )
    base_frame.visual(
        Box((0.09, 0.10, 1.95)),
        origin=Origin(xyz=(-1.56, 0.0, 0.975)),
        material=dark_powder,
        name="receiver_post",
    )
    base_frame.visual(
        Box((0.08, 0.10, 2.05)),
        origin=Origin(xyz=(0.90, 0.0, 1.025)),
        material=dark_powder,
        name="parking_side_post",
    )
    base_frame.visual(
        Box((2.58, 0.18, 0.16)),
        origin=Origin(xyz=(-0.33, 0.0, 2.08)),
        material=dark_powder,
        name="header_beam",
    )
    base_frame.visual(
        Box((0.34, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=concrete,
        name="support_footing",
    )
    base_frame.visual(
        Box((0.14, 0.18, 0.04)),
        origin=Origin(xyz=(-1.56, 0.0, -0.02)),
        material=concrete,
        name="receiver_footing",
    )
    base_frame.visual(
        Box((0.28, 0.24, 0.04)),
        origin=Origin(xyz=(0.90, 0.0, -0.02)),
        material=concrete,
        name="parking_footing",
    )
    base_frame.visual(
        Box((2.42, 0.10, 0.010)),
        origin=Origin(xyz=(-0.33, 0.0, 1.950)),
        material=galvanized_steel,
        name="track_top",
    )
    base_frame.visual(
        Box((2.42, 0.014, 0.080)),
        origin=Origin(xyz=(-0.33, -0.043, 1.905)),
        material=galvanized_steel,
        name="track_left_lip",
    )
    base_frame.visual(
        Box((2.42, 0.014, 0.080)),
        origin=Origin(xyz=(-0.33, 0.043, 1.905)),
        material=galvanized_steel,
        name="track_right_lip",
    )
    base_frame.visual(
        Box((0.050, 0.090, 0.030)),
        origin=Origin(xyz=(-1.50, 0.0, 1.910)),
        material=rubber,
        name="left_track_stop",
    )
    base_frame.visual(
        Box((0.050, 0.090, 0.030)),
        origin=Origin(xyz=(0.84, 0.0, 1.910)),
        material=rubber,
        name="right_track_stop",
    )

    guide_module = model.part("guide_module")
    guide_module.inertial = Inertial.from_geometry(
        Box((0.22, 0.20, 0.24)),
        mass=8.0,
        origin=Origin(xyz=(-0.01, 0.04, 0.19)),
    )
    guide_module.visual(
        Box((0.04, 0.14, 0.22)),
        origin=Origin(xyz=(0.08, 0.0, 0.19)),
        material=dark_powder,
        name="guide_mount_plate",
    )
    guide_module.visual(
        Box((0.18, 0.022, 0.046)),
        origin=Origin(xyz=(-0.01, 0.081, 0.277)),
        material=dark_powder,
        name="guide_support_arm",
    )
    guide_module.visual(
        Box((0.024, 0.15, 0.02)),
        origin=Origin(xyz=(-0.09, 0.019, 0.295)),
        material=dark_powder,
        name="guide_top_bridge",
    )
    guide_module.visual(
        Box((0.024, 0.012, 0.16)),
        origin=Origin(xyz=(-0.09, -0.05, 0.21)),
        material=dark_powder,
        name="guide_left_cheek",
    )
    guide_module.visual(
        Box((0.024, 0.012, 0.16)),
        origin=Origin(xyz=(-0.09, 0.05, 0.21)),
        material=dark_powder,
        name="guide_right_cheek",
    )
    guide_module.visual(
        Box((0.012, 0.034, 0.01)),
        origin=Origin(xyz=(-0.09, -0.027, 0.245)),
        material=galvanized_steel,
        name="guide_left_top_cap",
    )
    guide_module.visual(
        Box((0.012, 0.034, 0.01)),
        origin=Origin(xyz=(-0.09, -0.027, 0.155)),
        material=galvanized_steel,
        name="guide_left_bottom_cap",
    )
    guide_module.visual(
        Box((0.012, 0.034, 0.01)),
        origin=Origin(xyz=(-0.09, 0.027, 0.245)),
        material=galvanized_steel,
        name="guide_right_top_cap",
    )
    guide_module.visual(
        Box((0.012, 0.034, 0.01)),
        origin=Origin(xyz=(-0.09, 0.027, 0.155)),
        material=galvanized_steel,
        name="guide_right_bottom_cap",
    )
    guide_module.visual(
        Cylinder(radius=0.017, length=0.08),
        origin=Origin(xyz=(-0.09, -0.027, 0.20)),
        material=nylon,
        name="guide_left_roller",
    )
    guide_module.visual(
        Cylinder(radius=0.017, length=0.08),
        origin=Origin(xyz=(-0.09, 0.027, 0.20)),
        material=nylon,
        name="guide_right_roller",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.36, 0.05, 1.74)),
        mass=68.0,
        origin=Origin(xyz=(-0.68, 0.0, -0.87)),
    )
    gate_leaf.visual(
        Box((1.34, 0.05, 0.06)),
        origin=Origin(xyz=(-0.67, 0.0, -0.03)),
        material=dark_powder,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.34, 0.05, 0.08)),
        origin=Origin(xyz=(-0.67, 0.0, -1.68)),
        material=dark_powder,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.06, 0.05, 1.60)),
        origin=Origin(xyz=(-0.03, 0.0, -0.86)),
        material=dark_powder,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((0.06, 0.05, 1.60)),
        origin=Origin(xyz=(-1.31, 0.0, -0.86)),
        material=dark_powder,
        name="left_stile",
    )

    for index, x_pos in enumerate((-0.23, -0.45, -0.67, -0.89, -1.11), start=1):
        gate_leaf.visual(
            Box((0.03, 0.02, 1.52)),
            origin=Origin(xyz=(x_pos, 0.0, -0.86)),
            material=galvanized_steel,
            name=f"picket_{index}",
        )

    brace_dx = 1.18
    brace_dz = 1.46
    brace_len = sqrt(brace_dx * brace_dx + brace_dz * brace_dz)
    brace_angle = atan2(brace_dz, brace_dx)
    gate_leaf.visual(
        Box((brace_len, 0.025, 0.04)),
        origin=Origin(
            xyz=(-0.70, 0.0, -0.83),
            rpy=(0.0, -brace_angle, 0.0),
        ),
        material=galvanized_steel,
        name="diagonal_brace",
    )
    gate_leaf.visual(
        Box((0.05, 0.018, 0.12)),
        origin=Origin(xyz=(-1.34, 0.0, -1.56)),
        material=dark_powder,
        name="bottom_end_block",
    )
    gate_leaf.visual(
        Box((1.22, 0.020, 0.16)),
        origin=Origin(xyz=(-0.56, 0.0, -1.56)),
        material=galvanized_steel,
        name="guide_fin",
    )

    front_carriage = model.part("front_carriage")
    front_carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.04, 0.18)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )
    front_carriage.visual(
        Box((0.12, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=galvanized_steel,
        name="mount_plate",
    )
    front_carriage.visual(
        Box((0.04, 0.018, 0.148)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=galvanized_steel,
        name="hanger_strap",
    )
    front_carriage.visual(
        Box((0.06, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=dark_powder,
        name="axle_block",
    )
    front_carriage.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.160), rpy=(pi / 2.0, 0.0, 0.0)),
        material=nylon,
        name="trolley_wheel",
    )

    rear_carriage = model.part("rear_carriage")
    rear_carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.04, 0.18)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )
    rear_carriage.visual(
        Box((0.12, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=galvanized_steel,
        name="mount_plate",
    )
    rear_carriage.visual(
        Box((0.04, 0.018, 0.148)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=galvanized_steel,
        name="hanger_strap",
    )
    rear_carriage.visual(
        Box((0.06, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=dark_powder,
        name="axle_block",
    )
    rear_carriage.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.160), rpy=(pi / 2.0, 0.0, 0.0)),
        material=nylon,
        name="trolley_wheel",
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.FIXED,
        parent=base_frame,
        child=guide_module,
        origin=Origin(),
    )
    model.articulation(
        "base_to_gate",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=gate_leaf,
        origin=Origin(xyz=(-0.14, 0.0, 1.76)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=0.96,
        ),
    )
    model.articulation(
        "gate_to_front_carriage",
        ArticulationType.FIXED,
        parent=gate_leaf,
        child=front_carriage,
        origin=Origin(xyz=(-0.22, 0.0, 0.0)),
    )
    model.articulation(
        "gate_to_rear_carriage",
        ArticulationType.FIXED,
        parent=gate_leaf,
        child=rear_carriage,
        origin=Origin(xyz=(-1.03, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    front_carriage = object_model.get_part("front_carriage")
    rear_carriage = object_model.get_part("rear_carriage")
    slide = object_model.get_articulation("base_to_gate")

    ctx.expect_gap(
        gate_leaf,
        base_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="support_footing",
        min_gap=0.03,
        max_gap=0.08,
        name="gate clears the footing and ground plane",
    )
    ctx.expect_gap(
        base_frame,
        front_carriage,
        axis="z",
        positive_elem="track_top",
        negative_elem="trolley_wheel",
        min_gap=0.001,
        max_gap=0.010,
        name="front trolley wheel rides just below the track roof",
    )
    ctx.expect_gap(
        base_frame,
        rear_carriage,
        axis="z",
        positive_elem="track_top",
        negative_elem="trolley_wheel",
        min_gap=0.001,
        max_gap=0.010,
        name="rear trolley wheel rides just below the track roof",
    )

    closed_origin = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 0.96}):
        opened_origin = ctx.part_world_position(gate_leaf)
        ctx.expect_gap(
            base_frame,
            front_carriage,
            axis="z",
            positive_elem="track_top",
            negative_elem="trolley_wheel",
            min_gap=0.001,
            max_gap=0.010,
            name="front trolley stays captured at full travel",
        )
        ctx.expect_gap(
            base_frame,
            rear_carriage,
            axis="z",
            positive_elem="track_top",
            negative_elem="trolley_wheel",
            min_gap=0.001,
            max_gap=0.010,
            name="rear trolley stays captured at full travel",
        )

    ctx.check(
        "gate leaf opens toward the parking side",
        closed_origin is not None
        and opened_origin is not None
        and opened_origin[0] > closed_origin[0] + 0.80,
        details=f"closed={closed_origin}, opened={opened_origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
