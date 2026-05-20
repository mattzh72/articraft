from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    mesh_from_geometry,
    tube_from_spline_points,
)


PILLAR_CENTER_X = 1.85
PILLAR_WIDTH = 0.36
PILLAR_DEPTH = 0.46
PILLAR_HEIGHT = 1.90

LEAF_WIDTH = 1.65
LEAF_DEPTH = 0.045
LEAF_RAIL_SPAN = 1.12
FRAME_BAR = 0.045
PICKET_SIZE = 0.018
HINGE_AXIS_OFFSET_Y = -0.038
HINGE_BARREL_RADIUS = 0.014
HINGE_BARREL_LENGTH = 0.090
HINGE_Z_LOWER = 0.28
HINGE_Z_UPPER = HINGE_Z_LOWER + LEAF_RAIL_SPAN


def _ellipse_points(
    center_x: float,
    center_z: float,
    radius_x: float,
    radius_z: float,
    *,
    segments: int = 32,
) -> list[tuple[float, float, float]]:
    return [
        (
            center_x + radius_x * cos(2.0 * pi * i / segments),
            0.0,
            center_z + radius_z * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _add_hinge_part(part, *, sign: float, metal, name_prefix: str) -> None:
    part.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(xyz=(sign * 0.026, HINGE_AXIS_OFFSET_Y, 0.0)),
        material=metal,
        name=f"{name_prefix}_barrel",
    )
    part.visual(
        Box((0.11, 0.008, 0.050)),
        origin=Origin(xyz=(sign * 0.067, -0.029, 0.0)),
        material=metal,
        name=f"{name_prefix}_strap",
    )
    part.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(sign * 0.030, -0.031, 0.0)),
        material=metal,
        name=f"{name_prefix}_knuckle",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.13, 0.05, 0.10)),
        mass=2.5,
        origin=Origin(xyz=(sign * 0.05, -0.03, 0.0)),
    )


def _add_leaf_panel(part, *, sign: float, metal, ornament, mesh_name_prefix: str) -> None:
    rail_center_x = sign * (LEAF_WIDTH * 0.5)
    hinge_stile_center_x = sign * (FRAME_BAR * 0.5)
    latch_stile_center_x = sign * (LEAF_WIDTH - FRAME_BAR * 0.5)
    picket_centers = [sign * x for x in (0.275, 0.55, 0.825, 1.10, 1.375)]

    part.visual(
        Box((FRAME_BAR, LEAF_DEPTH, LEAF_RAIL_SPAN)),
        origin=Origin(xyz=(hinge_stile_center_x, 0.0, -LEAF_RAIL_SPAN * 0.5)),
        material=metal,
        name="hinge_stile",
    )
    part.visual(
        Box((FRAME_BAR, LEAF_DEPTH, LEAF_RAIL_SPAN)),
        origin=Origin(xyz=(latch_stile_center_x, 0.0, -LEAF_RAIL_SPAN * 0.5)),
        material=metal,
        name="latch_stile",
    )
    part.visual(
        Box((LEAF_WIDTH, LEAF_DEPTH, FRAME_BAR)),
        origin=Origin(xyz=(rail_center_x, 0.0, 0.0)),
        material=metal,
        name="top_rail",
    )
    part.visual(
        Box((LEAF_WIDTH, LEAF_DEPTH, FRAME_BAR)),
        origin=Origin(xyz=(rail_center_x, 0.0, -LEAF_RAIL_SPAN)),
        material=metal,
        name="bottom_rail",
    )
    part.visual(
        Box((LEAF_WIDTH - FRAME_BAR * 0.35, LEAF_DEPTH, 0.035)),
        origin=Origin(xyz=(rail_center_x, 0.0, -0.62)),
        material=metal,
        name="mid_rail",
    )

    picket_height = LEAF_RAIL_SPAN + 0.02
    for index, picket_x in enumerate(picket_centers):
        part.visual(
            Box((PICKET_SIZE, PICKET_SIZE, picket_height)),
            origin=Origin(xyz=(picket_x, 0.0, -LEAF_RAIL_SPAN * 0.5)),
            material=metal,
            name=f"picket_{index}",
        )
        part.visual(
            Cylinder(radius=0.0075, length=0.085),
            origin=Origin(xyz=(picket_x, 0.0, 0.065)),
            material=ornament,
            name=f"finial_stem_{index}",
        )
        part.visual(
            Box((0.020, 0.012, 0.020)),
            origin=Origin(xyz=(picket_x, 0.0, 0.115), rpy=(0.0, pi / 4.0, 0.0)),
            material=ornament,
            name=f"finial_head_{index}",
        )

    medallion_center_x = sign * 0.825
    medallion_center_z = -0.79
    medallion = tube_from_spline_points(
        _ellipse_points(
            medallion_center_x,
            medallion_center_z,
            radius_x=0.205,
            radius_z=0.185,
            segments=36,
        ),
        radius=0.010,
        samples_per_segment=3,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )
    part.visual(
        mesh_from_geometry(medallion, f"{mesh_name_prefix}_medallion"),
        material=ornament,
        name="medallion_ring",
    )
    part.visual(
        Box((0.42, PICKET_SIZE, PICKET_SIZE)),
        origin=Origin(xyz=(medallion_center_x, 0.0, medallion_center_z)),
        material=ornament,
        name="medallion_crossbar",
    )

    part.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, 0.08, LEAF_RAIL_SPAN + 0.18)),
        mass=62.0,
        origin=Origin(xyz=(rail_center_x, 0.0, -LEAF_RAIL_SPAN * 0.5)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_decorative_driveway_gate")

    pillar_stone = model.material("pillar_stone", rgba=(0.70, 0.70, 0.68, 1.0))
    cap_stone = model.material("cap_stone", rgba=(0.60, 0.60, 0.58, 1.0))
    footing_concrete = model.material("footing_concrete", rgba=(0.55, 0.55, 0.54, 1.0))
    gate_metal = model.material("gate_metal", rgba=(0.13, 0.13, 0.14, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.22, 0.22, 0.23, 1.0))
    ornament_metal = model.material("ornament_metal", rgba=(0.16, 0.16, 0.17, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((4.44, 0.48, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=footing_concrete,
        name="foundation_plinth",
    )
    support_frame.visual(
        Box((PILLAR_WIDTH, PILLAR_DEPTH, PILLAR_HEIGHT)),
        origin=Origin(xyz=(-PILLAR_CENTER_X, 0.0, PILLAR_HEIGHT * 0.5)),
        material=pillar_stone,
        name="left_pillar",
    )
    support_frame.visual(
        Box((PILLAR_WIDTH, PILLAR_DEPTH, PILLAR_HEIGHT)),
        origin=Origin(xyz=(PILLAR_CENTER_X, 0.0, PILLAR_HEIGHT * 0.5)),
        material=pillar_stone,
        name="right_pillar",
    )
    support_frame.visual(
        Box((0.44, 0.54, 0.10)),
        origin=Origin(xyz=(-PILLAR_CENTER_X, 0.0, PILLAR_HEIGHT + 0.05)),
        material=cap_stone,
        name="left_cap",
    )
    support_frame.visual(
        Box((0.44, 0.54, 0.10)),
        origin=Origin(xyz=(PILLAR_CENTER_X, 0.0, PILLAR_HEIGHT + 0.05)),
        material=cap_stone,
        name="right_cap",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((4.44, 0.60, 2.10)),
        mass=560.0,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
    )

    left_lower_hinge = model.part("left_lower_hinge")
    _add_hinge_part(left_lower_hinge, sign=1.0, metal=hinge_metal, name_prefix="left_lower")

    left_upper_hinge = model.part("left_upper_hinge")
    _add_hinge_part(left_upper_hinge, sign=1.0, metal=hinge_metal, name_prefix="left_upper")

    left_leaf = model.part("left_leaf")
    _add_leaf_panel(left_leaf, sign=1.0, metal=gate_metal, ornament=ornament_metal, mesh_name_prefix="left_leaf")

    right_lower_hinge = model.part("right_lower_hinge")
    _add_hinge_part(right_lower_hinge, sign=-1.0, metal=hinge_metal, name_prefix="right_lower")

    right_upper_hinge = model.part("right_upper_hinge")
    _add_hinge_part(right_upper_hinge, sign=-1.0, metal=hinge_metal, name_prefix="right_upper")

    right_leaf = model.part("right_leaf")
    _add_leaf_panel(right_leaf, sign=-1.0, metal=gate_metal, ornament=ornament_metal, mesh_name_prefix="right_leaf")

    left_lower_joint = model.articulation(
        "left_lower_pintle",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=left_lower_hinge,
        origin=Origin(xyz=(-PILLAR_CENTER_X + PILLAR_WIDTH * 0.5, 0.0, HINGE_Z_LOWER)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=0.8),
    )
    left_upper_joint = model.articulation(
        "left_upper_pintle",
        ArticulationType.REVOLUTE,
        parent=left_lower_hinge,
        child=left_upper_hinge,
        origin=Origin(xyz=(0.0, 0.0, LEAF_RAIL_SPAN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=0.8),
    )
    model.articulation(
        "left_leaf_mount",
        ArticulationType.FIXED,
        parent=left_upper_hinge,
        child=left_leaf,
        origin=Origin(),
    )

    right_lower_joint = model.articulation(
        "right_lower_pintle",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=right_lower_hinge,
        origin=Origin(xyz=(PILLAR_CENTER_X - PILLAR_WIDTH * 0.5, 0.0, HINGE_Z_LOWER)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=0.8),
    )
    right_upper_joint = model.articulation(
        "right_upper_pintle",
        ArticulationType.REVOLUTE,
        parent=right_lower_hinge,
        child=right_upper_hinge,
        origin=Origin(xyz=(0.0, 0.0, LEAF_RAIL_SPAN)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=0.8),
    )
    model.articulation(
        "right_leaf_mount",
        ArticulationType.FIXED,
        parent=right_upper_hinge,
        child=right_leaf,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support_frame = object_model.get_part("support_frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")

    left_lower_pintle = object_model.get_articulation("left_lower_pintle")
    left_upper_pintle = object_model.get_articulation("left_upper_pintle")
    right_lower_pintle = object_model.get_articulation("right_lower_pintle")
    right_upper_pintle = object_model.get_articulation("right_upper_pintle")

    left_latch = left_leaf.get_visual("latch_stile")
    right_latch = right_leaf.get_visual("latch_stile")
    left_bottom = left_leaf.get_visual("bottom_rail")
    right_bottom = right_leaf.get_visual("bottom_rail")
    footing = support_frame.get_visual("foundation_plinth")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    def _aabb_dims(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(maxs[i] - mins[i] for i in range(3))

    left_limits = left_lower_pintle.motion_limits
    right_limits = right_lower_pintle.motion_limits
    ctx.check(
        "paired pintles use mirrored vertical hinge axes",
        left_lower_pintle.axis == (0.0, 0.0, 1.0)
        and left_upper_pintle.axis == (0.0, 0.0, 1.0)
        and right_lower_pintle.axis == (0.0, 0.0, -1.0)
        and right_upper_pintle.axis == (0.0, 0.0, -1.0)
        and left_limits is not None
        and right_limits is not None
        and left_limits.lower == 0.0
        and left_limits.upper == 0.8
        and right_limits.lower == 0.0
        and right_limits.upper == 0.8,
        details=(
            f"left_lower_axis={left_lower_pintle.axis}, "
            f"left_upper_axis={left_upper_pintle.axis}, "
            f"right_lower_axis={right_lower_pintle.axis}, "
            f"right_upper_axis={right_upper_pintle.axis}, "
            f"left_limits={left_limits}, right_limits={right_limits}"
        ),
    )

    with ctx.pose(
        {
            left_lower_pintle: 0.0,
            left_upper_pintle: 0.0,
            right_lower_pintle: 0.0,
            right_upper_pintle: 0.0,
        }
    ):
        ctx.expect_gap(
            right_leaf,
            left_leaf,
            axis="x",
            min_gap=0.02,
            max_gap=0.05,
            positive_elem=right_latch,
            negative_elem=left_latch,
            name="closed leaves meet with a narrow center seam",
        )
        ctx.expect_overlap(
            left_leaf,
            right_leaf,
            axes="z",
            min_overlap=1.10,
            name="both leaves present equal rectangular panel height",
        )
        ctx.expect_gap(
            left_leaf,
            support_frame,
            axis="z",
            min_gap=0.12,
            max_gap=0.25,
            positive_elem=left_bottom,
            negative_elem=footing,
            name="left gate clears the plinth above the driveway",
        )
        ctx.expect_gap(
            right_leaf,
            support_frame,
            axis="z",
            min_gap=0.12,
            max_gap=0.25,
            positive_elem=right_bottom,
            negative_elem=footing,
            name="right gate clears the plinth above the driveway",
        )

        left_rest_box = ctx.part_world_aabb(left_leaf)
        right_rest_box = ctx.part_world_aabb(right_leaf)
        left_dims = _aabb_dims(left_rest_box)
        right_dims = _aabb_dims(right_rest_box)
        ctx.check(
            "both gate leaves are equal size",
            left_dims is not None
            and right_dims is not None
            and all(abs(left_dims[i] - right_dims[i]) < 0.01 for i in range(3)),
            details=f"left_dims={left_dims}, right_dims={right_dims}",
        )
        left_rest_latch_center = _aabb_center(ctx.part_element_world_aabb(left_leaf, elem=left_latch))
        right_rest_latch_center = _aabb_center(ctx.part_element_world_aabb(right_leaf, elem=right_latch))

    with ctx.pose(
        {
            left_lower_pintle: 0.35,
            left_upper_pintle: 0.35,
            right_lower_pintle: 0.35,
            right_upper_pintle: 0.35,
        }
    ):
        left_open_latch_center = _aabb_center(ctx.part_element_world_aabb(left_leaf, elem=left_latch))
        right_open_latch_center = _aabb_center(ctx.part_element_world_aabb(right_leaf, elem=right_latch))
        ctx.check(
            "left leaf swings away from the center toward +y",
            left_rest_latch_center is not None
            and left_open_latch_center is not None
            and left_open_latch_center[1] > left_rest_latch_center[1] + 0.75,
            details=f"rest={left_rest_latch_center}, open={left_open_latch_center}",
        )
        ctx.check(
            "right leaf swings away from the center toward +y",
            right_rest_latch_center is not None
            and right_open_latch_center is not None
            and right_open_latch_center[1] > right_rest_latch_center[1] + 0.75,
            details=f"rest={right_rest_latch_center}, open={right_open_latch_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
