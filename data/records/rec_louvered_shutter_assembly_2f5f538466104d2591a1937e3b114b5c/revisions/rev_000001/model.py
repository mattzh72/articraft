from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


OUTER_WIDTH = 0.70
OUTER_HEIGHT = 1.20
FRAME_DEPTH = 0.055
STILE_WIDTH = 0.075
RAIL_HEIGHT = 0.080

SLAT_COUNT = 8
SLAT_LENGTH = 0.520
SLAT_CHORD = 0.078
SLAT_THICKNESS = 0.014
PIN_RADIUS = 0.006
PIN_LENGTH = 0.096
PIN_CENTER_X = 0.303
PIN_CAP_RADIUS = 0.012
PIN_CAP_LENGTH = 0.006
PIN_CAP_CENTER_X = OUTER_WIDTH * 0.5 + PIN_CAP_LENGTH * 0.5

LOWER_SLAT_Z = -0.455
SLAT_PITCH = 0.130
SLAT_OPEN_ANGLE = math.pi / 2.0


def _slat_z(index: int) -> float:
    return LOWER_SLAT_Z + index * SLAT_PITCH


def _louver_blade_mesh():
    profile = rounded_rect_profile(
        SLAT_CHORD,
        SLAT_THICKNESS,
        radius=SLAT_THICKNESS * 0.45,
        corner_segments=6,
    )
    blade = ExtrudeGeometry.centered(profile, SLAT_LENGTH, cap=True, closed=True)
    # ExtrudeGeometry grows along local Z. Rotate the finished rounded-rectangle
    # extrusion so the long louver blade axis is the local/world X axis.
    blade.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(blade, "rounded_louver_blade")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_louvered_shutter")

    painted_white = model.material("painted_white", rgba=(0.88, 0.87, 0.82, 1.0))
    slat_paint = model.material("warm_off_white", rgba=(0.80, 0.78, 0.70, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    shadow_dark = model.material("shadow_dark", rgba=(0.11, 0.10, 0.09, 1.0))

    blade_mesh = _louver_blade_mesh()

    frame = model.part("frame")
    stile_z_center = 0.0
    rail_x_center = 0.0
    left_stile_x = -OUTER_WIDTH * 0.5 + STILE_WIDTH * 0.5
    right_stile_x = OUTER_WIDTH * 0.5 - STILE_WIDTH * 0.5
    top_rail_z = OUTER_HEIGHT * 0.5 - RAIL_HEIGHT * 0.5
    bottom_rail_z = -OUTER_HEIGHT * 0.5 + RAIL_HEIGHT * 0.5

    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(left_stile_x, 0.0, stile_z_center)),
        material=painted_white,
        name="side_stile_0",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(right_stile_x, 0.0, stile_z_center)),
        material=painted_white,
        name="side_stile_1",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(rail_x_center, 0.0, top_rail_z)),
        material=painted_white,
        name="top_rail",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(rail_x_center, 0.0, bottom_rail_z)),
        material=painted_white,
        name="bottom_rail",
    )

    # Thin dark inner rabbets along the inside of the side stiles give the
    # opening depth without closing off the shutter.
    frame.visual(
        Box((0.010, 0.010, OUTER_HEIGHT - 2.0 * RAIL_HEIGHT)),
        origin=Origin(xyz=(-OUTER_WIDTH * 0.5 + STILE_WIDTH + 0.003, FRAME_DEPTH * 0.5 - 0.002, 0.0)),
        material=shadow_dark,
        name="inner_reveal_0",
    )
    frame.visual(
        Box((0.010, 0.010, OUTER_HEIGHT - 2.0 * RAIL_HEIGHT)),
        origin=Origin(xyz=(OUTER_WIDTH * 0.5 - STILE_WIDTH - 0.003, FRAME_DEPTH * 0.5 - 0.002, 0.0)),
        material=shadow_dark,
        name="inner_reveal_1",
    )

    frame.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        mass=5.0,
        origin=Origin(),
    )

    for i in range(SLAT_COUNT):
        slat = model.part(f"slat_{i}")
        slat.visual(
            blade_mesh,
            origin=Origin(),
            material=slat_paint,
            name="blade",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(-PIN_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized_steel,
            name="pin_0",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(PIN_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized_steel,
            name="pin_1",
        )
        slat.visual(
            Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_LENGTH),
            origin=Origin(
                xyz=(-PIN_CAP_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized_steel,
            name="pin_head_0",
        )
        slat.visual(
            Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_LENGTH),
            origin=Origin(
                xyz=(PIN_CAP_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized_steel,
            name="pin_head_1",
        )
        slat.inertial = Inertial.from_geometry(
            Box((SLAT_LENGTH + 2.0 * PIN_LENGTH, SLAT_THICKNESS, SLAT_CHORD)),
            mass=0.28,
            origin=Origin(),
        )

        model.articulation(
            f"slat_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, _slat_z(i))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.0,
                lower=0.0,
                upper=SLAT_OPEN_ANGLE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    ctx.check(
        "eight_uncoupled_slats",
        all(object_model.get_part(f"slat_{i}") is not None for i in range(SLAT_COUNT)),
        "Expected eight separate slat parts.",
    )
    ctx.check(
        "each_slat_has_own_pivot",
        all(object_model.get_articulation(f"slat_pivot_{i}") is not None for i in range(SLAT_COUNT)),
        "Every slat should have its own revolute joint.",
    )

    for i in range(SLAT_COUNT):
        slat = object_model.get_part(f"slat_{i}")
        pivot = object_model.get_articulation(f"slat_pivot_{i}")
        if slat is None or pivot is None:
            continue

        ctx.allow_overlap(
            frame,
            slat,
            elem_a="side_stile_0",
            elem_b="pin_0",
            reason="The left pivot pin is intentionally represented as a shaft captured through the side stile bore.",
        )
        ctx.allow_overlap(
            frame,
            slat,
            elem_a="side_stile_1",
            elem_b="pin_1",
            reason="The right pivot pin is intentionally represented as a shaft captured through the side stile bore.",
        )
        ctx.expect_overlap(
            slat,
            frame,
            axes="x",
            elem_a="pin_0",
            elem_b="side_stile_0",
            min_overlap=0.040,
            name=f"slat_{i}_left_pin_enters_stile",
        )
        ctx.expect_overlap(
            slat,
            frame,
            axes="x",
            elem_a="pin_1",
            elem_b="side_stile_1",
            min_overlap=0.040,
            name=f"slat_{i}_right_pin_enters_stile",
        )
        ctx.expect_within(
            slat,
            frame,
            axes="z",
            inner_elem="pin_0",
            outer_elem="side_stile_0",
            margin=0.0,
            name=f"slat_{i}_left_pin_height_inside_stile",
        )
        ctx.expect_within(
            slat,
            frame,
            axes="z",
            inner_elem="pin_1",
            outer_elem="side_stile_1",
            margin=0.0,
            name=f"slat_{i}_right_pin_height_inside_stile",
        )

        limits = pivot.motion_limits
        ctx.check(
            f"slat_{i}_quarter_turn_limits",
            limits is not None
            and limits.lower == 0.0
            and abs(float(limits.upper) - SLAT_OPEN_ANGLE) < 1.0e-6,
            details=f"limits={limits}",
        )

    first_slat = object_model.get_part("slat_0")
    first_pivot = object_model.get_articulation("slat_pivot_0")
    if first_slat is not None and first_pivot is not None:
        closed_aabb = ctx.part_element_world_aabb(first_slat, elem="blade")
        with ctx.pose({first_pivot: SLAT_OPEN_ANGLE}):
            open_aabb = ctx.part_element_world_aabb(first_slat, elem="blade")

        def _extent(aabb, axis: int) -> float:
            if aabb is None:
                return 0.0
            lo, hi = aabb
            return float(hi[axis] - lo[axis])

        ctx.check(
            "open_pose_turns_blade_broad_face",
            closed_aabb is not None
            and open_aabb is not None
            and _extent(open_aabb, 1) > _extent(closed_aabb, 1) + 0.045
            and _extent(open_aabb, 2) < _extent(closed_aabb, 2) - 0.045,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
