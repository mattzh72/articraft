from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


LEFT_SUPPORT_X = 0.0
RIGHT_SUPPORT_X = 0.64
SUPPORT_SPAN = RIGHT_SUPPORT_X - LEFT_SUPPORT_X
SHAFT_AXIS_Z = 0.29

FRAME_LENGTH = 0.96
FRAME_CENTER_X = 0.33
FRAME_MIN_X = FRAME_CENTER_X - (FRAME_LENGTH / 2.0)
FRAME_MAX_X = FRAME_CENTER_X + (FRAME_LENGTH / 2.0)
RAIL_Y = 0.11
RAIL_WIDTH = 0.06
RAIL_HEIGHT = 0.08

SUPPORT_TOWER_LENGTH = 0.16
SUPPORT_TOWER_WIDTH = 0.24
SUPPORT_TOWER_HEIGHT = 0.12
SUPPORT_PAD_LENGTH = 0.18
SUPPORT_PAD_WIDTH = 0.16
SUPPORT_PAD_THICKNESS = 0.02
SUPPORT_PAD_CENTER_Z = SHAFT_AXIS_Z - 0.10

BLOCK_LENGTH = 0.11
BLOCK_WIDTH = 0.15
BLOCK_BASE_THICKNESS = 0.018
BLOCK_BOTTOM_TO_AXIS = 0.09
BLOCK_OUTER_RADIUS = 0.045
SHAFT_RADIUS = 0.025
BLOCK_BORE_RADIUS = SHAFT_RADIUS + 0.004
LEFT_SHAFT_OVERHANG = 0.10
RIGHT_SHAFT_OVERHANG = 0.25
SHAFT_BAR_MIN_X = LEFT_SUPPORT_X - LEFT_SHAFT_OVERHANG
SHAFT_BAR_MAX_X = RIGHT_SUPPORT_X + RIGHT_SHAFT_OVERHANG
SHAFT_BAR_LENGTH = SHAFT_BAR_MAX_X - SHAFT_BAR_MIN_X
SHAFT_BAR_CENTER_X = (SHAFT_BAR_MAX_X + SHAFT_BAR_MIN_X) / 2.0

LEFT_BLOCK_MIN_X = LEFT_SUPPORT_X - (BLOCK_LENGTH / 2.0)
LEFT_BLOCK_MAX_X = LEFT_SUPPORT_X + (BLOCK_LENGTH / 2.0)
RIGHT_BLOCK_MIN_X = RIGHT_SUPPORT_X - (BLOCK_LENGTH / 2.0)
RIGHT_BLOCK_MAX_X = RIGHT_SUPPORT_X + (BLOCK_LENGTH / 2.0)

COLLAR_THICKNESS = 0.022
COLLAR_RADIUS = 0.042
LEFT_COLLAR_CENTER_X = LEFT_BLOCK_MIN_X - (COLLAR_THICKNESS / 2.0)
RIGHT_HUB_START_X = RIGHT_BLOCK_MAX_X
FLANGE_HUB_LENGTH = 0.095
FLANGE_HUB_RADIUS = 0.05
FLANGE_HUB_CENTER_X = RIGHT_HUB_START_X + (FLANGE_HUB_LENGTH / 2.0)
FLANGE_PLATE_THICKNESS = 0.024
FLANGE_CENTER_X = RIGHT_HUB_START_X + FLANGE_HUB_LENGTH + (FLANGE_PLATE_THICKNESS / 2.0)
FLANGE_RADIUS = 0.145
FLANGE_BOLT_CIRCLE_RADIUS = 0.078
FLANGE_BOLT_HOLE_RADIUS = 0.009


def _box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def _cylinder_x(
    radius: float,
    length: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane(
            "YZ",
            origin=(center_x - (length / 2.0), center_y, center_z),
        )
        .circle(radius)
        .extrude(length)
    )


def _cylinder_z(
    radius: float,
    length: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane(
            "XY",
            origin=(center_x, center_y, center_z - (length / 2.0)),
        )
        .circle(radius)
        .extrude(length)
    )


def _frame_shape() -> cq.Workplane:
    frame = _box(FRAME_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, (FRAME_CENTER_X, RAIL_Y, RAIL_HEIGHT / 2.0))
    frame = frame.union(
        _box(FRAME_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, (FRAME_CENTER_X, -RAIL_Y, RAIL_HEIGHT / 2.0))
    )

    frame = frame.union(_box(0.08, 0.30, 0.06, (FRAME_MIN_X + 0.04, 0.0, 0.03)))
    frame = frame.union(_box(0.08, 0.30, 0.06, (FRAME_MAX_X - 0.04, 0.0, 0.03)))
    frame = frame.union(_box(0.10, 0.22, 0.05, (FRAME_CENTER_X, 0.0, 0.055)))

    for support_x in (LEFT_SUPPORT_X, RIGHT_SUPPORT_X):
        tower = _box(
            SUPPORT_TOWER_LENGTH,
            SUPPORT_TOWER_WIDTH,
            SUPPORT_TOWER_HEIGHT,
            (support_x, 0.0, RAIL_HEIGHT + (SUPPORT_TOWER_HEIGHT / 2.0)),
        )
        tower = tower.cut(
            _box(
                0.10,
                0.13,
                0.08,
                (support_x, 0.0, RAIL_HEIGHT + 0.055),
            )
        )
        tower = tower.union(
            _box(
                SUPPORT_PAD_LENGTH,
                SUPPORT_PAD_WIDTH,
                SUPPORT_PAD_THICKNESS,
                (support_x, 0.0, SUPPORT_PAD_CENTER_Z),
            )
        )
        frame = frame.union(tower)

    return frame


def _pillow_block_shape() -> cq.Workplane:
    foot = _box(
        BLOCK_LENGTH,
        BLOCK_WIDTH,
        BLOCK_BASE_THICKNESS,
        (0.0, 0.0, -BLOCK_BOTTOM_TO_AXIS + (BLOCK_BASE_THICKNESS / 2.0)),
    )
    left_pedestal = _box(BLOCK_LENGTH * 0.90, 0.034, 0.078, (0.0, -0.042, -0.041))
    right_pedestal = _box(BLOCK_LENGTH * 0.90, 0.034, 0.078, (0.0, 0.042, -0.041))
    top_bridge = _box(BLOCK_LENGTH * 0.86, 0.084, 0.030, (0.0, 0.0, 0.030))
    bearing_cartridge = _cylinder_x(BLOCK_OUTER_RADIUS, BLOCK_LENGTH)
    grease_boss = _cylinder_z(0.007, 0.018, center_z=0.057)

    block = foot.union(left_pedestal).union(right_pedestal).union(top_bridge).union(bearing_cartridge).union(grease_boss)
    block = block.cut(_cylinder_x(BLOCK_BORE_RADIUS, BLOCK_LENGTH + 0.01))
    return block


def _shaft_bar_shape() -> cq.Workplane:
    return _cylinder_x(SHAFT_RADIUS, SHAFT_BAR_LENGTH, center_x=SHAFT_BAR_CENTER_X)


def _retaining_collar_shape(center_x: float) -> cq.Workplane:
    return _cylinder_x(COLLAR_RADIUS, COLLAR_THICKNESS, center_x=center_x)


def _flange_hub_shape() -> cq.Workplane:
    return _cylinder_x(FLANGE_HUB_RADIUS, FLANGE_HUB_LENGTH, center_x=FLANGE_HUB_CENTER_X)


def _flange_plate_shape() -> cq.Workplane:
    flange = _cylinder_x(FLANGE_RADIUS, FLANGE_PLATE_THICKNESS, center_x=FLANGE_CENTER_X)
    for hole_index in range(6):
        angle = (2.0 * math.pi * hole_index) / 6.0
        hole_y = FLANGE_BOLT_CIRCLE_RADIUS * math.cos(angle)
        hole_z = FLANGE_BOLT_CIRCLE_RADIUS * math.sin(angle)
        flange = flange.cut(
            _cylinder_x(
                FLANGE_BOLT_HOLE_RADIUS,
                FLANGE_PLATE_THICKNESS + 0.01,
                center_x=FLANGE_CENTER_X,
                center_y=hole_y,
                center_z=hole_z,
            )
        )

    return flange


def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_spin_fixture")

    frame_finish = model.material("frame_finish", color=(0.16, 0.16, 0.18))
    block_finish = model.material("block_finish", color=(0.12, 0.40, 0.24))
    shaft_finish = model.material("shaft_finish", color=(0.72, 0.73, 0.75))
    flange_finish = model.material("flange_finish", color=(0.76, 0.77, 0.80))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_weldment"),
        material=frame_finish,
        name="frame_weldment",
    )

    left_block = model.part("left_pillow_block")
    left_block.visual(
        mesh_from_cadquery(_pillow_block_shape(), "left_pillow_block_housing"),
        material=block_finish,
        name="housing",
    )

    right_block = model.part("right_pillow_block")
    right_block.visual(
        mesh_from_cadquery(_pillow_block_shape(), "right_pillow_block_housing"),
        material=block_finish,
        name="housing",
    )

    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(_shaft_bar_shape(), "shaft_bar"),
        material=shaft_finish,
        name="shaft_bar",
    )
    shaft.visual(
        mesh_from_cadquery(_retaining_collar_shape(LEFT_COLLAR_CENTER_X), "left_retaining_collar"),
        material=shaft_finish,
        name="left_retaining_collar",
    )
    shaft.visual(
        mesh_from_cadquery(_flange_hub_shape(), "flange_hub"),
        material=flange_finish,
        name="flange_hub",
    )
    shaft.visual(
        mesh_from_cadquery(_flange_plate_shape(), "drive_flange"),
        material=flange_finish,
        name="drive_flange",
    )

    model.articulation(
        "frame_to_left_pillow_block",
        ArticulationType.FIXED,
        parent=frame,
        child=left_block,
        origin=Origin(xyz=(LEFT_SUPPORT_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "frame_to_right_pillow_block",
        ArticulationType.FIXED,
        parent=frame,
        child=right_block,
        origin=Origin(xyz=(RIGHT_SUPPORT_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(LEFT_SUPPORT_X, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    part_names = {part.name for part in object_model.parts}
    joint_names = {articulation.name for articulation in object_model.articulations}
    for expected_part in ("frame", "left_pillow_block", "right_pillow_block", "shaft"):
        ctx.check(
            f"has_{expected_part}",
            expected_part in part_names,
            f"Missing required part: {expected_part}",
        )
    ctx.check(
        "has_frame_to_shaft_joint",
        "frame_to_shaft" in joint_names,
        "Missing required shaft articulation",
    )

    frame = object_model.get_part("frame")
    left_block = object_model.get_part("left_pillow_block")
    right_block = object_model.get_part("right_pillow_block")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("frame_to_shaft")

    ctx.check(
        "shaft_joint_is_continuous_about_longitudinal_axis",
        (
            shaft_spin.articulation_type == ArticulationType.CONTINUOUS
            and _axis_matches(tuple(shaft_spin.axis), (1.0, 0.0, 0.0))
            and shaft_spin.motion_limits is not None
            and shaft_spin.motion_limits.lower is None
            and shaft_spin.motion_limits.upper is None
        ),
        "Shaft should use one continuous revolute-style articulation about +X",
    )

    ctx.expect_contact(
        left_block,
        frame,
        contact_tol=0.0015,
        name="left_pillow_block_seats_on_frame",
    )
    ctx.expect_contact(
        right_block,
        frame,
        contact_tol=0.0015,
        name="right_pillow_block_seats_on_frame",
    )
    ctx.expect_origin_gap(
        right_block,
        left_block,
        axis="x",
        min_gap=SUPPORT_SPAN - 0.001,
        max_gap=SUPPORT_SPAN + 0.001,
        name="pillow_blocks_span_the_fixture",
    )
    ctx.expect_origin_distance(
        left_block,
        shaft,
        axes="yz",
        max_dist=0.0005,
        name="shaft_axis_matches_left_support",
    )
    ctx.expect_overlap(
        shaft,
        left_block,
        axes="x",
        min_overlap=BLOCK_LENGTH * 0.90,
        elem_a="shaft_bar",
        name="shaft_runs_through_left_pillow_block",
    )
    ctx.expect_overlap(
        shaft,
        right_block,
        axes="x",
        min_overlap=BLOCK_LENGTH * 0.90,
        elem_a="shaft_bar",
        name="shaft_runs_through_right_pillow_block",
    )
    ctx.expect_contact(
        shaft,
        left_block,
        elem_a="left_retaining_collar",
        contact_tol=0.001,
        name="left_retaining_collar_locates_shaft_against_left_block",
    )
    ctx.expect_contact(
        shaft,
        right_block,
        elem_a="flange_hub",
        contact_tol=0.001,
        name="flange_hub_bears_against_right_block",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="z",
        min_gap=0.055,
        positive_elem="shaft_bar",
        name="shaft_bar_clears_the_frame",
    )
    ctx.expect_gap(
        shaft,
        right_block,
        axis="x",
        min_gap=0.09,
        max_gap=0.11,
        positive_elem="drive_flange",
        name="flange_sits_just_outboard_of_right_block",
    )

    shaft_bar_aabb = ctx.part_element_world_aabb(shaft, elem="shaft_bar")
    flange_aabb = ctx.part_element_world_aabb(shaft, elem="drive_flange")
    if shaft_bar_aabb is None or flange_aabb is None:
        ctx.fail(
            "shaft_visuals_accessible",
            "Could not resolve shaft_bar or drive_flange AABB for proportion checks",
        )
    else:
        shaft_diameter = min(
            shaft_bar_aabb[1][1] - shaft_bar_aabb[0][1],
            shaft_bar_aabb[1][2] - shaft_bar_aabb[0][2],
        )
        flange_diameter = max(
            flange_aabb[1][1] - flange_aabb[0][1],
            flange_aabb[1][2] - flange_aabb[0][2],
        )
        ctx.check(
            "flange_reads_as_large_circular_end_fixture",
            flange_diameter >= shaft_diameter * 5.0,
            (
                f"Expected flange diameter to dwarf shaft diameter; "
                f"got flange={flange_diameter:.3f} m, shaft={shaft_diameter:.3f} m"
            ),
        )

    with ctx.pose({shaft_spin: 1.3}):
        ctx.expect_origin_distance(
            left_block,
            shaft,
            axes="yz",
            max_dist=0.0005,
            name="shaft_stays_concentric_when_spun",
        )
        ctx.expect_overlap(
            shaft,
            right_block,
            axes="x",
            min_overlap=BLOCK_LENGTH * 0.90,
            elem_a="shaft_bar",
            name="shaft_stays_supported_in_right_block_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
