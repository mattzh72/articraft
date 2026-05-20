from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_HEIGHT = 0.180
LEAF_HEIGHT = 0.160
LEAF_WIDTH = 0.065
LEAF_THICKNESS = 0.010
MOUNT_PLATE_WIDTH = 0.082
MOUNT_PLATE_THICKNESS = 0.016
MOUNT_PLATE_INBOARD_X = -0.006
BARREL_RADIUS = 0.014
LEAF_Y_OFFSET = BARREL_RADIUS - 0.003
HINGE_CLEAR_X = 0.016
TAB_REACH_X = 0.018
PIN_RADIUS = 0.008
CHILD_BORE_RADIUS = 0.010
PIN_LENGTH = HINGE_HEIGHT
PIN_HEAD_RADIUS = 0.011
PIN_HEAD_HEIGHT = 0.004
PIN_TAIL_HEIGHT = 0.003
EDGE_FILLET = 0.0025
PLATE_FILLET = 0.0035

# Alternating knuckles: parent-child-parent-child-parent
PARENT_KNUCKLES = (
    (-0.082, -0.046),
    (-0.018, 0.018),
    (0.046, 0.082),
)
CHILD_KNUCKLES = (
    (-0.046, -0.018),
    (0.018, 0.046),
)


def _box_span(x_min: float, x_max: float, y_size: float, z_size: float, y_center: float = 0.0):
    return cq.Workplane("XY").box(
        x_max - x_min,
        y_size,
        z_size,
    ).translate(((x_min + x_max) * 0.5, y_center, 0.0))


def _solid_knuckle(z_min: float, z_max: float):
    return (
        cq.Workplane("XY")
        .circle(BARREL_RADIUS)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _tube_knuckle(z_min: float, z_max: float):
    outer = cq.Workplane("XY").circle(BARREL_RADIUS).extrude(z_max - z_min)
    bore = (
        cq.Workplane("XY")
        .circle(CHILD_BORE_RADIUS)
        .extrude(z_max - z_min + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(bore).translate((0.0, 0.0, z_min))


def _mounting_plate_shape():
    plate = _box_span(
        MOUNT_PLATE_INBOARD_X - MOUNT_PLATE_WIDTH,
        MOUNT_PLATE_INBOARD_X,
        MOUNT_PLATE_THICKNESS,
        HINGE_HEIGHT,
        y_center=-(LEAF_Y_OFFSET + 0.006),
    )
    return plate.edges("|Z").fillet(PLATE_FILLET)


def _leaf_plate_shape(direction: int):
    if direction < 0:
        plate = _box_span(
            -LEAF_WIDTH,
            -HINGE_CLEAR_X,
            LEAF_THICKNESS,
            LEAF_HEIGHT,
            y_center=-LEAF_Y_OFFSET,
        )
    else:
        plate = _box_span(
            HINGE_CLEAR_X,
            LEAF_WIDTH,
            LEAF_THICKNESS,
            LEAF_HEIGHT,
            y_center=LEAF_Y_OFFSET,
        )
    return plate.edges("|Z").fillet(EDGE_FILLET)


def _knuckle_tabs_shape(direction: int, segments):
    body = None
    if direction < 0:
        x_min, x_max = -TAB_REACH_X, 0.004
        y_center = -LEAF_Y_OFFSET
    else:
        x_min, x_max = -0.004, TAB_REACH_X
        y_center = LEAF_Y_OFFSET
    for z_min, z_max in segments:
        tab = _box_span(
            x_min,
            x_max,
            LEAF_THICKNESS,
            z_max - z_min,
            y_center=y_center,
        ).translate((0.0, 0.0, (z_min + z_max) * 0.5))
        body = tab if body is None else body.union(tab)
    return body


def _grounded_knuckles_shape():
    body = _knuckle_tabs_shape(-1, PARENT_KNUCKLES)
    for z_min, z_max in PARENT_KNUCKLES:
        body = body.union(_solid_knuckle(z_min, z_max))
    return body


def _swing_knuckles_shape():
    body = _knuckle_tabs_shape(1, CHILD_KNUCKLES)
    for z_min, z_max in CHILD_KNUCKLES:
        body = body.union(_tube_knuckle(z_min, z_max))
    return body


def _pin_shape():
    top_stem = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.078))
    )
    head = (
        cq.Workplane("XY")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_HEIGHT)
        .translate((0.0, 0.0, PIN_LENGTH * 0.5 - PIN_HEAD_HEIGHT))
    )
    bottom_stem = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(0.009)
        .translate((0.0, 0.0, -0.087))
    )
    tail = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS * 1.10)
        .extrude(PIN_TAIL_HEIGHT)
        .translate((0.0, 0.0, -PIN_LENGTH * 0.5))
    )
    return top_stem.union(head).union(bottom_stem).union(tail)


def _aabb_extent(aabb, axis_index: int) -> float:
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_weld_on_hinge")

    model.material("oxide_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("mount_plate", rgba=(0.26, 0.27, 0.29, 1.0))
    model.material("pin_steel", rgba=(0.69, 0.71, 0.74, 1.0))

    grounded_leaf = model.part("grounded_leaf")
    grounded_leaf.visual(
        mesh_from_cadquery(_mounting_plate_shape(), "mounting_plate"),
        material="mount_plate",
        name="mounting_plate",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_leaf_plate_shape(-1), "grounded_leaf_plate"),
        material="oxide_steel",
        name="grounded_leaf_plate",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_grounded_knuckles_shape(), "grounded_knuckles"),
        material="oxide_steel",
        name="grounded_knuckles",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_pin_shape(), "hinge_pin"),
        material="pin_steel",
        name="hinge_pin",
    )

    swing_leaf = model.part("swing_leaf")
    swing_leaf.visual(
        mesh_from_cadquery(_leaf_plate_shape(1), "swing_leaf_plate"),
        material="oxide_steel",
        name="swing_leaf_plate",
    )
    swing_leaf.visual(
        mesh_from_cadquery(_swing_knuckles_shape(), "swing_knuckles"),
        material="oxide_steel",
        name="swing_knuckles",
    )

    model.articulation(
        "hinge_pin_axis",
        ArticulationType.REVOLUTE,
        parent=grounded_leaf,
        child=swing_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded_leaf = object_model.get_part("grounded_leaf")
    swing_leaf = object_model.get_part("swing_leaf")
    hinge = object_model.get_articulation("hinge_pin_axis")

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

    ctx.check(
        "hinge_axis_is_vertical",
        tuple(hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {hinge.axis}",
    )
    ctx.check(
        "hinge_limits_open_away_from_mount",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 2.0,
        details="expected an outward-opening revolute range starting at 0 rad",
    )
    ctx.expect_contact(
        swing_leaf,
        grounded_leaf,
        elem_a="swing_knuckles",
        elem_b="grounded_knuckles",
        name="alternating_knuckles_are_supported",
    )

    plate_aabb = ctx.part_element_world_aabb(grounded_leaf, elem="mounting_plate")
    leaf_plate_aabb = ctx.part_element_world_aabb(grounded_leaf, elem="grounded_leaf_plate")
    if plate_aabb is None or leaf_plate_aabb is None:
        ctx.fail(
            "grounded_leaf_visuals_present",
            "expected mounting_plate and grounded_leaf_plate visuals on grounded_leaf",
        )
    else:
        plate_thickness = _aabb_extent(plate_aabb, 1)
        leaf_thickness = _aabb_extent(leaf_plate_aabb, 1)
        plate_height = _aabb_extent(plate_aabb, 2)
        leaf_height = _aabb_extent(leaf_plate_aabb, 2)
        ctx.check(
            "mounting_plate_reads_as_heavier_backer",
            plate_thickness > leaf_thickness + 0.004
            and plate_height > leaf_height + 0.015
            and plate_aabb[0][0] < leaf_plate_aabb[0][0] - 0.010,
            details=(
                "grounded leaf should carry a visibly thicker and wider mounting plate: "
                f"plate_thickness={plate_thickness:.4f}, leaf_thickness={leaf_thickness:.4f}, "
                f"plate_height={plate_height:.4f}, leaf_height={leaf_height:.4f}"
            ),
        )

    with ctx.pose({hinge: pi * 0.5}):
        ctx.expect_contact(
            swing_leaf,
            grounded_leaf,
            elem_a="swing_knuckles",
            elem_b="grounded_knuckles",
            name="knuckles_stay_supported_at_right_angle",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="right_angle_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
