from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BODY_LENGTH = 0.48
BODY_WIDTH = 0.22
BASE_HEIGHT = 0.06
RAIL_LENGTH = 0.24
RAIL_WIDTH = 0.024
RAIL_HEIGHT = 0.018
RAIL_OFFSET_Y = 0.05

CARRIAGE_LENGTH = 0.10
CARRIAGE_PAD_WIDTH = 0.022
CARRIAGE_PAD_HEIGHT = 0.016
CARRIAGE_BRIDGE_WIDTH = 0.112
CARRIAGE_BRIDGE_HEIGHT = 0.018
CARRIAGE_UPPER_LENGTH = 0.078
CARRIAGE_UPPER_WIDTH = 0.086
CARRIAGE_UPPER_HEIGHT = 0.026
CARRIAGE_NOSE_LENGTH = 0.04
CARRIAGE_NOSE_WIDTH = 0.064
CARRIAGE_NOSE_HEIGHT = 0.03
CARRIAGE_NOSE_CENTER_X = 0.054
SPINDLE_AXIS_Z = 0.041
SPINDLE_MOUNT_X = 0.072

SPINDLE_FLANGE_LENGTH = 0.014
SPINDLE_FLANGE_RADIUS = 0.026
SPINDLE_BODY_LENGTH = 0.045
SPINDLE_BODY_RADIUS = 0.022
SPINDLE_TIP_LENGTH = 0.033
SPINDLE_TIP_BASE_RADIUS = 0.01
SPINDLE_TIP_RADIUS = 0.0035

SLIDE_HOME_X = -0.05
SLIDE_TRAVEL = 0.08
CARRIAGE_HOME_Z = BASE_HEIGHT + RAIL_HEIGHT


def _body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, BASE_HEIGHT).translate((0.0, 0.0, BASE_HEIGHT / 2.0))

    rear_housing = (
        cq.Workplane("XY")
        .box(0.09, 0.11, 0.034)
        .translate((-0.175, 0.0, BASE_HEIGHT + 0.015))
    )
    front_bumper = (
        cq.Workplane("XY")
        .box(0.06, 0.10, 0.016)
        .translate((0.18, 0.0, BASE_HEIGHT + 0.006))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
        .translate((0.0, RAIL_OFFSET_Y, BASE_HEIGHT - 0.002 + (RAIL_HEIGHT / 2.0)))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
        .translate((0.0, -RAIL_OFFSET_Y, BASE_HEIGHT - 0.002 + (RAIL_HEIGHT / 2.0)))
    )

    body = base.union(rear_housing).union(front_bumper).union(left_rail).union(right_rail)
    return body.edges("|Z").fillet(0.01)


def _carriage_shape() -> cq.Workplane:
    left_pad = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_PAD_WIDTH, CARRIAGE_PAD_HEIGHT)
        .translate((0.0, RAIL_OFFSET_Y, CARRIAGE_PAD_HEIGHT / 2.0))
    )
    right_pad = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_PAD_WIDTH, CARRIAGE_PAD_HEIGHT)
        .translate((0.0, -RAIL_OFFSET_Y, CARRIAGE_PAD_HEIGHT / 2.0))
    )

    bridge = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_BRIDGE_WIDTH, CARRIAGE_BRIDGE_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_PAD_HEIGHT - 0.004 + (CARRIAGE_BRIDGE_HEIGHT / 2.0)))
    )
    upper_block = (
        cq.Workplane("XY")
        .box(CARRIAGE_UPPER_LENGTH, CARRIAGE_UPPER_WIDTH, CARRIAGE_UPPER_HEIGHT)
        .translate((-0.006, 0.0, 0.026 + (CARRIAGE_UPPER_HEIGHT / 2.0)))
    )
    spindle_nose = (
        cq.Workplane("XY")
        .box(CARRIAGE_NOSE_LENGTH, CARRIAGE_NOSE_WIDTH, CARRIAGE_NOSE_HEIGHT)
        .translate((CARRIAGE_NOSE_CENTER_X, 0.0, 0.024 + (CARRIAGE_NOSE_HEIGHT / 2.0)))
    )

    return left_pad.union(right_pad).union(bridge).union(upper_block).union(spindle_nose)


def _spindle_shape() -> cq.Workplane:
    flange = cq.Workplane("YZ").circle(SPINDLE_FLANGE_RADIUS).extrude(SPINDLE_FLANGE_LENGTH)
    body = (
        cq.Workplane("YZ")
        .circle(SPINDLE_BODY_RADIUS)
        .extrude(SPINDLE_BODY_LENGTH)
        .translate((SPINDLE_FLANGE_LENGTH, 0.0, 0.0))
    )
    tip = (
        cq.Workplane("YZ")
        .circle(SPINDLE_TIP_BASE_RADIUS)
        .workplane(offset=SPINDLE_TIP_LENGTH)
        .circle(SPINDLE_TIP_RADIUS)
        .loft(combine=True)
        .translate((SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH, 0.0, 0.0))
    )
    encoder_boss = (
        cq.Workplane("XY")
        .box(0.014, 0.012, 0.01)
        .translate((0.026, 0.02, 0.012))
    )
    return flange.union(body).union(tip).union(encoder_boss)


def _size_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_shuttle_spindle")

    model.material("body_charcoal", color=(0.18, 0.2, 0.23))
    model.material("carriage_gray", color=(0.58, 0.6, 0.64))
    model.material("spindle_steel", color=(0.72, 0.74, 0.77))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="body_charcoal",
        name="body_base",
    )
    body.visual(
        Box((0.10, 0.13, 0.04)),
        origin=Origin(xyz=(-0.17, 0.0, 0.02)),
        material="body_charcoal",
        name="rear_housing",
    )
    body.visual(
        Box((0.06, 0.10, 0.02)),
        origin=Origin(xyz=(0.19, 0.0, 0.01)),
        material="body_charcoal",
        name="front_nose",
    )
    body.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_OFFSET_Y, BASE_HEIGHT + (RAIL_HEIGHT / 2.0))),
        material="body_charcoal",
        name="left_rail",
    )
    body.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_OFFSET_Y, BASE_HEIGHT + (RAIL_HEIGHT / 2.0))),
        material="body_charcoal",
        name="right_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_PAD_WIDTH, CARRIAGE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_OFFSET_Y, CARRIAGE_PAD_HEIGHT / 2.0)),
        material="carriage_gray",
        name="left_runner",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_PAD_WIDTH, CARRIAGE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_OFFSET_Y, CARRIAGE_PAD_HEIGHT / 2.0)),
        material="carriage_gray",
        name="right_runner",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_BRIDGE_WIDTH, CARRIAGE_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.013 + (CARRIAGE_BRIDGE_HEIGHT / 2.0)),
        ),
        material="carriage_gray",
        name="runner_bridge",
    )
    carriage.visual(
        Box((CARRIAGE_UPPER_LENGTH, CARRIAGE_UPPER_WIDTH, CARRIAGE_UPPER_HEIGHT)),
        origin=Origin(
            xyz=(-0.006, 0.0, 0.031 + (CARRIAGE_UPPER_HEIGHT / 2.0)),
        ),
        material="carriage_gray",
        name="carriage_top",
    )
    carriage.visual(
        Box((CARRIAGE_NOSE_LENGTH, CARRIAGE_NOSE_WIDTH, CARRIAGE_NOSE_HEIGHT)),
        origin=Origin(
            xyz=(CARRIAGE_NOSE_CENTER_X, 0.0, 0.03 + (CARRIAGE_NOSE_HEIGHT / 2.0)),
        ),
        material="carriage_gray",
        name="spindle_nose",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_spindle_shape(), "service_shuttle_spindle"),
        material="spindle_steel",
        name="spindle_shell",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, CARRIAGE_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(SPINDLE_MOUNT_X, 0.0, SPINDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=8.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("body_to_carriage")
    spin = object_model.get_articulation("carriage_to_spindle")

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

    ctx.expect_contact(
        carriage,
        body,
        contact_tol=0.0015,
        name="carriage is supported on shuttle guide rails at home",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        contact_tol=0.001,
        name="spindle cartridge mounts directly to carriage nose",
    )
    ctx.expect_origin_gap(
        spindle,
        carriage,
        axis="x",
        min_gap=0.065,
        max_gap=0.085,
        name="spindle sits at one end of the carriage",
    )

    home_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            body,
            contact_tol=0.0015,
            name="carriage remains supported at full extension",
        )
        extended_position = ctx.part_world_position(carriage)

    if home_position is None or extended_position is None:
        ctx.fail("carriage travel can be measured", "carriage world position was unavailable")
    else:
        dx = extended_position[0] - home_position[0]
        dy = extended_position[1] - home_position[1]
        dz = extended_position[2] - home_position[2]
        ctx.check(
            "carriage prismatic joint translates along +x",
            abs(dx - SLIDE_TRAVEL) <= 1e-4 and abs(dy) <= 1e-6 and abs(dz) <= 1e-6,
            f"expected +x translation {SLIDE_TRAVEL:.3f} m, got dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}",
        )

    ctx.check(
        "spindle articulation rotates about its own axis",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        f"expected spindle axis (1, 0, 0), got {spin.axis}",
    )

    carriage_size = _size_from_aabb(ctx.part_world_aabb(carriage))
    spindle_size = _size_from_aabb(ctx.part_world_aabb(spindle))
    if carriage_size is None or spindle_size is None:
        ctx.fail("spindle and carriage can be sized", "missing AABB for carriage or spindle")
    else:
        ctx.check(
            "spindle cartridge is visibly smaller than the carriage",
            spindle_size[0] < carriage_size[0]
            and spindle_size[1] < carriage_size[1]
            and spindle_size[2] < carriage_size[2],
            f"carriage size={carriage_size}, spindle size={spindle_size}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
