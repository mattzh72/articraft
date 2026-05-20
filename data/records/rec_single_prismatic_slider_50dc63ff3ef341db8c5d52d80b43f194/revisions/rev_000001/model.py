from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.48
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.014

RAIL_LENGTH = 0.372
RAIL_WIDTH = 0.018
RAIL_HEIGHT = 0.012
RAIL_OFFSET_Y = 0.040

STOP_LENGTH = 0.012
STOP_WIDTH = 0.108
STOP_HEIGHT = 0.020
STOP_CAP_RADIUS = 0.004
STOP_CAP_HEIGHT = 0.003
STOP_CAP_OFFSET_Y = 0.032
STOP_CENTER_X = 0.204

SHUTTLE_LENGTH = 0.140
SHUTTLE_WIDTH = 0.106
SHUTTLE_BODY_Z0 = 0.008
SHUTTLE_BODY_HEIGHT = 0.034
BEARING_LENGTH = 0.112
BEARING_WIDTH = 0.026
BEARING_HEIGHT = 0.012

TRAVEL_HALF = 0.120


def _plate_shape() -> cq.Workplane:
    mount_x = BASE_LENGTH / 2.0 - 0.060
    mount_y = BASE_WIDTH / 2.0 - 0.040
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-mount_x, -mount_y),
                (-mount_x, mount_y),
                (mount_x, -mount_y),
                (mount_x, mount_y),
            ]
        )
        .cboreHole(0.007, 0.012, 0.004)
    )


def _rail_shape() -> cq.Workplane:
    half_w = RAIL_WIDTH / 2.0
    shoulder_z = RAIL_HEIGHT * 0.38
    crown_w = RAIL_WIDTH * 0.30
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (-half_w, 0.0),
                (-half_w, shoulder_z),
                (-crown_w, RAIL_HEIGHT),
                (crown_w, RAIL_HEIGHT),
                (half_w, shoulder_z),
                (half_w, 0.0),
            ]
        )
        .close()
        .extrude(RAIL_LENGTH / 2.0, both=True)
    )


def _stop_shape() -> cq.Workplane:
    block = cq.Workplane("XY").box(
        STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT, centered=(True, True, False)
    )
    cap_a = (
        cq.Workplane("XY")
        .cylinder(STOP_CAP_HEIGHT, STOP_CAP_RADIUS)
        .translate((0.0, -STOP_CAP_OFFSET_Y, STOP_HEIGHT + STOP_CAP_HEIGHT / 2.0))
    )
    cap_b = (
        cq.Workplane("XY")
        .cylinder(STOP_CAP_HEIGHT, STOP_CAP_RADIUS)
        .translate((0.0, STOP_CAP_OFFSET_Y, STOP_HEIGHT + STOP_CAP_HEIGHT / 2.0))
    )
    return block.union(cap_a).union(cap_b)


def _shuttle_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            SHUTTLE_LENGTH,
            SHUTTLE_WIDTH,
            SHUTTLE_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SHUTTLE_BODY_Z0))
    )
    left_bearing = (
        cq.Workplane("XY")
        .box(
            BEARING_LENGTH,
            BEARING_WIDTH,
            BEARING_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, RAIL_OFFSET_Y, 0.0))
    )
    right_bearing = (
        cq.Workplane("XY")
        .box(
            BEARING_LENGTH,
            BEARING_WIDTH,
            BEARING_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -RAIL_OFFSET_Y, 0.0))
    )

    shuttle = body.union(left_bearing).union(right_bearing)

    pocket_x = SHUTTLE_LENGTH / 2.0 - 0.036
    pocket_y = SHUTTLE_WIDTH / 2.0 - 0.028
    return (
        shuttle.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-pocket_x, -pocket_y),
                (-pocket_x, pocket_y),
                (pocket_x, -pocket_y),
                (pocket_x, pocket_y),
            ]
        )
        .circle(0.0045)
        .cutBlind(-0.003)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_shuttle_axis")

    model.material("base_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("rail_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("hardware_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("shuttle_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_plate_shape(), "base_plate"),
        material="base_black",
        name="plate",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape(), "left_rail"),
        origin=Origin(xyz=(0.0, RAIL_OFFSET_Y, BASE_THICKNESS)),
        material="rail_steel",
        name="left_rail",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape(), "right_rail"),
        origin=Origin(xyz=(0.0, -RAIL_OFFSET_Y, BASE_THICKNESS)),
        material="rail_steel",
        name="right_rail",
    )
    base.visual(
        mesh_from_cadquery(_stop_shape(), "left_stop"),
        origin=Origin(xyz=(-STOP_CENTER_X, 0.0, BASE_THICKNESS)),
        material="hardware_dark",
        name="left_stop",
    )
    base.visual(
        mesh_from_cadquery(_stop_shape(), "right_stop"),
        origin=Origin(xyz=(STOP_CENTER_X, 0.0, BASE_THICKNESS)),
        material="hardware_dark",
        name="right_stop",
    )
    stationary_height = BASE_THICKNESS + STOP_HEIGHT + STOP_CAP_HEIGHT
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, stationary_height)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, stationary_height / 2.0)),
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        mesh_from_cadquery(_shuttle_shape(), "shuttle"),
        material="shuttle_aluminum",
        name="carriage",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((SHUTTLE_LENGTH, SHUTTLE_WIDTH, SHUTTLE_BODY_Z0 + SHUTTLE_BODY_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, (SHUTTLE_BODY_Z0 + SHUTTLE_BODY_HEIGHT) / 2.0)),
    )

    model.articulation(
        "base_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF,
            upper=TRAVEL_HALF,
            effort=180.0,
            velocity=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shuttle = object_model.get_part("shuttle")
    slide = object_model.get_articulation("base_to_shuttle")

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

    limits = slide.motion_limits
    axis_ok = tuple(slide.axis) == (1.0, 0.0, 0.0)
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + TRAVEL_HALF) < 1e-9
        and abs(limits.upper - TRAVEL_HALF) < 1e-9
    )
    ctx.check(
        "slide_axis_and_limits",
        axis_ok and limits_ok,
        details=(
            f"axis={slide.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )

    ctx.expect_contact(shuttle, base, elem_b="left_rail", contact_tol=1e-6)
    ctx.expect_contact(shuttle, base, elem_b="right_rail", contact_tol=1e-6)

    lower_x = None
    upper_x = None
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({slide: limits.lower}):
            lower_pos = ctx.part_world_position(shuttle)
            lower_x = None if lower_pos is None else lower_pos[0]
            ctx.expect_gap(
                shuttle,
                base,
                axis="x",
                min_gap=0.006,
                max_gap=0.010,
                positive_elem="carriage",
                negative_elem="left_stop",
                name="left_end_stop_gap",
            )
            ctx.expect_contact(
                shuttle,
                base,
                elem_b="left_rail",
                contact_tol=1e-6,
                name="left_limit_left_rail_contact",
            )
            ctx.expect_contact(
                shuttle,
                base,
                elem_b="right_rail",
                contact_tol=1e-6,
                name="left_limit_right_rail_contact",
            )

        with ctx.pose({slide: limits.upper}):
            upper_pos = ctx.part_world_position(shuttle)
            upper_x = None if upper_pos is None else upper_pos[0]
            ctx.expect_gap(
                base,
                shuttle,
                axis="x",
                min_gap=0.006,
                max_gap=0.010,
                positive_elem="right_stop",
                negative_elem="carriage",
                name="right_end_stop_gap",
            )
            ctx.expect_contact(
                shuttle,
                base,
                elem_b="left_rail",
                contact_tol=1e-6,
                name="right_limit_left_rail_contact",
            )
            ctx.expect_contact(
                shuttle,
                base,
                elem_b="right_rail",
                contact_tol=1e-6,
                name="right_limit_right_rail_contact",
            )

    ctx.check(
        "shuttle_travels_along_positive_x",
        lower_x is not None and upper_x is not None and (upper_x - lower_x) > 0.23,
        details=f"lower_x={lower_x}, upper_x={upper_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
