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


BASE_LENGTH = 0.62
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.018
RAIL_LENGTH = 0.50
RAIL_WIDTH = 0.028
RAIL_PITCH = 0.12
RAIL_EMBED = 0.001
RAIL_EXPOSED = 0.010
RAIL_TOTAL_HEIGHT = RAIL_EMBED + RAIL_EXPOSED
RAIL_TOP_Z = BASE_THICKNESS + RAIL_EXPOSED

CARRIAGE_LENGTH = 0.19
CARRIAGE_WIDTH = 0.16
CARRIAGE_PAD_LENGTH = 0.15
CARRIAGE_PAD_WIDTH = 0.024
CARRIAGE_PAD_EXPOSED = 0.008
CARRIAGE_PAD_EMBED = 0.001
CARRIAGE_PAD_TOTAL_HEIGHT = CARRIAGE_PAD_EXPOSED + CARRIAGE_PAD_EMBED
CARRIAGE_DECK_THICKNESS = 0.014
CARRIAGE_DECK_BOTTOM_Z = CARRIAGE_PAD_EXPOSED
CARRIAGE_DECK_TOP_Z = CARRIAGE_DECK_BOTTOM_Z + CARRIAGE_DECK_THICKNESS
CARRIAGE_MOUNT_LENGTH = 0.09
CARRIAGE_MOUNT_WIDTH = 0.10
CARRIAGE_MOUNT_CENTER_X = 0.04
CARRIAGE_MOUNT_EXTRA_HEIGHT = 0.012
CARRIAGE_MOUNT_TOP_Z = CARRIAGE_DECK_TOP_Z + CARRIAGE_MOUNT_EXTRA_HEIGHT

HOUSING_FOOT_LENGTH = 0.08
HOUSING_FOOT_WIDTH = 0.10
HOUSING_FOOT_THICKNESS = 0.009
HOUSING_BODY_LENGTH = 0.085
HOUSING_BODY_WIDTH = 0.09
HOUSING_BODY_HEIGHT = 0.045
HOUSING_BODY_CENTER_X = 0.0025
HOUSING_AXIS_Z = 0.031
HOUSING_FRONT_X = 0.052

SLIDE_TRAVEL = 0.34


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.42, 0.07)
        .cutBlind(-0.004)
    )

    end_cheek = lambda x: (
        cq.Workplane("XY")
        .box(0.045, BASE_WIDTH * 0.78, 0.014, centered=(True, True, False))
        .translate((x, 0.0, BASE_THICKNESS))
    )
    return plate.union(end_cheek(-(RAIL_LENGTH * 0.5 - 0.0225))).union(
        end_cheek(RAIL_LENGTH * 0.5 - 0.0225)
    )


def _carriage_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_WIDTH,
            CARRIAGE_DECK_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_DECK_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.004)
    )
    deck = (
        deck.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.105, 0.082)
        .cutBlind(-0.006)
    )
    mount_riser = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_MOUNT_LENGTH,
            CARRIAGE_MOUNT_WIDTH,
            CARRIAGE_MOUNT_EXTRA_HEIGHT,
            centered=(True, True, False),
        )
        .translate((CARRIAGE_MOUNT_CENTER_X, 0.0, CARRIAGE_DECK_TOP_Z))
        .edges("|Z")
        .fillet(0.003)
    )
    rear_web = (
        cq.Workplane("XY")
        .box(0.055, 0.09, 0.008, centered=(True, True, False))
        .translate((-0.045, 0.0, CARRIAGE_DECK_TOP_Z - 0.002))
    )
    return deck.union(mount_riser).union(rear_web)


def _housing_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            HOUSING_BODY_LENGTH,
            HOUSING_BODY_WIDTH,
            HOUSING_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((HOUSING_BODY_CENTER_X, 0.0, HOUSING_FOOT_THICKNESS - 0.001))
        .edges("|Z")
        .fillet(0.008)
    )
    body = body.edges(">Z").fillet(0.006)
    front_bearing_boss = (
        cq.Workplane("YZ")
        .circle(0.026)
        .extrude(0.007)
        .translate((0.045, 0.0, HOUSING_AXIS_Z))
    )
    rear_cap = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.01)
        .translate((-0.042, 0.0, HOUSING_AXIS_Z))
    )
    return body.union(front_bearing_boss).union(rear_cap)


def _spindle_shape() -> cq.Workplane:
    rear_collar = cq.Workplane("YZ").circle(0.022).extrude(0.018)
    shaft = cq.Workplane("YZ").circle(0.012).extrude(0.04).translate((0.018, 0.0, 0.0))
    nose = cq.Workplane("YZ").circle(0.016).extrude(0.016).translate((0.058, 0.0, 0.0))
    tip = cq.Workplane("YZ").circle(0.010).extrude(0.012).translate((0.074, 0.0, 0.0))
    spindle = rear_collar.union(shaft).union(nose).union(tip)
    return spindle.edges(">X").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_spindle_shuttle")

    base_mat = model.material("base_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    rail_mat = model.material("rail_finish", rgba=(0.72, 0.74, 0.78, 1.0))
    carriage_mat = model.material("carriage_finish", rgba=(0.54, 0.57, 0.60, 1.0))
    housing_mat = model.material("housing_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    spindle_mat = model.material("spindle_finish", rgba=(0.83, 0.84, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_body"),
        material=base_mat,
        name="base_body",
    )
    for side, rail_y in (("left", RAIL_PITCH * 0.5), ("right", -RAIL_PITCH * 0.5)):
        base.visual(
            Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_TOTAL_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    rail_y,
                    BASE_THICKNESS - RAIL_EMBED + 0.5 * RAIL_TOTAL_HEIGHT,
                )
            ),
            material=rail_mat,
            name=f"{side}_rail",
        )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material=carriage_mat,
        name="carriage_body",
    )
    for side, pad_y in (("left", RAIL_PITCH * 0.5), ("right", -RAIL_PITCH * 0.5)):
        carriage.visual(
            Box((CARRIAGE_PAD_LENGTH, CARRIAGE_PAD_WIDTH, CARRIAGE_PAD_TOTAL_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    pad_y,
                    0.5 * CARRIAGE_PAD_TOTAL_HEIGHT,
                )
            ),
            material=rail_mat,
            name=f"{side}_pad",
        )
    carriage.visual(
        Box(
            (
                CARRIAGE_MOUNT_LENGTH,
                CARRIAGE_MOUNT_WIDTH,
                CARRIAGE_MOUNT_EXTRA_HEIGHT + 0.001,
            )
        ),
        origin=Origin(
            xyz=(
                CARRIAGE_MOUNT_CENTER_X,
                0.0,
                CARRIAGE_DECK_TOP_Z + 0.5 * CARRIAGE_MOUNT_EXTRA_HEIGHT,
            )
        ),
        material=carriage_mat,
        name="mount_pad",
    )

    spindle_housing = model.part("spindle_housing")
    spindle_housing.visual(
        mesh_from_cadquery(_housing_shape(), "spindle_housing_body"),
        material=housing_mat,
        name="housing_body",
    )
    spindle_housing.visual(
        Box((HOUSING_FOOT_LENGTH, HOUSING_FOOT_WIDTH, HOUSING_FOOT_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5 * HOUSING_FOOT_THICKNESS,
            )
        ),
        material=housing_mat,
        name="mount_foot",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_spindle_shape(), "spindle_body"),
        material=spindle_mat,
        name="spindle_body",
    )
    spindle.visual(
        Box((0.016, 0.008, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.027)),
        material=spindle_mat,
        name="index_lug",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=-0.5 * SLIDE_TRAVEL,
            upper=0.5 * SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_spindle_housing",
        ArticulationType.FIXED,
        parent=carriage,
        child=spindle_housing,
        origin=Origin(
            xyz=(
                CARRIAGE_MOUNT_CENTER_X,
                0.0,
                CARRIAGE_MOUNT_TOP_Z,
            )
        ),
    )
    model.articulation(
        "housing_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=spindle_housing,
        child=spindle,
        origin=Origin(xyz=(HOUSING_FRONT_X, 0.0, HOUSING_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    spindle_housing = object_model.get_part("spindle_housing")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("base_to_carriage")
    spin = object_model.get_articulation("housing_to_spindle")

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
        "slide_axis_is_positive_x",
        slide.axis == (1.0, 0.0, 0.0),
        details=f"expected slide axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "spin_axis_is_positive_x",
        spin.axis == (1.0, 0.0, 0.0),
        details=f"expected spindle axis (1, 0, 0), got {spin.axis}",
    )

    with ctx.pose({slide: 0.0, spin: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="left_pad",
            elem_b="left_rail",
            contact_tol=1e-4,
            name="left_pad_seats_on_left_rail",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a="right_pad",
            elem_b="right_rail",
            contact_tol=1e-4,
            name="right_pad_seats_on_right_rail",
        )
        ctx.expect_contact(
            spindle_housing,
            carriage,
            elem_a="mount_foot",
            elem_b="mount_pad",
            contact_tol=1e-4,
            name="housing_mounts_on_carriage_pad",
        )
        ctx.expect_contact(
            spindle,
            spindle_housing,
            contact_tol=1e-4,
            name="spindle_seats_at_front_bearing_face",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.12,
            elem_a="left_pad",
            elem_b="left_rail",
            name="carriage_retains_longitudinal_rail_support_at_center",
        )

    with ctx.pose({slide: slide.motion_limits.lower, spin: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="left_pad",
            elem_b="left_rail",
            contact_tol=1e-4,
            name="left_pad_contacts_left_rail_at_negative_limit",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="left_pad",
            elem_b="left_rail",
            name="left_pad_still_overlaps_left_rail_at_negative_limit",
        )

    with ctx.pose({slide: slide.motion_limits.upper, spin: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="right_pad",
            elem_b="right_rail",
            contact_tol=1e-4,
            name="right_pad_contacts_right_rail_at_positive_limit",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="right_pad",
            elem_b="right_rail",
            name="right_pad_still_overlaps_right_rail_at_positive_limit",
        )

    with ctx.pose({slide: slide.motion_limits.lower, spin: 0.0}):
        carriage_low = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper, spin: 0.0}):
        carriage_high = ctx.part_world_position(carriage)
    slide_ok = (
        carriage_low is not None
        and carriage_high is not None
        and carriage_high[0] - carriage_low[0] > 0.30
        and abs(carriage_high[1] - carriage_low[1]) < 1e-6
        and abs(carriage_high[2] - carriage_low[2]) < 1e-6
    )
    ctx.check(
        "carriage_translates_cleanly_along_x",
        slide_ok,
        details=f"lower pose={carriage_low}, upper pose={carriage_high}",
    )

    with ctx.pose({spin: 0.0}):
        lug_0 = ctx.part_element_world_aabb(spindle, elem="index_lug")
    with ctx.pose({spin: math.pi / 2.0}):
        lug_90 = ctx.part_element_world_aabb(spindle, elem="index_lug")

    def _center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    lug_0_center = _center(lug_0)
    lug_90_center = _center(lug_90)
    spin_ok = (
        lug_0_center is not None
        and lug_90_center is not None
        and abs(lug_0_center[0] - lug_90_center[0]) < 1e-6
        and abs(lug_0_center[1] - lug_90_center[1]) > 0.015
        and abs(lug_0_center[2] - lug_90_center[2]) > 0.015
    )
    ctx.check(
        "spindle_rotation_moves_index_lug_around_axis",
        spin_ok,
        details=f"q0={lug_0_center}, q90={lug_90_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
