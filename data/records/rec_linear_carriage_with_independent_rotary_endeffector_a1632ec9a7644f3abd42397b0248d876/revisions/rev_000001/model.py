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


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.180
PLATE_HEIGHT = 0.420
BASE_DEPTH = 0.075
BASE_WIDTH = 0.120
BASE_HEIGHT = 0.018
GUSSET_WIDTH = 0.060

GUIDE_DEPTH = 0.018
GUIDE_WIDTH = 0.028
GUIDE_LENGTH = 0.300
GUIDE_CENTER_X = (PLATE_THICKNESS / 2.0) + (GUIDE_DEPTH / 2.0)
GUIDE_CENTER_Z = 0.220

CARRIAGE_DEPTH = 0.054
CARRIAGE_WIDTH = 0.098
CARRIAGE_HEIGHT = 0.082
CARRIAGE_CENTER_X = 0.040
CARRIAGE_FRONT_X = CARRIAGE_CENTER_X + (CARRIAGE_DEPTH / 2.0)
CARRIAGE_SHOE_DEPTH = 0.012
CARRIAGE_SHOE_WIDTH = 0.042
CARRIAGE_SHOE_HEIGHT = 0.110
CARRIAGE_SHOE_CENTER_X = 0.015
CARRIAGE_PAD_RADIUS = 0.024
CARRIAGE_PAD_DEPTH = 0.012
CARRIAGE_HOME_Z = 0.140
SLIDE_TRAVEL = 0.160

FACE_RADIUS = 0.036
FACE_THICKNESS = 0.010
FACE_HUB_RADIUS = 0.014
FACE_HUB_DEPTH = 0.004
FACE_KEY_WIDTH = 0.010
FACE_KEY_HEIGHT = 0.024
FACE_KEY_DEPTH = 0.004


def _side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT).translate(
        (0.0, 0.0, PLATE_HEIGHT / 2.0)
    )
    foot = cq.Workplane("XY").box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT).translate(
        (-(BASE_DEPTH - PLATE_THICKNESS) / 2.0, 0.0, BASE_HEIGHT / 2.0)
    )
    brace = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-PLATE_THICKNESS / 2.0, BASE_HEIGHT),
                (-BASE_DEPTH * 0.86, BASE_HEIGHT),
                (-PLATE_THICKNESS / 2.0, 0.165),
            ]
        )
        .close()
        .extrude(GUSSET_WIDTH / 2.0, both=True)
    )
    return plate.union(foot).union(brace)


def _guide_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(GUIDE_DEPTH, GUIDE_WIDTH, GUIDE_LENGTH)
        .edges("|Z")
        .fillet(0.002)
    )


def _carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(CARRIAGE_DEPTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT).translate(
        (CARRIAGE_CENTER_X, 0.0, 0.0)
    )
    shoe = cq.Workplane("XY").box(
        CARRIAGE_SHOE_DEPTH,
        CARRIAGE_SHOE_WIDTH,
        CARRIAGE_SHOE_HEIGHT,
    ).translate((CARRIAGE_SHOE_CENTER_X, 0.0, 0.0))
    front_pad = (
        cq.Workplane("YZ")
        .circle(CARRIAGE_PAD_RADIUS)
        .extrude(CARRIAGE_PAD_DEPTH)
        .translate((CARRIAGE_FRONT_X - CARRIAGE_PAD_DEPTH, 0.0, 0.0))
    )
    return body.union(shoe).union(front_pad)


def _rotary_face_shape() -> cq.Workplane:
    disc = cq.Workplane("YZ").circle(FACE_RADIUS).extrude(FACE_THICKNESS)
    face_hub = (
        cq.Workplane("YZ")
        .circle(FACE_HUB_RADIUS)
        .extrude(FACE_HUB_DEPTH)
        .translate((FACE_THICKNESS - FACE_HUB_DEPTH, 0.0, 0.0))
    )
    disc = disc.union(face_hub)

    orientation_key = cq.Workplane("XY").box(FACE_KEY_DEPTH, FACE_KEY_WIDTH, FACE_KEY_HEIGHT).translate(
        (
            FACE_THICKNESS - (FACE_KEY_DEPTH / 2.0) - 0.001,
            FACE_RADIUS * 0.58,
            0.0,
        )
    )
    disc = disc.union(orientation_key)

    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_carriage_rotary_face")

    model.material("plate_gray", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("guide_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("face_aluminum", rgba=(0.83, 0.84, 0.86, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_shape(), "side_plate"),
        material="plate_gray",
        name="side_plate_body",
    )

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_guide_shape(), "guide"),
        material="guide_steel",
        name="guide_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material="carriage_gray",
        name="carriage_body",
    )

    rotary_face = model.part("rotary_face")
    rotary_face.visual(
        mesh_from_cadquery(_rotary_face_shape(), "rotary_face"),
        material="face_aluminum",
        name="rotary_face_body",
    )

    model.articulation(
        "plate_to_guide",
        ArticulationType.FIXED,
        parent=side_plate,
        child=guide,
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_CENTER_Z)),
    )
    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z - GUIDE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=250.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "face_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rotary_face,
        origin=Origin(xyz=(CARRIAGE_FRONT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=15.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    rotary_face = object_model.get_part("rotary_face")
    slide = object_model.get_articulation("guide_slide")
    spin = object_model.get_articulation("face_spin")

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
        "slide_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC, got {slide.articulation_type}",
    )
    ctx.check(
        "spin_joint_is_revolute",
        spin.articulation_type == ArticulationType.REVOLUTE,
        f"expected REVOLUTE, got {spin.articulation_type}",
    )
    ctx.check(
        "slide_axis_is_vertical",
        tuple(round(v, 6) for v in slide.axis) == (0.0, 0.0, 1.0),
        f"unexpected slide axis {slide.axis}",
    )
    ctx.check(
        "spin_axis_is_forward",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        f"unexpected spin axis {spin.axis}",
    )

    with ctx.pose({slide: 0.0, spin: 0.0}):
        ctx.expect_contact(guide, side_plate, name="guide_is_mounted_to_side_plate")
        ctx.expect_contact(carriage, guide, name="carriage_is_supported_by_guide_at_home")
        ctx.expect_contact(rotary_face, carriage, name="rotary_face_is_supported_by_carriage")
        ctx.expect_gap(
            carriage,
            side_plate,
            axis="x",
            min_gap=0.001,
            max_gap=0.020,
            name="carriage_runs_forward_of_plate",
        )
        ctx.expect_gap(
            rotary_face,
            side_plate,
            axis="x",
            min_gap=0.040,
            name="rotary_face_projects_forward_of_side_plate",
        )

    with ctx.pose({slide: SLIDE_TRAVEL, spin: 1.1}):
        ctx.expect_contact(carriage, guide, name="carriage_stays_supported_at_upper_travel")
        ctx.expect_contact(rotary_face, carriage, name="rotary_face_stays_supported_when_turned")

    with ctx.pose({slide: 0.0}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        high_pos = ctx.part_world_position(carriage)

    moves_up = (
        low_pos is not None
        and high_pos is not None
        and (high_pos[2] - low_pos[2]) > 0.14
        and abs(high_pos[0] - low_pos[0]) < 1e-6
        and abs(high_pos[1] - low_pos[1]) < 1e-6
    )
    ctx.check(
        "carriage_translates_straight_up_guide",
        moves_up,
        f"carriage positions were low={low_pos}, high={high_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
