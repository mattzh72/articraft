from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


HINGE_AXIS = (0.0, 1.0, 0.0)

BASE_INNER_GAP = 0.024
BASE_CHEEK_THICKNESS = 0.008
BASE_CHEEK_OFFSET_Y = BASE_INNER_GAP / 2.0 + BASE_CHEEK_THICKNESS / 2.0
SHOULDER_JOINT_X = 0.0
ELBOW_INNER_GAP = 0.020
ELBOW_FORK_THICKNESS = 0.007
ELBOW_FORK_OFFSET_Y = ELBOW_INNER_GAP / 2.0 + ELBOW_FORK_THICKNESS / 2.0
BASE_BACK_PLATE_THICKNESS = 0.012
BASE_BACK_PLATE_WIDTH = 0.090
BASE_BACK_PLATE_HEIGHT = 0.140
BASE_BACK_PLATE_CENTER_X = -0.032
BASE_CHEEK_LENGTH = 0.030
BASE_CHEEK_HEIGHT = 0.058

SHOULDER_BARREL_RADIUS = 0.0085
SHOULDER_BARREL_LENGTH = BASE_INNER_GAP
ELBOW_BARREL_RADIUS = 0.0080
ELBOW_BARREL_LENGTH = ELBOW_INNER_GAP

FIRST_LINK_LENGTH = 0.360
SECOND_LINK_LENGTH = 0.205
END_PAD_OFFSET = 0.195


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((cx, cy - length / 2.0, cz))


def beam_x(
    length: float,
    width: float,
    height: float,
    edge_radius: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    beam = cq.Workplane("XY").box(length, width, height)
    if edge_radius > 0.0:
        beam = beam.edges("|X").fillet(edge_radius)
    return beam.translate(center)


def slot_beam_x(
    length: float,
    height: float,
    width: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XZ").slot2D(length, height).extrude(width).translate((cx, cy - width / 2.0, cz))


def make_base_bracket_shape() -> cq.Workplane:
    back_plate = beam_x(
        BASE_BACK_PLATE_THICKNESS,
        BASE_BACK_PLATE_WIDTH,
        BASE_BACK_PLATE_HEIGHT,
        0.0025,
        (BASE_BACK_PLATE_CENTER_X, 0.0, 0.0),
    )
    left_cheek = beam_x(
        BASE_CHEEK_LENGTH,
        BASE_CHEEK_THICKNESS,
        BASE_CHEEK_HEIGHT,
        0.0020,
        (-0.015, BASE_CHEEK_OFFSET_Y, 0.0),
    )
    right_cheek = beam_x(
        BASE_CHEEK_LENGTH,
        BASE_CHEEK_THICKNESS,
        BASE_CHEEK_HEIGHT,
        0.0020,
        (-0.015, -BASE_CHEEK_OFFSET_Y, 0.0),
    )
    upper_bridge = beam_x(0.022, 0.042, 0.012, 0.0015, (-0.022, 0.0, 0.030))
    lower_bridge = beam_x(0.026, 0.042, 0.014, 0.0015, (-0.020, 0.0, -0.032))
    bracket = (
        back_plate.union(left_cheek)
        .union(right_cheek)
        .union(upper_bridge)
        .union(lower_bridge)
    )

    shoulder_bore = cylinder_y(0.0054, 0.060, (SHOULDER_JOINT_X, 0.0, 0.0))
    upper_mount_hole = cylinder_x(0.0045, 0.020, (BASE_BACK_PLATE_CENTER_X, 0.0, 0.036))
    lower_mount_hole = cylinder_x(0.0045, 0.020, (BASE_BACK_PLATE_CENTER_X, 0.0, -0.036))

    return bracket.cut(shoulder_bore).cut(upper_mount_hole).cut(lower_mount_hole)


def make_first_link_shape() -> cq.Workplane:
    shoulder_barrel = cylinder_y(SHOULDER_BARREL_RADIUS, SHOULDER_BARREL_LENGTH, (0.0, 0.0, 0.0))
    shoulder_neck = slot_beam_x(0.018, 0.014, 0.012, (0.024, 0.0, 0.0))
    main_body = slot_beam_x(0.214, 0.017, 0.012, (0.160, 0.0, 0.0))
    left_branch = beam_x(0.074, 0.006, 0.012, 0.0010, (0.300, 0.008, 0.0))
    right_branch = beam_x(0.074, 0.006, 0.012, 0.0010, (0.300, -0.008, 0.0))
    left_fork = beam_x(0.020, ELBOW_FORK_THICKNESS, 0.028, 0.0012, (0.344, ELBOW_FORK_OFFSET_Y, 0.0))
    right_fork = beam_x(0.020, ELBOW_FORK_THICKNESS, 0.028, 0.0012, (0.344, -ELBOW_FORK_OFFSET_Y, 0.0))
    link = (
        shoulder_barrel.union(shoulder_neck)
        .union(main_body)
        .union(left_branch)
        .union(right_branch)
        .union(left_fork)
        .union(right_fork)
    )

    shoulder_bore = cylinder_y(0.0054, 0.060, (0.0, 0.0, 0.0))
    elbow_bore = cylinder_y(0.0054, 0.060, (FIRST_LINK_LENGTH, 0.0, 0.0))

    return link.cut(shoulder_bore).cut(elbow_bore)


def make_second_link_shape() -> cq.Workplane:
    elbow_barrel = cylinder_y(ELBOW_BARREL_RADIUS, ELBOW_BARREL_LENGTH, (0.0, 0.0, 0.0))
    elbow_neck = slot_beam_x(0.016, 0.011, 0.010, (0.022, 0.0, 0.0))
    main_body = slot_beam_x(0.112, 0.014, 0.010, (0.098, 0.0, 0.0))
    distal_transition = slot_beam_x(0.032, 0.012, 0.010, (0.164, 0.0, 0.0))
    tip_lug = beam_x(0.028, 0.012, 0.010, 0.0010, (0.191, 0.0, 0.0))

    link = (
        elbow_barrel.union(elbow_neck)
        .union(main_body)
        .union(distal_transition)
        .union(tip_lug)
    )

    elbow_bore = cylinder_y(0.0053, 0.060, (0.0, 0.0, 0.0))

    return link.cut(elbow_bore)


def make_end_pad_metal_shape() -> cq.Workplane:
    stem = beam_x(0.010, 0.012, 0.008, 0.0010, (0.005, 0.0, 0.0))
    neck = beam_x(0.012, 0.012, 0.008, 0.0010, (0.016, 0.0, 0.0))
    plate = slot_beam_x(0.028, 0.010, 0.020, (0.024, 0.0, 0.0))
    return stem.union(neck).union(plate)


def make_end_pad_rubber_shape() -> cq.Workplane:
    return slot_beam_x(0.024, 0.004, 0.018, (0.024, 0.0, 0.007))


def make_shoulder_bolt_shape() -> cq.Workplane:
    shaft = cylinder_y(0.0051, BASE_INNER_GAP + 2.0 * BASE_CHEEK_THICKNESS, (0.0, 0.0, 0.0))
    left_head = cylinder_y(0.0072, 0.004, (0.0, BASE_CHEEK_OFFSET_Y + 0.006, 0.0))
    right_head = cylinder_y(0.0072, 0.004, (0.0, -BASE_CHEEK_OFFSET_Y - 0.006, 0.0))
    return shaft.union(left_head).union(right_head)


def make_elbow_bolt_shape() -> cq.Workplane:
    shaft = cylinder_y(0.0051, ELBOW_INNER_GAP + 2.0 * ELBOW_FORK_THICKNESS, (0.0, 0.0, 0.0))
    left_head = cylinder_y(0.0068, 0.004, (0.0, ELBOW_FORK_OFFSET_Y + 0.0055, 0.0))
    right_head = cylinder_y(0.0068, 0.004, (0.0, -ELBOW_FORK_OFFSET_Y - 0.0055, 0.0))
    return shaft.union(left_head).union(right_head)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_service_arm")

    bracket_steel = model.material("bracket_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    link_aluminum = model.material("link_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    pad_black = model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        mesh_from_cadquery(make_base_bracket_shape(), "base_bracket"),
        material=bracket_steel,
        name="base_bracket_shell",
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(make_first_link_shape(), "first_link"),
        material=link_aluminum,
        name="first_link_shell",
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(make_second_link_shape(), "second_link"),
        material=link_aluminum,
        name="second_link_shell",
    )

    end_pad = model.part("end_pad")
    end_pad.visual(
        mesh_from_cadquery(make_end_pad_metal_shape(), "end_pad_metal"),
        material=link_aluminum,
        name="pad_metal",
    )
    end_pad.visual(
        mesh_from_cadquery(make_end_pad_rubber_shape(), "end_pad_rubber"),
        material=pad_black,
        name="pad_rubber",
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=first_link,
        origin=Origin(),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=-0.75,
            upper=0.95,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=second_link,
        child=end_pad,
        origin=Origin(xyz=(END_PAD_OFFSET, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    end_pad = object_model.get_part("end_pad")
    shoulder = object_model.get_articulation("shoulder_hinge")
    elbow = object_model.get_articulation("elbow_hinge")

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
    ctx.allow_overlap(
        base_bracket,
        first_link,
        reason="Shoulder hinge uses a nested journal/barrel representation inside the bracket cheek envelope.",
    )
    ctx.allow_overlap(
        first_link,
        second_link,
        reason="Elbow hinge uses a nested fork-and-barrel representation around the shoulder bolt axis.",
    )
    ctx.allow_overlap(
        second_link,
        end_pad,
        reason="End pad stem is inserted into the distal lug as a bonded mount.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        all(part is not None for part in (base_bracket, first_link, second_link, end_pad)),
        details="Expected base bracket, both links, and the end pad.",
    )
    ctx.check(
        "hinges_are_parallel_revolutes",
        shoulder.joint_type == ArticulationType.REVOLUTE
        and elbow.joint_type == ArticulationType.REVOLUTE
        and tuple(shoulder.axis) == HINGE_AXIS
        and tuple(elbow.axis) == HINGE_AXIS,
        details="Shoulder and elbow should both be revolute joints about the same hinge axis.",
    )
    ctx.check(
        "first_link_longer_than_second",
        FIRST_LINK_LENGTH > SECOND_LINK_LENGTH * 1.5,
        details="The first reach link should read as the dominant long member.",
    )

    ctx.expect_contact(
        base_bracket,
        first_link,
        contact_tol=0.0025,
        name="shoulder_joint_stack_is_tightly_seated",
    )
    ctx.expect_contact(
        first_link,
        second_link,
        contact_tol=0.0025,
        name="elbow_joint_stack_is_tightly_seated",
    )
    ctx.expect_contact(
        second_link,
        end_pad,
        contact_tol=0.0015,
        name="end_pad_mount_is_tightly_seated",
    )

    ctx.expect_origin_gap(
        second_link,
        first_link,
        axis="x",
        min_gap=FIRST_LINK_LENGTH - 1e-6,
        max_gap=FIRST_LINK_LENGTH + 1e-6,
        name="elbow_origin_sits_at_first_link_tip",
    )
    ctx.expect_origin_gap(
        end_pad,
        second_link,
        axis="x",
        min_gap=END_PAD_OFFSET - 1e-6,
        max_gap=END_PAD_OFFSET + 1e-6,
        name="pad_origin_sits_at_second_link_tip",
    )

    with ctx.pose({shoulder: shoulder.motion_limits.lower, elbow: 0.25}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lowered_reach")
    with ctx.pose({shoulder: 0.30, elbow: elbow.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_tight_elbow_fold")
    with ctx.pose({shoulder: shoulder.motion_limits.upper, elbow: elbow.motion_limits.upper * 0.75}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_raised_compact_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
