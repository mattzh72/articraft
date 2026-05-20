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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_OUTER = 0.160
FRAME_DEPTH = 0.082
FRAME_BAR_THICKNESS = 0.014
FRAME_BAR_CENTER = FRAME_OUTER / 2.0 - FRAME_BAR_THICKNESS / 2.0
SIDE_PLATE_THICKNESS = 0.004
SIDE_PLATE_CENTER_X = FRAME_DEPTH / 2.0 - SIDE_PLATE_THICKNESS / 2.0
SIDE_PLATE_OUTER = 0.152
SIDE_PLATE_WINDOW_RADIUS = 0.058

ROLL_HUB_RADIUS = 0.018
ROLL_HUB_LENGTH = 0.012
ROLL_HUB_CENTER_X = FRAME_DEPTH / 2.0 + ROLL_HUB_LENGTH / 2.0 - 0.001

STATOR_POD_RADIUS = 0.018
STATOR_POD_LENGTH = 0.012
STATOR_POD_CENTER_X = ROLL_HUB_CENTER_X + (ROLL_HUB_LENGTH + STATOR_POD_LENGTH) / 2.0
STATOR_POST_CENTER_Z = 0.070
STATOR_POST_HEIGHT = 0.104
TOP_BRIDGE_Z = 0.128

PITCH_PAD_RADIUS = 0.013
PITCH_PAD_LENGTH = 0.010
PITCH_PAD_CENTER_Y = 0.061

BARREL_LENGTH = 0.030
BARREL_END_RADIUS = 0.038
BARREL_MID_RADIUS = 0.045
BARREL_INNER_RADIUS = 0.032

PITCH_HUB_RADIUS = 0.013
PITCH_HUB_LENGTH = 0.011004
PITCH_HUB_CENTER_Y = PITCH_PAD_CENTER_Y - (PITCH_PAD_LENGTH + PITCH_HUB_LENGTH) / 2.0


def make_cheek_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=-SIDE_PLATE_THICKNESS / 2.0)
        .rect(SIDE_PLATE_OUTER, SIDE_PLATE_OUTER)
        .extrude(SIDE_PLATE_THICKNESS)
    )
    opening = (
        cq.Workplane("YZ")
        .workplane(offset=-(SIDE_PLATE_THICKNESS + 0.002) / 2.0)
        .circle(SIDE_PLATE_WINDOW_RADIUS)
        .extrude(SIDE_PLATE_THICKNESS + 0.002)
    )
    return plate.cut(opening).edges("|X").fillet(0.004)


def make_barrel_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=-BARREL_LENGTH / 2.0)
        .circle(BARREL_END_RADIUS)
        .workplane(offset=BARREL_LENGTH / 2.0)
        .circle(BARREL_MID_RADIUS)
        .workplane(offset=BARREL_LENGTH / 2.0)
        .circle(BARREL_END_RADIUS)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=-(BARREL_LENGTH + 0.010) / 2.0)
        .circle(BARREL_INNER_RADIUS)
        .extrude(BARREL_LENGTH + 0.010)
    )
    return outer.cut(inner)


def make_mount_flange() -> cq.Workplane:
    plate_thickness = 0.004
    plate_size = 0.032
    hole_spacing = 0.020
    hole_diameter = 0.0035
    rib_reach = 0.066
    rib_width = 0.006

    plate = (
        cq.Workplane("YZ")
        .workplane(offset=-plate_thickness / 2.0)
        .rect(plate_size, plate_size)
        .extrude(plate_thickness)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_spacing / 2.0, -hole_spacing / 2.0),
                (-hole_spacing / 2.0, hole_spacing / 2.0),
                (hole_spacing / 2.0, -hole_spacing / 2.0),
                (hole_spacing / 2.0, hole_spacing / 2.0),
            ]
        )
        .hole(hole_diameter)
    )

    rib_y = cq.Workplane("XY").box(plate_thickness, rib_reach, rib_width).translate(
        (0.0, 0.0, 0.0)
    )
    rib_z = cq.Workplane("XY").box(plate_thickness, rib_width, rib_reach).translate(
        (0.0, 0.0, 0.0)
    )

    return plate.union(rib_y).union(rib_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gimbal_cube")

    stator_black = model.material("stator_black", rgba=(0.11, 0.12, 0.13, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    cradle_silver = model.material("cradle_silver", rgba=(0.75, 0.78, 0.81, 1.0))
    flange_silver = model.material("flange_silver", rgba=(0.84, 0.86, 0.88, 1.0))

    stator = model.part("stator")
    stator.visual(
        Cylinder(radius=STATOR_POD_RADIUS, length=STATOR_POD_LENGTH),
        origin=Origin(
            xyz=(-STATOR_POD_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=stator_black,
        name="left_pod",
    )
    stator.visual(
        Cylinder(radius=STATOR_POD_RADIUS, length=STATOR_POD_LENGTH),
        origin=Origin(
            xyz=(STATOR_POD_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=stator_black,
        name="right_pod",
    )
    stator.visual(
        Box((0.012, 0.016, STATOR_POST_HEIGHT)),
        origin=Origin(xyz=(-STATOR_POD_CENTER_X, 0.0, STATOR_POST_CENTER_Z)),
        material=stator_black,
        name="left_riser",
    )
    stator.visual(
        Box((0.012, 0.016, STATOR_POST_HEIGHT)),
        origin=Origin(xyz=(STATOR_POD_CENTER_X, 0.0, STATOR_POST_CENTER_Z)),
        material=stator_black,
        name="right_riser",
    )
    stator.visual(
        Box((2.0 * STATOR_POD_CENTER_X + 0.024, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, TOP_BRIDGE_Z)),
        material=stator_black,
        name="top_bridge",
    )
    stator.inertial = Inertial.from_geometry(
        Box((2.0 * STATOR_POD_CENTER_X + 0.024, 0.050, 0.140)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    frame = model.part("outer_frame")
    frame.visual(
        mesh_from_cadquery(make_cheek_plate(), "left_cheek"),
        origin=Origin(xyz=(-SIDE_PLATE_CENTER_X, 0.0, 0.0)),
        material=frame_gray,
        name="left_cheek",
    )
    frame.visual(
        mesh_from_cadquery(make_cheek_plate(), "right_cheek"),
        origin=Origin(xyz=(SIDE_PLATE_CENTER_X, 0.0, 0.0)),
        material=frame_gray,
        name="right_cheek",
    )
    frame.visual(
        Box((FRAME_DEPTH - 2.0 * SIDE_PLATE_THICKNESS, SIDE_PLATE_OUTER, FRAME_BAR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_BAR_CENTER)),
        material=frame_gray,
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_DEPTH - 2.0 * SIDE_PLATE_THICKNESS, SIDE_PLATE_OUTER, FRAME_BAR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_BAR_CENTER)),
        material=frame_gray,
        name="bottom_rail",
    )
    frame.visual(
        Box((FRAME_DEPTH - 2.0 * SIDE_PLATE_THICKNESS, FRAME_BAR_THICKNESS, SIDE_PLATE_OUTER)),
        origin=Origin(xyz=(0.0, FRAME_BAR_CENTER, 0.0)),
        material=frame_gray,
        name="front_rail",
    )
    frame.visual(
        Box((FRAME_DEPTH - 2.0 * SIDE_PLATE_THICKNESS, FRAME_BAR_THICKNESS, SIDE_PLATE_OUTER)),
        origin=Origin(xyz=(0.0, -FRAME_BAR_CENTER, 0.0)),
        material=frame_gray,
        name="rear_rail",
    )
    frame.visual(
        Box((0.010, 0.012, 0.132)),
        origin=Origin(xyz=(-0.041, 0.0, 0.0)),
        material=frame_gray,
        name="left_hub_web",
    )
    frame.visual(
        Box((0.010, 0.012, 0.132)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=frame_gray,
        name="right_hub_web",
    )
    frame.visual(
        Cylinder(radius=ROLL_HUB_RADIUS, length=ROLL_HUB_LENGTH),
        origin=Origin(xyz=(-ROLL_HUB_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="roll_hub_left",
    )
    frame.visual(
        Cylinder(radius=ROLL_HUB_RADIUS, length=ROLL_HUB_LENGTH),
        origin=Origin(xyz=(ROLL_HUB_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="roll_hub_right",
    )
    frame.visual(
        Cylinder(radius=PITCH_PAD_RADIUS, length=PITCH_PAD_LENGTH),
        origin=Origin(
            xyz=(0.0, PITCH_PAD_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=frame_gray,
        name="pitch_pad_front",
    )
    frame.visual(
        Cylinder(radius=PITCH_PAD_RADIUS, length=PITCH_PAD_LENGTH),
        origin=Origin(
            xyz=(0.0, -PITCH_PAD_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=frame_gray,
        name="pitch_pad_rear",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_DEPTH, FRAME_OUTER, FRAME_OUTER)),
        mass=0.42,
        origin=Origin(),
    )

    cradle = model.part("inner_cradle")
    cradle.visual(
        mesh_from_cadquery(make_barrel_shell(), "barrel_shell"),
        material=cradle_silver,
        name="barrel_shell",
    )
    cradle.visual(
        mesh_from_cadquery(make_mount_flange(), "mount_flange"),
        material=flange_silver,
        name="mount_flange",
    )
    cradle.visual(
        Cylinder(radius=PITCH_HUB_RADIUS, length=PITCH_HUB_LENGTH),
        origin=Origin(xyz=(0.0, PITCH_HUB_CENTER_Y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cradle_silver,
        name="pitch_hub_front",
    )
    cradle.visual(
        Cylinder(radius=PITCH_HUB_RADIUS, length=PITCH_HUB_LENGTH),
        origin=Origin(xyz=(0.0, -PITCH_HUB_CENTER_Y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cradle_silver,
        name="pitch_hub_rear",
    )
    cradle.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_MID_RADIUS, length=BARREL_LENGTH),
        mass=0.32,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "stator_to_frame_roll",
        ArticulationType.REVOLUTE,
        parent=stator,
        child=frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "frame_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.32,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stator = object_model.get_part("stator")
    frame = object_model.get_part("outer_frame")
    cradle = object_model.get_part("inner_cradle")
    roll = object_model.get_articulation("stator_to_frame_roll")
    pitch = object_model.get_articulation("frame_to_cradle_pitch")

    left_pod = stator.get_visual("left_pod")
    right_pod = stator.get_visual("right_pod")
    top_rail = frame.get_visual("top_rail")
    front_rail = frame.get_visual("front_rail")
    roll_hub_left = frame.get_visual("roll_hub_left")
    roll_hub_right = frame.get_visual("roll_hub_right")
    pitch_pad_front = frame.get_visual("pitch_pad_front")
    pitch_pad_rear = frame.get_visual("pitch_pad_rear")
    barrel_shell = cradle.get_visual("barrel_shell")
    mount_flange = cradle.get_visual("mount_flange")
    pitch_hub_front = cradle.get_visual("pitch_hub_front")
    pitch_hub_rear = cradle.get_visual("pitch_hub_rear")

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
        "expected_structure_present",
        all(part is not None for part in (stator, frame, cradle))
        and all(joint is not None for joint in (roll, pitch)),
        "Expected stator, outer frame, inner cradle, and both joints.",
    )
    ctx.check(
        "joint_axes_and_limits",
        tuple(round(v, 3) for v in roll.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in pitch.axis) == (0.0, 1.0, 0.0)
        and roll.motion_limits is not None
        and pitch.motion_limits is not None
        and roll.motion_limits.lower is not None
        and roll.motion_limits.upper is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and roll.motion_limits.lower < 0.0 < roll.motion_limits.upper
        and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper,
        "Roll should run on +X and pitch on +Y with bidirectional motion limits.",
    )

    ctx.expect_contact(
        frame,
        stator,
        elem_a=roll_hub_left,
        elem_b=left_pod,
        name="left_roll_bearing_contact",
    )
    ctx.expect_contact(
        frame,
        stator,
        elem_a=roll_hub_right,
        elem_b=right_pod,
        name="right_roll_bearing_contact",
    )
    ctx.expect_contact(
        frame,
        cradle,
        elem_a=pitch_pad_front,
        elem_b=pitch_hub_front,
        name="front_pitch_bearing_contact",
    )
    ctx.expect_contact(
        frame,
        cradle,
        elem_a=pitch_pad_rear,
        elem_b=pitch_hub_rear,
        name="rear_pitch_bearing_contact",
    )

    ctx.expect_gap(
        frame,
        cradle,
        axis="z",
        positive_elem=top_rail,
        negative_elem=barrel_shell,
        min_gap=0.010,
        max_gap=0.022,
        name="top_barrel_clearance",
    )
    ctx.expect_gap(
        frame,
        cradle,
        axis="y",
        positive_elem=front_rail,
        negative_elem=barrel_shell,
        min_gap=0.010,
        max_gap=0.022,
        name="front_barrel_clearance",
    )
    ctx.expect_within(
        cradle,
        frame,
        axes="yz",
        inner_elem=barrel_shell,
        margin=0.0,
        name="barrel_within_frame_envelope",
    )
    ctx.expect_overlap(
        cradle,
        frame,
        axes="yz",
        elem_a=mount_flange,
        min_overlap=0.030,
        name="central_flange_reads_within_module_footprint",
    )

    with ctx.pose({roll: 0.85, pitch: 0.28}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_articulated_pose")
        ctx.expect_contact(
            frame,
            cradle,
            elem_a=pitch_pad_front,
            elem_b=pitch_hub_front,
            name="front_pitch_contact_in_pose",
        )
        ctx.expect_within(
            cradle,
            frame,
            axes="yz",
            inner_elem=barrel_shell,
            margin=0.0,
            name="barrel_within_frame_in_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
