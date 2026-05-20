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


PLATE_WIDTH = 0.066
PLATE_HEIGHT = 0.112
PLATE_THICKNESS = 0.008
PLATE_FRONT_OFFSET = 0.034
MOUNT_HOLE_DIAMETER = 0.007

Z_JOINT_LUG_LENGTH = 0.014
Z_JOINT_CLEVIS_LENGTH = 0.022
Z_JOINT_CLEVIS_BRIDGE = 0.008
Z_JOINT_SLOT_HEIGHT = 0.016
Z_JOINT_OUTER_HEIGHT = 0.042
Z_JOINT_CHEEK_HEIGHT = (Z_JOINT_OUTER_HEIGHT - Z_JOINT_SLOT_HEIGHT) / 2.0
Z_JOINT_CHEEK_Z = Z_JOINT_SLOT_HEIGHT / 2.0 + Z_JOINT_CHEEK_HEIGHT / 2.0

FIRST_LINK_LENGTH = 0.172
SECOND_LINK_LENGTH = 0.142
SWIVEL_TO_TILT = 0.042

TILT_LUG_LENGTH = 0.014
TILT_LUG_WIDTH = 0.013
TILT_CLEVIS_LENGTH = 0.018
TILT_CLEVIS_BRIDGE = 0.008
TILT_SLOT_WIDTH = 0.015
TILT_OUTER_WIDTH = 0.036
TILT_HEIGHT = 0.030
TILT_CHEEK_WIDTH = (TILT_OUTER_WIDTH - TILT_SLOT_WIDTH) / 2.0
TILT_CHEEK_Y = TILT_SLOT_WIDTH / 2.0 + TILT_CHEEK_WIDTH / 2.0


def _x_box(length: float, width: float, height: float, x_min: float, *, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x_min, y, z))
    )


def _x_loft(
    x_start: float,
    x_end: float,
    start_width: float,
    start_height: float,
    end_width: float,
    end_height: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_start)
        .rect(start_width, start_height)
        .workplane(offset=x_end - x_start)
        .rect(end_width, end_height)
        .loft(combine=True)
    )


def _z_lug(width: float, height: float) -> cq.Workplane:
    top = _x_box(Z_JOINT_LUG_LENGTH, width, Z_JOINT_CHEEK_HEIGHT, 0.0, z=Z_JOINT_CHEEK_Z)
    bottom = _x_box(Z_JOINT_LUG_LENGTH, width, Z_JOINT_CHEEK_HEIGHT, 0.0, z=-Z_JOINT_CHEEK_Z)
    web = _x_box(Z_JOINT_LUG_LENGTH, width * 0.52, min(height, Z_JOINT_SLOT_HEIGHT * 0.82), 0.0)
    return top.union(bottom).union(web)


def _z_clevis(width: float, overall_height: float) -> cq.Workplane:
    block = _x_box(Z_JOINT_CLEVIS_LENGTH, width, overall_height, -Z_JOINT_CLEVIS_LENGTH)
    slot = _x_box(
        Z_JOINT_CLEVIS_LENGTH - Z_JOINT_CLEVIS_BRIDGE + 0.002,
        width + 0.004,
        Z_JOINT_SLOT_HEIGHT,
        -(Z_JOINT_CLEVIS_LENGTH - Z_JOINT_CLEVIS_BRIDGE) - 0.002,
    )
    return block.cut(slot)


def _tilt_lug(height: float) -> cq.Workplane:
    left = _x_box(TILT_LUG_LENGTH, TILT_CHEEK_WIDTH, height, 0.0, y=-TILT_CHEEK_Y)
    right = _x_box(TILT_LUG_LENGTH, TILT_CHEEK_WIDTH, height, 0.0, y=TILT_CHEEK_Y)
    web = _x_box(TILT_LUG_LENGTH, TILT_SLOT_WIDTH * 0.72, height * 0.72, 0.0)
    return left.union(right).union(web)


def _tilt_clevis(height: float) -> cq.Workplane:
    block = _x_box(TILT_CLEVIS_LENGTH, TILT_OUTER_WIDTH, height, -TILT_CLEVIS_LENGTH)
    slot = _x_box(
        TILT_CLEVIS_LENGTH - TILT_CLEVIS_BRIDGE + 0.002,
        TILT_SLOT_WIDTH,
        height + 0.004,
        -(TILT_CLEVIS_LENGTH - TILT_CLEVIS_BRIDGE) - 0.002,
    )
    return block.cut(slot)


def _wall_plate_shape() -> cq.Workplane:
    plate_x = -(PLATE_FRONT_OFFSET + PLATE_THICKNESS)
    plate = (
        cq.Workplane("YZ")
        .rect(PLATE_WIDTH, PLATE_HEIGHT)
        .extrude(PLATE_THICKNESS)
        .translate((plate_x, 0.0, 0.0))
    )
    plate_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (0.0, 0.031),
                (0.0, -0.031),
            ]
        )
        .circle(MOUNT_HOLE_DIAMETER / 2.0)
        .extrude(PLATE_THICKNESS + 0.004)
        .translate((plate_x - 0.002, 0.0, 0.0))
    )
    plate = plate.cut(plate_holes).edges("|X").fillet(0.006)

    support_block = _x_box(0.014, 0.052, 0.070, -0.036)
    support_taper = _x_loft(-0.030, -0.010, 0.046, 0.060, 0.034, 0.034)
    shoulder = _z_clevis(0.034, Z_JOINT_OUTER_HEIGHT)
    return plate.union(support_block).union(support_taper).union(shoulder)


def _arm_link_shape(
    *,
    length: float,
    body_width_start: float,
    body_width_end: float,
    body_thickness_start: float,
    body_thickness_end: float,
    name_seed: str,
) -> cq.Workplane:
    rear_lug = _z_lug(max(body_width_start * 0.56, 0.024), 0.013)
    body = _x_loft(
        0.010,
        length - Z_JOINT_CLEVIS_LENGTH,
        body_width_start,
        body_thickness_start,
        body_width_end,
        body_thickness_end,
    )
    front_clevis = _z_clevis(max(body_width_end, 0.028), Z_JOINT_OUTER_HEIGHT)
    front_clevis = front_clevis.translate((length, 0.0, 0.0))
    stiffener = _x_loft(
        0.040,
        length * 0.72,
        body_width_start * 0.68,
        body_thickness_start * 0.55,
        body_width_end * 0.72,
        body_thickness_end * 0.50,
    ).translate((0.0, 0.0, -0.003))

    shape = body.union(rear_lug).union(front_clevis)
    if "first" in name_seed:
        shape = shape.union(stiffener)
    return shape


def _head_swivel_shape() -> cq.Workplane:
    rear_lug = _z_lug(0.024, 0.013)
    neck = _x_loft(
        0.010,
        SWIVEL_TO_TILT - TILT_CLEVIS_LENGTH,
        0.032,
        0.026,
        0.026,
        0.022,
    )
    yoke_body = _tilt_clevis(TILT_HEIGHT).translate((SWIVEL_TO_TILT, 0.0, 0.0))
    housing = _x_box(0.014, 0.028, 0.026, SWIVEL_TO_TILT - 0.024)
    return rear_lug.union(neck).union(housing).union(yoke_body)


def _head_frame_shape() -> cq.Workplane:
    frame_outer_w = 0.108
    frame_outer_h = 0.084
    frame_inner_w = 0.072
    frame_inner_h = 0.048
    frame_plate_t = 0.006
    frame_plate_x = 0.032
    vesa_w = 0.075
    vesa_h = 0.075

    trunnion = _tilt_lug(0.022)
    neck = _x_loft(
        0.010,
        0.030,
        0.032,
        0.026,
        0.052,
        0.040,
    )
    frame = (
        cq.Workplane("YZ")
        .rect(frame_outer_w, frame_outer_h)
        .extrude(frame_plate_t)
        .translate((frame_plate_x, 0.0, 0.0))
    )
    frame_cutout = (
        cq.Workplane("YZ")
        .rect(frame_inner_w, frame_inner_h)
        .extrude(frame_plate_t + 0.004)
        .translate((frame_plate_x - 0.002, 0.0, 0.0))
    )
    vesa_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-vesa_w / 2.0, -vesa_h / 2.0),
                (-vesa_w / 2.0, vesa_h / 2.0),
                (vesa_w / 2.0, -vesa_h / 2.0),
                (vesa_w / 2.0, vesa_h / 2.0),
            ]
        )
        .circle(0.0032)
        .extrude(frame_plate_t + 0.004)
        .translate((frame_plate_x - 0.002, 0.0, 0.0))
    )
    frame = frame.cut(frame_cutout).cut(vesa_holes)
    frame_boss = _x_box(0.012, 0.060, 0.052, 0.024)
    return trunnion.union(neck).union(frame_boss).union(frame)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_display_bracket")

    textured_black = model.material("textured_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.24, 0.25, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material=dark_steel,
        name="wall_plate_shell",
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(
            _arm_link_shape(
                length=FIRST_LINK_LENGTH,
                body_width_start=0.050,
                body_width_end=0.038,
                body_thickness_start=0.020,
                body_thickness_end=0.018,
                name_seed="first_link",
            ),
            "first_link",
        ),
        material=satin_black,
        name="first_link_shell",
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(
            _arm_link_shape(
                length=SECOND_LINK_LENGTH,
                body_width_start=0.034,
                body_width_end=0.028,
                body_thickness_start=0.016,
                body_thickness_end=0.014,
                name_seed="second_link",
            ),
            "second_link",
        ),
        material=textured_black,
        name="second_link_shell",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        mesh_from_cadquery(_head_swivel_shape(), "head_swivel"),
        material=satin_black,
        name="head_swivel_shell",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(_head_frame_shape(), "head_frame"),
        material=dark_steel,
        name="head_frame_shell",
    )

    model.articulation(
        "wall_to_first",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-1.7, upper=1.7),
    )
    model.articulation(
        "second_to_swivel",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=head_swivel,
        origin=Origin(xyz=(SECOND_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.2, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=head_frame,
        origin=Origin(xyz=(SWIVEL_TO_TILT, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.22, upper=0.52),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    head_swivel = object_model.get_part("head_swivel")
    head_frame = object_model.get_part("head_frame")

    shoulder = object_model.get_articulation("wall_to_first")
    elbow = object_model.get_articulation("first_to_second")
    swivel = object_model.get_articulation("second_to_swivel")
    tilt = object_model.get_articulation("swivel_to_head")

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
        "all_joints_revolute",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder, elbow, swivel, tilt)
        ),
        "Expected four revolute articulations for the two arm joints plus head swivel and tilt.",
    )
    ctx.check(
        "joint_axes_match_bracket_mechanism",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0)
        and swivel.axis == (0.0, 0.0, 1.0)
        and tilt.axis == (0.0, 1.0, 0.0),
        "Arm joints and swivel should yaw about +Z while tilt should pitch about +Y.",
    )

    ctx.expect_contact(first_link, wall_plate, name="wall_plate_supports_first_link")
    ctx.expect_contact(second_link, first_link, name="first_link_supports_second_link")
    ctx.expect_contact(head_swivel, second_link, name="second_link_supports_swivel")
    ctx.expect_contact(head_frame, head_swivel, name="swivel_supports_head_frame")

    ctx.expect_gap(
        head_frame,
        wall_plate,
        axis="x",
        min_gap=0.18,
        name="head_frame_projects_out_from_wall_plate",
    )

    with ctx.pose({shoulder: 1.0, elbow: -0.85, swivel: 0.55, tilt: 0.30}):
        ctx.expect_gap(
            head_frame,
            wall_plate,
            axis="x",
            min_gap=0.06,
            name="head_frame_stays_in_front_of_wall_when_articulated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
