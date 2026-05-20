from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_OUTER_X = 0.160
FRAME_OUTER_Y = 0.118
FRAME_OUTER_Z = 0.142
SIDE_PLATE_T = 0.014
RAIL_T = 0.015
FRAME_CLEAR_RADIUS = 0.018

CARTRIDGE_LEN = 0.016
CARTRIDGE_BODY = 0.052
CARTRIDGE_BOSS_LEN = 0.006
CARTRIDGE_BORE_R = 0.0105
CARTRIDGE_OUTER_R = 0.024

SHAFT_R = 0.0095
SHAFT_LEN = 0.224
COLLAR_LEN = 0.008
COLLAR_R = 0.017

LEFT_PLATE_CENTER_X = -(FRAME_OUTER_X / 2.0 - SIDE_PLATE_T / 2.0)
RIGHT_PLATE_CENTER_X = FRAME_OUTER_X / 2.0 - SIDE_PLATE_T / 2.0
LEFT_CARTRIDGE_CENTER_X = -(FRAME_OUTER_X / 2.0 + CARTRIDGE_LEN / 2.0)
RIGHT_CARTRIDGE_CENTER_X = FRAME_OUTER_X / 2.0 + CARTRIDGE_LEN / 2.0
LEFT_COLLAR_CENTER_X = -(FRAME_OUTER_X / 2.0 + CARTRIDGE_LEN + CARTRIDGE_BOSS_LEN + COLLAR_LEN / 2.0)
RIGHT_COLLAR_CENTER_X = FRAME_OUTER_X / 2.0 + CARTRIDGE_LEN + CARTRIDGE_BOSS_LEN + COLLAR_LEN / 2.0


def _x_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _frame_shell() -> cq.Workplane:
    side_plate = cq.Workplane("XY").box(SIDE_PLATE_T, FRAME_OUTER_Y, FRAME_OUTER_Z)
    left_plate = side_plate.translate((LEFT_PLATE_CENTER_X, 0.0, 0.0))
    right_plate = side_plate.translate((RIGHT_PLATE_CENTER_X, 0.0, 0.0))

    rail_span_x = FRAME_OUTER_X - 2.0 * SIDE_PLATE_T
    top_rail = cq.Workplane("XY").box(rail_span_x, FRAME_OUTER_Y, RAIL_T).translate(
        (0.0, 0.0, FRAME_OUTER_Z / 2.0 - RAIL_T / 2.0)
    )
    bottom_rail = cq.Workplane("XY").box(rail_span_x, FRAME_OUTER_Y, RAIL_T).translate(
        (0.0, 0.0, -(FRAME_OUTER_Z / 2.0 - RAIL_T / 2.0))
    )

    hole_cutter = _x_cylinder(SIDE_PLATE_T + 0.010, FRAME_CLEAR_RADIUS)
    left_hole = hole_cutter.translate((LEFT_PLATE_CENTER_X, 0.0, 0.0))
    right_hole = hole_cutter.translate((RIGHT_PLATE_CENTER_X, 0.0, 0.0))

    return left_plate.union(right_plate).union(top_rail).union(bottom_rail).cut(left_hole).cut(right_hole)


def _bearing_cartridge(sign: float) -> cq.Workplane:
    center_x = LEFT_CARTRIDGE_CENTER_X if sign < 0 else RIGHT_CARTRIDGE_CENTER_X
    boss_shift = sign * (CARTRIDGE_LEN / 2.0 + CARTRIDGE_BOSS_LEN / 2.0)

    body = (
        cq.Workplane("XY")
        .box(CARTRIDGE_LEN, CARTRIDGE_BODY, CARTRIDGE_BODY)
        .edges("|X")
        .fillet(0.003)
    )
    boss = _x_cylinder(CARTRIDGE_BOSS_LEN, CARTRIDGE_OUTER_R).translate((boss_shift, 0.0, 0.0))
    bore = _x_cylinder(CARTRIDGE_LEN + CARTRIDGE_BOSS_LEN + 0.008, CARTRIDGE_BORE_R)
    return body.union(boss).cut(bore).translate((center_x, 0.0, 0.0))


def _shaft() -> cq.Workplane:
    return _x_cylinder(SHAFT_LEN, SHAFT_R)


def _collar(center_x: float) -> cq.Workplane:
    collar = _x_cylinder(COLLAR_LEN, COLLAR_R).cut(_x_cylinder(COLLAR_LEN + 0.004, SHAFT_R))
    return collar.translate((center_x, 0.0, 0.0))


def _payload_bracket() -> cq.Workplane:
    clamp_block = (
        cq.Workplane("XY")
        .box(0.026, 0.030, 0.018)
        .translate((0.0, 0.0, 0.012))
        .cut(_x_cylinder(0.032, SHAFT_R * 0.97))
    )
    upright = cq.Workplane("XY").box(0.016, 0.008, 0.028).translate((0.0, 0.012, 0.032))
    top_pad = cq.Workplane("XY").box(0.028, 0.020, 0.006).translate((0.0, 0.018, 0.046))
    gusset = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.016)
        .lineTo(0.0, 0.040)
        .lineTo(0.014, 0.040)
        .close()
        .extrude(0.016, both=True)
        .translate((0.0, 0.004, 0.0))
    )
    return clamp_block.union(upright).union(top_pad).union(gusset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_roll_axis_module", assets=ASSETS)

    frame_gray = model.material("frame_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.33, 0.35, 0.38, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.16, 0.17, 0.18, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.58, 0.60, 0.63, 1.0))

    frame = model.part("support_frame")
    frame.visual(
        mesh_from_cadquery(_frame_shell(), "support_frame_shell.obj", assets=ASSETS),
        name="frame_shell",
        material=frame_gray,
    )
    frame.visual(
        mesh_from_cadquery(_bearing_cartridge(-1.0), "left_bearing_cartridge.obj", assets=ASSETS),
        name="left_bearing_cartridge",
        material=bearing_black,
    )
    frame.visual(
        mesh_from_cadquery(_bearing_cartridge(1.0), "right_bearing_cartridge.obj", assets=ASSETS),
        name="right_bearing_cartridge",
        material=bearing_black,
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_X + 2.0 * CARTRIDGE_LEN, FRAME_OUTER_Y, FRAME_OUTER_Z)),
        mass=2.0,
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_cadquery(_shaft(), "rotary_shaft.obj", assets=ASSETS),
        name="shaft",
        material=steel_dark,
    )
    rotor.visual(
        mesh_from_cadquery(_collar(LEFT_COLLAR_CENTER_X), "left_retaining_collar.obj", assets=ASSETS),
        name="left_collar",
        material=steel_dark,
    )
    rotor.visual(
        mesh_from_cadquery(_collar(RIGHT_COLLAR_CENTER_X), "right_retaining_collar.obj", assets=ASSETS),
        name="right_collar",
        material=steel_dark,
    )
    rotor.visual(
        mesh_from_cadquery(_payload_bracket(), "payload_bracket.obj", assets=ASSETS),
        name="payload_bracket",
        material=bracket_gray,
    )
    rotor.inertial = Inertial.from_geometry(
        Box((SHAFT_LEN, 0.044, 0.060)),
        mass=0.85,
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=6.0,
            lower=-3.14159,
            upper=3.14159,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("support_frame")
    rotor = object_model.get_part("rotor")
    roll_joint = object_model.get_articulation("frame_to_rotor")

    frame_shell = frame.get_visual("frame_shell")
    left_cartridge = frame.get_visual("left_bearing_cartridge")
    right_cartridge = frame.get_visual("right_bearing_cartridge")
    shaft = rotor.get_visual("shaft")
    left_collar = rotor.get_visual("left_collar")
    right_collar = rotor.get_visual("right_collar")
    payload_bracket = rotor.get_visual("payload_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="roll_axis_pose_clearance")

    ctx.check("support_frame_exists", frame is not None, "support_frame part lookup failed")
    ctx.check("rotor_exists", rotor is not None, "rotor part lookup failed")
    ctx.check("frame_shell_exists", frame_shell is not None, "frame shell visual missing")
    ctx.check("left_cartridge_exists", left_cartridge is not None, "left bearing cartridge missing")
    ctx.check("right_cartridge_exists", right_cartridge is not None, "right bearing cartridge missing")
    ctx.check("shaft_exists", shaft is not None, "shaft visual missing")
    ctx.check("left_collar_exists", left_collar is not None, "left retaining collar missing")
    ctx.check("right_collar_exists", right_collar is not None, "right retaining collar missing")
    ctx.check("payload_bracket_exists", payload_bracket is not None, "payload bracket missing")

    joint_type = getattr(roll_joint, "articulation_type", None)
    joint_axis = tuple(getattr(roll_joint, "axis", ()))
    ctx.check(
        "roll_joint_is_revolute",
        joint_type == ArticulationType.REVOLUTE,
        f"expected revolute roll joint, got {joint_type!r}",
    )
    ctx.check(
        "roll_joint_axis_is_coaxial_x",
        joint_axis == (1.0, 0.0, 0.0),
        f"expected roll axis (1,0,0), got {joint_axis!r}",
    )

    ctx.expect_contact(
        rotor,
        frame,
        elem_a=left_collar,
        elem_b=left_cartridge,
        name="left_retaining_collar_seats_on_left_cartridge",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a=right_collar,
        elem_b=right_cartridge,
        name="right_retaining_collar_seats_on_right_cartridge",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="x",
        min_overlap=CARTRIDGE_LEN - 0.001,
        elem_a=shaft,
        elem_b=left_cartridge,
        name="shaft_runs_through_left_cartridge_depth",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="x",
        min_overlap=CARTRIDGE_LEN - 0.001,
        elem_a=shaft,
        elem_b=right_cartridge,
        name="shaft_runs_through_right_cartridge_depth",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="yz",
        margin=0.001,
        inner_elem=shaft,
        outer_elem=left_cartridge,
        name="shaft_centerline_stays_within_left_cartridge_body",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="yz",
        margin=0.001,
        inner_elem=shaft,
        outer_elem=right_cartridge,
        name="shaft_centerline_stays_within_right_cartridge_body",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="yz",
        min_overlap=0.020,
        elem_a=payload_bracket,
        elem_b=frame_shell,
        name="payload_bracket_sits_inside_frame_projection",
    )

    for angle in (-1.57, 1.57):
        with ctx.pose({roll_joint: angle}):
            ctx.expect_contact(
                rotor,
                frame,
                elem_a=left_collar,
                elem_b=left_cartridge,
                name=f"left_collar_remains_seated_at_roll_{angle:+.2f}",
            )
            ctx.expect_contact(
                rotor,
                frame,
                elem_a=right_collar,
                elem_b=right_cartridge,
                name=f"right_collar_remains_seated_at_roll_{angle:+.2f}",
            )
            ctx.expect_overlap(
                rotor,
                frame,
                axes="yz",
                min_overlap=0.020,
                elem_a=payload_bracket,
                elem_b=frame_shell,
                name=f"payload_bracket_projection_stays_inside_frame_at_roll_{angle:+.2f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
