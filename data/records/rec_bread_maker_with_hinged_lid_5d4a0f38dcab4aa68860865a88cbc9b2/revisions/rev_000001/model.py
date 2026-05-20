from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.360
BODY_D = 0.290
BODY_H = 0.318
BODY_FRONT_Y = -0.145
HINGE_Y = 0.122
HINGE_Z = 0.327

CHAMBER_Y = 0.010
CHAMBER_FLOOR_Z = 0.088
CHAMBER_W = 0.170
CHAMBER_D = 0.130

CONTROL_FACE_Y = -0.162
CONTROL_Z = 0.223

PAN_BASE_Z = CHAMBER_FLOOR_Z
PAN_H = 0.182


def _build_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
    )

    control_strip = (
        cq.Workplane("XY")
        .box(0.320, 0.028, 0.095, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, -0.151, 0.175))
    )

    chamber = cq.Workplane("XY").box(
        CHAMBER_W,
        CHAMBER_D,
        BODY_H - CHAMBER_FLOOR_Z + 0.030,
        centered=(True, True, False),
    ).translate((0.0, CHAMBER_Y, CHAMBER_FLOOR_Z))

    return body.union(control_strip).cut(chamber)


def _build_lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(0.320, 0.222, 0.024, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.015)
        .translate((0.0, -0.131, -0.001))
    )
    pull = cq.Workplane("XY").box(0.090, 0.018, 0.010, centered=(True, True, False)).translate(
        (0.0, -0.233, -0.003)
    )
    barrel = cq.Workplane("YZ").circle(0.007).extrude(0.130, both=True)
    bridge = cq.Workplane("XY").box(0.220, 0.018, 0.012, centered=(True, True, False)).translate(
        (0.0, -0.011, -0.002)
    )
    window_cut = cq.Workplane("XY").box(0.122, 0.062, 0.040, centered=(True, True, False)).translate(
        (0.0, -0.132, -0.010)
    )
    return lid.union(pull).union(barrel).union(bridge).cut(window_cut)


def _button_cap(part, size: tuple[float, float, float], material, name: str, offset_y: float) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=(0.0, offset_y, size[2] * 0.5)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bread_maker")

    body_white = model.material("body_white", rgba=(0.91, 0.91, 0.88, 1.0))
    panel_charcoal = model.material("panel_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.93, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.30, 0.35, 0.40, 0.40))
    pan_grey = model.material("pan_grey", rgba=(0.36, 0.37, 0.39, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_silver = model.material("button_silver", rgba=(0.78, 0.78, 0.78, 1.0))
    release_grey = model.material("release_grey", rgba=(0.69, 0.69, 0.70, 1.0))
    paddle_dark = model.material("paddle_dark", rgba=(0.23, 0.24, 0.25, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.62, 0.63, 0.65, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "bread_maker_body"),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        Box((0.274, 0.003, 0.110)),
        origin=Origin(xyz=(0.015, -0.1605, CONTROL_Z)),
        material=panel_charcoal,
        name="control_bezel",
    )
    for index, x_pos in enumerate((-0.095, 0.095)):
        body.visual(
            Box((0.020, 0.016, 0.018)),
            origin=Origin(xyz=(x_pos, 0.137, 0.318)),
            material=body_white,
            name=f"rear_post_{index}",
        )
        body.visual(
            Box((0.032, 0.016, 0.016)),
            origin=Origin(xyz=(x_pos, 0.137, 0.327)),
            material=body_white,
            name=f"rear_mount_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "bread_maker_lid"),
        material=lid_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.126, 0.066, 0.002)),
        origin=Origin(xyz=(0.0, -0.132, 0.010)),
        material=glass_tint,
        name="window",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.320, 0.222, 0.024)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -0.111, 0.011)),
    )

    pan = model.part("pan")
    pan.visual(
        Box((0.148, 0.108, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=pan_grey,
        name="pan_floor",
    )
    pan.visual(
        Box((0.004, 0.108, 0.176)),
        origin=Origin(xyz=(-0.072, 0.0, 0.088)),
        material=pan_grey,
        name="pan_left",
    )
    pan.visual(
        Box((0.004, 0.108, 0.176)),
        origin=Origin(xyz=(0.072, 0.0, 0.088)),
        material=pan_grey,
        name="pan_right",
    )
    pan.visual(
        Box((0.140, 0.004, 0.176)),
        origin=Origin(xyz=(0.0, -0.052, 0.088)),
        material=pan_grey,
        name="pan_front",
    )
    pan.visual(
        Box((0.140, 0.004, 0.176)),
        origin=Origin(xyz=(0.0, 0.052, 0.088)),
        material=pan_grey,
        name="pan_rear",
    )
    pan.visual(
        Box((0.158, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, -0.0565, 0.173)),
        material=pan_grey,
        name="rim_front",
    )
    pan.visual(
        Box((0.158, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, 0.0565, 0.173)),
        material=pan_grey,
        name="rim_rear",
    )
    pan.visual(
        Box((0.005, 0.108, 0.006)),
        origin=Origin(xyz=(-0.0765, 0.0, 0.173)),
        material=pan_grey,
        name="rim_left",
    )
    pan.visual(
        Box((0.005, 0.108, 0.006)),
        origin=Origin(xyz=(0.0765, 0.0, 0.173)),
        material=pan_grey,
        name="rim_right",
    )
    pan.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=spindle_metal,
        name="spindle_boss",
    )
    pan.inertial = Inertial.from_geometry(
        Box((0.158, 0.118, PAN_H)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, PAN_H * 0.5)),
    )
    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=paddle_dark,
        name="hub",
    )
    paddle.visual(
        Box((0.060, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(0.20, 0.0, 0.0)),
        material=paddle_dark,
        name="blade",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.060, 0.018, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.022,
                body_style="skirted",
                top_diameter=0.032,
                edge_radius=0.002,
                skirt=KnobSkirt(0.048, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "timer_dial",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="dial_cap",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.022),
        mass=0.07,
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    program_button_parts = []
    for index, x_pos in enumerate((0.030, 0.078, 0.126)):
        button = model.part(f"program_button_{index}")
        _button_cap(button, (0.028, 0.010, 0.012), button_silver, "button_cap", -0.007)
        button.inertial = Inertial.from_geometry(
            Box((0.028, 0.010, 0.012)),
            mass=0.02,
            origin=Origin(xyz=(0.0, -0.007, 0.006)),
        )
        program_button_parts.append((button, x_pos))

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.050, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.009)),
        material=release_grey,
        name="release_flange",
    )
    _button_cap(release_button, (0.046, 0.012, 0.014), release_grey, "release_cap", -0.008)
    release_button.inertial = Inertial.from_geometry(
        Box((0.046, 0.012, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.008, 0.007)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.42),
    )
    model.articulation(
        "body_to_pan",
        ArticulationType.FIXED,
        parent=body,
        child=pan,
        origin=Origin(xyz=(0.0, CHAMBER_Y, PAN_BASE_Z)),
    )
    model.articulation(
        "pan_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=pan,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(-0.094, CONTROL_FACE_Y, CONTROL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )

    for index, (button, x_pos) in enumerate(program_button_parts):
        model.articulation(
            f"body_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_FACE_Y, CONTROL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.002),
        )

    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, 0.289)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.08, lower=0.0, upper=0.002),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pan = object_model.get_part("pan")
    paddle = object_model.get_part("paddle")
    release_button = object_model.get_part("release_button")
    program_buttons = [object_model.get_part(f"program_button_{index}") for index in range(3)]

    lid_hinge = object_model.get_articulation("body_to_lid")
    paddle_joint = object_model.get_articulation("pan_to_paddle")
    dial_joint = object_model.get_articulation("body_to_dial")
    release_joint = object_model.get_articulation("body_to_release_button")

    ctx.expect_overlap(lid, pan, axes="xy", min_overlap=0.115, name="closed lid covers the pan opening")
    ctx.expect_overlap(paddle, pan, axes="xy", min_overlap=0.018, name="paddle stays centered in the pan")
    ctx.expect_within(pan, body, axes="xy", margin=0.11, name="pan sits within the bread maker footprint")

    pan_aabb = ctx.part_world_aabb(pan)
    if pan_aabb is not None:
        pan_size_z = float(pan_aabb[1][2] - pan_aabb[0][2])
        ctx.check("pan depth reads as a deep loaf chamber", pan_size_z >= 0.17, details=f"pan_size_z={pan_size_z:.4f}")
    else:
        ctx.fail("pan depth reads as a deep loaf chamber", "Missing pan AABB.")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10
        and open_lid_aabb[0][1] > closed_lid_aabb[0][1] + 0.05,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.check(
        "continuous rotary controls are present",
        paddle_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(float(v) for v in paddle_joint.axis) == (0.0, 0.0, 1.0)
        and abs(abs(float(dial_joint.axis[1])) - 1.0) < 1e-9,
        details=f"paddle_axis={paddle_joint.axis}, dial_axis={dial_joint.axis}",
    )

    rest_release_pos = ctx.part_world_position(release_button)
    with ctx.pose({release_joint: release_joint.motion_limits.upper}):
        pressed_release_pos = ctx.part_world_position(release_button)
    ctx.check(
        "release button pushes into the housing",
        rest_release_pos is not None
        and pressed_release_pos is not None
        and pressed_release_pos[1] > rest_release_pos[1] + 0.0015,
        details=f"rest={rest_release_pos}, pressed={pressed_release_pos}",
    )

    for index, button in enumerate(program_buttons):
        joint = object_model.get_articulation(f"body_to_program_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"program button {index} presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
