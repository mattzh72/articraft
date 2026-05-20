from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


BODY_HEIGHT = 0.188
UPPER_CENTER_Z = 0.034
MID_CENTER_Z = -0.018
LOWER_CENTER_Z = -0.054


def _rounded_block(
    width: float,
    depth: float,
    height: float,
    *,
    face_radius: float,
    side_radius: float,
):
    return cq.Workplane("XY").box(width, depth, height)


def _case_solid(
    *,
    upper_width: float,
    mid_width: float,
    lower_width: float,
    upper_depth: float,
    mid_depth: float,
    lower_depth: float,
    face_radius: float,
    side_radius: float,
):
    upper = _rounded_block(
        upper_width,
        upper_depth,
        0.120,
        face_radius=face_radius,
        side_radius=side_radius,
    ).translate((0.0, 0.0, UPPER_CENTER_Z))
    middle = _rounded_block(
        mid_width,
        mid_depth,
        0.030,
        face_radius=face_radius * 0.9,
        side_radius=side_radius,
    ).translate((0.0, 0.0, MID_CENTER_Z))
    lower = _rounded_block(
        lower_width,
        lower_depth,
        0.080,
        face_radius=face_radius,
        side_radius=side_radius,
    ).translate((0.0, 0.0, LOWER_CENTER_Z))
    return upper.union(middle).union(lower)


def _body_core_shape():
    return _case_solid(
        upper_width=0.084,
        mid_width=0.090,
        lower_width=0.096,
        upper_depth=0.040,
        mid_depth=0.041,
        lower_depth=0.042,
        face_radius=0.014,
        side_radius=0.006,
    )


def _body_holster_shape():
    outer = _case_solid(
        upper_width=0.092,
        mid_width=0.098,
        lower_width=0.104,
        upper_depth=0.046,
        mid_depth=0.047,
        lower_depth=0.048,
        face_radius=0.018,
        side_radius=0.008,
    )
    inner = _case_solid(
        upper_width=0.083,
        mid_width=0.089,
        lower_width=0.095,
        upper_depth=0.052,
        mid_depth=0.053,
        lower_depth=0.054,
        face_radius=0.013,
        side_radius=0.005,
    )
    return outer.cut(inner)


def _stand_shape():
    plate = _rounded_block(
        0.074,
        0.004,
        0.116,
        face_radius=0.010,
        side_radius=0.002,
    ).translate((0.0, 0.002, 0.058))
    barrel = (
        cq.Workplane("XY")
        .cylinder(0.074, 0.004)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    rib_left = cq.Workplane("XY").box(0.010, 0.008, 0.060).translate((-0.020, -0.001, 0.032))
    rib_right = cq.Workplane("XY").box(0.010, 0.008, 0.060).translate((0.020, -0.001, 0.032))
    toe = cq.Workplane("XY").box(0.056, 0.010, 0.008).translate((0.0, -0.001, 0.111))
    return plate.union(barrel).union(rib_left).union(rib_right).union(toe)


def _function_key_shape():
    return _rounded_block(
        0.016,
        0.0034,
        0.008,
        face_radius=0.003,
        side_radius=0.0015,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_digital_multimeter")

    case_black = model.material("case_black", rgba=(0.14, 0.15, 0.16, 1.0))
    holster_yellow = model.material("holster_yellow", rgba=(0.89, 0.72, 0.13, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.32, 0.34, 0.36, 1.0))
    key_gray = model.material("key_gray", rgba=(0.50, 0.53, 0.56, 1.0))
    button_gray = model.material("button_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    dial_black = model.material("dial_black", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.32, 0.56, 0.62, 0.40))
    lens = model.material("lens", rgba=(0.92, 0.95, 0.98, 0.70))
    port_black = model.material("port_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_core_shape(), "multimeter_body_core"),
        material=case_black,
        name="case_core",
    )
    body.visual(
        mesh_from_cadquery(_body_holster_shape(), "multimeter_holster"),
        material=holster_yellow,
        name="corner_bumper",
    )
    for index, (x_pos, z_pos) in enumerate(
        (
            (-0.039, 0.080),
            (0.039, 0.080),
            (-0.045, -0.078),
            (0.045, -0.078),
        )
    ):
        body.visual(
            Cylinder(radius=0.0105, length=0.048),
            origin=Origin(
                xyz=(x_pos, 0.0, z_pos),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=holster_yellow,
            name=f"bumper_round_{index}",
        )
    body.visual(
        Box((0.062, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, 0.0210, 0.057)),
        material=bezel_gray,
        name="screen_bezel",
    )
    body.visual(
        Box((0.054, 0.002, 0.028)),
        origin=Origin(xyz=(0.0, 0.0220, 0.057)),
        material=glass,
        name="screen",
    )
    body.visual(
        Box((0.078, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0205, -0.040)),
        material=bezel_gray,
        name="button_sill",
    )
    for index, x_pos in enumerate((-0.028, 0.0, 0.028)):
        body.visual(
            Cylinder(radius=0.0062, length=0.004),
            origin=Origin(
                xyz=(x_pos, 0.0210, -0.078),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=port_black,
            name=f"input_port_{index}",
        )
    body.visual(
        Cylinder(radius=0.0060, length=0.003),
        origin=Origin(
            xyz=(-0.026, 0.0215, 0.073),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=lens,
        name="flashlight_lens",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.012,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.054, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "multimeter_range_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_cap",
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "multimeter_stand"),
        material=case_black,
        name="stand_panel",
    )

    key_mesh = mesh_from_cadquery(_function_key_shape(), "multimeter_function_key")
    key_x_positions = (-0.022, 0.0, 0.022)
    for index, x_pos in enumerate(key_x_positions):
        key_part = model.part(f"function_key_{index}")
        key_part.visual(
            key_mesh,
            origin=Origin(xyz=(0.0, 0.0017, 0.0)),
            material=key_gray,
            name="key_cap",
        )
        model.articulation(
            f"body_to_function_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key_part,
            origin=Origin(xyz=(x_pos, 0.0190, -0.040)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0016,
            ),
        )

    flashlight_button = model.part("flashlight_button")
    flashlight_button.visual(
        Cylinder(radius=0.0052, length=0.0032),
        origin=Origin(
            xyz=(0.0, 0.0016, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=button_gray,
        name="button_cap",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0190, -0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.0250, -0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "body_to_flashlight_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flashlight_button,
        origin=Origin(xyz=(0.028, 0.0192, 0.071)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0015,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    flashlight_button = object_model.get_part("flashlight_button")
    function_keys = [object_model.get_part(f"function_key_{index}") for index in range(3)]

    stand_joint = object_model.get_articulation("body_to_stand")
    flashlight_joint = object_model.get_articulation("body_to_flashlight_button")

    ctx.expect_origin_distance(
        dial,
        body,
        axes="x",
        max_dist=0.004,
        name="dial stays centered on the body",
    )
    ctx.expect_origin_gap(
        dial,
        function_keys[1],
        axis="z",
        min_gap=0.030,
        name="dial sits above the function-key row",
    )
    ctx.expect_origin_gap(
        flashlight_button,
        dial,
        axis="z",
        min_gap=0.060,
        name="flashlight button sits near the top corner",
    )
    ctx.expect_origin_distance(
        flashlight_button,
        body,
        axes="x",
        min_dist=0.020,
        max_dist=0.040,
        name="flashlight button is offset toward one top corner",
    )

    with ctx.pose({stand_joint: 0.0}):
        ctx.expect_contact(
            body,
            stand,
            contact_tol=0.0035,
            name="rear stand nests against the back case when folded",
        )
        ctx.expect_overlap(
            body,
            stand,
            axes="xz",
            min_overlap=0.060,
            name="rear stand spans a broad portion of the back",
        )

    closed_stand_aabb = ctx.part_world_aabb(stand)
    stand_limits = stand_joint.motion_limits
    opened_stand_aabb = None
    if stand_limits is not None and stand_limits.upper is not None:
        with ctx.pose({stand_joint: stand_limits.upper}):
            opened_stand_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "rear stand swings out behind the meter",
        closed_stand_aabb is not None
        and opened_stand_aabb is not None
        and opened_stand_aabb[0][1] < closed_stand_aabb[0][1] - 0.040,
        details=f"closed={closed_stand_aabb}, opened={opened_stand_aabb}",
    )

    for index, key_part in enumerate(function_keys):
        key_joint = object_model.get_articulation(f"body_to_function_key_{index}")
        key_limits = key_joint.motion_limits
        rest_position = ctx.part_world_position(key_part)
        pressed_position = None
        if key_limits is not None and key_limits.upper is not None:
            with ctx.pose({key_joint: key_limits.upper}):
                pressed_position = ctx.part_world_position(key_part)
        ctx.check(
            f"function_key_{index}_presses_inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] < rest_position[1] - 0.0010,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    flash_limits = flashlight_joint.motion_limits
    flash_rest_position = ctx.part_world_position(flashlight_button)
    flash_pressed_position = None
    if flash_limits is not None and flash_limits.upper is not None:
        with ctx.pose({flashlight_joint: flash_limits.upper}):
            flash_pressed_position = ctx.part_world_position(flashlight_button)
    ctx.check(
        "flashlight button presses inward",
        flash_rest_position is not None
        and flash_pressed_position is not None
        and flash_pressed_position[1] < flash_rest_position[1] - 0.0010,
        details=f"rest={flash_rest_position}, pressed={flash_pressed_position}",
    )

    return ctx.report()


object_model = build_object_model()
