from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]


def _annular_plate(outer_radius: float, inner_radius: float, thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, 56),
            [_circle_profile(inner_radius, 48)],
            thickness,
            center=True,
        ),
        name,
    )


def _clevis_plate(width: float, height: float, hole_radius: float, thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(width, height),
            [_circle_profile(hole_radius, 40)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tool_wrist")

    base_mat = model.material("powder_coated_dark_gray", rgba=(0.10, 0.11, 0.12, 1.0))
    edge_mat = model.material("black_rubber_feet", rgba=(0.015, 0.015, 0.015, 1.0))
    bearing_mat = model.material("brushed_bearing_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    yaw_mat = model.material("anodized_blue_cartridge", rgba=(0.05, 0.24, 0.55, 1.0))
    pitch_mat = model.material("safety_orange_frame", rgba=(0.95, 0.42, 0.05, 1.0))
    output_mat = model.material("machined_tool_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    index_mat = model.material("white_roll_index", rgba=(0.92, 0.92, 0.86, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.34, 0.25, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=base_mat,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=base_mat,
        name="bearing_pedestal",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=bearing_mat,
        name="thrust_bearing_face",
    )
    for x, name in [(-0.115, "post_0"), (0.115, "post_1")]:
        base.visual(
            Box((0.020, 0.060, 0.105)),
            origin=Origin(xyz=(x, 0.0, 0.0825)),
            material=base_mat,
            name=name,
        )
    base.visual(
        Box((0.250, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, -0.111, 0.045)),
        material=base_mat,
        name="rear_tie_bar",
    )
    for x, y, name in [
        (-0.135, -0.095, "foot_0"),
        (0.135, -0.095, "foot_1"),
        (-0.135, 0.095, "foot_2"),
        (0.135, 0.095, "foot_3"),
    ]:
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.033)),
            material=edge_mat,
            name=name,
        )

    yaw = model.part("yaw_cartridge")
    yaw.visual(
        Cylinder(radius=0.072, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=yaw_mat,
        name="yaw_body",
    )
    yaw.visual(
        Cylinder(radius=0.094, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=yaw_mat,
        name="top_flange",
    )
    yaw.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=bearing_mat,
        name="lower_race",
    )
    yaw.visual(
        _clevis_plate(0.055, 0.100, 0.024, 0.025, "pitch_cheek_0_mesh"),
        origin=Origin(xyz=(0.0, -0.080, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=yaw_mat,
        name="pitch_cheek_0",
    )
    yaw.visual(
        _clevis_plate(0.055, 0.100, 0.024, 0.025, "pitch_cheek_1_mesh"),
        origin=Origin(xyz=(0.0, 0.080, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=yaw_mat,
        name="pitch_cheek_1",
    )
    yaw.visual(
        Box((0.070, 0.135, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=yaw_mat,
        name="cheek_bridge",
    )
    yaw.visual(
        _annular_plate(0.032, 0.023, 0.016, "pitch_bearing_0_mesh"),
        origin=Origin(xyz=(0.0, -0.1005, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="pitch_bearing_0",
    )
    yaw.visual(
        _annular_plate(0.032, 0.023, 0.016, "pitch_bearing_1_mesh"),
        origin=Origin(xyz=(0.0, 0.1005, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="pitch_bearing_1",
    )

    pitch = model.part("pitch_frame")
    pitch.visual(
        Cylinder(radius=0.020, length=0.214),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="pitch_axle",
    )
    pitch.visual(
        Box((0.080, 0.100, 0.070)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=pitch_mat,
        name="center_block",
    )
    pitch.visual(
        _annular_plate(0.045, 0.034, 0.060, "roll_bearing_mesh"),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pitch_mat,
        name="roll_bearing",
    )
    for y, name in [(-0.055, "roll_fork_0"), (0.055, "roll_fork_1")]:
        pitch.visual(
            Box((0.100, 0.014, 0.088)),
            origin=Origin(xyz=(0.105, y, 0.0)),
            material=pitch_mat,
            name=name,
        )

    roll = model.part("roll_output")
    roll.visual(
        Cylinder(radius=0.032, length=0.200),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=output_mat,
        name="output_shaft",
    )
    roll.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(xyz=(0.1825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=output_mat,
        name="tool_flange",
    )
    roll.visual(
        Cylinder(radius=0.026, length=0.035),
        origin=Origin(xyz=(0.2125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=output_mat,
        name="tool_stub",
    )
    roll.visual(
        Box((0.085, 0.012, 0.014)),
        origin=Origin(xyz=(0.095, 0.0, 0.039)),
        material=index_mat,
        name="roll_index",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw,
        child=pitch,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch,
        child=roll,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    yaw = object_model.get_part("yaw_cartridge")
    pitch = object_model.get_part("pitch_frame")
    roll = object_model.get_part("roll_output")
    yaw_joint = object_model.get_articulation("yaw")
    pitch_joint = object_model.get_articulation("pitch")
    roll_joint = object_model.get_articulation("roll")

    ctx.check("three supported wrist stages", len(object_model.parts) == 4 and len(object_model.articulations) == 3)
    ctx.check("yaw axis is vertical", tuple(yaw_joint.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch axis is crosswise", tuple(pitch_joint.axis) == (0.0, 1.0, 0.0))
    ctx.check("roll axis is output aligned", tuple(roll_joint.axis) == (1.0, 0.0, 0.0))

    ctx.allow_overlap(
        pitch,
        yaw,
        elem_a="pitch_axle",
        elem_b="pitch_cheek_0",
        reason="The pitch shaft is intentionally captured through the yaw clevis cheek bore.",
    )
    ctx.allow_overlap(
        pitch,
        yaw,
        elem_a="pitch_axle",
        elem_b="pitch_cheek_1",
        reason="The pitch shaft is intentionally captured through the opposite yaw clevis cheek bore.",
    )
    ctx.allow_overlap(
        pitch,
        yaw,
        elem_a="pitch_axle",
        elem_b="pitch_bearing_0",
        reason="The pitch shaft is intentionally seated through the yaw bearing bore.",
    )
    ctx.allow_overlap(
        pitch,
        yaw,
        elem_a="pitch_axle",
        elem_b="pitch_bearing_1",
        reason="The pitch shaft is intentionally seated through the opposite yaw bearing bore.",
    )
    ctx.allow_overlap(
        pitch,
        roll,
        elem_a="roll_bearing",
        elem_b="output_shaft",
        reason="The roll output shaft is intentionally seated through the pitch-frame bearing bore.",
    )

    ctx.expect_contact(
        yaw,
        base,
        elem_a="lower_race",
        elem_b="thrust_bearing_face",
        contact_tol=0.001,
        name="yaw cartridge sits on base bearing",
    )
    ctx.expect_overlap(
        pitch,
        yaw,
        axes="y",
        elem_a="pitch_axle",
        elem_b="pitch_bearing_0",
        min_overlap=0.010,
        name="pitch axle reaches yaw bearing",
    )
    ctx.expect_overlap(
        pitch,
        yaw,
        axes="y",
        elem_a="pitch_axle",
        elem_b="pitch_cheek_0",
        min_overlap=0.020,
        name="pitch axle is retained in near cheek",
    )
    ctx.expect_overlap(
        pitch,
        yaw,
        axes="y",
        elem_a="pitch_axle",
        elem_b="pitch_cheek_1",
        min_overlap=0.020,
        name="pitch axle is retained in far cheek",
    )
    ctx.expect_within(
        pitch,
        yaw,
        axes="xz",
        inner_elem="pitch_axle",
        outer_elem="pitch_bearing_0",
        margin=0.001,
        name="pitch axle is centered in bearing bore",
    )
    ctx.expect_overlap(
        roll,
        pitch,
        axes="x",
        elem_a="output_shaft",
        elem_b="roll_bearing",
        min_overlap=0.048,
        name="roll shaft passes through pitch bearing",
    )
    ctx.expect_within(
        roll,
        pitch,
        axes="yz",
        inner_elem="output_shaft",
        outer_elem="roll_bearing",
        margin=0.001,
        name="roll shaft stays centered in bearing bore",
    )

    rest_roll_pos = ctx.part_world_position(roll)
    with ctx.pose({yaw_joint: 0.65, pitch_joint: -0.45, roll_joint: 1.2}):
        moved_roll_pos = ctx.part_world_position(roll)
        ctx.expect_overlap(
            pitch,
            yaw,
            axes="y",
            elem_a="pitch_axle",
            elem_b="pitch_bearing_0",
            min_overlap=0.010,
            name="pitch axle remains aligned with its support at pose",
        )

    ctx.check(
        "compound wrist pose moves output frame",
        rest_roll_pos is not None
        and moved_roll_pos is not None
        and abs(moved_roll_pos[0] - rest_roll_pos[0]) > 0.020,
        details=f"rest={rest_roll_pos}, posed={moved_roll_pos}",
    )

    return ctx.report()


object_model = build_object_model()
