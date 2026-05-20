from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _washer_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 96,
):
    geom = MeshGeometry()
    half_height = height * 0.5
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, -half_height))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, half_height))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, -half_height))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, half_height))

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        j = (i + 1) % segments
        add_quad(outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        add_quad(inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        add_quad(outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        add_quad(outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return mesh_from_geometry(
        geom,
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_offset_rotary_bracket")

    powder = model.material("powder_coated_black", rgba=(0.08, 0.085, 0.09, 1.0))
    dark = model.material("dark_anodized", rgba=(0.13, 0.14, 0.15, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.02, 0.022, 0.025, 1.0))
    accent = model.material("red_index", rgba=(0.80, 0.08, 0.04, 1.0))

    lower_hub_mesh = _washer_mesh(
        "lower_platform_annular_hub",
        outer_radius=0.060,
        inner_radius=0.014,
        height=0.018,
    )
    lower_cap_mesh = _washer_mesh(
        "lower_platform_raised_ring",
        outer_radius=0.038,
        inner_radius=0.014,
        height=0.010,
    )
    thrust_mesh = _washer_mesh(
        "lower_stationary_thrust_washer",
        outer_radius=0.058,
        inner_radius=0.014,
        height=0.006,
    )
    cartridge_mesh = _washer_mesh(
        "rotary_cartridge_bored_body",
        outer_radius=0.032,
        inner_radius=0.012,
        height=0.050,
    )
    cartridge_cap_mesh = _washer_mesh(
        "rotary_cartridge_top_cap",
        outer_radius=0.028,
        inner_radius=0.012,
        height=0.006,
    )
    socket_mesh = _washer_mesh(
        "distal_open_socket_ring",
        outer_radius=0.052,
        inner_radius=0.038,
        height=0.006,
    )

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.014, 0.205, 0.320)),
        origin=Origin(xyz=(-0.095, 0.0, 0.100)),
        material=powder,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.128, 0.150, 0.014)),
        origin=Origin(xyz=(-0.031, 0.0, -0.020)),
        material=powder,
        name="lower_shelf",
    )
    for y in (-0.057, 0.057):
        backplate.visual(
            Box((0.112, 0.008, 0.070)),
            origin=Origin(xyz=(-0.041, y, -0.053)),
            material=powder,
            name=f"side_web_{0 if y < 0 else 1}",
        )
    backplate.visual(
        thrust_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=steel,
        name="thrust_ring",
    )
    backplate.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="lower_pin",
    )
    for y in (-0.070, 0.070):
        for z in (0.020, 0.175):
            backplate.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(-0.086, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"screw_boss_{int((y + 0.071) * 1000)}_{int(z * 1000)}",
            )
            backplate.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=Origin(xyz=(-0.083, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=shadow,
                name=f"screw_recess_{int((y + 0.071) * 1000)}_{int(z * 1000)}",
            )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        lower_hub_mesh,
        origin=Origin(),
        material=dark,
        name="lower_hub",
    )
    lower_platform.visual(
        lower_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="upper_bearing_ring",
    )
    lower_platform.visual(
        Box((0.150, 0.036, 0.024)),
        origin=Origin(xyz=(0.105, 0.0, 0.026)),
        material=dark,
        name="center_arm",
    )
    for y in (-0.047, 0.047):
        lower_platform.visual(
            Box((0.184, 0.012, 0.026)),
            origin=Origin(xyz=(0.137, y, 0.027)),
            material=dark,
            name=f"offset_rail_{0 if y < 0 else 1}",
        )
    lower_platform.visual(
        Box((0.040, 0.106, 0.012)),
        origin=Origin(xyz=(0.180, 0.0, 0.043)),
        material=dark,
        name="cross_tie",
    )
    for index, (x, y) in enumerate(
        (
            (0.235, -0.050),
            (0.235, 0.050),
            (0.185, 0.000),
            (0.285, 0.000),
        )
    ):
        lower_platform.visual(
            Cylinder(radius=0.010, length=0.038),
            origin=Origin(xyz=(x, y, 0.026)),
            material=steel,
            name=f"socket_post_{index}",
        )
    lower_platform.visual(
        socket_mesh,
        origin=Origin(xyz=(0.235, 0.0, 0.004)),
        material=steel,
        name="cartridge_socket",
    )
    lower_platform.visual(
        Box((0.014, 0.105, 0.012)),
        origin=Origin(xyz=(0.286, 0.0, 0.028)),
        material=steel,
        name="front_socket_bridge",
    )
    lower_platform.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.235, 0.0, -0.002)),
        material=steel,
        name="pin_root_boss",
    )
    lower_platform.visual(
        Box((0.080, 0.008, 0.004)),
        origin=Origin(xyz=(0.235, 0.0, -0.001)),
        material=steel,
        name="pin_spoke_x",
    )
    lower_platform.visual(
        Box((0.008, 0.080, 0.004)),
        origin=Origin(xyz=(0.235, 0.0, -0.001)),
        material=steel,
        name="pin_spoke_y",
    )
    lower_platform.visual(
        Cylinder(radius=0.009, length=0.068),
        origin=Origin(xyz=(0.235, 0.0, 0.028)),
        material=steel,
        name="cartridge_pin",
    )

    cartridge = model.part("cartridge")
    cartridge.visual(
        cartridge_mesh,
        origin=Origin(),
        material=powder,
        name="cartridge_body",
    )
    cartridge.visual(
        cartridge_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=steel,
        name="top_bearing_cap",
    )
    cartridge.visual(
        Box((0.030, 0.006, 0.004)),
        origin=Origin(xyz=(0.014, 0.0, 0.0325)),
        material=accent,
        name="index_tab",
    )
    cartridge.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(0.000, -0.026, -0.004)),
        material=dark,
        name="output_flat",
    )

    model.articulation(
        "plate_to_platform",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=lower_platform,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "platform_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=lower_platform,
        child=cartridge,
        origin=Origin(xyz=(0.235, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    backplate = object_model.get_part("backplate")
    lower_platform = object_model.get_part("lower_platform")
    cartridge = object_model.get_part("cartridge")
    lower_joint = object_model.get_articulation("plate_to_platform")
    cartridge_joint = object_model.get_articulation("platform_to_cartridge")

    revolutes = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two parallel rotary joints",
        len(revolutes) == 2
        and tuple(lower_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(cartridge_joint.axis) == (0.0, 0.0, 1.0),
        details=f"revolutes={len(revolutes)}, axes={lower_joint.axis}, {cartridge_joint.axis}",
    )

    ctx.expect_gap(
        lower_platform,
        backplate,
        axis="z",
        positive_elem="lower_hub",
        negative_elem="thrust_ring",
        min_gap=0.0,
        max_gap=0.001,
        name="lower platform rides on thrust washer",
    )
    ctx.expect_within(
        backplate,
        lower_platform,
        axes="xy",
        inner_elem="lower_pin",
        outer_elem="lower_hub",
        margin=0.001,
        name="lower pin is centered in bored platform",
    )
    ctx.expect_origin_gap(
        cartridge,
        lower_platform,
        axis="x",
        min_gap=0.20,
        name="cartridge axis is offset forward on the arm",
    )
    ctx.expect_within(
        cartridge,
        lower_platform,
        axes="xy",
        inner_elem="cartridge_body",
        outer_elem="cartridge_socket",
        margin=0.0,
        name="cartridge body sits inside the distal socket footprint",
    )

    rest_cartridge_pos = ctx.part_world_position(cartridge)
    with ctx.pose({lower_joint: 1.0}):
        swung_cartridge_pos = ctx.part_world_position(cartridge)
    ctx.check(
        "lower platform swings carried cartridge",
        rest_cartridge_pos is not None
        and swung_cartridge_pos is not None
        and swung_cartridge_pos[1] > rest_cartridge_pos[1] + 0.12,
        details=f"rest={rest_cartridge_pos}, swung={swung_cartridge_pos}",
    )

    rest_origin = ctx.part_world_position(cartridge)
    with ctx.pose({cartridge_joint: 1.2}):
        rotated_origin = ctx.part_world_position(cartridge)
    ctx.check(
        "cartridge spins about its own carried axis",
        rest_origin is not None
        and rotated_origin is not None
        and abs(rest_origin[0] - rotated_origin[0]) < 1e-6
        and abs(rest_origin[1] - rotated_origin[1]) < 1e-6,
        details=f"rest={rest_origin}, rotated={rotated_origin}",
    )

    return ctx.report()


object_model = build_object_model()
