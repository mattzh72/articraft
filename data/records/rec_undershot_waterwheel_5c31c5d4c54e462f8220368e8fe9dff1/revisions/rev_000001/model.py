from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, thickness: float, segments: int = 96):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments),
            [_circle_profile(inner_radius, segments)],
            thickness,
            center=True,
        ),
        name,
    )


def _add_bar_between(part, name: str, p1, p2, thickness: float, material: Material) -> None:
    """Rectangular tube with its local X axis spanning p1 -> p2."""
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    # All authored diagonal braces lie in the XZ planes of the side frames.
    pitch = math.atan2(-dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(xyz=((x1 + x2) * 0.5, (y1 + y2) * 0.5, (z1 + z2) * 0.5), rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_undershot_waterwheel")

    galvanized = Material("hot_dip_galvanized_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    stainless = Material("brushed_stainless_fasteners", rgba=(0.82, 0.84, 0.82, 1.0))
    dark_rubber = Material("black_epdm_seal", rgba=(0.01, 0.012, 0.012, 1.0))
    treated_wood = Material("sealed_oak_paddles", rgba=(0.54, 0.34, 0.18, 1.0))
    dark_frame = Material("powder_coated_side_frames", rgba=(0.10, 0.13, 0.14, 1.0))
    concrete = Material("cast_concrete_race", rgba=(0.43, 0.43, 0.40, 1.0))
    water_blue = Material("shallow_flow_channel", rgba=(0.10, 0.34, 0.62, 0.72))

    rim_mesh = _annulus_mesh("wheel_side_rim", outer_radius=0.605, inner_radius=0.535, thickness=0.040)
    hub_mesh = _annulus_mesh("sealed_hub_face", outer_radius=0.150, inner_radius=0.055, thickness=0.030)
    bearing_mesh = _annulus_mesh("bearing_flange_ring", outer_radius=0.165, inner_radius=0.070, thickness=0.080)
    seal_mesh = _annulus_mesh("epdm_lip_seal_ring", outer_radius=0.095, inner_radius=0.052, thickness=0.014)

    frame = model.part("frame")

    # Concrete undershot race and galvanized skid base.  The channel sits below
    # the lower paddle sweep, making the undershot purpose readable without
    # touching the rotating wheel.
    frame.visual(Box((1.88, 0.72, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=concrete, name="race_slab")
    frame.visual(Box((1.88, 0.055, 0.110)), origin=Origin(xyz=(0.0, 0.385, 0.065)), material=concrete, name="race_wall_0")
    frame.visual(Box((1.88, 0.055, 0.110)), origin=Origin(xyz=(0.0, -0.385, 0.065)), material=concrete, name="race_wall_1")
    frame.visual(Box((1.46, 0.58, 0.018)), origin=Origin(xyz=(0.03, 0.0, 0.084)), material=water_blue, name="low_water_surface")

    for y, idx in ((0.48, 0), (-0.48, 1)):
        frame.visual(Box((1.78, 0.125, 0.100)), origin=Origin(xyz=(0.0, y, 0.105)), material=dark_frame, name=f"base_skid_{idx}")
    for x, idx in ((0.80, 0), (-0.80, 1)):
        frame.visual(Box((0.145, 1.06, 0.110)), origin=Origin(xyz=(x, 0.0, 0.110)), material=dark_frame, name=f"base_cross_tie_{idx}")

    # Two side frames carry the centered bearing housings.  Pedestals, posts,
    # top ties, and diagonal braces overlap at joints so the support path is
    # physically continuous and fabrication-practical.
    for y, side in (
        (0.48, "side_0"),
        (-0.48, "side_1"),
    ):
        frame.visual(Box((0.145, 0.105, 1.300)), origin=Origin(xyz=(0.64, y, 0.755)), material=dark_frame, name=f"{side}_post_0")
        frame.visual(Box((0.145, 0.105, 1.300)), origin=Origin(xyz=(-0.64, y, 0.755)), material=dark_frame, name=f"{side}_post_1")
        frame.visual(Box((1.40, 0.105, 0.095)), origin=Origin(xyz=(0.0, y, 1.415)), material=dark_frame, name=f"{side}_top_tie")
        frame.visual(Box((0.320, 0.135, 0.585)), origin=Origin(xyz=(0.0, y, 0.415)), material=dark_frame, name=f"{side}_bearing_pedestal")

        _add_bar_between(frame, f"{side}_brace_0", (0.60, y, 0.215), (0.115, y, 0.725), 0.060, dark_frame)
        _add_bar_between(frame, f"{side}_brace_1", (-0.60, y, 0.215), (-0.115, y, 0.725), 0.060, dark_frame)
        _add_bar_between(frame, f"{side}_brace_2", (0.60, y, 1.330), (0.135, y, 0.925), 0.050, dark_frame)
        _add_bar_between(frame, f"{side}_brace_3", (-0.60, y, 1.330), (-0.135, y, 0.925), 0.050, dark_frame)

        frame.visual(
            Box((0.430, 0.170, 0.028)),
            origin=Origin(xyz=(0.0, y, 1.020), rpy=(0.0, -0.18, 0.0)),
            material=galvanized,
            name=f"{side}_drip_cap",
        )
        frame.visual(Box((0.030, 0.175, 0.070)), origin=Origin(xyz=(0.235, y, 0.982)), material=galvanized, name=f"{side}_drip_lip_0")
        frame.visual(Box((0.030, 0.175, 0.070)), origin=Origin(xyz=(-0.235, y, 0.982)), material=galvanized, name=f"{side}_drip_lip_1")

        # Stainless flange bolts on each bearing face.
        for j, (bx, bz) in enumerate(((0.118, 0.118), (-0.118, 0.118), (0.118, -0.118), (-0.118, -0.118))):
            frame.visual(
                Cylinder(radius=0.014, length=0.020),
                origin=Origin(xyz=(bx, y + math.copysign(0.042, y), 0.820 + bz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=stainless,
                name=f"{side}_flange_bolt_{j}",
            )

    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, 0.48, 0.820), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="side_0_bearing_flange",
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, -0.48, 0.820), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="side_1_bearing_flange",
    )
    # Stationary black lip seals are clearanced around the bore but intentionally
    # brush the rotating axle to read as weatherproof sealed interfaces.
    frame.visual(
        seal_mesh,
        origin=Origin(xyz=(0.0, 0.433, 0.820), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="side_0_lip_seal",
    )
    frame.visual(
        seal_mesh,
        origin=Origin(xyz=(0.0, -0.433, 0.820), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="side_1_lip_seal",
    )

    # Anchor plates and bolts tie the side frames down to the concrete race.
    for idx, (x, y) in enumerate(((0.64, 0.48), (-0.64, 0.48), (0.64, -0.48), (-0.64, -0.48))):
        frame.visual(Box((0.220, 0.155, 0.014)), origin=Origin(xyz=(x, y, 0.163)), material=galvanized, name=f"anchor_plate_{idx}")
        for j, ox in enumerate((-0.065, 0.065)):
            frame.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(x + ox, y, 0.175)),
                material=stainless,
                name=f"anchor_bolt_{idx}_{j}",
            )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.045, length=1.080),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.108, length=0.540),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hub_shell",
    )
    for y, side in ((0.250, "side_0"), (-0.250, "side_1")):
        wheel.visual(rim_mesh, origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=galvanized, name=f"{side}_rim")
        wheel.visual(hub_mesh, origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=galvanized, name=f"{side}_hub_plate")
        for i in range(8):
            a = i * math.tau / 8.0
            r_mid = 0.335
            wheel.visual(
                Box((0.430, 0.045, 0.050)),
                origin=Origin(xyz=(r_mid * math.cos(a), y, r_mid * math.sin(a)), rpy=(0.0, -a, 0.0)),
                material=galvanized,
                name=f"{side}_spoke_{i}",
            )

    # Undershot paddle boards: broad EPDM-sealed planks spanning the wheel
    # width, with stainless straps tying them to both side rims.
    for i in range(12):
        a = i * math.tau / 12.0
        x = 0.615 * math.cos(a)
        z = 0.615 * math.sin(a)
        wheel.visual(
            Box((0.040, 0.550, 0.165)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0 - a, 0.0)),
            material=treated_wood,
            name=f"paddle_{i}",
        )
        strap_r = 0.542
        sx = strap_r * math.cos(a)
        sz = strap_r * math.sin(a)
        wheel.visual(
            Box((0.030, 0.575, 0.026)),
            origin=Origin(xyz=(sx, 0.0, sz), rpy=(0.0, math.pi / 2.0 - a, 0.0)),
            material=stainless,
            name=f"paddle_strap_{i}",
        )

    # Outboard retained nuts and rubber thrower rings keep rain and spray away
    # from the bearing faces while staying outside the stationary flanges.
    for y, side in ((0.545, "side_0"), (-0.545, "side_1")):
        wheel.visual(
            Cylinder(radius=0.063, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"{side}_axle_nut",
        )
        wheel.visual(
            _annulus_mesh(f"{side}_rotary_thrower", outer_radius=0.078, inner_radius=0.046, thickness=0.016),
            origin=Origin(xyz=(0.0, y - math.copysign(0.010, y), 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_rubber,
            name=f"{side}_thrower_ring",
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="side_0_bearing_flange",
        elem_b="axle",
        reason="The centered axle is intentionally captured through the sealed side-bearing proxy.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="side_1_bearing_flange",
        elem_b="axle",
        reason="The centered axle is intentionally captured through the sealed side-bearing proxy.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="side_0_lip_seal",
        elem_b="axle",
        reason="The EPDM lip seal intentionally lightly compresses against the rotating axle.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="side_1_lip_seal",
        elem_b="axle",
        reason="The EPDM lip seal intentionally lightly compresses against the rotating axle.",
    )

    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="side_0_bearing_flange",
        margin=0.0,
        name="axle centered in first side bearing envelope",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="side_1_bearing_flange",
        margin=0.0,
        name="axle centered in second side bearing envelope",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="side_0_bearing_flange",
        min_overlap=0.065,
        name="axle passes through first side bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="side_1_bearing_flange",
        min_overlap=0.065,
        name="axle passes through second side bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="side_0_lip_seal",
        min_overlap=0.010,
        name="first lip seal bears on axle",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="side_1_lip_seal",
        min_overlap=0.010,
        name="second lip seal bears on axle",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(wheel)
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            inner_elem="axle",
            outer_elem="side_0_bearing_flange",
            margin=0.0,
            name="rotated axle remains centered in first side bearing",
        )
    ctx.check(
        "wheel rotates about fixed centered axle",
        rest_pos is not None
        and rotated_pos is not None
        and all(abs(rest_pos[i] - rotated_pos[i]) < 1e-9 for i in range(3)),
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
