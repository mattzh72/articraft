from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _tube_visual(part, name: str, points, radius: float, material: Material, *, segments: int = 14) -> None:
    """Add one capped tubular member following a short centerline."""
    geom = wire_from_points(
        points,
        radius=radius,
        radial_segments=segments,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.025,
    )
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transportable_ferris_wheel")

    steel = model.material("painted_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.10, 1.0))
    red = model.material("outrigger_red", rgba=(0.82, 0.08, 0.05, 1.0))
    blue = model.material("cabin_blue", rgba=(0.08, 0.25, 0.78, 1.0))
    orange = model.material("cabin_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    cream = model.material("cabin_cream", rgba=(0.95, 0.88, 0.68, 1.0))
    black = model.material("rubber_black", rgba=(0.015, 0.014, 0.012, 1.0))
    silver = model.material("galvanized_metal", rgba=(0.72, 0.75, 0.76, 1.0))

    base = model.part("base")
    base.visual(Box((1.75, 0.48, 0.12)), origin=Origin(xyz=(-0.10, 0.0, 0.14)), material=steel, name="deck")
    base.visual(Box((1.90, 0.10, 0.10)), origin=Origin(xyz=(-0.12, 0.0, 0.08)), material=steel, name="center_beam")
    base.visual(Box((1.55, 0.08, 0.07)), origin=Origin(xyz=(-0.18, 0.22, 0.21)), material=steel, name="side_rail_0")
    base.visual(Box((1.55, 0.08, 0.07)), origin=Origin(xyz=(-0.18, -0.22, 0.21)), material=steel, name="side_rail_1")

    # Trailer tongue and running gear make the lower assembly read as a towable base.
    _tube_visual(base, "tongue_0", [(0.70, 0.18, 0.16), (1.42, 0.0, 0.16)], 0.022, steel)
    _tube_visual(base, "tongue_1", [(0.70, -0.18, 0.16), (1.42, 0.0, 0.16)], 0.022, steel)
    base.visual(Box((0.18, 0.12, 0.07)), origin=Origin(xyz=(1.51, 0.0, 0.16)), material=silver, name="tow_coupler")
    base.visual(Cylinder(radius=0.024, length=0.84), origin=Origin(xyz=(-0.52, 0.0, 0.13), rpy=(math.pi / 2, 0.0, 0.0)), material=silver, name="road_axle")
    for y, suffix in ((0.43, "0"), (-0.43, "1")):
        base.visual(Cylinder(radius=0.155, length=0.075), origin=Origin(xyz=(-0.52, y, 0.13), rpy=(math.pi / 2, 0.0, 0.0)), material=black, name=f"road_tire_{suffix}")
        base.visual(Cylinder(radius=0.065, length=0.082), origin=Origin(xyz=(-0.52, y, 0.13), rpy=(math.pi / 2, 0.0, 0.0)), material=silver, name=f"road_hub_{suffix}")

    # Two compact A-frame towers straddle the wheel and carry the horizontal axle.
    axle_height = 1.20
    tower_top_z = axle_height - 0.07
    tower_y = 0.18
    for y, suffix in ((tower_y, "0"), (-tower_y, "1")):
        _tube_visual(base, f"tower_front_{suffix}", [(-0.55, y, 0.18), (0.0, y, tower_top_z)], 0.025, steel)
        _tube_visual(base, f"tower_rear_{suffix}", [(0.48, y, 0.18), (0.0, y, tower_top_z)], 0.025, steel)
        _tube_visual(base, f"tower_cross_{suffix}", [(-0.42, y, 0.42), (0.34, y, 0.78)], 0.014, silver)
        base.visual(Box((0.10, 0.08, 0.10)), origin=Origin(xyz=(0.0, y, axle_height - 0.10)), material=steel, name=f"bearing_post_{suffix}")
        base.visual(Box((0.15, 0.08, 0.10)), origin=Origin(xyz=(0.0, y, axle_height)), material=silver, name=f"axle_bearing_{suffix}")
        _tube_visual(base, f"upper_post_{suffix}", [(0.0, y, axle_height + 0.05), (0.0, y, axle_height + 0.78)], 0.018, silver)
    _tube_visual(base, "top_spreader", [(0.0, -tower_y, axle_height + 0.78), (0.0, tower_y, axle_height + 0.78)], 0.020, silver)
    _tube_visual(base, "base_spreader", [(-0.52, -tower_y, 0.22), (-0.52, tower_y, 0.22)], 0.018, steel)

    # Vertical hinge sockets for the two fold-out stabilizer outriggers.
    for y, suffix in ((0.30, "0"), (-0.30, "1")):
        base.visual(Cylinder(radius=0.038, length=0.16), origin=Origin(xyz=(-0.72, y, 0.17)), material=silver, name=f"outrigger_socket_{suffix}")
        base.visual(Box((0.13, 0.035, 0.08)), origin=Origin(xyz=(-0.72, y * 0.817, 0.17)), material=steel, name=f"socket_mount_{suffix}")

    wheel = model.part("wheel")
    rim_radius = 0.72
    wheel_side_y = 0.08
    for y, suffix in ((wheel_side_y, "0"), (-wheel_side_y, "1")):
        rim = TorusGeometry(radius=rim_radius, tube=0.020, radial_segments=28, tubular_segments=80)
        rim.rotate_x(math.pi / 2).translate(0.0, y, 0.0)
        wheel.visual(mesh_from_geometry(rim, f"rim_{suffix}"), material=yellow, name=f"rim_{suffix}")
        for i in range(12):
            angle = i * math.tau / 12.0
            end = (math.cos(angle) * (rim_radius - 0.020), y, math.sin(angle) * (rim_radius - 0.020))
            _tube_visual(wheel, f"spoke_{suffix}_{i}", [(0.0, y, 0.0), end], 0.0075, yellow, segments=10)

    for i in range(12):
        if i % 2 == 1:
            continue
        angle = i * math.tau / 12.0
        x = math.cos(angle) * rim_radius
        z = math.sin(angle) * rim_radius
        _tube_visual(wheel, f"rim_tie_{i}", [(x, -wheel_side_y, z), (x, wheel_side_y, z)], 0.007, yellow, segments=10)
    wheel.visual(Cylinder(radius=0.055, length=0.28), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=silver, name="hub")
    wheel.visual(Cylinder(radius=0.032, length=0.46), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="axle_stub")

    model.articulation(
        "base_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )

    gondola_radius = 0.52
    cabin_materials = [blue, orange, cream, blue, orange, cream]
    for i in range(6):
        angle = math.pi / 2.0 + i * math.tau / 6.0
        x = math.cos(angle) * gondola_radius
        z = math.sin(angle) * gondola_radius
        gondola = model.part(f"gondola_{i}")
        gondola.visual(Cylinder(radius=0.012, length=0.148), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=silver, name="pivot_bar")
        gondola.visual(Box((0.16, 0.020, 0.024)), origin=Origin(xyz=(0.0, 0.0, -0.006)), material=silver, name="top_yoke")
        gondola.visual(Box((0.030, 0.018, 0.18)), origin=Origin(xyz=(-0.065, 0.0, -0.090)), material=silver, name="hanger_0")
        gondola.visual(Box((0.030, 0.018, 0.18)), origin=Origin(xyz=(0.065, 0.0, -0.090)), material=silver, name="hanger_1")
        gondola.visual(Box((0.19, 0.095, 0.10)), origin=Origin(xyz=(0.0, 0.0, -0.205)), material=cabin_materials[i], name="basket_shell")
        gondola.visual(Box((0.17, 0.105, 0.035)), origin=Origin(xyz=(0.0, 0.0, -0.135)), material=cream, name="seat_rail")
        model.articulation(
            f"wheel_to_gondola_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.5),
        )

    for suffix, side_y, yaw, lower, upper in (
        ("0", 0.30, math.pi / 2.0, -1.35, 0.0),
        ("1", -0.30, -math.pi / 2.0, 0.0, 1.35),
    ):
        outrigger = model.part(f"outrigger_{suffix}")
        outrigger.visual(Cylinder(radius=0.036, length=0.15), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=silver, name="hinge_barrel")
        outrigger.visual(Box((0.055, 0.040, 0.045)), origin=Origin(xyz=(0.055, 0.0, -0.005)), material=silver, name="hinge_lug")
        _tube_visual(outrigger, "brace", [(0.060, 0.0, 0.0), (0.56, 0.0, -0.070)], 0.026, red)
        outrigger.visual(Box((0.20, 0.12, 0.035)), origin=Origin(xyz=(0.62, 0.0, -0.090)), material=silver, name="foot_pad")
        outrigger.visual(Cylinder(radius=0.020, length=0.11), origin=Origin(xyz=(0.56, 0.0, -0.055)), material=red, name="jack_screw")
        model.articulation(
            f"base_to_outrigger_{suffix}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=outrigger,
            origin=Origin(xyz=(-0.72, side_y, 0.17), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    base = object_model.get_part("base")
    wheel_joint = object_model.get_articulation("base_to_wheel")
    outrigger_0 = object_model.get_part("outrigger_0")
    outrigger_1 = object_model.get_part("outrigger_1")
    out_joint_0 = object_model.get_articulation("base_to_outrigger_0")
    out_joint_1 = object_model.get_articulation("base_to_outrigger_1")

    ctx.check(
        "wheel has continuous horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.expect_gap(
        object_model.get_part("gondola_3"),
        base,
        axis="z",
        min_gap=0.08,
        positive_elem="basket_shell",
        negative_elem="deck",
        name="lowest gondola clears trailer deck",
    )
    ctx.expect_overlap(
        wheel,
        base,
        axes="xy",
        min_overlap=0.03,
        elem_a="hub",
        elem_b="top_spreader",
        name="wheel hub is centered in tower spreader",
    )

    for suffix in ("0", "1"):
        outrigger = object_model.get_part(f"outrigger_{suffix}")
        ctx.allow_overlap(
            base,
            outrigger,
            elem_a=f"outrigger_socket_{suffix}",
            elem_b="hinge_barrel",
            reason="The outrigger hinge barrel is intentionally captured inside the base socket.",
        )
        ctx.allow_overlap(
            base,
            outrigger,
            elem_a=f"outrigger_socket_{suffix}",
            elem_b="hinge_lug",
            reason="The welded hinge lug locally nests into the socket mouth to support the folding brace.",
        )
        ctx.expect_contact(
            base,
            outrigger,
            elem_a=f"outrigger_socket_{suffix}",
            elem_b="hinge_lug",
            name=f"outrigger {suffix} hinge lug is seated at socket",
        )
        ctx.expect_within(
            outrigger,
            base,
            axes="xy",
            inner_elem="hinge_barrel",
            outer_elem=f"outrigger_socket_{suffix}",
            margin=0.002,
            name=f"outrigger {suffix} hinge barrel stays in socket",
        )
        ctx.expect_overlap(
            outrigger,
            base,
            axes="z",
            elem_a="hinge_barrel",
            elem_b=f"outrigger_socket_{suffix}",
            min_overlap=0.13,
            name=f"outrigger {suffix} hinge barrel retained vertically",
        )

    for suffix in ("0", "1"):
        ctx.allow_overlap(
            base,
            wheel,
            elem_a=f"axle_bearing_{suffix}",
            elem_b="axle_stub",
            reason="The wheel axle is intentionally seated through the tower bearing block.",
        )
        ctx.expect_within(
            wheel,
            base,
            axes="xz",
            inner_elem="axle_stub",
            outer_elem=f"axle_bearing_{suffix}",
            margin=0.0,
            name=f"wheel axle centered in bearing {suffix}",
        )
        ctx.expect_overlap(
            wheel,
            base,
            axes="y",
            elem_a="axle_stub",
            elem_b=f"axle_bearing_{suffix}",
            min_overlap=0.07,
            name=f"wheel axle passes through bearing {suffix}",
        )

    for i in range(6):
        joint = object_model.get_articulation(f"wheel_to_gondola_{i}")
        ctx.check(
            f"gondola {i} has horizontal pivot",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    deployed_0 = ctx.part_element_world_aabb(outrigger_0, elem="foot_pad")
    deployed_1 = ctx.part_element_world_aabb(outrigger_1, elem="foot_pad")
    with ctx.pose({out_joint_0: -1.2, out_joint_1: 1.2}):
        folded_0 = ctx.part_element_world_aabb(outrigger_0, elem="foot_pad")
        folded_1 = ctx.part_element_world_aabb(outrigger_1, elem="foot_pad")
    ctx.check(
        "outriggers fold inward from deployed stance",
        deployed_0 is not None
        and deployed_1 is not None
        and folded_0 is not None
        and folded_1 is not None
        and deployed_0[1][1] > folded_0[1][1] + 0.20
        and deployed_1[0][1] < folded_1[0][1] - 0.20,
        details=f"deployed_0={deployed_0}, folded_0={folded_0}, deployed_1={deployed_1}, folded_1={folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
