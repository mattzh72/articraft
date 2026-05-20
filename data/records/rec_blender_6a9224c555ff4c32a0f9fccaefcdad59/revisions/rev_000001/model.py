from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    """A simple vertical ring, authored in meters."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _frustum_shell(
    outer_bottom: float,
    outer_top: float,
    inner_bottom: float,
    inner_top: float,
    height: float,
    z0: float,
) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(outer_bottom)
        .workplane(offset=height)
        .circle(outer_top)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .circle(inner_bottom)
        .workplane(offset=height)
        .circle(inner_top)
        .loft(combine=True)
    )
    return outer.cut(inner).translate((0.0, 0.0, z0))


def _jar_glass_shell() -> cq.Workplane:
    """Hollow glass jar with a broad round body, sloped shoulder, and narrow open neck."""
    bottom = cq.Workplane("XY").circle(0.098).extrude(0.012).translate((0.0, 0.0, 0.034))
    body_wall = _annular_cylinder(0.105, 0.098, 0.277, 0.045)
    shoulder = _frustum_shell(0.105, 0.055, 0.098, 0.047, 0.036, 0.320)
    neck = _annular_cylinder(0.055, 0.047, 0.038, 0.354)
    drinking_lip = _annular_cylinder(0.064, 0.047, 0.010, 0.388)
    return bottom.union(body_wall).union(shoulder).union(neck).union(drinking_lip).clean()


def _bayonet_collar() -> cq.Workplane:
    """Plastic annular jar collar, leaving clearance for the base drive coupler."""
    return _annular_cylinder(0.087, 0.040, 0.035, 0.0).clean()


def _neck_rim() -> cq.Workplane:
    return _annular_cylinder(0.066, 0.047, 0.012, 0.386).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_smoothie_blender")

    brushed_metal = model.material("brushed_metal", rgba=(0.60, 0.62, 0.64, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    glass = model.material("clear_green_glass", rgba=(0.72, 0.95, 0.93, 0.36))
    stainless = model.material("stainless_blade", rgba=(0.86, 0.88, 0.87, 1.0))
    accent = model.material("speed_mark_white", rgba=(0.92, 0.94, 0.93, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.30, 0.23, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=brushed_metal,
        name="base_body",
    )
    base.visual(
        Box((0.17, 0.006, 0.075)),
        origin=Origin(xyz=(0.0, -0.118, 0.083)),
        material=dark_plastic,
        name="front_panel",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=dark_plastic,
        name="base_coupling",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=black_rubber,
        name="drive_coupler",
    )
    for index, x in enumerate((-0.105, 0.105)):
        for y in (-0.075, 0.075):
            base.visual(
                Box((0.055, 0.030, 0.010)),
                origin=Origin(xyz=(x, y, 0.005)),
                material=black_rubber,
                name=f"foot_{index}_{0 if y < 0 else 1}",
            )

    dial = model.part("speed_dial")
    dial.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="dial_cap",
    )
    dial.visual(
        Box((0.006, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, -0.029, 0.016)),
        material=accent,
        name="dial_pointer",
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.121, 0.083)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=-2.35, upper=2.35),
    )

    jar = model.part("jar")
    jar.visual(
        mesh_from_cadquery(_bayonet_collar(), "jar_bayonet_collar"),
        material=dark_plastic,
        name="jar_collar",
    )
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0
        radius = 0.090
        jar.visual(
            Box((0.034, 0.014, 0.010)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.011),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=dark_plastic,
            name=f"bayonet_lug_{i}",
        )
    jar.visual(
        mesh_from_cadquery(_jar_glass_shell(), "round_narrow_neck_jar"),
        material=glass,
        name="glass_shell",
    )
    jar.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_plastic,
        name="blade_bearing",
    )
    jar.visual(
        mesh_from_cadquery(_neck_rim(), "jar_neck_rim"),
        material=dark_plastic,
        name="neck_rim",
    )
    for side, x in enumerate((-0.036, 0.036)):
        jar.visual(
            Box((0.020, 0.028, 0.008)),
            origin=Origin(xyz=(x, 0.068, 0.398)),
            material=dark_plastic,
            name=f"hinge_leaf_{side}",
        )
        jar.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(x, 0.074, 0.406), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=f"hinge_knuckle_{side}",
        )

    model.articulation(
        "base_to_jar_bayonet",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=7.5, velocity=1.0, lower=0.0, upper=0.45),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="blade_hub",
    )
    for i in range(4):
        yaw = i * math.pi / 2.0 + (0.28 if i % 2 else -0.28)
        blade.visual(
            Box((0.070, 0.017, 0.003)),
            origin=Origin(
                xyz=(0.041 * math.cos(yaw), 0.041 * math.sin(yaw), 0.002),
                rpy=(0.12 if i % 2 else -0.12, 0.0, yaw),
            ),
            material=stainless,
            name=f"blade_wing_{i}",
        )
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=80.0),
    )

    lid = model.part("pour_lid")
    lid.visual(
        Cylinder(radius=0.060, length=0.016),
        origin=Origin(xyz=(0.0, -0.074, 0.008)),
        material=dark_plastic,
        name="lid_cap",
    )
    lid.visual(
        Box((0.055, 0.021, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, 0.008)),
        material=dark_plastic,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="lid_knuckle",
    )
    lid.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.132, 0.014)),
        material=dark_plastic,
        name="pour_spout",
    )
    model.articulation(
        "jar_to_pour_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.074, 0.398)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade = object_model.get_part("blade")
    lid = object_model.get_part("pour_lid")
    dial = object_model.get_part("speed_dial")
    bayonet = object_model.get_articulation("base_to_jar_bayonet")
    lid_hinge = object_model.get_articulation("jar_to_pour_lid")
    blade_spin = object_model.get_articulation("jar_to_blade")
    dial_turn = object_model.get_articulation("base_to_speed_dial")

    ctx.check(
        "primary mechanisms are articulated",
        bayonet is not None and lid_hinge is not None and blade_spin is not None,
        details="Expected bayonet twist, pour-lid hinge, and continuous blade spin articulations.",
    )
    ctx.check(
        "blade uses continuous rotation",
        getattr(blade_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"blade joint type={getattr(blade_spin, 'articulation_type', None)}",
    )
    ctx.check(
        "bayonet twist has short locking travel",
        bayonet.motion_limits is not None
        and bayonet.motion_limits.lower == 0.0
        and 0.35 <= bayonet.motion_limits.upper <= 0.55,
        details=f"limits={bayonet.motion_limits}",
    )

    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem="jar_collar",
        negative_elem="base_coupling",
        max_gap=0.0015,
        max_penetration=0.000001,
        name="jar collar seats on base coupling",
    )
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        positive_elem="lid_cap",
        negative_elem="neck_rim",
        max_gap=0.0015,
        max_penetration=0.000001,
        name="closed pour lid rests on neck rim",
    )
    ctx.expect_gap(
        base,
        dial,
        axis="y",
        positive_elem="front_panel",
        negative_elem="dial_cap",
        max_gap=0.0015,
        max_penetration=0.000001,
        name="speed dial sits proud of front panel",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        inner_elem="blade_hub",
        outer_elem="glass_shell",
        margin=0.0,
        name="blade hub is centered inside jar footprint",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb is not None else None

    lug_rest = aabb_center(ctx.part_element_world_aabb(jar, elem="bayonet_lug_0"))
    with ctx.pose({bayonet: bayonet.motion_limits.upper}):
        lug_twist = aabb_center(ctx.part_element_world_aabb(jar, elem="bayonet_lug_0"))
        ctx.expect_gap(
            jar,
            base,
            axis="z",
            positive_elem="jar_collar",
            negative_elem="base_coupling",
            max_gap=0.0015,
            max_penetration=0.000001,
            name="twisted jar remains seated",
        )
    ctx.check(
        "bayonet lug sweeps around coupling",
        lug_rest is not None
        and lug_twist is not None
        and math.hypot(lug_twist[0] - lug_rest[0], lug_twist[1] - lug_rest[1]) > 0.025,
        details=f"rest={lug_rest}, twist={lug_twist}",
    )

    lid_closed = aabb_center(ctx.part_element_world_aabb(lid, elem="lid_cap"))
    with ctx.pose({lid_hinge: 1.05}):
        lid_open = aabb_center(ctx.part_element_world_aabb(lid, elem="lid_cap"))
    ctx.check(
        "pour lid opens upward about edge hinge",
        lid_closed is not None and lid_open is not None and lid_open[2] > lid_closed[2] + 0.035,
        details=f"closed={lid_closed}, opened={lid_open}",
    )

    blade_origin = ctx.part_world_position(blade)
    with ctx.pose({blade_spin: 2.0 * math.pi, dial_turn: 1.0}):
        spun_origin = ctx.part_world_position(blade)
    ctx.check(
        "blade spin does not translate rotor",
        blade_origin is not None
        and spun_origin is not None
        and max(abs(blade_origin[i] - spun_origin[i]) for i in range(3)) < 1e-6,
        details=f"rest={blade_origin}, spun={spun_origin}",
    )

    return ctx.report()


object_model = build_object_model()
