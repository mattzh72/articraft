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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_between(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    """Return a cylinder and transform whose local +Z runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
    center = ((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5)
    return Cylinder(radius=0.018, length=length), Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _annular_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """CadQuery tube along local +Z with open bore and annular end faces."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solar_halpha_refractor_altaz")

    black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    red = model.material("halpha_red_anodized", rgba=(0.78, 0.08, 0.025, 1.0))
    gold = model.material("etalon_gold", rgba=(0.96, 0.62, 0.16, 1.0))
    silver = model.material("brushed_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    glass = model.material("deep_solar_glass", rgba=(0.05, 0.13, 0.18, 0.72))

    tripod = model.part("tripod_head")
    tripod.visual(Cylinder(radius=0.040, length=0.78), origin=Origin(xyz=(0.0, 0.0, 0.56)), material=black, name="center_column")
    tripod.visual(Cylinder(radius=0.125, length=0.080), origin=Origin(xyz=(0.0, 0.0, 0.960)), material=black, name="azimuth_base")
    tripod.visual(Cylinder(radius=0.090, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.235)), material=black, name="leg_crown")
    tripod.visual(Sphere(radius=0.055), origin=Origin(xyz=(0.0, 0.0, 0.270)), material=black, name="crown_boss")
    for i, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        hip = (0.070 * math.cos(angle), 0.070 * math.sin(angle), 0.250)
        foot = (0.620 * math.cos(angle), 0.620 * math.sin(angle), 0.040)
        leg_geom, leg_origin = _cylinder_between(hip, foot)
        tripod.visual(leg_geom, origin=leg_origin, material=black, name=f"tripod_leg_{i}")
        tripod.visual(
            Box((0.145, 0.050, 0.022)),
            origin=Origin(xyz=foot, rpy=(0.0, 0.0, angle)),
            material=black,
            name=f"rubber_foot_{i}",
        )

    arm = model.part("azimuth_arm")
    arm.visual(Cylinder(radius=0.108, length=0.052), origin=Origin(xyz=(0.0, 0.0, 0.026)), material=black, name="azimuth_cap")
    arm.visual(Cylinder(radius=0.045, length=0.280), origin=Origin(xyz=(0.0, 0.0, 0.166)), material=black, name="upright_post")
    arm.visual(Box((0.190, 0.240, 0.054)), origin=Origin(xyz=(0.070, 0.0, 0.295)), material=black, name="offset_bridge")
    arm.visual(Box((0.070, 0.030, 0.240)), origin=Origin(xyz=(0.140, 0.105, 0.295)), material=black, name="fork_cheek_0")
    arm.visual(Box((0.070, 0.030, 0.240)), origin=Origin(xyz=(0.140, -0.105, 0.295)), material=black, name="fork_cheek_1")
    arm.visual(Cylinder(radius=0.030, length=0.012), origin=Origin(xyz=(0.140, 0.126, 0.360), rpy=(math.pi / 2.0, 0.0, 0.0)), material=silver, name="altitude_bushing_0")
    arm.visual(Cylinder(radius=0.030, length=0.012), origin=Origin(xyz=(0.140, -0.126, 0.360), rpy=(math.pi / 2.0, 0.0, 0.0)), material=silver, name="altitude_bushing_1")

    telescope = model.part("telescope")
    telescope.visual(Cylinder(radius=0.045, length=0.530), origin=Origin(xyz=(-0.065, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)), material=red, name="main_tube")
    telescope.visual(Cylinder(radius=0.054, length=0.035), origin=Origin(xyz=(-0.345, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)), material=black, name="front_cell")
    telescope.visual(Cylinder(radius=0.044, length=0.006), origin=Origin(xyz=(-0.365, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)), material=glass, name="front_lens")
    telescope.visual(
        mesh_from_cadquery(_annular_tube(0.036, 0.025, 0.120), "etalon_filter_tube", tolerance=0.0008),
        origin=Origin(xyz=(0.200, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="etalon_filter",
    )
    telescope.visual(Cylinder(radius=0.041, length=0.018), origin=Origin(xyz=(0.194, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)), material=black, name="rear_retaining_ring")
    telescope.visual(Box((0.260, 0.014, 0.010)), origin=Origin(xyz=(0.330, 0.0, 0.107)), material=black, name="focuser_lower_rail")
    telescope.visual(Box((0.350, 0.040, 0.030)), origin=Origin(xyz=(-0.015, 0.0, 0.075)), material=black, name="dovetail_rail")
    telescope.visual(Box((0.090, 0.070, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=black, name="saddle_block")
    telescope.visual(Cylinder(radius=0.026, length=0.180), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=silver, name="altitude_shaft")

    draw_tube = model.part("draw_tube")
    draw_tube.visual(Cylinder(radius=0.018, length=0.260), origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=silver, name="sliding_tube")
    draw_tube.visual(Cylinder(radius=0.023, length=0.016), origin=Origin(xyz=(0.158, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=black, name="eyepiece_stop")

    eyepiece = model.part("eyepiece_tube")
    eyepiece.visual(
        mesh_from_cadquery(_annular_tube(0.024, 0.014, 0.105), "eyepiece_tube", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="eyepiece_barrel",
    )
    eyepiece.visual(
        mesh_from_cadquery(_annular_tube(0.031, 0.018, 0.026), "eyecup_ring", tolerance=0.0008),
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rubber_eyecup",
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=telescope,
        origin=Origin(xyz=(0.140, 0.0, 0.360)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-0.35, upper=1.15),
    )
    model.articulation(
        "focuser_draw",
        ArticulationType.PRISMATIC,
        parent=telescope,
        child=draw_tube,
        origin=Origin(xyz=(0.320, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.05, lower=0.0, upper=0.060),
    )
    model.articulation(
        "draw_to_eyepiece",
        ArticulationType.FIXED,
        parent=draw_tube,
        child=eyepiece,
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod_head")
    arm = object_model.get_part("azimuth_arm")
    telescope = object_model.get_part("telescope")
    draw_tube = object_model.get_part("draw_tube")
    eyepiece = object_model.get_part("eyepiece_tube")
    azimuth = object_model.get_articulation("azimuth_bearing")
    altitude = object_model.get_articulation("altitude_axis")
    focuser = object_model.get_articulation("focuser_draw")

    ctx.check("azimuth bearing is continuous", azimuth.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("altitude axis is revolute", altitude.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("focuser draw tube is prismatic", focuser.articulation_type == ArticulationType.PRISMATIC)

    with ctx.pose({azimuth: 0.0, altitude: 0.0, focuser: 0.0}):
        ctx.expect_gap(
            arm,
            tripod,
            axis="z",
            positive_elem="azimuth_cap",
            negative_elem="azimuth_base",
            max_gap=0.001,
            max_penetration=0.0,
            name="azimuth bearing cap seats on tripod head",
        )
        ctx.expect_gap(
            arm,
            telescope,
            axis="y",
            positive_elem="fork_cheek_0",
            negative_elem="altitude_shaft",
            max_gap=0.001,
            max_penetration=0.0,
            name="altitude shaft is captured between fork cheeks",
        )
        ctx.expect_within(
            draw_tube,
            telescope,
            axes="yz",
            inner_elem="sliding_tube",
            outer_elem="etalon_filter",
            margin=0.0,
            name="draw tube is centered in the etalon bore",
        )
        ctx.expect_overlap(
            draw_tube,
            telescope,
            axes="x",
            elem_a="sliding_tube",
            elem_b="etalon_filter",
            min_overlap=0.080,
            name="collapsed draw tube retains insertion",
        )
        ctx.expect_contact(
            draw_tube,
            telescope,
            elem_a="sliding_tube",
            elem_b="focuser_lower_rail",
            contact_tol=0.001,
            name="draw tube rides on focuser guide rail",
        )
        ctx.expect_contact(
            eyepiece,
            draw_tube,
            elem_a="eyepiece_barrel",
            elem_b="eyepiece_stop",
            contact_tol=0.002,
            name="eyepiece barrel is seated on draw tube stop",
        )

        rest_front = ctx.part_element_world_aabb(telescope, elem="front_lens")
        rest_draw = ctx.part_world_position(draw_tube)
        rest_scope = ctx.part_world_position(telescope)

    with ctx.pose({altitude: 0.80}):
        raised_front = ctx.part_element_world_aabb(telescope, elem="front_lens")

    with ctx.pose({focuser: 0.060}):
        extended_draw = ctx.part_world_position(draw_tube)
        ctx.expect_within(
            draw_tube,
            telescope,
            axes="yz",
            inner_elem="sliding_tube",
            outer_elem="etalon_filter",
            margin=0.0,
            name="extended draw tube stays centered in bore",
        )
        ctx.expect_overlap(
            draw_tube,
            telescope,
            axes="x",
            elem_a="sliding_tube",
            elem_b="etalon_filter",
            min_overlap=0.035,
            name="extended draw tube remains retained",
        )

    with ctx.pose({azimuth: math.pi / 2.0}):
        rotated_scope = ctx.part_world_position(telescope)

    def _aabb_mid_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_front_z = _aabb_mid_z(rest_front)
    raised_front_z = _aabb_mid_z(raised_front)
    ctx.check(
        "altitude joint raises the solar objective",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 0.15,
        details=f"rest_front_z={rest_front_z}, raised_front_z={raised_front_z}",
    )
    ctx.check(
        "focuser extends rearward along optical axis",
        rest_draw is not None and extended_draw is not None and extended_draw[0] > rest_draw[0] + 0.050,
        details=f"rest_draw={rest_draw}, extended_draw={extended_draw}",
    )
    ctx.check(
        "azimuth bearing swings the offset arm around tripod",
        rest_scope is not None
        and rotated_scope is not None
        and abs(rotated_scope[1] - rest_scope[0]) < 0.020
        and rotated_scope[1] > 0.100,
        details=f"rest_scope={rest_scope}, rotated_scope={rotated_scope}",
    )

    return ctx.report()


object_model = build_object_model()
