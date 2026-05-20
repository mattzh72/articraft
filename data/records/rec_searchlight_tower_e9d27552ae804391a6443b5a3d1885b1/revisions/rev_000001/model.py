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


def _cylinder_origin_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
) -> tuple[float, Origin]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("Cylinder endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
        rpy=(0.0, pitch, yaw),
    )
    return length, origin


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(shape, name, tolerance=0.001, angular_tolerance=0.08)


def _add_cylinder_between(part, p0, p1, radius, material, name):
    length, origin = _cylinder_origin_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_searchlight_tower")

    galvanized = model.material("hot_dip_galvanized_steel", rgba=(0.62, 0.66, 0.65, 1.0))
    stainless = model.material("brushed_stainless_hardware", rgba=(0.82, 0.84, 0.82, 1.0))
    dark_coat = model.material("black_powder_coat", rgba=(0.02, 0.025, 0.025, 1.0))
    rubber = model.material("black_epdm_gaskets", rgba=(0.005, 0.005, 0.004, 1.0))
    concrete = model.material("cast_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    glass = model.material("sealed_warm_glass", rgba=(0.80, 0.92, 1.0, 0.42))
    reflector = model.material("warm_reflector", rgba=(1.0, 0.78, 0.34, 1.0))

    tower = model.part("tower")
    tower.visual(Box((1.20, 1.20, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=concrete, name="concrete_pad")
    tower.visual(Cylinder(radius=0.39, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.197)), material=galvanized, name="base_plate")
    tower.visual(Cylinder(radius=0.078, length=2.50), origin=Origin(xyz=(0.0, 0.0, 1.46)), material=galvanized, name="mast_tube")
    tower.visual(Cylinder(radius=0.145, length=0.16), origin=Origin(xyz=(0.0, 0.0, 2.62)), material=galvanized, name="top_collar")
    tower.visual(Cylinder(radius=0.165, length=0.060), origin=Origin(xyz=(0.0, 0.0, 2.72)), material=galvanized, name="pan_seat")
    tower.visual(Cylinder(radius=0.205, length=0.024), origin=Origin(xyz=(0.0, 0.0, 2.655)), material=galvanized, name="fixed_drip_flange")

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            corner = f"{'pos' if sx > 0 else 'neg'}_{'pos' if sy > 0 else 'neg'}"
            tower.visual(
                Box((0.14, 0.14, 0.024)),
                origin=Origin(xyz=(0.42 * sx, 0.42 * sy, 0.185)),
                material=galvanized,
                name=f"leg_foot_{corner}",
            )
            _add_cylinder_between(tower, (0.42 * sx, 0.42 * sy, 0.190), (0.18 * sx, 0.18 * sy, 2.52), 0.023, galvanized, f"tower_leg_{corner}")
            _add_cylinder_between(tower, (0.34 * sx, 0.34 * sy, 0.46), (-0.18 * sx, 0.18 * sy, 1.42), 0.014, galvanized, f"cross_brace_{corner}_lower")
            _add_cylinder_between(tower, (0.20 * sx, 0.20 * sy, 1.42), (-0.16 * sx, 0.16 * sy, 2.32), 0.014, galvanized, f"cross_brace_{corner}_upper")

    for z, radius, name in ((0.44, 0.47, "lower_tie_ring"), (1.42, 0.34, "middle_tie_ring"), (2.33, 0.25, "upper_tie_ring")):
        tower.visual(Cylinder(radius=radius, length=0.030), origin=Origin(xyz=(0.0, 0.0, z)), material=galvanized, name=name)

    for x in (-0.27, 0.27):
        for y in (-0.27, 0.27):
            suffix = f"{'pos' if x > 0 else 'neg'}_{'pos' if y > 0 else 'neg'}"
            tower.visual(Cylinder(radius=0.030, length=0.010), origin=Origin(xyz=(x, y, 0.225)), material=stainless, name=f"anchor_washer_{suffix}")
            tower.visual(Cylinder(radius=0.016, length=0.070), origin=Origin(xyz=(x, y, 0.247)), material=stainless, name=f"anchor_stud_{suffix}")

    yoke = model.part("yoke")
    yoke.visual(Cylinder(radius=0.180, length=0.080), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=galvanized, name="turntable_bearing")
    yoke.visual(Cylinder(radius=0.245, length=0.036), origin=Origin(xyz=(0.0, 0.0, 0.115)), material=galvanized, name="rotating_drip_skirt")
    yoke.visual(Cylinder(radius=0.080, length=0.430), origin=Origin(xyz=(0.0, 0.0, 0.285)), material=galvanized, name="pan_pedestal")
    yoke.visual(Box((0.32, 0.86, 0.085)), origin=Origin(xyz=(0.0, 0.0, 0.540)), material=galvanized, name="yoke_crosshead")
    yoke.visual(Box((0.16, 0.090, 0.760)), origin=Origin(xyz=(0.0, 0.390, 0.900)), material=galvanized, name="yoke_arm_pos")
    yoke.visual(Box((0.240, 0.100, 0.205)), origin=Origin(xyz=(0.0, 0.335, 0.900)), material=galvanized, name="tilt_bearing_pos")
    yoke.visual(Cylinder(radius=0.105, length=0.026), origin=Origin(xyz=(0.0, 0.451, 0.900), rpy=(math.pi / 2.0, 0.0, 0.0)), material=stainless, name="weather_cap_pos")
    yoke.visual(Cylinder(radius=0.070, length=0.020), origin=Origin(xyz=(0.0, 0.287, 0.900), rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber, name="bearing_seal_pos")
    yoke.visual(Box((0.16, 0.090, 0.760)), origin=Origin(xyz=(0.0, -0.390, 0.900)), material=galvanized, name="yoke_arm_neg")
    yoke.visual(Box((0.240, 0.100, 0.205)), origin=Origin(xyz=(0.0, -0.335, 0.900)), material=galvanized, name="tilt_bearing_neg")
    yoke.visual(Cylinder(radius=0.105, length=0.026), origin=Origin(xyz=(0.0, -0.451, 0.900), rpy=(math.pi / 2.0, 0.0, 0.0)), material=stainless, name="weather_cap_neg")
    yoke.visual(Cylinder(radius=0.070, length=0.020), origin=Origin(xyz=(0.0, -0.287, 0.900), rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber, name="bearing_seal_neg")
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        for bx in (-0.070, 0.070):
            for bz in (0.835, 0.965):
                yoke.visual(
                    Cylinder(radius=0.014, length=0.018),
                    origin=Origin(xyz=(bx, sign * 0.444, bz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=stainless,
                    name=f"bearing_bolt_{label}_{'pos' if bx > 0 else 'neg'}_{'top' if bz > 0.9 else 'bottom'}",
                )

    head = model.part("lamp_head")
    head.visual(_tube_mesh(0.245, 0.192, 0.600, "lamp_hollow_shell"), origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_coat, name="lamp_hollow_shell")
    head.visual(_tube_mesh(0.265, 0.184, 0.060, "front_bezel_ring"), origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_coat, name="front_bezel")
    head.visual(_tube_mesh(0.198, 0.170, 0.014, "front_gasket_ring"), origin=Origin(xyz=(0.424, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=rubber, name="front_gasket")
    head.visual(Cylinder(radius=0.170, length=0.016), origin=Origin(xyz=(0.430, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=glass, name="front_lens")
    head.visual(Cylinder(radius=0.188, length=0.012), origin=Origin(xyz=(0.372, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=reflector, name="inner_reflector")
    head.visual(Cylinder(radius=0.235, length=0.040), origin=Origin(xyz=(-0.250, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_coat, name="rear_cap")
    head.visual(_tube_mesh(0.238, 0.200, 0.018, "rear_gasket_ring"), origin=Origin(xyz=(-0.222, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=rubber, name="rear_gasket")
    head.visual(Cylinder(radius=0.034, length=0.090), origin=Origin(xyz=(-0.310, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=rubber, name="rear_cable_gland")
    head.visual(Box((0.205, 0.585, 0.035)), origin=Origin(xyz=(0.485, 0.0, 0.266)), material=dark_coat, name="drip_hood_top")
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        head.visual(Box((0.205, 0.035, 0.125)), origin=Origin(xyz=(0.485, sign * 0.275, 0.205)), material=dark_coat, name=f"drip_hood_side_{label}")
    head.visual(Cylinder(radius=0.055, length=0.570), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=stainless, name="trunnion_shaft")
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        head.visual(Cylinder(radius=0.083, length=0.022), origin=Origin(xyz=(0.0, sign * 0.248, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber, name=f"trunnion_boot_{label}")

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.40, lower=-0.55, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("lamp_head")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.check("pan and tilt joints are separated", pan is not None and tilt is not None)
    ctx.expect_gap(
        yoke,
        tower,
        axis="z",
        positive_elem="turntable_bearing",
        negative_elem="pan_seat",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable is seated on mast pan bearing",
    )
    ctx.expect_overlap(
        yoke,
        tower,
        axes="xy",
        elem_a="turntable_bearing",
        elem_b="pan_seat",
        min_overlap=0.12,
        name="pan bearing footprint overlaps mast seat",
    )
    ctx.expect_gap(
        yoke,
        head,
        axis="y",
        positive_elem="tilt_bearing_pos",
        negative_elem="trunnion_shaft",
        max_gap=0.002,
        max_penetration=0.0,
        name="positive trunnion end is carried by bearing block",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="y",
        positive_elem="trunnion_shaft",
        negative_elem="tilt_bearing_neg",
        max_gap=0.002,
        max_penetration=0.0,
        name="negative trunnion end is carried by bearing block",
    )
    ctx.allow_overlap(
        yoke,
        head,
        elem_a="bearing_seal_pos",
        elem_b="trunnion_shaft",
        reason="EPDM rotary seal is intentionally compressed around the stainless trunnion shaft.",
    )
    ctx.expect_gap(
        yoke,
        head,
        axis="y",
        positive_elem="bearing_seal_pos",
        negative_elem="trunnion_shaft",
        max_penetration=0.010,
        name="positive tilt seal has only local compression",
    )
    ctx.allow_overlap(
        yoke,
        head,
        elem_a="bearing_seal_neg",
        elem_b="trunnion_shaft",
        reason="EPDM rotary seal is intentionally compressed around the stainless trunnion shaft.",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="y",
        positive_elem="trunnion_shaft",
        negative_elem="bearing_seal_neg",
        max_penetration=0.010,
        name="negative tilt seal has only local compression",
    )

    rest_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt: 0.60}):
        raised_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "positive tilt raises the searchlight lens",
        rest_lens is not None
        and raised_lens is not None
        and ((raised_lens[0][2] + raised_lens[1][2]) * 0.5) > ((rest_lens[0][2] + rest_lens[1][2]) * 0.5) + 0.10,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    with ctx.pose({pan: 0.90}):
        swept_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "pan joint slews the head around the mast",
        swept_lens is not None and abs((swept_lens[0][1] + swept_lens[1][1]) * 0.5) > 0.25,
        details=f"swept_lens={swept_lens}",
    )

    return ctx.report()


object_model = build_object_model()
