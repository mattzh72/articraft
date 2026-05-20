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
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_weapon_station")

    armor = Material("mat_olive_drab_armor", rgba=(0.23, 0.28, 0.18, 1.0))
    dark_armor = Material("mat_dark_olive", rgba=(0.13, 0.17, 0.12, 1.0))
    black = Material("mat_gun_black", rgba=(0.02, 0.022, 0.02, 1.0))
    steel = Material("mat_dark_steel", rgba=(0.36, 0.36, 0.34, 1.0))
    rubber = Material("mat_black_rubber", rgba=(0.01, 0.012, 0.011, 1.0))
    glass = Material("mat_sensor_glass", rgba=(0.04, 0.18, 0.24, 1.0))

    # Fixed compact pedestal and bearing race.  The azimuth joint is at the
    # top of this race, so the child turntable can sit on it without drifting.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.235, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_armor,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=armor,
        name="fixed_pedestal",
    )
    pedestal.visual(
        Cylinder(radius=0.180, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=steel,
        name="bearing_race",
    )

    # Rotating station body: bearing disk, deck, trunnion side supports,
    # front armor with hinge lugs, and a compact sensor box.
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.178, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="rotating_bearing",
    )
    turntable.visual(
        Box((0.420, 0.480, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=armor,
        name="upper_deck",
    )
    for idx, y in enumerate((-0.180, 0.180)):
        turntable.visual(
            Box((0.110, 0.050, 0.300)),
            origin=Origin(xyz=(0.020, y, 0.250)),
            material=armor,
            name=f"side_support_{idx}",
        )
        turntable.visual(
            Cylinder(radius=0.043, length=0.058),
            origin=Origin(xyz=(0.020, y, 0.300), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"trunnion_boss_{idx}",
        )
    turntable.visual(
        Box((0.040, 0.440, 0.140)),
        origin=Origin(xyz=(0.227, 0.0, 0.170)),
        material=armor,
        name="front_armor",
    )
    # Alternating hinge knuckles on the top edge of the front armor.  The
    # center gap is occupied by the flap's own barrel, so the moving flap stays
    # visually clipped to the fixed hinge line throughout its travel.
    turntable.visual(
        Cylinder(radius=0.010, length=0.095),
        origin=Origin(xyz=(0.260, -0.168, 0.250), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel_0",
    )
    turntable.visual(
        Cylinder(radius=0.010, length=0.095),
        origin=Origin(xyz=(0.260, 0.168, 0.250), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel_1",
    )
    turntable.visual(
        Cylinder(radius=0.0045, length=0.440),
        origin=Origin(xyz=(0.260, 0.0, 0.250), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    turntable.visual(
        Box((0.014, 0.100, 0.030)),
        origin=Origin(xyz=(0.251, -0.168, 0.239)),
        material=steel,
        name="hinge_leaf_0",
    )
    turntable.visual(
        Box((0.014, 0.100, 0.030)),
        origin=Origin(xyz=(0.251, 0.168, 0.239)),
        material=steel,
        name="hinge_leaf_1",
    )
    turntable.visual(
        Box((0.120, 0.120, 0.110)),
        origin=Origin(xyz=(0.090, -0.095, 0.155)),
        material=dark_armor,
        name="sensor_box",
    )
    turntable.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.159, -0.095, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="sensor_lens",
    )
    turntable.visual(
        Box((0.055, 0.110, 0.020)),
        origin=Origin(xyz=(0.050, -0.095, 0.105)),
        material=steel,
        name="sensor_foot",
    )

    # Elevating weapon cradle.  Its part frame is exactly the trunnion/elevation
    # axis; the receiver and barrel extend forward in local +X.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.030, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_axle",
    )
    cradle.visual(
        Box((0.190, 0.120, 0.090)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=black,
        name="receiver_block",
    )
    cradle.visual(
        Cylinder(radius=0.023, length=0.430),
        origin=Origin(xyz=(0.295, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.530, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="muzzle_sleeve",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.245),
        origin=Origin(xyz=(0.245, 0.038, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="recoil_tube",
    )
    cradle.visual(
        Box((0.090, 0.108, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, 0.054)),
        material=steel,
        name="top_rail",
    )

    # Manually adjustable top-hinged shield flap.  The joint frame is the hinge
    # pin line; the flap hangs down at q=0 and rotates outward/upward for q>0.
    shield_flap = model.part("shield_flap")
    shield_flap.visual(
        Cylinder(radius=0.010, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="flap_hinge_barrel",
    )
    shield_flap.visual(
        Box((0.022, 0.220, 0.178)),
        origin=Origin(xyz=(0.012, 0.0, -0.089)),
        material=armor,
        name="flap_plate",
    )
    shield_flap.visual(
        Box((0.010, 0.170, 0.030)),
        origin=Origin(xyz=(0.022, 0.0, -0.034)),
        material=dark_armor,
        name="stiffener_rib",
    )
    shield_flap.visual(
        Box((0.030, 0.105, 0.020)),
        origin=Origin(xyz=(0.016, 0.0, -0.169)),
        material=rubber,
        name="lower_grip_pad",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.020, 0.0, 0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.28, upper=0.62),
    )
    model.articulation(
        "shield_hinge",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=shield_flap,
        origin=Origin(xyz=(0.260, 0.0, 0.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    shield_flap = object_model.get_part("shield_flap")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")
    shield_hinge = object_model.get_articulation("shield_hinge")

    # The trunnion shaft is intentionally captured inside the side bosses and
    # cheek plates.  Those supports are simple solid proxies for bored metal
    # brackets, so the local shaft-in-boss intersections are the mechanism.
    for boss_name in ("trunnion_boss_0", "trunnion_boss_1"):
        ctx.allow_overlap(
            cradle,
            turntable,
            elem_a="trunnion_axle",
            elem_b=boss_name,
            reason="The elevating trunnion axle is intentionally captured in the side boss bore proxy.",
        )
        ctx.expect_overlap(
            cradle,
            turntable,
            axes="xyz",
            elem_a="trunnion_axle",
            elem_b=boss_name,
            min_overlap=0.020,
            name=f"{boss_name} captures trunnion axle",
        )

    for support_name in ("side_support_0", "side_support_1"):
        ctx.allow_overlap(
            cradle,
            turntable,
            elem_a="trunnion_axle",
            elem_b=support_name,
            reason="The trunnion axle passes through the side support bore represented by a solid cheek proxy.",
        )
        ctx.expect_overlap(
            cradle,
            turntable,
            axes="xyz",
            elem_a="trunnion_axle",
            elem_b=support_name,
            min_overlap=0.020,
            name=f"{support_name} retains trunnion axle",
        )

    ctx.expect_contact(
        pedestal,
        turntable,
        elem_a="bearing_race",
        elem_b="rotating_bearing",
        contact_tol=0.002,
        name="azimuth bearing surfaces meet",
    )

    ctx.allow_overlap(
        turntable,
        shield_flap,
        elem_a="hinge_pin",
        elem_b="flap_hinge_barrel",
        reason="The shield flap barrel is intentionally captured around the fixed top hinge pin.",
    )
    ctx.expect_overlap(
        turntable,
        shield_flap,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="flap_hinge_barrel",
        min_overlap=0.006,
        name="shield flap barrel remains on top hinge pin",
    )

    def _aabb_center(bounds):
        if bounds is None:
            return None
        low, high = bounds
        return (
            (float(low[0]) + float(high[0])) * 0.5,
            (float(low[1]) + float(high[1])) * 0.5,
            (float(low[2]) + float(high[2])) * 0.5,
        )

    # Positive azimuth rotates the whole station around the vertical bearing.
    rest_front = _aabb_center(ctx.part_element_world_aabb(turntable, elem="front_armor"))
    with ctx.pose({azimuth: 0.85}):
        rotated_front = _aabb_center(ctx.part_element_world_aabb(turntable, elem="front_armor"))
    ctx.check(
        "front armor follows azimuth rotation",
        rest_front is not None
        and rotated_front is not None
        and rotated_front[1] > rest_front[1] + 0.10,
        details=f"rest={rest_front}, rotated={rotated_front}",
    )

    # Positive elevation raises the barrel because the cradle extends forward
    # from the trunnion and the joint axis is -Y.
    rest_barrel = ctx.part_element_world_aabb(cradle, elem="barrel")
    rest_barrel_center = _aabb_center(rest_barrel)
    with ctx.pose({elevation: 0.55}):
        raised_barrel = ctx.part_element_world_aabb(cradle, elem="barrel")
        raised_barrel_center = _aabb_center(raised_barrel)
    ctx.check(
        "elevation joint raises barrel",
        rest_barrel_center is not None
        and raised_barrel_center is not None
        and raised_barrel_center[2] > rest_barrel_center[2] + 0.08,
        details=f"rest={rest_barrel_center}, raised={raised_barrel_center}",
    )

    # The shield flap barrel sits between two fixed hinge knuckles.  These gap
    # checks are repeated at an open pose to prove the flap remains clipped to
    # the top hinge line while moving instead of detaching.
    for pose_name, q in (("stowed", 0.0), ("raised", 0.85)):
        with ctx.pose({shield_hinge: q}):
            ctx.expect_gap(
                shield_flap,
                turntable,
                axis="y",
                positive_elem="flap_hinge_barrel",
                negative_elem="hinge_barrel_0",
                min_gap=0.004,
                max_gap=0.018,
                name=f"flap hinge lower-side clip gap {pose_name}",
            )
            ctx.expect_gap(
                turntable,
                shield_flap,
                axis="y",
                positive_elem="hinge_barrel_1",
                negative_elem="flap_hinge_barrel",
                min_gap=0.004,
                max_gap=0.018,
                name=f"flap hinge upper-side clip gap {pose_name}",
            )

    rest_flap = ctx.part_element_world_aabb(shield_flap, elem="flap_plate")
    rest_flap_center = _aabb_center(rest_flap)
    with ctx.pose({shield_hinge: 0.85}):
        raised_flap = ctx.part_element_world_aabb(shield_flap, elem="flap_plate")
        raised_flap_center = _aabb_center(raised_flap)
    ctx.check(
        "shield flap raises outward from top hinge",
        rest_flap_center is not None
        and raised_flap_center is not None
        and raised_flap_center[0] > rest_flap_center[0] + 0.040
        and raised_flap_center[2] > rest_flap_center[2] + 0.025,
        details=f"rest={rest_flap_center}, raised={raised_flap_center}",
    )

    return ctx.report()


object_model = build_object_model()
