from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_weapon_station")

    roof_paint = model.material("roof_paint", rgba=(0.31, 0.34, 0.30, 1.0))
    armor_paint = model.material("armor_paint", rgba=(0.43, 0.47, 0.39, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.24, 0.26, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))

    roof = model.part("roof")
    roof.visual(
        Box((1.45, 1.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=roof_paint,
        name="roof_panel",
    )
    roof.visual(
        Box((0.62, 0.46, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=dark_metal,
        name="roof_reinforcement",
    )
    roof.visual(
        Cylinder(radius=0.34, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="mounting_flange",
    )
    roof.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_metal,
        name="bearing_housing",
    )
    roof.visual(
        Cylinder(radius=0.31, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_metal,
        name="bearing_top_plate",
    )
    roof.visual(
        Box((0.20, 0.12, 0.08)),
        origin=Origin(xyz=(-0.28, 0.0, 0.040)),
        material=dark_metal,
        name="service_box",
    )
    roof.inertial = Inertial.from_geometry(
        Box((1.45, 1.10, 0.18)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.30, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_metal,
        name="traverse_ring",
    )
    turret.visual(
        Cylinder(radius=0.22, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=gunmetal,
        name="ring_skirt",
    )
    turret.visual(
        Box((0.74, 0.58, 0.03)),
        origin=Origin(xyz=(0.030, 0.0, 0.135)),
        material=armor_paint,
        name="turret_deck",
    )
    turret.visual(
        Box((0.56, 0.03, 0.44)),
        origin=Origin(xyz=(0.080, -0.235, 0.345)),
        material=armor_paint,
        name="left_side_plate",
    )
    turret.visual(
        Box((0.56, 0.03, 0.44)),
        origin=Origin(xyz=(0.080, 0.235, 0.345)),
        material=armor_paint,
        name="right_side_plate",
    )
    turret.visual(
        Box((0.04, 0.50, 0.28)),
        origin=Origin(xyz=(-0.180, 0.0, 0.255)),
        material=armor_paint,
        name="rear_plate",
    )
    turret.visual(
        Box((0.06, 0.50, 0.14)),
        origin=Origin(xyz=(0.360, 0.0, 0.160)),
        material=armor_paint,
        name="lower_front_plate",
    )
    turret.visual(
        Box((0.06, 0.50, 0.12)),
        origin=Origin(xyz=(0.350, 0.0, 0.480)),
        material=armor_paint,
        name="upper_front_plate",
    )
    turret.visual(
        Box((0.42, 0.50, 0.03)),
        origin=Origin(xyz=(0.080, 0.0, 0.555), rpy=(0.0, -0.10, 0.0)),
        material=armor_paint,
        name="shield_roof",
    )
    turret.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.050, -0.223, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion_housing",
    )
    turret.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.050, 0.223, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion_housing",
    )
    turret.visual(
        Box((0.36, 0.18, 0.20)),
        origin=Origin(xyz=(-0.020, 0.355, 0.240)),
        material=armor_paint,
        name="ammo_box_body",
    )
    turret.visual(
        Box((0.14, 0.06, 0.10)),
        origin=Origin(xyz=(0.120, 0.250, 0.220)),
        material=armor_paint,
        name="ammo_box_bridge",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.90, 0.86, 0.64)),
        mass=290.0,
        origin=Origin(xyz=(0.030, 0.040, 0.290)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.032, length=0.36),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(xyz=(0.0, -0.169, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion_cap",
    )
    cradle.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(xyz=(0.0, 0.169, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion_cap",
    )
    cradle.visual(
        Box((0.30, 0.18, 0.16)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=gunmetal,
        name="receiver",
    )
    cradle.visual(
        Box((0.14, 0.16, 0.18)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=gunmetal,
        name="breech_block",
    )
    cradle.visual(
        Box((0.20, 0.04, 0.06)),
        origin=Origin(xyz=(0.020, -0.110, 0.0)),
        material=dark_metal,
        name="left_saddle_arm",
    )
    cradle.visual(
        Box((0.20, 0.04, 0.06)),
        origin=Origin(xyz=(0.020, 0.110, 0.0)),
        material=dark_metal,
        name="right_saddle_arm",
    )
    cradle.visual(
        Box((0.12, 0.10, 0.06)),
        origin=Origin(xyz=(-0.080, 0.0, 0.080)),
        material=gunmetal,
        name="feed_cover",
    )
    cradle.visual(
        Cylinder(radius=0.070, length=0.16),
        origin=Origin(xyz=(0.320, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="barrel_jacket",
    )
    cradle.visual(
        Cylinder(radius=0.019, length=0.82),
        origin=Origin(xyz=(0.810, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.030, length=0.10),
        origin=Origin(xyz=(1.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="muzzle_device",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((1.42, 0.42, 0.22)),
        mass=95.0,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
    )

    ammo_lid = model.part("ammo_lid")
    ammo_lid.visual(
        Box((0.36, 0.18, 0.018)),
        origin=Origin(xyz=(0.180, 0.0, 0.009)),
        material=armor_paint,
        name="lid_skin",
    )
    ammo_lid.visual(
        Box((0.36, 0.012, 0.032)),
        origin=Origin(xyz=(0.180, 0.084, 0.016)),
        material=armor_paint,
        name="outer_lip",
    )
    ammo_lid.visual(
        Cylinder(radius=0.010, length=0.16),
        origin=Origin(xyz=(0.010, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_roll",
    )
    ammo_lid.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.04)),
        mass=8.0,
        origin=Origin(xyz=(0.180, 0.0, 0.018)),
    )

    model.articulation(
        "roof_to_turret_yaw",
        ArticulationType.CONTINUOUS,
        parent=roof,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "turret_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.050, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=1.0,
            lower=-0.15,
            upper=0.75,
        ),
    )
    model.articulation(
        "turret_to_ammo_lid",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=ammo_lid,
        origin=Origin(xyz=(-0.200, 0.355, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    ammo_lid = object_model.get_part("ammo_lid")

    yaw = object_model.get_articulation("roof_to_turret_yaw")
    pitch = object_model.get_articulation("turret_to_cradle_pitch")
    lid_hinge = object_model.get_articulation("turret_to_ammo_lid")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))

    with ctx.pose({yaw: 0.0, pitch: 0.0, lid_hinge: 0.0}):
        ctx.expect_gap(
            turret,
            roof,
            axis="z",
            positive_elem="traverse_ring",
            negative_elem="bearing_top_plate",
            max_gap=0.002,
            max_penetration=0.0,
            name="turret ring sits on bearing housing",
        )
        ctx.expect_overlap(
            turret,
            roof,
            axes="xy",
            elem_a="traverse_ring",
            elem_b="bearing_top_plate",
            min_overlap=0.18,
            name="turret ring overlaps bearing footprint",
        )
        ctx.expect_gap(
            ammo_lid,
            turret,
            axis="z",
            positive_elem="lid_skin",
            negative_elem="ammo_box_body",
            max_gap=0.002,
            max_penetration=0.0,
            name="ammo lid closes flush on box top",
        )
        ctx.expect_gap(
            cradle,
            turret,
            axis="y",
            positive_elem="receiver",
            negative_elem="left_side_plate",
            min_gap=0.09,
            name="receiver clears left shield wall",
        )
        ctx.expect_gap(
            turret,
            cradle,
            axis="y",
            positive_elem="right_side_plate",
            negative_elem="receiver",
            min_gap=0.09,
            name="receiver clears right shield wall",
        )

    rest_barrel = _aabb_center(ctx.part_element_world_aabb(cradle, elem="barrel"))
    with ctx.pose({pitch: 0.65}):
        elevated_barrel = _aabb_center(ctx.part_element_world_aabb(cradle, elem="barrel"))
    ctx.check(
        "positive pitch raises the gun barrel",
        rest_barrel is not None
        and elevated_barrel is not None
        and elevated_barrel[2] > rest_barrel[2] + 0.22,
        details=f"rest={rest_barrel}, elevated={elevated_barrel}",
    )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        forward_barrel = _aabb_center(ctx.part_element_world_aabb(cradle, elem="barrel"))
    with ctx.pose({yaw: math.pi / 2.0, pitch: 0.0}):
        slewed_barrel = _aabb_center(ctx.part_element_world_aabb(cradle, elem="barrel"))
    ctx.check(
        "positive yaw slews the weapon around the roof ring",
        forward_barrel is not None
        and slewed_barrel is not None
        and forward_barrel[0] > 0.70
        and slewed_barrel[1] > 0.70,
        details=f"forward={forward_barrel}, slewed={slewed_barrel}",
    )

    closed_lid = _aabb_center(ctx.part_element_world_aabb(ammo_lid, elem="lid_skin"))
    with ctx.pose({lid_hinge: 1.20}):
        open_lid = _aabb_center(ctx.part_element_world_aabb(ammo_lid, elem="lid_skin"))
    ctx.check(
        "ammo lid opens upward from the box",
        closed_lid is not None
        and open_lid is not None
        and open_lid[2] > closed_lid[2] + 0.09,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
