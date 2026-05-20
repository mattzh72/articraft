from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


AXLE_Z = 0.43
WHEEL_Y = 0.68
WHEEL_RADIUS = 0.43
WHEEL_WIDTH = 0.15

TRUNNION_X = 0.18
TRUNNION_Z = 0.82

TRAIL_HINGE_X = -0.48
TRAIL_HINGE_Y = 0.18
TRAIL_HINGE_Z = 0.32


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _trunnion_cheek_geometry():
    outer = [(-0.16, -0.14), (0.16, -0.14), (0.16, 0.14), (-0.16, 0.14)]
    hole = _circle_profile(0.086, 48)
    geom = ExtrudeWithHolesGeometry(outer, [hole], 0.035, center=True)
    geom.rotate_x(math.pi / 2.0)
    return geom


def _barrel_geometry():
    outer_profile = [
        (0.145, -0.24),
        (0.145, -0.10),
        (0.116, -0.05),
        (0.100, 0.32),
        (0.091, 0.82),
        (0.118, 0.88),
        (0.118, 1.02),
        (0.087, 1.08),
        (0.083, 1.17),
    ]
    inner_profile = [(0.050, -0.24), (0.050, 1.17)]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_pack_howitzer")

    olive = model.material("olive_drab", rgba=(0.28, 0.34, 0.20, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.18, 0.23, 0.15, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.16, 0.17, 0.16, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.42, 0.43, 0.40, 1.0))
    tire_steel = model.material("dark_steel_tire", rgba=(0.06, 0.065, 0.06, 1.0))
    bore_black = model.material("bore_black", rgba=(0.005, 0.005, 0.004, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.055, length=1.30),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="axle",
    )
    carriage.visual(
        Box((0.34, 0.60, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        material=olive,
        name="axle_saddle",
    )
    carriage.visual(
        Box((0.88, 0.18, 0.10)),
        origin=Origin(xyz=(-0.40, 0.0, 0.36)),
        material=dark_olive,
        name="rear_drawbar",
    )
    carriage.visual(
        Box((0.12, 0.13, 0.06)),
        origin=Origin(xyz=(-0.615, 0.080, TRAIL_HINGE_Z)),
        material=dark_olive,
        name="trail_hinge_bridge_0",
    )
    carriage.visual(
        Box((0.12, 0.13, 0.06)),
        origin=Origin(xyz=(-0.615, -0.080, TRAIL_HINGE_Z)),
        material=dark_olive,
        name="trail_hinge_bridge_1",
    )
    carriage.visual(
        Box((0.16, 0.08, 0.18)),
        origin=Origin(xyz=(TRAIL_HINGE_X, TRAIL_HINGE_Y, TRAIL_HINGE_Z)),
        material=worn_steel,
        name="trail_hinge_socket_0",
    )
    carriage.visual(
        Box((0.16, 0.08, 0.18)),
        origin=Origin(xyz=(TRAIL_HINGE_X, -TRAIL_HINGE_Y, TRAIL_HINGE_Z)),
        material=worn_steel,
        name="trail_hinge_socket_1",
    )
    for y, visual_name in ((0.245, "cradle_upright_0"), (-0.245, "cradle_upright_1")):
        carriage.visual(
            Box((0.10, 0.060, 0.35)),
            origin=Origin(xyz=(0.04, y, 0.56)),
            material=olive,
            name=visual_name,
        )
    carriage.visual(
        Box((0.56, 0.56, 0.090)),
        origin=Origin(xyz=(0.16, 0.0, 0.615)),
        material=olive,
        name="cradle_crossmember",
    )
    cheek_mesh = mesh_from_geometry(_trunnion_cheek_geometry(), "trunnion_cheek_plate")
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(TRUNNION_X, 0.225, TRUNNION_Z)),
        material=olive,
        name="trunnion_cheek_0",
    )
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(TRUNNION_X, -0.225, TRUNNION_Z)),
        material=olive,
        name="trunnion_cheek_1",
    )
    for y, visual_name in ((0.180, "cradle_rail_0"), (-0.180, "cradle_rail_1")):
        carriage.visual(
            Box((0.60, 0.040, 0.060)),
            origin=Origin(xyz=(0.32, y, 0.685)),
            material=dark_olive,
            name=visual_name,
        )
    carriage.visual(
        Box((0.22, 0.72, 0.030)),
        origin=Origin(xyz=(-0.09, 0.0, 0.565)),
        material=worn_steel,
        name="spring_equalizer",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_barrel_geometry(), "short_hollow_barrel"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_shell",
    )
    barrel.visual(
        Box((0.22, 0.26, 0.23)),
        origin=Origin(xyz=(-0.18, 0.0, 0.0)),
        material=gunmetal,
        name="breech_block",
    )
    barrel.visual(
        Cylinder(radius=0.065, length=0.36),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_pin",
    )
    for y, visual_name in ((0.180, "trunnion_collar_0"), (-0.180, "trunnion_collar_1")):
        barrel.visual(
            Cylinder(radius=0.095, length=0.055),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=visual_name,
        )
    for y, visual_name in ((0.080, "recoil_tube_0"), (-0.080, "recoil_tube_1")):
        barrel.visual(
            Cylinder(radius=0.032, length=0.82),
            origin=Origin(xyz=(0.34, y, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name=visual_name,
        )
    barrel.visual(
        Cylinder(radius=0.052, length=0.025),
        origin=Origin(xyz=(1.158, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bore_black,
        name="muzzle_bore",
    )

    for index, side in enumerate((1.0, -1.0)):
        trail = model.part(f"trail_{index}")
        trail.visual(
            Cylinder(radius=0.060, length=0.17),
            origin=Origin(),
            material=worn_steel,
            name="hinge_knuckle",
        )
        trail.visual(
            Box((0.16, 0.060, 0.070)),
            origin=Origin(xyz=(-0.08, 0.0, -0.010)),
            material=olive,
            name="trail_neck",
        )
        trail.visual(
            Box((1.22, 0.070, 0.080)),
            origin=Origin(xyz=(-0.74, 0.0, -0.025)),
            material=olive,
            name="trail_beam",
        )
        trail.visual(
            Box((0.20, 0.28, 0.18)),
            origin=Origin(xyz=(-1.38, 0.0, -0.135)),
            material=worn_steel,
            name="spade",
        )
        trail.visual(
            Box((0.24, 0.18, 0.040)),
            origin=Origin(xyz=(-1.32, 0.0, -0.235)),
            material=worn_steel,
            name="spade_foot",
        )
        model.articulation(
            f"carriage_to_trail_{index}",
            ArticulationType.REVOLUTE,
            parent=carriage,
            child=trail,
            origin=Origin(xyz=(TRAIL_HINGE_X, side * TRAIL_HINGE_Y, TRAIL_HINGE_Z)),
            axis=(0.0, 0.0, -side),
            motion_limits=MotionLimits(lower=0.0, upper=0.62, effort=90.0, velocity=0.7),
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.395,
            0.120,
            rim=WheelRim(inner_radius=0.305, flange_height=0.025, flange_thickness=0.010),
            hub=WheelHub(radius=0.075, width=0.145, cap_style="domed"),
            face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.011, window_radius=0.022),
            bore=WheelBore(style="round", diameter=0.070),
        ),
        "artillery_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.382,
            tread=TireTread(style="circumferential", depth=0.006, count=2),
            sidewall=TireSidewall(style="square", bulge=0.015),
            shoulder=TireShoulder(width=0.012, radius=0.004),
        ),
        "steel_road_tire",
    )
    for index, side in enumerate((1.0, -1.0)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(wheel_mesh, material=worn_steel, name="spoked_wheel")
        wheel.visual(tire_mesh, material=tire_steel, name="steel_tire")
        model.articulation(
            f"axle_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(0.0, side * WHEEL_Y, AXLE_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=18.0),
        )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=0.72, effort=180.0, velocity=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    trail_0 = object_model.get_part("trail_0")
    trail_1 = object_model.get_part("trail_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    trail_joint_0 = object_model.get_articulation("carriage_to_trail_0")
    trail_joint_1 = object_model.get_articulation("carriage_to_trail_1")
    wheel_joint_0 = object_model.get_articulation("axle_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("axle_to_wheel_1")

    ctx.allow_overlap(
        carriage,
        trail_0,
        elem_a="trail_hinge_socket_0",
        elem_b="hinge_knuckle",
        reason="The trail hinge knuckle and neck are locally captured inside the simplified hinge socket.",
    )
    ctx.allow_overlap(
        carriage,
        trail_0,
        elem_a="trail_hinge_socket_0",
        elem_b="trail_neck",
        reason="The front neck of the trail is intentionally tucked into the same hinge socket.",
    )
    ctx.allow_overlap(
        carriage,
        trail_1,
        elem_a="trail_hinge_socket_1",
        elem_b="hinge_knuckle",
        reason="The trail hinge knuckle and neck are locally captured inside the simplified hinge socket.",
    )
    ctx.allow_overlap(
        carriage,
        trail_1,
        elem_a="trail_hinge_socket_1",
        elem_b="trail_neck",
        reason="The front neck of the trail is intentionally tucked into the same hinge socket.",
    )
    ctx.allow_overlap(
        carriage,
        wheel_0,
        elem_a="axle",
        elem_b="spoked_wheel",
        reason="The axle is intentionally represented as seated a short distance inside the wheel hub.",
    )
    ctx.allow_overlap(
        carriage,
        wheel_1,
        elem_a="axle",
        elem_b="spoked_wheel",
        reason="The axle is intentionally represented as seated a short distance inside the wheel hub.",
    )

    ctx.check(
        "primary_articulations_present",
        all(
            joint is not None
            for joint in (
                barrel_joint,
                trail_joint_0,
                trail_joint_1,
                wheel_joint_0,
                wheel_joint_1,
            )
        ),
        "Expected barrel elevation, two split-trail hinges, and two wheel spin joints.",
    )
    ctx.check(
        "wheel_joints_continuous",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        "The two carriage wheels should spin on continuous axle joints.",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        positive_elem="barrel_shell",
        negative_elem="cradle_crossmember",
        min_gap=0.005,
        name="barrel clears lower cradle crossmember",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="trunnion_collar_0",
        elem_b="trunnion_cheek_0",
        contact_tol=0.002,
        name="barrel trunnion collar bears against cheek",
    )
    ctx.expect_overlap(
        carriage,
        trail_0,
        axes="xyz",
        elem_a="trail_hinge_socket_0",
        elem_b="hinge_knuckle",
        min_overlap=0.045,
        name="trail_0 hinge is captured at its socket",
    )
    ctx.expect_overlap(
        carriage,
        trail_1,
        axes="xyz",
        elem_a="trail_hinge_socket_1",
        elem_b="hinge_knuckle",
        min_overlap=0.045,
        name="trail_1 hinge is captured at its socket",
    )
    ctx.expect_overlap(
        carriage,
        trail_0,
        axes="xyz",
        elem_a="trail_hinge_socket_0",
        elem_b="trail_neck",
        min_overlap=0.040,
        name="trail_0 neck seats in hinge socket",
    )
    ctx.expect_overlap(
        carriage,
        trail_1,
        axes="xyz",
        elem_a="trail_hinge_socket_1",
        elem_b="trail_neck",
        min_overlap=0.040,
        name="trail_1 neck seats in hinge socket",
    )
    ctx.expect_overlap(
        carriage,
        wheel_0,
        axes="y",
        elem_a="axle",
        elem_b="spoked_wheel",
        min_overlap=0.030,
        name="wheel_0 hub remains seated on the axle",
    )
    ctx.expect_overlap(
        carriage,
        wheel_1,
        axes="y",
        elem_a="axle",
        elem_b="spoked_wheel",
        min_overlap=0.030,
        name="wheel_1 hub remains seated on the axle",
    )

    closed_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    closed_trail_0_aabb = ctx.part_world_aabb(trail_0)
    closed_trail_1_aabb = ctx.part_world_aabb(trail_1)
    with ctx.pose({barrel_joint: 0.62, trail_joint_0: 0.62, trail_joint_1: 0.62}):
        elevated_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
        spread_trail_0_aabb = ctx.part_world_aabb(trail_0)
        spread_trail_1_aabb = ctx.part_world_aabb(trail_1)

    ctx.check(
        "barrel_elevates_from_trunnions",
        closed_barrel_aabb is not None
        and elevated_barrel_aabb is not None
        and float(elevated_barrel_aabb[1][2] - closed_barrel_aabb[1][2]) > 0.45,
        details=f"closed={closed_barrel_aabb}, elevated={elevated_barrel_aabb}",
    )
    ctx.check(
        "split_trails_rotate_outward",
        closed_trail_0_aabb is not None
        and closed_trail_1_aabb is not None
        and spread_trail_0_aabb is not None
        and spread_trail_1_aabb is not None
        and float(spread_trail_0_aabb[1][1] - closed_trail_0_aabb[1][1]) > 0.55
        and float(closed_trail_1_aabb[0][1] - spread_trail_1_aabb[0][1]) > 0.55,
        details=(
            f"closed0={closed_trail_0_aabb}, spread0={spread_trail_0_aabb}, "
            f"closed1={closed_trail_1_aabb}, spread1={spread_trail_1_aabb}"
        ),
    )
    wheel_aabb = ctx.part_element_world_aabb(wheel_0, elem="steel_tire")
    ctx.check(
        "wheel_tire_reaches_ground",
        wheel_aabb is not None and abs(float(wheel_aabb[0][2])) < 0.010,
        details=f"wheel_0 tire aabb={wheel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
