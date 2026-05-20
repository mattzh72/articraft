from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_head_ceiling_fan")

    model.material("oil_rubbed_bronze", rgba=(0.10, 0.075, 0.045, 1.0))
    model.material("dark_metal", rgba=(0.025, 0.027, 0.030, 1.0))
    model.material("warm_brass", rgba=(0.62, 0.44, 0.18, 1.0))
    model.material("smoked_blades", rgba=(0.18, 0.20, 0.22, 0.86))
    model.material("black_wire", rgba=(0.012, 0.012, 0.013, 1.0))

    canopy_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.000, 0.880),
                (0.045, 0.880),
                (0.125, 0.905),
                (0.162, 0.960),
                (0.145, 0.995),
                (0.000, 0.995),
            ],
            segments=64,
        ),
        "ceiling_canopy",
    )
    guard_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.270, tube=0.006, radial_segments=18, tubular_segments=72),
        "fan_guard_ring",
    )
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.230,
            0.055,
            4,
            thickness=0.030,
            blade_pitch_deg=32.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="broad", tip_pitch_deg=14.0, camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.014),
        ),
        "four_blade_rotor",
    )

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.170, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 1.003)),
        material="dark_metal",
        name="ceiling_plate",
    )
    ceiling_mount.visual(
        canopy_mesh,
        material="oil_rubbed_bronze",
        name="canopy_shell",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.020, length=0.850),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material="oil_rubbed_bronze",
        name="downrod",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.044, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material="warm_brass",
        name="upper_rod_collar",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="warm_brass",
        name="lower_rod_collar",
    )

    crossbar = model.part("crossbar")
    crossbar.visual(
        Cylinder(radius=0.064, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material="warm_brass",
        name="yaw_bearing",
    )
    crossbar.visual(
        Cylinder(radius=0.024, length=1.420),
        origin=Origin(xyz=(0.0, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="oil_rubbed_bronze",
        name="bar_tube",
    )

    ring_inner = 0.742
    ring_outer = 0.818
    fan_center = 0.780
    fan_z = -0.060
    ring_radius = 0.270
    spoke_inner = 0.028
    spoke_outer = 0.268
    spoke_mid = (spoke_inner + spoke_outer) / 2.0
    spoke_len = spoke_outer - spoke_inner

    crossbar.visual(
        guard_ring_mesh,
        origin=Origin(xyz=(-ring_inner, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_wire",
        name="fan_0_inner_ring",
    )
    crossbar.visual(
        guard_ring_mesh,
        origin=Origin(xyz=(-ring_outer, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_wire",
        name="fan_0_outer_ring",
    )
    crossbar.visual(
        guard_ring_mesh,
        origin=Origin(xyz=(ring_inner, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_wire",
        name="fan_1_inner_ring",
    )
    crossbar.visual(
        guard_ring_mesh,
        origin=Origin(xyz=(ring_outer, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_wire",
        name="fan_1_outer_ring",
    )

    for idx, side in enumerate((-1.0, 1.0)):
        motor_x = side * 0.696
        inner_x = side * ring_inner
        outer_x = side * ring_outer
        center_x = side * fan_center

        crossbar.visual(
            Cylinder(radius=0.075, length=0.112),
            origin=Origin(xyz=(motor_x, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="oil_rubbed_bronze",
            name=f"fan_{idx}_motor",
        )
        crossbar.visual(
            Cylinder(radius=0.038, length=0.020),
            origin=Origin(xyz=(side * 0.748, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="warm_brass",
            name=f"fan_{idx}_bearing_cap",
        )

        for ring_name, ring_x in (("inner", inner_x), ("outer", outer_x)):
            crossbar.visual(
                Cylinder(radius=0.035, length=0.010),
                origin=Origin(xyz=(ring_x, 0.0, fan_z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material="black_wire",
                name=f"fan_{idx}_{ring_name}_guard_hub",
            )
            for spoke_i, theta in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, 5.0 * math.pi / 4.0, 3.0 * math.pi / 2.0, 7.0 * math.pi / 4.0)):
                crossbar.visual(
                    Cylinder(radius=0.0036, length=spoke_len),
                    origin=Origin(
                        xyz=(ring_x, spoke_mid * math.cos(theta), fan_z + spoke_mid * math.sin(theta)),
                        rpy=(theta - math.pi / 2.0, 0.0, 0.0),
                    ),
                    material="black_wire",
                    name=f"fan_{idx}_{ring_name}_spoke_{spoke_i}",
                )

        for tie_i, theta in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, 5.0 * math.pi / 4.0, 3.0 * math.pi / 2.0, 7.0 * math.pi / 4.0)):
            crossbar.visual(
                Cylinder(radius=0.0038, length=ring_outer - ring_inner + 0.016),
                origin=Origin(
                    xyz=(center_x, ring_radius * math.cos(theta), fan_z + ring_radius * math.sin(theta)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material="black_wire",
                name=f"fan_{idx}_cage_tie_{tie_i}",
            )

    fan_0 = model.part("fan_0")
    fan_0.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="smoked_blades",
        name="rotor",
    )
    fan_0.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="warm_brass",
        name="axle_stub",
    )

    fan_1 = model.part("fan_1")
    fan_1.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="smoked_blades",
        name="rotor",
    )
    fan_1.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="warm_brass",
        name="axle_stub",
    )

    model.articulation(
        "downrod_to_crossbar",
        ArticulationType.REVOLUTE,
        parent=ceiling_mount,
        child=crossbar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "crossbar_to_fan_0",
        ArticulationType.REVOLUTE,
        parent=crossbar,
        child=fan_0,
        origin=Origin(xyz=(-fan_center, 0.0, fan_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0, lower=0.0, upper=2.0 * math.pi),
    )
    model.articulation(
        "crossbar_to_fan_1",
        ArticulationType.REVOLUTE,
        parent=crossbar,
        child=fan_1,
        origin=Origin(xyz=(fan_center, 0.0, fan_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0, lower=0.0, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_mount = object_model.get_part("ceiling_mount")
    crossbar = object_model.get_part("crossbar")
    fan_0 = object_model.get_part("fan_0")
    fan_1 = object_model.get_part("fan_1")
    yaw = object_model.get_articulation("downrod_to_crossbar")
    axle_0 = object_model.get_articulation("crossbar_to_fan_0")
    axle_1 = object_model.get_articulation("crossbar_to_fan_1")

    ctx.expect_contact(
        ceiling_mount,
        crossbar,
        elem_a="lower_rod_collar",
        elem_b="yaw_bearing",
        contact_tol=0.001,
        name="downrod collar seats on crossbar yaw bearing",
    )
    ctx.expect_origin_distance(
        fan_0,
        fan_1,
        axes="x",
        min_dist=1.50,
        max_dist=1.60,
        name="fan axles are mounted at opposite crossbar ends",
    )
    ctx.expect_within(
        fan_0,
        crossbar,
        axes="yz",
        inner_elem="rotor",
        outer_elem="fan_0_outer_ring",
        margin=0.0,
        name="fan 0 rotor sits within its guard diameter",
    )
    ctx.expect_within(
        fan_1,
        crossbar,
        axes="yz",
        inner_elem="rotor",
        outer_elem="fan_1_outer_ring",
        margin=0.0,
        name="fan 1 rotor sits within its guard diameter",
    )
    ctx.expect_gap(
        fan_0,
        crossbar,
        axis="x",
        positive_elem="rotor",
        negative_elem="fan_0_outer_ring",
        min_gap=0.006,
        name="fan 0 rotor clears outboard guard",
    )
    ctx.expect_gap(
        crossbar,
        fan_0,
        axis="x",
        positive_elem="fan_0_inner_ring",
        negative_elem="rotor",
        min_gap=0.006,
        name="fan 0 rotor clears inboard guard",
    )
    ctx.expect_gap(
        fan_1,
        crossbar,
        axis="x",
        positive_elem="rotor",
        negative_elem="fan_1_inner_ring",
        min_gap=0.006,
        name="fan 1 rotor clears inboard guard",
    )
    ctx.expect_gap(
        crossbar,
        fan_1,
        axis="x",
        positive_elem="fan_1_outer_ring",
        negative_elem="rotor",
        min_gap=0.006,
        name="fan 1 rotor clears outboard guard",
    )

    with ctx.pose({yaw: 0.5, axle_0: math.pi / 2.0, axle_1: math.pi / 4.0}):
        ctx.expect_within(
            fan_0,
            crossbar,
            axes="yz",
            inner_elem="rotor",
            outer_elem="fan_0_outer_ring",
            margin=0.0,
            name="fan 0 remains guarded while spinning",
        )
        ctx.expect_within(
            fan_1,
            crossbar,
            axes="yz",
            inner_elem="rotor",
            outer_elem="fan_1_outer_ring",
            margin=0.0,
            name="fan 1 remains guarded while spinning",
        )

    ctx.check(
        "crossbar joint is vertical revolute",
        tuple(round(v, 3) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "fan heads have independent horizontal axles",
        axle_0 is not axle_1
        and tuple(round(v, 3) for v in axle_0.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in axle_1.axis) == (1.0, 0.0, 0.0),
        details=f"axle_0={axle_0.axis}, axle_1={axle_1.axis}",
    )

    return ctx.report()


object_model = build_object_model()
