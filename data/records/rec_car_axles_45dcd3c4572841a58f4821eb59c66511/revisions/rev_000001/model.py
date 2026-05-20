from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torque_tube_rear_axle")

    cast_iron = Material("dark_cast_iron", rgba=(0.055, 0.060, 0.060, 1.0))
    tube_paint = Material("satin_black_tube", rgba=(0.015, 0.017, 0.016, 1.0))
    worn_steel = Material("worn_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    black_steel = Material("blackened_propshaft", rgba=(0.025, 0.023, 0.022, 1.0))
    gasket = Material("oiled_gasket", rgba=(0.010, 0.009, 0.007, 1.0))

    axle = model.part("axle")

    # A flattened "banjo" pumpkin with a removable rear cover.
    banjo = SphereGeometry(1.0, width_segments=56, height_segments=28)
    banjo.scale(0.30, 0.22, 0.28)
    axle.visual(
        mesh_from_geometry(banjo, "banjo_housing"),
        material=cast_iron,
        name="banjo_housing",
    )
    axle.visual(
        Cylinder(radius=0.175, length=0.050),
        origin=Origin(xyz=(0.0, 0.230, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="rear_cover",
    )
    axle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.176, tube=0.012, radial_segments=14, tubular_segments=72), "cover_lip"),
        origin=Origin(xyz=(0.0, 0.262, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gasket,
        name="cover_lip",
    )
    for i in range(10):
        a = 2.0 * math.pi * i / 10.0
        axle.visual(
            Cylinder(radius=0.011, length=0.024),
            origin=Origin(
                xyz=(0.145 * math.cos(a), 0.258, 0.145 * math.sin(a)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=worn_steel,
            name=f"cover_bolt_{i}",
        )

    # Rigid axle tubes and wheel-bearing flange faces.
    axle.visual(
        Cylinder(radius=0.055, length=1.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="axle_tube",
    )
    axle.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.820, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="flange_0",
    )
    axle.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(-0.820, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="flange_1",
    )
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        # Four reinforcing ears on each banjo/flange plate keep the flange from
        # reading as a free disk.
        for zsign in (-1.0, 1.0):
            axle.visual(
                Box((0.060, 0.060, 0.135)),
                origin=Origin(xyz=(sign * 0.805, 0.0, zsign * 0.085)),
                material=cast_iron,
                name=f"flange_web_{suffix}_{int(zsign > 0)}",
            )

    # The forward torque tube is modeled as an open, thick-walled tube enclosing
    # the propshaft rather than as a solid cylinder.
    tube_length = 1.46
    tube_outer = 0.120
    tube_inner = 0.092
    torque_tube_shell = LatheGeometry.from_shell_profiles(
        [(tube_outer, -tube_length / 2.0), (tube_outer, tube_length / 2.0)],
        [(tube_inner, -tube_length / 2.0), (tube_inner, tube_length / 2.0)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    axle.visual(
        mesh_from_geometry(torque_tube_shell, "torque_tube_shell"),
        origin=Origin(xyz=(0.0, -0.930, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=tube_paint,
        name="torque_tube",
    )
    rear_bell_shell = LatheGeometry.from_shell_profiles(
        [(0.170, -0.085), (0.170, 0.085)],
        [(0.095, -0.085), (0.095, 0.085)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    axle.visual(
        mesh_from_geometry(rear_bell_shell, "rear_bell_shell"),
        origin=Origin(xyz=(0.0, -0.245, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="rear_bell",
    )
    axle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.118, tube=0.017, radial_segments=16, tubular_segments=64), "front_tube_lip"),
        origin=Origin(xyz=(0.0, -1.660, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="front_tube_lip",
    )
    # Simple internal saddles touch the underside of the propshaft and tie it
    # back into the tube wall, representing the hidden support bearings without
    # filling the hollow tube.
    axle.visual(
        Box((0.045, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, -0.345, -0.064)),
        material=worn_steel,
        name="rear_bearing_saddle",
    )
    axle.visual(
        Box((0.045, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, -1.545, -0.064)),
        material=worn_steel,
        name="front_bearing_saddle",
    )

    propshaft = model.part("propshaft")
    propshaft.visual(
        Cylinder(radius=0.034, length=1.44),
        origin=Origin(xyz=(0.0, -0.720, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_steel,
        name="shaft",
    )
    propshaft.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.0, -1.405, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="front_yoke_barrel",
    )
    for x in (-0.050, 0.050):
        propshaft.visual(
            Box((0.030, 0.085, 0.110)),
            origin=Origin(xyz=(x, -1.460, 0.0)),
            material=worn_steel,
            name=f"front_yoke_ear_{int(x > 0)}",
        )

    model.articulation(
        "axle_to_propshaft",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=propshaft,
        origin=Origin(xyz=(0.0, -0.250, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=80.0),
    )

    def add_hub(name: str, suffix: str, sign: float) -> None:
        hub = model.part(name)
        hub.visual(
            Cylinder(radius=0.073, length=0.115),
            origin=Origin(xyz=(sign * 0.0575, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name="inner_bearing",
        )
        hub.visual(
            Cylinder(radius=0.155, length=0.040),
            origin=Origin(xyz=(sign * 0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.092, length=0.075),
            origin=Origin(xyz=(sign * 0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name="dust_cap",
        )
        for i in range(5):
            a = 2.0 * math.pi * i / 5.0 + math.pi / 5.0
            hub.visual(
                Cylinder(radius=0.009, length=0.060),
                origin=Origin(
                    xyz=(sign * 0.167, 0.090 * math.cos(a), 0.090 * math.sin(a)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=worn_steel,
                name=f"wheel_stud_{i}",
            )
        model.articulation(
            f"axle_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=axle,
            child=hub,
            origin=Origin(xyz=(sign * 0.850, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=100.0, velocity=90.0),
        )

    add_hub("hub_0", "0", 1.0)
    add_hub("hub_1", "1", -1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_part("axle")
    propshaft = object_model.get_part("propshaft")
    hub_0 = object_model.get_part("hub_0")
    hub_1 = object_model.get_part("hub_1")
    hub_0_joint = object_model.get_articulation("axle_to_hub_0")
    hub_1_joint = object_model.get_articulation("axle_to_hub_1")
    propshaft_joint = object_model.get_articulation("axle_to_propshaft")

    ctx.check(
        "outer hubs use continuous revolute spin joints",
        hub_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and hub_1_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(hub_0_joint.axis[0]) > 0.99
        and abs(hub_1_joint.axis[0]) > 0.99,
        details=f"hub_0={hub_0_joint.articulation_type},{hub_0_joint.axis}; "
        f"hub_1={hub_1_joint.articulation_type},{hub_1_joint.axis}",
    )
    ctx.check(
        "propshaft spins along the torque tube axis",
        propshaft_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(propshaft_joint.axis[1]) > 0.99,
        details=f"propshaft={propshaft_joint.articulation_type},{propshaft_joint.axis}",
    )

    ctx.expect_within(
        propshaft,
        axle,
        axes="xz",
        inner_elem="shaft",
        outer_elem="torque_tube",
        margin=0.0,
        name="propshaft is radially enclosed by torque tube",
    )
    ctx.expect_overlap(
        propshaft,
        axle,
        axes="y",
        elem_a="shaft",
        elem_b="torque_tube",
        min_overlap=1.30,
        name="torque tube covers the propshaft length",
    )
    ctx.expect_contact(
        propshaft,
        axle,
        elem_a="shaft",
        elem_b="front_bearing_saddle",
        contact_tol=0.0005,
        name="front hidden bearing supports propshaft",
    )

    ctx.expect_gap(
        hub_0,
        axle,
        axis="x",
        positive_elem="inner_bearing",
        negative_elem="flange_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub_0 seats on its flange face",
    )
    ctx.expect_gap(
        axle,
        hub_1,
        axis="x",
        positive_elem="flange_1",
        negative_elem="inner_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub_1 seats on its flange face",
    )
    ctx.expect_overlap(
        hub_0,
        axle,
        axes="yz",
        elem_a="inner_bearing",
        elem_b="flange_0",
        min_overlap=0.10,
        name="hub_0 bearing is centered on flange face",
    )
    ctx.expect_overlap(
        hub_1,
        axle,
        axes="yz",
        elem_a="inner_bearing",
        elem_b="flange_1",
        min_overlap=0.10,
        name="hub_1 bearing is centered on flange face",
    )

    with ctx.pose({hub_0_joint: 0.9, hub_1_joint: -0.9, propshaft_joint: 1.2}):
        ctx.expect_gap(
            hub_0,
            axle,
            axis="x",
            positive_elem="inner_bearing",
            negative_elem="flange_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="spun hub_0 stays on flange face",
        )
        ctx.expect_gap(
            axle,
            hub_1,
            axis="x",
            positive_elem="flange_1",
            negative_elem="inner_bearing",
            max_gap=0.001,
            max_penetration=0.0,
            name="spun hub_1 stays on flange face",
        )

    return ctx.report()


object_model = build_object_model()
