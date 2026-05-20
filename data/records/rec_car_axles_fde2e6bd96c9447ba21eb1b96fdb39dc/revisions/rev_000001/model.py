from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_axle_rear_suspension")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.10, 0.11, 0.11, 1.0))
    black_paint = model.material("black_paint", rgba=(0.025, 0.027, 0.030, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.30, 0.32, 0.33, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.60, 0.61, 0.58, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.78, 0.76, 0.70, 1.0))

    center_beam = model.part("center_beam")
    center_beam.visual(
        Box((0.22, 1.05, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=black_paint,
        name="main_beam",
    )
    center_beam.visual(
        Box((0.52, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.242)),
        material=black_paint,
        name="saddle_plate",
    )
    center_beam.visual(
        Box((0.10, 0.42, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.257)),
        material=black_paint,
        name="center_web",
    )
    center_beam.visual(
        Box((0.46, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.145, -0.257)),
        material=black_paint,
        name="front_gusset",
    )
    center_beam.visual(
        Box((0.46, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, -0.145, -0.257)),
        material=black_paint,
        name="rear_gusset",
    )
    center_beam.inertial = Inertial.from_geometry(
        Box((0.55, 1.05, 0.22)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
    )

    differential = model.part("differential")
    differential.visual(
        Sphere(radius=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=cast_iron,
        name="pumpkin",
    )
    differential.visual(
        Cylinder(radius=0.155, length=0.045),
        origin=Origin(xyz=(0.0, -0.185, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="rear_cover",
    )
    differential.visual(
        Cylinder(radius=0.115, length=0.140),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_iron,
        name="output_bell_0",
    )
    differential.visual(
        Cylinder(radius=0.115, length=0.140),
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_iron,
        name="output_bell_1",
    )
    differential.visual(
        Cylinder(radius=0.140, length=0.025),
        origin=Origin(xyz=(0.2375, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_cast,
        name="pivot_face_0",
    )
    differential.visual(
        Cylinder(radius=0.140, length=0.025),
        origin=Origin(xyz=(-0.2375, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_cast,
        name="pivot_face_1",
    )
    differential.visual(
        Box((0.46, 0.30, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.197)),
        material=cast_iron,
        name="mount_flange",
    )
    differential.visual(
        Box((0.36, 0.09, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=cast_iron,
        name="lower_rib",
    )
    for index, (x, y) in enumerate(
        ((0.165, 0.105), (-0.165, 0.105), (0.165, -0.105), (-0.165, -0.105))
    ):
        differential.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(x, y, -0.166)),
            material=bolt_steel,
            name=f"mount_bolt_{index}",
        )
    for index in range(8):
        angle = 2.0 * pi * index / 8.0
        differential.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(
                xyz=(0.118 * cos(angle), -0.212, 0.118 * sin(angle)),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=bolt_steel,
            name=f"cover_bolt_{index}",
        )
        differential.visual(
            Box((0.018, 0.006, 0.018)),
            origin=Origin(
                xyz=(0.118 * cos(angle), -0.204, 0.118 * sin(angle))
            ),
            material=bolt_steel,
            name=f"cover_washer_{index}",
        )
    differential.inertial = Inertial.from_geometry(
        Sphere(radius=0.22),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    def add_half_axle(name: str, side: float):
        tube = model.part(name)
        tube.visual(
            Cylinder(radius=0.074, length=0.100),
            origin=Origin(xyz=(side * 0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name="inner_pivot_collar",
        )
        tube.visual(
            Cylinder(radius=0.045, length=0.640),
            origin=Origin(xyz=(side * 0.360, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name="axle_tube",
        )
        tube.visual(
            Cylinder(radius=0.070, length=0.060),
            origin=Origin(xyz=(side * 0.175, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name="welded_sleeve",
        )
        tube.visual(
            Cylinder(radius=0.095, length=0.120),
            origin=Origin(xyz=(side * 0.660, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_cast,
            name="outer_bearing",
        )
        tube.visual(
            Box((0.160, 0.050, 0.060)),
            origin=Origin(xyz=(side * 0.620, 0.0, -0.055)),
            material=axle_steel,
            name="bearing_tab",
        )
        tube.inertial = Inertial.from_geometry(
            Cylinder(radius=0.06, length=0.72),
            mass=18.0,
            origin=Origin(xyz=(side * 0.35, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        )
        return tube

    axle_tube_0 = add_half_axle("axle_tube_0", 1.0)
    axle_tube_1 = add_half_axle("axle_tube_1", -1.0)

    def add_hub(name: str, side: float):
        hub = model.part(name)
        hub.visual(
            Cylinder(radius=0.130, length=0.080),
            origin=Origin(xyz=(side * 0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_cast,
            name="brake_drum",
        )
        hub.visual(
            Cylinder(radius=0.165, length=0.060),
            origin=Origin(xyz=(side * 0.105, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_steel,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.070, length=0.090),
            origin=Origin(xyz=(side * 0.145, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_steel,
            name="center_cap",
        )
        for index in range(5):
            angle = 2.0 * pi * index / 5.0
            hub.visual(
                Cylinder(radius=0.014, length=0.060),
                origin=Origin(
                    xyz=(
                        side * 0.155,
                        0.095 * cos(angle),
                        0.095 * sin(angle),
                    ),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=bolt_steel,
                name=f"wheel_stud_{index}",
            )
        hub.inertial = Inertial.from_geometry(
            Cylinder(radius=0.16, length=0.18),
            mass=12.0,
            origin=Origin(xyz=(side * 0.08, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        )
        return hub

    hub_0 = add_hub("hub_0", 1.0)
    hub_1 = add_hub("hub_1", -1.0)

    model.articulation(
        "beam_to_differential",
        ArticulationType.FIXED,
        parent=center_beam,
        child=differential,
        origin=Origin(),
    )
    model.articulation(
        "differential_to_axle_tube_0",
        ArticulationType.REVOLUTE,
        parent=differential,
        child=axle_tube_0,
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1600.0, velocity=1.2, lower=-0.28, upper=0.28),
    )
    model.articulation(
        "differential_to_axle_tube_1",
        ArticulationType.REVOLUTE,
        parent=differential,
        child=axle_tube_1,
        origin=Origin(xyz=(-0.250, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1600.0, velocity=1.2, lower=-0.28, upper=0.28),
    )
    model.articulation(
        "axle_tube_0_to_hub",
        ArticulationType.CONTINUOUS,
        parent=axle_tube_0,
        child=hub_0,
        origin=Origin(xyz=(0.720, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=35.0),
    )
    model.articulation(
        "axle_tube_1_to_hub",
        ArticulationType.CONTINUOUS,
        parent=axle_tube_1,
        child=hub_1,
        origin=Origin(xyz=(-0.720, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("center_beam")
    diff = object_model.get_part("differential")
    axle_0 = object_model.get_part("axle_tube_0")
    axle_1 = object_model.get_part("axle_tube_1")
    hub_0 = object_model.get_part("hub_0")
    hub_1 = object_model.get_part("hub_1")
    swing_0 = object_model.get_articulation("differential_to_axle_tube_0")
    swing_1 = object_model.get_articulation("differential_to_axle_tube_1")
    spin_0 = object_model.get_articulation("axle_tube_0_to_hub")
    spin_1 = object_model.get_articulation("axle_tube_1_to_hub")

    ctx.expect_gap(
        diff,
        beam,
        axis="z",
        positive_elem="mount_flange",
        negative_elem="saddle_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="differential flange sits on center beam saddle",
    )
    ctx.expect_overlap(
        diff,
        beam,
        axes="xy",
        elem_a="mount_flange",
        elem_b="saddle_plate",
        min_overlap=0.25,
        name="differential flange is centered over the beam saddle",
    )

    ctx.expect_gap(
        axle_0,
        diff,
        axis="x",
        positive_elem="inner_pivot_collar",
        negative_elem="pivot_face_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="axle tube 0 pivots at differential end face",
    )
    ctx.expect_overlap(
        axle_0,
        diff,
        axes="yz",
        elem_a="inner_pivot_collar",
        elem_b="pivot_face_0",
        min_overlap=0.10,
        name="axle tube 0 pivot collar is concentric with end face",
    )
    ctx.expect_gap(
        diff,
        axle_1,
        axis="x",
        positive_elem="pivot_face_1",
        negative_elem="inner_pivot_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="axle tube 1 pivots at differential end face",
    )
    ctx.expect_overlap(
        axle_1,
        diff,
        axes="yz",
        elem_a="inner_pivot_collar",
        elem_b="pivot_face_1",
        min_overlap=0.10,
        name="axle tube 1 pivot collar is concentric with end face",
    )

    ctx.expect_gap(
        hub_0,
        axle_0,
        axis="x",
        positive_elem="brake_drum",
        negative_elem="outer_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub 0 seats on axle tube outer bearing",
    )
    ctx.expect_gap(
        axle_1,
        hub_1,
        axis="x",
        positive_elem="outer_bearing",
        negative_elem="brake_drum",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub 1 seats on axle tube outer bearing",
    )

    ctx.check(
        "half axle pivots are horizontal revolute joints",
        swing_0.articulation_type == ArticulationType.REVOLUTE
        and swing_1.articulation_type == ArticulationType.REVOLUTE
        and abs(swing_0.axis[1]) == 1.0
        and abs(swing_1.axis[1]) == 1.0,
        details=f"axes={swing_0.axis}, {swing_1.axis}",
    )
    ctx.check(
        "wheel hubs use continuous spin joints",
        spin_0.articulation_type == ArticulationType.CONTINUOUS
        and spin_1.articulation_type == ArticulationType.CONTINUOUS
        and abs(spin_0.axis[0]) == 1.0
        and abs(spin_1.axis[0]) == 1.0,
        details=f"axes={spin_0.axis}, {spin_1.axis}",
    )

    rest_hub_0 = ctx.part_world_position(hub_0)
    rest_hub_1 = ctx.part_world_position(hub_1)
    with ctx.pose({swing_0: 0.20, swing_1: 0.20}):
        raised_hub_0 = ctx.part_world_position(hub_0)
        raised_hub_1 = ctx.part_world_position(hub_1)
    ctx.check(
        "positive swing raises both outer hubs",
        rest_hub_0 is not None
        and rest_hub_1 is not None
        and raised_hub_0 is not None
        and raised_hub_1 is not None
        and raised_hub_0[2] > rest_hub_0[2] + 0.10
        and raised_hub_1[2] > rest_hub_1[2] + 0.10,
        details=f"rest={rest_hub_0}, {rest_hub_1}; raised={raised_hub_0}, {raised_hub_1}",
    )

    with ctx.pose({spin_0: 1.3, spin_1: -1.1}):
        spun_hub_0 = ctx.part_world_position(hub_0)
        spun_hub_1 = ctx.part_world_position(hub_1)
    ctx.check(
        "hub spin keeps the hub centers at the tube ends",
        rest_hub_0 is not None
        and rest_hub_1 is not None
        and spun_hub_0 is not None
        and spun_hub_1 is not None
        and max(abs(spun_hub_0[i] - rest_hub_0[i]) for i in range(3)) < 1e-6
        and max(abs(spun_hub_1[i] - rest_hub_1[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_hub_0}, {rest_hub_1}; spun={spun_hub_0}, {spun_hub_1}",
    )

    return ctx.report()


object_model = build_object_model()
