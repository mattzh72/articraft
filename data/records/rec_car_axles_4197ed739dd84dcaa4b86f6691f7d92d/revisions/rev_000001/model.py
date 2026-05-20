from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="live_rear_axle")

    cast_iron = model.material("dark_cast_iron", color=(0.08, 0.085, 0.08, 1.0))
    cover_black = model.material("stamped_black_cover", color=(0.015, 0.017, 0.016, 1.0))
    machined = model.material("machined_steel", color=(0.66, 0.64, 0.58, 1.0))
    rubber = model.material("black_bearing_seal", color=(0.01, 0.01, 0.01, 1.0))

    axle = model.part("axle_housing")

    # Integral cast differential "pumpkin" in the middle of the live axle.
    pumpkin = SphereGeometry(1.0, width_segments=56, height_segments=28).scale(0.30, 0.24, 0.28)
    axle.visual(
        mesh_from_geometry(pumpkin, "differential_pumpkin"),
        material=cast_iron,
        name="center_bowl",
    )

    # Two axle tubes are welded into the central housing and terminate in bearing ends.
    for side, x_sign in (("0", 1.0), ("1", -1.0)):
        bearing_name = "bearing_0" if side == "0" else "bearing_1"
        seal_name = "seal_0" if side == "0" else "seal_1"
        axle.visual(
            Cylinder(radius=0.050, length=0.76),
            origin=Origin(xyz=(x_sign * 0.60, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=cast_iron,
            name=f"tube_{side}",
        )
        axle.visual(
            Cylinder(radius=0.072, length=0.16),
            origin=Origin(xyz=(x_sign * 0.90, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=cast_iron,
            name=f"tube_sleeve_{side}",
        )
        axle.visual(
            Cylinder(radius=0.145, length=0.030),
            origin=Origin(xyz=(x_sign * 0.985, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=cast_iron,
            name=bearing_name,
        )
        axle.visual(
            Cylinder(radius=0.112, length=0.010),
            origin=Origin(xyz=(x_sign * 0.9975, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=seal_name,
        )
        axle.visual(
            Box((0.20, 0.11, 0.040)),
            origin=Origin(xyz=(x_sign * 0.57, 0.0, 0.060)),
            material=cast_iron,
            name=f"spring_pad_{side}",
        )

    # Rear stamped cover and cover bolts on the back of the differential.
    axle.visual(
        Cylinder(radius=0.225, length=0.050),
        origin=Origin(xyz=(0.0, 0.245, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cover_black,
        name="rear_cover",
    )
    for i in range(10):
        theta = 2.0 * pi * i / 10.0
        axle.visual(
            Cylinder(radius=0.011, length=0.024),
            origin=Origin(
                xyz=(0.182 * cos(theta), 0.276, 0.182 * sin(theta)),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=machined,
            name=f"cover_bolt_{i}",
        )
    axle.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.278, 0.055), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="fill_plug",
    )

    # Front pinion bearing snout: a stationary housing boss for the rotating input.
    axle.visual(
        Cylinder(radius=0.118, length=0.120),
        origin=Origin(xyz=(0.0, -0.300, 0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="front_bearing",
    )
    axle.visual(
        Cylinder(radius=0.067, length=0.010),
        origin=Origin(xyz=(0.0, -0.355, 0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="pinion_seal",
    )
    axle.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.288)),
        material=machined,
        name="drain_plug",
    )

    pinion = model.part("pinion_shaft")
    pinion.visual(
        Cylinder(radius=0.033, length=0.360),
        origin=Origin(xyz=(0.0, -0.065, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="shaft",
    )
    pinion.visual(
        Cylinder(radius=0.078, length=0.038),
        origin=Origin(xyz=(0.0, -0.255, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="flange",
    )
    pinion.visual(
        Box((0.130, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.292, 0.0)),
        material=machined,
        name="yoke_web",
    )
    for i, x in enumerate((-0.052, 0.052)):
        pinion.visual(
            Box((0.028, 0.130, 0.100)),
            origin=Origin(xyz=(x, -0.365, 0.0)),
            material=machined,
            name=f"yoke_ear_{i}",
        )

    def add_hub(name: str, outward_sign: float) -> None:
        hub = model.part(name)
        hub.visual(
            Cylinder(radius=0.142, length=0.070),
            origin=Origin(xyz=(outward_sign * 0.0375, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name="flange",
        )
        hub.visual(
            Cylinder(radius=0.060, length=0.090),
            origin=Origin(xyz=(outward_sign * 0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name="pilot",
        )
        hub.visual(
            Cylinder(radius=0.098, length=0.014),
            origin=Origin(xyz=(outward_sign * 0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="bearing_seal",
        )
        for i in range(5):
            theta = 2.0 * pi * i / 5.0
            hub.visual(
                Cylinder(radius=0.009, length=0.060),
                origin=Origin(
                    xyz=(outward_sign * 0.090, 0.105 * cos(theta), 0.105 * sin(theta)),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=machined,
                name=f"stud_{i}",
            )

    add_hub("hub_0", 1.0)
    add_hub("hub_1", -1.0)

    model.articulation(
        "pinion_spin",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=pinion,
        origin=Origin(xyz=(0.0, -0.355, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=80.0),
    )
    model.articulation(
        "hub_0_spin",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child="hub_0",
        origin=Origin(xyz=(1.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=100.0),
    )
    model.articulation(
        "hub_1_spin",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child="hub_1",
        origin=Origin(xyz=(-1.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=100.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_part("axle_housing")
    pinion = object_model.get_part("pinion_shaft")
    hub_0 = object_model.get_part("hub_0")
    hub_1 = object_model.get_part("hub_1")
    pinion_spin = object_model.get_articulation("pinion_spin")
    hub_0_spin = object_model.get_articulation("hub_0_spin")
    hub_1_spin = object_model.get_articulation("hub_1_spin")

    ctx.allow_overlap(
        axle,
        pinion,
        elem_a="front_bearing",
        elem_b="shaft",
        reason="The rotating pinion shaft is intentionally captured inside the simplified solid bearing snout.",
    )
    ctx.allow_overlap(
        axle,
        pinion,
        elem_a="pinion_seal",
        elem_b="shaft",
        reason="The lip seal is modeled as a thin solid ring compressed around the rotating pinion shaft.",
    )

    ctx.check(
        "pinion input is continuous",
        pinion_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in pinion_spin.axis) == (0.0, -1.0, 0.0),
        details=f"type={pinion_spin.articulation_type}, axis={pinion_spin.axis}",
    )
    ctx.check(
        "wheel hubs are continuous",
        hub_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and hub_1_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={hub_0_spin.articulation_type}, {hub_1_spin.articulation_type}",
    )
    ctx.check(
        "hub axes follow axle tubes",
        tuple(round(v, 6) for v in hub_0_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 6) for v in hub_1_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axes={hub_0_spin.axis}, {hub_1_spin.axis}",
    )

    ctx.expect_gap(
        hub_0,
        axle,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="flange",
        negative_elem="seal_0",
        name="hub_0 seated on bearing face",
    )
    ctx.expect_gap(
        axle,
        hub_1,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="seal_1",
        negative_elem="flange",
        name="hub_1 seated on bearing face",
    )
    ctx.expect_overlap(
        hub_0,
        axle,
        axes="yz",
        min_overlap=0.18,
        elem_a="flange",
        elem_b="bearing_0",
        name="hub_0 centered on tube bearing",
    )
    ctx.expect_overlap(
        hub_1,
        axle,
        axes="yz",
        min_overlap=0.18,
        elem_a="flange",
        elem_b="bearing_1",
        name="hub_1 centered on tube bearing",
    )
    ctx.expect_overlap(
        pinion,
        axle,
        axes="y",
        min_overlap=0.10,
        elem_a="shaft",
        elem_b="front_bearing",
        name="pinion shaft retained in front bearing",
    )
    ctx.expect_overlap(
        pinion,
        axle,
        axes="xz",
        min_overlap=0.055,
        elem_a="shaft",
        elem_b="front_bearing",
        name="pinion shaft coaxial with bearing snout",
    )

    rest_stud = ctx.part_element_world_aabb(hub_0, elem="stud_0")
    with ctx.pose({hub_0_spin: pi / 2.0}):
        spun_stud = ctx.part_element_world_aabb(hub_0, elem="stud_0")
    ctx.check(
        "hub lug visibly sweeps with rotation",
        rest_stud is not None
        and spun_stud is not None
        and spun_stud[0][2] > rest_stud[1][2] + 0.045,
        details=f"rest={rest_stud}, spun={spun_stud}",
    )

    rest_yoke = ctx.part_element_world_aabb(pinion, elem="yoke_ear_1")
    with ctx.pose({pinion_spin: pi / 2.0}):
        spun_yoke = ctx.part_element_world_aabb(pinion, elem="yoke_ear_1")
    ctx.check(
        "pinion yoke visibly sweeps with rotation",
        rest_yoke is not None
        and spun_yoke is not None
        and abs(spun_yoke[0][2] - rest_yoke[0][2]) > 0.035,
        details=f"rest={rest_yoke}, spun={spun_yoke}",
    )

    return ctx.report()


object_model = build_object_model()
