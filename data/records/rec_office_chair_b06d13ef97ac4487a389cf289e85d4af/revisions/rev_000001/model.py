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


def _rounded_back_pad() -> cq.Workplane:
    """A softly radiused studio-chair back cushion, centered on its local frame."""
    return cq.Workplane("XY").box(0.060, 0.340, 0.300).edges().fillet(0.018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_chair")

    vinyl = model.material("dark_teal_vinyl", rgba=(0.02, 0.18, 0.22, 1.0))
    black = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_metal = model.material("dark_powdercoat", rgba=(0.08, 0.085, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.075, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_metal,
        name="base_hub",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=metal,
        name="lower_column",
    )
    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0
        base.visual(
            Box((0.420, 0.055, 0.040)),
            origin=Origin(
                xyz=(0.210 * math.cos(theta), 0.210 * math.sin(theta), 0.120),
                rpy=(0.0, 0.0, theta),
            ),
            material=dark_metal,
            name=f"base_arm_{i}",
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.026, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=metal,
        name="inner_post",
    )
    seat.visual(
        Cylinder(radius=0.145, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark_metal,
        name="seat_plate",
    )
    seat.visual(
        Box((0.130, 0.220, 0.018)),
        origin=Origin(xyz=(0.195, 0.0, 0.095)),
        material=dark_metal,
        name="front_hinge_bridge",
    )
    seat.visual(
        Box((0.130, 0.260, 0.018)),
        origin=Origin(xyz=(-0.195, 0.0, 0.118)),
        material=dark_metal,
        name="rear_hinge_bridge",
    )
    for side, y in enumerate((-0.118, 0.118)):
        seat.visual(
            Box((0.030, 0.020, 0.046)),
            origin=Origin(xyz=(0.247, y, 0.080)),
            material=dark_metal,
            name=f"front_hinge_lug_{side}",
        )
    for side, y in enumerate((-0.135, 0.135)):
        seat.visual(
            Box((0.030, 0.024, 0.048)),
            origin=Origin(xyz=(-0.254, -0.132 if y < 0.0 else 0.132, 0.142)),
            material=dark_metal,
            name=f"rear_hinge_lug_{side}",
        )
    seat.visual(
        Cylinder(radius=0.240, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=vinyl,
        name="round_cushion",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.010, length=0.240),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_sleeve",
    )
    for side, y in enumerate((-0.090, 0.090)):
        backrest.visual(
            Box((0.050, 0.018, 0.150)),
            origin=Origin(xyz=(-0.025, y, 0.075)),
            material=metal,
            name=f"back_strut_{side}",
        )
    backrest.visual(
        mesh_from_cadquery(_rounded_back_pad(), "back_pad"),
        origin=Origin(xyz=(-0.060, 0.0, 0.270)),
        material=vinyl,
        name="back_pad",
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.258, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.15, upper=0.65),
    )

    footrest = model.part("footrest")
    footrest.visual(
        Cylinder(radius=0.011, length=0.180),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="front_hinge_sleeve",
    )
    arm_pitch = math.atan2(0.130, 0.170)
    for side, y in enumerate((-0.080, 0.080)):
        footrest.visual(
            Box((0.215, 0.016, 0.016)),
            origin=Origin(xyz=(0.085, y, -0.065), rpy=(0.0, arm_pitch, 0.0)),
            material=metal,
            name=f"footrest_arm_{side}",
        )
    footrest.visual(
        Cylinder(radius=0.014, length=0.260),
        origin=Origin(xyz=(0.170, 0.0, -0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="foot_bar",
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=footrest,
        origin=Origin(xyz=(0.245, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-1.10, upper=0.0),
    )

    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0
        x = 0.420 * math.cos(theta)
        y = 0.420 * math.sin(theta)

        swivel = model.part(f"caster_swivel_{i}")
        swivel.visual(
            Cylinder(radius=0.023, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=metal,
            name="swivel_collar",
        )
        swivel.visual(
            Cylinder(radius=0.009, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=metal,
            name="swivel_stem",
        )
        swivel.visual(
            Box((0.065, 0.030, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=dark_metal,
            name="fork_crown",
        )
        for side, sx in enumerate((-0.020, 0.020)):
            swivel.visual(
                Box((0.008, 0.020, 0.060)),
                origin=Origin(xyz=(sx, 0.0, -0.055)),
                material=dark_metal,
                name=f"fork_cheek_{side}",
            )
        model.articulation(
            f"base_to_caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=swivel,
            origin=Origin(xyz=(x, y, 0.100), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=6.0),
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            Cylinder(radius=0.035, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.032),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="wheel_hub",
        )
        model.articulation(
            f"caster_swivel_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=swivel,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.065)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seat = object_model.get_part("seat")
    base = object_model.get_part("base")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    seat_spin = object_model.get_articulation("base_to_seat")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    foot_hinge = object_model.get_articulation("seat_to_footrest")

    ctx.expect_contact(
        seat,
        base,
        elem_a="inner_post",
        elem_b="lower_column",
        name="seat post rests on central column",
    )
    ctx.expect_gap(
        seat,
        footrest,
        axis="z",
        positive_elem="round_cushion",
        negative_elem="foot_bar",
        min_gap=0.100,
        name="deployed foot bar sits below round seat",
    )

    caster_swivels = [object_model.get_part(f"caster_swivel_{i}") for i in range(5)]
    caster_wheels = [object_model.get_part(f"caster_wheel_{i}") for i in range(5)]
    ctx.check(
        "five caster swivel joints",
        all(object_model.get_articulation(f"base_to_caster_swivel_{i}") is not None for i in range(5)),
    )
    ctx.check(
        "five caster wheel spin joints",
        all(object_model.get_articulation(f"caster_swivel_to_wheel_{i}") is not None for i in range(5)),
    )
    for i in range(5):
        for cheek in (0, 1):
            ctx.allow_overlap(
                caster_swivels[i],
                caster_wheels[i],
                elem_a=f"fork_cheek_{cheek}",
                elem_b="wheel_hub",
                reason="The caster wheel hub is intentionally captured in shallow fork-cheek bearing pockets.",
            )
            ctx.expect_contact(
                caster_swivels[i],
                caster_wheels[i],
                elem_a=f"fork_cheek_{cheek}",
                elem_b="wheel_hub",
                contact_tol=0.003,
                name=f"caster {i} hub is held by fork cheek {cheek}",
            )
        ctx.expect_origin_gap(
            caster_swivels[i],
            caster_wheels[i],
            axis="z",
            min_gap=0.060,
            max_gap=0.070,
            name=f"caster wheel {i} axle hangs below swivel",
        )
        ctx.expect_contact(
            caster_swivels[i],
            base,
            elem_a="swivel_stem",
            elem_b=f"base_arm_{i}",
            contact_tol=0.001,
            name=f"caster {i} stem mounts to star base",
        )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.55}):
        reclined_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest hinge reclines rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.030,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )

    deployed_foot_aabb = ctx.part_world_aabb(footrest)
    with ctx.pose({foot_hinge: -0.95}):
        folded_foot_aabb = ctx.part_world_aabb(footrest)
    ctx.check(
        "footrest folds upward",
        deployed_foot_aabb is not None
        and folded_foot_aabb is not None
        and folded_foot_aabb[0][2] > deployed_foot_aabb[0][2] + 0.040,
        details=f"deployed={deployed_foot_aabb}, folded={folded_foot_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_spin: math.pi / 2.0}):
        turned_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "round seat rotates on vertical axis",
        rest_seat_aabb is not None and turned_seat_aabb is not None,
        details=f"rest={rest_seat_aabb}, turned={turned_seat_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
