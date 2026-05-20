from __future__ import annotations

import math

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
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_operator_chair")

    black = model.material("mat_black_plastic", rgba=(0.02, 0.022, 0.024, 1.0))
    charcoal = model.material("mat_charcoal_fabric", rgba=(0.10, 0.12, 0.14, 1.0))
    dark_trim = model.material("mat_dark_trim", rgba=(0.015, 0.016, 0.017, 1.0))
    metal = model.material("mat_satin_metal", rgba=(0.45, 0.47, 0.48, 1.0))

    seat_cushion_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.44, 0.42, 0.075, corner_segments=10),
            0.065,
        ),
        "rounded_seat_cushion",
    )
    back_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.38, 0.46, 0.060, corner_segments=10),
            0.055,
        ),
        "rounded_back_pad",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black,
        name="central_hub",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=metal,
        name="column",
    )
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        cx = 0.170 * math.cos(angle)
        cy = 0.170 * math.sin(angle)
        tx = 0.312 * math.cos(angle)
        ty = 0.312 * math.sin(angle)
        base.visual(
            Box((0.300, 0.052, 0.030)),
            origin=Origin(xyz=(cx, cy, 0.095), rpy=(0.0, 0.0, angle)),
            material=black,
            name=f"star_leg_{i}",
        )
        base.visual(
            Cylinder(radius=0.029, length=0.020),
            origin=Origin(xyz=(tx, ty, 0.090)),
            material=black,
            name=f"caster_pad_{i}",
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=metal,
        name="swivel_plug",
    )
    seat.visual(
        Box((0.315, 0.280, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=black,
        name="underseat_plate",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=charcoal,
        name="seat_cushion",
    )
    # Rear metal ears and segmented hinge barrels leave a center gap for the
    # backrest knuckle, making the hinge readable without broad overlap.
    for side, y in enumerate((-0.142, 0.142)):
        seat.visual(
            Box((0.052, 0.030, 0.085)),
            origin=Origin(xyz=(-0.225, y, 0.096)),
            material=metal,
            name=f"hinge_ear_{side}",
        )
        seat.visual(
            Cylinder(radius=0.018, length=0.072),
            origin=Origin(
                xyz=(-0.238, y, 0.130),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"hinge_barrel_{side}",
        )
    seat.visual(
        Box((0.060, 0.095, 0.030)),
        origin=Origin(xyz=(0.070, 0.178, 0.044)),
        material=black,
        name="lever_mount",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.017, length=0.218),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="center_hinge_barrel",
    )
    backrest.visual(
        back_pad_mesh,
        origin=Origin(
            xyz=(-0.028, 0.0, 0.248),
            rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
        ),
        material=charcoal,
        name="back_pad",
    )
    backrest.visual(
        Box((0.030, 0.180, 0.060)),
        origin=Origin(xyz=(-0.008, 0.0, 0.030)),
        material=metal,
        name="hinge_to_pad_web",
    )

    lever = model.part("lever")
    lever.visual(
        Box((0.018, 0.150, 0.012)),
        origin=Origin(xyz=(0.0, 0.075, -0.006)),
        material=metal,
        name="lever_arm",
    )
    lever.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.160, -0.008)),
        material=dark_trim,
        name="lever_tip",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.238, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.12, upper=0.55),
    )
    model.articulation(
        "seat_to_lever",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.070, 0.225, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.50, upper=0.35),
    )

    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        tx = 0.312 * math.cos(angle)
        ty = 0.312 * math.sin(angle)

        yoke = model.part(f"caster_yoke_{i}")
        yoke.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=metal,
            name="swivel_disk",
        )
        yoke.visual(
            Box((0.060, 0.070, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=metal,
            name="top_bridge",
        )
        for side, y in enumerate((-0.029, 0.029)):
            yoke.visual(
                Box((0.045, 0.010, 0.070)),
                origin=Origin(xyz=(0.0, y, -0.050)),
                material=metal,
                name=f"side_plate_{side}",
            )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            Cylinder(radius=0.030, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.036),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="wheel_hub",
        )
        wheel.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="axle_end",
        )

        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=yoke,
            origin=Origin(xyz=(tx, ty, 0.080)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )
        model.articulation(
            f"caster_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    lever = object_model.get_part("lever")
    swivel = object_model.get_articulation("base_to_seat")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    lever_pivot = object_model.get_articulation("seat_to_lever")

    ctx.expect_contact(
        seat,
        base,
        elem_a="swivel_plug",
        elem_b="column",
        contact_tol=0.001,
        name="seat swivel plug is seated on the column",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="swivel_plug",
        elem_b="column",
        min_overlap=0.020,
        name="seat spindle is centered over the support column",
    )

    caster_swivels = [
        joint for joint in object_model.articulations if joint.name.startswith("base_to_caster_")
    ]
    wheel_spins = [
        joint for joint in object_model.articulations if joint.name.startswith("caster_to_wheel_")
    ]
    ctx.check(
        "five caster swivel joints are present",
        len(caster_swivels) == 5,
        details=f"found {len(caster_swivels)} caster swivel joints",
    )
    ctx.check(
        "five caster wheel spin joints are present",
        len(wheel_spins) == 5,
        details=f"found {len(wheel_spins)} wheel spin joints",
    )
    ctx.expect_within(
        "caster_wheel_0",
        "caster_yoke_0",
        axes="y",
        inner_elem="tire",
        outer_elem="top_bridge",
        margin=0.002,
        name="caster wheel is captured between yoke side plates",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.45}):
        reclined_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "positive backrest hinge reclines rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.050,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )

    rest_lever_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: 0.30}):
        raised_lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "under-seat lever rotates upward on its pivot",
        rest_lever_aabb is not None
        and raised_lever_aabb is not None
        and raised_lever_aabb[1][2] > rest_lever_aabb[1][2] + 0.020,
        details=f"rest={rest_lever_aabb}, raised={raised_lever_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "chair seat swivels about the vertical column",
        rest_seat_aabb is not None
        and turned_seat_aabb is not None
        and abs((rest_seat_aabb[1][0] - rest_seat_aabb[0][0]) - (turned_seat_aabb[1][1] - turned_seat_aabb[0][1])) < 0.030,
        details=f"rest={rest_seat_aabb}, turned={turned_seat_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
