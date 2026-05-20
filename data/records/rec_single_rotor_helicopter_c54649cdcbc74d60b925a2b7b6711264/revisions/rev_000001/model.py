from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="training_helicopter")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    training_red = model.material("training_red", rgba=(0.78, 0.08, 0.08, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.18, 0.22, 0.27, 0.78))
    rotor_gray = model.material("rotor_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    skid_black = model.material("skid_black", rgba=(0.08, 0.08, 0.09, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        Box((1.75, 1.00, 0.64)),
        origin=Origin(xyz=(0.20, 0.0, 0.63)),
        material=body_white,
        name="cabin_shell",
    )
    airframe.visual(
        Cylinder(radius=0.34, length=0.80),
        origin=Origin(xyz=(1.28, 0.0, 0.74), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_white,
        name="nose_shell",
    )
    airframe.visual(
        Box((0.95, 0.42, 0.18)),
        origin=Origin(xyz=(-0.08, 0.0, 0.93)),
        material=body_white,
        name="roof_spine",
    )
    airframe.visual(
        Cylinder(radius=0.16, length=0.74),
        origin=Origin(xyz=(-0.96, 0.0, 0.70), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_white,
        name="tail_cone",
    )
    airframe.visual(
        Cylinder(radius=0.065, length=2.85),
        origin=Origin(xyz=(-2.36, 0.0, 0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="tail_boom",
    )
    airframe.visual(
        Box((0.24, 0.055, 0.80)),
        origin=Origin(xyz=(-3.89, 0.0, 1.02)),
        material=body_white,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.52, 0.03, 0.16)),
        origin=Origin(xyz=(-3.46, 0.0, 0.72)),
        material=body_white,
        name="tailplane",
    )
    airframe.visual(
        Box((0.82, 0.92, 0.46)),
        origin=Origin(xyz=(0.67, 0.0, 0.83)),
        material=canopy_tint,
        name="canopy",
    )
    airframe.visual(
        Box((1.82, 0.94, 0.10)),
        origin=Origin(xyz=(0.16, 0.0, 0.58)),
        material=training_red,
        name="cabin_stripe",
    )
    airframe.visual(
        Cylinder(radius=0.045, length=2.20),
        origin=Origin(xyz=(-0.05, 0.62, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=skid_black,
        name="skid_0",
    )
    airframe.visual(
        Cylinder(radius=0.045, length=2.20),
        origin=Origin(xyz=(-0.05, -0.62, 0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=skid_black,
        name="skid_1",
    )
    airframe.visual(
        Cylinder(radius=0.035, length=1.26),
        origin=Origin(xyz=(0.35, 0.0, 0.18), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="front_cross_tube",
    )
    airframe.visual(
        Cylinder(radius=0.035, length=1.26),
        origin=Origin(xyz=(-0.65, 0.0, 0.18), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="rear_cross_tube",
    )
    for name, x, y in (
        ("front_strut_0", 0.35, 0.30),
        ("front_strut_1", 0.35, -0.30),
        ("rear_strut_0", -0.65, 0.30),
        ("rear_strut_1", -0.65, -0.30),
    ):
        airframe.visual(
            Cylinder(radius=0.03, length=0.22),
            origin=Origin(xyz=(x, y, 0.27)),
            material=metal_gray,
            name=name,
        )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=metal_gray,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.10, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=metal_gray,
        name="hub",
    )
    main_rotor.visual(
        Box((3.70, 0.18, 0.05)),
        origin=Origin(xyz=(1.85, 0.0, 0.19)),
        material=rotor_gray,
        name="blade_0",
    )
    main_rotor.visual(
        Box((3.70, 0.18, 0.05)),
        origin=Origin(xyz=(-1.85, 0.0, 0.19)),
        material=rotor_gray,
        name="blade_1",
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.02, length=0.16),
        origin=Origin(xyz=(0.0, 0.08, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="shaft",
    )
    tail_rotor.visual(
        Cylinder(radius=0.05, length=0.05),
        origin=Origin(xyz=(0.0, 0.12, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.10, 0.03, 0.38)),
        origin=Origin(xyz=(0.0, 0.12, 0.19)),
        material=rotor_gray,
        name="tail_blade_0",
    )
    tail_rotor.visual(
        Box((0.10, 0.03, 0.38)),
        origin=Origin(xyz=(0.0, 0.12, -0.19)),
        material=rotor_gray,
        name="tail_blade_1",
    )

    transmission_cover = model.part("transmission_cover")
    transmission_cover.visual(
        Box((0.46, 0.32, 0.10)),
        origin=Origin(xyz=(0.23, 0.0, -0.05)),
        material=body_white,
        name="cover_shell",
    )
    transmission_cover.visual(
        Box((0.16, 0.10, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=training_red,
        name="cover_fairing",
    )

    baggage_hatch = model.part("baggage_hatch")
    baggage_hatch.visual(
        Box((0.52, 0.024, 0.42)),
        origin=Origin(xyz=(0.26, 0.012, 0.0)),
        material=body_white,
        name="hatch_panel",
    )
    baggage_hatch.visual(
        Box((0.07, 0.012, 0.08)),
        origin=Origin(xyz=(0.39, 0.024, 0.0)),
        material=metal_gray,
        name="hatch_handle",
    )

    model.articulation(
        "airframe_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(0.02, 0.0, 1.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=40.0),
    )
    model.articulation(
        "airframe_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-3.82, 0.0275, 1.00)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=70.0),
    )
    model.articulation(
        "airframe_to_transmission_cover",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=transmission_cover,
        origin=Origin(xyz=(-0.68, 0.0, 1.12)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "airframe_to_baggage_hatch",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=baggage_hatch,
        origin=Origin(xyz=(-0.68, 0.50, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    transmission_cover = object_model.get_part("transmission_cover")
    baggage_hatch = object_model.get_part("baggage_hatch")

    main_rotor_joint = object_model.get_articulation("airframe_to_main_rotor")
    tail_rotor_joint = object_model.get_articulation("airframe_to_tail_rotor")
    cover_hinge = object_model.get_articulation("airframe_to_transmission_cover")
    hatch_hinge = object_model.get_articulation("airframe_to_baggage_hatch")

    def extents(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    with ctx.pose({cover_hinge: 0.0, hatch_hinge: 0.0}):
        ctx.expect_gap(
            transmission_cover,
            airframe,
            axis="z",
            positive_elem="cover_shell",
            negative_elem="roof_spine",
            max_gap=0.001,
            max_penetration=0.0,
            name="transmission cover seats on the roof spine",
        )
        ctx.expect_overlap(
            transmission_cover,
            airframe,
            axes="xy",
            elem_a="cover_shell",
            elem_b="roof_spine",
            min_overlap=0.20,
            name="transmission cover stays centered over the doghouse",
        )
        ctx.expect_gap(
            baggage_hatch,
            airframe,
            axis="y",
            positive_elem="hatch_panel",
            negative_elem="cabin_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="baggage hatch closes against the aft fuselage side",
        )
        ctx.expect_overlap(
            baggage_hatch,
            airframe,
            axes="xz",
            elem_a="hatch_panel",
            elem_b="cabin_shell",
            min_overlap=0.18,
            name="baggage hatch stays within the fuselage opening footprint",
        )
        closed_cover_aabb = ctx.part_element_world_aabb(transmission_cover, elem="cover_shell")
        closed_hatch_aabb = ctx.part_element_world_aabb(baggage_hatch, elem="hatch_panel")
        main_rotor_rest = ctx.part_world_aabb(main_rotor)
        tail_rotor_rest = ctx.part_world_aabb(tail_rotor)

    with ctx.pose({cover_hinge: 1.05}):
        open_cover_aabb = ctx.part_element_world_aabb(transmission_cover, elem="cover_shell")

    with ctx.pose({hatch_hinge: 1.15}):
        open_hatch_aabb = ctx.part_element_world_aabb(baggage_hatch, elem="hatch_panel")

    with ctx.pose({main_rotor_joint: math.pi / 2.0}):
        main_rotor_turn = ctx.part_world_aabb(main_rotor)

    with ctx.pose({tail_rotor_joint: math.pi / 2.0}):
        tail_rotor_turn = ctx.part_world_aabb(tail_rotor)

    ctx.check(
        "transmission cover lifts upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.18,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )
    ctx.check(
        "baggage hatch swings outward",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][1] > closed_hatch_aabb[1][1] + 0.18,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    main_rotor_rest_extents = extents(main_rotor_rest)
    main_rotor_turn_extents = extents(main_rotor_turn)
    ctx.check(
        "main rotor turns about the mast axis",
        main_rotor_rest_extents is not None
        and main_rotor_turn_extents is not None
        and main_rotor_rest_extents[0] > 7.1
        and main_rotor_rest_extents[1] < 0.5
        and main_rotor_turn_extents[1] > 7.1
        and main_rotor_turn_extents[0] < 0.5,
        details=f"rest={main_rotor_rest_extents}, turned={main_rotor_turn_extents}",
    )

    tail_rotor_rest_extents = extents(tail_rotor_rest)
    tail_rotor_turn_extents = extents(tail_rotor_turn)
    ctx.check(
        "tail rotor turns about the transverse tail axis",
        tail_rotor_rest_extents is not None
        and tail_rotor_turn_extents is not None
        and tail_rotor_rest_extents[2] > 0.70
        and tail_rotor_rest_extents[0] < 0.20
        and tail_rotor_turn_extents[0] > 0.70
        and tail_rotor_turn_extents[2] < 0.20,
        details=f"rest={tail_rotor_rest_extents}, turned={tail_rotor_turn_extents}",
    )

    return ctx.report()


object_model = build_object_model()
