from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_shuttle")

    body_mat = model.material("charcoal_powder_coat", rgba=(0.07, 0.08, 0.09, 1.0))
    deck_mat = model.material("dark_cast_aluminum", rgba=(0.16, 0.17, 0.18, 1.0))
    rail_mat = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    carriage_mat = model.material("blue_gray_anodized", rgba=(0.22, 0.28, 0.34, 1.0))
    spindle_mat = model.material("satin_spindle_steel", rgba=(0.74, 0.76, 0.75, 1.0))
    dark_mat = model.material("blackened_tool_steel", rgba=(0.035, 0.035, 0.038, 1.0))
    accent_mat = model.material("service_blue", rgba=(0.0, 0.28, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.62, 0.25, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=body_mat,
        name="base_pan",
    )
    body.visual(
        Box((0.56, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=deck_mat,
        name="raised_deck",
    )
    body.visual(
        Box((0.52, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, -0.06, 0.100)),
        material=rail_mat,
        name="rail_0",
    )
    body.visual(
        Box((0.52, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.06, 0.100)),
        material=rail_mat,
        name="rail_1",
    )
    for idx, x in enumerate((-0.285, 0.285)):
        body.visual(
            Box((0.035, 0.18, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.1125)),
            material=dark_mat,
            name=f"end_stop_{idx}",
        )
    body.visual(
        Box((0.20, 0.006, 0.030)),
        origin=Origin(xyz=(-0.06, 0.124, 0.055)),
        material=accent_mat,
        name="service_panel",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.155, 0.070)),
        origin=Origin(),
        material=carriage_mat,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.16, 0.105, 0.025)),
        origin=Origin(xyz=(-0.025, 0.0, 0.047)),
        material=carriage_mat,
        name="top_cover",
    )
    carriage.visual(
        Box((0.090, 0.006, 0.011)),
        origin=Origin(xyz=(-0.030, 0.052, 0.056)),
        material=accent_mat,
        name="travel_mark",
    )

    # A fixed square bearing collar at the working end makes the separate
    # rotating spindle read as captured by the short sliding carriage.
    collar_x = 0.128
    collar_z = 0.060
    carriage.visual(
        Box((0.038, 0.116, 0.014)),
        origin=Origin(xyz=(collar_x, 0.0, collar_z + 0.029)),
        material=carriage_mat,
        name="bearing_top",
    )
    carriage.visual(
        Box((0.038, 0.116, 0.014)),
        origin=Origin(xyz=(collar_x, 0.0, collar_z - 0.029)),
        material=carriage_mat,
        name="bearing_bottom",
    )
    for idx, y in enumerate((-0.029, 0.029)):
        carriage.visual(
            Box((0.038, 0.014, 0.076)),
            origin=Origin(xyz=(collar_x, y, collar_z)),
            material=carriage_mat,
            name=f"bearing_side_{idx}",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.080),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="cartridge",
    )
    spindle.visual(
        Box((0.050, 0.008, 0.006)),
        origin=Origin(xyz=(0.048, 0.0, 0.025)),
        material=accent_mat,
        name="index_mark",
    )
    spindle.visual(
        mesh_from_geometry(ConeGeometry(0.012, 0.052, radial_segments=32), "spindle_tip"),
        origin=Origin(xyz=(0.104, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="tip",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.12, 0.0, 0.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.22),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(collar_x, 0.0, collar_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=40.0, lower=-math.pi, upper=math.pi
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("body_to_carriage")
    spin = object_model.get_articulation("carriage_to_spindle")

    def aabb_size(aabb):
        lo, hi = aabb
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def aabb_center(aabb):
        lo, hi = aabb
        return (
            0.5 * (lo[0] + hi[0]),
            0.5 * (lo[1] + hi[1]),
            0.5 * (lo[2] + hi[2]),
        )

    ctx.check(
        "one slider and one spindle joint",
        len(object_model.parts) == 3
        and slide.articulation_type == ArticulationType.PRISMATIC
        and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"parts={len(object_model.parts)}, slide={slide.articulation_type}, spin={spin.articulation_type}",
    )

    ctx.expect_gap(
        carriage,
        body,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=1e-5,
        name="carriage rests on guide rail",
    )
    ctx.expect_overlap(
        carriage,
        body,
        axes="xy",
        elem_a="carriage_block",
        elem_b="rail_1",
        min_overlap=0.015,
        name="carriage footprint spans rail",
    )

    carriage_size = aabb_size(ctx.part_element_world_aabb(carriage, elem="carriage_block"))
    cartridge_size = aabb_size(ctx.part_element_world_aabb(spindle, elem="cartridge"))
    ctx.check(
        "spindle cartridge is visibly smaller than carriage",
        cartridge_size[1] < carriage_size[1] * 0.40
        and cartridge_size[2] < carriage_size[2] * 0.75,
        details=f"cartridge={cartridge_size}, carriage={carriage_size}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            body,
            axis="z",
            positive_elem="carriage_block",
            negative_elem="rail_1",
            max_gap=0.001,
            max_penetration=1e-5,
            name="extended carriage remains on guide rail",
        )
    ctx.check(
        "prismatic carriage travels forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    mark_rest = aabb_center(ctx.part_element_world_aabb(spindle, elem="index_mark"))
    with ctx.pose({spin: math.pi / 2.0}):
        mark_turned = aabb_center(ctx.part_element_world_aabb(spindle, elem="index_mark"))
        spindle_pos_turned = ctx.part_world_position(spindle)
    spindle_pos = ctx.part_world_position(spindle)
    ctx.check(
        "spindle rotates about its own axis",
        spindle_pos is not None
        and spindle_pos_turned is not None
        and abs(spindle_pos_turned[0] - spindle_pos[0]) < 1e-6
        and abs(mark_turned[1] - mark_rest[1]) > 0.015,
        details=f"mark_rest={mark_rest}, mark_turned={mark_turned}",
    )

    return ctx.report()


object_model = build_object_model()
