from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _body_shell(width: float, depth: float, height: float, wall: float):
    floor = 0.018
    front_sill_h = 0.055
    rail_depth = 0.058
    rail_h = 0.045

    outer = _box_solid((width, depth, height), (0.0, 0.0, height / 2.0))
    cavity_len_y = depth - wall + 0.030
    cavity = _box_solid(
        (width - 2.0 * wall, cavity_len_y, height - floor + 0.030),
        (0.0, (wall + 0.030) / 2.0, (height + floor + 0.030) / 2.0),
    )
    shell = outer.cut(cavity)

    front_sill = _box_solid(
        (width, wall, front_sill_h),
        (0.0, depth / 2.0 - wall / 2.0, front_sill_h / 2.0),
    )
    hinge_rail = _box_solid(
        (width, rail_depth, rail_h),
        (0.0, -depth / 2.0 + rail_depth / 2.0, height + rail_h / 2.0),
    )
    side_lip_h = 0.026
    side_lip_y = depth - rail_depth
    side_lip_z = height + side_lip_h / 2.0
    side_lip_center_y = -depth / 2.0 + rail_depth + side_lip_y / 2.0
    left_lip = _box_solid(
        (wall, side_lip_y, side_lip_h),
        (-width / 2.0 + wall / 2.0, side_lip_center_y, side_lip_z),
    )
    right_lip = _box_solid(
        (wall, side_lip_y, side_lip_h),
        (width / 2.0 - wall / 2.0, side_lip_center_y, side_lip_z),
    )

    return shell.union(front_sill).union(hinge_rail).union(left_lip).union(right_lip)


def _lid_frame(width: float, length: float, thickness: float):
    outer = _box_solid((width, length, thickness), (0.0, length / 2.0, 0.0))
    opening = _box_solid(
        (width - 0.060, length - 0.078, thickness + 0.004),
        (0.0, 0.050 + (length - 0.078) / 2.0, 0.0),
    )
    return outer.cut(opening)


def _cradle_trim_ring(outer_radius: float, inner_radius: float, thickness: float):
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def _soft_cushion(size: tuple[float, float, float]):
    cushion = cq.Workplane("XY").box(*size)
    try:
        return cushion.edges().fillet(0.006)
    except Exception:
        return cushion


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("dark_walnut", rgba=(0.20, 0.105, 0.045, 1.0))
    satin_black = model.material("satin_black", rgba=(0.006, 0.007, 0.008, 1.0))
    velvet = model.material("charcoal_velvet", rgba=(0.025, 0.022, 0.030, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.64, 0.32, 1.0))
    glass = model.material("smoked_glass", rgba=(0.45, 0.62, 0.70, 0.34))
    leather = model.material("black_leather", rgba=(0.015, 0.012, 0.010, 1.0))

    width = 0.180
    depth = 0.170
    height = 0.300
    wall = 0.012

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(width, depth, height, wall), "body_shell"),
        material=walnut,
        name="hollow_body",
    )
    body.visual(
        Box((width - 2.0 * wall, 0.004, height - 0.075)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + 0.002, 0.153)),
        material=velvet,
        name="back_liner",
    )
    body.visual(
        Box((width - 2.0 * wall, depth - wall - 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.004, 0.0205)),
        material=velvet,
        name="floor_liner",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.023),
        origin=Origin(xyz=(0.0, -0.0615, 0.180), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="spindle",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.009),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + 0.0045, 0.180), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="spindle_boss",
    )

    lid = model.part("lid")
    lid_len = depth + 0.012
    lid_width = width + 0.020
    lid_thick = 0.016
    lid.visual(
        mesh_from_cadquery(_lid_frame(lid_width, lid_len, lid_thick), "lid_frame"),
        material=walnut,
        name="wood_frame",
    )
    lid.visual(
        Box((lid_width - 0.056, lid_len - 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.050 + (lid_len - 0.078) / 2.0, -0.002)),
        material=glass,
        name="glass_pane",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=lid_width + 0.016),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="hinge_barrel",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.057, length=0.010),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="cradle_plate",
    )
    cradle.visual(
        mesh_from_cadquery(_cradle_trim_ring(0.064, 0.052, 0.006), "cradle_trim_ring"),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=brass,
        name="trim_ring",
    )
    cradle.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub",
    )
    cradle.visual(
        mesh_from_cadquery(_soft_cushion((0.092, 0.030, 0.055)), "soft_cushion"),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=velvet,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.018, 0.006, 0.069)),
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        material=leather,
        name="keeper_band",
    )
    cradle.visual(
        Box((0.104, 0.005, 0.010)),
        origin=Origin(xyz=(0.0, 0.0415, 0.032)),
        material=leather,
        name="top_retainer",
    )
    cradle.visual(
        Box((0.104, 0.005, 0.010)),
        origin=Origin(xyz=(0.0, 0.0415, -0.032)),
        material=leather,
        name="bottom_retainer",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.003, height + 0.053)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.050, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    ctx.check(
        "lid has rear revolute hinge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.0,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "cradle spins continuously on spindle",
        cradle_spin.articulation_type == ArticulationType.CONTINUOUS
        and cradle_spin.axis == (0.0, 1.0, 0.0),
        details=f"type={cradle_spin.articulation_type}, axis={cradle_spin.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, cradle_spin: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            name="closed lid seats on deep hinge frame",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.120,
            name="closed lid covers the presentation box",
        )
        ctx.expect_contact(
            body,
            cradle,
            elem_a="spindle",
            elem_b="hub",
            contact_tol=0.001,
            name="cradle hub seats on spindle tip",
        )
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            margin=0.006,
            name="rotating cradle fits inside tall narrow body",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.05}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    with ctx.pose({cradle_spin: pi / 2.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            margin=0.006,
            name="cradle remains clear while spinning",
        )

    return ctx.report()


object_model = build_object_model()
