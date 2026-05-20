from __future__ import annotations

import math

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


TOP_X = 0.62
TOP_Y = 0.38
TOP_THICKNESS = 0.024
SLEEVE_BOTTOM_Z = 0.112
SLEEVE_LENGTH = 0.380
SLEEVE_TOP_Z = SLEEVE_BOTTOM_Z + SLEEVE_LENGTH


def _rounded_tabletop() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(TOP_X, TOP_Y)
        .extrude(TOP_THICKNESS)
        .edges("|Z")
        .fillet(0.018)
    )


def _square_tube(outer: float, inner: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(outer, outer).rect(inner, inner).extrude(length)


def _add_caster(
    model: ArticulatedObject,
    base,
    index: int,
    x: float,
    y: float,
    z: float,
    materials: dict[str, Material],
) -> None:
    fork = model.part(f"caster_fork_{index}")
    fork.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=materials["metal_dark"],
        name="swivel_bearing",
    )
    fork.visual(
        Box((0.046, 0.014, 0.009)),
        origin=Origin(xyz=(0.0, 0.038, -0.0145)),
        material=materials["metal_dark"],
        name="fork_crown",
    )
    fork.visual(
        Box((0.018, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0205, -0.011)),
        material=materials["metal_dark"],
        name="swivel_neck",
    )
    for side, sx in (("a", -0.019), ("b", 0.019)):
        fork.visual(
            Box((0.0045, 0.096, 0.050)),
            origin=Origin(xyz=(sx, -0.007, -0.038)),
            material=materials["metal_dark"],
            name=f"fork_cheek_{side}",
        )
    for name, sx in (("axle_boss_0", -0.023), ("axle_boss_1", 0.023)):
        fork.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(sx, 0.0, -0.037), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=materials["brushed_metal"],
            name=name,
        )

    model.articulation(
        f"base_to_caster_{index}",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=fork,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    wheel = model.part(f"caster_wheel_{index}")
    wheel.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["rubber"],
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.011, length=0.0335),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["brushed_metal"],
        name="hub",
    )
    model.articulation(
        f"caster_{index}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    lock = model.part(f"caster_lock_{index}")
    lock.visual(
        Cylinder(radius=0.0035, length=0.0335),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["lock_red"],
        name="lock_pivot",
    )
    lock.visual(
        Box((0.026, 0.038, 0.006)),
        origin=Origin(xyz=(0.0, -0.020, 0.000)),
        material=materials["lock_red"],
        name="lock_pedal",
    )
    model.articulation(
        f"caster_{index}_lock_pivot",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=lock,
        origin=Origin(xyz=(0.0, -0.038, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=0.85),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_overbed_table")

    materials = {
        "laminate": model.material("warm_white_laminate", rgba=(0.88, 0.86, 0.80, 1.0)),
        "edge": model.material("charcoal_edge_trim", rgba=(0.06, 0.065, 0.07, 1.0)),
        "metal_dark": model.material("powder_coated_black_steel", rgba=(0.02, 0.022, 0.025, 1.0)),
        "brushed_metal": model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0)),
        "rubber": model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0)),
        "lock_red": model.material("red_lock_pedal", rgba=(0.78, 0.04, 0.025, 1.0)),
    }

    base = model.part("base_frame")
    # A low, narrow frame, offset slightly toward the rear relative to the mast.
    for name, sx in (("side_rail_0", -0.250), ("side_rail_1", 0.250)):
        base.visual(
            Box((0.035, 0.360, 0.026)),
            origin=Origin(xyz=(sx, 0.050, 0.085)),
            material=materials["metal_dark"],
            name=name,
        )
    for name, sy in (("front_crossbar", -0.130), ("rear_crossbar", 0.230)):
        base.visual(
            Box((0.535, 0.035, 0.026)),
            origin=Origin(xyz=(0.0, sy, 0.085)),
            material=materials["metal_dark"],
            name=name,
        )
    base.visual(
        Box((0.038, 0.325, 0.024)),
        origin=Origin(xyz=(0.0, 0.050, 0.086)),
        material=materials["metal_dark"],
        name="center_spine",
    )
    base.visual(
        Box((0.135, 0.105, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=materials["metal_dark"],
        name="mast_foot_plate",
    )
    base.visual(
        mesh_from_cadquery(_square_tube(0.058, 0.046, SLEEVE_LENGTH), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_BOTTOM_Z)),
        material=materials["brushed_metal"],
        name="outer_sleeve",
    )
    for name, px, py, size in (
        ("glide_pad_x0", -0.0215, 0.0, (0.005, 0.020, 0.050)),
        ("glide_pad_x1", 0.0215, 0.0, (0.005, 0.020, 0.050)),
        ("glide_pad_y0", 0.0, -0.0215, (0.020, 0.005, 0.050)),
        ("glide_pad_y1", 0.0, 0.0215, (0.020, 0.005, 0.050)),
    ):
        base.visual(
            Box(size),
            origin=Origin(xyz=(px, py, SLEEVE_TOP_Z - 0.035)),
            material=materials["edge"],
            name=name,
        )
    for cx in (-0.250, 0.250):
        for cy in (-0.130, 0.230):
            base.visual(
                Box((0.060, 0.050, 0.008)),
                origin=Origin(xyz=(cx, cy, 0.068)),
                material=materials["metal_dark"],
                name=f"caster_socket_{len(base.visuals)}",
            )

    for i, (cx, cy) in enumerate(
        ((-0.250, -0.130), (0.250, -0.130), (-0.250, 0.230), (0.250, 0.230))
    ):
        _add_caster(model, base, i, cx, cy, 0.064, materials)

    mast = model.part("inner_column")
    mast.visual(
        Box((0.038, 0.038, 0.620)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=materials["brushed_metal"],
        name="sliding_mast",
    )
    mast.visual(
        Box((0.170, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.376)),
        material=materials["metal_dark"],
        name="support_head",
    )
    mast.visual(
        Box((0.240, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.396)),
        material=materials["metal_dark"],
        name="tilt_top_plate",
    )
    for name, sx in (("hinge_cheek_0", -0.100), ("hinge_cheek_1", 0.100)):
        mast.visual(
            Box((0.014, 0.026, 0.020)),
            origin=Origin(xyz=(sx, 0.0, 0.404)),
            material=materials["metal_dark"],
            name=name,
        )
    mast.visual(
        Cylinder(radius=0.006, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.415), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["brushed_metal"],
        name="tilt_hinge_pin",
    )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.180),
    )

    top = model.part("table_top")
    top.visual(
        mesh_from_cadquery(_rounded_tabletop(), "rounded_laminate_top"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=materials["laminate"],
        name="top_panel",
    )
    top.visual(
        Box((0.570, 0.018, 0.034)),
        origin=Origin(xyz=(0.0, -TOP_Y / 2.0 + 0.012, 0.047)),
        material=materials["edge"],
        name="front_lip",
    )
    top.visual(
        Box((0.590, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, TOP_Y / 2.0 - 0.006, 0.036)),
        material=materials["edge"],
        name="rear_edge",
    )
    for name, sx in (("side_edge_0", -TOP_X / 2.0 + 0.006), ("side_edge_1", TOP_X / 2.0 - 0.006)):
        top.visual(
            Box((0.010, 0.350, 0.012)),
            origin=Origin(xyz=(sx, 0.0, 0.036)),
            material=materials["edge"],
            name=name,
        )

    model.articulation(
        "top_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    mast = object_model.get_part("inner_column")
    top = object_model.get_part("table_top")
    slide = object_model.get_articulation("column_slide")
    tilt = object_model.get_articulation("top_tilt")

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="sliding_mast",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="sliding mast remains centered in the outer sleeve footprint",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="sliding_mast",
        elem_b="outer_sleeve",
        min_overlap=0.20,
        name="collapsed column keeps a long retained insertion",
    )

    rest_mast_position = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.180}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="sliding_mast",
            elem_b="outer_sleeve",
            min_overlap=0.06,
            name="extended column still remains captured in the sleeve",
        )
        extended_mast_position = ctx.part_world_position(mast)
    ctx.check(
        "column slide moves the support head upward",
        rest_mast_position is not None
        and extended_mast_position is not None
        and extended_mast_position[2] > rest_mast_position[2] + 0.15,
        details=f"rest={rest_mast_position}, extended={extended_mast_position}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(top, elem="front_lip")
    with ctx.pose({tilt: -0.25}):
        tilted_lip_aabb = ctx.part_element_world_aabb(top, elem="front_lip")
    rest_front_lip_z = None if rest_lip_aabb is None else (rest_lip_aabb[0][2] + rest_lip_aabb[1][2]) / 2.0
    tilted_front_lip_z = (
        None if tilted_lip_aabb is None else (tilted_lip_aabb[0][2] + tilted_lip_aabb[1][2]) / 2.0
    )
    ctx.check(
        "negative top tilt raises the front retaining lip",
        rest_front_lip_z is not None
        and tilted_front_lip_z is not None
        and tilted_front_lip_z > rest_front_lip_z + 0.035,
        details=f"rest_z={rest_front_lip_z}, tilted_z={tilted_front_lip_z}",
    )

    return ctx.report()


object_model = build_object_model()
