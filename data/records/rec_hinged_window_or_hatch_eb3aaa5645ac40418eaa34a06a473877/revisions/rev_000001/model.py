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
    model = ArticulatedObject(name="top_hinged_awning_window")

    aluminum = model.material("dark_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.80, 0.95, 0.36))
    gasket = model.material("black_rubber_gasket", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("brushed_hinge_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    highlight = model.material("soft_glass_reflection", rgba=(0.90, 0.97, 1.0, 0.55))

    frame = model.part("frame")
    frame.visual(Box((1.20, 0.09, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.40)), material=aluminum, name="top_rail")
    frame.visual(Box((1.20, 0.09, 0.10)), origin=Origin(xyz=(0.0, 0.0, -0.40)), material=aluminum, name="bottom_rail")
    frame.visual(Box((0.10, 0.09, 0.90)), origin=Origin(xyz=(-0.55, 0.0, 0.0)), material=aluminum, name="side_stile_0")
    frame.visual(Box((0.10, 0.09, 0.90)), origin=Origin(xyz=(0.55, 0.0, 0.0)), material=aluminum, name="side_stile_1")

    # A thin black compression stop shows the inset opening that the sash closes against.
    frame.visual(Box((1.00, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.051, 0.341)), material=gasket, name="top_gasket")
    frame.visual(Box((1.00, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.051, -0.341)), material=gasket, name="bottom_gasket")
    frame.visual(Box((0.018, 0.018, 0.70)), origin=Origin(xyz=(-0.491, 0.051, 0.0)), material=gasket, name="side_gasket_0")
    frame.visual(Box((0.018, 0.018, 0.70)), origin=Origin(xyz=(0.491, 0.051, 0.0)), material=gasket, name="side_gasket_1")

    # Two compact hinge halves are fixed to the upper frame.  Their barrels sit on
    # the shared horizontal pivot line, with space for the sash-side knuckle.
    for index, x_center in enumerate((-0.30, 0.30)):
        for side_index, x_offset in enumerate((-0.042, 0.042)):
            frame.visual(
                Box((0.036, 0.012, 0.050)),
                origin=Origin(xyz=(x_center + x_offset, 0.050, 0.347)),
                material=steel,
                name=f"frame_hinge_leaf_{index}_{side_index}",
            )
            frame.visual(
                Cylinder(radius=0.014, length=0.036),
                origin=Origin(xyz=(x_center + x_offset, 0.055, 0.320), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"frame_hinge_knuckle_{index}_{side_index}",
            )

    sash = model.part("sash")
    sash.visual(Box((0.92, 0.050, 0.055)), origin=Origin(xyz=(0.0, -0.050, -0.0275)), material=aluminum, name="sash_top")
    sash.visual(Box((0.92, 0.050, 0.055)), origin=Origin(xyz=(0.0, -0.050, -0.6125)), material=aluminum, name="sash_bottom")
    sash.visual(Box((0.055, 0.050, 0.640)), origin=Origin(xyz=(-0.4325, -0.050, -0.320)), material=aluminum, name="sash_side_0")
    sash.visual(Box((0.055, 0.050, 0.640)), origin=Origin(xyz=(0.4325, -0.050, -0.320)), material=aluminum, name="sash_side_1")
    sash.visual(Box((0.810, 0.008, 0.530)), origin=Origin(xyz=(0.0, -0.050, -0.320)), material=glass, name="glass_pane")
    sash.visual(
        Box((0.018, 0.004, 0.42)),
        origin=Origin(xyz=(-0.18, -0.044, -0.31), rpy=(0.0, 0.22, 0.0)),
        material=highlight,
        name="reflection_strip_0",
    )
    sash.visual(
        Box((0.012, 0.004, 0.28)),
        origin=Origin(xyz=(0.17, -0.044, -0.29), rpy=(0.0, 0.22, 0.0)),
        material=highlight,
        name="reflection_strip_1",
    )

    for index, x_center in enumerate((-0.30, 0.30)):
        sash.visual(
            Box((0.048, 0.030, 0.046)),
            origin=Origin(xyz=(x_center, -0.010, -0.032)),
            material=steel,
            name=f"sash_hinge_leaf_{index}",
        )
        sash.visual(
            Cylinder(radius=0.014, length=0.036),
            origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"sash_hinge_knuckle_{index}",
        )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, 0.055, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.radians(60.0)),
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

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    hinge = object_model.get_articulation("frame_to_sash")

    ctx.check(
        "awning hinge opens about sixty degrees",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and abs(hinge.motion_limits.upper - math.radians(60.0)) < 1e-6,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_gasket",
            negative_elem="sash_top",
            min_gap=0.005,
            max_gap=0.025,
            name="closed sash clears upper weatherstrip",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="sash_bottom",
            negative_elem="bottom_gasket",
            min_gap=0.005,
            max_gap=0.025,
            name="closed sash clears lower weatherstrip",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            inner_elem="glass_pane",
            margin=0.0,
            name="closed glass pane stays inside outer frame outline",
        )

    closed_aabb = ctx.part_element_world_aabb(sash, elem="sash_bottom")
    with ctx.pose({hinge: math.radians(60.0)}):
        open_aabb = ctx.part_element_world_aabb(sash, elem="sash_bottom")
        ctx.expect_origin_gap(sash, frame, axis="y", min_gap=0.0, name="opened sash origin remains on exterior side")
    ctx.check(
        "bottom of sash swings outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] > closed_aabb[0][1] + 0.35
        and open_aabb[0][2] > closed_aabb[0][2] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
