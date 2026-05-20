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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supported_tilt_head")

    frame_paint = model.material("charcoal_powder_coat", rgba=(0.10, 0.12, 0.14, 1.0))
    bearing_finish = model.material("black_bearing_blocks", rgba=(0.02, 0.025, 0.03, 1.0))
    shaft_finish = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    cradle_paint = model.material("blue_cradle_paint", rgba=(0.10, 0.24, 0.55, 1.0))
    liner_rubber = model.material("dark_rubber_liners", rgba=(0.025, 0.025, 0.025, 1.0))

    shaft_z = 0.55
    tower_x = 0.34
    tower_width = 0.08
    tower_depth = 0.18
    tower_height = 0.58

    frame = model.part("frame")
    frame.visual(
        Box((0.82, 0.32, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_paint,
        name="base_plate",
    )
    for idx, (x, bearing_name) in enumerate(((-tower_x, "bearing_0"), (tower_x, "bearing_1"))):
        frame.visual(
            Box((tower_width, tower_depth, tower_height)),
            origin=Origin(xyz=(x, 0.0, 0.06 + tower_height / 2.0 - 0.002)),
            material=frame_paint,
            name=f"tower_{idx}",
        )
        frame.visual(
            Box((0.13, 0.24, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.077)),
            material=frame_paint,
            name=f"tower_foot_{idx}",
        )
        frame.visual(
            Box((0.028, 0.23, 0.23)),
            origin=Origin(xyz=(x * 0.985, 0.0, 0.30)),
            material=frame_paint,
            name=f"web_gusset_{idx}",
        )
        bearing_ring = TorusGeometry(
            0.036,
            0.014,
            radial_segments=32,
            tubular_segments=12,
        ).rotate_y(math.pi / 2.0)
        frame.visual(
            mesh_from_geometry(bearing_ring, f"bearing_ring_{idx}"),
            origin=Origin(xyz=(x - math.copysign(tower_width / 2.0, x), 0.0, shaft_z)),
            material=bearing_finish,
            name=bearing_name,
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.025, length=0.584),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_finish,
        name="cross_shaft",
    )
    for idx, x in enumerate((-0.232, 0.232)):
        cradle.visual(
            Cylinder(radius=0.046, length=0.052),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shaft_finish,
            name=f"shaft_collar_{idx}",
        )
        cradle.visual(
            Box((0.034, 0.30, 0.31)),
            origin=Origin(xyz=(x, 0.0, -0.145)),
            material=cradle_paint,
            name=f"side_plate_{idx}",
        )

    cradle.visual(
        Box((0.48, 0.32, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=cradle_paint,
        name="tray_floor",
    )
    cradle.visual(
        Box((0.48, 0.035, 0.15)),
        origin=Origin(xyz=(0.0, 0.177, -0.205)),
        material=cradle_paint,
        name="front_wall",
    )
    cradle.visual(
        Box((0.48, 0.035, 0.15)),
        origin=Origin(xyz=(0.0, -0.177, -0.205)),
        material=cradle_paint,
        name="rear_wall",
    )
    cradle.visual(
        Box((0.40, 0.24, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.2615)),
        material=liner_rubber,
        name="payload_pad",
    )

    model.articulation(
        "frame_to_cradle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.70, upper=0.70),
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
    cradle = object_model.get_part("cradle")
    tilt = object_model.get_articulation("frame_to_cradle")

    ctx.allow_overlap(
        frame,
        cradle,
        elem_a="bearing_0",
        elem_b="cross_shaft",
        reason="The cross-shaft is intentionally captured in the left bearing bore of the supported pivot.",
    )
    ctx.allow_overlap(
        frame,
        cradle,
        elem_a="bearing_1",
        elem_b="cross_shaft",
        reason="The cross-shaft is intentionally captured in the right bearing bore of the supported pivot.",
    )

    ctx.check("single_pitch_joint", len(object_model.articulations) == 1, "Expected one revolute tilt joint.")
    ctx.expect_overlap(
        cradle,
        frame,
        axes="yz",
        elem_a="cross_shaft",
        elem_b="bearing_0",
        min_overlap=0.04,
        name="shaft_aligned_with_bearing_0",
    )
    ctx.expect_overlap(
        cradle,
        frame,
        axes="yz",
        elem_a="cross_shaft",
        elem_b="bearing_1",
        min_overlap=0.04,
        name="shaft_aligned_with_bearing_1",
    )
    ctx.expect_within(
        cradle,
        frame,
        axes="x",
        inner_elem="tray_floor",
        outer_elem="base_plate",
        margin=0.0,
        name="payload_tray_between_towers",
    )

    rest_aabb = ctx.part_element_world_aabb(cradle, elem="tray_floor")
    with ctx.pose({tilt: 0.55}):
        pitched_aabb = ctx.part_element_world_aabb(cradle, elem="tray_floor")
    ctx.check(
        "cradle_pitches_about_shaft",
        rest_aabb is not None
        and pitched_aabb is not None
        and abs(
            ((pitched_aabb[0][1] + pitched_aabb[1][1]) / 2.0)
            - ((rest_aabb[0][1] + rest_aabb[1][1]) / 2.0)
        )
        > 0.09,
        details=f"rest={rest_aabb}, pitched={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
