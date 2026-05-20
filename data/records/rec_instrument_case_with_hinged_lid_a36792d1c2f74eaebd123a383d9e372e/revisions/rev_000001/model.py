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


CASE_LENGTH = 0.68
CASE_DEPTH = 0.29
LOWER_HEIGHT = 0.120
LID_HEIGHT = 0.105
WALL = 0.018
BOTTOM_THICKNESS = 0.020
TOP_THICKNESS = 0.018
HINGE_OFFSET = 0.014
LID_SEAM_GAP = 0.0015

FRONT_Y = -CASE_DEPTH / 2.0
BACK_Y = CASE_DEPTH / 2.0


def _lower_tray_shape() -> cq.Workplane:
    """Shallow, hollow lower case shell with softened outside corners."""

    outer = cq.Workplane("XY").box(
        CASE_LENGTH,
        CASE_DEPTH,
        LOWER_HEIGHT,
        centered=(True, True, False),
    )
    outer = outer.edges("|Z").fillet(0.014)

    cavity = (
        cq.Workplane("XY")
        .box(
            CASE_LENGTH - 2.0 * WALL,
            CASE_DEPTH - 2.0 * WALL,
            LOWER_HEIGHT + 0.010,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BOTTOM_THICKNESS))
    )
    return outer.cut(cavity)


def _lid_shell_shape() -> cq.Workplane:
    """Hollow top cap modeled in the lid frame, whose origin is the hinge axis."""

    lid_center_y = -(CASE_DEPTH / 2.0 + HINGE_OFFSET)
    outer = (
        cq.Workplane("XY")
        .box(CASE_LENGTH, CASE_DEPTH, LID_HEIGHT, centered=(True, True, False))
        .translate((0.0, lid_center_y, 0.0))
    )
    outer = outer.edges("|Z").fillet(0.014)

    underside_cut = (
        cq.Workplane("XY")
        .box(
            CASE_LENGTH - 2.0 * WALL,
            CASE_DEPTH - 2.0 * WALL,
            LID_HEIGHT - TOP_THICKNESS + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, lid_center_y, -0.001))
    )
    return outer.cut(underside_cut)


def _handle_loop_shape() -> cq.Workplane:
    """One connected U-shaped suitcase bail handle in its pivot frame."""

    half_span = 0.135
    drop = 0.090
    bar = 0.020
    thickness_y = 0.018
    y = -0.006

    left_arm = (
        cq.Workplane("XY")
        .box(bar, thickness_y, drop, centered=(True, True, True))
        .translate((-half_span, y, -drop / 2.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(bar, thickness_y, drop, centered=(True, True, True))
        .translate((half_span, y, -drop / 2.0))
    )
    grip = (
        cq.Workplane("XY")
        .box(2.0 * half_span + bar, thickness_y, bar, centered=(True, True, True))
        .translate((0.0, y, -drop))
    )

    def pivot_boss(x_center: float) -> cq.Workplane:
        # Cylinder is extruded normal to the YZ workplane, so it lies on local X.
        return (
            cq.Workplane("YZ")
            .circle(0.014)
            .extrude(0.040)
            .translate((x_center - 0.020, y, 0.0))
        )

    return left_arm.union(right_arm).union(grip).union(pivot_boss(-half_span)).union(
        pivot_boss(half_span)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_opening_trumpet_case")

    black_vinyl = model.material("black_vinyl", rgba=(0.015, 0.014, 0.013, 1.0))
    edge_black = model.material("edge_black", rgba=(0.035, 0.032, 0.030, 1.0))
    burgundy_velvet = model.material("burgundy_velvet", rgba=(0.34, 0.035, 0.060, 1.0))
    dark_velvet = model.material("dark_velvet_recess", rgba=(0.12, 0.010, 0.020, 1.0))
    brass = model.material("brass_hardware", rgba=(0.86, 0.62, 0.24, 1.0))
    handle_rubber = model.material("handle_rubber", rgba=(0.020, 0.019, 0.018, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_cadquery(_lower_tray_shape(), "lower_tray", tolerance=0.0008),
        material=black_vinyl,
        name="lower_tray",
    )
    lower_shell.visual(
        Box((CASE_LENGTH - 2.0 * WALL - 0.014, CASE_DEPTH - 2.0 * WALL - 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS + 0.003)),
        material=burgundy_velvet,
        name="floor_liner",
    )
    # Molded plush contours that make the open case read as an instrument case.
    lower_shell.visual(
        Cylinder(radius=0.074, length=0.007),
        origin=Origin(xyz=(-0.215, -0.012, BOTTOM_THICKNESS + 0.009)),
        material=dark_velvet,
        name="bell_recess",
    )
    lower_shell.visual(
        Box((0.310, 0.052, 0.007)),
        origin=Origin(xyz=(0.065, 0.035, BOTTOM_THICKNESS + 0.009)),
        material=dark_velvet,
        name="body_recess",
    )
    lower_shell.visual(
        Box((0.120, 0.040, 0.007)),
        origin=Origin(xyz=(0.245, -0.060, BOTTOM_THICKNESS + 0.009)),
        material=dark_velvet,
        name="mouthpiece_recess",
    )

    # Rear hinge barrel and leaf are fixed to the lower shell; the articulated
    # joint shares this same rear horizontal axis.
    lower_shell.visual(
        Box((CASE_LENGTH - 0.070, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, BACK_Y + 0.004, LOWER_HEIGHT + 0.015)),
        material=brass,
        name="rear_hinge_leaf",
    )
    lower_shell.visual(
        Cylinder(radius=0.008, length=CASE_LENGTH - 0.080),
        origin=Origin(
            xyz=(0.0, BACK_Y + HINGE_OFFSET, LOWER_HEIGHT + 0.022),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="rear_hinge_barrel",
    )

    for i, x in enumerate((-0.215, 0.215)):
        lower_shell.visual(
            Box((0.064, 0.010, 0.040)),
            origin=Origin(xyz=(x, FRONT_Y - 0.005, LOWER_HEIGHT - 0.020)),
            material=brass,
            name=f"catch_plate_{i}",
        )

    for i, x in enumerate((-0.135, 0.135)):
        lower_shell.visual(
            Box((0.048, 0.018, 0.050)),
            origin=Origin(xyz=(x, FRONT_Y - 0.009, 0.073)),
            material=edge_black,
            name=f"handle_mount_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "lid_shell", tolerance=0.0008),
        material=black_vinyl,
        name="lid_shell",
    )
    lid.visual(
        Box((CASE_LENGTH - 2.0 * WALL - 0.020, CASE_DEPTH - 2.0 * WALL - 0.020, 0.005)),
        origin=Origin(
            xyz=(0.0, -(CASE_DEPTH / 2.0 + HINGE_OFFSET), LID_HEIGHT - TOP_THICKNESS + 0.002)
        ),
        material=burgundy_velvet,
        name="lid_liner",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.030, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -(CASE_DEPTH + HINGE_OFFSET) - 0.003, 0.018)),
        material=brass,
        name="front_lid_band",
    )

    lid_hinge = model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, BACK_Y + HINGE_OFFSET, LOWER_HEIGHT + LID_SEAM_GAP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=12.0, velocity=1.4),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_loop_shape(), "handle_loop", tolerance=0.0008),
        material=handle_rubber,
        name="handle_loop",
    )
    model.articulation(
        "lower_shell_to_handle",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.024, 0.086)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.12, upper=1.35, effort=4.0, velocity=2.0),
    )

    latch_parent_y = -(CASE_DEPTH + HINGE_OFFSET) - 0.006
    for i, x in enumerate((-0.215, 0.215)):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.006, length=0.060),
            origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="latch_knuckle",
        )
        latch.visual(
            Box((0.052, 0.006, 0.056)),
            origin=Origin(xyz=(0.0, -0.004, -0.030)),
            material=brass,
            name="latch_plate",
        )
        model.articulation(
            f"lid_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x, latch_parent_y, 0.052)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=1.0, velocity=2.5),
        )

    # Keep symbolic references in model metadata for tests and for downstream
    # readers that want the principal mechanisms without inferring names.
    model.meta["primary_joints"] = [lid_hinge.name, "lower_shell_to_handle"]
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

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("lower_shell_to_lid")
    handle_hinge = object_model.get_articulation("lower_shell_to_handle")

    for i in range(2):
        ctx.allow_overlap(
            lower_shell,
            handle,
            elem_a=f"handle_mount_{i}",
            elem_b="handle_loop",
            reason=(
                "The handle pivot boss is intentionally captured in the front "
                "mount block so the bail has a real local pivot support."
            ),
        )
        ctx.expect_gap(
            lower_shell,
            handle,
            axis="y",
            max_gap=0.003,
            max_penetration=0.004,
            positive_elem=f"handle_mount_{i}",
            negative_elem="handle_loop",
            name=f"handle pivot_{i} is seated in its front mount",
        )
        ctx.expect_overlap(
            lower_shell,
            handle,
            axes="xz",
            min_overlap=0.010,
            elem_a=f"handle_mount_{i}",
            elem_b="handle_loop",
            name=f"handle pivot_{i} aligns with front mount",
        )

        latch = object_model.get_part(f"latch_{i}")
        ctx.allow_overlap(
            lid,
            latch,
            elem_a="front_lid_band",
            elem_b="latch_plate",
            reason=(
                "The latch plate is locally seated into the lid's front band, "
                "representing the hidden hinge/hasp capture at the case seam."
            ),
        )
        ctx.expect_gap(
            lid,
            latch,
            axis="y",
            max_gap=0.003,
            max_penetration=0.004,
            positive_elem="front_lid_band",
            negative_elem="latch_plate",
            name=f"latch_{i} plate is seated against front band",
        )
        ctx.expect_overlap(
            lid,
            latch,
            axes="xz",
            min_overlap=0.010,
            elem_a="front_lid_band",
            elem_b="latch_plate",
            name=f"latch_{i} plate spans the front band",
        )

    with ctx.pose({lid_hinge: 0.0, handle_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            positive_elem="lid_shell",
            negative_elem="lower_tray",
            name="closed lid sits just above lower shell rim",
        )
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="xy",
            min_overlap=0.20,
            elem_a="lid_shell",
            elem_b="lower_tray",
            name="lid footprint covers the lower case",
        )

        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_loop")

    with ctx.pose({lid_hinge: 1.25}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid opens upward on rear horizontal hinge",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.14,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    with ctx.pose({handle_hinge: 1.05}):
        lifted_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_loop")

    ctx.check(
        "handle swings outward from front pivots",
        closed_handle_aabb is not None
        and lifted_handle_aabb is not None
        and lifted_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.035,
        details=f"closed={closed_handle_aabb}, lifted={lifted_handle_aabb}",
    )

    for i in range(2):
        latch = object_model.get_part(f"latch_{i}")
        latch_joint = object_model.get_articulation(f"lid_to_latch_{i}")
        with ctx.pose({latch_joint: 0.0}):
            closed = ctx.part_element_world_aabb(latch, elem="latch_plate")
        with ctx.pose({latch_joint: 0.85}):
            flipped = ctx.part_element_world_aabb(latch, elem="latch_plate")
        ctx.check(
            f"latch_{i} flips up from the front band",
            closed is not None and flipped is not None and flipped[0][2] > closed[0][2] + 0.010,
            details=f"closed={closed}, flipped={flipped}",
        )

    return ctx.report()


object_model = build_object_model()
