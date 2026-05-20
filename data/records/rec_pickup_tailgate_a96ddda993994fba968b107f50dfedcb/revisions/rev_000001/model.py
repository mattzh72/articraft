from __future__ import annotations

import math

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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_step")

    paint = model.material("deep_blue_paint", color=(0.03, 0.12, 0.24, 1.0))
    dark_paint = model.material("shadowed_blue_stamp", color=(0.02, 0.08, 0.18, 1.0))
    bedliner = model.material("black_bedliner", color=(0.01, 0.012, 0.012, 1.0))
    dark_plastic = model.material("black_plastic", color=(0.005, 0.005, 0.006, 1.0))
    step_metal = model.material("dark_step_metal", color=(0.12, 0.13, 0.13, 1.0))
    hinge_metal = model.material("satin_hinge_metal", color=(0.55, 0.55, 0.52, 1.0))

    # Root: a simplified pickup bed rear opening.  The cross sill ties the two
    # bed-side supports together so the tailgate clearly hangs between them.
    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((0.30, 1.96, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, 0.61)),
        material=bedliner,
        name="rear_sill",
    )
    for side, y in (("side_0", -0.90), ("side_1", 0.90)):
        bed_frame.visual(
            Box((0.34, 0.12, 0.66)),
            origin=Origin(xyz=(0.19, y * 1.08, 0.94)),
            material=paint,
            name=side,
        )
        bed_frame.visual(
            Box((0.040, 0.080, 0.070)),
            origin=Origin(xyz=(0.015, y * 1.039, 0.62)),
            material=hinge_metal,
            name=f"hinge_bracket_{side[-1]}",
        )
        bed_frame.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(
                xyz=(0.010, y * 1.070, 0.62),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"hinge_socket_{side[-1]}",
        )
    for idx, y in enumerate((-0.860, 0.860)):
        bed_frame.visual(
            Cylinder(radius=0.008, length=0.090),
            origin=Origin(
                xyz=(0.0, y, 0.62),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"lower_hinge_pin_{idx}",
        )

    # Tailgate frame: local origin is the lower transverse hinge line.  The
    # main shell is a single CadQuery solid with a real inner-face pocket for
    # the step panel, rather than a step simply pasted onto a flat slab.
    tailgate_body = (
        cq.Workplane("XY")
        .box(0.075, 1.62, 0.60, centered=(True, True, True))
        .translate((-0.0375, 0.0, 0.30))
    )
    step_pocket = (
        cq.Workplane("XY")
        .box(0.055, 0.98, 0.39, centered=(True, True, True))
        .translate((-0.0025, 0.0, 0.31))
    )
    tailgate_body = tailgate_body.cut(step_pocket)

    tailgate = model.part("tailgate")
    tailgate.visual(
        mesh_from_cadquery(tailgate_body, "tailgate_shell", tolerance=0.002),
        material=paint,
        name="tailgate_shell",
    )
    tailgate.visual(
        Box((0.004, 1.22, 0.24)),
        origin=Origin(xyz=(-0.077, 0.0, 0.35)),
        material=dark_paint,
        name="outer_stamp",
    )
    tailgate.visual(
        Box((0.004, 0.50, 0.12)),
        origin=Origin(xyz=(-0.081, 0.0, 0.25)),
        material=bedliner,
        name="license_recess",
    )
    tailgate.visual(
        Box((0.004, 1.48, 0.065)),
        origin=Origin(xyz=(0.002, 0.0, 0.555)),
        material=bedliner,
        name="inner_upper_liner",
    )
    tailgate.visual(
        Box((0.004, 1.48, 0.060)),
        origin=Origin(xyz=(0.002, 0.0, 0.065)),
        material=bedliner,
        name="inner_lower_liner",
    )
    for idx, y in enumerate((-0.505, 0.505)):
        tailgate.visual(
            Box((0.004, 0.030, 0.390)),
            origin=Origin(xyz=(0.002, y, 0.31)),
            material=bedliner,
            name=f"recess_side_trim_{idx}",
        )
    for idx, y in enumerate((-0.855, 0.855)):
        tailgate.visual(
            Cylinder(radius=0.025, length=0.080),
            origin=Origin(
                xyz=(0.0, y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"lower_hinge_tube_{idx}",
        )
        tailgate.visual(
            Box((0.060, 0.035, 0.018)),
            origin=Origin(xyz=(-0.020, math.copysign(0.812, y), 0.015)),
            material=hinge_metal,
            name=f"lower_hinge_leaf_{idx}",
        )
    tailgate.visual(
        Cylinder(radius=0.006, length=1.050),
        origin=Origin(
            xyz=(-0.012, 0.0, 0.145),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_metal,
        name="step_hinge_pin",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=1.2, lower=0.0, upper=math.radians(95.0)),
    )

    # Fold-out step panel nested in the inner-face pocket.  Its local origin is
    # the bottom hinge line of the step panel.
    step_panel = model.part("step_panel")
    step_panel.visual(
        Box((0.010, 0.780, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=step_metal,
        name="step_plate",
    )
    step_panel.visual(
        Cylinder(radius=0.012, length=0.780),
        origin=Origin(
            xyz=(-0.002, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_metal,
        name="step_hinge_barrel",
    )
    for idx, z in enumerate((0.070, 0.115, 0.160, 0.205, 0.250, 0.295)):
        step_panel.visual(
            Box((0.004, 0.660, 0.010)),
            origin=Origin(xyz=(0.007, 0.0, z)),
            material=bedliner,
            name=f"tread_rib_{idx}",
        )
    step_panel.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(
            xyz=(0.010, 0.0, 0.245),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=step_metal,
        name="handle_boss",
    )

    model.articulation(
        "tailgate_to_step_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=step_panel,
        origin=Origin(xyz=(-0.010, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=math.radians(100.0)),
    )

    # Small rotating release handle captured by the boss on the step panel.
    release_handle = model.part("release_handle")
    release_handle.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(
            xyz=(-0.009, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="pivot_pin",
    )
    release_handle.visual(
        Box((0.020, 0.180, 0.050)),
        origin=Origin(xyz=(0.010, 0.0, -0.035)),
        material=dark_plastic,
        name="pull_paddle",
    )
    release_handle.visual(
        Box((0.024, 0.145, 0.008)),
        origin=Origin(xyz=(0.021, 0.0, -0.055)),
        material=bedliner,
        name="finger_groove",
    )

    model.articulation(
        "step_panel_to_release_handle",
        ArticulationType.REVOLUTE,
        parent=step_panel,
        child=release_handle,
        origin=Origin(xyz=(0.024, 0.0, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=math.radians(55.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    step_panel = object_model.get_part("step_panel")
    release_handle = object_model.get_part("release_handle")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    step_hinge = object_model.get_articulation("tailgate_to_step_panel")
    handle_pivot = object_model.get_articulation("step_panel_to_release_handle")

    ctx.allow_overlap(
        step_panel,
        release_handle,
        elem_a="handle_boss",
        elem_b="pivot_pin",
        reason="The release-handle pivot pin is intentionally captured inside the molded boss on the step panel.",
    )
    ctx.allow_overlap(
        tailgate,
        step_panel,
        elem_a="step_hinge_pin",
        elem_b="step_hinge_barrel",
        reason="The fold-out step hinge barrel intentionally rotates around the fixed hinge pin in the tailgate pocket.",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            bed_frame,
            tailgate,
            elem_a=f"lower_hinge_pin_{idx}",
            elem_b=f"lower_hinge_tube_{idx}",
            reason="The main tailgate lower hinge tube intentionally rotates around the bed-side hinge pin.",
        )

    ctx.expect_within(
        tailgate,
        bed_frame,
        axes="y",
        margin=0.001,
        elem_a="tailgate_shell",
        elem_b="rear_sill",
        name="tailgate spans between bed-side supports",
    )
    ctx.expect_within(
        step_panel,
        tailgate,
        axes="yz",
        margin=0.002,
        elem_a="step_plate",
        elem_b="tailgate_shell",
        name="step panel is nested within the inner tailgate pocket",
    )
    ctx.expect_overlap(
        release_handle,
        step_panel,
        axes="x",
        min_overlap=0.006,
        elem_a="pivot_pin",
        elem_b="handle_boss",
        name="release handle pivot pin remains seated in boss",
    )
    ctx.expect_within(
        release_handle,
        step_panel,
        axes="yz",
        margin=0.002,
        elem_a="pivot_pin",
        elem_b="handle_boss",
        name="release handle pivot is centered in boss",
    )
    ctx.expect_overlap(
        tailgate,
        step_panel,
        axes="y",
        min_overlap=0.70,
        elem_a="step_hinge_pin",
        elem_b="step_hinge_barrel",
        name="step hinge barrel is retained on tailgate pin",
    )
    for idx in (0, 1):
        ctx.expect_overlap(
            bed_frame,
            tailgate,
            axes="y",
            min_overlap=0.06,
            elem_a=f"lower_hinge_pin_{idx}",
            elem_b=f"lower_hinge_tube_{idx}",
            name=f"tailgate lower hinge pin {idx} retained in tube",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    closed_tailgate_center = _aabb_center(ctx.part_world_aabb(tailgate))
    with ctx.pose({tailgate_hinge: math.radians(90.0)}):
        open_tailgate_center = _aabb_center(ctx.part_world_aabb(tailgate))
    ctx.check(
        "main tailgate rotates rearward and downward",
        closed_tailgate_center is not None
        and open_tailgate_center is not None
        and open_tailgate_center[0] < closed_tailgate_center[0] - 0.20
        and open_tailgate_center[2] < closed_tailgate_center[2] - 0.20,
        details=f"closed={closed_tailgate_center}, open={open_tailgate_center}",
    )

    closed_step_center = _aabb_center(ctx.part_world_aabb(step_panel))
    with ctx.pose({step_hinge: math.radians(75.0)}):
        folded_step_center = _aabb_center(ctx.part_world_aabb(step_panel))
    ctx.check(
        "step panel folds outward from inner face",
        closed_step_center is not None
        and folded_step_center is not None
        and folded_step_center[0] > closed_step_center[0] + 0.08,
        details=f"closed={closed_step_center}, folded={folded_step_center}",
    )

    closed_handle_center = _aabb_center(ctx.part_world_aabb(release_handle))
    with ctx.pose({handle_pivot: math.radians(45.0)}):
        turned_handle_center = _aabb_center(ctx.part_world_aabb(release_handle))
    ctx.check(
        "release handle rotates about local pivot",
        closed_handle_center is not None
        and turned_handle_center is not None
        and abs(turned_handle_center[1] - closed_handle_center[1]) > 0.015,
        details=f"closed={closed_handle_center}, turned={turned_handle_center}",
    )

    return ctx.report()


object_model = build_object_model()
