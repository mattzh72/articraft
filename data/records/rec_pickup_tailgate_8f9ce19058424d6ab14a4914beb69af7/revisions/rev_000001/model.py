from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="split_function_pickup_tailgate")

    bed_paint = model.material("bed_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    tailgate_paint = model.material("tailgate_paint", rgba=(0.62, 0.66, 0.70, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.13, 0.13, 0.14, 1.0))
    work_surface = model.material("work_surface", rgba=(0.25, 0.27, 0.29, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((1.62, 0.78, 0.04)),
        origin=Origin(xyz=(0.0, -0.39, -0.02)),
        material=bed_paint,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((0.05, 0.78, 0.47)),
        origin=Origin(xyz=(0.785, -0.39, 0.225)),
        material=bed_paint,
        name="right_bedside",
    )
    bed_frame.visual(
        Box((0.05, 0.78, 0.47)),
        origin=Origin(xyz=(-0.785, -0.39, 0.225)),
        material=bed_paint,
        name="left_bedside",
    )
    bed_frame.visual(
        Box((1.62, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, -0.04, 0.025)),
        material=bed_paint,
        name="rear_sill",
    )
    bed_frame.visual(
        Box((0.10, 0.12, 0.28)),
        origin=Origin(xyz=(0.735, -0.06, 0.14)),
        material=trim_dark,
        name="right_hinge_pocket",
    )
    bed_frame.visual(
        Box((0.10, 0.12, 0.28)),
        origin=Origin(xyz=(-0.735, -0.06, 0.14)),
        material=trim_dark,
        name="left_hinge_pocket",
    )

    tailgate = model.part("main_tailgate")
    tailgate.visual(
        Box((1.50, 0.024, 0.58)),
        origin=Origin(xyz=(0.0, 0.048, 0.29)),
        material=tailgate_paint,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.50, 0.040, 0.10)),
        origin=Origin(xyz=(0.0, 0.020, 0.05)),
        material=trim_dark,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((1.50, 0.040, 0.05)),
        origin=Origin(xyz=(0.0, 0.020, 0.555)),
        material=trim_dark,
        name="top_rail",
    )
    tailgate.visual(
        Box((0.06, 0.040, 0.43)),
        origin=Origin(xyz=(0.72, 0.020, 0.315)),
        material=trim_dark,
        name="right_side_rail",
    )
    tailgate.visual(
        Box((0.06, 0.040, 0.43)),
        origin=Origin(xyz=(-0.72, 0.020, 0.315)),
        material=trim_dark,
        name="left_side_rail",
    )
    tailgate.visual(
        Box((1.50, 0.040, 0.05)),
        origin=Origin(xyz=(0.0, 0.020, 0.285)),
        material=trim_dark,
        name="mid_rail",
    )
    tailgate.visual(
        Box((1.34, 0.040, 0.16)),
        origin=Origin(xyz=(0.0, 0.020, 0.18)),
        material=trim_dark,
        name="lower_inner_panel",
    )
    tailgate.visual(
        Box((1.18, 0.028, 0.035)),
        origin=Origin(xyz=(0.0, 0.014, 0.135)),
        material=tailgate_paint,
        name="license_plate_pad",
    )
    tailgate.visual(
        Box((0.60, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.023, 0.319)),
        material=trim_dark,
        name="work_hinge_header",
    )

    work_panel = model.part("inner_work_panel")
    work_panel.visual(
        Box((1.34, 0.022, 0.198)),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=work_surface,
        name="work_surface_panel",
    )
    work_panel.visual(
        Box((1.26, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.189)),
        material=trim_dark,
        name="work_top_rib",
    )
    work_panel.visual(
        Box((0.030, 0.018, 0.168)),
        origin=Origin(xyz=(0.64, 0.0, 0.100)),
        material=trim_dark,
        name="right_work_edge_rib",
    )
    work_panel.visual(
        Box((0.030, 0.018, 0.168)),
        origin=Origin(xyz=(-0.64, 0.0, 0.100)),
        material=trim_dark,
        name="left_work_edge_rib",
    )
    work_panel.visual(
        Box((0.080, 0.014, 0.028)),
        origin=Origin(xyz=(-0.24, 0.0, 0.016)),
        material=trim_dark,
        name="left_hinge_strap",
    )
    work_panel.visual(
        Box((0.080, 0.014, 0.028)),
        origin=Origin(xyz=(0.24, 0.0, 0.016)),
        material=trim_dark,
        name="right_hinge_strap",
    )
    work_panel.visual(
        Cylinder(radius=0.007, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_dark,
        name="work_hinge_barrel",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=1.2,
            lower=0.0,
            upper=1.58,
        ),
    )
    model.articulation(
        "tailgate_to_work_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=work_panel,
        origin=Origin(xyz=(0.0, 0.010, 0.319)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=0.0,
            upper=1.58,
        ),
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
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("main_tailgate")
    work_panel = object_model.get_part("inner_work_panel")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    work_hinge = object_model.get_articulation("tailgate_to_work_panel")

    ctx.check("bed frame authored", bed_frame is not None)
    ctx.check("main tailgate authored", tailgate is not None)
    ctx.check("inner work panel authored", work_panel is not None)

    with ctx.pose({tailgate_hinge: 0.0, work_hinge: 0.0}):
        ctx.expect_gap(
            tailgate,
            bed_frame,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed tailgate seats on rear bed edge",
        )
        ctx.expect_overlap(
            tailgate,
            bed_frame,
            axes="xz",
            min_overlap=0.40,
            name="tailgate spans the bed opening",
        )
        ctx.expect_within(
            work_panel,
            tailgate,
            axes="xz",
            margin=0.03,
            name="stowed work panel stays inside the tailgate frame",
        )

        closed_tailgate_aabb = ctx.part_world_aabb(tailgate)
        closed_work_aabb = ctx.part_world_aabb(work_panel)

    with ctx.pose({tailgate_hinge: 1.50, work_hinge: 0.0}):
        open_tailgate_aabb = ctx.part_world_aabb(tailgate)

    with ctx.pose({tailgate_hinge: 0.0, work_hinge: 1.45}):
        open_work_aabb = ctx.part_world_aabb(work_panel)

    ctx.check(
        "tailgate rotates downward from the lower hinge",
        closed_tailgate_aabb is not None
        and open_tailgate_aabb is not None
        and open_tailgate_aabb[1][1] > closed_tailgate_aabb[1][1] + 0.35
        and open_tailgate_aabb[1][2] < closed_tailgate_aabb[1][2] - 0.35,
        details=f"closed={closed_tailgate_aabb}, open={open_tailgate_aabb}",
    )
    ctx.check(
        "work panel folds outward from the tailgate upper half",
        closed_work_aabb is not None
        and open_work_aabb is not None
        and open_work_aabb[1][1] > closed_work_aabb[1][1] + 0.16
        and open_work_aabb[1][2] < closed_work_aabb[1][2] - 0.14,
        details=f"closed={closed_work_aabb}, open={open_work_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
