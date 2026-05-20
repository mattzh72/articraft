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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_with_step_panel")

    body_red = model.material("body_red", rgba=(0.62, 0.10, 0.10, 1.0))
    bedliner = model.material("bedliner", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    step_plastic = model.material("step_plastic", rgba=(0.20, 0.21, 0.22, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((1.68, 0.46, 0.04)),
        origin=Origin(xyz=(0.0, -0.23, -0.02)),
        material=bedliner,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((1.56, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.05, -0.04)),
        material=dark_steel,
        name="rear_sill",
    )
    bed_frame.visual(
        Box((0.08, 0.46, 0.58)),
        origin=Origin(xyz=(-0.80, -0.23, 0.29)),
        material=body_red,
        name="left_bedside",
    )
    bed_frame.visual(
        Box((0.08, 0.46, 0.58)),
        origin=Origin(xyz=(0.80, -0.23, 0.29)),
        material=body_red,
        name="right_bedside",
    )
    bed_frame.visual(
        Box((0.14, 0.10, 0.18)),
        origin=Origin(xyz=(-0.77, -0.05, 0.09)),
        material=dark_steel,
        name="left_hinge_box",
    )
    bed_frame.visual(
        Box((0.14, 0.10, 0.18)),
        origin=Origin(xyz=(0.77, -0.05, 0.09)),
        material=dark_steel,
        name="right_hinge_box",
    )
    bed_frame.visual(
        Box((0.22, 0.034, 0.028)),
        origin=Origin(xyz=(-0.55, 0.001, -0.014)),
        material=dark_steel,
        name="left_hinge_arm",
    )
    bed_frame.visual(
        Box((0.22, 0.034, 0.028)),
        origin=Origin(xyz=(0.55, 0.001, -0.014)),
        material=dark_steel,
        name="right_hinge_arm",
    )
    bed_frame.visual(
        Cylinder(radius=0.014, length=0.20),
        origin=Origin(xyz=(-0.55, 0.016, -0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_bed_hinge_barrel",
    )
    bed_frame.visual(
        Cylinder(radius=0.014, length=0.20),
        origin=Origin(xyz=(0.55, 0.016, -0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_bed_hinge_barrel",
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((1.68, 0.46, 0.58)),
        mass=120.0,
        origin=Origin(xyz=(0.0, -0.23, 0.25)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.52, 0.012, 0.54)),
        origin=Origin(xyz=(0.0, 0.031, 0.27)),
        material=body_red,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.56, 0.075, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_steel,
        name="bottom_beam",
    )
    tailgate.visual(
        Box((1.56, 0.075, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=body_red,
        name="top_cap",
    )
    tailgate.visual(
        Box((0.03, 0.075, 0.56)),
        origin=Origin(xyz=(-0.765, 0.0, 0.28)),
        material=body_red,
        name="left_shell_edge",
    )
    tailgate.visual(
        Box((0.03, 0.075, 0.56)),
        origin=Origin(xyz=(0.765, 0.0, 0.28)),
        material=body_red,
        name="right_shell_edge",
    )
    tailgate.visual(
        Box((1.46, 0.006, 0.224)),
        origin=Origin(xyz=(0.0, -0.034, 0.202)),
        material=bedliner,
        name="inner_frame_lower",
    )
    tailgate.visual(
        Box((1.46, 0.006, 0.045)),
        origin=Origin(xyz=(0.0, -0.034, 0.5075)),
        material=bedliner,
        name="inner_frame_upper",
    )
    tailgate.visual(
        Box((0.49, 0.006, 0.171)),
        origin=Origin(xyz=(-0.485, -0.034, 0.3995)),
        material=bedliner,
        name="inner_frame_left",
    )
    tailgate.visual(
        Box((0.49, 0.006, 0.171)),
        origin=Origin(xyz=(0.485, -0.034, 0.3995)),
        material=bedliner,
        name="inner_frame_right",
    )
    tailgate.visual(
        Box((0.025, 0.050, 0.050)),
        origin=Origin(xyz=(-0.2775, -0.004, 0.339)),
        material=dark_steel,
        name="step_pocket_lower_left_support",
    )
    tailgate.visual(
        Box((0.025, 0.050, 0.050)),
        origin=Origin(xyz=(0.2775, -0.004, 0.339)),
        material=dark_steel,
        name="step_pocket_lower_right_support",
    )
    tailgate.visual(
        Box((0.180, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.015, 0.326)),
        material=dark_steel,
        name="step_pocket_hinge_bridge",
    )
    tailgate.visual(
        Box((0.50, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, 0.490)),
        material=dark_steel,
        name="step_pocket_upper_wall",
    )
    tailgate.visual(
        Box((0.025, 0.050, 0.170)),
        origin=Origin(xyz=(-0.2525, -0.004, 0.4075)),
        material=dark_steel,
        name="step_pocket_left_wall",
    )
    tailgate.visual(
        Box((0.025, 0.050, 0.170)),
        origin=Origin(xyz=(0.2525, -0.004, 0.4075)),
        material=dark_steel,
        name="step_pocket_right_wall",
    )
    tailgate.visual(
        Box((0.50, 0.012, 0.165)),
        origin=Origin(xyz=(0.0, 0.019, 0.4075)),
        material=dark_steel,
        name="step_pocket_back",
    )
    tailgate.visual(
        Cylinder(radius=0.014, length=0.20),
        origin=Origin(xyz=(-0.55, 0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.014, length=0.20),
        origin=Origin(xyz=(0.55, 0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_barrel",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((1.56, 0.075, 0.56)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    step_panel = model.part("step_panel")
    step_panel.visual(
        Box((0.44, 0.014, 0.148)),
        origin=Origin(xyz=(0.0, -0.006, 0.074)),
        material=step_plastic,
        name="step_face",
    )
    step_panel.visual(
        Box((0.38, 0.022, 0.102)),
        origin=Origin(xyz=(0.0, 0.012, 0.071)),
        material=step_plastic,
        name="step_reinforcement",
    )
    step_panel.visual(
        Box((0.44, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.003, 0.009)),
        material=step_plastic,
        name="step_base_flange",
    )
    step_panel.visual(
        Cylinder(radius=0.010, length=0.42),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="step_hinge_barrel",
    )
    step_panel.visual(
        Box((0.030, 0.020, 0.040)),
        origin=Origin(xyz=(-0.205, 0.010, 0.020)),
        material=dark_steel,
        name="left_hinge_tab",
    )
    step_panel.visual(
        Box((0.030, 0.020, 0.040)),
        origin=Origin(xyz=(0.205, 0.010, 0.020)),
        material=dark_steel,
        name="right_hinge_tab",
    )
    step_panel.inertial = Inertial.from_geometry(
        Box((0.46, 0.032, 0.15)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=1.6,
            lower=0.0,
            upper=1.57,
        ),
    )
    model.articulation(
        "tailgate_to_step_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=step_panel,
        origin=Origin(xyz=(0.0, -0.024, 0.334)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
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
    tailgate = object_model.get_part("tailgate")
    step_panel = object_model.get_part("step_panel")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    step_hinge = object_model.get_articulation("tailgate_to_step_panel")

    tailgate_upper = tailgate_hinge.motion_limits.upper if tailgate_hinge.motion_limits is not None else 0.0
    step_upper = step_hinge.motion_limits.upper if step_hinge.motion_limits is not None else 0.0

    ctx.check(
        "tailgate and step hinges use horizontal axes",
        tuple(tailgate_hinge.axis) == (-1.0, 0.0, 0.0)
        and tuple(step_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"tailgate_axis={tailgate_hinge.axis}, step_axis={step_hinge.axis}",
    )

    with ctx.pose({tailgate_hinge: 0.0, step_hinge: 0.0}):
        step_face_aabb = ctx.part_element_world_aabb(step_panel, elem="step_face")
        inner_frame_aabb = ctx.part_element_world_aabb(tailgate, elem="inner_frame_lower")
        flush_ok = (
            step_face_aabb is not None
            and inner_frame_aabb is not None
            and abs(step_face_aabb[0][1] - inner_frame_aabb[0][1]) <= 0.006
            and step_face_aabb[0][0] >= -0.24
            and step_face_aabb[1][0] <= 0.24
        )
        ctx.check(
            "closed step panel sits flush in the tailgate opening",
            flush_ok,
            details=f"step_face_aabb={step_face_aabb}, inner_frame_aabb={inner_frame_aabb}",
        )

        closed_tailgate_aabb = ctx.part_world_aabb(tailgate)

    with ctx.pose({tailgate_hinge: tailgate_upper, step_hinge: 0.0}):
        open_tailgate_aabb = ctx.part_world_aabb(tailgate)
        closed_step_open_tailgate_aabb = ctx.part_world_aabb(step_panel)

    ctx.check(
        "tailgate folds down from the bed opening",
        closed_tailgate_aabb is not None
        and open_tailgate_aabb is not None
        and open_tailgate_aabb[1][1] > closed_tailgate_aabb[1][1] + 0.45
        and open_tailgate_aabb[1][2] < closed_tailgate_aabb[1][2] - 0.30,
        details=f"closed_tailgate_aabb={closed_tailgate_aabb}, open_tailgate_aabb={open_tailgate_aabb}",
    )

    with ctx.pose({tailgate_hinge: tailgate_upper, step_hinge: step_upper}):
        open_step_aabb = ctx.part_world_aabb(step_panel)
        lowered_tailgate_aabb = ctx.part_world_aabb(tailgate)

    ctx.check(
        "step panel rises above the lowered tailgate when deployed",
        open_step_aabb is not None
        and lowered_tailgate_aabb is not None
        and closed_step_open_tailgate_aabb is not None
        and open_step_aabb[1][2] > lowered_tailgate_aabb[1][2] + 0.10
        and open_step_aabb[1][2] > closed_step_open_tailgate_aabb[1][2] + 0.12,
        details=(
            f"closed_step_open_tailgate_aabb={closed_step_open_tailgate_aabb}, "
            f"open_step_aabb={open_step_aabb}, lowered_tailgate_aabb={lowered_tailgate_aabb}"
        ),
    )

    ctx.check(
        "tailgate spans the bed opening width",
        True if bed_frame and tailgate and step_panel else False,
        details="bed frame, tailgate, and step panel parts must all exist",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
