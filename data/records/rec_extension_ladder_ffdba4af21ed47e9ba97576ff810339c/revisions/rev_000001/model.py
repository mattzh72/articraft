from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="pontoon_boat_boarding_ladder")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.79, 0.81, 0.83, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    clamp_bracket = model.part("clamp_bracket")
    clamp_bracket.visual(
        Box((0.18, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_aluminum,
        name="top_saddle",
    )
    clamp_bracket.visual(
        Box((0.014, 0.34, 0.19)),
        origin=Origin(xyz=(0.083, 0.0, -0.083)),
        material=brushed_aluminum,
        name="outer_backplate",
    )
    clamp_bracket.visual(
        Box((0.014, 0.28, 0.090)),
        origin=Origin(xyz=(-0.083, 0.0, -0.039)),
        material=brushed_aluminum,
        name="inner_hook",
    )
    clamp_bracket.visual(
        Box((0.060, 0.120, 0.016)),
        origin=Origin(xyz=(-0.040, 0.0, -0.086)),
        material=brushed_aluminum,
        name="clamp_bridge",
    )
    clamp_bracket.visual(
        Box((0.030, 0.180, 0.050)),
        origin=Origin(xyz=(-0.020, 0.0, -0.113)),
        material=rubber_black,
        name="pressure_pad",
    )
    clamp_bracket.visual(
        Cylinder(radius=0.009, length=0.102),
        origin=Origin(xyz=(0.032, 0.0, -0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="clamp_screw",
    )
    clamp_bracket.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.101, 0.0, -0.106), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="turn_knob",
    )
    clamp_bracket.visual(
        Box((0.018, 0.050, 0.060)),
        origin=Origin(xyz=(0.079, 0.145, -0.160)),
        material=satin_aluminum,
        name="left_hinge_lug",
    )
    clamp_bracket.visual(
        Box((0.018, 0.050, 0.060)),
        origin=Origin(xyz=(0.079, -0.145, -0.160)),
        material=satin_aluminum,
        name="right_hinge_lug",
    )
    clamp_bracket.inertial = Inertial.from_geometry(
        Box((0.18, 0.34, 0.22)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
    )

    ladder_frame = model.part("ladder_frame")
    ladder_frame.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.011, 0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="left_hinge_barrel",
    )
    ladder_frame.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.011, -0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="right_hinge_barrel",
    )
    ladder_frame.visual(
        Box((0.052, 0.024, 0.050)),
        origin=Origin(xyz=(0.046, 0.145, -0.020)),
        material=satin_aluminum,
        name="left_hinge_strap",
    )
    ladder_frame.visual(
        Box((0.052, 0.024, 0.050)),
        origin=Origin(xyz=(0.046, -0.145, -0.020)),
        material=satin_aluminum,
        name="right_hinge_strap",
    )
    ladder_frame.visual(
        Cylinder(radius=0.016, length=0.840),
        origin=Origin(xyz=(0.080, 0.145, -0.430)),
        material=brushed_aluminum,
        name="left_stile",
    )
    ladder_frame.visual(
        Cylinder(radius=0.016, length=0.840),
        origin=Origin(xyz=(0.080, -0.145, -0.430)),
        material=brushed_aluminum,
        name="right_stile",
    )
    ladder_frame.visual(
        Cylinder(radius=0.012, length=0.290),
        origin=Origin(xyz=(0.070, 0.0, -0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="top_crossbrace",
    )
    ladder_frame.visual(
        Box((0.105, 0.290, 0.018)),
        origin=Origin(xyz=(0.092, 0.0, -0.220)),
        material=satin_aluminum,
        name="upper_step",
    )
    ladder_frame.visual(
        Box((0.070, 0.220, 0.004)),
        origin=Origin(xyz=(0.104, 0.0, -0.209)),
        material=rubber_black,
        name="upper_step_pad",
    )
    ladder_frame.visual(
        Cylinder(radius=0.012, length=0.260),
        origin=Origin(xyz=(0.073, 0.0, -0.760), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="bottom_crossbrace",
    )
    ladder_frame.visual(
        Box((0.044, 0.030, 0.056)),
        origin=Origin(xyz=(0.090, 0.132, -0.420)),
        material=satin_aluminum,
        name="middle_left_mount",
    )
    ladder_frame.visual(
        Box((0.044, 0.030, 0.056)),
        origin=Origin(xyz=(0.090, -0.132, -0.420)),
        material=satin_aluminum,
        name="middle_right_mount",
    )
    ladder_frame.visual(
        Box((0.044, 0.030, 0.056)),
        origin=Origin(xyz=(0.090, 0.132, -0.645)),
        material=satin_aluminum,
        name="lower_left_mount",
    )
    ladder_frame.visual(
        Box((0.044, 0.030, 0.056)),
        origin=Origin(xyz=(0.090, -0.132, -0.645)),
        material=satin_aluminum,
        name="lower_right_mount",
    )
    ladder_frame.inertial = Inertial.from_geometry(
        Box((0.18, 0.34, 0.86)),
        mass=7.5,
        origin=Origin(xyz=(0.090, 0.0, -0.430)),
    )

    def add_folding_step(part_name: str, tread_name: str, pad_name: str) -> None:
        step = model.part(part_name)
        step.visual(
            Box((0.095, 0.240, 0.016)),
            origin=Origin(xyz=(0.0475, 0.0, 0.0)),
            material=satin_aluminum,
            name=tread_name,
        )
        step.visual(
            Box((0.065, 0.180, 0.004)),
            origin=Origin(xyz=(0.055, 0.0, 0.010)),
            material=rubber_black,
            name=pad_name,
        )
        step.visual(
            Cylinder(radius=0.010, length=0.210),
            origin=Origin(xyz=(0.092, 0.0, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_aluminum,
            name=f"{part_name}_front_roller",
        )
        step.visual(
            Box((0.030, 0.028, 0.040)),
            origin=Origin(xyz=(0.015, 0.108, 0.0)),
            material=satin_aluminum,
            name=f"{part_name}_left_cheek",
        )
        step.visual(
            Box((0.030, 0.028, 0.040)),
            origin=Origin(xyz=(0.015, -0.108, 0.0)),
            material=satin_aluminum,
            name=f"{part_name}_right_cheek",
        )
        step.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.007, 0.108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardware,
            name=f"{part_name}_left_barrel",
        )
        step.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.007, -0.108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardware,
            name=f"{part_name}_right_barrel",
        )
        step.inertial = Inertial.from_geometry(
            Box((0.10, 0.24, 0.05)),
            mass=1.2,
            origin=Origin(xyz=(0.050, 0.0, 0.0)),
        )

    add_folding_step("middle_step", "middle_tread", "middle_pad")
    add_folding_step("lower_step", "lower_tread", "lower_pad")

    model.articulation(
        "clamp_to_ladder",
        ArticulationType.REVOLUTE,
        parent=clamp_bracket,
        child=ladder_frame,
        origin=Origin(xyz=(0.090, 0.0, -0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.6,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "frame_to_middle_step",
        ArticulationType.REVOLUTE,
        parent=ladder_frame,
        child="middle_step",
        origin=Origin(xyz=(0.115, 0.0, -0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "frame_to_lower_step",
        ArticulationType.REVOLUTE,
        parent=ladder_frame,
        child="lower_step",
        origin=Origin(xyz=(0.115, 0.0, -0.645)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
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

    clamp_bracket = object_model.get_part("clamp_bracket")
    ladder_frame = object_model.get_part("ladder_frame")
    middle_step = object_model.get_part("middle_step")
    lower_step = object_model.get_part("lower_step")

    clamp_to_ladder = object_model.get_articulation("clamp_to_ladder")
    frame_to_middle_step = object_model.get_articulation("frame_to_middle_step")
    frame_to_lower_step = object_model.get_articulation("frame_to_lower_step")

    with ctx.pose(
        {
            clamp_to_ladder: 0.0,
            frame_to_middle_step: 0.0,
            frame_to_lower_step: 0.0,
        }
    ):
        ctx.expect_contact(
            ladder_frame,
            clamp_bracket,
            elem_a="left_hinge_barrel",
            elem_b="left_hinge_lug",
            contact_tol=1e-6,
            name="main hinge barrel bears on the clamp lug",
        )
        ctx.expect_contact(
            middle_step,
            ladder_frame,
            elem_a="middle_step_left_barrel",
            elem_b="middle_left_mount",
            contact_tol=1e-6,
            name="middle step hinge barrel bears on the stile mount",
        )
        ctx.expect_contact(
            lower_step,
            ladder_frame,
            elem_a="lower_step_left_barrel",
            elem_b="lower_left_mount",
            contact_tol=1e-6,
            name="lower step hinge barrel bears on the stile mount",
        )
        ctx.expect_overlap(
            middle_step,
            ladder_frame,
            axes="y",
            min_overlap=0.20,
            name="middle step spans between the ladder stiles",
        )
        ctx.expect_overlap(
            lower_step,
            ladder_frame,
            axes="y",
            min_overlap=0.20,
            name="lower step spans between the ladder stiles",
        )
        ctx.expect_gap(
            ladder_frame,
            middle_step,
            axis="z",
            positive_elem="upper_step",
            min_gap=0.12,
            name="middle step sits below the fixed upper step",
        )
        ctx.expect_gap(
            middle_step,
            lower_step,
            axis="z",
            min_gap=0.16,
            name="lower step sits below the middle step",
        )

        deployed_frame_box = ctx.part_world_aabb(ladder_frame)
        deployed_middle_box = ctx.part_world_aabb(middle_step)
        deployed_lower_box = ctx.part_world_aabb(lower_step)

    with ctx.pose({clamp_to_ladder: 1.0}):
        folded_frame_box = ctx.part_world_aabb(ladder_frame)
        ctx.check(
            "main stile frame folds upward from the clamp",
            deployed_frame_box is not None
            and folded_frame_box is not None
            and folded_frame_box[0][2] > deployed_frame_box[0][2] + 0.20,
            details=f"deployed={deployed_frame_box}, folded={folded_frame_box}",
        )

    with ctx.pose({frame_to_middle_step: 1.30, frame_to_lower_step: 1.30}):
        folded_middle_box = ctx.part_world_aabb(middle_step)
        folded_lower_box = ctx.part_world_aabb(lower_step)
        ctx.check(
            "middle step folds upward at its stile hinges",
            deployed_middle_box is not None
            and folded_middle_box is not None
            and folded_middle_box[1][2] > deployed_middle_box[1][2] + 0.07,
            details=f"deployed={deployed_middle_box}, folded={folded_middle_box}",
        )
        ctx.check(
            "lower step folds upward at its stile hinges",
            deployed_lower_box is not None
            and folded_lower_box is not None
            and folded_lower_box[1][2] > deployed_lower_box[1][2] + 0.07,
            details=f"deployed={deployed_lower_box}, folded={folded_lower_box}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
