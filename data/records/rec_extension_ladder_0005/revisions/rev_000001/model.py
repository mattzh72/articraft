from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

SAFE_ASSET_ROOT = "/tmp"

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        return SAFE_ASSET_ROOT


os.getcwd = _safe_getcwd
os.chdir(SAFE_ASSET_ROOT)

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_OUTER_WIDTH = 0.56
FRAME_OUTER_LENGTH = 0.76
FRAME_DEPTH = 0.12
FRAME_OPENING_WIDTH = 0.46

UPPER_LENGTH = 0.98
FLY_LENGTH = 1.02
UPPER_STEP_POSITIONS = (0.18, 0.38, 0.58, 0.78)
FLY_STEP_POSITIONS = (0.18, 0.38, 0.58, 0.78)

UPPER_STILE_X = 0.205
UPPER_GUIDE_X = 0.175
FLY_RAIL_X = 0.188
HINGE_Z = -0.320


def _add_step_supports(section_part, prefix: str, positions: tuple[float, ...], material, *, span: float) -> None:
    for index, step_y in enumerate(positions, start=1):
        section_part.visual(
            Box((span, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, step_y + 0.006, 0.015)),
            material=material,
            name=f"{prefix}_step_{index}_support_bar",
        )


def _build_step_part(model: ArticulatedObject, part_name: str, tread_material, hinge_material):
    step = model.part(part_name)
    step.visual(
        Box((0.120, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.026)),
        material=hinge_material,
        name="hinge_leaf",
    )
    step.visual(
        Box((0.060, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, -0.005, 0.051)),
        material=hinge_material,
        name="center_web",
    )
    step.visual(
        Box((0.290, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, -0.020, 0.086)),
        material=tread_material,
        name="tread",
    )
    step.visual(
        Box((0.290, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.028, 0.112)),
        material=tread_material,
        name="nose_lip",
    )
    step.inertial = Inertial.from_geometry(
        Box((0.300, 0.040, 0.124)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.006, 0.062)),
    )
    return step


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="attic_ladder")

    frame_paint = model.material("frame_paint", rgba=(0.93, 0.92, 0.89, 1.0))
    pine = model.material("pine", rgba=(0.77, 0.63, 0.42, 1.0))
    tread_pine = model.material("tread_pine", rgba=(0.71, 0.56, 0.35, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.55, 0.58, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    frame = model.part("attic_frame")
    frame.visual(
        Box((0.050, FRAME_DEPTH, FRAME_OUTER_LENGTH)),
        origin=Origin(xyz=(-0.255, -0.060, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((0.050, FRAME_DEPTH, FRAME_OUTER_LENGTH)),
        origin=Origin(xyz=(0.255, -0.060, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_OPENING_WIDTH, FRAME_DEPTH, 0.060)),
        origin=Origin(xyz=(0.0, -0.060, 0.350)),
        material=frame_paint,
        name="front_header",
    )
    frame.visual(
        Box((FRAME_OPENING_WIDTH, FRAME_DEPTH, 0.060)),
        origin=Origin(xyz=(0.0, -0.060, -0.350)),
        material=frame_paint,
        name="rear_header",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, 0.026, FRAME_OUTER_LENGTH + 0.060)),
        origin=Origin(xyz=(0.0, -0.103, 0.0)),
        material=frame_paint,
        name="ceiling_flange",
    )
    frame.visual(
        Box((0.380, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, -0.021, HINGE_Z)),
        material=steel,
        name="pivot_spine",
    )
    frame.visual(
        Box((0.032, 0.024, 0.100)),
        origin=Origin(xyz=(-0.160, -0.012, HINGE_Z)),
        material=steel,
        name="left_pivot_block",
    )
    frame.visual(
        Box((0.032, 0.024, 0.100)),
        origin=Origin(xyz=(0.160, -0.012, HINGE_Z)),
        material=steel,
        name="right_pivot_block",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_LENGTH)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
    )

    upper = model.part("upper_section")
    upper.visual(
        Box((0.032, 0.024, 0.050)),
        origin=Origin(xyz=(-0.160, 0.012, 0.0)),
        material=steel,
        name="left_hanger_tab",
    )
    upper.visual(
        Box((0.032, 0.024, 0.050)),
        origin=Origin(xyz=(0.160, 0.012, 0.0)),
        material=steel,
        name="right_hanger_tab",
    )
    upper.visual(
        Box((0.392, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=pine,
        name="top_spreader",
    )
    upper.visual(
        Box((0.022, UPPER_LENGTH, 0.030)),
        origin=Origin(xyz=(-UPPER_STILE_X, UPPER_LENGTH * 0.5, 0.0)),
        material=pine,
        name="left_outer_stile",
    )
    upper.visual(
        Box((0.022, UPPER_LENGTH, 0.030)),
        origin=Origin(xyz=(UPPER_STILE_X, UPPER_LENGTH * 0.5, 0.0)),
        material=pine,
        name="right_outer_stile",
    )
    upper.visual(
        Box((0.010, 0.840, 0.018)),
        origin=Origin(xyz=(-UPPER_GUIDE_X, 0.430, 0.0)),
        material=steel,
        name="left_guide_bar",
    )
    upper.visual(
        Box((0.010, 0.840, 0.018)),
        origin=Origin(xyz=(UPPER_GUIDE_X, 0.430, 0.0)),
        material=steel,
        name="right_guide_bar",
    )
    upper.visual(
        Box((0.350, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.500, 0.0)),
        material=steel,
        name="mid_rail_tie",
    )
    _add_step_supports(upper, "upper", UPPER_STEP_POSITIONS, steel, span=0.410)
    upper.inertial = Inertial.from_geometry(
        Box((0.430, UPPER_LENGTH, 0.110)),
        mass=7.2,
        origin=Origin(xyz=(0.0, UPPER_LENGTH * 0.5, 0.015)),
    )

    fly = model.part("fly_section")
    fly.visual(
        Box((0.016, 0.040, 0.032)),
        origin=Origin(xyz=(-FLY_RAIL_X, 0.020, 0.005)),
        material=pine,
        name="left_carriage_cheek",
    )
    fly.visual(
        Box((0.016, 0.040, 0.032)),
        origin=Origin(xyz=(FLY_RAIL_X, 0.020, 0.005)),
        material=pine,
        name="right_carriage_cheek",
    )
    fly.visual(
        Box((0.392, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.030, 0.016)),
        material=pine,
        name="top_carriage",
    )
    fly.visual(
        Box((0.008, FLY_LENGTH, 0.022)),
        origin=Origin(xyz=(-FLY_RAIL_X, FLY_LENGTH * 0.5, 0.0)),
        material=pine,
        name="left_fly_rail",
    )
    fly.visual(
        Box((0.008, FLY_LENGTH, 0.022)),
        origin=Origin(xyz=(FLY_RAIL_X, FLY_LENGTH * 0.5, 0.0)),
        material=pine,
        name="right_fly_rail",
    )
    fly.visual(
        Box((0.392, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.540, 0.0)),
        material=steel,
        name="mid_tie_bar",
    )
    fly.visual(
        Box((0.392, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.900, 0.0)),
        material=steel,
        name="lower_tie_bar",
    )
    fly.visual(
        Box((0.004, 0.180, 0.018)),
        origin=Origin(xyz=(-0.182, 0.180, 0.0)),
        material=steel,
        name="left_slider_pad",
    )
    fly.visual(
        Box((0.004, 0.180, 0.018)),
        origin=Origin(xyz=(0.182, 0.180, 0.0)),
        material=steel,
        name="right_slider_pad",
    )
    fly.visual(
        Box((0.060, 0.040, 0.030)),
        origin=Origin(xyz=(-0.183, 0.995, 0.003)),
        material=dark_rubber,
        name="left_foot",
    )
    fly.visual(
        Box((0.060, 0.040, 0.030)),
        origin=Origin(xyz=(0.183, 0.995, 0.003)),
        material=dark_rubber,
        name="right_foot",
    )
    _add_step_supports(fly, "fly", FLY_STEP_POSITIONS, steel, span=0.376)
    fly.inertial = Inertial.from_geometry(
        Box((0.400, FLY_LENGTH, 0.110)),
        mass=5.8,
        origin=Origin(xyz=(0.0, FLY_LENGTH * 0.5, 0.015)),
    )

    model.articulation(
        "frame_to_upper_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.75, lower=-1.22, upper=0.0),
    )
    model.articulation(
        "upper_to_fly_slide",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=fly,
        origin=Origin(xyz=(0.0, 0.330, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.40, lower=0.0, upper=0.340),
    )

    for index, step_y in enumerate(UPPER_STEP_POSITIONS, start=1):
        step = _build_step_part(model, f"upper_step_{index}", tread_pine, steel)
        model.articulation(
            f"upper_step_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=upper,
            child=step,
            origin=Origin(xyz=(0.0, step_y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=1.25),
        )

    for index, step_y in enumerate(FLY_STEP_POSITIONS, start=1):
        step = _build_step_part(model, f"fly_step_{index}", tread_pine, steel)
        model.articulation(
            f"fly_step_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=fly,
            child=step,
            origin=Origin(xyz=(0.0, step_y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)

    frame = object_model.get_part("attic_frame")
    upper = object_model.get_part("upper_section")
    fly = object_model.get_part("fly_section")
    upper_fold = object_model.get_articulation("frame_to_upper_fold")
    fly_slide = object_model.get_articulation("upper_to_fly_slide")

    upper_steps = [object_model.get_part(f"upper_step_{index}") for index in range(1, 5)]
    fly_steps = [object_model.get_part(f"fly_step_{index}") for index in range(1, 5)]
    upper_step_joints = [object_model.get_articulation(f"upper_step_{index}_hinge") for index in range(1, 5)]
    fly_step_joints = [object_model.get_articulation(f"fly_step_{index}_hinge") for index in range(1, 5)]

    frame_left_pivot_block = frame.get_visual("left_pivot_block")
    frame_right_pivot_block = frame.get_visual("right_pivot_block")
    upper_left_hanger_tab = upper.get_visual("left_hanger_tab")
    upper_right_hanger_tab = upper.get_visual("right_hanger_tab")
    upper_left_guide_bar = upper.get_visual("left_guide_bar")
    upper_right_guide_bar = upper.get_visual("right_guide_bar")
    upper_right_outer_stile = upper.get_visual("right_outer_stile")
    fly_left_slider_pad = fly.get_visual("left_slider_pad")
    fly_right_slider_pad = fly.get_visual("right_slider_pad")
    fly_left_rail = fly.get_visual("left_fly_rail")
    fly_right_rail = fly.get_visual("right_fly_rail")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(upper, frame, elem_a=upper_left_hanger_tab, elem_b=frame_left_pivot_block)
    ctx.expect_contact(upper, frame, elem_a=upper_right_hanger_tab, elem_b=frame_right_pivot_block)
    ctx.expect_contact(fly, upper, elem_a=fly_left_slider_pad, elem_b=upper_left_guide_bar)
    ctx.expect_contact(fly, upper, elem_a=fly_right_slider_pad, elem_b=upper_right_guide_bar)
    ctx.expect_gap(
        fly,
        upper,
        axis="x",
        min_gap=0.001,
        positive_elem=fly_right_rail,
        negative_elem=upper_right_guide_bar,
        name="fly right rail clears right guide bar",
    )
    ctx.expect_gap(
        upper,
        fly,
        axis="x",
        min_gap=0.001,
        positive_elem=upper_left_guide_bar,
        negative_elem=fly_left_rail,
        name="fly left rail clears left guide bar",
    )
    ctx.expect_gap(
        upper,
        fly,
        axis="x",
        min_gap=0.001,
        positive_elem=upper_right_outer_stile,
        negative_elem=fly_right_rail,
        name="outer stile captures fly rail",
    )

    for prefix, parent, steps, joints in (
        ("upper", upper, upper_steps, upper_step_joints),
        ("fly", fly, fly_steps, fly_step_joints),
    ):
        for index, (step, step_joint) in enumerate(zip(steps, joints), start=1):
            support_bar = parent.get_visual(f"{prefix}_step_{index}_support_bar")
            hinge_leaf = step.get_visual("hinge_leaf")
            tread = step.get_visual("tread")

            ctx.expect_contact(
                step,
                parent,
                elem_a=hinge_leaf,
                elem_b=support_bar,
                name=f"{prefix} step {index} hinge leaf seats on support bar",
            )
            ctx.expect_gap(
                step,
                parent,
                axis="z",
                min_gap=0.030,
                positive_elem=tread,
                negative_elem=support_bar,
                name=f"{prefix} step {index} tread stands proud when deployed",
            )

            rest_aabb = ctx.part_world_aabb(step)
            with ctx.pose({step_joint: 1.10}):
                folded_aabb = ctx.part_world_aabb(step)
                ctx.fail_if_isolated_parts(name=f"{prefix} step {index} folded no floating")
                if rest_aabb is not None and folded_aabb is not None:
                    ctx.check(
                        f"{prefix} step {index} folds downward",
                        folded_aabb[1][2] < rest_aabb[1][2] - 0.015,
                        details=(
                            f"Expected folded max-z at least 15 mm lower; "
                            f"rest={rest_aabb[1][2]:.3f} folded={folded_aabb[1][2]:.3f}"
                        ),
                    )

    rest_fly_aabb = ctx.part_world_aabb(fly)
    if rest_fly_aabb is not None:
        ctx.check(
            "fly section extends below upper section at rest",
            rest_fly_aabb[1][1] > 1.30,
            details=f"Expected fly max-y above 1.30 m, got {rest_fly_aabb[1][1]:.3f}",
        )

    with ctx.pose({fly_slide: 0.30}):
        extended_fly_aabb = ctx.part_world_aabb(fly)
        ctx.expect_contact(fly, upper, elem_a=fly_left_slider_pad, elem_b=upper_left_guide_bar)
        ctx.expect_contact(fly, upper, elem_a=fly_right_slider_pad, elem_b=upper_right_guide_bar)
        ctx.expect_gap(
            fly,
            upper,
            axis="x",
            min_gap=0.001,
            positive_elem=fly_right_rail,
            negative_elem=upper_right_guide_bar,
            name="fly rail clears guide bar when extended",
        )
        ctx.expect_gap(
            upper,
            fly,
            axis="x",
            min_gap=0.001,
            positive_elem=upper_right_outer_stile,
            negative_elem=fly_right_rail,
            name="outer stile still captures fly rail when extended",
        )
        ctx.fail_if_isolated_parts(name="fly slide extended no floating")
        if rest_fly_aabb is not None and extended_fly_aabb is not None:
            ctx.check(
                "fly section telescopes outward",
                extended_fly_aabb[1][1] > rest_fly_aabb[1][1] + 0.20,
                details=(
                    f"Expected fly max-y to grow by at least 0.20 m; "
                    f"rest={rest_fly_aabb[1][1]:.3f} extended={extended_fly_aabb[1][1]:.3f}"
                ),
            )

    deployed_upper_aabb = ctx.part_world_aabb(upper)
    if deployed_upper_aabb is not None:
        with ctx.pose({upper_fold: -1.15}):
            stowed_upper_aabb = ctx.part_world_aabb(upper)
            ctx.fail_if_isolated_parts(name="upper folded no floating")
            if stowed_upper_aabb is not None:
                ctx.check(
                    "upper section shortens ladder projection when folded",
                    (stowed_upper_aabb[1][1] - stowed_upper_aabb[0][1])
                    < (deployed_upper_aabb[1][1] - deployed_upper_aabb[0][1]) - 0.25,
                    details=(
                        f"Expected folded y-span to shrink by at least 0.25 m; "
                        f"rest={deployed_upper_aabb[1][1] - deployed_upper_aabb[0][1]:.3f} "
                        f"folded={stowed_upper_aabb[1][1] - stowed_upper_aabb[0][1]:.3f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
