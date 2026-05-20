from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LEN = 0.360
OUTER_WIDTH = 0.072
OUTER_HEIGHT = 0.014
SHEET = 0.0025
LIP_WIDTH = 0.006

RUNNER_LEN = 0.300
RUNNER_WIDTH = 0.050
RUNNER_BODY_HEIGHT = 0.0055
RUNNER_FRONT_FACE_LEN = 0.010
RUNNER_FRONT_FACE_WIDTH = 0.018
RUNNER_FRONT_FACE_HEIGHT = 0.010

JOINT_REAR_X = 0.055
MAX_EXTENSION = 0.180

REAR_STOP_LEN = 0.012
REAR_STOP_WIDTH = 0.018
REAR_STOP_HEIGHT = 0.0075

FRONT_STOP_LEN = 0.012
FRONT_STOP_WIDTH = 0.012
FRONT_STOP_HEIGHT = 0.0075

INNER_STOP_LUG_LEN = 0.012
INNER_STOP_LUG_WIDTH = 0.010
INNER_STOP_LUG_HEIGHT = 0.004
INNER_STOP_LUG_FRONT_X = OUTER_LEN - FRONT_STOP_LEN - JOINT_REAR_X - MAX_EXTENSION

REAR_STOP_Y = 0.020
FRONT_STOP_Y = 0.021
INNER_STOP_LUG_Y = 0.020
FRONT_FACE_CENTER_Z = 0.0065

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_extension_slide")

    zinc_steel = model.material("zinc_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.18, 0.18, 0.20, 1.0))

    outer_section = model.part("outer_section")
    outer_section.visual(
        Box((OUTER_LEN, OUTER_WIDTH, SHEET)),
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, SHEET / 2.0)),
        material=zinc_steel,
        name="base_plate",
    )
    outer_section.visual(
        Box((OUTER_LEN, SHEET, OUTER_HEIGHT - SHEET)),
        origin=Origin(
            xyz=(
                OUTER_LEN / 2.0,
                OUTER_WIDTH / 2.0 - SHEET / 2.0,
                SHEET + (OUTER_HEIGHT - SHEET) / 2.0,
            )
        ),
        material=zinc_steel,
        name="left_wall",
    )
    outer_section.visual(
        Box((OUTER_LEN, SHEET, OUTER_HEIGHT - SHEET)),
        origin=Origin(
            xyz=(
                OUTER_LEN / 2.0,
                -OUTER_WIDTH / 2.0 + SHEET / 2.0,
                SHEET + (OUTER_HEIGHT - SHEET) / 2.0,
            )
        ),
        material=zinc_steel,
        name="right_wall",
    )
    outer_section.visual(
        Box((OUTER_LEN, LIP_WIDTH, SHEET)),
        origin=Origin(
            xyz=(
                OUTER_LEN / 2.0,
                OUTER_WIDTH / 2.0 - SHEET - LIP_WIDTH / 2.0,
                OUTER_HEIGHT - SHEET / 2.0,
            )
        ),
        material=zinc_steel,
        name="left_lip",
    )
    outer_section.visual(
        Box((OUTER_LEN, LIP_WIDTH, SHEET)),
        origin=Origin(
            xyz=(
                OUTER_LEN / 2.0,
                -OUTER_WIDTH / 2.0 + SHEET + LIP_WIDTH / 2.0,
                OUTER_HEIGHT - SHEET / 2.0,
            )
        ),
        material=zinc_steel,
        name="right_lip",
    )
    outer_section.visual(
        Box((REAR_STOP_LEN, REAR_STOP_WIDTH, REAR_STOP_HEIGHT)),
        origin=Origin(
            xyz=(
                JOINT_REAR_X - REAR_STOP_LEN / 2.0,
                REAR_STOP_Y,
                SHEET + REAR_STOP_HEIGHT / 2.0,
            )
        ),
        material=dark_hardware,
        name="rear_stop",
    )
    outer_section.visual(
        Box((FRONT_STOP_LEN, FRONT_STOP_WIDTH, FRONT_STOP_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_LEN - FRONT_STOP_LEN / 2.0,
                FRONT_STOP_Y,
                SHEET + FRONT_STOP_HEIGHT / 2.0,
            )
        ),
        material=dark_hardware,
        name="front_stop",
    )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((RUNNER_LEN - RUNNER_FRONT_FACE_LEN, RUNNER_WIDTH, RUNNER_BODY_HEIGHT)),
        origin=Origin(
            xyz=((RUNNER_LEN - RUNNER_FRONT_FACE_LEN) / 2.0, 0.0, RUNNER_BODY_HEIGHT / 2.0)
        ),
        material=zinc_steel,
        name="body",
    )
    inner_runner.visual(
        Box((RUNNER_FRONT_FACE_LEN, RUNNER_FRONT_FACE_WIDTH, RUNNER_FRONT_FACE_HEIGHT)),
        origin=Origin(
            xyz=(
                RUNNER_LEN - RUNNER_FRONT_FACE_LEN / 2.0,
                0.0,
                FRONT_FACE_CENTER_Z,
            )
        ),
        material=dark_hardware,
        name="front_face",
    )
    inner_runner.visual(
        Box((INNER_STOP_LUG_LEN, INNER_STOP_LUG_WIDTH, INNER_STOP_LUG_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_STOP_LUG_FRONT_X - INNER_STOP_LUG_LEN / 2.0,
                INNER_STOP_LUG_Y,
                RUNNER_BODY_HEIGHT + INNER_STOP_LUG_HEIGHT / 2.0,
            )
        ),
        material=dark_hardware,
        name="stop_lug",
    )

    model.articulation(
        "outer_to_runner",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=inner_runner,
        origin=Origin(xyz=(JOINT_REAR_X, 0.0, SHEET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=MAX_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_section = object_model.get_part("outer_section")
    inner_runner = object_model.get_part("inner_runner")
    slide = object_model.get_articulation("outer_to_runner")
    rear_stop = outer_section.get_visual("rear_stop")
    front_stop = outer_section.get_visual("front_stop")
    front_face = inner_runner.get_visual("front_face")
    stop_lug = inner_runner.get_visual("stop_lug")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = slide.motion_limits
    joint_ok = (
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == MAX_EXTENSION
    )
    ctx.check(
        "slide uses one prismatic stage on the shared axis",
        joint_ok,
        details=(
            f"type={slide.articulation_type}, axis={slide.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )
    ctx.check(
        "stop hardware visuals are present",
        all(v is not None for v in (rear_stop, front_stop, front_face, stop_lug)),
        details="Expected rear stop, front stop, square front face, and inner stop lug visuals",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            inner_runner,
            outer_section,
            axes="yz",
            margin=0.001,
            name="runner stays captured inside the outer channel when closed",
        )
        ctx.expect_contact(
            inner_runner,
            outer_section,
            elem_b="rear_stop",
            name="runner closes against the rear stop hardware",
        )
        ctx.expect_overlap(
            inner_runner,
            outer_section,
            axes="x",
            min_overlap=0.28,
            name="closed slide keeps substantial runner engagement",
        )

    with ctx.pose({slide: MAX_EXTENSION}):
        ctx.expect_within(
            inner_runner,
            outer_section,
            axes="yz",
            margin=0.001,
            name="runner remains laterally captured at full extension",
        )
        ctx.expect_contact(
            inner_runner,
            outer_section,
            elem_a="stop_lug",
            elem_b="front_stop",
            name="front stop lug limits full extension",
        )
        ctx.expect_overlap(
            inner_runner,
            outer_section,
            axes="x",
            min_overlap=0.12,
            name="full extension still leaves guided overlap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
