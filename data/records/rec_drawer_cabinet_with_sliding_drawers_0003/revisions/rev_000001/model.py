from __future__ import annotations

from math import pi

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

DRAWER_COUNT = 5

CARCASS_WIDTH = 1.08
CARCASS_DEPTH = 0.52
CARCASS_HEIGHT = 1.18
SIDE_THICKNESS = 0.030
TOP_THICKNESS = 0.032
BOTTOM_THICKNESS = 0.032
BACK_THICKNESS = 0.012
TOP_OVERHANG_X = 0.010
TOP_OVERHANG_Y = 0.012

INNER_WIDTH = CARCASS_WIDTH - 2.0 * SIDE_THICKNESS
DRAWER_REVEAL = 0.012
DRAWER_FRONT_HEIGHT = 0.208
DRAWER_FRONT_THICKNESS = 0.024
DRAWER_FRONT_WIDTH = INNER_WIDTH - 2.0 * DRAWER_REVEAL

DRAWER_BOX_WIDTH = INNER_WIDTH - 0.060
DRAWER_BOX_DEPTH = 0.372
DRAWER_BOX_HEIGHT = 0.176
DRAWER_SIDE_THICKNESS = 0.014
DRAWER_BACK_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.012
DRAWER_SIDE_HEIGHT = DRAWER_BOX_HEIGHT - DRAWER_BOTTOM_THICKNESS

DRAWER_TRAVEL = 0.214
DRAWER_FRONT_PROUD = 0.002
DRAWER_JOINT_Y = (
    CARCASS_DEPTH * 0.5
    + DRAWER_FRONT_PROUD
    - DRAWER_BOX_DEPTH * 0.5
    - DRAWER_FRONT_THICKNESS
)

GUIDE_THICKNESS = 0.012
GUIDE_LENGTH = 0.388
GUIDE_HEIGHT = 0.018
GUIDE_Y = -0.034
GUIDE_X = INNER_WIDTH * 0.5 - GUIDE_THICKNESS * 0.5

RUNNER_THICKNESS = 0.018
RUNNER_LENGTH = 0.338
RUNNER_HEIGHT = 0.016
RUNNER_Y = -0.020
RUNNER_Z = -0.028
RUNNER_X = GUIDE_X - GUIDE_THICKNESS * 0.5 - RUNNER_THICKNESS * 0.5

KNOB_RADIUS = 0.018
KNOB_STEM_RADIUS = 0.006
KNOB_STEM_LENGTH = 0.050
KNOB_X_OFFSET = DRAWER_FRONT_WIDTH * 0.24


def _drawer_center_z(index: int) -> float:
    return (
        BOTTOM_THICKNESS
        + DRAWER_REVEAL
        + DRAWER_FRONT_HEIGHT * 0.5
        + index * (DRAWER_FRONT_HEIGHT + DRAWER_REVEAL)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_dresser")

    carcass_wood = model.material("carcass_wood", rgba=(0.45, 0.30, 0.18, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.57, 0.38, 0.22, 1.0))
    knob_wood = model.material("knob_wood", rgba=(0.29, 0.19, 0.11, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.62, 0.64, 0.67, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((SIDE_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                -CARCASS_WIDTH * 0.5 + SIDE_THICKNESS * 0.5,
                0.0,
                CARCASS_HEIGHT * 0.5,
            )
        ),
        material=carcass_wood,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                CARCASS_WIDTH * 0.5 - SIDE_THICKNESS * 0.5,
                0.0,
                CARCASS_HEIGHT * 0.5,
            )
        ),
        material=carcass_wood,
        name="right_side",
    )
    carcass.visual(
        Box((CARCASS_WIDTH + 2.0 * TOP_OVERHANG_X, CARCASS_DEPTH + 2.0 * TOP_OVERHANG_Y, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CARCASS_HEIGHT - TOP_THICKNESS * 0.5)),
        material=carcass_wood,
        name="top_panel",
    )
    carcass.visual(
        Box((CARCASS_WIDTH, CARCASS_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=carcass_wood,
        name="bottom_panel",
    )
    carcass.visual(
        Box(
            (
                CARCASS_WIDTH - 2.0 * SIDE_THICKNESS,
                BACK_THICKNESS,
                CARCASS_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -CARCASS_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                BOTTOM_THICKNESS
                + (CARCASS_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS) * 0.5,
            )
        ),
        material=carcass_wood,
        name="back_panel",
    )
    carcass.visual(
        Box((INNER_WIDTH, 0.020, 0.050)),
        origin=Origin(
            xyz=(0.0, -CARCASS_DEPTH * 0.5 + BACK_THICKNESS + 0.010, 0.090)
        ),
        material=carcass_wood,
        name="rear_stretcher",
    )

    for index in range(DRAWER_COUNT):
        z_pos = _drawer_center_z(index) + RUNNER_Z
        carcass.visual(
            Box((GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT)),
            origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, z_pos)),
            material=rail_metal,
            name=f"left_guide_{index + 1}",
        )
        carcass.visual(
            Box((GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT)),
            origin=Origin(xyz=(GUIDE_X, GUIDE_Y, z_pos)),
            material=rail_metal,
            name=f"right_guide_{index + 1}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((CARCASS_WIDTH, CARCASS_DEPTH, CARCASS_HEIGHT)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, CARCASS_HEIGHT * 0.5)),
    )

    for index in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{index + 1}")
        front_center_y = DRAWER_BOX_DEPTH * 0.5 + DRAWER_FRONT_THICKNESS * 0.5
        knob_stem_center_y = (
            DRAWER_BOX_DEPTH * 0.5 + DRAWER_FRONT_THICKNESS + KNOB_STEM_LENGTH * 0.5
        )
        knob_center_y = (
            DRAWER_BOX_DEPTH * 0.5
            + DRAWER_FRONT_THICKNESS
            + KNOB_STEM_LENGTH
            + KNOB_RADIUS
        )

        drawer.visual(
            Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS)),
            origin=Origin(
                xyz=(0.0, 0.0, -DRAWER_BOX_HEIGHT * 0.5 + DRAWER_BOTTOM_THICKNESS * 0.5)
            ),
            material=drawer_wood,
            name="bottom",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_SIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    -DRAWER_BOX_WIDTH * 0.5 + DRAWER_SIDE_THICKNESS * 0.5,
                    0.0,
                    DRAWER_BOTTOM_THICKNESS * 0.5,
                )
            ),
            material=drawer_wood,
            name="left_side",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_SIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    DRAWER_BOX_WIDTH * 0.5 - DRAWER_SIDE_THICKNESS * 0.5,
                    0.0,
                    DRAWER_BOTTOM_THICKNESS * 0.5,
                )
            ),
            material=drawer_wood,
            name="right_side",
        )
        drawer.visual(
            Box((DRAWER_BOX_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_SIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    -DRAWER_BOX_DEPTH * 0.5 + DRAWER_BACK_THICKNESS * 0.5,
                    DRAWER_BOTTOM_THICKNESS * 0.5,
                )
            ),
            material=drawer_wood,
            name="back",
        )
        drawer.visual(
            Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
            origin=Origin(xyz=(0.0, front_center_y, 0.0)),
            material=drawer_wood,
            name="front",
        )
        drawer.visual(
            Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_LENGTH),
            origin=Origin(
                xyz=(-KNOB_X_OFFSET, knob_stem_center_y, 0.0),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=knob_wood,
            name="left_knob_stem",
        )
        drawer.visual(
            Sphere(radius=KNOB_RADIUS),
            origin=Origin(xyz=(-KNOB_X_OFFSET, knob_center_y, 0.0)),
            material=knob_wood,
            name="left_knob",
        )
        drawer.visual(
            Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_LENGTH),
            origin=Origin(
                xyz=(KNOB_X_OFFSET, knob_stem_center_y, 0.0),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=knob_wood,
            name="right_knob_stem",
        )
        drawer.visual(
            Sphere(radius=KNOB_RADIUS),
            origin=Origin(xyz=(KNOB_X_OFFSET, knob_center_y, 0.0)),
            material=knob_wood,
            name="right_knob",
        )
        drawer.visual(
            Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(-RUNNER_X, RUNNER_Y, RUNNER_Z)),
            material=rail_metal,
            name="left_runner",
        )
        drawer.visual(
            Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(RUNNER_X, RUNNER_Y, RUNNER_Z)),
            material=rail_metal,
            name="right_runner",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((DRAWER_FRONT_WIDTH, DRAWER_BOX_DEPTH + 0.06, DRAWER_FRONT_HEIGHT)),
            mass=4.0,
            origin=Origin(xyz=(0.0, 0.03, 0.0)),
        )
        model.articulation(
            f"carcass_to_drawer_{index + 1}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, DRAWER_JOINT_Y, _drawer_center_z(index))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.40,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    carcass = object_model.get_part("carcass")
    drawers = [object_model.get_part(f"drawer_{index + 1}") for index in range(DRAWER_COUNT)]
    slides = [
        object_model.get_articulation(f"carcass_to_drawer_{index + 1}")
        for index in range(DRAWER_COUNT)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    carcass_aabb = ctx.part_world_aabb(carcass)
    if carcass_aabb is not None:
        carcass_min, carcass_max = carcass_aabb
        carcass_width = carcass_max[0] - carcass_min[0]
        carcass_depth = carcass_max[1] - carcass_min[1]
        carcass_height = carcass_max[2] - carcass_min[2]
        ctx.check(
            "carcass_realistic_width",
            0.95 <= carcass_width <= 1.10,
            f"expected dresser width near 1.0 m, got {carcass_width:.3f} m",
        )
        ctx.check(
            "carcass_realistic_depth",
            0.45 <= carcass_depth <= 0.60,
            f"expected dresser depth near 0.5 m, got {carcass_depth:.3f} m",
        )
        ctx.check(
            "carcass_realistic_height",
            1.05 <= carcass_height <= 1.30,
            f"expected dresser height near 1.2 m, got {carcass_height:.3f} m",
        )
        ctx.check(
            "carcass_grounded",
            abs(carcass_min[2]) <= 1e-6,
            f"carcass should sit on the floor plane, min z was {carcass_min[2]:.6f}",
        )

    for index, drawer in enumerate(drawers):
        number = index + 1
        slide = slides[index]
        limits = slide.motion_limits
        front = drawer.get_visual("front")
        left_knob = drawer.get_visual("left_knob")
        right_knob = drawer.get_visual("right_knob")
        left_knob_stem = drawer.get_visual("left_knob_stem")
        right_knob_stem = drawer.get_visual("right_knob_stem")
        left_runner = drawer.get_visual("left_runner")
        right_runner = drawer.get_visual("right_runner")
        left_guide = carcass.get_visual(f"left_guide_{number}")
        right_guide = carcass.get_visual(f"right_guide_{number}")

        ctx.check(
            f"drawer_{number}_uses_prismatic_slide",
            slide.articulation_type == ArticulationType.PRISMATIC,
            f"expected prismatic articulation, got {slide.articulation_type}",
        )
        ctx.check(
            f"drawer_{number}_slide_axis_forward",
            tuple(slide.axis) == (0.0, 1.0, 0.0),
            f"expected slide axis (0, 1, 0), got {slide.axis}",
        )
        ctx.check(
            f"drawer_{number}_travel_limit_realistic",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.18 <= limits.upper <= 0.25,
            f"expected 0.18 m to 0.25 m drawer travel, got {limits}",
        )
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=left_knob_stem,
            elem_b=front,
            name=f"drawer_{number}_left_knob_stem_mounted_to_front",
        )
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=left_knob,
            elem_b=left_knob_stem,
            name=f"drawer_{number}_left_knob_attached_to_stem",
        )
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=right_knob_stem,
            elem_b=front,
            name=f"drawer_{number}_right_knob_stem_mounted_to_front",
        )
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=right_knob,
            elem_b=right_knob_stem,
            name=f"drawer_{number}_right_knob_attached_to_stem",
        )
        with ctx.pose({slide: 0.0}):
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a=left_runner,
                elem_b=left_guide,
                name=f"drawer_{number}_left_runner_contacts_guide_closed",
            )
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a=right_runner,
                elem_b=right_guide,
                name=f"drawer_{number}_right_runner_contacts_guide_closed",
            )
            ctx.expect_within(
                drawer,
                carcass,
                axes=("x", "z"),
                inner_elem=front,
                name=f"drawer_{number}_front_centered_in_opening_closed",
            )
            ctx.expect_gap(
                drawer,
                carcass,
                axis="y",
                min_gap=0.030,
                positive_elem=left_knob,
                name=f"drawer_{number}_left_knob_proud_of_case_closed",
            )
            ctx.expect_gap(
                drawer,
                carcass,
                axis="y",
                min_gap=0.030,
                positive_elem=right_knob,
                name=f"drawer_{number}_right_knob_proud_of_case_closed",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"drawer_{number}_closed_pose_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"drawer_{number}_closed_pose_no_floating")
        if limits is not None and limits.upper is not None:
            with ctx.pose({slide: limits.upper}):
                ctx.expect_contact(
                    drawer,
                    carcass,
                    elem_a=left_runner,
                    elem_b=left_guide,
                    name=f"drawer_{number}_left_runner_contacts_guide_open",
                )
                ctx.expect_contact(
                    drawer,
                    carcass,
                    elem_a=right_runner,
                    elem_b=right_guide,
                    name=f"drawer_{number}_right_runner_contacts_guide_open",
                )
                ctx.expect_gap(
                    drawer,
                    carcass,
                    axis="y",
                    min_gap=0.18,
                    positive_elem=front,
                    name=f"drawer_{number}_opens_forward",
                )
                ctx.expect_within(
                    drawer,
                    carcass,
                    axes="x",
                    inner_elem=front,
                    name=f"drawer_{number}_front_stays_centered_when_open",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"drawer_{number}_open_pose_no_overlap"
                )
                ctx.fail_if_isolated_parts(
                    name=f"drawer_{number}_open_pose_no_floating"
                )

    for slide in slides:
        limits = slide.motion_limits
        if limits is not None and limits.lower is not None:
            with ctx.pose({slide: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{slide.name}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{slide.name}_lower_no_floating")
        if limits is not None and limits.upper is not None:
            with ctx.pose({slide: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{slide.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{slide.name}_upper_no_floating")

    for index in range(DRAWER_COUNT - 1):
        lower_front = drawers[index].get_visual("front")
        upper_front = drawers[index + 1].get_visual("front")
        ctx.expect_gap(
            drawers[index + 1],
            drawers[index],
            axis="z",
            min_gap=0.011,
            max_gap=0.013,
            positive_elem=upper_front,
            negative_elem=lower_front,
            name=f"drawer_reveal_{index + 1}_to_{index + 2}",
        )

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END
