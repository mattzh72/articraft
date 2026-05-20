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


PLATE_THICKNESS = 0.0070
PLATE_HALF = PLATE_THICKNESS * 0.5

AXLE_SHAFT_RADIUS = 0.0042
AXLE_SHAFT_LENGTH = 0.0096
BUTTON_RADIUS = 0.0115
BUTTON_THICKNESS = 0.0022

HUB_SIDE_CENTER = 0.0115
HUB_SIDE_LENGTH = 0.0200
HUB_SIDE_WIDTH = 0.0100
HUB_CORNER_RADIUS = 0.0064
HUB_CORNER_CENTER = 0.0130

ARM_BAR_WIDTH = 0.0100
ARM_BAR_LENGTH = 0.0180
ARM_BAR_CENTER = 0.0190
ARM_NECK_RADIUS = 0.0050
ARM_NECK_CENTER = 0.0270

LOBE_RADIUS = 0.0140
LOBE_CENTER = 0.0300
WEIGHT_RADIUS = 0.0095
WEIGHT_THICKNESS = PLATE_THICKNESS * 0.86

BRANCH_ANGLES = (
    0.0,
    2.0 * math.pi / 3.0,
    4.0 * math.pi / 3.0,
)


def _xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]]
) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_lobe_spinner")

    body_black = model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    button_silver = model.material("button_silver", rgba=(0.84, 0.86, 0.89, 1.0))

    axle = model.part("axle")
    axle.visual(
        Cylinder(radius=AXLE_SHAFT_RADIUS, length=AXLE_SHAFT_LENGTH),
        material=bearing_steel,
        name="shaft",
    )
    axle.visual(
        Cylinder(radius=BUTTON_RADIUS, length=BUTTON_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, PLATE_HALF + BUTTON_THICKNESS * 0.5)),
        material=button_silver,
        name="top_button",
    )
    axle.visual(
        Cylinder(radius=BUTTON_RADIUS, length=BUTTON_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -PLATE_HALF - BUTTON_THICKNESS * 0.5)),
        material=button_silver,
        name="bottom_button",
    )
    axle.inertial = Inertial.from_geometry(
        Box((BUTTON_RADIUS * 2.2, BUTTON_RADIUS * 2.2, AXLE_SHAFT_LENGTH)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    spinner = model.part("spinner")
    spinner.inertial = Inertial.from_geometry(
        Cylinder(radius=LOBE_CENTER + LOBE_RADIUS, length=PLATE_THICKNESS),
        mass=0.036,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    for index, angle in enumerate(BRANCH_ANGLES, start=1):
        spinner.visual(
            Box((HUB_SIDE_LENGTH, HUB_SIDE_WIDTH, PLATE_THICKNESS)),
            origin=Origin(
                xyz=(*_xy(HUB_SIDE_CENTER, angle), 0.0),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=body_black,
            name=f"hub_side_{index}",
        )
    for index, angle in enumerate((math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0), start=1):
        spinner.visual(
            Cylinder(radius=HUB_CORNER_RADIUS, length=PLATE_THICKNESS),
            origin=Origin(xyz=(*_xy(HUB_CORNER_CENTER, angle), 0.0)),
            material=matte_graphite,
            name=f"hub_corner_{index}",
        )

    for index, angle in enumerate(BRANCH_ANGLES, start=1):
        spinner.visual(
            Box((ARM_BAR_LENGTH, ARM_BAR_WIDTH, PLATE_THICKNESS)),
            origin=Origin(
                xyz=(*_xy(ARM_BAR_CENTER, angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=body_black,
            name=f"arm_{index}_bar",
        )
        spinner.visual(
            Cylinder(radius=ARM_NECK_RADIUS, length=PLATE_THICKNESS),
            origin=Origin(xyz=(*_xy(ARM_NECK_CENTER, angle), 0.0)),
            material=body_black,
            name=f"arm_{index}_neck",
        )
        spinner.visual(
            Cylinder(radius=LOBE_RADIUS, length=PLATE_THICKNESS),
            origin=Origin(xyz=(*_xy(LOBE_CENTER, angle), 0.0)),
            material=matte_graphite,
            name=f"lobe_{index}_shell",
        )
        spinner.visual(
            Cylinder(radius=WEIGHT_RADIUS, length=WEIGHT_THICKNESS),
            origin=Origin(xyz=(*_xy(LOBE_CENTER, angle), 0.0)),
            material=bearing_steel,
            name=f"lobe_{index}_weight",
        )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=spinner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_part("axle")
    spinner = object_model.get_part("spinner")
    axle_spin = object_model.get_articulation("axle_spin")

    top_button = axle.get_visual("top_button")
    bottom_button = axle.get_visual("bottom_button")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_distance(axle, spinner, axes="xy", max_dist=1e-6)
    ctx.expect_contact(axle, spinner, elem_a=top_button)
    ctx.expect_contact(axle, spinner, elem_a=bottom_button)
    ctx.expect_gap(
        axle,
        spinner,
        axis="z",
        max_gap=0.0002,
        max_penetration=1e-6,
        positive_elem=top_button,
    )
    ctx.expect_gap(
        spinner,
        axle,
        axis="z",
        max_gap=0.0002,
        max_penetration=1e-6,
        negative_elem=bottom_button,
    )
    ctx.expect_overlap(axle, spinner, axes="xy", min_overlap=0.018)

    for index, expected_angle in enumerate(BRANCH_ANGLES, start=1):
        lobe_aabb = ctx.part_element_world_aabb(spinner, elem=f"lobe_{index}_shell")
        if lobe_aabb is None:
            ctx.fail(f"lobe_{index}_aabb_available", "Missing world AABB for spinner lobe.")
            continue
        lobe_center = _aabb_center(lobe_aabb)
        expected_x, expected_y = _xy(LOBE_CENTER, expected_angle)
        ctx.check(
            f"lobe_{index}_placement",
            abs(lobe_center[0] - expected_x) <= 0.001
            and abs(lobe_center[1] - expected_y) <= 0.001
            and abs(lobe_center[2]) <= 1e-6,
            (
                f"Expected lobe {index} near ({expected_x:.4f}, {expected_y:.4f}, 0.0000) "
                f"but found ({lobe_center[0]:.4f}, {lobe_center[1]:.4f}, {lobe_center[2]:.4f})."
            ),
        )

    with ctx.pose({axle_spin: math.pi * 0.5}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spinner_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="spinner_quarter_turn_no_floating")
        ctx.expect_contact(axle, spinner, elem_a=top_button, name="top_button_contact_quarter_turn")
        ctx.expect_contact(axle, spinner, elem_a=bottom_button, name="bottom_button_contact_quarter_turn")
        ctx.expect_overlap(axle, spinner, axes="xy", min_overlap=0.018, name="axle_overlap_quarter_turn")

        lobe_1_aabb = ctx.part_element_world_aabb(spinner, elem="lobe_1_shell")
        if lobe_1_aabb is None:
            ctx.fail("spinner_joint_rotates_lobe_1_about_z", "Missing lobe 1 world AABB at quarter turn.")
        else:
            lobe_1_center = _aabb_center(lobe_1_aabb)
            ctx.check(
                "spinner_joint_rotates_lobe_1_about_z",
                abs(lobe_1_center[0]) <= 0.001 and abs(lobe_1_center[1] - LOBE_CENTER) <= 0.001,
                (
                    f"Expected quarter-turn lobe 1 center near (0.0000, {LOBE_CENTER:.4f}, 0.0000) "
                    f"but found ({lobe_1_center[0]:.4f}, {lobe_1_center[1]:.4f}, {lobe_1_center[2]:.4f})."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
