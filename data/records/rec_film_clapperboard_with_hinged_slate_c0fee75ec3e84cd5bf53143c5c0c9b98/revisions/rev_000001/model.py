from __future__ import annotations

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


BOARD_WIDTH = 0.300
BOARD_DEPTH = 0.012
BOARD_HEIGHT = 0.215

PANEL_WIDTH = 0.256
PANEL_DEPTH = 0.002
PANEL_HEIGHT = 0.160
PANEL_CENTER_Z = 0.096

HOUSING_WIDTH = 0.236
HOUSING_DEPTH = 0.030
HOUSING_HEIGHT = 0.046
HOUSING_CENTER_Y = 0.006
HOUSING_CENTER_Z = BOARD_HEIGHT + HOUSING_HEIGHT / 2.0

DISPLAY_WIDTH = 0.196
DISPLAY_BEZEL_DEPTH = 0.004
DISPLAY_BEZEL_HEIGHT = 0.028
DISPLAY_WINDOW_DEPTH = 0.002
DISPLAY_WINDOW_HEIGHT = 0.017

HINGE_Z = BOARD_HEIGHT + HOUSING_HEIGHT + 0.006
HINGE_Y = 0.019

CLAPSTICK_LENGTH = 0.334
CLAPSTICK_DEPTH = 0.018
CLAPSTICK_HEIGHT = 0.040
CLAPSTICK_ROLL = -0.35
CLAPSTICK_CENTER_Y = 0.015303459239261778
CLAPSTICK_CENTER_Z = 0.015702549811138885

ROCKER_PIVOT_X = HOUSING_WIDTH / 2.0
ROCKER_PIVOT_Y = HOUSING_CENTER_Y
ROCKER_PIVOT_Z = HOUSING_CENTER_Z


def add_clap_segment(part, *, x_center: float, length: float, material: str, name: str) -> None:
    part.visual(
        Box((length, CLAPSTICK_DEPTH, CLAPSTICK_HEIGHT)),
        origin=Origin(
            xyz=(x_center, CLAPSTICK_CENTER_Y, CLAPSTICK_CENTER_Z),
            rpy=(CLAPSTICK_ROLL, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_timecode_slate")

    model.material("frame_black", color=(0.10, 0.10, 0.11))
    model.material("frame_dark", color=(0.18, 0.19, 0.21))
    model.material("erase_white", color=(0.96, 0.97, 0.98))
    model.material("display_bezel", color=(0.05, 0.05, 0.06))
    model.material("display_red", color=(0.64, 0.09, 0.08))
    model.material("clap_white", color=(0.95, 0.95, 0.93))
    model.material("clap_red", color=(0.72, 0.17, 0.14))
    model.material("switch_charcoal", color=(0.15, 0.15, 0.15))

    slate_body = model.part("slate_body")
    slate_body.visual(
        Box((BOARD_WIDTH, BOARD_DEPTH, BOARD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_HEIGHT / 2.0)),
        material="frame_black",
        name="board_shell",
    )
    slate_body.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, BOARD_DEPTH / 2.0 + PANEL_DEPTH / 2.0, PANEL_CENTER_Z),
        ),
        material="erase_white",
        name="erase_panel",
    )
    slate_body.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, HOUSING_CENTER_Z)),
        material="frame_dark",
        name="housing_shell",
    )
    slate_body.visual(
        Box((DISPLAY_WIDTH, DISPLAY_BEZEL_DEPTH, DISPLAY_BEZEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HOUSING_CENTER_Y + HOUSING_DEPTH / 2.0 + DISPLAY_BEZEL_DEPTH / 2.0,
                HOUSING_CENTER_Z,
            ),
        ),
        material="display_bezel",
        name="display_bezel",
    )
    slate_body.visual(
        Box((DISPLAY_WIDTH * 0.84, DISPLAY_WINDOW_DEPTH, DISPLAY_WINDOW_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HOUSING_CENTER_Y + HOUSING_DEPTH / 2.0 + DISPLAY_BEZEL_DEPTH + DISPLAY_WINDOW_DEPTH / 2.0,
                HOUSING_CENTER_Z,
            ),
        ),
        material="display_red",
        name="display_window",
    )
    slate_body.visual(
        Box((0.250, 0.011, 0.012)),
        origin=Origin(xyz=(0.0, 0.009, HINGE_Z)),
        material="frame_dark",
        name="hinge_mount",
    )
    clapstick = model.part("clapstick")
    clapstick.visual(
        Cylinder(radius=0.0045, length=0.268),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="frame_black",
        name="hinge_barrel",
    )
    segment_length = CLAPSTICK_LENGTH / 6.0
    segment_centers = (
        -2.5 * segment_length,
        -1.5 * segment_length,
        -0.5 * segment_length,
        0.5 * segment_length,
        1.5 * segment_length,
        2.5 * segment_length,
    )
    segment_materials = (
        "clap_red",
        "clap_white",
        "frame_black",
        "clap_white",
        "frame_black",
        "clap_red",
    )
    for index, (center_x, material) in enumerate(zip(segment_centers, segment_materials)):
        add_clap_segment(
            clapstick,
            x_center=center_x,
            length=segment_length,
            material=material,
            name=f"clap_segment_{index}",
        )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.0018, length=0.016),
        origin=Origin(xyz=(0.0018, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="switch_charcoal",
        name="rocker_pivot",
    )
    power_rocker.visual(
        Box((0.004, 0.016, 0.018)),
        origin=Origin(xyz=(0.0038, 0.0, 0.0)),
        material="switch_charcoal",
        name="rocker_body",
    )
    power_rocker.visual(
        Box((0.006, 0.014, 0.024)),
        origin=Origin(xyz=(0.0066, 0.0, 0.0)),
        material="switch_charcoal",
        name="rocker_cap",
    )

    model.articulation(
        "body_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=slate_body,
        child=clapstick,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=1.2),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=slate_body,
        child=power_rocker,
        origin=Origin(xyz=(ROCKER_PIVOT_X, ROCKER_PIVOT_Y, ROCKER_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slate_body = object_model.get_part("slate_body")
    clapstick = object_model.get_part("clapstick")
    power_rocker = object_model.get_part("power_rocker")
    clap_joint = object_model.get_articulation("body_to_clapstick")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")

    ctx.expect_overlap(
        clapstick,
        slate_body,
        axes="x",
        min_overlap=0.24,
        name="clapstick spans the slate width",
    )
    ctx.expect_gap(
        power_rocker,
        slate_body,
        axis="x",
        positive_elem="rocker_cap",
        negative_elem="housing_shell",
        min_gap=0.0005,
        max_gap=0.008,
        name="power rocker sits slightly proud of the housing side",
    )
    ctx.expect_overlap(
        power_rocker,
        slate_body,
        axes="yz",
        elem_a="rocker_cap",
        elem_b="housing_shell",
        min_overlap=0.010,
        name="power rocker stays aligned with the housing side",
    )

    clap_limits = clap_joint.motion_limits
    rocker_limits = rocker_joint.motion_limits

    closed_clap_aabb = ctx.part_world_aabb(clapstick)
    rest_rocker_aabb = ctx.part_world_aabb(power_rocker)

    opened_clap_aabb = None
    tipped_rocker_aabb = None

    if clap_limits is not None and clap_limits.upper is not None:
        with ctx.pose({clap_joint: clap_limits.upper}):
            opened_clap_aabb = ctx.part_world_aabb(clapstick)

    if rocker_limits is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            tipped_rocker_aabb = ctx.part_world_aabb(power_rocker)

    ctx.check(
        "clapstick opens upward",
        closed_clap_aabb is not None
        and opened_clap_aabb is not None
        and opened_clap_aabb[0][1] < closed_clap_aabb[0][1] - 0.020
        and opened_clap_aabb[1][2] > closed_clap_aabb[1][2] + 0.002,
        details=f"closed={closed_clap_aabb}, opened={opened_clap_aabb}",
    )
    ctx.check(
        "power rocker tilts outward",
        rest_rocker_aabb is not None
        and tipped_rocker_aabb is not None
        and tipped_rocker_aabb[1][0] > rest_rocker_aabb[1][0] + 0.001,
        details=f"rest={rest_rocker_aabb}, tipped={tipped_rocker_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
