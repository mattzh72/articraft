from __future__ import annotations

import math

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


def _rotated_y(x: float, y: float, z: float, angle: float) -> tuple[float, float, float]:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        cos_a * x + sin_a * z,
        y,
        -sin_a * x + cos_a * z,
    )


def _point_on_slope(
    center: tuple[float, float, float],
    x: float,
    y: float,
    z: float,
    angle: float,
) -> tuple[float, float, float]:
    rx, ry, rz = _rotated_y(x, y, z, angle)
    return (center[0] + rx, center[1] + ry, center[2] + rz)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo_i + hi_i) * 0.5 for lo_i, hi_i in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_register")

    model.material("housing_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("drawer_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("console_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    model.material("accent_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("key_light", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("display_black", rgba=(0.05, 0.06, 0.07, 1.0))
    model.material("screen_glow", rgba=(0.16, 0.26, 0.34, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.34, 0.32, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="housing_gray",
        name="bottom_panel",
    )
    housing.visual(
        Box((0.34, 0.012, 0.11)),
        origin=Origin(xyz=(0.0, 0.154, 0.065)),
        material="housing_gray",
        name="side_wall_0",
    )
    housing.visual(
        Box((0.34, 0.012, 0.11)),
        origin=Origin(xyz=(0.0, -0.154, 0.065)),
        material="housing_gray",
        name="side_wall_1",
    )
    housing.visual(
        Box((0.012, 0.296, 0.11)),
        origin=Origin(xyz=(-0.164, 0.0, 0.065)),
        material="housing_gray",
        name="back_wall",
    )
    housing.visual(
        Box((0.31, 0.296, 0.012)),
        origin=Origin(xyz=(-0.009, 0.0, 0.114)),
        material="housing_gray",
        name="top_deck",
    )
    housing.visual(
        Box((0.024, 0.296, 0.022)),
        origin=Origin(xyz=(0.158, 0.0, 0.109)),
        material="housing_gray",
        name="opening_header",
    )
    housing.visual(
        Box((0.24, 0.016, 0.008)),
        origin=Origin(xyz=(0.003, 0.140, 0.022)),
        material="accent_dark",
        name="runner_0",
    )
    housing.visual(
        Box((0.24, 0.016, 0.008)),
        origin=Origin(xyz=(0.003, -0.140, 0.022)),
        material="accent_dark",
        name="runner_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.246, 0.248, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="drawer_gray",
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.246, 0.006, 0.05)),
        origin=Origin(xyz=(0.0, 0.127, 0.031)),
        material="drawer_gray",
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.246, 0.006, 0.05)),
        origin=Origin(xyz=(0.0, -0.127, 0.031)),
        material="drawer_gray",
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.006, 0.242, 0.05)),
        origin=Origin(xyz=(-0.120, 0.0, 0.031)),
        material="drawer_gray",
        name="tray_back",
    )
    drawer.visual(
        Box((0.014, 0.286, 0.072)),
        origin=Origin(xyz=(0.130, 0.0, 0.036)),
        material="drawer_gray",
        name="drawer_front",
    )
    drawer.visual(
        Box((0.008, 0.14, 0.012)),
        origin=Origin(xyz=(0.141, 0.0, 0.038)),
        material="accent_dark",
        name="handle_strip",
    )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0.032, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=0.0, upper=0.11),
    )

    console = model.part("console")
    console.visual(
        Box((0.06, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="console_gray",
        name="support_pedestal",
    )
    console.visual(
        Box((0.10, 0.12, 0.02)),
        origin=Origin(xyz=(0.01, 0.0, 0.040)),
        material="console_gray",
        name="support_bridge",
    )
    console.visual(
        Box((0.19, 0.20, 0.032)),
        origin=Origin(xyz=(0.030, 0.0, 0.055)),
        material="console_gray",
        name="pod_body",
    )
    console.visual(
        Box((0.095, 0.19, 0.04)),
        origin=Origin(xyz=(-0.055, 0.0, 0.082)),
        material="console_gray",
        name="rear_housing",
    )

    deck_angle = 0.32
    deck_center = (0.052, 0.0, 0.074)
    console.visual(
        Box((0.18, 0.19, 0.02)),
        origin=Origin(xyz=deck_center, rpy=(0.0, deck_angle, 0.0)),
        material="accent_dark",
        name="keypad_deck",
    )
    console.visual(
        Box((0.04, 0.08, 0.04)),
        origin=Origin(xyz=(-0.118, 0.0, 0.102)),
        material="console_gray",
        name="screen_plinth",
    )

    key_x = (0.052, 0.022, -0.008)
    key_y = (-0.052, -0.017, 0.018, 0.053)
    for row, local_x in enumerate(key_x):
        for col, local_y in enumerate(key_y):
            console.visual(
                Box((0.024, 0.024, 0.008)),
                origin=Origin(
                    xyz=_point_on_slope(deck_center, local_x, local_y, 0.013, deck_angle),
                    rpy=(0.0, deck_angle, 0.0),
                ),
                material="key_light",
                name=f"key_{row}_{col}",
            )
    console.visual(
        Box((0.038, 0.03, 0.01)),
        origin=Origin(
            xyz=_point_on_slope(deck_center, -0.040, 0.060, 0.014, deck_angle),
            rpy=(0.0, deck_angle, 0.0),
        ),
        material="key_light",
        name="enter_key",
    )

    model.articulation(
        "housing_to_console",
        ArticulationType.FIXED,
        parent=housing,
        child=console,
        origin=Origin(xyz=(-0.050, 0.0, 0.120)),
    )

    receipt_flap = model.part("receipt_flap")
    receipt_flap.visual(
        Box((0.060, 0.148, 0.008)),
        origin=Origin(xyz=(0.030, 0.0, 0.004)),
        material="console_gray",
        name="cover_panel",
    )
    receipt_flap.visual(
        Box((0.012, 0.148, 0.014)),
        origin=Origin(xyz=(0.054, 0.0, 0.011)),
        material="accent_dark",
        name="front_lip",
    )
    model.articulation(
        "console_to_receipt_flap",
        ArticulationType.REVOLUTE,
        parent=console,
        child=receipt_flap,
        origin=Origin(xyz=(-0.099, 0.0, 0.102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    screen_stem = model.part("screen_stem")
    screen_stem.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="accent_dark",
        name="swivel_puck",
    )
    screen_stem.visual(
        Cylinder(radius=0.011, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material="accent_dark",
        name="stem_tube",
    )
    screen_stem.visual(
        Box((0.07, 0.028, 0.018)),
        origin=Origin(xyz=(-0.035, 0.0, 0.123)),
        material="accent_dark",
        name="head_arm",
    )
    model.articulation(
        "console_to_screen_stem",
        ArticulationType.REVOLUTE,
        parent=console,
        child=screen_stem,
        origin=Origin(xyz=(-0.118, 0.0, 0.122)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.05, upper=1.05),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.024, 0.18, 0.13)),
        origin=Origin(xyz=(-0.012, 0.0, 0.065)),
        material="display_black",
        name="screen_casing",
    )
    screen.visual(
        Box((0.002, 0.15, 0.102)),
        origin=Origin(xyz=(-0.023, 0.0, 0.071)),
        material="screen_glow",
        name="screen_glass",
    )
    screen.visual(
        Box((0.014, 0.06, 0.02)),
        origin=Origin(xyz=(-0.005, 0.0, 0.014)),
        material="accent_dark",
        name="screen_chin",
    )
    model.articulation(
        "screen_stem_to_screen",
        ArticulationType.REVOLUTE,
        parent=screen_stem,
        child=screen,
        origin=Origin(xyz=(-0.070, 0.0, 0.123)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-0.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    console = object_model.get_part("console")
    receipt_flap = object_model.get_part("receipt_flap")
    screen = object_model.get_part("screen")

    drawer_joint = object_model.get_articulation("housing_to_drawer")
    flap_joint = object_model.get_articulation("console_to_receipt_flap")
    swivel_joint = object_model.get_articulation("console_to_screen_stem")
    tilt_joint = object_model.get_articulation("screen_stem_to_screen")

    ctx.expect_gap(
        console,
        housing,
        axis="z",
        positive_elem="pod_body",
        negative_elem="top_deck",
        min_gap=0.03,
        max_gap=0.05,
        name="control pod stays visibly above the drawer housing",
    )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_within(
            drawer,
            housing,
            axes="yz",
            margin=0.01,
            name="drawer stays centered between the housing runners",
        )
        closed_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            housing,
            axes="x",
            min_overlap=0.12,
            name="drawer remains retained in the housing at full extension",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > closed_drawer_pos[0] + 0.09,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_gap(
            receipt_flap,
            console,
            axis="z",
            positive_elem="cover_panel",
            negative_elem="rear_housing",
            max_gap=0.001,
            max_penetration=1e-6,
            name="receipt flap seats on the printer housing when closed",
        )
        flap_closed = _aabb_center(ctx.part_element_world_aabb(receipt_flap, elem="cover_panel"))

    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        flap_open = _aabb_center(ctx.part_element_world_aabb(receipt_flap, elem="cover_panel"))

    ctx.check(
        "receipt flap opens upward",
        flap_closed is not None
        and flap_open is not None
        and flap_open[2] > flap_closed[2] + 0.02,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    with ctx.pose({swivel_joint: 0.0, tilt_joint: 0.0}):
        screen_neutral = _aabb_center(ctx.part_element_world_aabb(screen, elem="screen_glass"))

    with ctx.pose({swivel_joint: swivel_joint.motion_limits.upper, tilt_joint: 0.0}):
        screen_swiveled = _aabb_center(ctx.part_element_world_aabb(screen, elem="screen_glass"))

    ctx.check(
        "screen swivels around the vertical post",
        screen_neutral is not None
        and screen_swiveled is not None
        and abs(screen_swiveled[1] - screen_neutral[1]) > 0.045,
        details=f"neutral={screen_neutral}, swiveled={screen_swiveled}",
    )

    with ctx.pose({swivel_joint: 0.0, tilt_joint: tilt_joint.motion_limits.upper}):
        screen_tilted = _aabb_center(ctx.part_element_world_aabb(screen, elem="screen_glass"))

    ctx.check(
        "screen tilts backward at the head hinge",
        screen_neutral is not None
        and screen_tilted is not None
        and screen_tilted[0] < screen_neutral[0] - 0.02,
        details=f"neutral={screen_neutral}, tilted={screen_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
