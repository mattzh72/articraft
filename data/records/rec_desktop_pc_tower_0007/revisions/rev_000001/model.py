from __future__ import annotations

import math
import os
from pathlib import Path

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/tmp")
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd

ASSETS = AssetContext(Path("/tmp"))

CASE_W = 0.19
CASE_D = 0.41
CASE_H = 0.43
SHELL_T = 0.012
FRONT_T = 0.014
REAR_T = 0.014
PANEL_T = 0.006

BAY_W = 0.148
BAY_H = 0.044
BAY_Z0 = 0.316
BAY_Z1 = BAY_Z0 + BAY_H

PANEL_Z0 = 0.020
PANEL_H = CASE_H - 2.0 * PANEL_Z0
PANEL_Y0 = -CASE_D / 2.0 + REAR_T
PANEL_D = CASE_D - REAR_T - FRONT_T

TRAY_W = BAY_W - 0.006
TRAY_H = 0.010
TRAY_LEN = 0.205
TRAY_FACE_T = 0.004
TRAY_FRONT_INSET = FRONT_T + 0.021
TRAY_CENTER_Y = CASE_D / 2.0 - TRAY_FRONT_INSET - TRAY_LEN / 2.0

BAY_FLOOR_T = 0.008
BAY_FLOOR_Z = BAY_Z0 + BAY_FLOOR_T / 2.0
TRAY_CENTER_Z = BAY_Z0 + BAY_FLOOR_T + TRAY_H / 2.0

BEZEL_W = BAY_W - 0.006
BEZEL_H = BAY_H - 0.006
BEZEL_T = 0.004
BEZEL_HINGE_Z = BAY_Z0 - 0.006

SIDE_HINGE_POST_R = 0.003
SIDE_HINGE_BARREL_R = 0.0045
BEZEL_HINGE_PIN_R = 0.0024
BEZEL_HINGE_BARREL_R = 0.0035
SIDE_HINGE_AXIS_X = CASE_W / 2.0 + SIDE_HINGE_BARREL_R + 0.001
SIDE_HINGE_AXIS_Y = -CASE_D / 2.0 - 0.010
SIDE_PANEL_CENTER_Y = -SIDE_HINGE_AXIS_Y


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_desktop_tower", assets=ASSETS)

    case_beige = model.material("case_beige", rgba=(0.82, 0.79, 0.69, 1.0))
    case_shadow = model.material("case_shadow", rgba=(0.71, 0.68, 0.59, 1.0))
    plastic_beige = model.material("plastic_beige", rgba=(0.85, 0.81, 0.71, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.18, 0.18, 0.18, 1.0))
    led_green = model.material("led_green", rgba=(0.25, 0.75, 0.35, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((CASE_W, CASE_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_T / 2.0)),
        material=case_beige,
        name="bottom_shell",
    )
    chassis.visual(
        Box((CASE_W, CASE_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, CASE_H - SHELL_T / 2.0)),
        material=case_beige,
        name="top_shell",
    )
    chassis.visual(
        Box((CASE_W, REAR_T, CASE_H - 2.0 * SHELL_T)),
        origin=Origin(
            xyz=(0.0, -CASE_D / 2.0 + REAR_T / 2.0, CASE_H / 2.0),
        ),
        material=case_shadow,
        name="rear_wall",
    )
    chassis.visual(
        Box((SHELL_T, CASE_D - FRONT_T - REAR_T, CASE_H - 2.0 * SHELL_T)),
        origin=Origin(
            xyz=(-CASE_W / 2.0 + SHELL_T / 2.0, 0.0, CASE_H / 2.0),
        ),
        material=case_shadow,
        name="left_wall",
    )
    chassis.visual(
        Cylinder(radius=SIDE_HINGE_POST_R, length=CASE_H - 2.0 * SHELL_T),
        origin=Origin(
            xyz=(SIDE_HINGE_AXIS_X, SIDE_HINGE_AXIS_Y, CASE_H / 2.0),
        ),
        material=case_shadow,
        name="side_panel_hinge_post",
    )

    front_column_w = (CASE_W - BAY_W) / 2.0
    chassis.visual(
        Box((front_column_w, FRONT_T, CASE_H)),
        origin=Origin(
            xyz=(
                -CASE_W / 2.0 + front_column_w / 2.0,
                CASE_D / 2.0 - FRONT_T / 2.0,
                CASE_H / 2.0,
            ),
        ),
        material=case_beige,
        name="front_left_column",
    )
    chassis.visual(
        Box((front_column_w, FRONT_T, CASE_H)),
        origin=Origin(
            xyz=(
                CASE_W / 2.0 - front_column_w / 2.0,
                CASE_D / 2.0 - FRONT_T / 2.0,
                CASE_H / 2.0,
            ),
        ),
        material=case_beige,
        name="front_right_column",
    )
    chassis.visual(
        Box((BAY_W, FRONT_T, BAY_Z0)),
        origin=Origin(
            xyz=(0.0, CASE_D / 2.0 - FRONT_T / 2.0, BAY_Z0 / 2.0),
        ),
        material=case_beige,
        name="front_lower_panel",
    )
    chassis.visual(
        Box((BAY_W, FRONT_T, CASE_H - BAY_Z1)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_D / 2.0 - FRONT_T / 2.0,
                BAY_Z1 + (CASE_H - BAY_Z1) / 2.0,
            ),
        ),
        material=case_beige,
        name="front_upper_panel",
    )
    chassis.visual(
        Box((BAY_W, CASE_D - FRONT_T - REAR_T, BAY_FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, BAY_FLOOR_Z)),
        material=dark_plastic,
        name="bay_floor",
    )
    chassis.visual(
        Cylinder(radius=BEZEL_HINGE_PIN_R, length=BEZEL_W - 0.012),
        origin=Origin(
            xyz=(0.0, CASE_D / 2.0, BEZEL_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=case_shadow,
        name="bezel_hinge_pin",
    )
    chassis.visual(
        Box((0.022, 0.006, 0.022)),
        origin=Origin(xyz=(-0.048, CASE_D / 2.0 + 0.003, 0.084)),
        material=plastic_beige,
        name="power_button",
    )
    chassis.visual(
        Box((0.008, 0.004, 0.008)),
        origin=Origin(xyz=(0.052, CASE_D / 2.0 + 0.002, 0.075)),
        material=led_green,
        name="status_led",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, CASE_H / 2.0)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((PANEL_T, PANEL_D, PANEL_H)),
        origin=Origin(
            xyz=(
                CASE_W / 2.0 + PANEL_T / 2.0 - SIDE_HINGE_AXIS_X,
                SIDE_PANEL_CENTER_Y,
                PANEL_H / 2.0,
            )
        ),
        material=plastic_beige,
        name="side_skin",
    )
    side_panel.visual(
        Cylinder(radius=SIDE_HINGE_BARREL_R, length=PANEL_H),
        origin=Origin(xyz=(0.0, 0.0, PANEL_H / 2.0)),
        material=case_shadow,
        name="panel_hinge_barrel",
    )
    side_panel.visual(
        Box((0.010, 0.040, 0.070)),
        origin=Origin(
            xyz=(
                CASE_W / 2.0 + PANEL_T + 0.005 - SIDE_HINGE_AXIS_X,
                SIDE_PANEL_CENTER_Y + PANEL_D / 2.0 - 0.050,
                PANEL_H / 2.0 - 0.015,
            ),
        ),
        material=case_shadow,
        name="latch_tab",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((PANEL_T, PANEL_D, PANEL_H)),
        mass=0.9,
        origin=Origin(
            xyz=(
                CASE_W / 2.0 + PANEL_T / 2.0 - SIDE_HINGE_AXIS_X,
                SIDE_PANEL_CENTER_Y,
                PANEL_H / 2.0,
            )
        ),
    )

    bezel = model.part("drive_bezel")
    bezel.visual(
        Cylinder(radius=BEZEL_HINGE_BARREL_R, length=BEZEL_W - 0.008),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=case_shadow,
        name="bezel_hinge_barrel",
    )
    bezel.visual(
        Box((BEZEL_W, BEZEL_T, BEZEL_H)),
        origin=Origin(xyz=(0.0, BEZEL_T / 2.0, BEZEL_H / 2.0)),
        material=plastic_beige,
        name="bezel_door",
    )
    bezel.visual(
        Box((0.048, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, BEZEL_T + 0.002, 0.007)),
        material=case_shadow,
        name="bezel_handle",
    )
    bezel.inertial = Inertial.from_geometry(
        Box((BEZEL_W, BEZEL_T, BEZEL_H)),
        mass=0.12,
        origin=Origin(xyz=(0.0, BEZEL_T / 2.0, BEZEL_H / 2.0)),
    )

    tray = model.part("drive_tray")
    tray.visual(
        Box((TRAY_W, TRAY_LEN, TRAY_H)),
        origin=Origin(),
        material=dark_plastic,
        name="tray_slab",
    )
    tray.visual(
        Box((TRAY_W * 0.96, TRAY_FACE_T, 0.032)),
        origin=Origin(
            xyz=(0.0, TRAY_LEN / 2.0 - TRAY_FACE_T / 2.0, 0.011),
        ),
        material=plastic_beige,
        name="tray_front",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_W, TRAY_LEN, 0.032)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(SIDE_HINGE_AXIS_X, SIDE_HINGE_AXIS_Y, PANEL_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-1.55,
            upper=0.0,
        ),
    )
    model.articulation(
        "drive_bezel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=bezel,
        origin=Origin(xyz=(0.0, CASE_D / 2.0, BEZEL_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.35,
            upper=0.0,
        ),
    )
    model.articulation(
        "drive_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_CENTER_Y, TRAY_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.25,
            lower=0.0,
            upper=0.13,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    bezel = object_model.get_part("drive_bezel")
    tray = object_model.get_part("drive_tray")

    side_panel_hinge = object_model.get_articulation("side_panel_hinge")
    drive_bezel_hinge = object_model.get_articulation("drive_bezel_hinge")
    drive_tray_slide = object_model.get_articulation("drive_tray_slide")

    side_hinge_post = chassis.get_visual("side_panel_hinge_post")
    bezel_hinge_pin = chassis.get_visual("bezel_hinge_pin")
    side_skin = side_panel.get_visual("side_skin")
    panel_hinge_barrel = side_panel.get_visual("panel_hinge_barrel")
    latch_tab = side_panel.get_visual("latch_tab")
    bezel_hinge_barrel = bezel.get_visual("bezel_hinge_barrel")
    bezel_door = bezel.get_visual("bezel_door")
    bezel_handle = bezel.get_visual("bezel_handle")
    tray_slab = tray.get_visual("tray_slab")
    tray_front = tray.get_visual("tray_front")
    bay_floor = chassis.get_visual("bay_floor")
    front_lower_panel = chassis.get_visual("front_lower_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        side_panel,
        chassis,
        elem_a=panel_hinge_barrel,
        elem_b=side_hinge_post,
        reason="The side panel rides on a captured rear hinge knuckle.",
    )
    ctx.allow_overlap(
        side_panel,
        chassis,
        elem_a=side_skin,
        elem_b=side_hinge_post,
        reason="The stamped panel skin wraps tightly around the hinge line.",
    )
    ctx.allow_overlap(
        bezel,
        chassis,
        elem_a=bezel_hinge_barrel,
        elem_b=bezel_hinge_pin,
        reason="The drive-bay flap rotates on a captured plastic hinge barrel.",
    )
    ctx.allow_overlap(
        bezel,
        chassis,
        elem_a=bezel_door,
        elem_b=bezel_hinge_pin,
        reason="The lower edge of the bezel door is molded around the hinge pin.",
    )

    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        side_panel,
        chassis,
        elem_a=panel_hinge_barrel,
        elem_b=side_hinge_post,
        name="side panel remains captured on the rear hinge",
    )
    ctx.expect_overlap(
        side_panel,
        chassis,
        axes="yz",
        min_overlap=0.30,
        elem_a=side_skin,
        name="side panel covers the tower side opening",
    )

    ctx.expect_contact(
        bezel,
        chassis,
        elem_a=bezel_hinge_barrel,
        elem_b=bezel_hinge_pin,
        name="drive bezel remains captured on its lower hinge pin",
    )
    ctx.expect_overlap(
        bezel,
        chassis,
        axes="xz",
        min_overlap=0.03,
        elem_a=bezel_door,
        name="drive bezel door spans the full drive-bay opening",
    )

    ctx.expect_contact(tray, chassis, elem_a=tray_slab, elem_b=bay_floor)
    ctx.expect_gap(
        tray,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_slab,
        negative_elem=bay_floor,
        name="tray rides directly on the drive bay floor",
    )
    ctx.expect_within(
        tray,
        chassis,
        axes="x",
        inner_elem=tray_slab,
        name="tray stays centered inside the 5.25-inch bay",
    )
    ctx.expect_gap(
        chassis,
        tray,
        axis="y",
        min_gap=0.004,
        max_gap=0.03,
        positive_elem=front_lower_panel,
        negative_elem=tray_front,
        name="tray front sits just behind the bay opening at rest",
    )

    side_skin_rest = ctx.part_element_world_aabb(side_panel, elem=side_skin)
    bezel_door_rest = ctx.part_element_world_aabb(bezel, elem=bezel_door)
    bezel_handle_rest = ctx.part_element_world_aabb(bezel, elem=bezel_handle)
    side_panel_rest = ctx.part_world_aabb(side_panel)
    tray_rest = ctx.part_world_position(tray)

    if side_skin_rest is not None:
        ctx.check(
            "side panel sits on the right side of the tower",
            abs(side_skin_rest[0][0] - CASE_W / 2.0) <= 0.001,
            details=f"side skin min x={side_skin_rest[0][0]:.4f}",
        )

    if bezel_door_rest is not None:
        ctx.check(
            "drive bezel sits flush with the front fascia",
            abs(bezel_door_rest[0][1] - CASE_D / 2.0) <= 0.001,
            details=f"bezel door min y={bezel_door_rest[0][1]:.4f}",
        )

    if side_panel_rest is not None:
        with ctx.pose({side_panel_hinge: -1.2}):
            side_panel_open = ctx.part_world_aabb(side_panel)
            ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_barrel, elem_b=side_hinge_post)
            ctx.expect_gap(
                side_panel,
                chassis,
                axis="x",
                min_gap=0.20,
                positive_elem=latch_tab,
                name="opened side panel swings well clear of the chassis",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="side_panel_open_clear")
            ctx.fail_if_isolated_parts(name="side_panel_open_supported")
            if side_panel_open is not None:
                ctx.check(
                    "side panel rotates outward on a vertical hinge",
                    side_panel_open[1][0] > side_panel_rest[1][0] + 0.25,
                    details=(
                        f"closed max x={side_panel_rest[1][0]:.4f}, "
                        f"open max x={side_panel_open[1][0]:.4f}"
                    ),
                )
                ctx.check(
                    "side panel swings toward the rear when opened",
                    side_panel_open[1][1] < side_panel_rest[1][1] - 0.20,
                    details=(
                        f"closed max y={side_panel_rest[1][1]:.4f}, "
                        f"open max y={side_panel_open[1][1]:.4f}"
                    ),
                )

    if bezel_handle_rest is not None:
        with ctx.pose({drive_bezel_hinge: -1.2}):
            bezel_handle_open = ctx.part_element_world_aabb(bezel, elem=bezel_handle)
            ctx.expect_contact(bezel, chassis, elem_a=bezel_hinge_barrel, elem_b=bezel_hinge_pin)
            ctx.fail_if_parts_overlap_in_current_pose(name="drive_bezel_open_clear")
            ctx.fail_if_isolated_parts(name="drive_bezel_open_supported")
            if bezel_handle_open is not None:
                ctx.check(
                    "drive bezel handle moves outward when opened",
                    bezel_handle_open[1][1] > bezel_handle_rest[1][1] + 0.004,
                    details=(
                        f"closed handle max y={bezel_handle_rest[1][1]:.4f}, "
                        f"open handle max y={bezel_handle_open[1][1]:.4f}"
                    ),
                )
                ctx.check(
                    "drive bezel flips downward from its lower hinge",
                    bezel_handle_open[0][2] < bezel_handle_rest[0][2] - 0.005,
                    details=(
                        f"closed handle min z={bezel_handle_rest[0][2]:.4f}, "
                        f"open handle min z={bezel_handle_open[0][2]:.4f}"
                    ),
                )

    if tray_rest is not None:
        with ctx.pose({drive_tray_slide: 0.11}):
            tray_open = ctx.part_world_position(tray)
            ctx.expect_contact(tray, chassis, elem_a=tray_slab, elem_b=bay_floor)
            ctx.expect_gap(
                tray,
                chassis,
                axis="y",
                min_gap=0.06,
                positive_elem=tray_front,
                negative_elem=front_lower_panel,
                name="tray projects out of the bay when ejected",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="drive_tray_open_clear")
            ctx.fail_if_isolated_parts(name="drive_tray_open_supported")
            if tray_open is not None:
                ctx.check(
                    "drive tray slides forward along the case depth axis",
                    tray_open[1] > tray_rest[1] + 0.10,
                    details=f"closed y={tray_rest[1]:.4f}, open y={tray_open[1]:.4f}",
                )

    with ctx.pose({drive_bezel_hinge: -1.2, drive_tray_slide: 0.11}):
        ctx.expect_gap(
            tray,
            bezel,
            axis="y",
            min_gap=0.03,
            positive_elem=tray_front,
            negative_elem=bezel_door,
            name="ejected tray clears the opened drive bezel",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="tray_and_bezel_combined_clear")
        ctx.fail_if_isolated_parts(name="tray_and_bezel_combined_supported")

    for articulation in (side_panel_hinge, drive_bezel_hinge, drive_tray_slide):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{articulation.name}_lower_limit_clear"
            )
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_limit_supported")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{articulation.name}_upper_limit_clear"
            )
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_limit_supported")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
