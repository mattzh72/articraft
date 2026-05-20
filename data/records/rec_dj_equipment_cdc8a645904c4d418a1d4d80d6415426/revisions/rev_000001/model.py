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


BODY_W = 0.320
BODY_D = 0.400
BODY_H = 0.090
SIDE_T = 0.012
FRONT_T = 0.015
REAR_T = 0.015
TOP_T = 0.008
BOTTOM_T = 0.008

TRAY_W = 0.136
TRAY_D = 0.145
TRAY_H = 0.014
TRAY_TRAVEL = 0.110
SLOT_BOTTOM = 0.022
SLOT_H = 0.026

PLATTER_R = 0.094
PLATTER_T = 0.016

SCREEN_W = 0.134
SCREEN_H = 0.082
SCREEN_D = 0.018
SCREEN_BARREL_R = 0.006
SCREEN_BARREL_W = 0.140
SCREEN_BASE_TILT = 0.34


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cdj_media_player")

    model.material("body_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("deck_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("platter_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("screen_bezel", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("display_glow", rgba=(0.30, 0.72, 0.52, 1.0))
    model.material("tray_face", rgba=(0.60, 0.62, 0.64, 1.0))
    model.material("accent_dark", rgba=(0.20, 0.20, 0.22, 1.0))

    body = model.part("body")

    inner_depth = BODY_D - FRONT_T - REAR_T
    inner_height = BODY_H - TOP_T - BOTTOM_T
    cheek_w = (BODY_W - TRAY_W) / 2.0
    tray_floor_d = 0.180

    body.visual(
        Box((BODY_W - 2.0 * SIDE_T, inner_depth, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T / 2.0)),
        material="body_dark",
        name="bottom_plate",
    )
    body.visual(
        Box((BODY_W, inner_depth, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_T / 2.0)),
        material="body_dark",
        name="deck_top",
    )
    body.visual(
        Box((SIDE_T, inner_depth, inner_height)),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + SIDE_T / 2.0, 0.0, BOTTOM_T + inner_height / 2.0)
        ),
        material="body_dark",
        name="left_wall",
    )
    body.visual(
        Box((SIDE_T, inner_depth, inner_height)),
        origin=Origin(
            xyz=(BODY_W / 2.0 - SIDE_T / 2.0, 0.0, BOTTOM_T + inner_height / 2.0)
        ),
        material="body_dark",
        name="right_wall",
    )
    body.visual(
        Box((BODY_W, FRONT_T, SLOT_BOTTOM)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + FRONT_T / 2.0, SLOT_BOTTOM / 2.0)
        ),
        material="body_dark",
        name="front_lower_panel",
    )
    body.visual(
        Box((BODY_W, FRONT_T, BODY_H - (SLOT_BOTTOM + SLOT_H))),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + FRONT_T / 2.0,
                SLOT_BOTTOM + SLOT_H + (BODY_H - (SLOT_BOTTOM + SLOT_H)) / 2.0,
            )
        ),
        material="body_dark",
        name="front_upper_panel",
    )
    body.visual(
        Box((cheek_w, FRONT_T, SLOT_H)),
        origin=Origin(
            xyz=(
                -TRAY_W / 2.0 - cheek_w / 2.0,
                -BODY_D / 2.0 + FRONT_T / 2.0,
                SLOT_BOTTOM + SLOT_H / 2.0,
            )
        ),
        material="body_dark",
        name="front_left_cheek",
    )
    body.visual(
        Box((cheek_w, FRONT_T, SLOT_H)),
        origin=Origin(
            xyz=(
                TRAY_W / 2.0 + cheek_w / 2.0,
                -BODY_D / 2.0 + FRONT_T / 2.0,
                SLOT_BOTTOM + SLOT_H / 2.0,
            )
        ),
        material="body_dark",
        name="front_right_cheek",
    )
    body.visual(
        Box((BODY_W, REAR_T, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - REAR_T / 2.0, BODY_H / 2.0)),
        material="body_dark",
        name="rear_wall",
    )
    body.visual(
        Box((TRAY_W + 0.018, tray_floor_d, 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + FRONT_T + tray_floor_d / 2.0,
                SLOT_BOTTOM - 0.002,
            )
        ),
        material="accent_dark",
        name="tray_floor",
    )
    body.visual(
        Box((0.004, tray_floor_d, 0.024)),
        origin=Origin(
            xyz=(
                -TRAY_W / 2.0 - 0.005,
                -BODY_D / 2.0 + FRONT_T + tray_floor_d / 2.0,
                SLOT_BOTTOM + 0.012,
            )
        ),
        material="accent_dark",
        name="tray_left_guide",
    )
    body.visual(
        Box((0.004, tray_floor_d, 0.024)),
        origin=Origin(
            xyz=(
                TRAY_W / 2.0 + 0.005,
                -BODY_D / 2.0 + FRONT_T + tray_floor_d / 2.0,
                SLOT_BOTTOM + 0.012,
            )
        ),
        material="accent_dark",
        name="tray_right_guide",
    )
    body.visual(
        Box((0.064, 0.110, 0.003)),
        origin=Origin(xyz=(-0.112, -0.030, BODY_H + 0.0015)),
        material="deck_trim",
        name="left_control_bank",
    )
    body.visual(
        Box((0.022, 0.180, 0.002)),
        origin=Origin(xyz=(0.132, -0.010, BODY_H + 0.001)),
        material="deck_trim",
        name="pitch_strip",
    )
    body.visual(
        Box((0.036, 0.060, 0.003)),
        origin=Origin(xyz=(0.118, -0.115, BODY_H + 0.0015)),
        material="deck_trim",
        name="transport_cluster",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=PLATTER_R, length=PLATTER_T),
        origin=Origin(xyz=(0.0, 0.0, PLATTER_T / 2.0)),
        material="platter_black",
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.060, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, PLATTER_T - 0.002)),
        material="accent_dark",
        name="touch_pad",
    )
    platter.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, PLATTER_T + 0.005)),
        material="tray_face",
        name="spindle_cap",
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_H)),
        origin=Origin(xyz=(0.0, TRAY_D / 2.0, 0.0)),
        material="accent_dark",
        name="drawer",
    )
    tray.visual(
        Box((TRAY_W + 0.008, FRONT_T, SLOT_H)),
        origin=Origin(xyz=(0.0, -FRONT_T / 2.0, 0.005)),
        material="tray_face",
        name="tray_face",
    )
    tray.visual(
        Cylinder(radius=0.050, length=0.002),
        origin=Origin(xyz=(0.0, 0.058, TRAY_H / 2.0 + 0.001)),
        material="deck_trim",
        name="disc_recess",
    )

    screen = model.part("screen")
    screen.visual(
        Cylinder(radius=SCREEN_BARREL_R, length=SCREEN_BARREL_W),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="accent_dark",
        name="hinge_barrel",
    )
    screen.visual(
        Box((0.118, 0.014, 0.020)),
        origin=Origin(
            xyz=(0.0, -0.010, 0.010),
            rpy=(SCREEN_BASE_TILT, 0.0, 0.0),
        ),
        material="screen_bezel",
        name="hinge_bridge",
    )
    screen.visual(
        Box((SCREEN_W, SCREEN_D, SCREEN_H)),
        origin=Origin(
            xyz=(0.0, -0.038, 0.048),
            rpy=(SCREEN_BASE_TILT, 0.0, 0.0),
        ),
        material="screen_bezel",
        name="screen_housing",
    )
    screen.visual(
        Box((0.116, 0.003, 0.060)),
        origin=Origin(
            xyz=(0.0, -0.029, 0.052),
            rpy=(SCREEN_BASE_TILT, 0.0, 0.0),
        ),
        material="display_glow",
        name="lcd_panel",
    )

    model.articulation(
        "body_to_platter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=platter,
        origin=Origin(xyz=(0.0, 0.010, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=15.0,
            lower=-6.0,
            upper=6.0,
        ),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + FRONT_T, SLOT_BOTTOM + TRAY_H / 2.0)
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - REAR_T, BODY_H + SCREEN_BARREL_R)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    platter = object_model.get_part("platter")
    tray = object_model.get_part("tray")
    screen = object_model.get_part("screen")

    platter_joint = object_model.get_articulation("body_to_platter")
    tray_joint = object_model.get_articulation("body_to_tray")
    screen_joint = object_model.get_articulation("body_to_screen")

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

    ctx.check(
        "prompt articulations use the intended axes",
        platter_joint.axis == (0.0, 0.0, 1.0)
        and tray_joint.axis == (0.0, -1.0, 0.0)
        and screen_joint.axis == (-1.0, 0.0, 0.0),
        details=(
            f"platter={platter_joint.axis}, tray={tray_joint.axis}, "
            f"screen={screen_joint.axis}"
        ),
    )

    ctx.expect_contact(
        platter,
        body,
        elem_a="platter_disc",
        elem_b="deck_top",
        name="platter is physically seated on the deck",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="drawer",
        elem_b="tray_floor",
        name="tray is supported by the tray floor",
    )
    ctx.expect_contact(
        screen,
        body,
        elem_a="hinge_barrel",
        elem_b="deck_top",
        name="screen hinge barrel is mounted on the top deck",
    )

    ctx.expect_within(
        tray,
        body,
        axes="x",
        inner_elem="drawer",
        outer_elem="tray_floor",
        margin=0.009,
        name="tray drawer stays centered between guides",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="drawer",
        elem_b="tray_floor",
        min_overlap=0.140,
        name="closed tray remains fully parked in the chassis",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        ctx.expect_contact(
            tray,
            body,
            elem_a="drawer",
            elem_b="tray_floor",
            name="extended tray stays supported by the tray floor",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="drawer",
            elem_b="tray_floor",
            min_overlap=0.030,
            name="extended tray retains insertion in the chassis",
        )
        tray_extended = ctx.part_world_position(tray)

    ctx.check(
        "tray slides outward from the front face",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] < tray_rest[1] - 0.080,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    screen_rest = ctx.part_element_world_aabb(screen, elem="screen_housing")
    with ctx.pose({screen_joint: 0.85}):
        screen_raised = ctx.part_element_world_aabb(screen, elem="screen_housing")

    ctx.check(
        "screen hinge tips the display upward and rearward",
        screen_rest is not None
        and screen_raised is not None
        and screen_raised[1][1] > screen_rest[1][1] + 0.050
        and screen_raised[1][2] > screen_rest[1][2] + 0.010,
        details=f"rest={screen_rest}, raised={screen_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
