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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="casino_slot_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.66, 0.08, 0.10, 1.0))
    trim_gold = model.material("trim_gold", rgba=(0.83, 0.69, 0.18, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.24, 0.28, 0.36))
    charcoal = model.material("charcoal", rgba=(0.11, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    ivory = model.material("ivory", rgba=(0.93, 0.92, 0.86, 1.0))
    reel_red = model.material("reel_red", rgba=(0.70, 0.14, 0.14, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.74, 0.58, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.70, 0.02, 1.58)),
        origin=Origin(xyz=(0.0, -0.25, 0.89)),
        material=charcoal,
        name="back_wall",
    )
    cabinet.visual(
        Box((0.06, 0.54, 1.58)),
        origin=Origin(xyz=(-0.31, 0.0, 0.89)),
        material=cabinet_red,
        name="left_side_wall",
    )
    cabinet.visual(
        Box((0.06, 0.54, 1.58)),
        origin=Origin(xyz=(0.31, 0.0, 0.89)),
        material=cabinet_red,
        name="right_side_wall",
    )
    cabinet.visual(
        Box((0.56, 0.34, 0.46)),
        origin=Origin(xyz=(0.0, -0.07, 0.33)),
        material=cabinet_red,
        name="lower_body_core",
    )
    cabinet.visual(
        Box((0.56, 0.28, 0.30)),
        origin=Origin(xyz=(0.0, -0.10, 1.33)),
        material=cabinet_red,
        name="upper_body_core",
    )
    cabinet.visual(
        Box((0.58, 0.52, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.51)),
        material=charcoal,
        name="roof_deck",
    )
    cabinet.visual(
        Box((0.62, 0.03, 0.32)),
        origin=Origin(xyz=(0.0, 0.255, 0.22)),
        material=trim_gold,
        name="lower_front_panel",
    )
    cabinet.visual(
        Box((0.46, 0.03, 0.12)),
        origin=Origin(xyz=(0.0, 0.255, 0.44)),
        material=trim_gold,
        name="tray_support_panel",
    )
    cabinet.visual(
        Box((0.50, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, 0.17, 0.53)),
        material=trim_gold,
        name="deck_ledge",
    )
    cabinet.visual(
        Box((0.50, 0.04, 0.07)),
        origin=Origin(xyz=(0.0, 0.245, 0.585)),
        material=trim_gold,
        name="window_bottom",
    )
    cabinet.visual(
        Box((0.09, 0.04, 0.50)),
        origin=Origin(xyz=(-0.205, 0.245, 0.86)),
        material=trim_gold,
        name="window_left",
    )
    cabinet.visual(
        Box((0.09, 0.04, 0.50)),
        origin=Origin(xyz=(0.205, 0.245, 0.86)),
        material=trim_gold,
        name="window_right",
    )
    cabinet.visual(
        Box((0.50, 0.04, 0.07)),
        origin=Origin(xyz=(0.0, 0.245, 1.135)),
        material=trim_gold,
        name="window_top",
    )
    cabinet.visual(
        Box((0.03, 0.03, 0.46)),
        origin=Origin(xyz=(-0.055, 0.24, 0.86)),
        material=trim_gold,
        name="window_mullion_left",
    )
    cabinet.visual(
        Box((0.03, 0.03, 0.46)),
        origin=Origin(xyz=(0.055, 0.24, 0.86)),
        material=trim_gold,
        name="window_mullion_right",
    )
    cabinet.visual(
        Box((0.40, 0.01, 0.40)),
        origin=Origin(xyz=(0.0, 0.228, 0.86)),
        material=smoked_glass,
        name="window_glass",
    )
    cabinet.visual(
        Box((0.56, 0.03, 0.12)),
        origin=Origin(xyz=(0.0, 0.255, 1.23)),
        material=trim_gold,
        name="upper_front_panel",
    )
    cabinet.visual(
        Box((0.56, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, 0.20, 1.29)),
        material=trim_gold,
        name="marquee_header",
    )
    cabinet.visual(
        Box((0.60, 0.07, 0.36)),
        origin=Origin(xyz=(0.0, 0.215, 1.50)),
        material=trim_gold,
        name="marquee_front",
    )
    cabinet.visual(
        Box((0.60, 0.01, 0.24)),
        origin=Origin(xyz=(0.0, 0.226, 1.50)),
        material=smoked_glass,
        name="marquee_glass",
    )
    cabinet.visual(
        Box((0.60, 0.50, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.70)),
        material=charcoal,
        name="marquee_top",
    )
    cabinet.visual(
        Cylinder(radius=0.05, length=0.06),
        origin=Origin(xyz=(0.355, 0.08, 0.80), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="pivot_boss",
    )
    cabinet.visual(
        Box((0.06, 0.14, 0.22)),
        origin=Origin(xyz=(0.355, 0.08, 0.80)),
        material=trim_gold,
        name="pivot_plinth",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.74, 0.58, 1.72)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
    )

    reel_bank = model.part("reel_bank")
    reel_bank.visual(
        Box((0.44, 0.02, 0.54)),
        origin=Origin(xyz=(0.0, 0.01, 0.27)),
        material=charcoal,
        name="backplate",
    )
    reel_bank.visual(
        Box((0.03, 0.30, 0.54)),
        origin=Origin(xyz=(-0.235, 0.15, 0.27)),
        material=charcoal,
        name="left_bracket",
    )
    reel_bank.visual(
        Box((0.03, 0.30, 0.54)),
        origin=Origin(xyz=(0.235, 0.15, 0.27)),
        material=charcoal,
        name="right_bracket",
    )
    reel_bank.visual(
        Box((0.44, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.15, 0.04)),
        material=charcoal,
        name="lower_crossbar",
    )
    reel_bank.visual(
        Box((0.44, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.15, 0.50)),
        material=charcoal,
        name="upper_crossbar",
    )
    reel_bank.visual(
        Cylinder(radius=0.012, length=0.44),
        origin=Origin(xyz=(0.0, 0.26, 0.27), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="reel_shaft",
    )
    for x_pos, reel_name in ((-0.13, "left_reel"), (0.0, "center_reel"), (0.13, "right_reel")):
        reel_bank.visual(
            Cylinder(radius=0.11, length=0.10),
            origin=Origin(xyz=(x_pos, 0.26, 0.27), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=ivory,
            name=reel_name,
        )
        reel_bank.visual(
            Box((0.07, 0.14, 0.025)),
            origin=Origin(xyz=(x_pos, 0.363, 0.27)),
            material=reel_red,
            name=f"{reel_name}_symbol_band",
        )
    reel_bank.inertial = Inertial.from_geometry(
        Box((0.50, 0.40, 0.54)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.20, 0.27)),
    )

    coin_tray = model.part("coin_tray")
    coin_tray.visual(
        Box((0.34, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.01, 0.025)),
        material=steel,
        name="tray_back",
    )
    coin_tray.visual(
        Box((0.34, 0.16, 0.015)),
        origin=Origin(xyz=(0.0, 0.10, 0.0075)),
        material=steel,
        name="tray_floor",
    )
    coin_tray.visual(
        Box((0.015, 0.16, 0.045)),
        origin=Origin(xyz=(-0.1625, 0.10, 0.0225)),
        material=steel,
        name="tray_left_wall",
    )
    coin_tray.visual(
        Box((0.015, 0.16, 0.045)),
        origin=Origin(xyz=(0.1625, 0.10, 0.0225)),
        material=steel,
        name="tray_right_wall",
    )
    coin_tray.visual(
        Box((0.34, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.19, 0.015)),
        material=steel,
        name="tray_front_lip",
    )
    coin_tray.inertial = Inertial.from_geometry(
        Box((0.34, 0.18, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.09, 0.025)),
    )

    handle_arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.028, 0.0, 0.02),
                (0.040, 0.016, 0.08),
                (0.050, 0.038, 0.18),
                (0.058, 0.075, 0.32),
                (0.060, 0.120, 0.46),
            ],
            radius=0.015,
            samples_per_segment=18,
            radial_segments=18,
        ),
        "slot_machine_pull_handle_arm",
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Cylinder(radius=0.035, length=0.03),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="handle_hub",
    )
    pull_handle.visual(
        handle_arm_mesh,
        material=satin_black,
        name="handle_arm",
    )
    pull_handle.visual(
        Cylinder(radius=0.032, length=0.14),
        origin=Origin(xyz=(0.060, 0.120, 0.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="pull_grip",
    )
    pull_handle.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.060, 0.050, 0.46)),
        material=satin_black,
        name="grip_lower_cap",
    )
    pull_handle.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.060, 0.190, 0.46)),
        material=satin_black,
        name="grip_upper_cap",
    )
    pull_handle.inertial = Inertial.from_geometry(
        Box((0.14, 0.24, 0.56)),
        mass=1.8,
        origin=Origin(xyz=(0.07, 0.10, 0.28)),
    )

    model.articulation(
        "cabinet_to_reel_bank",
        ArticulationType.FIXED,
        parent=cabinet,
        child=reel_bank,
        origin=Origin(xyz=(0.0, -0.24, 0.60)),
    )
    model.articulation(
        "cabinet_to_coin_tray",
        ArticulationType.FIXED,
        parent=cabinet,
        child=coin_tray,
        origin=Origin(xyz=(0.0, 0.27, 0.12)),
    )
    model.articulation(
        "cabinet_to_pull_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=pull_handle,
        origin=Origin(xyz=(0.385, 0.08, 0.80)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    reel_bank = object_model.get_part("reel_bank")
    coin_tray = object_model.get_part("coin_tray")
    pull_handle = object_model.get_part("pull_handle")
    handle_joint = object_model.get_articulation("cabinet_to_pull_handle")

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

    ctx.expect_contact(
        reel_bank,
        cabinet,
        elem_a="backplate",
        elem_b="back_wall",
        name="reel bank mounts against the cabinet back wall",
    )
    ctx.expect_contact(
        coin_tray,
        cabinet,
        elem_a="tray_back",
        elem_b="lower_front_panel",
        name="coin tray is mounted to the lower front panel",
    )
    ctx.expect_contact(
        pull_handle,
        cabinet,
        elem_a="handle_hub",
        elem_b="pivot_boss",
        name="pull handle is carried by the side pivot boss",
    )
    ctx.expect_gap(
        cabinet,
        reel_bank,
        axis="y",
        positive_elem="window_glass",
        negative_elem="center_reel",
        min_gap=0.01,
        max_gap=0.15,
        name="reels sit visibly behind the reel window",
    )
    ctx.expect_overlap(
        reel_bank,
        cabinet,
        axes="xz",
        elem_a="left_reel",
        elem_b="window_glass",
        min_overlap=0.07,
        name="left reel remains visible through the window",
    )
    ctx.expect_overlap(
        reel_bank,
        cabinet,
        axes="xz",
        elem_a="center_reel",
        elem_b="window_glass",
        min_overlap=0.08,
        name="center reel remains visible through the window",
    )
    ctx.expect_overlap(
        reel_bank,
        cabinet,
        axes="xz",
        elem_a="right_reel",
        elem_b="window_glass",
        min_overlap=0.07,
        name="right reel remains visible through the window",
    )

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    if cabinet_aabb is None:
        ctx.fail("cabinet bounds resolve", "cabinet world aabb was unavailable")
    else:
        dims = tuple(cabinet_aabb[1][axis] - cabinet_aabb[0][axis] for axis in range(3))
        ctx.check(
            "slot machine keeps realistic tall-cabinet proportions",
            0.68 <= dims[0] <= 0.82 and 0.54 <= dims[1] <= 0.66 and 1.68 <= dims[2] <= 1.78,
            details=f"dims={dims}",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    grip_rest = ctx.part_element_world_aabb(pull_handle, elem="pull_grip")
    upper = 0.95
    if handle_joint.motion_limits is not None and handle_joint.motion_limits.upper is not None:
        upper = handle_joint.motion_limits.upper

    with ctx.pose({handle_joint: upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pulled handle pose stays clear of the cabinet")
        grip_pulled = ctx.part_element_world_aabb(pull_handle, elem="pull_grip")

    grip_rest_center = _aabb_center(grip_rest)
    grip_pulled_center = _aabb_center(grip_pulled)
    ctx.check(
        "pull handle swings downward and outward",
        grip_rest_center is not None
        and grip_pulled_center is not None
        and grip_pulled_center[2] < grip_rest_center[2] - 0.16
        and grip_pulled_center[1] > grip_rest_center[1] + 0.16,
        details=f"rest={grip_rest_center}, pulled={grip_pulled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
