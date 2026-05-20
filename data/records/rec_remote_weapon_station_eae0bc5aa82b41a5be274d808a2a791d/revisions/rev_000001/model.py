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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    armor = model.material("armor", rgba=(0.42, 0.45, 0.42, 1.0))
    dark_armor = model.material("dark_armor", rgba=(0.20, 0.22, 0.22, 1.0))
    launcher_front = model.material("launcher_front", rgba=(0.16, 0.17, 0.16, 1.0))
    optics_glass = model.material("optics_glass", rgba=(0.14, 0.28, 0.34, 1.0))

    cyl_x = (0.0, math.pi / 2.0, 0.0)
    cyl_y = (math.pi / 2.0, 0.0, 0.0)

    pedestal = model.part("pedestal_base")
    pedestal.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_armor,
        name="floor_ring",
    )
    pedestal.visual(
        Box((0.42, 0.38, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=armor,
        name="pedestal_body",
    )
    pedestal.visual(
        Box((0.30, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=armor,
        name="shoulder_box",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=dark_armor,
        name="turntable_ring",
    )

    azimuth_frame = model.part("azimuth_frame")
    azimuth_frame.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_armor,
        name="azimuth_bearing",
    )
    azimuth_frame.visual(
        Box((0.42, 0.28, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=armor,
        name="deck",
    )
    azimuth_frame.visual(
        Box((0.05, 0.18, 0.24)),
        origin=Origin(xyz=(-0.16, 0.0, 0.22)),
        material=armor,
        name="left_support",
    )
    azimuth_frame.visual(
        Box((0.05, 0.18, 0.24)),
        origin=Origin(xyz=(0.16, 0.0, 0.22)),
        material=armor,
        name="right_support",
    )
    azimuth_frame.visual(
        Box((0.03, 0.05, 0.12)),
        origin=Origin(xyz=(-0.145, -0.115, 0.18)),
        material=armor,
        name="left_rear_gusset",
    )
    azimuth_frame.visual(
        Box((0.03, 0.05, 0.12)),
        origin=Origin(xyz=(0.145, -0.115, 0.18)),
        material=armor,
        name="right_rear_gusset",
    )
    azimuth_frame.visual(
        Box((0.03, 0.04, 0.03)),
        origin=Origin(xyz=(-0.145, 0.10, 0.345)),
        material=armor,
        name="left_front_cap",
    )
    azimuth_frame.visual(
        Box((0.03, 0.04, 0.03)),
        origin=Origin(xyz=(0.145, 0.10, 0.345)),
        material=armor,
        name="right_front_cap",
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Box((0.24, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, -0.01, -0.03)),
        material=dark_armor,
        name="trunnion_block",
    )
    weapon_cradle.visual(
        Box((0.14, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.08, -0.03)),
        material=dark_armor,
        name="receiver",
    )
    weapon_cradle.visual(
        Box((0.09, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.05, 0.03)),
        material=dark_armor,
        name="breech_cover",
    )
    weapon_cradle.visual(
        Box((0.08, 0.14, 0.13)),
        origin=Origin(xyz=(-0.08, -0.03, 0.08)),
        material=armor,
        name="ammo_box",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.05, length=0.02),
        origin=Origin(xyz=(-0.125, 0.0, 0.0), rpy=cyl_x),
        material=dark_armor,
        name="left_trunnion",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.05, length=0.02),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=cyl_x),
        material=dark_armor,
        name="right_trunnion",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.028, length=0.72),
        origin=Origin(xyz=(0.0, 0.46, -0.02), rpy=cyl_y),
        material=dark_armor,
        name="barrel_shroud",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.91, -0.02), rpy=cyl_y),
        material=dark_armor,
        name="muzzle",
    )

    missile_pod = model.part("missile_pod")
    missile_pod.visual(
        Box((0.03, 0.12, 0.14)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=dark_armor,
        name="mount_block",
    )
    missile_pod.visual(
        Box((0.18, 0.46, 0.20)),
        origin=Origin(xyz=(0.12, 0.03, 0.0)),
        material=armor,
        name="launcher_body",
    )
    missile_pod.visual(
        Box((0.16, 0.22, 0.04)),
        origin=Origin(xyz=(0.12, -0.08, 0.12)),
        material=armor,
        name="top_fairing",
    )
    missile_pod.visual(
        Box((0.17, 0.03, 0.18)),
        origin=Origin(xyz=(0.12, 0.245, 0.0)),
        material=launcher_front,
        name="front_panel",
    )
    for idx, (x_pos, z_pos) in enumerate(((0.08, 0.05), (0.16, 0.05), (0.08, -0.05), (0.16, -0.05)), start=1):
        missile_pod.visual(
            Cylinder(radius=0.036, length=0.025),
            origin=Origin(xyz=(x_pos, 0.255, z_pos), rpy=cyl_y),
            material=dark_armor,
            name=f"tube_{idx}",
        )

    optic_bracket = model.part("optic_bracket")
    optic_bracket.visual(
        Box((0.03, 0.08, 0.12)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=dark_armor,
        name="base_block",
    )
    optic_bracket.visual(
        Box((0.066, 0.04, 0.04)),
        origin=Origin(xyz=(-0.051, 0.011, -0.02)),
        material=dark_armor,
        name="support_arm",
    )
    optic_bracket.visual(
        Box((0.012, 0.07, 0.168)),
        origin=Origin(xyz=(-0.084, 0.01, 0.0)),
        material=armor,
        name="inner_cheek",
    )
    optic_bracket.visual(
        Box((0.012, 0.07, 0.168)),
        origin=Origin(xyz=(-0.156, 0.01, 0.0)),
        material=armor,
        name="outer_cheek",
    )
    optic_bracket.visual(
        Box((0.012, 0.028, 0.018)),
        origin=Origin(xyz=(-0.084, 0.01, 0.054)),
        material=armor,
        name="inner_retainer",
    )
    optic_bracket.visual(
        Box((0.012, 0.028, 0.018)),
        origin=Origin(xyz=(-0.156, 0.01, 0.054)),
        material=armor,
        name="outer_retainer",
    )
    optic_bracket.visual(
        Box((0.084, 0.022, 0.008)),
        origin=Origin(xyz=(-0.120, 0.01, 0.088)),
        material=armor,
        name="top_bridge",
    )

    optic_pod = model.part("optic_pod")
    optic_pod.visual(
        Box((0.05, 0.11, 0.09)),
        origin=Origin(xyz=(0.0, 0.01, 0.0)),
        material=dark_armor,
        name="sensor_body",
    )
    optic_pod.visual(
        Box((0.056, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -0.01, 0.05)),
        material=armor,
        name="hood",
    )
    optic_pod.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=cyl_x),
        material=dark_armor,
        name="left_trunnion",
    )
    optic_pod.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=cyl_x),
        material=dark_armor,
        name="right_trunnion",
    )
    optic_pod.visual(
        Cylinder(radius=0.024, length=0.045),
        origin=Origin(xyz=(0.0, 0.085, 0.006), rpy=cyl_y),
        material=optics_glass,
        name="main_lens",
    )
    optic_pod.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.085, -0.028), rpy=cyl_y),
        material=optics_glass,
        name="aux_lens",
    )

    model.articulation(
        "pedestal_azimuth",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=azimuth_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "cradle_elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_frame,
        child=weapon_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-0.15,
            upper=0.80,
        ),
    )
    model.articulation(
        "frame_to_missile_pod",
        ArticulationType.FIXED,
        parent=azimuth_frame,
        child=missile_pod,
        origin=Origin(xyz=(0.185, 0.03, 0.22)),
    )
    model.articulation(
        "frame_to_optic_bracket",
        ArticulationType.FIXED,
        parent=azimuth_frame,
        child=optic_bracket,
        origin=Origin(xyz=(-0.185, 0.08, 0.19)),
    )
    model.articulation(
        "optic_tilt",
        ArticulationType.REVOLUTE,
        parent=optic_bracket,
        child=optic_pod,
        origin=Origin(xyz=(-0.120, 0.01, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_base")
    azimuth_frame = object_model.get_part("azimuth_frame")
    weapon_cradle = object_model.get_part("weapon_cradle")
    missile_pod = object_model.get_part("missile_pod")
    optic_bracket = object_model.get_part("optic_bracket")
    optic_pod = object_model.get_part("optic_pod")

    pedestal_azimuth = object_model.get_articulation("pedestal_azimuth")
    cradle_elevation = object_model.get_articulation("cradle_elevation")
    optic_tilt = object_model.get_articulation("optic_tilt")

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
    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "all_key_parts_present",
        all(part is not None for part in (pedestal, azimuth_frame, weapon_cradle, missile_pod, optic_bracket, optic_pod)),
        "Missing one or more major station parts.",
    )
    ctx.check(
        "azimuth_axis_vertical",
        tuple(round(v, 3) for v in pedestal_azimuth.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical azimuth axis, got {pedestal_azimuth.axis}.",
    )
    ctx.check(
        "cradle_axis_horizontal",
        tuple(round(v, 3) for v in cradle_elevation.axis) == (1.0, 0.0, 0.0),
        f"Expected horizontal cradle axis, got {cradle_elevation.axis}.",
    )
    ctx.check(
        "optic_tilt_axis_horizontal",
        tuple(round(v, 3) for v in optic_tilt.axis) == (1.0, 0.0, 0.0),
        f"Expected horizontal optic tilt axis, got {optic_tilt.axis}.",
    )

    ctx.expect_contact(
        azimuth_frame,
        pedestal,
        elem_a="azimuth_bearing",
        elem_b="turntable_ring",
        name="frame_seated_on_turntable",
    )
    ctx.expect_contact(
        weapon_cradle,
        azimuth_frame,
        elem_a="left_trunnion",
        elem_b="left_support",
        name="left_trunnion_supported",
    )
    ctx.expect_contact(
        weapon_cradle,
        azimuth_frame,
        elem_a="right_trunnion",
        elem_b="right_support",
        name="right_trunnion_supported",
    )
    ctx.expect_contact(
        missile_pod,
        azimuth_frame,
        elem_a="mount_block",
        elem_b="right_support",
        name="missile_pod_mounted",
    )
    ctx.expect_contact(
        optic_bracket,
        azimuth_frame,
        elem_a="base_block",
        elem_b="left_support",
        name="optic_bracket_mounted",
    )

    ctx.expect_origin_gap(
        missile_pod,
        weapon_cradle,
        axis="x",
        min_gap=0.16,
        name="missile_pod_on_positive_side",
    )
    ctx.expect_origin_gap(
        weapon_cradle,
        optic_pod,
        axis="x",
        min_gap=0.25,
        name="optic_pod_on_negative_side",
    )

    with ctx.pose({optic_tilt: 0.45, cradle_elevation: 0.35}):
        ctx.expect_gap(
            optic_bracket,
            optic_pod,
            axis="x",
            positive_elem="inner_cheek",
            negative_elem="right_trunnion",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="optic_inner_clip_clearance",
        )
        ctx.expect_gap(
            optic_pod,
            optic_bracket,
            axis="x",
            positive_elem="left_trunnion",
            negative_elem="outer_cheek",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="optic_outer_clip_clearance",
        )
        ctx.expect_within(
            optic_pod,
            optic_bracket,
            axes="x",
            margin=0.01,
            name="optic_pod_captured_between_cheeks",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
