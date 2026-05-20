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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="towed_aa_missile_launcher")

    olive = model.material("olive_drab", rgba=(0.36, 0.42, 0.27, 1.0))
    steel = model.material("steel_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    dark = model.material("dark_metal", rgba=(0.24, 0.26, 0.28, 1.0))
    rubber = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    optics = model.material("optics_glass", rgba=(0.16, 0.24, 0.30, 1.0))

    base = model.part("trailer_base")
    base.visual(
        Box((1.40, 1.05, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=olive,
        name="platform",
    )
    base.visual(
        Box((1.15, 0.08, 0.08)),
        origin=Origin(xyz=(0.92, 0.20, 0.21), rpy=(0.0, 0.0, -0.16)),
        material=olive,
        name="drawbar_beam_pos_y",
    )
    base.visual(
        Box((1.15, 0.08, 0.08)),
        origin=Origin(xyz=(0.92, -0.20, 0.21), rpy=(0.0, 0.0, 0.16)),
        material=olive,
        name="drawbar_beam_neg_y",
    )
    base.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(1.49, 0.0, 0.18)),
        material=olive,
        name="hitch_head",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(1.62, 0.0, 0.18), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="tow_eye",
    )
    base.visual(
        Box((0.32, 1.46, 0.12)),
        origin=Origin(xyz=(-0.10, 0.0, 0.26)),
        material=dark,
        name="axle_beam",
    )
    base.visual(
        Cylinder(radius=0.31, length=0.14),
        origin=Origin(xyz=(-0.10, 0.80, 0.31), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_pos_y",
    )
    base.visual(
        Cylinder(radius=0.31, length=0.14),
        origin=Origin(xyz=(-0.10, -0.80, 0.31), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_neg_y",
    )
    base.visual(
        Box((0.26, 1.05, 0.10)),
        origin=Origin(xyz=(-0.64, 0.0, 0.18)),
        material=olive,
        name="rear_crossmember",
    )
    base.visual(
        Box((0.08, 0.08, 0.24)),
        origin=Origin(xyz=(-0.66, 0.34, 0.12)),
        material=dark,
        name="stabilizer_leg_pos_y",
    )
    base.visual(
        Box((0.08, 0.08, 0.24)),
        origin=Origin(xyz=(-0.66, -0.34, 0.12)),
        material=dark,
        name="stabilizer_leg_neg_y",
    )
    base.visual(
        Box((0.16, 0.18, 0.04)),
        origin=Origin(xyz=(-0.66, 0.34, 0.02)),
        material=steel,
        name="stabilizer_pad_pos_y",
    )
    base.visual(
        Box((0.16, 0.18, 0.04)),
        origin=Origin(xyz=(-0.66, -0.34, 0.02)),
        material=steel,
        name="stabilizer_pad_neg_y",
    )
    base.inertial = Inertial.from_geometry(
        Box((2.45, 1.74, 0.70)),
        mass=950.0,
        origin=Origin(xyz=(0.38, 0.0, 0.35)),
    )

    turret = model.part("slew_cradle")
    turret.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="slew_ring",
    )
    turret.visual(
        Cylinder(radius=0.18, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=olive,
        name="pedestal",
    )
    turret.visual(
        Box((0.28, 0.12, 0.18)),
        origin=Origin(xyz=(0.02, 0.22, 0.31)),
        material=olive,
        name="shoulder_pos_y",
    )
    turret.visual(
        Box((0.28, 0.12, 0.18)),
        origin=Origin(xyz=(0.02, -0.22, 0.31)),
        material=olive,
        name="shoulder_neg_y",
    )
    turret.visual(
        Box((0.18, 0.10, 0.60)),
        origin=Origin(xyz=(0.02, 0.33, 0.52)),
        material=olive,
        name="arm_pos_y",
    )
    turret.visual(
        Box((0.18, 0.10, 0.60)),
        origin=Origin(xyz=(0.02, -0.33, 0.52)),
        material=olive,
        name="arm_neg_y",
    )
    turret.visual(
        Box((0.26, 0.10, 0.16)),
        origin=Origin(xyz=(0.12, 0.33, 0.74)),
        material=dark,
        name="arm_block_pos_y",
    )
    turret.visual(
        Box((0.26, 0.10, 0.16)),
        origin=Origin(xyz=(0.12, -0.33, 0.74)),
        material=dark,
        name="arm_block_neg_y",
    )
    turret.visual(
        Cylinder(radius=0.045, length=0.58),
        origin=Origin(xyz=(-0.06, 0.0, 0.62), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crossbrace",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.86)),
        mass=320.0,
        origin=Origin(xyz=(0.02, 0.0, 0.43)),
    )

    pack = model.part("launch_pack")
    pack.visual(
        Box((0.30, 0.24, 0.14)),
        origin=Origin(xyz=(-0.12, 0.0, 0.04)),
        material=dark,
        name="backbone",
    )
    pack.visual(
        Box((0.12, 0.11, 0.10)),
        origin=Origin(xyz=(-0.02, 0.165, 0.02)),
        material=dark,
        name="trunnion_web_pos_y",
    )
    pack.visual(
        Box((0.12, 0.11, 0.10)),
        origin=Origin(xyz=(-0.02, -0.165, 0.02)),
        material=dark,
        name="trunnion_web_neg_y",
    )
    pack.visual(
        Box((0.10, 0.10, 0.12)),
        origin=Origin(xyz=(0.02, 0.22, 0.02)),
        material=steel,
        name="pivot_lug_pos_y",
    )
    pack.visual(
        Box((0.10, 0.10, 0.12)),
        origin=Origin(xyz=(0.02, -0.22, 0.02)),
        material=steel,
        name="pivot_lug_neg_y",
    )
    pack.visual(
        Cylinder(radius=0.04, length=0.56),
        origin=Origin(xyz=(0.00, 0.0, 0.02), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )
    pack.visual(
        Box((0.70, 0.32, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.08)),
        material=olive,
        name="saddle",
    )
    pack.visual(
        Box((0.34, 0.045, 0.15)),
        origin=Origin(xyz=(0.18, 0.14, 0.03)),
        material=dark,
        name="stiffener_pos_y",
    )
    pack.visual(
        Box((0.34, 0.045, 0.15)),
        origin=Origin(xyz=(0.18, -0.14, 0.03)),
        material=dark,
        name="stiffener_neg_y",
    )
    pack.visual(
        Box((1.55, 0.055, 0.045)),
        origin=Origin(xyz=(0.76, 0.14, 0.12)),
        material=steel,
        name="rail_pos_y",
    )
    pack.visual(
        Box((1.55, 0.055, 0.045)),
        origin=Origin(xyz=(0.76, -0.14, 0.12)),
        material=steel,
        name="rail_neg_y",
    )
    pack.visual(
        Box((0.08, 0.055, 0.13)),
        origin=Origin(xyz=(1.54, 0.14, 0.16)),
        material=steel,
        name="front_stop_pos_y",
    )
    pack.visual(
        Box((0.08, 0.055, 0.13)),
        origin=Origin(xyz=(1.54, -0.14, 0.16)),
        material=steel,
        name="front_stop_neg_y",
    )
    pack.inertial = Inertial.from_geometry(
        Box((1.75, 0.42, 0.24)),
        mass=220.0,
        origin=Origin(xyz=(0.72, 0.0, 0.11)),
    )

    sight = model.part("sight_module")
    sight.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.00, -0.05, 0.05)),
        material=dark,
        name="mount",
    )
    sight.visual(
        Box((0.28, 0.18, 0.18)),
        origin=Origin(xyz=(0.03, -0.13, 0.14)),
        material=olive,
        name="housing",
    )
    sight.visual(
        Cylinder(radius=0.038, length=0.18),
        origin=Origin(xyz=(0.16, -0.13, 0.14), rpy=(0.0, pi / 2.0, 0.0)),
        material=optics,
        name="optic_barrel",
    )
    sight.visual(
        Cylinder(radius=0.026, length=0.10),
        origin=Origin(xyz=(-0.13, -0.13, 0.13), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="eyepiece",
    )
    sight.inertial = Inertial.from_geometry(
        Box((0.34, 0.20, 0.22)),
        mass=35.0,
        origin=Origin(xyz=(0.03, -0.12, 0.13)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=0.8,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "pack_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=pack,
        origin=Origin(xyz=(0.12, 0.0, 0.72)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.7,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "pack_to_sight",
        ArticulationType.FIXED,
        parent=pack,
        child=sight,
        origin=Origin(xyz=(-0.075, -0.17, 0.08)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("trailer_base")
    turret = object_model.get_part("slew_cradle")
    pack = object_model.get_part("launch_pack")
    sight = object_model.get_part("sight_module")
    yaw = object_model.get_articulation("base_yaw")
    pitch = object_model.get_articulation("pack_pitch")

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
        "primary_parts_present",
        all(part is not None for part in (base, turret, pack, sight)),
        details="Launcher must include the trailer base, slew cradle, launch pack, and sight module.",
    )
    ctx.check(
        "yaw_axis_is_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical slew axis, got {yaw.axis!r}.",
    )
    ctx.check(
        "pitch_axis_is_transverse",
        abs(pitch.axis[0]) < 1e-9 and abs(abs(pitch.axis[1]) - 1.0) < 1e-9 and abs(pitch.axis[2]) < 1e-9,
        details=f"Expected transverse elevation axis, got {pitch.axis!r}.",
    )
    ctx.check(
        "pitch_limit_reads_as_anti_air_mount",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower >= -0.05
        and pitch.motion_limits.upper >= 1.2,
        details="Elevation cradle should start near level and raise well above the horizon.",
    )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        ctx.expect_gap(
            turret,
            base,
            axis="z",
            positive_elem="slew_ring",
            negative_elem="platform",
            max_gap=0.002,
            max_penetration=0.0,
            name="slew_ring_seated_on_base",
        )
        ctx.expect_gap(
            turret,
            pack,
            axis="y",
            positive_elem="arm_block_pos_y",
            negative_elem="pivot_lug_pos_y",
            min_gap=0.008,
            max_gap=0.020,
            name="positive_side_trunnion_clearance",
        )
        ctx.expect_gap(
            pack,
            turret,
            axis="y",
            positive_elem="pivot_lug_neg_y",
            negative_elem="arm_block_neg_y",
            min_gap=0.008,
            max_gap=0.020,
            name="negative_side_trunnion_clearance",
        )
        ctx.expect_contact(
            pack,
            turret,
            elem_a="trunnion_shaft",
            elem_b="arm_block_pos_y",
            name="positive_trunnion_bearing_contact",
        )
        ctx.expect_contact(
            pack,
            turret,
            elem_a="trunnion_shaft",
            elem_b="arm_block_neg_y",
            name="negative_trunnion_bearing_contact",
        )
        ctx.expect_contact(
            sight,
            pack,
            elem_a="mount",
            elem_b="pivot_lug_neg_y",
            name="sight_mount_contacts_pack_hardpoint",
        )
        ctx.expect_gap(
            pack,
            sight,
            axis="y",
            positive_elem="rail_neg_y",
            negative_elem="housing",
            min_gap=0.020,
            max_gap=0.080,
            name="sight_sits_beside_rail_pack",
        )

    with ctx.pose({pitch: 0.85}):
        ctx.expect_gap(
            pack,
            turret,
            axis="z",
            positive_elem="rail_pos_y",
            negative_elem="slew_ring",
            min_gap=0.22,
            name="elevated_rails_clear_slew_ring",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
