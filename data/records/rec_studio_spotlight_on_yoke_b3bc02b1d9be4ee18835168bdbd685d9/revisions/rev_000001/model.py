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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_studio_spotlight")

    base_metal = model.material("base_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    yoke_metal = model.material("yoke_metal", rgba=(0.15, 0.16, 0.17, 1.0))
    shell_paint = model.material("shell_paint", rgba=(0.24, 0.25, 0.27, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.48, 0.50, 0.53, 1.0))
    friction_knob = model.material("friction_knob", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.80, 0.84, 0.88, 0.38))

    base = model.part("base")
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.170, 0.118, 0.016), 0.014),
        "desktop_spotlight_base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_metal,
        name="base_shell",
    )
    base.visual(
        Box((0.074, 0.080, 0.020)),
        origin=Origin(xyz=(-0.028, 0.0, 0.024)),
        material=base_metal,
        name="rear_ballast",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(-0.002, 0.0, 0.028)),
        material=trim_metal,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(-0.002, 0.0, 0.045)),
        material=trim_metal,
        name="pan_seat",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.170, 0.118, 0.050)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_metal,
        name="swivel_disc",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=yoke_metal,
        name="swivel_collar",
    )
    yoke.visual(
        Box((0.042, 0.090, 0.012)),
        origin=Origin(xyz=(0.004, 0.0, 0.036)),
        material=yoke_metal,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.010, 0.006, 0.074)),
        origin=Origin(xyz=(0.010, 0.046, 0.079)),
        material=yoke_metal,
        name="left_arm",
    )
    yoke.visual(
        Box((0.010, 0.006, 0.074)),
        origin=Origin(xyz=(0.010, -0.046, 0.079)),
        material=yoke_metal,
        name="right_arm",
    )
    yoke.visual(
        Box((0.016, 0.004, 0.026)),
        origin=Origin(xyz=(0.002, 0.043, 0.108)),
        material=yoke_metal,
        name="left_trunnion_web",
    )
    yoke.visual(
        Box((0.016, 0.004, 0.026)),
        origin=Origin(xyz=(0.002, -0.043, 0.108)),
        material=yoke_metal,
        name="right_trunnion_web",
    )
    yoke.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.014, 0.042, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="left_inner_friction",
    )
    yoke.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.014, -0.042, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="right_inner_friction",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.014, 0.051, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=friction_knob,
        name="left_lock_knob",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.014, -0.051, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=friction_knob,
        name="right_lock_knob",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.060, 0.110, 0.120)),
        mass=0.65,
        origin=Origin(xyz=(0.010, 0.0, 0.060)),
    )

    head = model.part("head")
    can_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.024, -0.026),
                (0.030, -0.021),
                (0.035, -0.010),
                (0.038, 0.012),
                (0.039, 0.038),
                (0.040, 0.058),
                (0.041, 0.070),
            ],
            inner_profile=[
                (0.010, -0.022),
                (0.026, -0.018),
                (0.031, -0.009),
                (0.034, 0.012),
                (0.035, 0.038),
                (0.036, 0.056),
                (0.037, 0.064),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "desktop_spotlight_can_shell",
    )
    head.visual(
        can_shell_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_paint,
        name="can_shell",
    )
    head.visual(
        Cylinder(radius=0.037, length=0.014),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_metal,
        name="rear_heat_sink",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_metal,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_metal,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.038, length=0.002),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.014, 0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="left_trunnion_boss",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.014, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="right_trunnion_boss",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.014, 0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.014, -0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.118, 0.082, 0.082)),
        mass=1.2,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(-0.002, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.014, 0.0, 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-1.05,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    pan = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")

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
        yoke,
        base,
        elem_a="swivel_disc",
        elem_b="pan_seat",
        name="yoke_turntable_seated",
    )
    ctx.expect_contact(
        head,
        yoke,
        elem_a="left_trunnion",
        elem_b="left_inner_friction",
        name="left_trunnion_captured",
    )
    ctx.expect_contact(
        head,
        yoke,
        elem_a="right_trunnion",
        elem_b="right_inner_friction",
        name="right_trunnion_captured",
    )
    ctx.expect_gap(
        yoke,
        head,
        axis="y",
        positive_elem="left_arm",
        negative_elem="can_shell",
        min_gap=0.0015,
        max_gap=0.006,
        name="left_yoke_clearance",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="y",
        positive_elem="can_shell",
        negative_elem="right_arm",
        min_gap=0.0015,
        max_gap=0.006,
        name="right_yoke_clearance",
    )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.015,
            max_gap=0.030,
            name="stowed_head_compact_clearance",
        )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        neutral_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({pan: 0.0, tilt: tilt.motion_limits.lower}):
        stowed_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({pan: 0.0, tilt: 0.95}):
        raised_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "stowed_lens_drops_compactly",
        stowed_lens[2] < neutral_lens[2] - 0.045,
        details=f"neutral_z={neutral_lens[2]:.4f}, stowed_z={stowed_lens[2]:.4f}",
    )
    ctx.check(
        "tilt_raises_beam_center",
        raised_lens[2] > neutral_lens[2] + 0.045,
        details=f"neutral_z={neutral_lens[2]:.4f}, raised_z={raised_lens[2]:.4f}",
    )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        centered_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({pan: 1.2, tilt: 0.0}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "pan_swings_head_sideways",
        abs(panned_lens[1] - centered_lens[1]) > 0.050,
        details=f"center_y={centered_lens[1]:.4f}, panned_y={panned_lens[1]:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
