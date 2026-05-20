from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_mesh(name: str, *, inner_radius: float, outer_radius: float, length: float):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -0.5 * length),
                (outer_radius, 0.5 * length),
            ],
            [
                (inner_radius, -0.5 * length),
                (inner_radius, 0.5 * length),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_on_yoke", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.64, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.48, 0.58, 0.64, 0.35))
    soft_rubber = model.material("soft_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    yoke_arm_mesh = _save_mesh(
        "yoke_arm.obj",
        sweep_profile_along_spline(
            [
                (-0.032, 0.0, 0.018),
                (-0.028, 0.0, 0.052),
                (-0.015, 0.0, 0.096),
                (0.000, 0.0, 0.135),
            ],
            profile=rounded_rect_profile(0.018, 0.012, radius=0.0032, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )
    pan_bearing_mesh = _ring_mesh(
        "pan_bearing_sleeve.obj",
        inner_radius=0.0165,
        outer_radius=0.0260,
        length=0.0260,
    )
    pan_thrust_mesh = _ring_mesh(
        "pan_thrust_ring.obj",
        inner_radius=0.0180,
        outer_radius=0.0600,
        length=0.0060,
    )
    trunnion_boss_mesh = _ring_mesh(
        "trunnion_boss.obj",
        inner_radius=0.0105,
        outer_radius=0.0220,
        length=0.0160,
    )
    trunnion_trim_mesh = _ring_mesh(
        "trunnion_trim_ring.obj",
        inner_radius=0.0105,
        outer_radius=0.0185,
        length=0.0080,
    )
    bezel_ring_mesh = _ring_mesh(
        "front_bezel_ring.obj",
        inner_radius=0.0560,
        outer_radius=0.0730,
        length=0.0120,
    )
    can_shell_mesh = _save_mesh(
        "spotlight_can_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0, -0.076),
                (0.030, -0.074),
                (0.043, -0.070),
                (0.048, -0.058),
                (0.056, -0.018),
                (0.063, 0.030),
                (0.066, 0.072),
                (0.068, 0.096),
            ],
            [
                (0.0, -0.064),
                (0.024, -0.061),
                (0.037, -0.056),
                (0.050, -0.015),
                (0.057, 0.034),
                (0.059, 0.086),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    seam_ring_mesh = _ring_mesh(
        "seam_ring.obj",
        inner_radius=0.0520,
        outer_radius=0.0585,
        length=0.0040,
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.135, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=matte_graphite,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=soft_rubber,
        name="foot_pad",
    )
    base.visual(
        Cylinder(radius=0.111, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=satin_aluminum,
        name="base_trim",
    )
    base.visual(
        Cylinder(radius=0.023, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=matte_graphite,
        name="stem",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=satin_aluminum,
        name="stem_break_ring",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=dark_steel,
        name="upper_collar",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=satin_aluminum,
        name="thrust_plate",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=dark_steel,
        name="pan_spindle",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.118),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    yoke_frame = model.part("yoke_frame")
    yoke_frame.visual(
        pan_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="pan_bearing_sleeve",
    )
    yoke_frame.visual(
        pan_thrust_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_aluminum,
        name="pan_thrust_ring",
    )
    yoke_frame.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=matte_graphite,
        name="pan_hub",
    )
    yoke_frame.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=satin_aluminum,
        name="hub_cap",
    )
    yoke_frame.visual(
        yoke_arm_mesh,
        origin=Origin(xyz=(-0.020, 0.069, 0.0)),
        material=matte_graphite,
        name="left_yoke_arm",
    )
    yoke_frame.visual(
        yoke_arm_mesh,
        origin=Origin(xyz=(-0.020, -0.069, 0.0)),
        material=matte_graphite,
        name="right_yoke_arm",
    )
    yoke_frame.visual(
        Box((0.060, 0.132, 0.012)),
        origin=Origin(xyz=(-0.038, 0.0, 0.024)),
        material=matte_graphite,
        name="lower_bridge",
    )
    yoke_frame.visual(
        Cylinder(radius=0.008, length=0.132),
        origin=Origin(xyz=(-0.060, 0.0, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_tie_bar",
    )
    yoke_frame.visual(
        Box((0.022, 0.016, 0.034)),
        origin=Origin(xyz=(-0.060, 0.061, 0.042)),
        material=dark_steel,
        name="left_rear_gusset",
    )
    yoke_frame.visual(
        Box((0.022, 0.016, 0.034)),
        origin=Origin(xyz=(-0.060, -0.061, 0.042)),
        material=dark_steel,
        name="right_rear_gusset",
    )
    yoke_frame.visual(
        trunnion_boss_mesh,
        origin=Origin(xyz=(0.0, 0.069, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_boss",
    )
    yoke_frame.visual(
        trunnion_boss_mesh,
        origin=Origin(xyz=(0.0, -0.069, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_boss",
    )
    yoke_frame.visual(
        trunnion_trim_mesh,
        origin=Origin(xyz=(0.0, 0.081, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="left_trunnion_trim",
    )
    yoke_frame.visual(
        trunnion_trim_mesh,
        origin=Origin(xyz=(0.0, -0.081, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="right_trunnion_trim",
    )
    yoke_frame.inertial = Inertial.from_geometry(
        Box((0.120, 0.170, 0.170)),
        mass=2.4,
        origin=Origin(xyz=(-0.005, 0.0, 0.080)),
    )

    spotlight_can = model.part("spotlight_can")
    spotlight_can.visual(
        can_shell_mesh,
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="can_shell",
    )
    spotlight_can.visual(
        bezel_ring_mesh,
        origin=Origin(xyz=(0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="front_bezel",
    )
    spotlight_can.visual(
        Cylinder(radius=0.058, length=0.026),
        origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="inner_baffle",
    )
    spotlight_can.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_glass,
        name="front_lens",
    )
    spotlight_can.visual(
        seam_ring_mesh,
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="rear_seam_ring",
    )
    spotlight_can.visual(
        seam_ring_mesh,
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="focus_seam_ring",
    )
    spotlight_can.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(-0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_driver_cap",
    )
    spotlight_can.visual(
        Cylinder(radius=0.020, length=0.106),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_barrel",
    )
    spotlight_can.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="left_trunnion_shoulder",
    )
    spotlight_can.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, -0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="right_trunnion_shoulder",
    )
    spotlight_can.visual(
        Cylinder(radius=0.0095, length=0.030),
        origin=Origin(xyz=(0.0, 0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_stub",
    )
    spotlight_can.visual(
        Cylinder(radius=0.0095, length=0.030),
        origin=Origin(xyz=(0.0, -0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_stub",
    )
    spotlight_can.inertial = Inertial.from_geometry(
        Box((0.190, 0.140, 0.140)),
        mass=2.1,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "pan_stage",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.8),
    )
    model.articulation(
        "tilt_stage",
        ArticulationType.REVOLUTE,
        parent=yoke_frame,
        child=spotlight_can,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.55,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    yoke_frame = object_model.get_part("yoke_frame")
    spotlight_can = object_model.get_part("spotlight_can")

    pan_stage = object_model.get_articulation("pan_stage")
    tilt_stage = object_model.get_articulation("tilt_stage")

    thrust_plate = base.get_visual("thrust_plate")
    pan_spindle = base.get_visual("pan_spindle")

    pan_bearing_sleeve = yoke_frame.get_visual("pan_bearing_sleeve")
    pan_thrust_ring = yoke_frame.get_visual("pan_thrust_ring")
    left_trunnion_boss = yoke_frame.get_visual("left_trunnion_boss")
    right_trunnion_boss = yoke_frame.get_visual("right_trunnion_boss")

    front_bezel = spotlight_can.get_visual("front_bezel")
    front_lens = spotlight_can.get_visual("front_lens")
    left_trunnion_shoulder = spotlight_can.get_visual("left_trunnion_shoulder")
    right_trunnion_shoulder = spotlight_can.get_visual("right_trunnion_shoulder")
    left_trunnion_stub = spotlight_can.get_visual("left_trunnion_stub")
    right_trunnion_stub = spotlight_can.get_visual("right_trunnion_stub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "spotlight_parts_and_joints_present",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details="Expected base, yoke frame, spotlight can, plus pan and tilt stages.",
    )

    ctx.expect_contact(
        yoke_frame,
        base,
        elem_a=pan_thrust_ring,
        elem_b=thrust_plate,
        name="pan_stage_has_axial_support_contact",
    )
    ctx.expect_within(
        base,
        yoke_frame,
        axes="xy",
        inner_elem=pan_spindle,
        outer_elem=pan_bearing_sleeve,
        margin=0.002,
        name="pan_spindle_runs_inside_bearing_sleeve",
    )
    ctx.expect_contact(
        spotlight_can,
        yoke_frame,
        elem_a=left_trunnion_shoulder,
        elem_b=left_trunnion_boss,
        name="left_trunnion_seats_on_boss",
    )
    ctx.expect_contact(
        spotlight_can,
        yoke_frame,
        elem_a=right_trunnion_shoulder,
        elem_b=right_trunnion_boss,
        name="right_trunnion_seats_on_boss",
    )
    ctx.expect_within(
        spotlight_can,
        yoke_frame,
        axes="xz",
        inner_elem=left_trunnion_stub,
        outer_elem=left_trunnion_boss,
        margin=0.002,
        name="left_stub_stays_centered_in_boss",
    )
    ctx.expect_within(
        spotlight_can,
        yoke_frame,
        axes="xz",
        inner_elem=right_trunnion_stub,
        outer_elem=right_trunnion_boss,
        margin=0.002,
        name="right_stub_stays_centered_in_boss",
    )
    ctx.expect_within(
        spotlight_can,
        spotlight_can,
        axes="yz",
        inner_elem=front_lens,
        outer_elem=front_bezel,
        margin=0.001,
        name="lens_is_captured_by_front_bezel",
    )
    ctx.expect_gap(
        spotlight_can,
        base,
        axis="z",
        min_gap=0.054,
        name="spotlight_head_clears_base_at_rest",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    rest_bezel_aabb = ctx.part_element_world_aabb(spotlight_can, elem=front_bezel)
    assert rest_bezel_aabb is not None
    rest_bezel_center = _aabb_center(rest_bezel_aabb)

    with ctx.pose({pan_stage: math.pi / 2.0}):
        pan_bezel_aabb = ctx.part_element_world_aabb(spotlight_can, elem=front_bezel)
        assert pan_bezel_aabb is not None
        pan_bezel_center = _aabb_center(pan_bezel_aabb)
        ctx.expect_contact(yoke_frame, base, elem_a=pan_thrust_ring, elem_b=thrust_plate)
        ctx.check(
            "pan_stage_rotates_head_around_vertical_axis",
            abs(pan_bezel_center[0]) < 0.01 and pan_bezel_center[1] > rest_bezel_center[0] - 0.01,
            details="A 90-degree pan should swing the bezel center from the +X aim direction to the +Y side.",
        )

    with ctx.pose({tilt_stage: -0.35}):
        raised_bezel_aabb = ctx.part_element_world_aabb(spotlight_can, elem=front_bezel)
        assert raised_bezel_aabb is not None
        raised_bezel_center = _aabb_center(raised_bezel_aabb)
        ctx.expect_contact(spotlight_can, yoke_frame, elem_a=left_trunnion_shoulder, elem_b=left_trunnion_boss)
        ctx.expect_contact(spotlight_can, yoke_frame, elem_a=right_trunnion_shoulder, elem_b=right_trunnion_boss)
        ctx.check(
            "tilt_stage_raises_bezel_when_aimed_up",
            raised_bezel_center[2] > rest_bezel_center[2] + 0.03,
            details="Negative tilt on this yoke should lift the bezel center measurably above its rest height.",
        )

    with ctx.pose({tilt_stage: 0.35}):
        ctx.expect_gap(
            spotlight_can,
            base,
            axis="z",
            min_gap=0.010,
            name="down_tilt_still_clears_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
