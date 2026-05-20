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
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_premium", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.21, 0.23, 0.25, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.63, 0.66, 0.69, 1.0))
    soft_black = model.material("soft_black", rgba=(0.06, 0.07, 0.08, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.80, 0.88, 0.42))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    can_shell_mesh = _save_mesh(
        "spotlight_can_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.028, -0.190),
                (0.050, -0.182),
                (0.072, -0.160),
                (0.082, -0.125),
                (0.090, -0.060),
                (0.095, 0.040),
                (0.098, 0.108),
                (0.104, 0.148),
                (0.110, 0.165),
            ],
            [
                (0.000, -0.176),
                (0.040, -0.168),
                (0.066, -0.146),
                (0.074, -0.120),
                (0.082, -0.055),
                (0.086, 0.040),
                (0.089, 0.108),
                (0.094, 0.148),
                (0.097, 0.165),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    bezel_mesh = _save_mesh(
        "spotlight_bezel.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.101, 0.148),
                (0.107, 0.156),
                (0.113, 0.170),
                (0.114, 0.182),
            ],
            [
                (0.090, 0.148),
                (0.094, 0.158),
                (0.097, 0.170),
                (0.098, 0.182),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
    )

    floor_base = model.part("floor_base")
    floor_base.visual(
        Cylinder(radius=0.180, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=matte_black,
        name="base_plate",
    )
    floor_base.visual(
        Cylinder(radius=0.154, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=satin_graphite,
        name="base_trim",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0) + (math.pi / 6.0)
        floor_base.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(
                xyz=(0.118 * math.cos(angle), 0.118 * math.sin(angle), 0.004),
            ),
            material=rubber,
            name=f"foot_pad_{index:02d}",
        )
    floor_base.visual(
        Cylinder(radius=0.050, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        material=satin_graphite,
        name="column_shroud",
    )
    floor_base.visual(
        Box((0.050, 0.018, 0.090)),
        origin=Origin(xyz=(-0.020, 0.0, 0.094)),
        material=soft_black,
        name="rear_cable_cover",
    )
    floor_base.visual(
        Cylinder(radius=0.082, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        material=satin_graphite,
        name="bearing_stator",
    )
    floor_base.visual(
        Cylinder(radius=0.100, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=satin_aluminum,
        name="bearing_top_flange",
    )
    floor_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.306),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.095, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_aluminum,
        name="pan_rotor",
    )
    pan_yoke.visual(
        Cylinder(radius=0.062, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=satin_graphite,
        name="pan_hub",
    )
    pan_yoke.visual(
        Box((0.120, 0.160, 0.070)),
        origin=Origin(xyz=(-0.100, 0.0, 0.073)),
        material=satin_graphite,
        name="backbone_block",
    )
    pan_yoke.visual(
        Box((0.060, 0.090, 0.180)),
        origin=Origin(xyz=(-0.165, 0.0, 0.160)),
        material=satin_graphite,
        name="yoke_spine",
    )
    pan_yoke.visual(
        Box((0.040, 0.228, 0.028)),
        origin=Origin(xyz=(-0.145, 0.0, 0.190)),
        material=satin_graphite,
        name="lower_bridge",
    )
    pan_yoke.visual(
        Box((0.018, 0.024, 0.110)),
        origin=Origin(xyz=(-0.123, 0.126, 0.259)),
        material=satin_graphite,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.018, 0.024, 0.110)),
        origin=Origin(xyz=(-0.123, -0.126, 0.259)),
        material=satin_graphite,
        name="right_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.062, 0.024, 0.036)),
        origin=Origin(xyz=(-0.083, 0.126, 0.280)),
        material=satin_graphite,
        name="left_collar_web",
    )
    pan_yoke.visual(
        Box((0.062, 0.024, 0.036)),
        origin=Origin(xyz=(-0.083, -0.126, 0.280)),
        material=satin_graphite,
        name="right_collar_web",
    )
    pan_yoke.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(
            xyz=(-0.006, 0.122, 0.280),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="left_bearing_collar",
    )
    pan_yoke.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(
            xyz=(-0.006, -0.122, 0.280),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="right_bearing_collar",
    )
    pan_yoke.visual(
        Box((0.048, 0.064, 0.050)),
        origin=Origin(xyz=(-0.192, 0.0, 0.086)),
        material=soft_black,
        name="pan_drive_pod",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(
            xyz=(-0.218, 0.0, 0.086),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="pan_drive_cap",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.280, 0.300, 0.360)),
        mass=7.0,
        origin=Origin(xyz=(-0.090, 0.0, 0.180)),
    )

    spotlight_can = model.part("spotlight_can")
    spotlight_can.visual(
        can_shell_mesh,
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_graphite,
        name="can_shell",
    )
    spotlight_can.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="front_bezel",
    )
    spotlight_can.visual(
        Cylinder(radius=0.095, length=0.074),
        origin=Origin(
            xyz=(0.216, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=soft_black,
        name="lens_tube",
    )
    spotlight_can.visual(
        Cylinder(radius=0.079, length=0.008),
        origin=Origin(
            xyz=(0.254, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="front_lens",
    )
    spotlight_can.visual(
        Cylinder(radius=0.084, length=0.036),
        origin=Origin(
            xyz=(-0.032, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=soft_black,
        name="rear_service_band",
    )
    spotlight_can.visual(
        Cylinder(radius=0.061, length=0.032),
        origin=Origin(
            xyz=(-0.098, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="rear_cap",
    )
    spotlight_can.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(-0.118, -0.034, -0.042)),
        material=matte_black,
        name="cable_gland",
    )
    spotlight_can.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.092, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="left_hub_root",
    )
    spotlight_can.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.092, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="right_hub_root",
    )
    spotlight_can.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.105, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="left_trunnion",
    )
    spotlight_can.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(
            xyz=(0.0, -0.105, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="right_trunnion",
    )
    spotlight_can.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.380),
        mass=5.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_yoke_pan",
        ArticulationType.CONTINUOUS,
        parent=floor_base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.306)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.5),
    )
    model.articulation(
        "yoke_to_can_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight_can,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.42,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    floor_base = object_model.get_part("floor_base")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight_can = object_model.get_part("spotlight_can")

    pan = object_model.get_articulation("base_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_can_tilt")
    tilt_limits = tilt.motion_limits

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
    ctx.allow_overlap(
        spotlight_can,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_bearing_collar",
        reason="The left trunnion is intentionally shown seated into its bearing collar to read as a supported spindle assembly.",
    )
    ctx.allow_overlap(
        spotlight_can,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_bearing_collar",
        reason="The right trunnion is intentionally shown seated into its bearing collar to read as a supported spindle assembly.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        pan_yoke,
        floor_base,
        elem_a="pan_rotor",
        elem_b="bearing_top_flange",
        name="pan_stage_seated_on_bearing",
    )
    ctx.expect_gap(
        pan_yoke,
        floor_base,
        axis="z",
        positive_elem="pan_rotor",
        negative_elem="bearing_top_flange",
        max_gap=0.0005,
        max_penetration=0.0,
        name="pan_stage_bearing_gap_control",
    )
    ctx.expect_contact(
        spotlight_can,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_bearing_collar",
        name="left_trunnion_supported",
    )
    ctx.expect_contact(
        spotlight_can,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_bearing_collar",
        name="right_trunnion_supported",
    )
    ctx.expect_overlap(
        spotlight_can,
        pan_yoke,
        axes="xz",
        min_overlap=0.060,
        elem_a="left_trunnion",
        elem_b="left_bearing_collar",
        name="left_trunnion_aligned_on_axis",
    )
    ctx.expect_overlap(
        spotlight_can,
        pan_yoke,
        axes="xz",
        min_overlap=0.060,
        elem_a="right_trunnion",
        elem_b="right_bearing_collar",
        name="right_trunnion_aligned_on_axis",
    )

    base_aabb = ctx.part_world_aabb(floor_base)
    can_aabb = ctx.part_world_aabb(spotlight_can)
    if base_aabb is not None and can_aabb is not None:
        base_width = base_aabb[1][0] - base_aabb[0][0]
        can_length = can_aabb[1][0] - can_aabb[0][0]
        ctx.check(
            "base_is_stable_for_fixture_scale",
            base_width >= 0.80 * can_length,
            f"Base width {base_width:.3f} m should read stable against can length {can_length:.3f} m.",
        )

    bezel_aabb = ctx.part_element_world_aabb(spotlight_can, elem="front_bezel")
    lens_aabb = ctx.part_element_world_aabb(spotlight_can, elem="front_lens")
    if bezel_aabb is not None and lens_aabb is not None:
        ctx.check(
            "front_lens_recessed_behind_bezel",
            lens_aabb[1][0] < bezel_aabb[1][0] - 0.010,
            "Front lens should sit noticeably behind the bezel lip.",
        )
        lens_span_y = lens_aabb[1][1] - lens_aabb[0][1]
        bezel_span_y = bezel_aabb[1][1] - bezel_aabb[0][1]
        ctx.check(
            "front_lens_captured_by_bezel",
            lens_span_y < bezel_span_y,
            "Front bezel should read larger than the lens aperture.",
        )

    rest_front = ctx.part_element_world_aabb(spotlight_can, elem="front_bezel")
    lower_front = None
    upper_front = None

    if tilt_limits is not None and tilt_limits.lower is not None:
        with ctx.pose({tilt: tilt_limits.lower}):
            lower_front = ctx.part_element_world_aabb(spotlight_can, elem="front_bezel")
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_lower_no_floating")
            ctx.expect_contact(
                spotlight_can,
                pan_yoke,
                elem_a="left_trunnion",
                elem_b="left_bearing_collar",
                name="left_trunnion_contact_at_lower_tilt",
            )
            ctx.expect_contact(
                spotlight_can,
                pan_yoke,
                elem_a="right_trunnion",
                elem_b="right_bearing_collar",
                name="right_trunnion_contact_at_lower_tilt",
            )
            ctx.expect_gap(
                spotlight_can,
                floor_base,
                axis="z",
                min_gap=0.010,
                name="lower_tilt_keeps_can_clear_of_base",
            )

    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_front = ctx.part_element_world_aabb(spotlight_can, elem="front_bezel")
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_upper_no_floating")
            ctx.expect_contact(
                spotlight_can,
                pan_yoke,
                elem_a="left_trunnion",
                elem_b="left_bearing_collar",
                name="left_trunnion_contact_at_upper_tilt",
            )
            ctx.expect_contact(
                spotlight_can,
                pan_yoke,
                elem_a="right_trunnion",
                elem_b="right_bearing_collar",
                name="right_trunnion_contact_at_upper_tilt",
            )

    if rest_front is not None and lower_front is not None and upper_front is not None:
        rest_center = _aabb_center(rest_front)
        lower_center = _aabb_center(lower_front)
        upper_center = _aabb_center(upper_front)
        ctx.check(
            "tilt_stage_moves_front_arc_down_and_up",
            lower_center[2] < rest_center[2] - 0.090 and upper_center[2] > rest_center[2] + 0.090,
            "Tilt motion should swing the spotlight front clearly below and above its rest position.",
        )

    pan_reference_front = None
    with ctx.pose({tilt: 0.40}):
        pan_reference_front = ctx.part_element_world_aabb(spotlight_can, elem="front_bezel")

    with ctx.pose({pan: 1.10, tilt: 0.40}):
        turned_front = ctx.part_element_world_aabb(spotlight_can, elem="front_bezel")
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.expect_contact(
            spotlight_can,
            pan_yoke,
            elem_a="left_trunnion",
            elem_b="left_bearing_collar",
            name="left_trunnion_contact_in_combined_pose",
        )
        ctx.expect_contact(
            spotlight_can,
            pan_yoke,
            elem_a="right_trunnion",
            elem_b="right_bearing_collar",
            name="right_trunnion_contact_in_combined_pose",
        )
        if pan_reference_front is not None and turned_front is not None:
            rest_center = _aabb_center(pan_reference_front)
            turned_center = _aabb_center(turned_front)
            radial_shift = math.hypot(
                turned_center[0] - rest_center[0],
                turned_center[1] - rest_center[1],
            )
            ctx.check(
                "pan_stage_changes_heading",
                radial_shift > 0.120,
                "Pan stage should visibly move the spotlight heading around the vertical axis.",
            )
            ctx.check(
                "pan_stage_preserves_elevation",
                abs(turned_center[2] - rest_center[2]) < 0.010,
                "Pan should mainly change heading, not lift the spotlight significantly.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
