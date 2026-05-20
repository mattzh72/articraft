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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, f"tracking_dish_{name}_mesh")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracking_dish")

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.69, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.87, 0.89, 0.91, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.37, 0.40, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))

    reflector_shell = _mesh(
        "reflector_shell",
        LatheGeometry(
            [
                (1.82, 0.848),
                (1.78, 0.824),
                (1.54, 0.606),
                (1.16, 0.378),
                (0.74, 0.176),
                (0.25, 0.040),
                (0.00, 0.000),
                (0.00, 0.028),
                (0.22, 0.052),
                (0.71, 0.190),
                (1.13, 0.390),
                (1.50, 0.624),
                (1.75, 0.838),
            ],
            segments=88,
        ),
    )
    reflector_rim = _mesh(
        "reflector_rim",
        TorusGeometry(radius=1.79, tube=0.036, radial_segments=16, tubular_segments=88),
    )
    rear_truss_ring = _mesh(
        "rear_truss_ring",
        TorusGeometry(radius=1.05, tube=0.034, radial_segments=16, tubular_segments=72),
    )
    hub_ring = _mesh(
        "hub_ring",
        TorusGeometry(radius=0.42, tube=0.030, radial_segments=14, tubular_segments=56),
    )
    side_yoke_arm = _mesh(
        "side_yoke_arm",
        sweep_profile_along_spline(
            [
                (-0.02, 0.34, 0.24),
                (0.06, 0.42, 0.82),
                (0.12, 0.49, 1.40),
                (0.18, 0.54, 1.82),
                (0.22, 0.56, 2.00),
                (0.22, 0.56, 2.08),
            ],
            profile=rounded_rect_profile(0.17, 0.25, radius=0.040, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    side_yoke_brace = _mesh(
        "side_yoke_brace",
        tube_from_spline_points(
            [
                (-0.24, 0.16, 0.20),
                (-0.14, 0.24, 0.84),
                (0.00, 0.36, 1.46),
                (0.16, 0.50, 1.90),
            ],
            radius=0.046,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    rear_truss_strut = _mesh(
        "rear_truss_strut",
        tube_from_spline_points(
            [
                (-0.04, 0.00, 0.00),
                (0.24, 0.00, 0.00),
                (0.50, 0.00, 0.22),
                (0.72, 0.00, 0.92),
            ],
            radius=0.033,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    rim_truss_strut = _mesh(
        "rim_truss_strut",
        tube_from_spline_points(
            [
                (0.70, 0.00, 1.02),
                (0.92, 0.00, 1.23),
                (1.10, 0.00, 1.50),
                (1.22, 0.00, 1.76),
            ],
            radius=0.027,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        ),
    )
    feed_support_strut = _mesh(
        "feed_support_strut",
        tube_from_spline_points(
            [
                (1.23, 0.00, 1.72),
                (1.46, 0.00, 1.16),
                (1.67, 0.00, 0.52),
                (1.80, 0.00, 0.00),
            ],
            radius=0.024,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((1.90, 1.90, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="ground_pad",
    )
    pedestal.visual(
        Box((0.92, 0.92, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=machinery_gray,
        name="mount_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.28, length=1.21),
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
        material=painted_steel,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.40, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.68)),
        material=machinery_gray,
        name="column_cap",
    )
    pedestal.visual(
        Box((0.42, 0.28, 0.46)),
        origin=Origin(xyz=(-0.24, 0.32, 0.65)),
        material=machinery_gray,
        name="service_box",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((1.90, 1.90, 1.78)),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.42, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_steel,
        name="turntable_drum",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.58, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=machinery_gray,
        name="turntable_plate",
    )
    azimuth_stage.visual(
        Box((0.46, 0.40, 0.30)),
        origin=Origin(xyz=(-0.20, 0.02, 0.33)),
        material=machinery_gray,
        name="azimuth_drive_housing",
    )
    azimuth_stage.visual(
        Box((0.30, 0.24, 0.94)),
        origin=Origin(xyz=(0.06, 0.46, 0.65)),
        material=painted_steel,
        name="yoke_base",
    )
    azimuth_stage.visual(
        Box((0.14, 0.18, 1.28)),
        origin=Origin(xyz=(0.20, 0.69, 1.42)),
        material=painted_steel,
        name="side_yoke_arm",
    )
    azimuth_stage.visual(
        Box((0.08, 0.12, 1.02)),
        origin=Origin(xyz=(0.18, 0.54, 1.12)),
        material=dark_steel,
        name="side_yoke_brace",
    )
    azimuth_stage.visual(
        Box((0.08, 0.08, 0.16)),
        origin=Origin(xyz=(0.20, 0.58, 2.06)),
        material=machinery_gray,
        name="bearing_head",
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((1.10, 1.20, 2.20)),
        mass=920.0,
        origin=Origin(xyz=(0.02, 0.30, 1.10)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    dish_assembly.visual(
        Box((0.12, 0.12, 0.12)),
        origin=Origin(xyz=(0.06, -0.10, 0.0)),
        material=dark_steel,
        name="joint_bridge",
    )
    dish_assembly.visual(
        Box((0.24, 0.18, 0.22)),
        origin=Origin(xyz=(0.18, -0.20, 0.0)),
        material=machinery_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        Box((0.22, 0.16, 0.18)),
        origin=Origin(xyz=(0.18, -0.36, 0.0)),
        material=machinery_gray,
        name="rear_equipment_box",
    )
    dish_assembly.visual(
        Box((0.58, 0.10, 0.10)),
        origin=Origin(xyz=(0.44, -0.26, 0.0)),
        material=dark_steel,
        name="rear_truss_spine",
    )
    dish_assembly.visual(
        Box((0.54, 0.08, 0.08)),
        origin=Origin(xyz=(0.46, -0.30, 0.22), rpy=(0.0, -0.54, 0.0)),
        material=dark_steel,
        name="rear_truss_upper",
    )
    dish_assembly.visual(
        Box((0.54, 0.08, 0.08)),
        origin=Origin(xyz=(0.46, -0.30, -0.22), rpy=(0.0, 0.54, 0.0)),
        material=dark_steel,
        name="rear_truss_lower",
    )
    dish_assembly.visual(
        reflector_shell,
        origin=Origin(xyz=(0.66, -0.42, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="main_reflector",
    )
    dish_assembly.visual(
        Box((1.46, 0.08, 0.08)),
        origin=Origin(xyz=(1.42, -0.50, 0.0)),
        material=aluminum,
        name="feed_boom",
    )
    dish_assembly.visual(
        Box((0.18, 0.18, 0.18)),
        origin=Origin(xyz=(2.18, -0.50, 0.0)),
        material=matte_black,
        name="receiver_box",
    )
    dish_assembly.visual(
        Cylinder(radius=0.08, length=0.20),
        origin=Origin(xyz=(2.37, -0.50, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((3.00, 3.70, 3.70)),
        mass=760.0,
        origin=Origin(xyz=(1.18, -0.38, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.73)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.65,
            lower=-2.9,
            upper=2.9,
        ),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=dish_assembly,
        origin=Origin(xyz=(0.20, 0.49, 2.06)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.55,
            lower=-0.10,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    azimuth_stage = object_model.get_part("azimuth_stage")
    dish_assembly = object_model.get_part("dish_assembly")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_rotation = object_model.get_articulation("elevation_rotation")

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
        "azimuth_joint_is_vertical_revolute",
        azimuth_rotation.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical revolute azimuth axis, got {azimuth_rotation.axis!r}",
    )
    ctx.check(
        "elevation_joint_is_transverse_revolute",
        elevation_rotation.articulation_type == ArticulationType.REVOLUTE
        and abs(elevation_rotation.axis[0]) < 1e-9
        and abs(abs(elevation_rotation.axis[1]) - 1.0) < 1e-9
        and abs(elevation_rotation.axis[2]) < 1e-9,
        details=f"expected transverse elevation axis, got {elevation_rotation.axis!r}",
    )

    ctx.expect_contact(
        azimuth_stage,
        pedestal,
        elem_a="turntable_drum",
        elem_b="column_cap",
        contact_tol=0.002,
        name="turntable_seated_on_column_top",
    )
    ctx.expect_gap(
        azimuth_stage,
        dish_assembly,
        axis="y",
        positive_elem="bearing_head",
        negative_elem="trunnion_shaft",
        max_gap=0.001,
        max_penetration=1e-6,
        name="dish_supported_by_side_yoke",
    )
    ctx.expect_gap(
        dish_assembly,
        pedestal,
        axis="z",
        min_gap=0.08,
        name="dish_clears_pedestal_in_rest_pose",
    )
    ctx.expect_origin_gap(
        dish_assembly,
        azimuth_stage,
        axis="z",
        min_gap=1.95,
        max_gap=2.15,
        name="elevation_axis_is_above_turntable",
    )

    reflector_aabb = ctx.part_element_world_aabb(dish_assembly, elem="main_reflector")
    feed_aabb = ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
    yoke_aabb = ctx.part_element_world_aabb(azimuth_stage, elem="side_yoke_arm")
    rear_truss_aabb = ctx.part_element_world_aabb(dish_assembly, elem="rear_truss_spine")

    if reflector_aabb is None:
        ctx.fail("reflector_present", "main_reflector AABB was unavailable")
    else:
        reflector_depth = reflector_aabb[1][0] - reflector_aabb[0][0]
        reflector_width = reflector_aabb[1][1] - reflector_aabb[0][1]
        reflector_height = reflector_aabb[1][2] - reflector_aabb[0][2]
        ctx.check(
            "reflector_is_large_and_deep",
            reflector_width > 3.45 and reflector_height > 3.45 and reflector_depth > 0.78,
            details=(
                f"reflector dims were depth={reflector_depth:.3f}, "
                f"width={reflector_width:.3f}, height={reflector_height:.3f}"
            ),
        )

    if reflector_aabb is None or feed_aabb is None:
        ctx.fail("feed_horn_present", "feed_horn or main_reflector AABB was unavailable")
    else:
        ctx.check(
            "feed_horn_projects_ahead_of_reflector",
            feed_aabb[0][0] > reflector_aabb[1][0] + 0.45,
            details=(
                f"feed horn started at x={feed_aabb[0][0]:.3f} while reflector front ended at "
                f"x={reflector_aabb[1][0]:.3f}"
            ),
        )

    if yoke_aabb is None:
        ctx.fail("side_yoke_present", "side_yoke_arm AABB was unavailable")
    else:
        yoke_center_y = 0.5 * (yoke_aabb[0][1] + yoke_aabb[1][1])
        ctx.check(
            "yoke_is_offset_to_one_side",
            yoke_center_y > 0.20,
            details=f"expected a single side yoke on +y, got center y={yoke_center_y:.3f}",
        )

    if reflector_aabb is None or rear_truss_aabb is None:
        ctx.fail("rear_truss_present", "rear_truss_spine or reflector AABB was unavailable")
    else:
        ctx.check(
            "rear_truss_sits_behind_reflector_rim",
            rear_truss_aabb[1][0] < reflector_aabb[1][0] - 0.25,
            details=(
                f"rear truss spine front x={rear_truss_aabb[1][0]:.3f}, "
                f"reflector front x={reflector_aabb[1][0]:.3f}"
            ),
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for articulation, label in (
        (azimuth_rotation, "azimuth_rotation"),
        (elevation_rotation, "elevation_rotation"),
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(
                    contact_tol=0.002,
                    name=f"{label}_lower_no_floating",
                )
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(
                    contact_tol=0.002,
                    name=f"{label}_upper_no_floating",
                )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=18,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=18,
        contact_tol=0.002,
        name="sampled_pose_no_floating",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
