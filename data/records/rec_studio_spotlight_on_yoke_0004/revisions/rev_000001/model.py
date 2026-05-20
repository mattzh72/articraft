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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_studio_spotlight", assets=ASSETS)

    powder_coat = model.material("powder_coat", rgba=(0.27, 0.29, 0.31, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    molded_black = model.material("molded_black", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    zinc = model.material("zinc", rgba=(0.72, 0.74, 0.76, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.50, 0.60, 0.66, 0.35))
    dark_interior = model.material("dark_interior", rgba=(0.05, 0.05, 0.06, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _axis_rpy(axis: str) -> tuple[float, float, float]:
        if axis == "x":
            return (0.0, math.pi / 2.0, 0.0)
        if axis == "y":
            return (-math.pi / 2.0, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    def _add_bolt(
        part,
        *,
        center: tuple[float, float, float],
        axis: str = "z",
        radius: float = 0.004,
        length: float = 0.008,
        material=zinc,
        name: str | None = None,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=_axis_rpy(axis)),
            material=material,
            name=name,
        )

    can_shell = _save_mesh(
        "spotlight_can_shell.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.048, 0.000),
                (0.072, 0.010),
                (0.086, 0.060),
                (0.090, 0.168),
                (0.088, 0.220),
                (0.094, 0.244),
            ],
            inner_profile=[
                (0.040, 0.006),
                (0.068, 0.016),
                (0.076, 0.060),
                (0.078, 0.168),
                (0.080, 0.220),
            ],
            segments=72,
        ),
    )
    front_ring = _save_mesh(
        "spotlight_front_ring.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.086, 0.000),
                (0.096, 0.004),
                (0.096, 0.014),
                (0.089, 0.018),
            ],
            inner_profile=[
                (0.080, 0.000),
                (0.084, 0.004),
                (0.084, 0.018),
            ],
            segments=64,
        ),
    )
    carry_handle = _save_mesh(
        "spotlight_carry_handle.obj",
        tube_from_spline_points(
            [
                (0.006, 0.0, 0.080),
                (0.034, 0.0, 0.116),
                (0.086, 0.0, 0.134),
                (0.126, 0.0, 0.118),
                (0.148, 0.0, 0.088),
            ],
            radius=0.008,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    base_assembly = model.part("base_assembly")
    base_assembly.visual(
        Box((0.46, 0.34, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=powder_coat,
        name="base_plate",
    )
    base_assembly.visual(
        Box((0.30, 0.18, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=powder_coat,
        name="ballast_housing",
    )
    base_assembly.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=housing_gray,
        name="column_shroud",
    )
    base_assembly.visual(
        Cylinder(radius=0.048, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=housing_gray,
        name="center_column",
    )
    base_assembly.visual(
        Cylinder(radius=0.070, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material=molded_black,
        name="pan_bearing_collar",
    )
    base_assembly.visual(
        Box((0.120, 0.030, 0.095)),
        origin=Origin(xyz=(-0.018, 0.060, 0.131)),
        material=housing_gray,
        name="left_column_cheek",
    )
    base_assembly.visual(
        Box((0.120, 0.030, 0.095)),
        origin=Origin(xyz=(-0.018, -0.060, 0.131)),
        material=housing_gray,
        name="right_column_cheek",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            base_assembly.visual(
                Cylinder(radius=0.022, length=0.010),
                origin=Origin(xyz=(0.175 * x_sign, 0.125 * y_sign, 0.005)),
                material=rubber_black,
                name=f"foot_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_bolt(
                base_assembly,
                center=(0.095 * x_sign, 0.050 * y_sign, 0.089),
                axis="z",
                radius=0.005,
                length=0.010,
            )

    base_assembly.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 0.24)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    yoke_head = model.part("yoke_head")
    yoke_head.visual(
        Cylinder(radius=0.072, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=molded_black,
        name="pan_drum",
    )
    yoke_head.visual(
        Cylinder(radius=0.105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=powder_coat,
        name="rotary_stage_plate",
    )
    yoke_head.visual(
        Box((0.090, 0.250, 0.018)),
        origin=Origin(xyz=(-0.120, 0.0, 0.072)),
        material=powder_coat,
        name="yoke_cross_member",
    )
    yoke_head.visual(
        Box((0.090, 0.116, 0.052)),
        origin=Origin(xyz=(-0.145, 0.0, 0.058)),
        material=powder_coat,
        name="pan_drive_body",
    )
    yoke_head.visual(
        Box((0.050, 0.090, 0.080)),
        origin=Origin(xyz=(-0.136, 0.0, 0.106)),
        material=housing_gray,
        name="yoke_riser_core",
    )
    yoke_head.visual(
        Box((0.320, 0.018, 0.200)),
        origin=Origin(xyz=(0.100, 0.108, 0.125)),
        material=powder_coat,
        name="left_yoke_bracket",
    )
    yoke_head.visual(
        Box((0.320, 0.018, 0.200)),
        origin=Origin(xyz=(0.100, -0.108, 0.125)),
        material=powder_coat,
        name="right_yoke_bracket",
    )
    for side_name, side_y in (("left", 0.108), ("right", -0.108)):
        for x_center, z_center in ((-0.050, 0.080), (0.046, 0.080)):
            yoke_head.visual(
                Box((0.046, 0.018, 0.080)),
                origin=Origin(xyz=(x_center, side_y, z_center)),
                material=powder_coat,
                name=f"{side_name}_gusset_{'rear' if x_center < 0.0 else 'front'}",
            )
        yoke_head.visual(
            Box((0.120, 0.018, 0.082)),
            origin=Origin(xyz=(0.130, side_y, 0.126)),
            material=housing_gray,
            name=f"{side_name}_trunnion_block",
        )
    yoke_head.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.130, 0.103, 0.126), rpy=_axis_rpy("y")),
        material=housing_gray,
        name="left_bearing_boss",
    )
    yoke_head.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.130, -0.103, 0.126), rpy=_axis_rpy("y")),
        material=housing_gray,
        name="right_bearing_boss",
    )
    yoke_head.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.130, 0.120, 0.126), rpy=_axis_rpy("y")),
        material=molded_black,
        name="left_tilt_lock_knob",
    )
    yoke_head.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.130, -0.120, 0.126), rpy=_axis_rpy("y")),
        material=molded_black,
        name="right_tilt_lock_knob",
    )

    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        _add_bolt(
            yoke_head,
            center=(0.082 * math.cos(angle), 0.082 * math.sin(angle), 0.050),
            axis="z",
            radius=0.0045,
            length=0.008,
        )

    for side_name, side_y in (("left", 0.112), ("right", -0.112)):
        for idx, (x_pos, z_pos) in enumerate(((-0.024, 0.074), (0.130, 0.126), (0.220, 0.194))):
            _add_bolt(
                yoke_head,
                center=(x_pos, side_y, z_pos),
                axis="y",
                radius=0.0042,
                length=0.012,
                name=f"{side_name}_bracket_bolt_{idx}",
            )

    yoke_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.30, 0.24)),
        mass=5.1,
        origin=Origin(xyz=(-0.030, 0.0, 0.12)),
    )

    lamp_can = model.part("lamp_can")
    lamp_can.visual(
        can_shell,
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_gray,
        name="can_shell",
    )
    lamp_can.visual(
        front_ring,
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="front_retaining_ring",
    )
    lamp_can.visual(
        Cylinder(radius=0.080, length=0.006),
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=_axis_rpy("x")),
        material=smoked_glass,
        name="lens_glass",
    )
    lamp_can.visual(
        Cylinder(radius=0.078, length=0.038),
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=_axis_rpy("x")),
        material=dark_interior,
        name="interior_baffle",
    )
    lamp_can.visual(
        Box((0.108, 0.086, 0.052)),
        origin=Origin(xyz=(0.090, 0.0, 0.058)),
        material=powder_coat,
        name="service_box",
    )
    for idx, fin_y in enumerate((-0.030, -0.010, 0.010, 0.030)):
        lamp_can.visual(
            Box((0.058, 0.007, 0.022)),
            origin=Origin(xyz=(0.102, fin_y, 0.086)),
            material=housing_gray,
            name=f"cooling_fin_{idx}",
        )
    lamp_can.visual(
        carry_handle,
        origin=Origin(xyz=(0.072, 0.0, 0.000)),
        material=molded_black,
        name="carry_handle",
    )
    lamp_can.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.028, 0.0, 0.058), rpy=_axis_rpy("x")),
        material=molded_black,
        name="cable_gland",
    )
    lamp_can.visual(
        Box((0.200, 0.090, 0.070)),
        origin=Origin(xyz=(0.100, 0.040, 0.015)),
        material=housing_gray,
        name="left_trunnion_web",
    )
    lamp_can.visual(
        Box((0.200, 0.090, 0.070)),
        origin=Origin(xyz=(0.100, -0.040, 0.015)),
        material=housing_gray,
        name="right_trunnion_web",
    )
    lamp_can.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.085, 0.0), rpy=_axis_rpy("y")),
        material=housing_gray,
        name="left_trunnion_collar",
    )
    lamp_can.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, -0.085, 0.0), rpy=_axis_rpy("y")),
        material=housing_gray,
        name="right_trunnion_collar",
    )

    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        _add_bolt(
            lamp_can,
            center=(0.305, 0.084 * math.cos(angle), 0.084 * math.sin(angle)),
            axis="x",
            radius=0.004,
            length=0.008,
            name=f"front_ring_bolt_{index}",
        )

    lamp_can.inertial = Inertial.from_geometry(
        Box((0.31, 0.22, 0.24)),
        mass=5.6,
        origin=Origin(xyz=(0.150, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_pan",
        ArticulationType.REVOLUTE,
        parent=base_assembly,
        child=yoke_head,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke_head,
        child=lamp_can,
        origin=Origin(xyz=(0.130, 0.0, 0.126)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_assembly = object_model.get_part("base_assembly")
    yoke_head = object_model.get_part("yoke_head")
    lamp_can = object_model.get_part("lamp_can")
    base_to_pan = object_model.get_articulation("base_to_pan")
    pan_to_tilt = object_model.get_articulation("pan_to_tilt")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=28,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    with ctx.pose({base_to_pan: 0.0, pan_to_tilt: 0.0}):
        ctx.expect_contact(
            yoke_head,
            base_assembly,
            elem_a="pan_drum",
            elem_b="pan_bearing_collar",
            name="pan_stage_seated_on_bearing",
        )
        ctx.expect_contact(
            lamp_can,
            yoke_head,
            elem_a="left_trunnion_collar",
            elem_b="left_bearing_boss",
            name="left_tilt_collar_supported",
        )
        ctx.expect_contact(
            lamp_can,
            yoke_head,
            elem_a="right_trunnion_collar",
            elem_b="right_bearing_boss",
            name="right_tilt_collar_supported",
        )

    def _aabb_center(aabb):
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    base_aabb = ctx.part_world_aabb(base_assembly)
    lamp_aabb = ctx.part_world_aabb(lamp_can)
    if base_aabb is None or lamp_aabb is None:
        ctx.fail("aabb_queries_available", "Expected world AABBs for base and lamp can.")
        return ctx.report()

    base_width = base_aabb[1][0] - base_aabb[0][0]
    base_depth = base_aabb[1][1] - base_aabb[0][1]
    total_height = max(base_aabb[1][2], lamp_aabb[1][2]) - min(base_aabb[0][2], lamp_aabb[0][2])
    ctx.check(
        "stable_base_footprint",
        base_width >= 0.42 and base_depth >= 0.30,
        details=f"Base footprint too small: width={base_width:.3f}, depth={base_depth:.3f}",
    )
    ctx.check(
        "realistic_overall_height",
        0.48 <= total_height <= 0.62,
        details=f"Unexpected overall height: {total_height:.3f} m",
    )

    lens_rest_aabb = ctx.part_element_world_aabb(lamp_can, elem="lens_glass")
    if lens_rest_aabb is None:
        ctx.fail("lens_aabb_available", "Expected lens_glass world AABB.")
        return ctx.report()
    lens_rest_center = _aabb_center(lens_rest_aabb)

    tilt_limits = pan_to_tilt.motion_limits
    pan_limits = base_to_pan.motion_limits
    if tilt_limits is None or tilt_limits.lower is None or tilt_limits.upper is None:
        ctx.fail("tilt_limits_defined", "Tilt articulation must have finite motion limits.")
        return ctx.report()
    if pan_limits is None or pan_limits.lower is None or pan_limits.upper is None:
        ctx.fail("pan_limits_defined", "Pan articulation must have finite motion limits.")
        return ctx.report()

    with ctx.pose({pan_to_tilt: tilt_limits.upper}):
        lens_up_aabb = ctx.part_element_world_aabb(lamp_can, elem="lens_glass")
        if lens_up_aabb is None:
            ctx.fail("lens_up_aabb_available", "Expected lens AABB at positive tilt.")
        else:
            lens_up_center = _aabb_center(lens_up_aabb)
            ctx.check(
                "positive_tilt_raises_beam",
                lens_up_center[2] > lens_rest_center[2] + 0.08,
                details=(
                    f"Lens center did not rise enough: rest_z={lens_rest_center[2]:.3f}, "
                    f"tilted_z={lens_up_center[2]:.3f}"
                ),
            )
        ctx.expect_contact(lamp_can, yoke_head, elem_a="left_trunnion_collar", elem_b="left_bearing_boss")
        ctx.expect_contact(lamp_can, yoke_head, elem_a="right_trunnion_collar", elem_b="right_bearing_boss")
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="tilt_upper_no_floating")

    with ctx.pose({pan_to_tilt: tilt_limits.lower}):
        lens_down_aabb = ctx.part_element_world_aabb(lamp_can, elem="lens_glass")
        if lens_down_aabb is None:
            ctx.fail("lens_down_aabb_available", "Expected lens AABB at negative tilt.")
        else:
            lens_down_center = _aabb_center(lens_down_aabb)
            ctx.check(
                "negative_tilt_lowers_beam",
                lens_down_center[2] < lens_rest_center[2] - 0.08,
                details=(
                    f"Lens center did not lower enough: rest_z={lens_rest_center[2]:.3f}, "
                    f"tilted_z={lens_down_center[2]:.3f}"
                ),
            )
        ctx.expect_contact(lamp_can, yoke_head, elem_a="left_trunnion_collar", elem_b="left_bearing_boss")
        ctx.expect_contact(lamp_can, yoke_head, elem_a="right_trunnion_collar", elem_b="right_bearing_boss")
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="tilt_lower_no_floating")

    with ctx.pose({base_to_pan: pan_limits.upper}):
        lens_pan_aabb = ctx.part_element_world_aabb(lamp_can, elem="lens_glass")
        if lens_pan_aabb is None:
            ctx.fail("lens_pan_aabb_available", "Expected lens AABB at pan limit.")
        else:
            lens_pan_center = _aabb_center(lens_pan_aabb)
            ctx.check(
                "pan_motion_swings_head",
                abs(lens_pan_center[1] - lens_rest_center[1]) > 0.09,
                details=(
                    f"Lens center did not sweep laterally enough: rest_y={lens_rest_center[1]:.3f}, "
                    f"pan_y={lens_pan_center[1]:.3f}"
                ),
            )
        ctx.expect_contact(yoke_head, base_assembly, elem_a="pan_drum", elem_b="pan_bearing_collar")
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="pan_upper_no_floating")

    with ctx.pose({base_to_pan: pan_limits.lower}):
        ctx.expect_contact(yoke_head, base_assembly, elem_a="pan_drum", elem_b="pan_bearing_collar")
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="pan_lower_no_floating")

    with ctx.pose({base_to_pan: pan_limits.upper, pan_to_tilt: tilt_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_upper_pose_no_floating")

    with ctx.pose({base_to_pan: pan_limits.lower, pan_to_tilt: tilt_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_lower_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_lower_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
