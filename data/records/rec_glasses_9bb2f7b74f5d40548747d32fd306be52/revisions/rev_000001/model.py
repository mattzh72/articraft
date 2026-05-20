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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _temple_arm_mesh(name: str, outboard_x: float):
    arm_profile = rounded_rect_profile(
        width=0.0052,
        height=0.0018,
        radius=0.0006,
        corner_segments=6,
    )
    arm_path = [
        (outboard_x, -0.0002, 0.0),
        (outboard_x, -0.048, 0.0),
        (outboard_x, -0.100, -0.004),
        (outboard_x * 0.95, -0.132, -0.016),
        (outboard_x * 0.85, -0.148, -0.031),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            arm_path,
            profile=arm_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
        name,
    )


def _rim_mesh(name: str, outer_width: float, outer_height: float, rim_thickness: float, depth: float):
    outer_radius = min(outer_width, outer_height) * 0.18
    inner_radius = max(0.0012, outer_radius - rim_thickness * 0.55)
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_width,
            outer_height,
            outer_radius,
            corner_segments=10,
        ),
        [
            rounded_rect_profile(
                outer_width - 2.0 * rim_thickness,
                outer_height - 2.0 * rim_thickness,
                inner_radius,
                corner_segments=10,
            )
        ],
        height=depth,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _bridge_mesh(name: str):
    bridge_profile = rounded_rect_profile(
        width=0.0036,
        height=0.0028,
        radius=0.0009,
        corner_segments=6,
    )
    bridge_path = [
        (-0.0095, 0.0, 0.0063),
        (-0.0040, 0.0, 0.0042),
        (0.0, 0.0, 0.0032),
        (0.0040, 0.0, 0.0042),
        (0.0095, 0.0, 0.0063),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            bridge_path,
            profile=bridge_profile,
            samples_per_segment=18,
            cap_profile=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_eyeglasses")

    acetate = model.material("acetate", rgba=(0.10, 0.10, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.70, 0.72, 0.75, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.82, 0.90, 0.98, 0.20))

    rim_thickness = 0.004
    frame_depth = 0.0032
    lens_opening_width = 0.045
    lens_opening_height = 0.030
    rim_outer_width = lens_opening_width + 2.0 * rim_thickness
    rim_outer_height = lens_opening_height + 2.0 * rim_thickness
    rim_center_x = (lens_opening_width + 0.016) * 0.5
    hinge_block_width = 0.006
    hinge_block_height = 0.010
    hinge_z = 0.006
    barrel_radius = 0.0016
    barrel_length = 0.010
    hinge_x = rim_center_x + rim_outer_width * 0.5 + hinge_block_width * 0.5
    left_hinge_block_depth = 0.0056
    right_hinge_block_depth = 0.0116
    left_hinge_block_y = -0.0016
    right_hinge_block_y = -0.0046
    left_hinge_origin_y = left_hinge_block_y - left_hinge_block_depth * 0.5 - barrel_radius
    right_hinge_origin_y = right_hinge_block_y - right_hinge_block_depth * 0.5 - barrel_radius

    front_frame = model.part("front_frame")
    front_frame.visual(
        _rim_mesh(
            "left_rim_shell_mesh_v3",
            rim_outer_width,
            rim_outer_height,
            rim_thickness,
            frame_depth,
        ),
        origin=Origin(xyz=(-rim_center_x, 0.0, 0.0)),
        material=acetate,
        name="left_rim_shell",
    )
    front_frame.visual(
        _rim_mesh(
            "right_rim_shell_mesh_v3",
            rim_outer_width,
            rim_outer_height,
            rim_thickness,
            frame_depth,
        ),
        origin=Origin(xyz=(rim_center_x, 0.0, 0.0)),
        material=acetate,
        name="right_rim_shell",
    )
    front_frame.visual(
        _bridge_mesh("eyeglass_bridge_mesh_v3"),
        material=acetate,
        name="bridge",
    )
    front_frame.visual(
        Box((0.005, frame_depth, 0.012)),
        origin=Origin(
            xyz=(
                -(rim_center_x + rim_outer_width * 0.5 + 0.0005),
                -0.0006,
                0.006,
            )
        ),
        material=acetate,
        name="left_endpiece",
    )
    front_frame.visual(
        Box((0.005, frame_depth, 0.012)),
        origin=Origin(
            xyz=(
                rim_center_x + rim_outer_width * 0.5 + 0.0005,
                -0.0006,
                0.006,
            )
        ),
        material=acetate,
        name="right_endpiece",
    )
    front_frame.visual(
        Box((lens_opening_width, 0.0012, lens_opening_height)),
        origin=Origin(xyz=(-rim_center_x, 0.0, 0.0)),
        material=lens_clear,
        name="left_lens",
    )
    front_frame.visual(
        Box((lens_opening_width, 0.0012, lens_opening_height)),
        origin=Origin(xyz=(rim_center_x, 0.0, 0.0)),
        material=lens_clear,
        name="right_lens",
    )
    front_frame.visual(
        Box((hinge_block_width, left_hinge_block_depth, hinge_block_height)),
        origin=Origin(xyz=(-hinge_x, left_hinge_block_y, hinge_z)),
        material=metal,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((hinge_block_width, right_hinge_block_depth, hinge_block_height)),
        origin=Origin(xyz=(hinge_x, right_hinge_block_y, hinge_z)),
        material=metal,
        name="right_hinge_block",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.126, frame_depth, rim_outer_height)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        material=metal,
        name="left_hinge_barrel",
    )
    left_temple.visual(
        _temple_arm_mesh("left_temple_arm_mesh_v2", outboard_x=-0.0022),
        material=acetate,
        name="left_temple_arm",
    )
    left_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.145, 0.034)),
        mass=0.006,
        origin=Origin(xyz=(-0.006, -0.072, -0.013)),
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        material=metal,
        name="right_hinge_barrel",
    )
    right_temple.visual(
        _temple_arm_mesh("right_temple_arm_mesh_v2", outboard_x=0.0022),
        material=acetate,
        name="right_temple_arm",
    )
    right_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.145, 0.034)),
        mass=0.006,
        origin=Origin(xyz=(0.006, -0.072, -0.013)),
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, left_hinge_origin_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=6.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, right_hinge_origin_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=6.0,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")

    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

    left_hinge_block = front_frame.get_visual("left_hinge_block")
    right_hinge_block = front_frame.get_visual("right_hinge_block")
    left_hinge_barrel = left_temple.get_visual("left_hinge_barrel")
    right_hinge_barrel = right_temple.get_visual("right_hinge_barrel")
    left_temple_arm = left_temple.get_visual("left_temple_arm")
    right_temple_arm = right_temple.get_visual("right_temple_arm")

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
        "left_hinge_axis_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_vertical",
        right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={right_hinge.axis}",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    ctx.check(
        "left_hinge_limit_range",
        left_limits is not None
        and math.isclose(left_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and 1.45 <= (left_limits.upper or 0.0) <= 1.60,
        details=f"limits={left_limits}",
    )
    ctx.check(
        "right_hinge_limit_range",
        right_limits is not None
        and math.isclose(right_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and 1.45 <= (right_limits.upper or 0.0) <= 1.60,
        details=f"limits={right_limits}",
    )

    frame_aabb = ctx.part_world_aabb(front_frame)
    if frame_aabb is None:
        ctx.fail("front_frame_has_geometry", "front_frame AABB is unavailable")
    else:
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "front_frame_width_realistic",
            0.12 <= frame_width <= 0.14,
            details=f"width={frame_width:.4f}",
        )
        ctx.check(
            "front_frame_height_realistic",
            0.035 <= frame_height <= 0.045,
            details=f"height={frame_height:.4f}",
        )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_temple,
            front_frame,
            elem_a=left_hinge_barrel,
            elem_b=left_hinge_block,
            name="left_hinge_contact_at_rest",
        )
        ctx.expect_contact(
            right_temple,
            front_frame,
            elem_a=right_hinge_barrel,
            elem_b=right_hinge_block,
            name="right_hinge_contact_at_rest",
        )
        ctx.expect_gap(
            front_frame,
            left_temple,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=left_hinge_block,
            negative_elem=left_hinge_barrel,
            name="left_hinge_barrel_sits_behind_frame",
        )
        ctx.expect_gap(
            front_frame,
            right_temple,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=right_hinge_block,
            negative_elem=right_hinge_barrel,
            name="right_hinge_barrel_sits_behind_frame",
        )
        left_origin = ctx.part_world_position(left_temple)
        right_origin = ctx.part_world_position(right_temple)
        if left_origin is None or right_origin is None:
            ctx.fail("temple_origins_available", "Temple part origins unavailable in rest pose")
        else:
            ctx.check(
                "left_temple_origin_behind_front",
                -0.0075 <= left_origin[1] <= -0.0045,
                details=f"left_origin_y={left_origin[1]:.6f}",
            )
            ctx.check(
                "right_temple_origin_behind_front",
                -0.0135 <= right_origin[1] <= -0.0105,
                details=f"right_origin_y={right_origin[1]:.6f}",
            )
            ctx.check(
                "right_hinge_staggered_behind_left",
                right_origin[1] < left_origin[1] - 0.004,
                details=f"left_y={left_origin[1]:.6f} right_y={right_origin[1]:.6f}",
            )

    if left_limits is not None and left_limits.lower is not None and left_limits.upper is not None:
        with ctx.pose({left_hinge: left_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="left_hinge_lower_no_floating")
        with ctx.pose({left_hinge: left_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="left_hinge_upper_no_floating")
            ctx.expect_overlap(
                left_temple,
                front_frame,
                axes="xz",
                min_overlap=0.010,
                name="left_temple_folds_inward_over_frame",
            )
            left_arm_aabb = ctx.part_element_world_aabb(left_temple, elem=left_temple_arm)
            left_block_aabb = ctx.part_element_world_aabb(front_frame, elem=left_hinge_block)
            if left_arm_aabb is None or left_block_aabb is None:
                ctx.fail("left_folded_arm_aabb_available", "Left folded arm or hinge block AABB unavailable")
            else:
                ctx.check(
                    "left_temple_folded_clear_of_front_plane",
                    left_arm_aabb[1][1] <= left_block_aabb[0][1] + 0.0005,
                    details=(
                        f"left_arm_max_y={left_arm_aabb[1][1]:.6f} "
                        f"left_block_rear_y={left_block_aabb[0][1]:.6f}"
                    ),
                )

    if right_limits is not None and right_limits.lower is not None and right_limits.upper is not None:
        with ctx.pose({right_hinge: right_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="right_hinge_lower_no_floating")
        with ctx.pose({right_hinge: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="right_hinge_upper_no_floating")
            ctx.expect_overlap(
                right_temple,
                front_frame,
                axes="xz",
                min_overlap=0.010,
                name="right_temple_folds_inward_over_frame",
            )
            right_arm_aabb = ctx.part_element_world_aabb(right_temple, elem=right_temple_arm)
            right_block_aabb = ctx.part_element_world_aabb(front_frame, elem=right_hinge_block)
            if right_arm_aabb is None or right_block_aabb is None:
                ctx.fail("right_folded_arm_aabb_available", "Right folded arm or hinge block AABB unavailable")
            else:
                ctx.check(
                    "right_temple_folded_clear_of_front_plane",
                    right_arm_aabb[1][1] <= right_block_aabb[0][1] + 0.0005,
                    details=(
                        f"right_arm_max_y={right_arm_aabb[1][1]:.6f} "
                        f"right_block_rear_y={right_block_aabb[0][1]:.6f}"
                    ),
                )

    if left_limits is not None and right_limits is not None:
        with ctx.pose({left_hinge: left_limits.upper, right_hinge: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_temples_folded_no_overlap")
            ctx.fail_if_isolated_parts(name="both_temples_folded_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
