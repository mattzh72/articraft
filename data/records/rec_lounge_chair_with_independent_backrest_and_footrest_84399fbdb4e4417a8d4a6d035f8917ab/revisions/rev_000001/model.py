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
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _segment_mesh(
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
):
    return _save_mesh(
        name,
        wire_from_points(
            [start, end],
            radius=radius,
            cap_ends=True,
            radial_segments=16,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zero_gravity_lounge_chair")

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    mesh_fabric = model.material("mesh_fabric", rgba=(0.39, 0.43, 0.45, 1.0))
    armrest_pad = model.material("armrest_pad", rgba=(0.20, 0.16, 0.10, 1.0))
    pivot_cap = model.material("pivot_cap", rgba=(0.67, 0.69, 0.72, 1.0))

    ground_side_y = 0.305
    moving_side_y = 0.255
    ground_tube_radius = 0.018
    moving_tube_radius = 0.016
    footrest_tube_radius = 0.015
    main_pivot_origin = (0.04, 0.0, 0.58)
    footrest_pivot_origin = (0.60, 0.0, 0.46)

    ground_frame = model.part("ground_frame")

    left_ground_nodes = {
        "rear_foot": (-0.58, ground_side_y, 0.045),
        "rear_knee": (-0.36, ground_side_y, 0.24),
        "main_node": (0.00, ground_side_y, 0.60),
        "arm_node": (0.28, ground_side_y, 0.60),
        "front_node": (0.48, ground_side_y, 0.54),
        "front_foot": (0.76, ground_side_y, 0.045),
        "main_bracket": (0.04, ground_side_y, 0.58),
        "footrest_bracket": (0.60, ground_side_y, 0.46),
    }
    right_ground_nodes = {name: (x, -y, z) for name, (x, y, z) in left_ground_nodes.items()}

    ground_frame.visual(
        _segment_mesh("ground_left_lower_runner_v3", left_ground_nodes["rear_foot"], left_ground_nodes["front_foot"], ground_tube_radius),
        material=powder_coat,
        name="left_ground_support",
    )
    for seg_name, start_key, end_key in (
        ("ground_left_rear_leg_v3", "rear_foot", "rear_knee"),
        ("ground_left_rear_rise_v3", "rear_knee", "main_node"),
        ("ground_left_arm_rail_v3", "main_node", "arm_node"),
        ("ground_left_front_brace_v3", "arm_node", "front_node"),
        ("ground_left_front_leg_v3", "front_node", "front_foot"),
        ("ground_left_main_bracket_v3", "main_node", "main_bracket"),
        ("ground_left_footrest_bracket_v3", "front_node", "footrest_bracket"),
    ):
        ground_frame.visual(
            _segment_mesh(seg_name, left_ground_nodes[start_key], left_ground_nodes[end_key], 0.012 if "bracket" in seg_name else ground_tube_radius),
            material=powder_coat,
        )

    ground_frame.visual(
        _segment_mesh("ground_right_lower_runner_v3", right_ground_nodes["rear_foot"], right_ground_nodes["front_foot"], ground_tube_radius),
        material=powder_coat,
        name="right_ground_support",
    )
    for seg_name, start_key, end_key in (
        ("ground_right_rear_leg_v3", "rear_foot", "rear_knee"),
        ("ground_right_rear_rise_v3", "rear_knee", "main_node"),
        ("ground_right_arm_rail_v3", "main_node", "arm_node"),
        ("ground_right_front_brace_v3", "arm_node", "front_node"),
        ("ground_right_front_leg_v3", "front_node", "front_foot"),
        ("ground_right_main_bracket_v3", "main_node", "main_bracket"),
        ("ground_right_footrest_bracket_v3", "front_node", "footrest_bracket"),
    ):
        ground_frame.visual(
            _segment_mesh(seg_name, right_ground_nodes[start_key], right_ground_nodes[end_key], 0.012 if "bracket" in seg_name else ground_tube_radius),
            material=powder_coat,
        )

    ground_frame.visual(
        _segment_mesh("ground_front_crossbar_v3", left_ground_nodes["front_foot"], right_ground_nodes["front_foot"], 0.015),
        material=powder_coat,
        name="front_lower_crossbar",
    )
    ground_frame.visual(
        _segment_mesh("ground_rear_crossbar_v3", left_ground_nodes["rear_foot"], right_ground_nodes["rear_foot"], 0.015),
        material=powder_coat,
        name="rear_lower_crossbar",
    )
    ground_frame.visual(
        _segment_mesh("ground_armrest_crossbar_v3", left_ground_nodes["arm_node"], right_ground_nodes["arm_node"], 0.015),
        material=powder_coat,
        name="armrest_crossbar",
    )
    ground_frame.visual(
        Box((0.26, 0.06, 0.03)),
        origin=Origin(xyz=(0.18, ground_side_y, 0.615)),
        material=armrest_pad,
        name="left_armrest",
    )
    ground_frame.visual(
        Box((0.26, 0.06, 0.03)),
        origin=Origin(xyz=(0.18, -ground_side_y, 0.615)),
        material=armrest_pad,
        name="right_armrest",
    )
    ground_frame.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(main_pivot_origin[0], 0.301, main_pivot_origin[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="main_pivot_left_lug",
    )
    ground_frame.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(main_pivot_origin[0], -0.301, main_pivot_origin[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="main_pivot_right_lug",
    )
    ground_frame.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(footrest_pivot_origin[0], 0.300, footrest_pivot_origin[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="footrest_left_lug",
    )
    ground_frame.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(footrest_pivot_origin[0], -0.300, footrest_pivot_origin[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="footrest_right_lug",
    )

    recline_frame = model.part("recline_frame")

    left_recline_nodes = {
        "pivot": (0.00, moving_side_y, 0.00),
        "seat_mid": (0.18, moving_side_y, -0.05),
        "seat_front": (0.46, moving_side_y, -0.11),
        "back_low": (-0.22, moving_side_y, 0.16),
        "head_top": (-0.60, moving_side_y, 0.47),
    }
    right_recline_nodes = {name: (x, -y, z) for name, (x, y, z) in left_recline_nodes.items()}

    recline_frame.visual(
        _segment_mesh("recline_left_pivot_to_seat_v3", left_recline_nodes["pivot"], left_recline_nodes["seat_mid"], moving_tube_radius),
        material=dark_frame,
        name="left_recline_rail",
    )
    for seg_name, start_key, end_key in (
        ("recline_left_seat_front_v3", "seat_mid", "seat_front"),
        ("recline_left_back_lower_v3", "pivot", "back_low"),
        ("recline_left_back_upper_v3", "back_low", "head_top"),
    ):
        recline_frame.visual(
            _segment_mesh(seg_name, left_recline_nodes[start_key], left_recline_nodes[end_key], moving_tube_radius),
            material=dark_frame,
        )

    recline_frame.visual(
        _segment_mesh("recline_right_pivot_to_seat_v3", right_recline_nodes["pivot"], right_recline_nodes["seat_mid"], moving_tube_radius),
        material=dark_frame,
        name="right_recline_rail",
    )
    for seg_name, start_key, end_key in (
        ("recline_right_seat_front_v3", "seat_mid", "seat_front"),
        ("recline_right_back_lower_v3", "pivot", "back_low"),
        ("recline_right_back_upper_v3", "back_low", "head_top"),
    ):
        recline_frame.visual(
            _segment_mesh(seg_name, right_recline_nodes[start_key], right_recline_nodes[end_key], moving_tube_radius),
            material=dark_frame,
        )

    recline_frame.visual(
        _segment_mesh("recline_seat_front_crossbar_v3", left_recline_nodes["seat_front"], right_recline_nodes["seat_front"], 0.014),
        material=dark_frame,
        name="seat_front_crossbar",
    )
    recline_frame.visual(
        _segment_mesh("recline_seat_mid_crossbar_v3", left_recline_nodes["seat_mid"], right_recline_nodes["seat_mid"], 0.014),
        material=dark_frame,
        name="seat_mid_crossbar",
    )
    recline_frame.visual(
        _segment_mesh("recline_lumbar_crossbar_v3", left_recline_nodes["back_low"], right_recline_nodes["back_low"], 0.014),
        material=dark_frame,
        name="lumbar_crossbar",
    )
    recline_frame.visual(
        _segment_mesh("recline_head_crossbar_v3", left_recline_nodes["head_top"], right_recline_nodes["head_top"], 0.014),
        material=dark_frame,
        name="head_crossbar",
    )
    recline_frame.visual(
        Box((0.50, 0.512, 0.012)),
        origin=Origin(xyz=(0.26, 0.0, -0.055), rpy=(0.0, -0.18, 0.0)),
        material=mesh_fabric,
        name="seat_sling",
    )
    recline_frame.visual(
        Box((0.62, 0.512, 0.012)),
        origin=Origin(xyz=(-0.33, 0.0, 0.25), rpy=(0.0, -1.00, 0.0)),
        material=mesh_fabric,
        name="back_sling",
    )
    recline_frame.visual(
        Box((0.12, 0.512, 0.04)),
        origin=Origin(xyz=(-0.60, 0.0, 0.49), rpy=(0.0, -1.00, 0.0)),
        material=mesh_fabric,
        name="head_pillow",
    )
    recline_frame.visual(
        Cylinder(radius=0.021, length=0.030),
        origin=Origin(xyz=(0.0, 0.271, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="main_pivot_left_boss",
    )
    recline_frame.visual(
        Cylinder(radius=0.021, length=0.030),
        origin=Origin(xyz=(0.0, -0.271, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="main_pivot_right_boss",
    )

    footrest_panel = model.part("footrest_panel")

    left_footrest_nodes = {
        "pivot": (0.00, moving_side_y, 0.00),
        "mid": (0.16, moving_side_y, -0.03),
        "front": (0.34, moving_side_y, -0.08),
    }
    right_footrest_nodes = {name: (x, -y, z) for name, (x, y, z) in left_footrest_nodes.items()}

    footrest_panel.visual(
        _segment_mesh("footrest_left_rear_arm_v3", left_footrest_nodes["pivot"], left_footrest_nodes["mid"], footrest_tube_radius),
        material=dark_frame,
        name="left_footrest_arm",
    )
    footrest_panel.visual(
        _segment_mesh("footrest_left_front_arm_v3", left_footrest_nodes["mid"], left_footrest_nodes["front"], footrest_tube_radius),
        material=dark_frame,
    )
    footrest_panel.visual(
        _segment_mesh("footrest_right_rear_arm_v3", right_footrest_nodes["pivot"], right_footrest_nodes["mid"], footrest_tube_radius),
        material=dark_frame,
        name="right_footrest_arm",
    )
    footrest_panel.visual(
        _segment_mesh("footrest_right_front_arm_v3", right_footrest_nodes["mid"], right_footrest_nodes["front"], footrest_tube_radius),
        material=dark_frame,
    )
    footrest_panel.visual(
        _segment_mesh("footrest_rear_crossbar_v3", left_footrest_nodes["pivot"], right_footrest_nodes["pivot"], 0.013),
        material=dark_frame,
        name="footrest_rear_crossbar",
    )
    footrest_panel.visual(
        _segment_mesh("footrest_front_crossbar_v3", left_footrest_nodes["front"], right_footrest_nodes["front"], 0.013),
        material=dark_frame,
        name="footrest_front_crossbar",
    )
    footrest_panel.visual(
        Box((0.30, 0.512, 0.012)),
        origin=Origin(xyz=(0.18, 0.0, -0.045), rpy=(0.0, -0.18, 0.0)),
        material=mesh_fabric,
        name="footrest_sling",
    )
    footrest_panel.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.0, 0.270, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="footrest_left_boss",
    )
    footrest_panel.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.0, -0.270, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pivot_cap,
        name="footrest_right_boss",
    )

    model.articulation(
        "ground_to_recline_frame",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=recline_frame,
        origin=Origin(xyz=main_pivot_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-0.20, upper=1.10),
    )
    model.articulation(
        "ground_to_footrest_panel",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=footrest_panel,
        origin=Origin(xyz=footrest_pivot_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-0.35, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_frame = object_model.get_part("ground_frame")
    recline_frame = object_model.get_part("recline_frame")
    footrest_panel = object_model.get_part("footrest_panel")
    recline_joint = object_model.get_articulation("ground_to_recline_frame")
    footrest_joint = object_model.get_articulation("ground_to_footrest_panel")

    main_pivot_left_lug = ground_frame.get_visual("main_pivot_left_lug")
    main_pivot_right_lug = ground_frame.get_visual("main_pivot_right_lug")
    footrest_left_lug = ground_frame.get_visual("footrest_left_lug")
    footrest_right_lug = ground_frame.get_visual("footrest_right_lug")
    main_pivot_left_boss = recline_frame.get_visual("main_pivot_left_boss")
    main_pivot_right_boss = recline_frame.get_visual("main_pivot_right_boss")
    footrest_left_boss = footrest_panel.get_visual("footrest_left_boss")
    footrest_right_boss = footrest_panel.get_visual("footrest_right_boss")

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

    ctx.check("ground_frame_present", ground_frame is not None, "ground frame missing")
    ctx.check("recline_frame_present", recline_frame is not None, "recline frame missing")
    ctx.check("footrest_panel_present", footrest_panel is not None, "footrest panel missing")

    ctx.check(
        "recline_axis_is_transverse",
        tuple(recline_joint.axis) == (0.0, 1.0, 0.0),
        f"expected recline axis (0,1,0), got {recline_joint.axis}",
    )
    ctx.check(
        "footrest_axis_is_transverse",
        tuple(footrest_joint.axis) == (0.0, 1.0, 0.0),
        f"expected footrest axis (0,1,0), got {footrest_joint.axis}",
    )
    ctx.check(
        "recline_limits_are_plausible",
        recline_joint.motion_limits is not None
        and recline_joint.motion_limits.lower is not None
        and recline_joint.motion_limits.upper is not None
        and recline_joint.motion_limits.lower < 0.0 < recline_joint.motion_limits.upper,
        "recline frame should rotate from slightly upright into a deep recline",
    )
    ctx.check(
        "footrest_limits_are_plausible",
        footrest_joint.motion_limits is not None
        and footrest_joint.motion_limits.lower is not None
        and footrest_joint.motion_limits.upper is not None
        and footrest_joint.motion_limits.lower < 0.0 < footrest_joint.motion_limits.upper,
        "footrest should rotate independently around its front hinge line",
    )

    ctx.expect_origin_gap(
        recline_frame,
        ground_frame,
        axis="z",
        min_gap=0.54,
        max_gap=0.62,
        name="recline_frame_pivot_height",
    )
    ctx.expect_origin_gap(
        footrest_panel,
        recline_frame,
        axis="x",
        min_gap=0.50,
        max_gap=0.62,
        name="footrest_hinge_ahead_of_recline_axis",
    )
    ctx.expect_origin_gap(
        recline_frame,
        footrest_panel,
        axis="z",
        min_gap=0.05,
        max_gap=0.12,
        name="recline_axis_above_footrest_hinge",
    )

    with ctx.pose({recline_joint: 0.0, footrest_joint: 0.0}):
        ctx.expect_contact(
            ground_frame,
            recline_frame,
            elem_a=main_pivot_left_lug,
            elem_b=main_pivot_left_boss,
            contact_tol=0.002,
            name="left_main_pivot_contact",
        )
        ctx.expect_contact(
            ground_frame,
            recline_frame,
            elem_a=main_pivot_right_lug,
            elem_b=main_pivot_right_boss,
            contact_tol=0.002,
            name="right_main_pivot_contact",
        )
        ctx.expect_contact(
            ground_frame,
            footrest_panel,
            elem_a=footrest_left_lug,
            elem_b=footrest_left_boss,
            contact_tol=0.002,
            name="left_footrest_hinge_contact",
        )
        ctx.expect_contact(
            ground_frame,
            footrest_panel,
            elem_a=footrest_right_lug,
            elem_b=footrest_right_boss,
            contact_tol=0.002,
            name="right_footrest_hinge_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
