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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    corner_radius: float,
    z_shift: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_val, z_val + z_shift)
        for y_val, z_val in rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_dropper_seatpost")

    receiver_black = model.material("receiver_black", rgba=(0.11, 0.11, 0.12, 1.0))
    post_black = model.material("post_black", rgba=(0.16, 0.16, 0.17, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.26, 0.26, 0.28, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    saddle_cover = model.material("saddle_cover", rgba=(0.08, 0.08, 0.09, 1.0))

    seat_tube_receiver = model.part("seat_tube_receiver")
    seat_tube_receiver.visual(
        _shell_mesh(
            outer_profile=[
                (0.0185, 0.0),
                (0.0185, 0.048),
                (0.0225, 0.048),
                (0.0225, 0.070),
            ],
            inner_profile=[
                (0.01635, 0.0),
                (0.01635, 0.070),
            ],
            name="seat_tube_receiver_shell",
        ),
        material=receiver_black,
        name="seat_tube_shell",
    )
    seat_tube_receiver.visual(
        Box((0.010, 0.014, 0.016)),
        origin=Origin(xyz=(0.027, 0.010, 0.059)),
        material=receiver_black,
        name="binder_ear_front",
    )
    seat_tube_receiver.visual(
        Box((0.010, 0.014, 0.016)),
        origin=Origin(xyz=(0.027, -0.010, 0.059)),
        material=receiver_black,
        name="binder_ear_rear",
    )
    seat_tube_receiver.visual(
        Cylinder(radius=0.0034, length=0.030),
        origin=Origin(xyz=(0.031, 0.0, 0.059), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="binder_bolt",
    )
    seat_tube_receiver.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.031, 0.018, 0.059), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="binder_bolt_head",
    )
    seat_tube_receiver.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.031, -0.018, 0.059), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="binder_bolt_nut",
    )
    seat_tube_receiver.inertial = Inertial.from_geometry(
        Box((0.065, 0.065, 0.080)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _shell_mesh(
            outer_profile=[
                (0.0159, 0.0),
                (0.0159, 0.272),
            ],
            inner_profile=[
                (0.0135, 0.0),
                (0.0135, 0.272),
            ],
            name="outer_tube_shell",
        ),
        material=post_black,
        name="outer_shell",
    )
    outer_tube.visual(
        _shell_mesh(
            outer_profile=[
                (0.0188, 0.070),
                (0.0188, 0.076),
            ],
            inner_profile=[
                (0.0136, 0.070),
                (0.0136, 0.076),
            ],
            name="outer_tube_lower_stop_collar",
        ),
        material=seal_gray,
        name="lower_stop_collar",
    )
    outer_tube.visual(
        _shell_mesh(
            outer_profile=[
                (0.0176, 0.256),
                (0.0176, 0.272),
            ],
            inner_profile=[
                (0.0139, 0.256),
                (0.0139, 0.272),
            ],
            name="outer_tube_wiper_head",
        ),
        material=seal_gray,
        name="wiper_head",
    )
    outer_tube.visual(
        Box((0.014, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.0185, 0.148)),
        material=post_black,
        name="cable_port_boss",
    )
    outer_tube.visual(
        Cylinder(radius=0.0046, length=0.010),
        origin=Origin(xyz=(0.0, 0.023, 0.148), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seal_gray,
        name="cable_port",
    )
    outer_tube.visual(
        Box((0.016, 0.011, 0.020)),
        origin=Origin(xyz=(-0.0185, 0.0, 0.103)),
        material=post_black,
        name="cable_anchor_body",
    )
    outer_tube.visual(
        Cylinder(radius=0.0038, length=0.014),
        origin=Origin(xyz=(-0.024, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="cable_anchor_barrel",
    )
    outer_tube.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(-0.0185, 0.0, 0.114), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cable_anchor_screw",
    )
    outer_tube.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.024, 0.0, 0.103),
                    (-0.022, 0.010, 0.114),
                    (-0.014, 0.018, 0.128),
                    (0.0, 0.022, 0.145),
                ],
                radius=0.0024,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            "outer_tube_cable_housing",
        ),
        material=seal_gray,
        name="cable_housing",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Box((0.042, 0.042, 0.282)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0132, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=satin_black,
        name="stanchion",
    )
    inner_post.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.352)),
        material=post_black,
        name="crown_spine",
    )
    inner_post.visual(
        Box((0.062, 0.036, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.374)),
        material=post_black,
        name="clamp_bridge",
    )
    inner_post.visual(
        Box((0.088, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, 0.022, 0.381)),
        material=steel,
        name="left_lower_saddle",
    )
    inner_post.visual(
        Box((0.088, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, -0.022, 0.381)),
        material=steel,
        name="right_lower_saddle",
    )
    inner_post.visual(
        Box((0.078, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.022, 0.4126)),
        material=steel,
        name="left_upper_clamp",
    )
    inner_post.visual(
        Box((0.078, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.022, 0.4126)),
        material=steel,
        name="right_upper_clamp",
    )
    inner_post.visual(
        Box((0.026, 0.038, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.4126)),
        material=steel,
        name="upper_center_bridge",
    )
    inner_post.visual(
        Box((0.020, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.398)),
        material=steel,
        name="upper_pedestal",
    )
    inner_post.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(xyz=(0.030, 0.0, 0.399)),
        material=steel,
        name="front_bolt",
    )
    inner_post.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(xyz=(-0.030, 0.0, 0.399)),
        material=steel,
        name="rear_bolt",
    )
    inner_post.visual(
        Cylinder(radius=0.0052, length=0.005),
        origin=Origin(xyz=(0.030, 0.0, 0.4165)),
        material=steel,
        name="front_bolt_head",
    )
    inner_post.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.030, 0.0, 0.382)),
        material=steel,
        name="front_bolt_nut",
    )
    inner_post.visual(
        Cylinder(radius=0.0052, length=0.005),
        origin=Origin(xyz=(-0.030, 0.0, 0.4165)),
        material=steel,
        name="rear_bolt_head",
    )
    inner_post.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(-0.030, 0.0, 0.382)),
        material=steel,
        name="rear_bolt_nut",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.095, 0.055, 0.620)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(-0.125, 0.138, 0.020, 0.009, 0.052),
                    _yz_section(-0.060, 0.148, 0.023, 0.010, 0.056),
                    _yz_section(0.015, 0.118, 0.024, 0.010, 0.060),
                    _yz_section(0.085, 0.074, 0.021, 0.009, 0.057),
                    _yz_section(0.136, 0.054, 0.017, 0.007, 0.053),
                ]
            ),
            "saddle_shell_body",
        ),
        material=saddle_cover,
        name="shell",
    )
    saddle.visual(
        Box((0.020, 0.034, 0.006)),
        origin=Origin(xyz=(0.136, 0.0, 0.053)),
        material=saddle_cover,
        name="nose_pad",
    )
    saddle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.115, 0.022, 0.018),
                    (-0.075, 0.022, 0.008),
                    (-0.035, 0.022, 0.001),
                    (-0.015, 0.022, 0.0),
                    (0.015, 0.022, 0.0),
                    (0.045, 0.022, 0.001),
                    (0.095, 0.022, 0.008),
                    (0.120, 0.022, 0.015),
                ],
                radius=0.0035,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
            "saddle_left_rail",
        ),
        material=steel,
        name="left_rail",
    )
    saddle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.115, -0.022, 0.018),
                    (-0.075, -0.022, 0.008),
                    (-0.035, -0.022, 0.001),
                    (-0.015, -0.022, 0.0),
                    (0.015, -0.022, 0.0),
                    (0.045, -0.022, 0.001),
                    (0.095, -0.022, 0.008),
                    (0.120, -0.022, 0.015),
                ],
                radius=0.0035,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
            "saddle_right_rail",
        ),
        material=steel,
        name="right_rail",
    )
    saddle.visual(
        Cylinder(radius=0.0045, length=0.044),
        origin=Origin(xyz=(-0.055, 0.022, 0.023)),
        material=steel,
        name="rear_left_support",
    )
    saddle.visual(
        Cylinder(radius=0.0045, length=0.044),
        origin=Origin(xyz=(-0.055, -0.022, 0.023)),
        material=steel,
        name="rear_right_support",
    )
    saddle.visual(
        Cylinder(radius=0.0045, length=0.046),
        origin=Origin(xyz=(0.060, 0.022, 0.024)),
        material=steel,
        name="front_left_support",
    )
    saddle.visual(
        Cylinder(radius=0.0045, length=0.046),
        origin=Origin(xyz=(0.060, -0.022, 0.024)),
        material=steel,
        name="front_right_support",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.275, 0.155, 0.100)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "receiver_to_outer_tube",
        ArticulationType.FIXED,
        parent=seat_tube_receiver,
        child=outer_tube,
        origin=Origin(),
    )
    model.articulation(
        "outer_tube_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=0.150,
        ),
    )
    model.articulation(
        "inner_post_to_saddle",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.388)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    seat_tube_receiver = object_model.get_part("seat_tube_receiver")
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    saddle = object_model.get_part("saddle")
    dropper = object_model.get_articulation("outer_tube_to_inner_post")
    saddle_tilt = object_model.get_articulation("inner_post_to_saddle")

    seat_tube_receiver.get_visual("seat_tube_shell")
    outer_tube.get_visual("outer_shell")
    outer_tube.get_visual("lower_stop_collar")
    outer_tube.get_visual("cable_port")
    outer_tube.get_visual("cable_anchor_body")
    inner_post.get_visual("stanchion")
    inner_post.get_visual("left_lower_saddle")
    inner_post.get_visual("right_lower_saddle")
    inner_post.get_visual("left_upper_clamp")
    inner_post.get_visual("right_upper_clamp")
    saddle.get_visual("shell")
    saddle.get_visual("left_rail")
    saddle.get_visual("right_rail")
    saddle.get_visual("nose_pad")

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
        seat_tube_receiver,
        outer_tube,
        elem_a="seat_tube_shell",
        elem_b="lower_stop_collar",
        contact_tol=0.0003,
        name="outer tube collar seats on the binder receiver",
    )
    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_shell",
        margin=0.003,
        name="stanchion stays centered in the outer tube",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_shell",
        min_overlap=0.18,
        name="stanchion remains inserted at rest",
    )
    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="left_rail",
        elem_b="left_lower_saddle",
        contact_tol=0.0015,
        name="left saddle rail seats on the lower clamp plate",
    )
    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="right_rail",
        elem_b="right_lower_saddle",
        contact_tol=0.0015,
        name="right saddle rail seats on the lower clamp plate",
    )
    ctx.expect_gap(
        inner_post,
        saddle,
        axis="z",
        positive_elem="left_upper_clamp",
        negative_elem="left_rail",
        max_gap=0.002,
        max_penetration=0.0008,
        name="left upper clamp captures the left rail",
    )
    ctx.expect_gap(
        inner_post,
        saddle,
        axis="z",
        positive_elem="right_upper_clamp",
        negative_elem="right_rail",
        max_gap=0.002,
        max_penetration=0.0008,
        name="right upper clamp captures the right rail",
    )

    drop_upper = dropper.motion_limits.upper if dropper.motion_limits is not None else 0.15
    rest_pos = ctx.part_world_position(inner_post)
    with ctx.pose({dropper: drop_upper or 0.15}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_shell",
            margin=0.003,
            name="extended stanchion stays centered in the outer tube",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_shell",
            min_overlap=0.045,
            name="extended stanchion retains insertion in the outer tube",
        )
        extended_pos = ctx.part_world_position(inner_post)
    ctx.check(
        "dropper extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    tilt_upper = saddle_tilt.motion_limits.upper if saddle_tilt.motion_limits is not None else 0.22
    rest_nose_center = None
    raised_nose_center = None
    with ctx.pose({saddle_tilt: 0.0}):
        rest_nose_aabb = ctx.part_element_world_aabb(saddle, elem="nose_pad")
        if rest_nose_aabb is not None:
            rest_nose_center = 0.5 * (rest_nose_aabb[0][2] + rest_nose_aabb[1][2])
    with ctx.pose({saddle_tilt: tilt_upper or 0.22}):
        raised_nose_aabb = ctx.part_element_world_aabb(saddle, elem="nose_pad")
        if raised_nose_aabb is not None:
            raised_nose_center = 0.5 * (raised_nose_aabb[0][2] + raised_nose_aabb[1][2])
    ctx.check(
        "positive tilt raises the saddle nose",
        rest_nose_center is not None
        and raised_nose_center is not None
        and raised_nose_center > rest_nose_center + 0.012,
        details=f"rest_nose_z={rest_nose_center}, raised_nose_z={raised_nose_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
