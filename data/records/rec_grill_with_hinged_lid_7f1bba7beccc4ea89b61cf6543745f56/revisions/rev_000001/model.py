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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kettle_charcoal_grill")

    enamel_black = model.material("enamel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    aluminum = model.material("aluminum", rgba=(0.77, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    handle_black = model.material("handle_black", rgba=(0.12, 0.10, 0.09, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _y_axis_origin(*, xyz=(0.0, 0.0, 0.0)) -> Origin:
        return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))

    rear_handle_geom = tube_from_spline_points(
        [
            (-0.330, 0.220, 0.130),
            (-0.350, 0.220, 0.400),
            (-0.350, 0.220, 0.660),
            (-0.360, 0.000, 0.820),
            (-0.350, -0.220, 0.660),
            (-0.350, -0.220, 0.400),
            (-0.330, -0.220, 0.130),
        ],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=20,
    )
    front_leg_geom = tube_from_spline_points(
        [(0.235, 0.0, 0.468), (0.260, 0.0, 0.340), (0.330, 0.0, 0.020)],
        radius=0.012,
        samples_per_segment=10,
        radial_segments=18,
    )
    lower_brace_geom = tube_from_spline_points(
        [(-0.330, 0.0, 0.130), (-0.060, 0.0, 0.180), (0.220, 0.0, 0.320)],
        radius=0.009,
        samples_per_segment=10,
        radial_segments=18,
    )
    frame_main_geom = rear_handle_geom.copy()
    frame_main_geom.merge(front_leg_geom)
    frame_main_geom.merge(lower_brace_geom)

    stand_frame = model.part("stand_frame")
    stand_frame.visual(
        _save_mesh("stand_frame_main_v4", frame_main_geom),
        material=satin_black,
        name="stand_frame_main",
    )
    stand_frame.visual(
        Box((0.060, 0.060, 0.024)),
        origin=Origin(xyz=(0.235, 0.0, 0.480)),
        material=satin_black,
        name="front_support_pin",
    )
    stand_frame.visual(
        Cylinder(radius=0.009, length=0.50),
        origin=_y_axis_origin(xyz=(-0.330, 0.0, 0.130)),
        material=satin_black,
        name="axle",
    )
    stand_frame.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.330, 0.0, 0.012)),
        material=satin_black,
        name="front_foot",
    )
    stand_frame.inertial = Inertial.from_geometry(
        Box((0.70, 0.62, 0.84)),
        mass=7.5,
        origin=Origin(xyz=(-0.05, 0.0, 0.38)),
    )

    bowl_shell_profile = [
        (0.282, 0.288),
        (0.266, 0.228),
        (0.238, 0.168),
        (0.160, 0.100),
        (0.075, 0.030),
        (0.030, 0.0),
        (0.0, 0.024),
        (0.060, 0.046),
        (0.148, 0.104),
        (0.226, 0.169),
        (0.254, 0.228),
        (0.274, 0.282),
    ]

    lower_bowl = model.part("lower_bowl")
    lower_bowl.visual(
        _save_mesh(
            "lower_bowl_shell_v2",
            LatheGeometry(bowl_shell_profile, segments=72),
        ),
        material=enamel_black,
        name="bowl_shell",
    )
    lower_bowl.visual(
        Box((0.040, 0.060, 0.024)),
        origin=Origin(xyz=(0.185, 0.0, 0.100)),
        material=satin_black,
        name="front_mount_pad",
    )
    lower_bowl.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=_y_axis_origin(xyz=(-0.282, -0.041, 0.288)),
        material=steel,
        name="left_hinge_ear",
    )
    lower_bowl.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=_y_axis_origin(xyz=(-0.282, 0.041, 0.288)),
        material=steel,
        name="right_hinge_ear",
    )
    lower_bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.29, length=0.30),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    lid_shell_profile = [
        (0.294, 0.0),
        (0.260, 0.075),
        (0.180, 0.155),
        (0.070, 0.190),
        (0.015, 0.195),
        (0.008, 0.187),
        (0.060, 0.181),
        (0.170, 0.150),
        (0.252, 0.075),
        (0.288, 0.006),
    ]

    lid = model.part("lid")
    lid.visual(
        _save_mesh(
            "lid_shell_v3",
            LatheGeometry(lid_shell_profile, segments=72),
        ),
        origin=Origin(xyz=(0.282, 0.0, 0.002)),
        material=enamel_black,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.056),
        origin=_y_axis_origin(),
        material=steel,
        name="hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.420, -0.050, 0.145)),
        material=steel,
        name="handle_left_post",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.420, 0.050, 0.145)),
        material=steel,
        name="handle_right_post",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.120),
        origin=_y_axis_origin(xyz=(0.420, 0.0, 0.170)),
        material=handle_black,
        name="lid_handle_grip",
    )
    lid.visual(
        Box((0.090, 0.052, 0.004)),
        origin=Origin(xyz=(0.420, 0.0, 0.170)),
        material=steel,
        name="handle_heat_shield",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.282, 0.0, 0.194)),
        material=steel,
        name="vent_mount",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.60, 0.60, 0.22)),
        mass=3.0,
        origin=Origin(xyz=(0.286, 0.0, 0.08)),
    )

    vent_damper = model.part("vent_damper")
    vent_damper.visual(
        Cylinder(radius=0.048, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=aluminum,
        name="damper_plate",
    )
    vent_damper.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="damper_collar",
    )
    vent_damper.visual(
        Box((0.026, 0.010, 0.004)),
        origin=Origin(xyz=(0.032, 0.0, 0.006)),
        material=aluminum,
        name="damper_tab",
    )
    vent_damper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=0.01),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.110, length=0.055),
        origin=_y_axis_origin(),
        material=rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.082, length=0.032),
        origin=_y_axis_origin(),
        material=aluminum,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=_y_axis_origin(),
        material=steel,
        name="hub_core",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.055),
        mass=0.9,
        origin=_y_axis_origin(),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.110, length=0.055),
        origin=_y_axis_origin(),
        material=rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.082, length=0.032),
        origin=_y_axis_origin(),
        material=aluminum,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=_y_axis_origin(),
        material=steel,
        name="hub_core",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.055),
        mass=0.9,
        origin=_y_axis_origin(),
    )

    model.articulation(
        "frame_to_bowl",
        ArticulationType.FIXED,
        parent=stand_frame,
        child=lower_bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_bowl,
        child=lid,
        origin=Origin(xyz=(-0.282, 0.0, 0.288)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.95,
            upper=0.0,
        ),
    )
    model.articulation(
        "vent_damper_spin",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=vent_damper,
        origin=Origin(xyz=(0.282, 0.0, 0.197)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=stand_frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.330, 0.280, 0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=15.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=stand_frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.330, -0.280, 0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_frame = object_model.get_part("stand_frame")
    lower_bowl = object_model.get_part("lower_bowl")
    lid = object_model.get_part("lid")
    vent_damper = object_model.get_part("vent_damper")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    vent_damper_spin = object_model.get_articulation("vent_damper_spin")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

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
    ctx.allow_overlap(
        lid,
        lower_bowl,
        elem_a="hinge_barrel",
        elem_b="bowl_shell",
        reason="Rear hinge barrel is captured tight against the bowl rear lip at the pivot line.",
    )
    ctx.allow_overlap(
        lid,
        lower_bowl,
        elem_a="lid_shell",
        elem_b="left_hinge_ear",
        reason="The hidden hinge ear sits inside the lid's unmodeled rear hinge pocket.",
    )
    ctx.allow_overlap(
        lid,
        lower_bowl,
        elem_a="lid_shell",
        elem_b="right_hinge_ear",
        reason="The hidden hinge ear sits inside the lid's unmodeled rear hinge pocket.",
    )
    ctx.allow_overlap(
        lid,
        vent_damper,
        elem_a="vent_mount",
        elem_b="damper_collar",
        reason="The vent damper is clamped through the lid by a shared center fastener.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lid_hinge_axis_is_left_right",
        tuple(lid_hinge.axis) == (0.0, 1.0, 0.0),
        f"expected lid hinge axis (0, 1, 0), got {lid_hinge.axis}",
    )
    ctx.check(
        "wheel_axes_match_axle",
        tuple(left_wheel_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"wheel axes were {left_wheel_spin.axis} and {right_wheel_spin.axis}",
    )
    ctx.check(
        "vent_axis_is_vertical",
        tuple(vent_damper_spin.axis) == (0.0, 0.0, 1.0),
        f"expected vent axis (0, 0, 1), got {vent_damper_spin.axis}",
    )

    ctx.expect_contact(
        stand_frame,
        lower_bowl,
        elem_a="front_support_pin",
        elem_b="front_mount_pad",
        contact_tol=0.001,
    )
    ctx.expect_contact(
        lid,
        lower_bowl,
        elem_a="hinge_barrel",
        elem_b="left_hinge_ear",
        contact_tol=0.005,
    )
    ctx.expect_contact(
        lid,
        lower_bowl,
        elem_a="hinge_barrel",
        elem_b="right_hinge_ear",
        contact_tol=0.005,
    )
    ctx.expect_contact(
        vent_damper,
        lid,
        elem_a="damper_collar",
        elem_b="vent_mount",
        contact_tol=0.001,
    )
    ctx.expect_contact(
        stand_frame,
        left_wheel,
        elem_a="axle",
        elem_b="hub_core",
        contact_tol=0.001,
    )
    ctx.expect_contact(
        stand_frame,
        right_wheel,
        elem_a="axle",
        elem_b="hub_core",
        contact_tol=0.001,
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(lid, lower_bowl, axes="xy", min_overlap=0.52)
        ctx.expect_gap(
            lid,
            lower_bowl,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="bowl_shell",
            max_gap=0.004,
            max_penetration=0.0,
        )

    limits = lid_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
        with ctx.pose({lid_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
            ctx.expect_contact(
                lid,
                lower_bowl,
                elem_a="hinge_barrel",
                elem_b="left_hinge_ear",
                contact_tol=0.005,
                name="lid_upper_left_hinge_contact",
            )
            ctx.expect_contact(
                lid,
                lower_bowl,
                elem_a="hinge_barrel",
                elem_b="right_hinge_ear",
                contact_tol=0.005,
                name="lid_upper_right_hinge_contact",
            )

    bowl_aabb = ctx.part_world_aabb(lower_bowl)
    frame_aabb = ctx.part_world_aabb(stand_frame)
    wheel_aabb = ctx.part_world_aabb(left_wheel)
    handle_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_handle_grip")
    assert bowl_aabb is not None
    assert frame_aabb is not None
    assert wheel_aabb is not None
    assert handle_rest_aabb is not None

    bowl_size_x = bowl_aabb[1][0] - bowl_aabb[0][0]
    bowl_size_z = bowl_aabb[1][2] - bowl_aabb[0][2]
    wheel_diameter = wheel_aabb[1][2] - wheel_aabb[0][2]
    frame_height = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "bowl_scale_realistic",
        0.54 <= bowl_size_x <= 0.60 and 0.27 <= bowl_size_z <= 0.31,
        f"bowl extents were {(bowl_size_x, bowl_size_z)}",
    )
    ctx.check(
        "wheel_scale_realistic",
        0.20 <= wheel_diameter <= 0.24,
        f"wheel diameter was {wheel_diameter}",
    )
    ctx.check(
        "frame_height_realistic",
        frame_height >= 0.82,
        f"frame height was {frame_height}",
    )

    open_angle = limits.lower if limits is not None and limits.lower is not None else -0.95
    with ctx.pose({lid_hinge: open_angle}):
        handle_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_handle_grip")
        assert handle_open_aabb is not None
        ctx.check(
            "lid_opens_upward",
            handle_open_aabb[1][2] > handle_rest_aabb[1][2] + 0.20,
            f"closed handle top z {handle_rest_aabb[1][2]}, open handle top z {handle_open_aabb[1][2]}",
        )
        ctx.check(
            "lid_swings_rearward",
            handle_open_aabb[1][0] < handle_rest_aabb[0][0] - 0.20,
            f"closed handle x range {handle_rest_aabb[0][0], handle_rest_aabb[1][0]}, open handle x range {handle_open_aabb[0][0], handle_open_aabb[1][0]}",
        )

    with ctx.pose(
        {
            left_wheel_spin: math.pi / 2.0,
            right_wheel_spin: -math.pi / 3.0,
            vent_damper_spin: math.pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_and_vent_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_and_vent_pose_no_floating")
        ctx.expect_contact(
            stand_frame,
            left_wheel,
            elem_a="axle",
            elem_b="hub_core",
            contact_tol=0.001,
            name="left_wheel_spun_contact",
        )
        ctx.expect_contact(
            stand_frame,
            right_wheel,
            elem_a="axle",
            elem_b="hub_core",
            contact_tol=0.001,
            name="right_wheel_spun_contact",
        )
        ctx.expect_contact(
            vent_damper,
            lid,
            elem_a="damper_collar",
            elem_b="vent_mount",
            contact_tol=0.001,
            name="damper_spun_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
