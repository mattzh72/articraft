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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solar_observation_refractor")

    tripod_metal = model.material("tripod_metal", rgba=(0.26, 0.28, 0.31, 1.0))
    carbon_fiber = model.material("carbon_fiber", rgba=(0.12, 0.14, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    head_gray = model.material("head_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    head_dark = model.material("head_dark", rgba=(0.21, 0.23, 0.25, 1.0))
    tube_white = model.material("tube_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    anodized_red = model.material("anodized_red", rgba=(0.69, 0.14, 0.10, 1.0))
    solar_filter = model.material("solar_filter", rgba=(0.78, 0.46, 0.12, 0.72))

    def x_axis_origin(
        xyz: tuple[float, float, float],
        *,
        extra_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Origin:
        return Origin(
            xyz=xyz,
            rpy=(
                extra_rpy[0],
                extra_rpy[1] + (math.pi / 2.0),
                extra_rpy[2],
            ),
        )

    def y_axis_origin(
        xyz: tuple[float, float, float],
        *,
        extra_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Origin:
        return Origin(
            xyz=xyz,
            rpy=(
                extra_rpy[0] + (math.pi / 2.0),
                extra_rpy[1],
                extra_rpy[2],
            ),
        )

    main_tube_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=((0.046, -0.26), (0.046, 0.26)),
            inner_profile=((0.041, -0.255), (0.041, 0.255)),
            segments=64,
        ),
        "refractor_main_tube",
    )
    dew_shield_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=((0.057, -0.09), (0.057, 0.09)),
            inner_profile=((0.052, -0.085), (0.052, 0.085)),
            segments=64,
        ),
        "refractor_dew_shield",
    )

    tripod_center = model.part("tripod_center")
    tripod_center.visual(
        Cylinder(radius=0.055, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=tripod_metal,
        name="tripod_crown",
    )
    tripod_center.visual(
        Cylinder(radius=0.092, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 1.068)),
        material=head_dark,
        name="azimuth_seat",
    )
    tripod_center.visual(
        Cylinder(radius=0.022, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=tripod_metal,
        name="spreader_column",
    )
    tripod_center.visual(
        Cylinder(radius=0.05, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=tripod_metal,
        name="tray_hub",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        tripod_center.visual(
            Box((0.09, 0.046, 0.045)),
            origin=Origin(
                xyz=(0.07 * cos_a, 0.07 * sin_a, 0.968),
                rpy=(0.0, 0.0, angle),
            ),
            material=tripod_metal,
            name=f"leg_socket_{index}",
        )
        tripod_center.visual(
            Box((0.22, 0.05, 0.008)),
            origin=Origin(
                xyz=(0.15 * cos_a, 0.15 * sin_a, 0.60),
                rpy=(0.0, 0.0, angle),
            ),
            material=tripod_metal,
            name=f"tray_arm_{index}",
        )
    tripod_center.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 1.11)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
    )

    for index in range(3):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.019, length=0.06),
            origin=Origin(xyz=(0.0, 0.0, 0.03)),
            material=tripod_metal,
            name="top_ferrule",
        )
        leg.visual(
            Cylinder(radius=0.019, length=0.58),
            origin=Origin(xyz=(0.0, 0.0, -0.29)),
            material=carbon_fiber,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.014, length=0.52),
            origin=Origin(xyz=(0.0, 0.0, -0.75)),
            material=carbon_fiber,
            name="lower_tube",
        )
        leg.visual(
            Cylinder(radius=0.026, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, -0.485)),
            material=tripod_metal,
            name="leg_clamp",
        )
        leg.visual(
            Sphere(radius=0.03),
            origin=Origin(xyz=(0.0, 0.0, -1.024)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.07, 0.07, 1.12)),
            mass=0.72,
            origin=Origin(xyz=(0.0, 0.0, -0.50)),
        )

        leg_angle = index * (2.0 * math.pi / 3.0)
        model.articulation(
            f"tripod_center_to_leg_{index}",
            ArticulationType.FIXED,
            parent=tripod_center,
            child=leg,
            origin=Origin(
                xyz=(0.07 * math.cos(leg_angle), 0.07 * math.sin(leg_angle), 0.968),
                rpy=(0.0, -math.radians(25.0), leg_angle),
            ),
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.094, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=head_dark,
        name="turntable_plate",
    )
    head.visual(
        Cylinder(radius=0.076, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=head_gray,
        name="azimuth_drum",
    )
    head.visual(
        Box((0.09, 0.085, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=head_gray,
        name="center_column",
    )
    head.visual(
        Box((0.14, 0.125, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=head_gray,
        name="yoke_base",
    )
    head.visual(
        Box((0.12, 0.024, 0.18)),
        origin=Origin(xyz=(0.0, 0.07, 0.25)),
        material=head_gray,
        name="left_arm",
    )
    head.visual(
        Box((0.12, 0.024, 0.18)),
        origin=Origin(xyz=(0.0, -0.07, 0.25)),
        material=head_gray,
        name="right_arm",
    )
    head.visual(
        Box((0.05, 0.14, 0.036)),
        origin=Origin(xyz=(-0.04, 0.0, 0.147)),
        material=head_dark,
        name="rear_brace",
    )
    head.visual(
        Cylinder(radius=0.04, length=0.022),
        origin=y_axis_origin((0.0, 0.069, 0.25)),
        material=head_dark,
        name="left_altitude_bearing",
    )
    head.visual(
        Cylinder(radius=0.04, length=0.022),
        origin=y_axis_origin((0.0, -0.069, 0.25)),
        material=head_dark,
        name="right_altitude_bearing",
    )
    head.visual(
        Box((0.055, 0.05, 0.045)),
        origin=Origin(xyz=(-0.02, 0.091, 0.205)),
        material=head_dark,
        name="altitude_control_housing",
    )
    head.visual(
        Box((0.04, 0.06, 0.04)),
        origin=Origin(xyz=(0.09, 0.0, 0.07)),
        material=head_dark,
        name="azimuth_control_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.19, 0.36)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        main_tube_mesh,
        origin=x_axis_origin((0.04, 0.0, 0.0)),
        material=tube_white,
        name="main_tube_shell",
    )
    optical_tube.visual(
        dew_shield_mesh,
        origin=x_axis_origin((0.35, 0.0, 0.0)),
        material=trim_black,
        name="dew_shield_shell",
    )
    optical_tube.visual(
        Cylinder(radius=0.052, length=0.09),
        origin=x_axis_origin((0.315, 0.0, 0.0)),
        material=trim_black,
        name="front_coupling_sleeve",
    )
    optical_tube.visual(
        Cylinder(radius=0.062, length=0.04),
        origin=x_axis_origin((0.445, 0.0, 0.0)),
        material=anodized_red,
        name="objective_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.056, length=0.008),
        origin=x_axis_origin((0.466, 0.0, 0.0)),
        material=solar_filter,
        name="front_filter",
    )
    optical_tube.visual(
        Cylinder(radius=0.05, length=0.06),
        origin=x_axis_origin((-0.25, 0.0, 0.0)),
        material=trim_black,
        name="rear_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.033, length=0.10),
        origin=x_axis_origin((-0.33, 0.0, 0.0)),
        material=trim_black,
        name="focuser_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.02, length=0.10),
        origin=x_axis_origin((-0.43, 0.0, 0.0)),
        material=head_gray,
        name="drawtube",
    )
    optical_tube.visual(
        Cylinder(radius=0.017, length=0.04),
        origin=x_axis_origin((-0.50, 0.0, 0.0)),
        material=trim_black,
        name="diagonal_nosepiece",
    )
    optical_tube.visual(
        Box((0.045, 0.032, 0.055)),
        origin=Origin(xyz=(-0.54, 0.0, 0.012), rpy=(0.0, math.radians(40.0), 0.0)),
        material=trim_black,
        name="diagonal_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.014, length=0.08),
        origin=Origin(xyz=(-0.565, 0.0, 0.04)),
        material=head_gray,
        name="eyepiece_barrel",
    )
    optical_tube.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(-0.565, 0.0, 0.095)),
        material=rubber,
        name="eyecup",
    )
    optical_tube.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=x_axis_origin((-0.05, 0.0, 0.0)),
        material=head_dark,
        name="rear_tube_ring",
    )
    optical_tube.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=x_axis_origin((0.055, 0.0, 0.0)),
        material=head_dark,
        name="front_tube_ring",
    )
    optical_tube.visual(
        Box((0.14, 0.082, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=head_dark,
        name="mounting_cradle",
    )
    optical_tube.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=y_axis_origin((0.0, 0.046, 0.0)),
        material=head_dark,
        name="left_trunnion",
    )
    optical_tube.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=y_axis_origin((0.0, -0.046, 0.0)),
        material=head_dark,
        name="right_trunnion",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.80, 0.14, 0.18)),
        mass=2.3,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    altitude_knob = model.part("altitude_knob")
    altitude_knob.visual(
        Cylinder(radius=0.01, length=0.006),
        origin=y_axis_origin((0.0, 0.003, 0.0)),
        material=head_dark,
        name="collar",
    )
    altitude_knob.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=y_axis_origin((0.0, 0.013, 0.0)),
        material=head_dark,
        name="knob_body",
    )
    altitude_knob.visual(
        Cylinder(radius=0.003, length=0.026),
        origin=x_axis_origin((0.0, 0.018, 0.014)),
        material=head_gray,
        name="knob_handle",
    )
    altitude_knob.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, 0.04)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
    )

    azimuth_knob = model.part("azimuth_knob")
    azimuth_knob.visual(
        Cylinder(radius=0.01, length=0.006),
        origin=x_axis_origin((0.003, 0.0, 0.0)),
        material=head_dark,
        name="collar",
    )
    azimuth_knob.visual(
        Cylinder(radius=0.02, length=0.014),
        origin=x_axis_origin((0.013, 0.0, 0.0)),
        material=head_dark,
        name="knob_body",
    )
    azimuth_knob.visual(
        Cylinder(radius=0.003, length=0.03),
        origin=Origin(xyz=(0.018, 0.015, 0.0)),
        material=head_gray,
        name="knob_handle",
    )
    azimuth_knob.inertial = Inertial.from_geometry(
        Box((0.04, 0.04, 0.04)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    model.articulation(
        "head_azimuth",
        ArticulationType.REVOLUTE,
        parent=tripod_center,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "tube_altitude",
        ArticulationType.REVOLUTE,
        parent=head,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "altitude_fine_adjust",
        ArticulationType.REVOLUTE,
        parent=head,
        child=altitude_knob,
        origin=Origin(xyz=(-0.02, 0.116, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=12.0,
            lower=-6.0 * math.pi,
            upper=6.0 * math.pi,
        ),
    )
    model.articulation(
        "azimuth_fine_adjust",
        ArticulationType.REVOLUTE,
        parent=head,
        child=azimuth_knob,
        origin=Origin(xyz=(0.11, 0.0, 0.07)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=12.0,
            lower=-6.0 * math.pi,
            upper=6.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    def require_part(name: str):
        try:
            part = object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - defensive test plumbing
            ctx.check(f"part_present_{name}", False, str(exc))
            return None
        ctx.check(f"part_present_{name}", True)
        return part

    def require_joint(name: str):
        try:
            articulation = object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - defensive test plumbing
            ctx.check(f"joint_present_{name}", False, str(exc))
            return None
        ctx.check(f"joint_present_{name}", True)
        return articulation

    tripod_center = require_part("tripod_center")
    head = require_part("head")
    optical_tube = require_part("optical_tube")
    altitude_knob = require_part("altitude_knob")
    azimuth_knob = require_part("azimuth_knob")
    legs = [require_part(f"leg_{index}") for index in range(3)]

    head_azimuth = require_joint("head_azimuth")
    tube_altitude = require_joint("tube_altitude")
    altitude_fine_adjust = require_joint("altitude_fine_adjust")
    azimuth_fine_adjust = require_joint("azimuth_fine_adjust")

    if tripod_center is not None:
        for leg in legs:
            if leg is not None:
                ctx.allow_overlap(
                    tripod_center,
                    leg,
                    reason="Tripod leg ferrule is intentionally nested into the crown socket.",
                )

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

    if (
        tripod_center is not None
        and head is not None
        and optical_tube is not None
        and altitude_knob is not None
        and azimuth_knob is not None
    ):
        ctx.expect_gap(
            head,
            tripod_center,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="head_seats_on_tripod_azimuth_plate",
        )
        ctx.expect_overlap(
            head,
            tripod_center,
            axes="xy",
            min_overlap=0.12,
            name="head_bearing_overlaps_tripod_planform",
        )
        ctx.expect_contact(
            optical_tube,
            head,
            elem_a="left_trunnion",
            elem_b="left_altitude_bearing",
            name="left_trunnion_contacts_left_bearing",
        )
        ctx.expect_contact(
            optical_tube,
            head,
            elem_a="right_trunnion",
            elem_b="right_altitude_bearing",
            name="right_trunnion_contacts_right_bearing",
        )
        ctx.expect_contact(altitude_knob, head, name="altitude_knob_is_mounted")
        ctx.expect_contact(azimuth_knob, head, name="azimuth_knob_is_mounted")

    if tripod_center is not None:
        for leg in legs:
            if leg is not None:
                ctx.expect_contact(
                    leg,
                    tripod_center,
                    name=f"{leg.name}_is_socketed_into_tripod_crown",
                )
                foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
                if foot_aabb is None:
                    ctx.check(f"{leg.name}_foot_pad_exists", False, "No foot_pad AABB")
                else:
                    ctx.check(
                        f"{leg.name}_foot_reaches_ground",
                        abs(foot_aabb[0][2]) <= 0.01,
                        f"Foot bottom z={foot_aabb[0][2]:.4f} m",
                    )

    if (
        head_azimuth is not None
        and tube_altitude is not None
        and altitude_fine_adjust is not None
        and azimuth_fine_adjust is not None
    ):
        ctx.check(
            "azimuth_axis_is_vertical",
            tuple(head_azimuth.axis) == (0.0, 0.0, 1.0),
            f"axis={head_azimuth.axis}",
        )
        ctx.check(
            "altitude_axis_is_horizontal",
            tuple(tube_altitude.axis) == (0.0, -1.0, 0.0),
            f"axis={tube_altitude.axis}",
        )
        ctx.check(
            "altitude_knob_axis_matches_side_mount",
            tuple(altitude_fine_adjust.axis) == (0.0, 1.0, 0.0),
            f"axis={altitude_fine_adjust.axis}",
        )
        ctx.check(
            "azimuth_knob_axis_matches_front_mount",
            tuple(azimuth_fine_adjust.axis) == (1.0, 0.0, 0.0),
            f"axis={azimuth_fine_adjust.axis}",
        )

    if optical_tube is not None and tube_altitude is not None:
        objective_rest = ctx.part_element_world_aabb(optical_tube, elem="objective_cell")
        if objective_rest is None:
            ctx.check("objective_cell_exists", False, "No objective_cell AABB")
        else:
            rest_center = aabb_center(objective_rest)
            with ctx.pose({tube_altitude: math.radians(55.0)}):
                objective_raised = ctx.part_element_world_aabb(optical_tube, elem="objective_cell")
                if objective_raised is None:
                    ctx.check("objective_cell_moves_with_altitude_pose", False, "No raised objective AABB")
                else:
                    raised_center = aabb_center(objective_raised)
                    ctx.check(
                        "altitude_pose_raises_front_of_refractor",
                        raised_center[2] > rest_center[2] + 0.25,
                        f"rest_z={rest_center[2]:.4f}, raised_z={raised_center[2]:.4f}",
                    )
                if head is not None:
                    ctx.expect_contact(
                        optical_tube,
                        head,
                        elem_a="left_trunnion",
                        elem_b="left_altitude_bearing",
                        name="left_trunnion_remains_supported_in_raised_pose",
                    )
                    ctx.expect_contact(
                        optical_tube,
                        head,
                        elem_a="right_trunnion",
                        elem_b="right_altitude_bearing",
                        name="right_trunnion_remains_supported_in_raised_pose",
                    )

    if optical_tube is not None and head_azimuth is not None:
        objective_rest = ctx.part_element_world_aabb(optical_tube, elem="objective_cell")
        if objective_rest is not None:
            rest_center = aabb_center(objective_rest)
            with ctx.pose({head_azimuth: math.radians(60.0)}):
                objective_swung = ctx.part_element_world_aabb(optical_tube, elem="objective_cell")
                if objective_swung is None:
                    ctx.check("objective_cell_moves_with_azimuth_pose", False, "No azimuthed objective AABB")
                else:
                    swung_center = aabb_center(objective_swung)
                    ctx.check(
                        "azimuth_pose_swings_tube_around_vertical_axis",
                        swung_center[1] > rest_center[1] + 0.30 and swung_center[0] < rest_center[0] - 0.15,
                        (
                            f"rest_xy=({rest_center[0]:.4f}, {rest_center[1]:.4f}), "
                            f"swung_xy=({swung_center[0]:.4f}, {swung_center[1]:.4f})"
                        ),
                    )

    if altitude_knob is not None and altitude_fine_adjust is not None:
        handle_rest = ctx.part_element_world_aabb(altitude_knob, elem="knob_handle")
        if handle_rest is None:
            ctx.check("altitude_knob_handle_exists", False, "No altitude knob handle AABB")
        else:
            rest_center = aabb_center(handle_rest)
            with ctx.pose({altitude_fine_adjust: 1.2}):
                handle_turned = ctx.part_element_world_aabb(altitude_knob, elem="knob_handle")
                if handle_turned is None:
                    ctx.check("altitude_knob_rotates", False, "No turned altitude knob handle AABB")
                else:
                    turned_center = aabb_center(handle_turned)
                    ctx.check(
                        "altitude_knob_pose_turns_handle",
                        abs(turned_center[0] - rest_center[0]) > 0.01
                        and abs(turned_center[2] - rest_center[2]) > 0.004,
                        (
                            f"rest_xz=({rest_center[0]:.4f}, {rest_center[2]:.4f}), "
                            f"turned_xz=({turned_center[0]:.4f}, {turned_center[2]:.4f})"
                        ),
                    )

    if azimuth_knob is not None and azimuth_fine_adjust is not None:
        handle_rest = ctx.part_element_world_aabb(azimuth_knob, elem="knob_handle")
        if handle_rest is None:
            ctx.check("azimuth_knob_handle_exists", False, "No azimuth knob handle AABB")
        else:
            rest_center = aabb_center(handle_rest)
            with ctx.pose({azimuth_fine_adjust: 1.2}):
                handle_turned = ctx.part_element_world_aabb(azimuth_knob, elem="knob_handle")
                if handle_turned is None:
                    ctx.check("azimuth_knob_rotates", False, "No turned azimuth knob handle AABB")
                else:
                    turned_center = aabb_center(handle_turned)
                    ctx.check(
                        "azimuth_knob_pose_turns_handle",
                        math.hypot(
                            turned_center[1] - rest_center[1],
                            turned_center[2] - rest_center[2],
                        )
                        > 0.015,
                        (
                            f"rest_yz=({rest_center[1]:.4f}, {rest_center[2]:.4f}), "
                            f"turned_yz=({turned_center[1]:.4f}, {turned_center[2]:.4f})"
                        ),
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
