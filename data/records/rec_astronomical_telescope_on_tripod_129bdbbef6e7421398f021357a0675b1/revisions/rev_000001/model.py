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
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_refractor_altaz_tripod")

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def distance(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def rpy_for_cylinder(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        length_xy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(length_xy, dz)
        return (0.0, pitch, yaw)

    def add_member(
        part,
        a: tuple[float, float, float],
        b: tuple[float, float, float],
        *,
        radius: float,
        material,
        name: str | None = None,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=distance(a, b)),
            origin=Origin(xyz=midpoint(a, b), rpy=rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    white_paint = model.material("white_paint", rgba=(0.91, 0.92, 0.93, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    anodized_gray = model.material("anodized_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.56, 0.74, 0.86, 0.28))
    grip_foam = model.material("grip_foam", rgba=(0.12, 0.12, 0.13, 1.0))

    ota_shell_profile_outer = [
        (0.042, -0.150),
        (0.050, -0.145),
        (0.052, -0.080),
        (0.052, 0.090),
        (0.056, 0.118),
        (0.064, 0.145),
        (0.066, 0.192),
        (0.068, 0.204),
    ]
    ota_shell_profile_inner = [
        (0.036, -0.145),
        (0.044, -0.140),
        (0.046, -0.076),
        (0.046, 0.102),
        (0.051, 0.128),
        (0.059, 0.150),
        (0.061, 0.198),
    ]
    ota_shell_mesh = save_mesh(
        "ota_shell",
        LatheGeometry.from_shell_profiles(
            ota_shell_profile_outer,
            ota_shell_profile_inner,
            segments=64,
        ).rotate_y(math.pi / 2.0),
    )

    side_arm_mesh = save_mesh(
        "mount_side_arm",
        sweep_profile_along_spline(
            [
                (0.000, 0.074, 0.038),
                (0.006, 0.084, 0.074),
                (0.012, 0.086, 0.118),
                (0.010, 0.079, 0.160),
            ],
            profile=rounded_rect_profile(0.040, 0.082, radius=0.011, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    pan_handle_mesh = save_mesh(
        "pan_handle",
        tube_from_spline_points(
            [
                (-0.018, 0.020, 0.072),
                (-0.065, -0.020, 0.098),
                (-0.128, -0.075, 0.116),
            ],
            radius=0.008,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    tripod_support = model.part("tripod_support")
    tripod_support.visual(
        Cylinder(radius=0.070, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        material=dark_metal,
        name="column_cap",
    )
    tripod_support.visual(
        Cylinder(radius=0.045, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        material=anodized_gray,
        name="center_column",
    )
    tripod_support.visual(
        Cylinder(radius=0.058, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.389)),
        material=dark_metal,
        name="tripod_crown",
    )
    tripod_support.visual(
        Cylinder(radius=0.050, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.422)),
        material=satin_black,
        name="threaded_collar",
    )
    tripod_support.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=dark_metal,
        name="spreader_hub",
    )

    for leg_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        ca = math.cos(angle)
        sa = math.sin(angle)
        top = (0.050 * ca, 0.050 * sa, 0.380)
        knee = (0.260 * ca, 0.260 * sa, 0.245)
        foot = (0.520 * ca, 0.520 * sa, 0.024)
        spreader_anchor = (0.238 * ca, 0.238 * sa, 0.280)

        add_member(
            tripod_support,
            (0.034 * ca, 0.034 * sa, 0.388),
            knee,
            radius=0.015,
            material=anodized_gray,
            name=f"upper_leg_{leg_index}",
        )
        add_member(
            tripod_support,
            knee,
            foot,
            radius=0.012,
            material=anodized_gray,
            name=f"lower_leg_{leg_index}",
        )
        tripod_support.visual(
            Box((0.040, 0.024, 0.028)),
            origin=Origin(
                xyz=knee,
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=satin_black,
            name=f"leg_clamp_{leg_index}",
        )
        tripod_support.visual(
            Box((0.050, 0.026, 0.022)),
            origin=Origin(
                xyz=foot,
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"foot_pad_{leg_index}",
        )
        add_member(
            tripod_support,
            (0.018 * ca, 0.018 * sa, 0.304),
            spreader_anchor,
            radius=0.0065,
            material=dark_metal,
            name=f"spreader_bar_{leg_index}",
        )
        tripod_support.visual(
            Cylinder(radius=0.012, length=0.020),
            origin=Origin(xyz=spreader_anchor, rpy=(0.0, math.pi / 2.0, angle)),
            material=satin_black,
            name=f"spreader_joint_{leg_index}",
        )
        tripod_support.visual(
            Box((0.030, 0.020, 0.040)),
            origin=Origin(
                xyz=top,
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=dark_metal,
            name=f"leg_shoulder_{leg_index}",
        )

    tripod_support.inertial = Inertial.from_geometry(
        Box((1.10, 1.10, 0.88)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.078, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=dark_metal,
        name="azimuth_base",
    )
    mount_head.visual(
        Cylinder(radius=0.054, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=satin_black,
        name="azimuth_housing",
    )
    mount_head.visual(
        Box((0.112, 0.050, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.128)),
        material=dark_metal,
        name="mount_shoulder",
    )
    mount_head.visual(side_arm_mesh, material=dark_metal, name="side_arm")
    mount_head.visual(
        Box((0.082, 0.060, 0.076)),
        origin=Origin(xyz=(0.004, 0.076, 0.160)),
        material=dark_metal,
        name="altitude_head",
    )
    mount_head.visual(
        Cylinder(radius=0.031, length=0.100),
        origin=Origin(xyz=(0.000, 0.100, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="altitude_boss",
    )
    mount_head.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.000, 0.040, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="altitude_clutch_knob",
    )
    mount_head.visual(pan_handle_mesh, material=dark_metal, name="pan_handle_tube")
    mount_head.visual(
        Cylinder(radius=0.012, length=0.105),
        origin=Origin(
            xyz=(-0.140, -0.086, 0.118),
            rpy=(math.pi / 2.0, 0.20, 0.0),
        ),
        material=grip_foam,
        name="pan_handle_grip",
    )
    mount_head.inertial = Inertial.from_geometry(
        Box((0.28, 0.24, 0.24)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.03, 0.11)),
    )

    optical_tube = model.part("optical_tube")
    tube_center_y = 0.155
    optical_tube.visual(
        ota_shell_mesh,
        origin=Origin(xyz=(0.018, tube_center_y, 0.0)),
        material=white_paint,
        name="ota_shell",
    )
    optical_tube.visual(
        Cylinder(radius=0.064, length=0.020),
        origin=Origin(
            xyz=(0.206, tube_center_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="objective_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.060, length=0.004),
        origin=Origin(
            xyz=(0.216, tube_center_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="objective_lens",
    )
    optical_tube.visual(
        Cylinder(radius=0.040, length=0.092),
        origin=Origin(
            xyz=(-0.177, tube_center_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=white_paint,
        name="rear_adapter",
    )
    optical_tube.visual(
        Cylinder(radius=0.041, length=0.030),
        origin=Origin(
            xyz=(-0.144, tube_center_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=white_paint,
        name="rear_transition",
    )
    optical_tube.visual(
        Cylinder(radius=0.034, length=0.074),
        origin=Origin(
            xyz=(-0.186, tube_center_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="focuser_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.025, length=0.094),
        origin=Origin(
            xyz=(-0.232, tube_center_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=anodized_gray,
        name="drawtube",
    )
    optical_tube.visual(
        Box((0.050, 0.046, 0.062)),
        origin=Origin(
            xyz=(-0.292, tube_center_y, 0.034),
            rpy=(0.0, -math.pi / 4.0, 0.0),
        ),
        material=satin_black,
        name="diagonal_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.018, length=0.086),
        origin=Origin(xyz=(-0.316, tube_center_y, 0.092)),
        material=satin_black,
        name="eyepiece_barrel",
    )
    optical_tube.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(-0.316, tube_center_y, 0.146)),
        material=rubber,
        name="eyecup",
    )
    optical_tube.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(
            xyz=(-0.182, tube_center_y + 0.041, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="focus_knob_upper",
    )
    optical_tube.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(
            xyz=(-0.182, tube_center_y - 0.041, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="focus_knob_lower",
    )
    optical_tube.visual(
        Box((0.086, 0.040, 0.090)),
        origin=Origin(xyz=(0.000, 0.120, 0.0)),
        material=dark_metal,
        name="saddle_bridge",
    )
    optical_tube.visual(
        Box((0.052, 0.016, 0.014)),
        origin=Origin(xyz=(0.030, tube_center_y, 0.058)),
        material=satin_black,
        name="finder_shoe",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.58, 0.36, 0.26)),
        mass=3.2,
        origin=Origin(xyz=(-0.020, 0.140, 0.010)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod_support,
        child=mount_head,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2),
    )
    model.articulation(
        "altitude_tilt",
        ArticulationType.REVOLUTE,
        parent=mount_head,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.050, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=-0.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_support = object_model.get_part("tripod_support")
    mount_head = object_model.get_part("mount_head")
    optical_tube = object_model.get_part("optical_tube")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    altitude_tilt = object_model.get_articulation("altitude_tilt")

    def aabb_center(aabb) -> tuple[float, float, float]:
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    def planar_radius(point: tuple[float, float, float]) -> float:
        return math.hypot(point[0], point[1])

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
        mount_head,
        tripod_support,
        elem_a="azimuth_base",
        elem_b="column_cap",
        name="mount_head_seats_on_column_cap",
    )
    ctx.expect_contact(
        optical_tube,
        mount_head,
        elem_a="saddle_bridge",
        elem_b="altitude_boss",
        name="optical_tube_contacts_altitude_boss",
    )
    ctx.expect_gap(
        optical_tube,
        tripod_support,
        axis="z",
        min_gap=0.030,
        name="tube_clears_tripod_height",
    )
    ctx.check(
        "azimuth_axis_is_vertical",
        azimuth_rotation.axis == (0.0, 0.0, 1.0),
        f"Expected vertical azimuth axis, got {azimuth_rotation.axis!r}",
    )
    ctx.check(
        "altitude_axis_is_horizontal",
        altitude_tilt.axis == (0.0, -1.0, 0.0),
        f"Expected side-mounted altitude axis, got {altitude_tilt.axis!r}",
    )
    limits = altitude_tilt.motion_limits
    ctx.check(
        "altitude_motion_limits_match_manual_mount",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0
        and limits.upper > 1.0,
        f"Unexpected altitude limits: {limits!r}",
    )

    objective_rest = ctx.part_element_world_aabb(optical_tube, elem="objective_lens")
    assert objective_rest is not None
    objective_rest_center = aabb_center(objective_rest)

    with ctx.pose({altitude_tilt: 1.0}):
        objective_up = ctx.part_element_world_aabb(optical_tube, elem="objective_lens")
        assert objective_up is not None
        objective_up_center = aabb_center(objective_up)
        ctx.check(
            "altitude_motion_raises_objective",
            objective_up_center[2] > objective_rest_center[2] + 0.12,
            (
                "Objective did not rise enough under altitude motion: "
                f"rest={objective_rest_center}, up={objective_up_center}"
            ),
        )
        ctx.expect_contact(
            optical_tube,
            mount_head,
            elem_a="saddle_bridge",
            elem_b="altitude_boss",
            name="tube_stays_mounted_at_high_altitude",
        )

    with ctx.pose({azimuth_rotation: math.pi / 3.0}):
        objective_az = ctx.part_element_world_aabb(optical_tube, elem="objective_lens")
        assert objective_az is not None
        objective_az_center = aabb_center(objective_az)
        ctx.check(
            "azimuth_motion_swings_tube_sideways",
            objective_az_center[0] < objective_rest_center[0] - 0.20
            and objective_az_center[1] > objective_rest_center[1] + 0.06
            and abs(planar_radius(objective_az_center) - planar_radius(objective_rest_center))
            < 0.01,
            (
                "Objective did not swing in azimuth as expected: "
                f"rest={objective_rest_center}, turned={objective_az_center}"
            ),
        )

    with ctx.pose({altitude_tilt: -0.25}):
        ctx.expect_gap(
            optical_tube,
            tripod_support,
            axis="z",
            min_gap=0.020,
            name="tube_clears_tripod_even_near_level_down",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
