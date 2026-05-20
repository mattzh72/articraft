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
    wire_from_points,
)


DEFAULT_LATITUDE = math.radians(34.0)


def _rotate_pitch_yaw(
    vec: tuple[float, float, float],
    *,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    x, y, z = vec
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    x_pitch = cp * x + sp * z
    z_pitch = -sp * x + cp * z

    return (
        cy * x_pitch - sy * y,
        sy * x_pitch + cy * y,
        z_pitch,
    )


def _tube_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    flare: float = 0.0,
) -> object:
    half = length * 0.5
    outer = [
        (outer_radius + flare * 0.15, -half),
        (outer_radius + flare, -half + min(length * 0.12, 0.016)),
        (outer_radius, half - min(length * 0.08, 0.012)),
        (outer_radius, half),
    ]
    inner = [
        (inner_radius, -half + 0.0015),
        (inner_radius, half - 0.0015),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="astrograph_equatorial_wedge_mount")

    tripod_black = model.material("tripod_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.67, 0.69, 0.72, 1.0))
    tube_white = model.material("tube_white", rgba=(0.92, 0.93, 0.95, 1.0))
    anodized_red = model.material("anodized_red", rgba=(0.56, 0.12, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.53, 0.68, 0.82, 0.35))

    main_tube_mesh = _tube_shell_mesh(
        "astro_main_tube_shell",
        outer_radius=0.049,
        inner_radius=0.046,
        length=0.360,
    )
    dew_shield_mesh = _tube_shell_mesh(
        "astro_dew_shield_shell",
        outer_radius=0.058,
        inner_radius=0.055,
        length=0.140,
        flare=0.002,
    )
    tube_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.051, tube=0.006, radial_segments=16, tubular_segments=44),
        "astro_tube_ring",
    )
    belt_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (-0.036, 0.074, 0.075),
                (0.000, 0.078, 0.075),
                (0.074, 0.000, 0.075),
                (0.000, -0.078, 0.075),
                (-0.036, -0.074, 0.075),
                (-0.108, -0.022, 0.075),
                (-0.134, 0.000, 0.075),
                (-0.108, 0.022, 0.075),
            ],
            radius=0.0023,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.020,
            cap_ends=False,
        ),
        "astro_ra_belt",
    )

    tripod_hub = model.part("tripod_hub")
    tripod_hub.visual(
        Cylinder(radius=0.096, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.865)),
        material=tripod_black,
        name="crown",
    )
    tripod_hub.visual(
        Cylinder(radius=0.045, length=0.226),
        origin=Origin(xyz=(0.0, 0.0, 0.995)),
        material=dark_gray,
        name="center_column",
    )
    tripod_hub.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.118)),
        material=graphite,
        name="pier_cap",
    )
    tripod_hub.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.30)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
    )

    wedge_base = model.part("wedge_base")
    wedge_base.visual(
        Box((0.190, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_gray,
        name="base_plate",
    )
    wedge_base.visual(
        Box((0.120, 0.012, 0.092)),
        origin=Origin(xyz=(0.015, 0.056, 0.064)),
        material=graphite,
        name="left_cheek",
    )
    wedge_base.visual(
        Box((0.120, 0.012, 0.092)),
        origin=Origin(xyz=(0.015, -0.056, 0.064)),
        material=graphite,
        name="right_cheek",
    )
    wedge_base.visual(
        Box((0.050, 0.120, 0.040)),
        origin=Origin(xyz=(-0.055, 0.0, 0.020)),
        material=graphite,
        name="front_hinge_block",
    )
    wedge_base.visual(
        Box((0.034, 0.080, 0.022)),
        origin=Origin(xyz=(0.032, 0.0, 0.011)),
        material=graphite,
        name="rear_adjust_block",
    )
    wedge_base.visual(
        Cylinder(radius=0.008, length=0.074),
        origin=Origin(xyz=(0.044, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="latitude_adjust_screw",
    )
    wedge_base.inertial = Inertial.from_geometry(
        Box((0.190, 0.150, 0.110)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    wedge_head = model.part("wedge_head")
    wedge_head.visual(
        Cylinder(radius=0.011, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="latitude_pivot_shaft",
    )
    wedge_head.visual(
        Box((0.060, 0.010, 0.090)),
        origin=Origin(xyz=(0.020, 0.037, 0.050)),
        material=dark_gray,
        name="left_head_arm",
    )
    wedge_head.visual(
        Box((0.060, 0.010, 0.090)),
        origin=Origin(xyz=(0.020, -0.037, 0.050)),
        material=dark_gray,
        name="right_head_arm",
    )
    wedge_head.visual(
        Box((0.220, 0.080, 0.010)),
        origin=Origin(xyz=(0.090, 0.0, 0.088)),
        material=satin_silver,
        name="tilt_plate",
    )
    wedge_head.visual(
        Box((0.110, 0.090, 0.018)),
        origin=Origin(xyz=(0.108, 0.0, 0.079)),
        material=graphite,
        name="polar_spine",
    )
    wedge_head.visual(
        Box((0.050, 0.060, 0.018)),
        origin=Origin(xyz=(0.172, 0.0, 0.079)),
        material=graphite,
        name="rear_stiffener",
    )
    wedge_head.inertial = Inertial.from_geometry(
        Box((0.250, 0.120, 0.100)),
        mass=1.9,
        origin=Origin(xyz=(0.090, 0.0, 0.040)),
    )

    ra_assembly = model.part("ra_assembly")
    ra_assembly.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=graphite,
        name="ra_flange",
    )
    ra_assembly.visual(
        Cylinder(radius=0.046, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=satin_silver,
        name="ra_housing",
    )
    ra_assembly.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=anodized_red,
        name="drive_pulley",
    )
    ra_assembly.visual(
        Box((0.080, 0.030, 0.030)),
        origin=Origin(xyz=(-0.055, 0.0, 0.070)),
        material=graphite,
        name="motor_bridge",
    )
    ra_assembly.visual(
        Box((0.070, 0.050, 0.060)),
        origin=Origin(xyz=(-0.108, 0.0, 0.062)),
        material=dark_gray,
        name="motor_housing",
    )
    ra_assembly.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.108, 0.0, 0.075)),
        material=anodized_red,
        name="motor_pulley",
    )
    ra_assembly.visual(
        belt_mesh,
        material=rubber_black,
        name="drive_belt",
    )
    ra_assembly.visual(
        Box((0.070, 0.060, 0.130)),
        origin=Origin(xyz=(0.055, 0.0, 0.210)),
        material=dark_gray,
        name="saddle_tower",
    )
    ra_assembly.visual(
        Box((0.180, 0.036, 0.012)),
        origin=Origin(xyz=(0.120, 0.0, 0.281)),
        material=satin_silver,
        name="saddle_base",
    )
    ra_assembly.visual(
        Box((0.180, 0.006, 0.030)),
        origin=Origin(xyz=(0.120, 0.019, 0.302)),
        material=graphite,
        name="saddle_wall_pos",
    )
    ra_assembly.visual(
        Box((0.180, 0.006, 0.030)),
        origin=Origin(xyz=(0.120, -0.019, 0.302)),
        material=graphite,
        name="saddle_wall_neg",
    )
    ra_assembly.visual(
        Cylinder(radius=0.038, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=graphite,
        name="upper_bearing_cap",
    )
    ra_assembly.inertial = Inertial.from_geometry(
        Box((0.280, 0.120, 0.330)),
        mass=3.0,
        origin=Origin(xyz=(0.030, 0.0, 0.165)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Box((0.360, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_silver,
        name="dovetail_bar",
    )
    for index, ring_x in enumerate((-0.140, 0.125)):
        optical_tube.visual(
            Box((0.026, 0.028, 0.044)),
            origin=Origin(xyz=(ring_x, 0.0, 0.032)),
            material=anodized_red,
            name=f"ring_block_{index}",
        )
        optical_tube.visual(
            tube_ring_mesh,
            origin=Origin(xyz=(ring_x, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=anodized_red,
            name=f"tube_ring_{index}",
        )
    optical_tube.visual(
        main_tube_mesh,
        origin=Origin(xyz=(0.010, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_white,
        name="main_tube_shell",
    )
    optical_tube.visual(
        dew_shield_mesh,
        origin=Origin(xyz=(-0.240, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dew_shield_shell",
    )
    optical_tube.visual(
        Cylinder(radius=0.056, length=0.020),
        origin=Origin(xyz=(-0.175, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dew_shield_adapter",
    )
    optical_tube.visual(
        Cylinder(radius=0.053, length=0.012),
        origin=Origin(xyz=(-0.168, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dew_shield_collar",
    )
    optical_tube.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(-0.180, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dew_shield_joiner",
    )
    optical_tube.visual(
        Cylinder(radius=0.058, length=0.020),
        origin=Origin(xyz=(-0.304, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="objective_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.057, length=0.002),
        origin=Origin(xyz=(-0.313, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="objective_glass",
    )
    optical_tube.visual(
        Cylinder(radius=0.049, length=0.016),
        origin=Origin(xyz=(0.198, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_adapter",
    )
    optical_tube.visual(
        Cylinder(radius=0.038, length=0.094),
        origin=Origin(xyz=(0.253, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="focuser_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.029, length=0.090),
        origin=Origin(xyz=(0.345, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="drawtube",
    )
    optical_tube.visual(
        Cylinder(radius=0.042, length=0.082),
        origin=Origin(xyz=(0.431, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="imaging_camera",
    )
    optical_tube.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.478, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_red,
        name="camera_nosepiece",
    )
    optical_tube.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.250, 0.040, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="focuser_knob_left",
    )
    optical_tube.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.250, -0.040, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="focuser_knob_right",
    )
    optical_tube.visual(
        Box((0.050, 0.020, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.132)),
        material=graphite,
        name="finder_shoe",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.800, 0.140, 0.150)),
        mass=3.3,
        origin=Origin(xyz=(0.080, 0.0, 0.078)),
    )

    saddle_knob = model.part("saddle_knob")
    saddle_knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="knob_hub",
    )
    saddle_knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="knob_body",
    )
    saddle_knob.visual(
        Cylinder(radius=0.004, length=0.035),
        origin=Origin(xyz=(0.012, 0.015, 0.0)),
        material=graphite,
        name="knob_handle",
    )
    saddle_knob.inertial = Inertial.from_geometry(
        Box((0.045, 0.032, 0.035)),
        mass=0.08,
        origin=Origin(xyz=(0.010, 0.016, 0.0)),
    )

    model.articulation(
        "tripod_to_wedge_base",
        ArticulationType.FIXED,
        parent=tripod_hub,
        child=wedge_base,
        origin=Origin(xyz=(0.0, 0.0, 1.128)),
    )
    model.articulation(
        "wedge_latitude",
        ArticulationType.REVOLUTE,
        parent=wedge_base,
        child=wedge_head,
        origin=Origin(
            xyz=(-0.030, 0.0, 0.053),
            rpy=(0.0, DEFAULT_LATITUDE, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.30,
            lower=-math.radians(12.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "wedge_to_ra",
        ArticulationType.CONTINUOUS,
        parent=wedge_head,
        child=ra_assembly,
        origin=Origin(xyz=(0.155, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5),
    )
    model.articulation(
        "ra_to_optical_tube",
        ArticulationType.FIXED,
        parent=ra_assembly,
        child=optical_tube,
        origin=Origin(xyz=(0.120, 0.0, 0.293)),
    )
    model.articulation(
        "saddle_knob_spin",
        ArticulationType.REVOLUTE,
        parent=ra_assembly,
        child=saddle_knob,
        origin=Origin(xyz=(0.155, 0.022, 0.304)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    leg_specs = [
        ("front", 0.0),
        ("left", 2.0 * math.pi / 3.0),
        ("right", 4.0 * math.pi / 3.0),
    ]
    hip_radius = 0.074
    hip_z = 0.852
    upper_length = 0.460
    lower_length = 0.540
    outer_x = 0.032
    outer_y = 0.018
    wall = 0.0035
    leg_pitch = -math.radians(23.0)

    for leg_name, yaw in leg_specs:
        hip_point = (hip_radius * math.cos(yaw), hip_radius * math.sin(yaw), hip_z)
        leg_rpy = (0.0, leg_pitch, yaw)
        socket_offset = _rotate_pitch_yaw((0.0, 0.0, -0.010), pitch=leg_pitch, yaw=yaw)
        tripod_hub.visual(
            Box((0.054, 0.034, 0.020)),
            origin=Origin(
                xyz=(
                    hip_point[0] + socket_offset[0],
                    hip_point[1] + socket_offset[1],
                    hip_point[2] + socket_offset[2],
                ),
                rpy=leg_rpy,
            ),
            material=graphite,
            name=f"{leg_name}_socket",
        )

        upper_leg = model.part(f"upper_leg_{leg_name}")
        upper_leg.visual(
            Box((outer_x, outer_y, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=graphite,
            name="top_shoe",
        )
        upper_leg.visual(
            Box((wall, outer_y, upper_length)),
            origin=Origin(xyz=(outer_x * 0.5 - wall * 0.5, 0.0, -0.260)),
            material=tripod_black,
            name="rail_pos_x",
        )
        upper_leg.visual(
            Box((wall, outer_y, upper_length)),
            origin=Origin(xyz=(-(outer_x * 0.5 - wall * 0.5), 0.0, -0.260)),
            material=tripod_black,
            name="rail_neg_x",
        )
        upper_leg.visual(
            Box((outer_x - 2.0 * wall, wall, upper_length)),
            origin=Origin(xyz=(0.0, outer_y * 0.5 - wall * 0.5, -0.260)),
            material=tripod_black,
            name="rail_pos_y",
        )
        upper_leg.visual(
            Box((outer_x - 2.0 * wall, wall, upper_length)),
            origin=Origin(xyz=(0.0, -(outer_y * 0.5 - wall * 0.5), -0.260)),
            material=tripod_black,
            name="rail_neg_y",
        )
        upper_leg.inertial = Inertial.from_geometry(
            Box((0.054, 0.034, 0.500)),
            mass=0.85,
            origin=Origin(xyz=(0.0, 0.0, -0.250)),
        )

        lower_leg = model.part(f"lower_leg_{leg_name}")
        lower_leg.visual(
            Box((0.018, 0.011, 0.546)),
            origin=Origin(xyz=(0.0, 0.0, -0.262)),
            material=satin_silver,
            name="inner_tube",
        )
        lower_leg.visual(
            Box((0.012, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0017)),
            material=graphite,
            name="top_stop",
        )
        lower_leg.visual(
            Box((0.050, 0.024, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, -0.543)),
            material=rubber_black,
            name="foot_pad",
        )
        lower_leg.inertial = Inertial.from_geometry(
            Box((0.050, 0.024, 0.560)),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.0, -0.280)),
        )

        model.articulation(
            f"tripod_to_upper_leg_{leg_name}",
            ArticulationType.FIXED,
            parent=tripod_hub,
            child=upper_leg,
            origin=Origin(xyz=hip_point, rpy=leg_rpy),
        )
        model.articulation(
            f"upper_to_lower_leg_{leg_name}",
            ArticulationType.PRISMATIC,
            parent=upper_leg,
            child=lower_leg,
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=14.0,
                velocity=0.08,
                lower=0.0,
                upper=0.220,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_hub = object_model.get_part("tripod_hub")
    wedge_base = object_model.get_part("wedge_base")
    wedge_head = object_model.get_part("wedge_head")
    ra_assembly = object_model.get_part("ra_assembly")
    optical_tube = object_model.get_part("optical_tube")
    saddle_knob = object_model.get_part("saddle_knob")

    wedge_latitude = object_model.get_articulation("wedge_latitude")
    wedge_to_ra = object_model.get_articulation("wedge_to_ra")
    saddle_knob_spin = object_model.get_articulation("saddle_knob_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    for leg_name in ("front", "left", "right"):
        ctx.allow_overlap(
            tripod_hub,
            object_model.get_part(f"upper_leg_{leg_name}"),
            reason="Each tripod leg is captured by the crown casting at the hinge socket.",
        )
    ctx.allow_overlap(
        ra_assembly,
        wedge_head,
        reason="The RA bearing flange is represented as a seated interface on the tilted wedge head plate.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(wedge_base, tripod_hub)
    ctx.expect_contact(wedge_head, wedge_base)
    ctx.expect_contact(ra_assembly, wedge_head)
    ctx.expect_contact(optical_tube, ra_assembly)
    ctx.expect_contact(saddle_knob, ra_assembly)

    ctx.check(
        "wedge_latitude_axis",
        wedge_latitude.axis == (0.0, 1.0, 0.0),
        f"unexpected wedge latitude axis: {wedge_latitude.axis}",
    )
    ctx.check(
        "ra_axis_continuous",
        wedge_to_ra.articulation_type == ArticulationType.CONTINUOUS
        and wedge_to_ra.axis == (0.0, 0.0, 1.0),
        f"unexpected RA articulation: type={wedge_to_ra.articulation_type}, axis={wedge_to_ra.axis}",
    )
    ctx.check(
        "saddle_knob_axis",
        saddle_knob_spin.axis == (0.0, 1.0, 0.0),
        f"unexpected saddle knob axis: {saddle_knob_spin.axis}",
    )

    for leg_name in ("front", "left", "right"):
        upper_leg = object_model.get_part(f"upper_leg_{leg_name}")
        lower_leg = object_model.get_part(f"lower_leg_{leg_name}")
        leg_joint = object_model.get_articulation(f"upper_to_lower_leg_{leg_name}")
        ctx.expect_contact(upper_leg, tripod_hub, name=f"{leg_name}_upper_leg_contact")
        ctx.expect_contact(lower_leg, upper_leg, name=f"{leg_name}_lower_leg_contact")
        ctx.check(
            f"{leg_name}_leg_prismatic_axis",
            leg_joint.articulation_type == ArticulationType.PRISMATIC
            and leg_joint.axis == (0.0, 0.0, -1.0),
            f"unexpected {leg_name} leg axis: type={leg_joint.articulation_type}, axis={leg_joint.axis}",
        )

    front_leg = object_model.get_part("lower_leg_front")
    front_leg_joint = object_model.get_articulation("upper_to_lower_leg_front")
    front_rest = ctx.part_world_position(front_leg)
    ctx.check("front_leg_rest_position_available", front_rest is not None, "front lower leg has no world position")
    if front_rest is not None:
        with ctx.pose({front_leg_joint: 0.220}):
            front_extended = ctx.part_world_position(front_leg)
            ctx.check(
                "front_leg_extension_motion",
                front_extended is not None and front_extended[2] < front_rest[2] - 0.15,
                f"front leg did not extend downward enough: rest={front_rest}, extended={front_extended}",
            )

    tube_rest = ctx.part_world_position(optical_tube)
    ctx.check("tube_rest_position_available", tube_rest is not None, "optical tube has no world position")
    if tube_rest is not None:
        with ctx.pose({wedge_to_ra: math.pi / 2.0}):
            tube_quarter = ctx.part_world_position(optical_tube)
            ctx.check(
                "ra_rotation_moves_tube",
                tube_quarter is not None
                and math.hypot(tube_quarter[0] - tube_rest[0], tube_quarter[1] - tube_rest[1]) > 0.10,
                f"RA rotation did not swing optical tube enough: rest={tube_rest}, quarter={tube_quarter}",
            )

        with ctx.pose({wedge_latitude: -math.radians(10.0)}):
            tube_high = ctx.part_world_position(optical_tube)
            ctx.check(
                "latitude_adjust_changes_tube_height",
                tube_high is not None and tube_high[2] > tube_rest[2] + 0.04,
                f"latitude adjustment did not raise tube enough: rest={tube_rest}, high={tube_high}",
            )

    knob_rest = ctx.part_element_world_aabb(saddle_knob, elem="knob_handle")
    ctx.check("knob_rest_aabb_available", knob_rest is not None, "saddle knob handle has no AABB")
    if knob_rest is not None:
        with ctx.pose({saddle_knob_spin: 1.0}):
            knob_turned = ctx.part_element_world_aabb(saddle_knob, elem="knob_handle")
            ctx.check(
                "saddle_knob_rotates_handle",
                knob_turned is not None
                and math.hypot(
                    ((knob_turned[0][0] + knob_turned[1][0]) - (knob_rest[0][0] + knob_rest[1][0])) * 0.5,
                    ((knob_turned[0][2] + knob_turned[1][2]) - (knob_rest[0][2] + knob_rest[1][2])) * 0.5,
                )
                > 0.004,
                f"saddle knob handle did not sweep enough: rest={knob_rest}, turned={knob_turned}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
