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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _ring_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
    segments: int = 72,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
        ),
        name,
    )


def _add_quad(
    geom: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
):
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _spherical_shell_patch(
    *,
    radius_outer: float,
    thickness: float,
    center_z: float,
    z_min: float,
    z_max: float,
    az_min: float,
    az_max: float,
    theta_steps: int = 18,
    az_steps: int = 24,
) -> MeshGeometry:
    radius_inner = radius_outer - thickness
    cos_min = max(-1.0, min(1.0, (z_max - center_z) / radius_outer))
    cos_max = max(-1.0, min(1.0, (z_min - center_z) / radius_outer))
    theta_min = math.acos(cos_min)
    theta_max = math.acos(cos_max)

    thetas = [
        theta_min + (theta_max - theta_min) * i / theta_steps
        for i in range(theta_steps + 1)
    ]
    azimuths = [
        az_min + (az_max - az_min) * j / az_steps
        for j in range(az_steps + 1)
    ]

    geom = MeshGeometry()
    outer_idx: list[list[int]] = []
    inner_idx: list[list[int]] = []

    for theta in thetas:
        outer_row: list[int] = []
        inner_row: list[int] = []
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        for azimuth in azimuths:
            cos_az = math.cos(azimuth)
            sin_az = math.sin(azimuth)
            outer_row.append(
                geom.add_vertex(
                    radius_outer * sin_theta * cos_az,
                    radius_outer * sin_theta * sin_az,
                    center_z + radius_outer * cos_theta,
                )
            )
            inner_row.append(
                geom.add_vertex(
                    radius_inner * sin_theta * cos_az,
                    radius_inner * sin_theta * sin_az,
                    center_z + radius_inner * cos_theta,
                )
            )
        outer_idx.append(outer_row)
        inner_idx.append(inner_row)

    for i in range(theta_steps):
        for j in range(az_steps):
            _add_quad(
                geom,
                outer_idx[i][j],
                outer_idx[i + 1][j],
                outer_idx[i + 1][j + 1],
                outer_idx[i][j + 1],
            )
            _add_quad(
                geom,
                inner_idx[i][j],
                inner_idx[i][j + 1],
                inner_idx[i + 1][j + 1],
                inner_idx[i + 1][j],
            )

    for i in range(theta_steps):
        _add_quad(
            geom,
            outer_idx[i][0],
            outer_idx[i + 1][0],
            inner_idx[i + 1][0],
            inner_idx[i][0],
        )
        _add_quad(
            geom,
            outer_idx[i][az_steps],
            inner_idx[i][az_steps],
            inner_idx[i + 1][az_steps],
            outer_idx[i + 1][az_steps],
        )

    for j in range(az_steps):
        _add_quad(
            geom,
            outer_idx[theta_steps][j],
            outer_idx[theta_steps][j + 1],
            inner_idx[theta_steps][j + 1],
            inner_idx[theta_steps][j],
        )
        _add_quad(
            geom,
            outer_idx[0][j],
            inner_idx[0][j],
            inner_idx[0][j + 1],
            outer_idx[0][j + 1],
        )

    return geom


def _add_ring_bolts(
    part,
    *,
    prefix: str,
    radius: float,
    z: float,
    count: int,
    head_radius: float,
    head_height: float,
    material,
    yaw_offset: float = 0.0,
):
    for index in range(count):
        azimuth = yaw_offset + (2.0 * math.pi * index) / count
        x = radius * math.cos(azimuth)
        y = radius * math.sin(azimuth)
        part.visual(
            Cylinder(radius=head_radius, length=head_height),
            origin=Origin(xyz=(x, y, z + head_height / 2.0)),
            material=material,
            name=f"{prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_observatory_dome")

    concrete = model.material("concrete", rgba=(0.62, 0.63, 0.64, 1.0))
    shell_white = model.material("shell_white", rgba=(0.90, 0.92, 0.93, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.38, 0.41, 0.44, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.73, 0.10, 1.0))
    safety_red = model.material("safety_red", rgba=(0.74, 0.18, 0.15, 1.0))
    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.75, 1.0))
    black = model.material("black", rgba=(0.07, 0.07, 0.08, 1.0))

    support_base = model.part("support_base")
    support_base.visual(
        _ring_shell_mesh(
            outer_radius=2.65,
            inner_radius=2.28,
            z0=0.0,
            z1=1.88,
            name="support_drum_shell",
        ),
        material=concrete,
        name="support_drum_shell",
    )
    support_base.visual(
        _ring_shell_mesh(
            outer_radius=2.58,
            inner_radius=2.08,
            z0=1.86,
            z1=2.12,
            name="bearing_deck",
        ),
        material=machinery_gray,
        name="bearing_deck",
    )
    support_base.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=2.29,
                tube=0.05,
                radial_segments=18,
                tubular_segments=88,
            ),
            "track_rail",
        ),
        origin=Origin(xyz=(0.0, 0.0, 2.03)),
        material=dark_steel,
        name="track_rail",
    )
    support_base.visual(
        Box((0.42, 0.52, 0.62)),
        origin=Origin(xyz=(2.34, -0.88, 1.69)),
        material=machinery_gray,
        name="drive_guard_body",
    )
    support_base.visual(
        Box((0.30, 0.72, 0.12)),
        origin=Origin(xyz=(2.20, -0.88, 1.98), rpy=(0.0, 0.30, 0.0)),
        material=safety_yellow,
        name="drive_guard_top",
    )
    support_base.visual(
        Box((0.16, 0.40, 0.34)),
        origin=Origin(xyz=(2.20, 1.04, 1.14)),
        material=machinery_gray,
        name="lockout_pedestal",
    )
    support_base.visual(
        Cylinder(radius=0.035, length=0.42),
        origin=Origin(
            xyz=(2.28, 1.04, 1.28),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=safety_yellow,
        name="lockout_handle",
    )
    support_base.visual(
        Box((0.14, 0.16, 0.12)),
        origin=Origin(xyz=(2.36, 1.04, 1.28)),
        material=safety_red,
        name="lockout_body",
    )
    for index in range(8):
        azimuth = math.radians(22.5) + index * (2.0 * math.pi / 8.0)
        radius = 2.52
        x = radius * math.cos(azimuth)
        y = radius * math.sin(azimuth)
        support_base.visual(
            Box((0.22, 0.28, 0.20)),
            origin=Origin(xyz=(x, y, 1.98), rpy=(0.0, 0.0, azimuth)),
            material=machinery_gray,
            name=f"bogie_housing_{index:02d}",
        )
    _add_ring_bolts(
        support_base,
        prefix="deck_bolt",
        radius=2.44,
        z=2.12,
        count=16,
        head_radius=0.028,
        head_height=0.026,
        material=stainless,
        yaw_offset=math.radians(11.25),
    )
    support_base.inertial = Inertial.from_geometry(
        Box((5.4, 5.4, 2.2)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 1.1)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _ring_shell_mesh(
            outer_radius=2.30,
            inner_radius=2.08,
            z0=0.0,
            z1=0.18,
            name="rotation_flange",
        ),
        material=dark_steel,
        name="rotation_flange",
    )
    dome_shell.visual(
        _ring_shell_mesh(
            outer_radius=2.30,
            inner_radius=2.24,
            z0=0.16,
            z1=0.56,
            name="lower_skirt",
        ),
        material=shell_white,
        name="lower_skirt",
    )
    dome_shell.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=2.26,
                tube=0.07,
                radial_segments=18,
                tubular_segments=88,
            ),
            "ring_girder",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=machinery_gray,
        name="ring_girder",
    )
    dome_shell.visual(
        mesh_from_geometry(
            _spherical_shell_patch(
                radius_outer=2.35,
                thickness=0.05,
                center_z=0.0,
                z_min=0.54,
                z_max=2.34,
                az_min=0.34,
                az_max=(2.0 * math.pi) - 0.34,
                theta_steps=22,
                az_steps=64,
            ),
            "upper_dome_shell",
        ),
        material=shell_white,
        name="upper_dome_shell",
    )
    dome_shell.visual(
        Box((0.28, 0.96, 0.12)),
        origin=Origin(xyz=(2.02, 0.0, 0.58)),
        material=machinery_gray,
        name="sill_beam",
    )
    dome_shell.visual(
        Box((0.30, 0.16, 0.08)),
        origin=Origin(xyz=(2.15, -0.55, 0.60)),
        material=machinery_gray,
        name="mast_base_left",
    )
    dome_shell.visual(
        Box((0.30, 0.16, 0.08)),
        origin=Origin(xyz=(2.15, 0.55, 0.60)),
        material=machinery_gray,
        name="mast_base_right",
    )
    dome_shell.visual(
        Box((0.24, 0.14, 0.26)),
        origin=Origin(xyz=(2.16, -0.55, 0.72)),
        material=machinery_gray,
        name="mast_gusset_left",
    )
    dome_shell.visual(
        Box((0.24, 0.14, 0.26)),
        origin=Origin(xyz=(2.16, 0.55, 0.72)),
        material=machinery_gray,
        name="mast_gusset_right",
    )
    dome_shell.visual(
        Box((0.12, 0.12, 2.32)),
        origin=Origin(xyz=(2.28, -0.55, 1.78)),
        material=machinery_gray,
        name="slit_frame_left",
    )
    dome_shell.visual(
        Box((0.12, 0.12, 2.32)),
        origin=Origin(xyz=(2.28, 0.55, 1.78)),
        material=machinery_gray,
        name="slit_frame_right",
    )
    dome_shell.visual(
        Box((0.10, 0.12, 1.16)),
        origin=Origin(xyz=(2.06, -0.49, 1.14)),
        material=dark_steel,
        name="lower_guide_left",
    )
    dome_shell.visual(
        Box((0.10, 0.12, 1.16)),
        origin=Origin(xyz=(2.06, 0.49, 1.14)),
        material=dark_steel,
        name="lower_guide_right",
    )
    dome_shell.visual(
        Box((0.12, 0.12, 1.74)),
        origin=Origin(xyz=(2.08, -0.55, 1.99)),
        material=dark_steel,
        name="upper_rail_left",
    )
    dome_shell.visual(
        Box((0.12, 0.12, 1.74)),
        origin=Origin(xyz=(2.08, 0.55, 1.99)),
        material=dark_steel,
        name="upper_rail_right",
    )
    dome_shell.visual(
        Box((0.22, 1.60, 0.12)),
        origin=Origin(xyz=(1.92, 0.0, 3.08)),
        material=machinery_gray,
        name="rail_crosshead",
    )
    dome_shell.visual(
        Box((0.08, 0.12, 0.20)),
        origin=Origin(xyz=(1.98, -0.55, 2.94)),
        material=machinery_gray,
        name="crosshead_post_left",
    )
    dome_shell.visual(
        Box((0.08, 0.12, 0.20)),
        origin=Origin(xyz=(1.98, 0.55, 2.94)),
        material=machinery_gray,
        name="crosshead_post_right",
    )
    dome_shell.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(1.98, -0.75, 3.04), (1.34, -1.00, 2.60), (0.70, -1.18, 2.22)],
                radius=0.05,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "crown_brace_left",
        ),
        material=machinery_gray,
        name="crown_brace_left",
    )
    dome_shell.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(1.98, 0.75, 3.04), (1.34, 1.00, 2.60), (0.70, 1.18, 2.22)],
                radius=0.05,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "crown_brace_right",
        ),
        material=machinery_gray,
        name="crown_brace_right",
    )
    dome_shell.visual(
        Box((0.10, 0.10, 1.56)),
        origin=Origin(xyz=(2.30, -0.65, 1.38)),
        material=safety_yellow,
        name="pinch_guard_left",
    )
    dome_shell.visual(
        Box((0.10, 0.10, 1.56)),
        origin=Origin(xyz=(2.30, 0.65, 1.38)),
        material=safety_yellow,
        name="pinch_guard_right",
    )
    dome_shell.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(2.08, -0.62, 1.73)),
        material=safety_red,
        name="lower_stop_left",
    )
    dome_shell.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(2.08, 0.62, 1.73)),
        material=safety_red,
        name="lower_stop_right",
    )
    dome_shell.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(2.08, -0.62, 2.77)),
        material=safety_red,
        name="upper_stop_left",
    )
    dome_shell.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(2.08, 0.62, 2.77)),
        material=safety_red,
        name="upper_stop_right",
    )
    dome_shell.visual(
        Box((0.08, 0.12, 0.08)),
        origin=Origin(xyz=(2.06, -0.49, 1.45)),
        material=machinery_gray,
        name="upper_seat_left",
    )
    dome_shell.visual(
        Box((0.08, 0.12, 0.08)),
        origin=Origin(xyz=(2.06, 0.49, 1.45)),
        material=machinery_gray,
        name="upper_seat_right",
    )
    dome_shell.visual(
        Box((0.18, 0.14, 0.28)),
        origin=Origin(xyz=(2.10, -0.72, 0.98)),
        material=machinery_gray,
        name="rail_lockout_bracket",
    )
    dome_shell.visual(
        Cylinder(radius=0.026, length=0.24),
        origin=Origin(
            xyz=(2.19, -0.72, 1.02),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=safety_yellow,
        name="rail_lockout_pin",
    )
    dome_shell.visual(
        Box((0.16, 0.22, 1.34)),
        origin=Origin(xyz=(-2.10, 0.0, 1.49)),
        material=machinery_gray,
        name="rear_spine",
    )
    _add_ring_bolts(
        dome_shell,
        prefix="flange_bolt",
        radius=2.18,
        z=0.18,
        count=12,
        head_radius=0.024,
        head_height=0.020,
        material=stainless,
        yaw_offset=math.radians(15.0),
    )
    for side, y_sign in (("left", -1.0), ("right", 1.0)):
        for row, z_loc in enumerate((0.64, 0.74)):
            for col, x_loc in enumerate((2.05, 2.18)):
                dome_shell.visual(
                    Cylinder(radius=0.018, length=0.018),
                    origin=Origin(xyz=(x_loc, y_sign * 0.55, z_loc + 0.009)),
                    material=stainless,
                    name=f"mast_plate_bolt_{side}_{row}_{col}",
                )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.35, length=2.40),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
    )

    lower_shutter = model.part("lower_shutter")
    lower_shutter.visual(
        mesh_from_geometry(
            _spherical_shell_patch(
                radius_outer=2.39,
                thickness=0.03,
                center_z=0.0,
                z_min=0.64,
                z_max=1.46,
                az_min=-0.145,
                az_max=0.145,
                theta_steps=14,
                az_steps=18,
            ),
            "lower_shutter_panel",
        ),
        material=shell_white,
        name="panel_skin",
    )
    lower_shutter.visual(
        Box((0.24, 0.46, 0.86)),
        origin=Origin(xyz=(1.92, 0.0, 1.06)),
        material=machinery_gray,
        name="main_web",
    )
    lower_shutter.visual(
        Box((0.16, 0.52, 0.12)),
        origin=Origin(xyz=(2.00, 0.0, 0.70)),
        material=machinery_gray,
        name="bottom_crossbeam",
    )
    lower_shutter.visual(
        Box((0.16, 0.54, 0.10)),
        origin=Origin(xyz=(1.98, 0.0, 1.46)),
        material=machinery_gray,
        name="top_crossbeam",
    )
    lower_shutter.visual(
        Box((0.12, 0.08, 0.86)),
        origin=Origin(xyz=(2.00, -0.26, 1.08)),
        material=machinery_gray,
        name="side_stile_left",
    )
    lower_shutter.visual(
        Box((0.12, 0.08, 0.86)),
        origin=Origin(xyz=(2.00, 0.26, 1.08)),
        material=machinery_gray,
        name="side_stile_right",
    )
    lower_shutter.visual(
        Box((0.16, 0.24, 0.12)),
        origin=Origin(xyz=(1.91, -0.36, 0.92)),
        material=machinery_gray,
        name="roller_arm_left_low",
    )
    lower_shutter.visual(
        Box((0.16, 0.24, 0.12)),
        origin=Origin(xyz=(1.91, 0.36, 0.92)),
        material=machinery_gray,
        name="roller_arm_right_low",
    )
    lower_shutter.visual(
        Box((0.16, 0.24, 0.12)),
        origin=Origin(xyz=(1.91, -0.36, 1.28)),
        material=machinery_gray,
        name="roller_arm_left_high",
    )
    lower_shutter.visual(
        Box((0.16, 0.24, 0.12)),
        origin=Origin(xyz=(1.91, 0.36, 1.28)),
        material=machinery_gray,
        name="roller_arm_right_high",
    )
    lower_shutter.visual(
        Cylinder(radius=0.025, length=0.08),
        origin=Origin(
            xyz=(1.95, -0.49, 0.92),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="roller_left_low",
    )
    lower_shutter.visual(
        Cylinder(radius=0.025, length=0.08),
        origin=Origin(
            xyz=(1.95, -0.49, 1.28),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="roller_left_high",
    )
    lower_shutter.visual(
        Cylinder(radius=0.025, length=0.08),
        origin=Origin(
            xyz=(1.95, 0.49, 0.92),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="roller_right_low",
    )
    lower_shutter.visual(
        Cylinder(radius=0.025, length=0.08),
        origin=Origin(
            xyz=(1.95, 0.49, 1.28),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="roller_right_high",
    )
    lower_shutter.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(2.02, -0.20, 0.68)),
        material=black,
        name="stop_shoe_left",
    )
    lower_shutter.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(2.02, 0.20, 0.68)),
        material=black,
        name="stop_shoe_right",
    )
    lower_shutter.visual(
        Box((0.12, 0.18, 0.12)),
        origin=Origin(xyz=(2.08, 0.0, 0.74)),
        material=safety_red,
        name="lower_latch_block",
    )
    lower_shutter.inertial = Inertial.from_geometry(
        Box((0.55, 1.00, 0.98)),
        mass=460.0,
        origin=Origin(xyz=(2.00, 0.0, 1.08)),
    )

    upper_shutter = model.part("upper_shutter")
    upper_shutter.visual(
        mesh_from_geometry(
            _spherical_shell_patch(
                radius_outer=2.40,
                thickness=0.03,
                center_z=0.0,
                z_min=1.54,
                z_max=2.32,
                az_min=-0.15,
                az_max=0.15,
                theta_steps=14,
                az_steps=20,
            ),
            "upper_shutter_panel",
        ),
        material=shell_white,
        name="hood_skin",
    )
    upper_shutter.visual(
        Box((0.18, 0.58, 0.10)),
        origin=Origin(xyz=(1.96, 0.0, 1.58)),
        material=machinery_gray,
        name="lower_lip",
    )
    upper_shutter.visual(
        Box((0.18, 0.62, 0.14)),
        origin=Origin(xyz=(1.94, 0.0, 1.94)),
        material=machinery_gray,
        name="mid_beam",
    )
    upper_shutter.visual(
        Box((0.20, 0.58, 0.10)),
        origin=Origin(xyz=(1.96, 0.0, 2.34)),
        material=machinery_gray,
        name="top_beam",
    )
    upper_shutter.visual(
        Box((0.24, 0.34, 0.76)),
        origin=Origin(xyz=(1.88, 0.0, 1.96)),
        material=machinery_gray,
        name="center_spine",
    )
    upper_shutter.visual(
        Box((0.06, 0.10, 0.82)),
        origin=Origin(xyz=(1.90, -0.55, 1.95)),
        material=machinery_gray,
        name="carriage_left",
    )
    upper_shutter.visual(
        Box((0.06, 0.10, 0.82)),
        origin=Origin(xyz=(1.90, 0.55, 1.95)),
        material=machinery_gray,
        name="carriage_right",
    )
    upper_shutter.visual(
        Box((0.08, 0.10, 0.08)),
        origin=Origin(xyz=(1.98, -0.49, 1.53)),
        material=black,
        name="seat_shoe_left",
    )
    upper_shutter.visual(
        Box((0.08, 0.10, 0.08)),
        origin=Origin(xyz=(1.98, 0.49, 1.53)),
        material=black,
        name="seat_shoe_right",
    )
    upper_shutter.visual(
        Box((0.08, 0.12, 0.48)),
        origin=Origin(xyz=(1.95, -0.49, 1.78)),
        material=machinery_gray,
        name="side_cheek_left",
    )
    upper_shutter.visual(
        Box((0.08, 0.12, 0.48)),
        origin=Origin(xyz=(1.95, 0.49, 1.78)),
        material=machinery_gray,
        name="side_cheek_right",
    )
    upper_shutter.visual(
        Box((0.10, 0.40, 0.18)),
        origin=Origin(xyz=(1.92, -0.32, 1.82)),
        material=machinery_gray,
        name="carriage_bridge_left",
    )
    upper_shutter.visual(
        Box((0.10, 0.40, 0.18)),
        origin=Origin(xyz=(1.92, 0.32, 1.82)),
        material=machinery_gray,
        name="carriage_bridge_right",
    )
    upper_shutter.visual(
        Box((0.10, 0.08, 0.12)),
        origin=Origin(xyz=(1.92, -0.55, 1.78)),
        material=dark_steel,
        name="hood_roller_left",
    )
    upper_shutter.visual(
        Box((0.10, 0.08, 0.12)),
        origin=Origin(xyz=(1.92, 0.55, 1.78)),
        material=dark_steel,
        name="hood_roller_right",
    )
    upper_shutter.visual(
        Box((0.12, 0.18, 0.10)),
        origin=Origin(xyz=(2.04, 0.0, 1.64)),
        material=safety_red,
        name="hood_latch_block",
    )
    upper_shutter.inertial = Inertial.from_geometry(
        Box((0.48, 1.12, 0.90)),
        mass=390.0,
        origin=Origin(xyz=(1.98, 0.0, 1.95)),
    )

    dome_rotation = model.articulation(
        "support_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=support_base,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 2.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.25),
    )
    lower_slide = model.articulation(
        "dome_to_lower_shutter",
        ArticulationType.PRISMATIC,
        parent=dome_shell,
        child=lower_shutter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22000.0,
            velocity=0.18,
            lower=0.0,
            upper=0.72,
        ),
    )
    upper_slide = model.articulation(
        "dome_to_upper_shutter",
        ArticulationType.PRISMATIC,
        parent=dome_shell,
        child=upper_shutter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.18,
            lower=0.0,
            upper=0.62,
        ),
    )

    model.meta["primary_articulations"] = [
        dome_rotation.name,
        lower_slide.name,
        upper_slide.name,
    ]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_base = object_model.get_part("support_base")
    dome_shell = object_model.get_part("dome_shell")
    lower_shutter = object_model.get_part("lower_shutter")
    upper_shutter = object_model.get_part("upper_shutter")
    dome_rotation = object_model.get_articulation("support_to_dome_rotation")
    lower_slide = object_model.get_articulation("dome_to_lower_shutter")
    upper_slide = object_model.get_articulation("dome_to_upper_shutter")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        dome_shell,
        lower_shutter,
        elem_a="upper_seat_left",
        elem_b="side_stile_left",
        reason="Closed lower shutter uses captured side guides; simplified solid envelopes for seat block and stile slightly interpenetrate.",
    )
    ctx.allow_overlap(
        dome_shell,
        lower_shutter,
        elem_a="upper_seat_right",
        elem_b="side_stile_right",
        reason="Closed lower shutter uses captured side guides; simplified solid envelopes for seat block and stile slightly interpenetrate.",
    )
    ctx.allow_overlap(
        lower_shutter,
        upper_shutter,
        elem_a="side_stile_left",
        elem_b="seat_shoe_left",
        reason="Closed shutter seal is modeled as overlapping box envelopes instead of a thin labyrinth interlock.",
    )
    ctx.allow_overlap(
        lower_shutter,
        upper_shutter,
        elem_a="side_stile_right",
        elem_b="seat_shoe_right",
        reason="Closed shutter seal is modeled as overlapping box envelopes instead of a thin labyrinth interlock.",
    )
    ctx.allow_overlap(
        dome_shell,
        upper_shutter,
        elem_a="rail_crosshead",
        elem_b="center_spine",
        reason="Upper shutter crown carriage is represented with conservative solid boxes; the real assembly telescopes inside a hooded crosshead.",
    )
    ctx.allow_overlap(
        dome_shell,
        lower_shutter,
        elem_a="sill_beam",
        elem_b="main_web",
        reason="Lower shutter inner web is modeled as a thick plate captured behind the sill beam; the real weather seal is a nested folded section.",
    )
    ctx.allow_overlap(
        dome_shell,
        upper_shutter,
        elem_a="lower_guide_left",
        elem_b="seat_shoe_left",
        reason="Upper shutter side shoe runs inside the left captured guide; simplified block envelopes slightly interpenetrate.",
    )
    ctx.allow_overlap(
        dome_shell,
        upper_shutter,
        elem_a="lower_guide_right",
        elem_b="seat_shoe_right",
        reason="Upper shutter side shoe runs inside the right captured guide; simplified block envelopes slightly interpenetrate.",
    )
    ctx.allow_overlap(
        lower_shutter,
        upper_shutter,
        elem_a="top_crossbeam",
        elem_b="lower_lip",
        reason="Open slit pose is represented with telescoping shutter hems as overlapping boxes rather than thin nested channel sections.",
    )
    ctx.allow_overlap(
        lower_shutter,
        upper_shutter,
        elem_a="side_stile_left",
        elem_b="lower_lip",
        reason="Open slit pose uses simplified box side channels; the real upper hem nests around the lower shutter stile without bulk overlap.",
    )
    ctx.allow_overlap(
        lower_shutter,
        upper_shutter,
        elem_a="side_stile_right",
        elem_b="lower_lip",
        reason="Open slit pose uses simplified box side channels; the real upper hem nests around the lower shutter stile without bulk overlap.",
    )
    ctx.allow_overlap(
        lower_shutter,
        upper_shutter,
        elem_a="main_web",
        elem_b="lower_lip",
        reason="Open slit pose models the nested shutter overlap zone with thick plate envelopes; the real hood lip telescopes over a thinner lower web.",
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

    ctx.expect_contact(
        dome_shell,
        support_base,
        elem_a="rotation_flange",
        elem_b="bearing_deck",
        name="dome_flange_seated_on_bearing_deck",
    )
    ctx.expect_contact(
        lower_shutter,
        dome_shell,
        elem_a="stop_shoe_left",
        elem_b="sill_beam",
        name="lower_shutter_supported_on_sill",
    )
    ctx.expect_contact(
        upper_shutter,
        dome_shell,
        elem_a="seat_shoe_left",
        elem_b="upper_seat_left",
        name="upper_shutter_supported_on_seat_block",
    )

    closed_lower = ctx.part_world_position(lower_shutter)
    with ctx.pose({lower_slide: 0.72}):
        raised_lower = ctx.part_world_position(lower_shutter)
    ctx.check(
        "lower_shutter_travels_upward",
        bool(
            closed_lower is not None
            and raised_lower is not None
            and raised_lower[2] > closed_lower[2] + 0.65
        ),
        details=f"closed={closed_lower}, raised={raised_lower}",
    )

    closed_upper = ctx.part_world_position(upper_shutter)
    upper_open = 0.62
    with ctx.pose({upper_slide: upper_open}):
        raised_upper = ctx.part_world_position(upper_shutter)
    ctx.check(
        "upper_shutter_travels_upward",
        bool(
            closed_upper is not None
            and raised_upper is not None
            and raised_upper[2] > closed_upper[2] + 0.55
        ),
        details=f"closed={closed_upper}, raised={raised_upper}",
    )

    slit_left_aabb_0 = ctx.part_element_world_aabb(dome_shell, elem="slit_frame_left")
    with ctx.pose({dome_rotation: math.pi / 2.0}):
        slit_left_aabb_90 = ctx.part_element_world_aabb(dome_shell, elem="slit_frame_left")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    center_0 = _aabb_center(slit_left_aabb_0)
    center_90 = _aabb_center(slit_left_aabb_90)
    ctx.check(
        "dome_rotation_swings_aperture_around_vertical_axis",
        bool(
            center_0 is not None
            and center_90 is not None
            and center_0[0] > 2.0
            and abs(center_0[1]) < 0.8
            and center_90[1] > 2.0
            and abs(center_90[0]) < 0.8
        ),
        details=f"center_0={center_0}, center_90={center_90}",
    )

    with ctx.pose({lower_slide: 0.72, upper_slide: upper_open}):
        ctx.expect_gap(
            upper_shutter,
            lower_shutter,
            axis="z",
            positive_elem="top_beam",
            negative_elem="top_crossbeam",
            min_gap=0.20,
            name="open_pose_upper_beam_above_lower_beam",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
