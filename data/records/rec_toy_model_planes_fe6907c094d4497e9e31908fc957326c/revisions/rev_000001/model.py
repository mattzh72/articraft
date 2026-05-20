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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(x: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    corner = min(radius, width * 0.25, height * 0.25)
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, corner_segments=6, radius=corner)]


def _guard_ring_mesh(
    *,
    ring_radius: float,
    tube_radius: float,
    x_center: float,
    mesh_name: str,
):
    ring_points = [
        (
            x_center,
            ring_radius * math.cos((2.0 * math.pi * i) / 24.0),
            ring_radius * math.sin((2.0 * math.pi * i) / 24.0),
        )
        for i in range(24)
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            ring_points,
            radius=tube_radius,
            samples_per_segment=4,
            closed_spline=True,
            radial_segments=18,
            cap_ends=False,
            up_hint=(1.0, 0.0, 0.0),
        ),
        mesh_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_model_plane")

    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    medium_steel = model.material("medium_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    aircraft_gray = model.material("aircraft_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.90, 0.41, 0.10, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.77, 0.79, 0.82, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))

    stand_base = model.part("stand_base")
    stand_base.visual(
        Box((0.280, 0.180, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_steel,
        name="base_plate",
    )
    stand_base.visual(
        Box((0.130, 0.100, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=medium_steel,
        name="column_doubler",
    )
    stand_base.visual(
        Box((0.220, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.080, 0.018)),
        material=medium_steel,
        name="front_toe_rail",
    )
    stand_base.visual(
        Box((0.220, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.080, 0.018)),
        material=medium_steel,
        name="rear_toe_rail",
    )
    for x_pos in (-0.090, 0.090):
        for y_pos in (-0.055, 0.055):
            stand_base.visual(
                Cylinder(radius=0.007, length=0.010),
                origin=Origin(xyz=(x_pos, y_pos, 0.021)),
                material=fastener_steel,
                name=f"anchor_bolt_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.280, 0.180, 0.035)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    stand_post = model.part("stand_post")
    stand_post.visual(
        Box((0.048, 0.082, 0.170)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_steel,
        name="main_post",
    )
    stand_post.visual(
        Box((0.072, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.179)),
        material=medium_steel,
        name="pivot_cap",
    )
    stand_post.visual(
        Box((0.040, 0.012, 0.064)),
        origin=Origin(xyz=(0.0, 0.054, 0.148)),
        material=medium_steel,
        name="right_cheek_plate",
    )
    stand_post.visual(
        Box((0.040, 0.012, 0.064)),
        origin=Origin(xyz=(0.0, -0.054, 0.148)),
        material=medium_steel,
        name="left_cheek_plate",
    )
    stand_post.visual(
        Box((0.018, 0.096, 0.020)),
        origin=Origin(xyz=(0.032, 0.0, 0.162)),
        material=safety_orange,
        name="forward_stop_block",
    )
    stand_post.visual(
        Box((0.018, 0.096, 0.020)),
        origin=Origin(xyz=(-0.032, 0.0, 0.162)),
        material=safety_orange,
        name="rear_stop_block",
    )
    stand_post.visual(
        Box((0.092, 0.010, 0.040)),
        origin=Origin(
            xyz=(0.028, 0.0, 0.050),
            rpy=(0.0, -math.radians(35.0), 0.0),
        ),
        material=medium_steel,
        name="forward_gusset",
    )
    stand_post.visual(
        Box((0.092, 0.010, 0.040)),
        origin=Origin(
            xyz=(-0.028, 0.0, 0.050),
            rpy=(0.0, math.radians(35.0), 0.0),
        ),
        material=medium_steel,
        name="rear_gusset",
    )
    stand_post.inertial = Inertial.from_geometry(
        Box((0.092, 0.096, 0.188)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
    )

    yoke_frame = model.part("yoke_frame")
    yoke_frame.visual(
        Box((0.200, 0.012, 0.080)),
        origin=Origin(xyz=(0.080, 0.066, -0.018)),
        material=medium_steel,
        name="right_side_plate",
    )
    yoke_frame.visual(
        Box((0.200, 0.012, 0.080)),
        origin=Origin(xyz=(0.080, -0.066, -0.018)),
        material=medium_steel,
        name="left_side_plate",
    )
    yoke_frame.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.004, 0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="right_pivot_pin",
    )
    yoke_frame.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.004, -0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="left_pivot_pin",
    )
    yoke_frame.visual(
        Box((0.200, 0.132, 0.018)),
        origin=Origin(xyz=(0.180, 0.0, -0.090)),
        material=dark_steel,
        name="saddle_plate",
    )
    yoke_frame.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.280, 0.046, -0.072)),
        material=safety_orange,
        name="right_forward_stop_tab",
    )
    yoke_frame.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.280, -0.046, -0.072)),
        material=safety_orange,
        name="left_forward_stop_tab",
    )
    yoke_frame.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.160, 0.046, -0.072)),
        material=safety_orange,
        name="right_rear_stop_tab",
    )
    yoke_frame.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.160, -0.046, -0.072)),
        material=safety_orange,
        name="left_rear_stop_tab",
    )
    yoke_frame.visual(
        Box((0.150, 0.012, 0.028)),
        origin=Origin(
            xyz=(0.155, 0.066, -0.055),
            rpy=(0.0, -math.radians(32.0), 0.0),
        ),
        material=dark_steel,
        name="right_forward_brace",
    )
    yoke_frame.visual(
        Box((0.150, 0.012, 0.028)),
        origin=Origin(
            xyz=(0.155, -0.066, -0.055),
            rpy=(0.0, -math.radians(32.0), 0.0),
        ),
        material=dark_steel,
        name="left_forward_brace",
    )
    yoke_frame.visual(
        Box((0.080, 0.012, 0.028)),
        origin=Origin(
            xyz=(0.040, 0.066, -0.056),
            rpy=(0.0, math.radians(34.0), 0.0),
        ),
        material=dark_steel,
        name="right_rear_brace",
    )
    yoke_frame.visual(
        Box((0.080, 0.012, 0.028)),
        origin=Origin(
            xyz=(0.040, -0.066, -0.056),
            rpy=(0.0, math.radians(34.0), 0.0),
        ),
        material=dark_steel,
        name="left_rear_brace",
    )
    yoke_frame.visual(
        Box((0.020, 0.010, 0.026)),
        origin=Origin(xyz=(-0.010, 0.074, -0.004)),
        material=safety_orange,
        name="right_lockout_ear",
    )
    yoke_frame.visual(
        Box((0.020, 0.010, 0.026)),
        origin=Origin(xyz=(-0.010, -0.074, -0.004)),
        material=safety_orange,
        name="left_lockout_ear",
    )
    yoke_frame.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(
            xyz=(-0.010, 0.080, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=fastener_steel,
        name="right_lockout_pin",
    )
    yoke_frame.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(
            xyz=(-0.010, -0.080, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=fastener_steel,
        name="left_lockout_pin",
    )
    for y_pos, side_name in ((0.070, "right"), (-0.070, "left")):
        for x_pos, suffix in ((-0.010, "aft"), (0.022, "fwd")):
            yoke_frame.visual(
                Cylinder(radius=0.0055, length=0.006),
                origin=Origin(
                    xyz=(x_pos, y_pos, 0.008),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=fastener_steel,
                name=f"{side_name}_plate_bolt_{suffix}",
            )
    yoke_frame.inertial = Inertial.from_geometry(
        Box((0.320, 0.126, 0.080)),
        mass=0.95,
        origin=Origin(xyz=(0.140, 0.0, -0.020)),
    )

    fuselage = model.part("fuselage")
    fuselage.visual(
        Box((0.160, 0.084, 0.084)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=aircraft_gray,
        name="center_body",
    )
    fuselage.visual(
        Box((0.110, 0.074, 0.074)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=aircraft_gray,
        name="forward_body",
    )
    fuselage.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aircraft_gray,
        name="cowl_shell",
    )
    fuselage.visual(
        Box((0.050, 0.044, 0.050)),
        origin=Origin(xyz=(-0.165, 0.0, 0.0)),
        material=aircraft_gray,
        name="tail_boom",
    )
    fuselage.visual(
        Box((0.100, 0.050, 0.014)),
        origin=Origin(xyz=(0.005, 0.0, -0.074)),
        material=dark_steel,
        name="mount_rail",
    )
    fuselage.visual(
        Box((0.018, 0.022, 0.044)),
        origin=Origin(xyz=(-0.020, 0.0, -0.059)),
        material=medium_steel,
        name="rear_mount_stanchion",
    )
    fuselage.visual(
        Box((0.018, 0.022, 0.044)),
        origin=Origin(xyz=(0.025, 0.0, -0.059)),
        material=medium_steel,
        name="front_mount_stanchion",
    )
    fuselage.visual(
        Box((0.084, 0.072, 0.008)),
        origin=Origin(xyz=(-0.005, 0.0, 0.046)),
        material=medium_steel,
        name="wing_mount_pad",
    )
    fuselage.visual(
        Box((0.052, 0.036, 0.008)),
        origin=Origin(xyz=(-0.145, 0.0, 0.029)),
        material=medium_steel,
        name="tail_mount_pad",
    )
    fuselage.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.169, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="nose_bearing_barrel",
    )
    fuselage.visual(
        Box((0.018, 0.070, 0.070)),
        origin=Origin(xyz=(0.181, 0.0, 0.0)),
        material=medium_steel,
        name="guard_mount_block",
    )
    fuselage.visual(
        Box((0.130, 0.020, 0.014)),
        origin=Origin(xyz=(-0.075, 0.0, -0.056)),
        material=medium_steel,
        name="belly_keel",
    )
    fuselage.visual(
        Box((0.100, 0.020, 0.006)),
        origin=Origin(xyz=(-0.070, 0.0, 0.045)),
        material=medium_steel,
        name="top_spine",
    )
    fuselage.visual(
        Box((0.090, 0.010, 0.036)),
        origin=Origin(xyz=(0.010, 0.047, -0.004)),
        material=medium_steel,
        name="right_yoke_doubler",
    )
    fuselage.visual(
        Box((0.090, 0.010, 0.036)),
        origin=Origin(xyz=(0.010, -0.047, -0.004)),
        material=medium_steel,
        name="left_yoke_doubler",
    )
    for x_pos in (-0.022, 0.022):
        for y_pos, side_name in ((-0.018, "left"), (0.018, "right")):
            fuselage.visual(
                Cylinder(radius=0.0045, length=0.006),
                origin=Origin(xyz=(x_pos, y_pos, 0.047), rpy=(0.0, 0.0, 0.0)),
                material=fastener_steel,
                name=f"wing_pad_bolt_{side_name}_{'aft' if x_pos < 0 else 'fwd'}",
            )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.380, 0.100, 0.110)),
        mass=0.95,
        origin=Origin(xyz=(0.000, 0.0, 0.0)),
    )

    wing = model.part("wing")
    wing_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (-0.062, -0.055),
                (-0.055, -0.150),
                (-0.048, -0.235),
                (0.015, -0.182),
                (0.074, -0.028),
                (0.090, 0.0),
                (0.074, 0.028),
                (0.015, 0.182),
                (-0.048, 0.235),
                (-0.055, 0.150),
                (-0.062, 0.055),
            ],
            0.012,
        ),
        "industrial_plane_main_wing",
    )
    wing.visual(wing_mesh, material=aircraft_gray, name="wing_panel")
    wing.visual(
        Box((0.090, 0.090, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.014)),
        material=medium_steel,
        name="center_strap",
    )
    wing.visual(
        Box((0.060, 0.012, 0.018)),
        origin=Origin(xyz=(0.010, 0.038, 0.009)),
        material=medium_steel,
        name="right_root_clamp",
    )
    wing.visual(
        Box((0.060, 0.012, 0.018)),
        origin=Origin(xyz=(0.010, -0.038, 0.009)),
        material=medium_steel,
        name="left_root_clamp",
    )
    for x_pos in (-0.026, 0.026):
        for y_pos in (-0.020, 0.020):
            wing.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(xyz=(x_pos, y_pos, 0.017)),
                material=fastener_steel,
                name=f"center_strap_bolt_{'aft' if x_pos < 0 else 'fwd'}_{'left' if y_pos < 0 else 'right'}",
            )
    wing.inertial = Inertial.from_geometry(
        Box((0.160, 0.470, 0.018)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    tailplane = model.part("tailplane")
    tail_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (-0.045, -0.060),
                (-0.038, -0.094),
                (0.010, -0.076),
                (0.032, -0.012),
                (0.038, 0.0),
                (0.032, 0.012),
                (0.010, 0.076),
                (-0.038, 0.094),
                (-0.045, 0.060),
            ],
            0.008,
        ),
        "industrial_plane_tailplane",
    )
    tailplane.visual(tail_mesh, material=aircraft_gray, name="tailplane_panel")
    tailplane.visual(
        Box((0.040, 0.050, 0.003)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0095)),
        material=medium_steel,
        name="tail_center_strap",
    )
    for y_pos in (-0.014, 0.014):
        tailplane.visual(
            Cylinder(radius=0.0035, length=0.005),
            origin=Origin(xyz=(-0.006, y_pos, 0.011)),
            material=fastener_steel,
            name=f"tail_strap_bolt_{'left' if y_pos < 0 else 'right'}",
        )
    tailplane.inertial = Inertial.from_geometry(
        Box((0.090, 0.190, 0.012)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    fin = model.part("fin")
    fin_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (-0.030, 0.0),
                (0.014, 0.0),
                (0.028, 0.022),
                (0.008, 0.070),
                (-0.024, 0.060),
            ],
            0.008,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.004, 0.0),
        "industrial_plane_fin",
    )
    fin.visual(fin_mesh, material=aircraft_gray, name="fin_panel")
    fin.visual(
        Box((0.030, 0.010, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, 0.009)),
        material=medium_steel,
        name="fin_base_doubler",
    )
    fin.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(-0.004, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="fin_base_bolt",
    )
    fin.inertial = Inertial.from_geometry(
        Box((0.060, 0.010, 0.072)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    prop_guard = model.part("prop_guard")
    prop_guard.visual(
        _guard_ring_mesh(
            ring_radius=0.058,
            tube_radius=0.004,
            x_center=0.032,
            mesh_name="industrial_plane_prop_guard_ring",
        ),
        material=safety_orange,
        name="guard_ring",
    )
    diagonal_pairs = ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0))
    for sx, sz in diagonal_pairs:
        pad_y = sx * 0.026
        pad_z = sz * 0.026
        pad_name = f"mount_pad_{'p' if sx > 0 else 'n'}y_{'p' if sz > 0 else 'n'}z"
        prop_guard.visual(
            Box((0.008, 0.012, 0.012)),
            origin=Origin(xyz=(0.004, pad_y, pad_z)),
            material=medium_steel,
            name=pad_name,
        )
        _add_member(
            prop_guard,
            (0.008, pad_y, pad_z),
            (0.032, sx * (0.058 / math.sqrt(2.0)), sz * (0.058 / math.sqrt(2.0))),
            radius=0.004,
            material=medium_steel,
            name=f"guard_strut_{'p' if sx > 0 else 'n'}y_{'p' if sz > 0 else 'n'}z",
        )
    prop_guard.inertial = Inertial.from_geometry(
        Box((0.050, 0.130, 0.130)),
        mass=0.10,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_steel,
        name="backplate",
    )
    propeller.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="hub",
    )
    propeller.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_steel,
        name="retaining_nut",
    )
    propeller.visual(
        Box((0.004, 0.104, 0.016)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=prop_black,
        name="blade_y",
    )
    propeller.visual(
        Box((0.004, 0.016, 0.104)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=prop_black,
        name="blade_z",
    )
    propeller.visual(
        Box((0.008, 0.028, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=medium_steel,
        name="blade_y_root",
    )
    propeller.visual(
        Box((0.008, 0.012, 0.028)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=medium_steel,
        name="blade_z_root",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.032, 0.110, 0.110)),
        mass=0.08,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=stand_base,
        child=stand_post,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "stand_pitch",
        ArticulationType.REVOLUTE,
        parent=stand_post,
        child=yoke_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.179)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.30,
            upper=0.50,
        ),
    )
    model.articulation(
        "yoke_to_fuselage",
        ArticulationType.FIXED,
        parent=yoke_frame,
        child=fuselage,
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
    )
    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing,
        origin=Origin(xyz=(-0.005, 0.0, 0.050)),
    )
    model.articulation(
        "fuselage_to_tailplane",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tailplane,
        origin=Origin(xyz=(-0.145, 0.0, 0.038)),
    )
    model.articulation(
        "tailplane_to_fin",
        ArticulationType.FIXED,
        parent=tailplane,
        child=fin,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "fuselage_to_prop_guard",
        ArticulationType.FIXED,
        parent=fuselage,
        child=prop_guard,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
    )
    model.articulation(
        "prop_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand_base = object_model.get_part("stand_base")
    stand_post = object_model.get_part("stand_post")
    yoke_frame = object_model.get_part("yoke_frame")
    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")
    tailplane = object_model.get_part("tailplane")
    fin = object_model.get_part("fin")
    prop_guard = object_model.get_part("prop_guard")
    propeller = object_model.get_part("propeller")
    stand_pitch = object_model.get_articulation("stand_pitch")
    prop_spin = object_model.get_articulation("prop_spin")

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

    ctx.expect_contact(stand_post, stand_base, name="stand post seated on base")
    ctx.expect_contact(wing, fuselage, name="wing supported by fuselage pad")
    ctx.expect_contact(tailplane, fuselage, name="tailplane supported by tail pad")
    ctx.expect_contact(fin, tailplane, name="fin supported by tailplane")
    ctx.expect_contact(prop_guard, fuselage, name="prop guard mounted on nose")
    ctx.expect_contact(
        propeller,
        fuselage,
        elem_a="backplate",
        name="propeller backplate mounted on nose face",
    )

    with ctx.pose({stand_pitch: 0.0}):
        ctx.expect_contact(yoke_frame, stand_post, name="yoke carried by stand cheeks")
        ctx.expect_contact(fuselage, yoke_frame, name="fuselage seated in yoke saddle")

    upper_pitch = 0.50
    with ctx.pose({stand_pitch: upper_pitch}):
        ctx.expect_contact(
            yoke_frame,
            stand_post,
            name="yoke remains mounted when pitched up",
        )

    with ctx.pose({prop_spin: 0.0}):
        ctx.expect_gap(
            prop_guard,
            propeller,
            axis="x",
            positive_elem="guard_ring",
            negative_elem="blade_y",
            min_gap=0.012,
            max_gap=0.025,
            name="guard ring sits ahead of propeller blade plane",
        )
        ctx.expect_within(
            propeller,
            prop_guard,
            axes="yz",
            outer_elem="guard_ring",
            margin=0.0,
            name="propeller stays inside guard footprint",
        )

    with ctx.pose({prop_spin: math.pi / 4.0}):
        ctx.expect_within(
            propeller,
            prop_guard,
            axes="yz",
            outer_elem="guard_ring",
            margin=0.0,
            name="propeller stays inside guard footprint when rotated",
        )

    closed_guard_position = ctx.part_world_position(prop_guard)
    with ctx.pose({stand_pitch: upper_pitch}):
        pitched_guard_position = ctx.part_world_position(prop_guard)
    nose_raises = (
        closed_guard_position is not None
        and pitched_guard_position is not None
        and pitched_guard_position[2] > closed_guard_position[2] + 0.030
        and pitched_guard_position[0] < closed_guard_position[0] - 0.010
    )
    ctx.check(
        "stand pitch raises the nose upward",
        nose_raises,
        details=(
            f"closed_guard_position={closed_guard_position}, "
            f"pitched_guard_position={pitched_guard_position}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
