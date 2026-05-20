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
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _spherical_point(
    radius: float,
    theta: float,
    phi: float,
    *,
    z_center: float,
) -> tuple[float, float, float]:
    sin_theta = math.sin(theta)
    return (
        radius * sin_theta * math.cos(phi),
        radius * sin_theta * math.sin(phi),
        z_center + radius * math.cos(theta),
    )


def _spherical_shell_band(
    *,
    outer_radius: float,
    inner_radius: float,
    theta_start: float,
    theta_end: float,
    phi_start: float,
    phi_end: float,
    theta_segments: int,
    phi_segments: int,
    z_center: float,
    closed_phi: bool,
    cap_phi_start: bool,
    cap_phi_end: bool,
) -> MeshGeometry:
    geom = MeshGeometry()

    phi_count = phi_segments if closed_phi else phi_segments + 1
    phi_denominator = phi_segments if closed_phi else max(phi_segments, 1)

    outer_ids: list[list[int]] = []
    inner_ids: list[list[int]] = []

    for theta_index in range(theta_segments + 1):
        t = theta_index / max(theta_segments, 1)
        theta = theta_start + (theta_end - theta_start) * t
        outer_row: list[int] = []
        inner_row: list[int] = []
        for phi_index in range(phi_count):
            phi_t = phi_index / phi_denominator
            phi = phi_start + (phi_end - phi_start) * phi_t
            outer_row.append(
                geom.add_vertex(*_spherical_point(outer_radius, theta, phi, z_center=z_center))
            )
            inner_row.append(
                geom.add_vertex(*_spherical_point(inner_radius, theta, phi, z_center=z_center))
            )
        outer_ids.append(outer_row)
        inner_ids.append(inner_row)

    phi_span = phi_count if closed_phi else phi_count - 1
    for theta_index in range(theta_segments):
        for phi_index in range(phi_span):
            next_phi = (phi_index + 1) % phi_count
            _add_quad(
                geom,
                outer_ids[theta_index][phi_index],
                outer_ids[theta_index][next_phi],
                outer_ids[theta_index + 1][next_phi],
                outer_ids[theta_index + 1][phi_index],
            )
            _add_quad(
                geom,
                inner_ids[theta_index][phi_index],
                inner_ids[theta_index + 1][phi_index],
                inner_ids[theta_index + 1][next_phi],
                inner_ids[theta_index][next_phi],
            )

    if not closed_phi and cap_phi_start:
        for theta_index in range(theta_segments):
            _add_quad(
                geom,
                outer_ids[theta_index][0],
                outer_ids[theta_index + 1][0],
                inner_ids[theta_index + 1][0],
                inner_ids[theta_index][0],
            )
    if not closed_phi and cap_phi_end:
        last_index = phi_count - 1
        for theta_index in range(theta_segments):
            _add_quad(
                geom,
                outer_ids[theta_index][last_index],
                inner_ids[theta_index][last_index],
                inner_ids[theta_index + 1][last_index],
                outer_ids[theta_index + 1][last_index],
            )

    for phi_index in range(phi_span):
        next_phi = (phi_index + 1) % phi_count
        _add_quad(
            geom,
            outer_ids[0][phi_index],
            inner_ids[0][phi_index],
            inner_ids[0][next_phi],
            outer_ids[0][next_phi],
        )
        _add_quad(
            geom,
            outer_ids[theta_segments][phi_index],
            outer_ids[theta_segments][next_phi],
            inner_ids[theta_segments][next_phi],
            inner_ids[theta_segments][phi_index],
        )

    return geom


def _build_shutter_panel_mesh(
    *,
    length: float,
    width_top: float,
    width_bottom: float,
    thickness: float,
    camber: float,
) -> MeshGeometry:
    section_positions = (0.0, 0.42, 0.92, length)
    sections: list[list[tuple[float, float, float]]] = []
    for x_pos in section_positions:
        t = x_pos / max(length, 1e-9)
        width = width_top + (width_bottom - width_top) * t
        half_width = width * 0.5
        half_thickness = thickness * 0.5
        z_center = -camber * (t**1.6)
        sections.append(
            [
                (x_pos, -half_width, z_center - half_thickness),
                (x_pos, half_width, z_center - half_thickness),
                (x_pos, half_width, z_center + half_thickness),
                (x_pos, -half_width, z_center + half_thickness),
            ]
        )
    return section_loft(sections)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.69, 0.70, 0.71, 1.0))
    steel = model.material("steel", rgba=(0.32, 0.34, 0.37, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    dome_white = model.material("dome_white", rgba=(0.88, 0.90, 0.92, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.53, 0.56, 0.60, 1.0))
    slit_black = model.material("slit_black", rgba=(0.08, 0.09, 0.10, 1.0))

    outer_radius = 2.20
    shell_thickness = 0.12
    inner_radius = outer_radius - shell_thickness
    shell_center_z = 0.18
    slit_half_angle = 0.23
    hinge_pitch = -1.03
    hinge_x = 1.22
    hinge_z = 2.04
    shutter_length = 1.10

    upper_shell_mesh = mesh_from_geometry(
        _spherical_shell_band(
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            theta_start=0.14,
            theta_end=0.58,
            phi_start=slit_half_angle,
            phi_end=(2.0 * math.pi) - slit_half_angle,
            theta_segments=12,
            phi_segments=60,
            z_center=shell_center_z,
            closed_phi=False,
            cap_phi_start=True,
            cap_phi_end=True,
        ),
        "observatory_upper_shell",
    )
    lower_shell_mesh = mesh_from_geometry(
        _spherical_shell_band(
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            theta_start=0.58,
            theta_end=1.47,
            phi_start=slit_half_angle,
            phi_end=(2.0 * math.pi) - slit_half_angle,
            theta_segments=18,
            phi_segments=52,
            z_center=shell_center_z,
            closed_phi=False,
            cap_phi_start=True,
            cap_phi_end=True,
        ),
        "observatory_lower_shell",
    )
    base_ring = model.part("base_ring")
    base_ring.visual(
        Cylinder(radius=3.05, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="foundation_pad",
    )
    base_ring.visual(
        Cylinder(radius=2.72, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
        material=concrete,
        name="support_curb",
    )
    base_ring.visual(
        Cylinder(radius=2.52, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        material=steel,
        name="bearing_ring",
    )
    base_ring.visual(
        Cylinder(radius=0.42, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=trim_gray,
        name="central_support",
    )
    base_ring.visual(
        Cylinder(radius=0.64, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        material=dark_steel,
        name="thrust_cap",
    )
    base_ring.visual(
        Box((1.20, 0.78, 0.40)),
        origin=Origin(xyz=(-1.75, 0.0, 0.60)),
        material=steel,
        name="drive_housing",
    )
    base_ring.inertial = Inertial.from_geometry(
        Box((6.10, 6.10, 1.22)),
        mass=12000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        Cylinder(radius=2.48, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="turntable_plate",
    )
    dome_shell.visual(
        Cylinder(radius=2.40, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=steel,
        name="rotating_skirt",
    )
    dome_shell.visual(
        upper_shell_mesh,
        material=dome_white,
        name="upper_shell",
    )
    dome_shell.visual(
        lower_shell_mesh,
        material=dome_white,
        name="lower_shell",
    )
    dome_shell.visual(
        Sphere(radius=0.34),
        origin=Origin(xyz=(0.0, 0.0, shell_center_z + outer_radius - 0.02)),
        material=dome_white,
        name="crown_cap",
    )
    dome_shell.visual(
        Box((0.22, 0.20, 1.56)),
        origin=Origin(xyz=(1.92, 0.31, 1.20), rpy=(0.0, 0.56, 0.0)),
        material=steel,
        name="left_slit_jamb",
    )
    dome_shell.visual(
        Box((0.22, 0.20, 1.56)),
        origin=Origin(xyz=(1.92, -0.31, 1.20), rpy=(0.0, 0.56, 0.0)),
        material=steel,
        name="right_slit_jamb",
    )
    dome_shell.visual(
        Box((0.70, 0.64, 0.20)),
        origin=Origin(xyz=(2.00, 0.0, 0.49), rpy=(0.0, 0.22, 0.0)),
        material=slit_black,
        name="slit_base_recess",
    )
    dome_shell.visual(
        Box((0.80, 0.08, 0.40)),
        origin=Origin(xyz=(1.60, 0.26, 1.99)),
        material=dark_steel,
        name="left_hinge_cheek",
    )
    dome_shell.visual(
        Box((0.80, 0.08, 0.40)),
        origin=Origin(xyz=(1.60, -0.26, 1.99)),
        material=dark_steel,
        name="right_hinge_cheek",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.48, length=2.70),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.35)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        Cylinder(radius=0.04, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    shutter_leaf.visual(
        Box((0.16, 0.44, 0.05)),
        origin=Origin(xyz=(0.08, 0.0, -0.04)),
        material=dark_steel,
        name="hinge_leaf_plate",
    )
    shutter_leaf.visual(
        Box((shutter_length, 0.44, 0.045)),
        origin=Origin(xyz=(shutter_length * 0.5, 0.0, -0.08)),
        material=dome_white,
        name="shutter_panel",
    )
    shutter_leaf.visual(
        Box((shutter_length - 0.10, 0.04, 0.065)),
        origin=Origin(xyz=(0.53, 0.19, -0.07)),
        material=trim_gray,
        name="left_stiffener",
    )
    shutter_leaf.visual(
        Box((shutter_length - 0.10, 0.04, 0.065)),
        origin=Origin(xyz=(0.53, -0.19, -0.07)),
        material=trim_gray,
        name="right_stiffener",
    )
    shutter_leaf.visual(
        Box((0.22, 0.30, 0.05)),
        origin=Origin(xyz=(0.92, 0.0, -0.07)),
        material=trim_gray,
        name="lower_reinforcement",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((1.14, 0.52, 0.16)),
        mass=110.0,
        origin=Origin(xyz=(0.55, 0.0, -0.06)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65000.0, velocity=0.35),
    )
    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(0.0, hinge_pitch, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.55,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    base_to_dome = object_model.get_articulation("base_to_dome")
    dome_to_shutter = object_model.get_articulation("dome_to_shutter")

    ctx.expect_gap(
        dome_shell,
        base_ring,
        axis="z",
        positive_elem="turntable_plate",
        negative_elem="thrust_cap",
        max_gap=0.03,
        max_penetration=0.001,
        name="rotating stage sits on the central thrust cap",
    )
    ctx.expect_overlap(
        dome_shell,
        base_ring,
        axes="xy",
        elem_a="turntable_plate",
        elem_b="thrust_cap",
        min_overlap=0.90,
        name="central support remains centered under the stage",
    )
    with ctx.pose({base_to_dome: math.radians(90)}):
        ctx.expect_gap(
            dome_shell,
            base_ring,
            axis="z",
            positive_elem="turntable_plate",
            negative_elem="thrust_cap",
            max_gap=0.03,
            max_penetration=0.001,
            name="stage support stays aligned after azimuth rotation",
        )

    closed_panel = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")
    with ctx.pose({dome_to_shutter: 1.10}):
        open_panel = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")

    shutter_opens = False
    if closed_panel is not None and open_panel is not None:
        closed_position = tuple(
            (closed_panel[0][axis] + closed_panel[1][axis]) * 0.5 for axis in range(3)
        )
        open_position = tuple(
            (open_panel[0][axis] + open_panel[1][axis]) * 0.5 for axis in range(3)
        )
        shutter_opens = (
            open_position[0] < closed_position[0] - 0.18
            and open_position[2] > closed_position[2] + 0.03
        )
    else:
        closed_position = None
        open_position = None
    ctx.check(
        "shutter leaf retracts upward toward the crown when opened",
        shutter_opens,
        details=f"closed={closed_position}, open={open_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
