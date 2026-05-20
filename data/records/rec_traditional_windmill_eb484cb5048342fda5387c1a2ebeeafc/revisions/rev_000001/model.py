from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


TOWER_HEIGHT = 6.20
ROTOR_RADIUS = 3.32


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_shell(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    axis: str = "z",
    segments: int = 56,
) -> MeshGeometry:
    ring = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * length),
            (outer_radius, 0.5 * length),
        ],
        [
            (inner_radius, -0.5 * length),
            (inner_radius, 0.5 * length),
        ],
        segments=segments,
    )
    if axis == "x":
        ring.rotate_y(math.pi / 2.0)
    elif axis == "y":
        ring.rotate_x(math.pi / 2.0)
    return ring


def _build_tower_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (1.45, 0.22),
            (1.39, 0.90),
            (1.30, 2.40),
            (1.16, 4.30),
            (0.98, 6.12),
        ],
        [
            (1.10, 0.30),
            (1.05, 0.98),
            (0.98, 2.45),
            (0.87, 4.32),
            (0.72, 6.02),
        ],
        segments=84,
    )


def _cap_section(x_pos: float, width: float, height: float) -> list[tuple[float, float, float]]:
    return [
        (x_pos, -0.50 * width, 0.00 * height),
        (x_pos, -0.44 * width, 0.22 * height),
        (x_pos, -0.22 * width, 0.60 * height),
        (x_pos, -0.08 * width, 0.84 * height),
        (x_pos, 0.00, 1.00 * height),
        (x_pos, 0.08 * width, 0.84 * height),
        (x_pos, 0.22 * width, 0.60 * height),
        (x_pos, 0.44 * width, 0.22 * height),
        (x_pos, 0.50 * width, 0.00 * height),
        (x_pos, 0.28 * width, 0.08 * height),
        (x_pos, 0.00, 0.12 * height),
        (x_pos, -0.28 * width, 0.08 * height),
    ]


def _build_cap_shell() -> MeshGeometry:
    return section_loft(
        [
            _cap_section(-0.82, 1.56, 0.70),
            _cap_section(-0.28, 1.84, 0.96),
            _cap_section(0.30, 1.90, 1.05),
            _cap_section(0.82, 1.58, 0.88),
            _cap_section(1.18, 0.90, 0.54),
        ]
    )


def _build_single_blade_mesh() -> MeshGeometry:
    blade_x = 0.50
    geometry = MeshGeometry()
    geometry.merge(BoxGeometry((0.12, 0.28, 0.62)).translate(blade_x - 0.05, 0.0, 0.80))
    geometry.merge(BoxGeometry((0.055, 0.046, 2.62)).translate(blade_x, 0.20, 1.98))
    geometry.merge(BoxGeometry((0.055, 0.046, 2.62)).translate(blade_x, -0.20, 1.98))
    geometry.merge(BoxGeometry((0.030, 0.10, 2.72)).translate(blade_x, 0.0, 1.92))

    for index in range(10):
        z_pos = 0.84 + (0.27 * index)
        width = 0.46 - (0.015 * index)
        geometry.merge(BoxGeometry((0.030, width, 0.035)).translate(blade_x, 0.0, z_pos))

    geometry.merge(BoxGeometry((0.040, 0.32, 0.05)).translate(blade_x, 0.0, 3.30))
    return geometry


def _build_blade_lattice_mesh() -> MeshGeometry:
    blade = _build_single_blade_mesh()
    return _merge_geometries(
        blade,
        blade.copy().rotate_x(math.pi / 2.0),
        blade.copy().rotate_x(math.pi),
        blade.copy().rotate_x(3.0 * math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    tower_stone = model.material("tower_stone", rgba=(0.72, 0.69, 0.63, 1.0))
    cap_paint = model.material("cap_paint", rgba=(0.34, 0.21, 0.16, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.20, 0.21, 0.22, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.84, 0.80, 0.72, 1.0))

    tower_shell_mesh = _save_mesh("windmill_tower_shell", _build_tower_shell())
    curb_ring_mesh = _save_mesh(
        "windmill_curb_ring",
        _ring_shell(inner_radius=0.70, outer_radius=1.02, length=0.10, axis="z"),
    )
    cap_shell_mesh = _save_mesh("windmill_cap_shell", _build_cap_shell())
    flange_ring_mesh = _save_mesh(
        "windmill_thrust_flange",
        _ring_shell(inner_radius=0.72, outer_radius=1.02, length=0.06, axis="z"),
    )
    yaw_sleeve_mesh = _save_mesh(
        "windmill_yaw_sleeve",
        _ring_shell(inner_radius=0.085, outer_radius=0.15, length=0.30, axis="z"),
    )
    front_bearing_mesh = _save_mesh(
        "windmill_front_bearing",
        _ring_shell(inner_radius=0.05, outer_radius=0.15, length=0.14, axis="x"),
    )
    rear_bearing_mesh = _save_mesh(
        "windmill_rear_bearing",
        _ring_shell(inner_radius=0.055, outer_radius=0.19, length=0.12, axis="x"),
    )
    blade_lattice_mesh = _save_mesh("windmill_blade_lattice", _build_blade_lattice_mesh())

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=1.52, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=tower_stone,
        name="base_plinth",
    )
    tower.visual(tower_shell_mesh, material=tower_stone, name="tower_shell")
    tower.visual(
        curb_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - 0.05)),
        material=dark_iron,
        name="curb_ring",
    )
    tower.visual(
        Box((1.56, 1.56, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - 0.07)),
        material=dark_iron,
        name="top_deck",
    )
    tower.visual(
        Cylinder(radius=0.07, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT + 0.06)),
        material=dark_iron,
        name="yaw_pintle",
    )
    tower.inertial = Inertial.from_geometry(
        Box((3.10, 3.10, TOWER_HEIGHT)),
        mass=14000.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT / 2.0)),
    )

    cap = model.part("cap")
    cap.visual(flange_ring_mesh, origin=Origin(xyz=(0.0, 0.0, 0.03)), material=dark_iron, name="thrust_flange")
    cap.visual(yaw_sleeve_mesh, origin=Origin(xyz=(0.0, 0.0, 0.15)), material=dark_iron, name="yaw_sleeve")
    cap.visual(
        Box((0.16, 0.86, 0.74)),
        origin=Origin(xyz=(-0.06, 0.0, 0.67)),
        material=cap_paint,
        name="rear_panel",
    )
    cap.visual(
        Box((0.20, 0.52, 0.10)),
        origin=Origin(xyz=(0.14, 0.0, 0.25)),
        material=dark_iron,
        name="rear_cross_tie",
    )
    cap.visual(
        Box((0.26, 0.32, 0.10)),
        origin=Origin(xyz=(0.04, 0.0, 0.25)),
        material=dark_iron,
        name="yaw_bridge",
    )
    for side in (-1.0, 1.0):
        side_name = "left" if side > 0 else "right"
        cap.visual(
            Box((0.26, 0.20, 0.20)),
            origin=Origin(xyz=(0.06, side * 0.80, 0.16)),
            material=dark_iron,
            name=f"curb_shoe_{side_name}",
        )
        cap.visual(
            Box((0.28, 0.42, 0.16)),
            origin=Origin(xyz=(0.12, side * 0.54, 0.24)),
            material=dark_iron,
            name=f"flange_link_{side_name}",
        )
        cap.visual(
            Box((0.28, 0.16, 0.48)),
            origin=Origin(xyz=(0.16, side * 0.28, 0.34)),
            material=dark_iron,
            name=f"rear_post_{side_name}",
        )
        cap.visual(
            Box((1.22, 0.16, 0.20)),
            origin=Origin(xyz=(0.86, side * 0.28, 0.48)),
            material=dark_iron,
            name=f"side_sill_{side_name}",
        )
        cap.visual(
            Box((0.64, 0.14, 0.58)),
            origin=Origin(xyz=(1.06, side * 0.28, 0.79)),
            material=dark_iron,
            name=f"side_frame_{side_name}",
        )
        cap.visual(
            Box((1.46, 0.10, 0.86)),
            origin=Origin(
                xyz=(0.72, side * 0.42, 0.95),
                rpy=(-side * 0.62, 0.0, 0.0),
            ),
            material=cap_paint,
            name=f"roof_panel_{side_name}",
        )
    cap.visual(
        Box((1.18, 0.60, 0.18)),
        origin=Origin(xyz=(1.04, 0.0, 0.60)),
        material=dark_iron,
        name="bearing_bed",
    )
    cap.visual(
        Box((0.62, 0.52, 0.18)),
        origin=Origin(xyz=(1.46, 0.0, 1.05)),
        material=dark_iron,
        name="upper_bridge",
    )
    cap.visual(
        rear_bearing_mesh,
        origin=Origin(xyz=(1.28, 0.0, 0.78)),
        material=dark_iron,
        name="rear_bearing_seat",
    )
    cap.visual(
        front_bearing_mesh,
        origin=Origin(xyz=(1.62, 0.0, 0.78)),
        material=dark_iron,
        name="front_bearing_seat",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.20, 2.20, 1.45)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.045, length=0.86),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="main_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.075, length=0.07),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="rear_collar",
    )
    rotor.visual(
        Cylinder(radius=0.045, length=0.12),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="rear_journal",
    )
    rotor.visual(
        Cylinder(radius=0.045, length=0.14),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="front_journal",
    )
    rotor.visual(
        Cylinder(radius=0.17, length=0.30),
        origin=Origin(xyz=(0.58, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.23, length=0.09),
        origin=Origin(xyz=(0.74, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub_flange",
    )
    rotor.visual(
        Box((0.10, 0.18, 1.30)),
        origin=Origin(xyz=(0.54, 0.0, 0.0)),
        material=dark_iron,
        name="vertical_stock",
    )
    rotor.visual(
        Box((0.10, 1.30, 0.18)),
        origin=Origin(xyz=(0.54, 0.0, 0.0)),
        material=dark_iron,
        name="horizontal_stock",
    )
    rotor.visual(blade_lattice_mesh, material=weathered_wood, name="blade_lattice")
    rotor.inertial = Inertial.from_geometry(
        Box((0.90, 2.0 * ROTOR_RADIUS, 2.0 * ROTOR_RADIUS)),
        mass=980.0,
        origin=Origin(xyz=(0.56, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.15),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(1.28, 0.0, 0.78)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_rotor")

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
        "yaw_axis_vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        f"expected vertical cap yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "rotor_axis_horizontal",
        spin.axis == (1.0, 0.0, 0.0),
        f"expected rotor spin axis along +X, got {spin.axis}",
    )

    with ctx.pose({yaw: 0.0, spin: 0.0}):
        ctx.expect_within(
            tower,
            cap,
            inner_elem="yaw_pintle",
            outer_elem="yaw_sleeve",
            axes="xy",
            margin=0.02,
            name="cap_pintle_centered_in_yaw_sleeve",
        )
        ctx.expect_contact(
            cap,
            tower,
            elem_a="thrust_flange",
            elem_b="curb_ring",
            contact_tol=5e-4,
            name="cap_thrust_ring_contact",
        )
        ctx.expect_overlap(
            cap,
            tower,
            elem_a="thrust_flange",
            elem_b="curb_ring",
            axes="xy",
            min_overlap=1.3,
            name="cap_thrust_ring_plan_overlap",
        )
        ctx.expect_within(
            rotor,
            cap,
            inner_elem="rear_journal",
            outer_elem="rear_bearing_seat",
            axes="yz",
            margin=0.01,
            name="rear_bearing_centers_rotor_shaft",
        )
        ctx.expect_within(
            rotor,
            cap,
            inner_elem="front_journal",
            outer_elem="front_bearing_seat",
            axes="yz",
            margin=0.01,
            name="front_bearing_centers_rotor_shaft",
        )
        ctx.expect_origin_gap(
            rotor,
            cap,
            axis="x",
            min_gap=1.10,
            max_gap=1.34,
            name="rotor_projects_forward_of_cap_axis",
        )

    with ctx.pose({yaw: math.radians(35.0), spin: math.radians(22.0)}):
        ctx.expect_within(
            tower,
            cap,
            inner_elem="yaw_pintle",
            outer_elem="yaw_sleeve",
            axes="xy",
            margin=0.02,
            name="cap_remains_guided_when_yawed",
        )
        ctx.expect_within(
            rotor,
            cap,
            inner_elem="front_journal",
            outer_elem="front_bearing_seat",
            axes="yz",
            margin=0.01,
            name="rotor_remains_centered_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
