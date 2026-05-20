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
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _box_mesh(
    size: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    mesh = BoxGeometry(size)
    if rpy[0]:
        mesh.rotate_x(rpy[0])
    if rpy[1]:
        mesh.rotate_y(rpy[1])
    if rpy[2]:
        mesh.rotate_z(rpy[2])
    mesh.translate(*xyz)
    return mesh


def _cylinder_mesh(
    radius: float,
    length: float,
    *,
    axis: str = "z",
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    mesh = CylinderGeometry(radius=radius, height=length, radial_segments=40)
    if axis == "x":
        mesh.rotate_y(math.pi / 2.0)
    elif axis == "y":
        mesh.rotate_x(math.pi / 2.0)
    mesh.translate(*xyz)
    return mesh


def _cone_mesh(
    radius: float,
    length: float,
    *,
    axis: str = "z",
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    mesh = ConeGeometry(radius=radius, height=length, radial_segments=40, closed=True)
    if axis == "x":
        mesh.rotate_y(-math.pi / 2.0)
    elif axis == "y":
        mesh.rotate_x(math.pi / 2.0)
    mesh.translate(*xyz)
    return mesh


def _tube_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    axis: str = "z",
) -> MeshGeometry:
    mesh = LatheGeometry.from_shell_profiles(
        [(-0.0 + outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(-0.0 + inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        mesh.rotate_y(math.pi / 2.0)
    elif axis == "y":
        mesh.rotate_x(math.pi / 2.0)
    return mesh


def _tapered_shroud_mesh(
    *,
    inner_radius: float,
    outer_radius_rear: float,
    outer_radius_front: float,
    length: float,
    axis: str = "x",
) -> MeshGeometry:
    mesh = LatheGeometry.from_shell_profiles(
        [
            (outer_radius_rear, -length / 2.0),
            (0.5 * (outer_radius_rear + outer_radius_front), 0.0),
            (outer_radius_front, length / 2.0),
        ],
        [
            (inner_radius, -length / 2.0),
            (inner_radius, 0.0),
            (inner_radius, length / 2.0),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        mesh.rotate_y(math.pi / 2.0)
    elif axis == "y":
        mesh.rotate_x(math.pi / 2.0)
    return mesh


def _tower_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (2.36, 0.45),
            (2.30, 1.10),
            (2.16, 3.40),
            (1.96, 6.00),
            (1.76, 7.78),
        ],
        [
            (2.08, 0.55),
            (2.02, 1.18),
            (1.88, 3.40),
            (1.68, 6.00),
            (1.48, 7.74),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _cap_frame_mesh() -> MeshGeometry:
    return _merge_geometries(
        _box_mesh((3.70, 2.26, 0.24), xyz=(-0.05, 0.0, 0.18)),
        _box_mesh((3.86, 0.18, 0.84), xyz=(-0.02, 1.04, 0.60)),
        _box_mesh((3.86, 0.18, 0.84), xyz=(-0.02, -1.04, 0.60)),
        _box_mesh((0.22, 1.92, 0.78), xyz=(-1.79, 0.0, 0.57)),
        _box_mesh((1.92, 0.20, 0.22), xyz=(0.62, 0.54, 0.96)),
        _box_mesh((1.92, 0.20, 0.22), xyz=(0.62, -0.54, 0.96)),
        _box_mesh((0.92, 0.22, 0.20), xyz=(0.52, 0.0, 1.02)),
        _box_mesh((0.96, 0.18, 0.62), xyz=(1.92, 0.34, 1.17)),
        _box_mesh((0.96, 0.18, 0.62), xyz=(1.92, -0.34, 1.17)),
        _box_mesh((0.92, 0.44, 0.08), xyz=(1.92, 0.0, 1.52)),
        _box_mesh((0.92, 0.44, 0.08), xyz=(1.92, 0.0, 0.82)),
    )


def _cap_roof_mesh() -> MeshGeometry:
    return _merge_geometries(
        _box_mesh((3.62, 1.56, 0.16), xyz=(-0.10, 0.80, 1.22), rpy=(-0.63, 0.0, 0.0)),
        _box_mesh((3.62, 1.56, 0.16), xyz=(-0.10, -0.80, 1.22), rpy=(0.63, 0.0, 0.0)),
        _box_mesh((3.36, 0.22, 0.22), xyz=(-0.10, 0.0, 1.56)),
    )


def _single_blade_mesh() -> MeshGeometry:
    blade_parts = [
        _box_mesh((0.16, 0.18, 1.32), xyz=(0.70, 0.0, 0.78)),
        _box_mesh((0.12, 1.06, 0.18), xyz=(0.68, 0.0, 1.42)),
        _box_mesh((0.08, 0.12, 3.86), xyz=(0.66, 0.50, 3.34)),
        _box_mesh((0.08, 0.12, 3.86), xyz=(0.66, -0.50, 3.34)),
        _box_mesh((0.08, 1.02, 0.16), xyz=(0.66, 0.0, 5.20)),
        _box_mesh((0.05, 0.08, 4.18), xyz=(0.66, 0.12, 3.27), rpy=(-0.23, 0.0, 0.0)),
        _box_mesh((0.05, 0.08, 4.18), xyz=(0.66, -0.12, 3.27), rpy=(0.23, 0.0, 0.0)),
    ]
    for z_pos in (1.86, 2.44, 3.02, 3.60, 4.18, 4.76):
        blade_parts.append(_box_mesh((0.05, 0.94, 0.08), xyz=(0.66, 0.0, z_pos)))
    return _merge_geometries(*blade_parts)


def _blade_lattice_mesh() -> MeshGeometry:
    single_blade = _single_blade_mesh()
    blades = MeshGeometry()
    for index in range(4):
        blades.merge(single_blade.copy().rotate_x(index * (math.pi / 2.0)))
    return blades


def _hub_core_mesh() -> MeshGeometry:
    return _merge_geometries(
        _cylinder_mesh(0.24, 0.04, axis="x", xyz=(0.10, 0.0, 0.0)),
        _cylinder_mesh(0.34, 0.10, axis="x", xyz=(0.20, 0.0, 0.0)),
        _cylinder_mesh(0.22, 0.34, axis="x", xyz=(0.42, 0.0, 0.0)),
        _cone_mesh(0.22, 0.28, axis="x", xyz=(0.73, 0.0, 0.0)),
    )


def _weather_hood_mesh() -> MeshGeometry:
    return _merge_geometries(
        _box_mesh((0.96, 1.18, 0.10), xyz=(2.48, 0.0, 1.74), rpy=(0.0, -0.16, 0.0)),
        _box_mesh((0.62, 0.12, 0.22), xyz=(2.42, 0.60, 1.63), rpy=(0.0, -0.16, 0.0)),
        _box_mesh((0.62, 0.12, 0.22), xyz=(2.42, -0.60, 1.63), rpy=(0.0, -0.16, 0.0)),
        _box_mesh((0.28, 1.00, 0.14), xyz=(2.00, 0.0, 1.67)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_weatherproof_windmill")

    stone = model.material("stone", rgba=(0.73, 0.71, 0.67, 1.0))
    oak = model.material("oak", rgba=(0.45, 0.31, 0.18, 1.0))
    shingle = model.material("shingle", rgba=(0.18, 0.18, 0.20, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.64, 0.66, 1.0))
    black_steel = model.material("black_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    sail_timber = model.material("sail_timber", rgba=(0.82, 0.79, 0.72, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=2.78, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=stone,
        name="foundation_plinth",
    )
    tower.visual(_save_mesh("tower_shell", _tower_shell_mesh()), material=stone, name="masonry_shell")
    tower.visual(
        Cylinder(radius=2.24, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=stone,
        name="lower_shell_transition",
    )
    tower.visual(
        Cylinder(radius=2.22, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=stone,
        name="base_corbel_ring",
    )
    tower.visual(
        Cylinder(radius=1.84, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 7.82)),
        material=stone,
        name="upper_shell_transition",
    )
    tower.visual(
        Cylinder(radius=1.62, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 7.88)),
        material=stone,
        name="tower_head_cap",
    )
    tower.visual(
        Cylinder(radius=0.26, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 8.05)),
        material=black_steel,
        name="kingpin_base_boss",
    )
    tower.visual(
        _save_mesh("tower_curb", _tube_mesh(inner_radius=0.24, outer_radius=1.74, length=0.18)),
        origin=Origin(xyz=(0.0, 0.0, 8.07)),
        material=galvanized,
        name="tower_curb",
    )
    tower.visual(
        Cylinder(radius=0.10, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 8.24)),
        material=black_steel,
        name="kingpin_post",
    )

    cap = model.part("cap")
    cap.visual(
        _save_mesh("yaw_bearing_ring", _tube_mesh(inner_radius=0.28, outer_radius=1.70, length=0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=galvanized,
        name="yaw_bearing_ring",
    )
    cap.visual(
        _save_mesh("yaw_sleeve", _tube_mesh(inner_radius=0.14, outer_radius=0.22, length=0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=galvanized,
        name="yaw_sleeve",
    )
    cap.visual(
        Box((0.26, 0.46, 0.18)),
        origin=Origin(xyz=(0.34, 0.32, 0.13)),
        material=oak,
        name="yaw_support_front_left",
    )
    cap.visual(
        Box((0.26, 0.46, 0.18)),
        origin=Origin(xyz=(0.34, -0.32, 0.13)),
        material=oak,
        name="yaw_support_front_right",
    )
    cap.visual(
        Box((0.26, 0.46, 0.18)),
        origin=Origin(xyz=(-0.34, 0.32, 0.13)),
        material=oak,
        name="yaw_support_rear_left",
    )
    cap.visual(
        Box((0.26, 0.46, 0.18)),
        origin=Origin(xyz=(-0.34, -0.32, 0.13)),
        material=oak,
        name="yaw_support_rear_right",
    )
    cap.visual(
        Box((0.72, 0.72, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=oak,
        name="yaw_sleeve_cap_plate",
    )
    cap.visual(
        Box((0.12, 0.12, 0.20)),
        origin=Origin(xyz=(0.34, 0.32, 0.30)),
        material=oak,
        name="yaw_post_front_left",
    )
    cap.visual(
        Box((0.12, 0.12, 0.20)),
        origin=Origin(xyz=(0.34, -0.32, 0.30)),
        material=oak,
        name="yaw_post_front_right",
    )
    cap.visual(
        Box((0.12, 0.12, 0.20)),
        origin=Origin(xyz=(-0.34, 0.32, 0.30)),
        material=oak,
        name="yaw_post_rear_left",
    )
    cap.visual(
        Box((0.12, 0.12, 0.20)),
        origin=Origin(xyz=(-0.34, -0.32, 0.30)),
        material=oak,
        name="yaw_post_rear_right",
    )
    cap.visual(
        Box((2.96, 0.34, 0.18)),
        origin=Origin(xyz=(0.00, 0.72, 0.13)),
        material=oak,
        name="left_sill",
    )
    cap.visual(
        Box((2.96, 0.34, 0.18)),
        origin=Origin(xyz=(0.00, -0.72, 0.13)),
        material=oak,
        name="right_sill",
    )
    cap.visual(
        Box((0.28, 1.74, 0.76)),
        origin=Origin(xyz=(-1.34, 0.0, 0.51)),
        material=oak,
        name="rear_bulkhead",
    )
    cap.visual(
        Box((3.02, 0.18, 0.18)),
        origin=Origin(xyz=(0.26, 0.54, 0.86)),
        material=oak,
        name="left_side_rail",
    )
    cap.visual(
        Box((3.02, 0.18, 0.18)),
        origin=Origin(xyz=(0.26, -0.54, 0.86)),
        material=oak,
        name="right_side_rail",
    )
    cap.visual(
        Box((0.60, 0.12, 0.62)),
        origin=Origin(xyz=(1.80, 0.44, 1.06)),
        material=oak,
        name="left_cheek_plate",
    )
    cap.visual(
        Box((0.60, 0.12, 0.62)),
        origin=Origin(xyz=(1.80, -0.44, 1.06)),
        material=oak,
        name="right_cheek_plate",
    )
    cap.visual(
        Box((1.18, 0.18, 0.16)),
        origin=Origin(xyz=(1.48, 0.56, 0.74)),
        material=oak,
        name="left_nose_rail",
    )
    cap.visual(
        Box((1.18, 0.18, 0.16)),
        origin=Origin(xyz=(1.48, -0.56, 0.74)),
        material=oak,
        name="right_nose_rail",
    )
    cap.visual(
        Box((0.18, 0.18, 0.72)),
        origin=Origin(xyz=(-1.04, 0.72, 0.49)),
        material=oak,
        name="left_rear_post",
    )
    cap.visual(
        Box((0.18, 0.18, 0.72)),
        origin=Origin(xyz=(-1.04, -0.72, 0.49)),
        material=oak,
        name="right_rear_post",
    )
    cap.visual(
        Box((0.18, 0.18, 0.72)),
        origin=Origin(xyz=(0.96, 0.72, 0.49)),
        material=oak,
        name="left_front_post",
    )
    cap.visual(
        Box((0.18, 0.18, 0.72)),
        origin=Origin(xyz=(0.96, -0.72, 0.49)),
        material=oak,
        name="right_front_post",
    )
    cap.visual(
        Box((3.64, 1.58, 0.16)),
        origin=Origin(xyz=(-0.10, 0.80, 1.22), rpy=(-0.63, 0.0, 0.0)),
        material=shingle,
        name="roof_left",
    )
    cap.visual(
        Box((3.64, 1.58, 0.16)),
        origin=Origin(xyz=(-0.10, -0.80, 1.22), rpy=(0.63, 0.0, 0.0)),
        material=shingle,
        name="roof_right",
    )
    cap.visual(
        Box((3.36, 0.22, 0.22)),
        origin=Origin(xyz=(-0.10, 0.0, 1.56)),
        material=shingle,
        name="roof_ridge",
    )
    cap.visual(
        _save_mesh(
            "rear_bearing_sleeve",
            _tube_mesh(inner_radius=0.075, outer_radius=0.15, length=0.14, axis="x"),
        ),
        origin=Origin(xyz=(1.50, 0.0, 1.06)),
        material=galvanized,
        name="rear_bearing_sleeve",
    )
    cap.visual(
        _save_mesh(
            "front_bearing_sleeve",
            _tube_mesh(inner_radius=0.075, outer_radius=0.15, length=0.18, axis="x"),
        ),
        origin=Origin(xyz=(1.86, 0.0, 1.06)),
        material=galvanized,
        name="front_bearing_sleeve",
    )
    cap.visual(
        Box((0.96, 0.58, 0.10)),
        origin=Origin(xyz=(1.68, 0.0, 1.28)),
        material=oak,
        name="bearing_bed_upper",
    )
    cap.visual(
        Box((0.96, 0.58, 0.10)),
        origin=Origin(xyz=(1.68, 0.0, 0.84)),
        material=oak,
        name="bearing_bed_lower",
    )
    cap.visual(
        Box((0.14, 0.16, 0.44)),
        origin=Origin(xyz=(1.50, 0.23, 1.06)),
        material=oak,
        name="rear_bearing_web_left",
    )
    cap.visual(
        Box((0.14, 0.16, 0.44)),
        origin=Origin(xyz=(1.50, -0.23, 1.06)),
        material=oak,
        name="rear_bearing_web_right",
    )
    cap.visual(
        Box((0.18, 0.16, 0.44)),
        origin=Origin(xyz=(1.86, 0.23, 1.06)),
        material=oak,
        name="front_bearing_web_left",
    )
    cap.visual(
        Box((0.18, 0.16, 0.44)),
        origin=Origin(xyz=(1.86, -0.23, 1.06)),
        material=oak,
        name="front_bearing_web_right",
    )
    cap.visual(
        Box((0.78, 1.12, 0.10)),
        origin=Origin(xyz=(1.84, 0.0, 1.72), rpy=(0.0, -0.18, 0.0)),
        material=galvanized,
        name="weather_hood_top",
    )
    cap.visual(
        Box((0.58, 0.12, 0.26)),
        origin=Origin(xyz=(1.74, 0.56, 1.57), rpy=(0.0, -0.18, 0.0)),
        material=galvanized,
        name="weather_hood_side_left",
    )
    cap.visual(
        Box((0.58, 0.12, 0.26)),
        origin=Origin(xyz=(1.74, -0.56, 1.57), rpy=(0.0, -0.18, 0.0)),
        material=galvanized,
        name="weather_hood_side_right",
    )
    cap.visual(
        Box((0.22, 1.06, 0.14)),
        origin=Origin(xyz=(1.44, 0.0, 1.34)),
        material=galvanized,
        name="weather_hood_rear_lip",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.07, length=0.98),
        origin=Origin(xyz=(-0.27, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(-0.69, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_thrust_collar",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(-0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="front_thrust_collar",
    )
    rotor.visual(_save_mesh("hub_core", _hub_core_mesh()), material=galvanized, name="hub_core")
    rotor.visual(
        _save_mesh("blade_lattice", _blade_lattice_mesh()),
        material=sail_timber,
        name="blade_lattice",
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 8.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(2.10, 0.0, 1.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw_joint = object_model.get_articulation("tower_to_cap")
    rotor_joint = object_model.get_articulation("cap_to_rotor")

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
        "tower_to_cap axis is vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        f"Expected vertical yaw axis, got {yaw_joint.axis!r}",
    )
    ctx.check(
        "cap_to_rotor axis is fore-aft",
        rotor_joint.axis == (1.0, 0.0, 0.0),
        f"Expected fore-aft rotor axis, got {rotor_joint.axis!r}",
    )

    ctx.expect_contact(cap, tower, elem_a="yaw_bearing_ring", elem_b="tower_curb")
    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="yaw_bearing_ring",
        negative_elem="tower_curb",
        max_gap=0.001,
        max_penetration=0.0,
        name="cap bearing ring seats on curb",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        elem_a="yaw_bearing_ring",
        elem_b="tower_curb",
        min_overlap=3.2,
        name="cap curb footprint overlaps tower curb",
    )

    ctx.expect_contact(rotor, cap, elem_a="front_thrust_collar", elem_b="front_bearing_sleeve")
    ctx.expect_contact(rotor, cap, elem_a="rear_thrust_collar", elem_b="rear_bearing_sleeve")
    ctx.expect_gap(
        rotor,
        tower,
        axis="z",
        positive_elem="blade_lattice",
        negative_elem="foundation_plinth",
        min_gap=3.0,
        name="blade lattice clears plinth",
    )

    with ctx.pose({yaw_joint: math.pi / 2.0}):
        rotor_pos = ctx.part_world_position(rotor)
        ok = (
            rotor_pos is not None
            and abs(rotor_pos[0]) <= 0.08
            and rotor_pos[1] >= 1.24
            and rotor_pos[2] >= 9.0
        )
        ctx.check(
            "cap yaw carries rotor around tower",
            ok,
            f"Unexpected rotor position after quarter-turn yaw: {rotor_pos!r}",
        )

    with ctx.pose({rotor_joint: math.pi / 4.0}):
        ctx.expect_contact(
            rotor,
            cap,
            elem_a="front_thrust_collar",
            elem_b="front_bearing_sleeve",
            name="front collar stays seated while spinning",
        )
        ctx.expect_contact(
            rotor,
            cap,
            elem_a="rear_thrust_collar",
            elem_b="rear_bearing_sleeve",
            name="rear collar stays seated while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
