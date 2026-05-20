from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _connect_rings(
    geom: MeshGeometry,
    ring_a: list[int],
    ring_b: list[int],
    *,
    flip: bool = False,
) -> None:
    count = len(ring_a)
    for i in range(count):
        a0 = ring_a[i]
        a1 = ring_a[(i + 1) % count]
        b0 = ring_b[i]
        b1 = ring_b[(i + 1) % count]
        if flip:
            geom.add_face(a0, b1, b0)
            geom.add_face(a0, a1, b1)
        else:
            geom.add_face(a0, b0, b1)
            geom.add_face(a0, b1, a1)


def _ring_vertices(
    geom: MeshGeometry, radius: float, z: float, segments: int
) -> list[int]:
    ring: list[int] = []
    for i in range(segments):
        angle = math.tau * i / segments
        ring.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z))
    return ring


def _revolve_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
    close_top: bool = True,
    close_bottom: bool = True,
) -> MeshGeometry:
    geom = MeshGeometry()

    outer_rings = [_ring_vertices(geom, radius, z, segments) for radius, z in outer_profile]
    inner_rings = [_ring_vertices(geom, radius, z, segments) for radius, z in inner_profile]

    for ring_a, ring_b in zip(outer_rings[:-1], outer_rings[1:]):
        _connect_rings(geom, ring_a, ring_b)
    for ring_a, ring_b in zip(inner_rings[:-1], inner_rings[1:]):
        _connect_rings(geom, ring_a, ring_b, flip=True)

    if close_top:
        _connect_rings(geom, outer_rings[0], inner_rings[0], flip=True)
    if close_bottom:
        _connect_rings(geom, inner_rings[-1], outer_rings[-1], flip=True)

    return geom


def _annular_band_mesh(
    *,
    outer_bottom: float,
    outer_top: float,
    inner_bottom: float,
    inner_top: float,
    z_bottom: float,
    z_top: float,
    name: str,
    segments: int = 72,
):
    return mesh_from_geometry(
        _revolve_shell(
            [(outer_bottom, z_bottom), (outer_top, z_top)],
            [(inner_bottom, z_bottom), (inner_top, z_top)],
            segments=segments,
            close_top=True,
            close_bottom=True,
        ),
        name,
    )


def _bell_shell_mesh():
    return mesh_from_geometry(
        _revolve_shell(
            [
                (0.12, -0.82),
                (0.20, -0.90),
                (0.36, -1.04),
                (0.52, -1.34),
                (0.62, -1.58),
            ],
            [
                (0.06, -0.84),
                (0.13, -0.92),
                (0.28, -1.08),
                (0.44, -1.36),
                (0.54, -1.54),
            ],
            segments=88,
            close_top=True,
            close_bottom=False,
        ),
        "bronze_bell_shell",
    )


def _polar_point(
    angle: float,
    radial: float,
    tangential: float,
    z: float,
) -> tuple[float, float, float]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (
        radial * ca - tangential * sa,
        radial * sa + tangential * ca,
        z,
    )


def _add_radial_box(
    part,
    size: tuple[float, float, float],
    *,
    angle: float,
    radial: float,
    tangential: float,
    z: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=_polar_point(angle, radial, tangential, z), rpy=(0.0, 0.0, angle)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_belfry_tower")

    stone = model.material("stone", rgba=(0.66, 0.66, 0.63, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.54, 0.54, 0.52, 1.0))
    roof_slate = model.material("roof_slate", rgba=(0.22, 0.23, 0.25, 1.0))
    oak = model.material("oak", rgba=(0.46, 0.30, 0.16, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.56, 0.38, 0.18, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.19, 1.0))

    lower_shell = _annular_band_mesh(
        outer_bottom=1.95,
        outer_top=1.82,
        inner_bottom=1.42,
        inner_top=1.40,
        z_bottom=0.38,
        z_top=6.55,
        name="tower_lower_shell",
    )
    sill_ring = _annular_band_mesh(
        outer_bottom=1.84,
        outer_top=1.80,
        inner_bottom=1.41,
        inner_top=1.40,
        z_bottom=6.55,
        z_top=6.80,
        name="tower_sill_ring",
    )
    cornice_ring = _annular_band_mesh(
        outer_bottom=1.83,
        outer_top=1.73,
        inner_bottom=1.40,
        inner_top=1.34,
        z_bottom=8.55,
        z_top=8.95,
        name="tower_cornice_ring",
    )
    roof_mesh = mesh_from_geometry(
        ConeGeometry(radius=1.55, height=1.55, radial_segments=72).translate(0.0, 0.0, 9.725),
        "tower_roof",
    )
    bell_shell = _bell_shell_mesh()

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=2.05, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=stone_dark,
        name="plinth",
    )
    tower.visual(lower_shell, material=stone, name="shaft")
    tower.visual(sill_ring, material=stone_dark, name="sill_ring")

    window_angles = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    for idx, angle in enumerate(window_angles):
        _add_radial_box(
            tower,
            (0.42, 1.58, 0.16),
            angle=angle,
            radial=1.61,
            tangential=0.0,
            z=6.88,
            material=stone_dark,
            name=f"window_sill_{idx}",
        )
        for side, tangential in (("left", -0.63), ("right", 0.63)):
            _add_radial_box(
                tower,
                (0.42, 0.28, 1.33),
                angle=angle,
                radial=1.61,
                tangential=tangential,
                z=7.465,
                material=stone,
                name=f"{side}_jamb_{idx}",
            )

        arch_radius = 0.50
        arch_angles = (
            math.radians(160),
            math.radians(138),
            math.radians(116),
            math.radians(90),
            math.radians(64),
            math.radians(42),
            math.radians(20),
        )
        for stone_index, arch_angle in enumerate(arch_angles):
            _add_radial_box(
                tower,
                (0.42, 0.18, 0.22),
                angle=angle,
                radial=1.61,
                tangential=arch_radius * math.cos(arch_angle),
                z=8.05 + arch_radius * math.sin(arch_angle),
                material=stone,
                name=f"arch_stone_{idx}_{stone_index}",
            )

    for idx, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        _add_radial_box(
            tower,
            (0.42, 0.84, 1.81),
            angle=angle,
            radial=1.61,
            tangential=0.0,
            z=7.675,
            material=stone,
            name=f"corner_pier_{idx}",
        )

    tower.visual(cornice_ring, material=stone_dark, name="cornice")
    tower.visual(roof_mesh, material=roof_slate, name="roof")
    tower.visual(
        Cylinder(radius=0.06, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 10.61)),
        material=roof_slate,
        name="finial",
    )
    tower.visual(
        Box((3.10, 0.20, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 8.56)),
        material=oak,
        name="tie_beam",
    )
    for idx, angle in enumerate((math.pi / 2.0, 3.0 * math.pi / 2.0)):
        _add_radial_box(
            tower,
            (0.36, 0.46, 0.82),
            angle=angle,
            radial=1.60,
            tangential=0.0,
            z=8.14,
            material=stone_dark,
            name=f"beam_socket_{idx}",
        )
    for side, y_sign in (("left", -1.0), ("right", 1.0)):
        for x_sign, suffix in ((-1.0, "inner"), (1.0, "outer")):
            tower.visual(
                Box((0.16, 0.64, 0.26)),
                origin=Origin(xyz=(x_sign * 0.14, y_sign * 1.18, 8.19)),
                material=oak,
                name=f"{side}_bearing_bracket_{suffix}",
            )
        tower.visual(
            Cylinder(radius=0.06, length=0.14),
            origin=Origin(
                xyz=(0.0, y_sign * 0.90, 8.28),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_iron,
            name=f"{side}_bearing",
        )

    bell = model.part("bell_yoke")
    bell.visual(
        Box((0.18, 1.52, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=oak,
        name="yoke_beam",
    )
    for side, y_sign in (("left", -1.0), ("right", 1.0)):
        bell.visual(
            Cylinder(radius=0.04, length=0.20),
            origin=Origin(
                xyz=(0.0, y_sign * 0.86, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_iron,
            name=f"{side}_gudgeon",
        )
    for side, x_sign in (("front", 1.0), ("rear", -1.0)):
        bell.visual(
            Box((0.08, 0.20, 0.63)),
            origin=Origin(xyz=(x_sign * 0.13, 0.0, -0.385)),
            material=oak,
            name=f"{side}_yoke_cheek",
        )
    bell.visual(
        Box((0.18, 0.50, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.76)),
        material=oak,
        name="crown_block",
    )
    bell.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.86)),
        material=aged_bronze,
        name="crown_boss",
    )
    bell.visual(bell_shell, material=aged_bronze, name="bell_shell")
    for side, y_sign, x_web, x_eye in (("left", -1.0, 0.04, 0.012), ("right", 1.0, -0.04, -0.012)):
        bell.visual(
            Box((0.03, 0.04, 0.19)),
            origin=Origin(xyz=(x_web, y_sign * 0.09, -0.905)),
            material=dark_iron,
            name=f"{side}_hanger_web",
        )
        bell.visual(
            Cylinder(radius=0.028, length=0.05),
            origin=Origin(
                xyz=(x_eye, y_sign * 0.09, -1.00),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_iron,
            name=f"{side}_hanger_eye",
        )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="clapper_pin",
    )
    clapper.visual(
        Cylinder(radius=0.016, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=dark_iron,
        name="clapper_shank",
    )
    clapper.visual(
        Cylinder(radius=0.028, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, -0.49)),
        material=dark_iron,
        name="clapper_rod",
    )
    clapper.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.84)),
        material=dark_iron,
        name="clapper_head",
    )
    clapper.visual(
        Sphere(radius=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.98)),
        material=dark_iron,
        name="clapper_bulb",
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 8.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.0,
            lower=-0.78,
            upper=0.78,
        ),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -1.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=3.0,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell_yoke")
    clapper = object_model.get_part("clapper")
    bell_joint = object_model.get_articulation("tower_to_bell")
    clapper_joint = object_model.get_articulation("bell_to_clapper")

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
        bell,
        clapper,
        elem_a="left_hanger_eye",
        elem_b="clapper_pin",
        reason="Clapper pin intentionally passes through the left hanger eye.",
    )
    ctx.allow_overlap(
        bell,
        clapper,
        elem_a="right_hanger_eye",
        elem_b="clapper_pin",
        reason="Clapper pin intentionally passes through the right hanger eye.",
    )
    ctx.allow_overlap(
        tower,
        bell,
        elem_a="left_bearing",
        elem_b="left_gudgeon",
        reason="Left gudgeon journal runs inside the tower bearing.",
    )
    ctx.allow_overlap(
        tower,
        bell,
        elem_a="right_bearing",
        elem_b="right_gudgeon",
        reason="Right gudgeon journal runs inside the tower bearing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(bell, tower, inner_elem="left_gudgeon", outer_elem="left_bearing", axes="xz", name="left_gudgeon_is_seated_in_bearing")
    ctx.expect_within(bell, tower, inner_elem="right_gudgeon", outer_elem="right_bearing", axes="xz", name="right_gudgeon_is_seated_in_bearing")
    ctx.expect_overlap(tower, bell, elem_a="left_bearing", elem_b="left_gudgeon", axes="y", min_overlap=0.12, name="left_gudgeon_overlaps_bearing_length")
    ctx.expect_overlap(tower, bell, elem_a="right_bearing", elem_b="right_gudgeon", axes="y", min_overlap=0.12, name="right_gudgeon_overlaps_bearing_length")
    ctx.expect_contact(bell, clapper, elem_a="left_hanger_eye", elem_b="clapper_pin", name="left_hanger_supports_clapper")
    ctx.expect_contact(bell, clapper, elem_a="right_hanger_eye", elem_b="clapper_pin", name="right_hanger_supports_clapper")
    ctx.expect_within(bell, tower, axes="xy", margin=0.05, name="bell_stays_inside_tower_plan")

    bell_rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    assert bell_rest_aabb is not None
    bell_rest_center_x = 0.5 * (bell_rest_aabb[0][0] + bell_rest_aabb[1][0])
    with ctx.pose({bell_joint: 0.65}):
        bell_forward_aabb = ctx.part_world_aabb(bell)
        assert bell_forward_aabb is not None
        bell_forward_shell = ctx.part_element_world_aabb(bell, elem="bell_shell")
        assert bell_forward_shell is not None
        bell_forward_center_x = 0.5 * (bell_forward_shell[0][0] + bell_forward_shell[1][0])
        ctx.check(
            "bell_swings_toward_positive_x",
            bell_forward_center_x > bell_rest_center_x + 0.42,
            f"expected bell shell center-x to increase by > 0.42 m, got {bell_forward_center_x - bell_rest_center_x:.3f} m",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_forward_bell_swing")

    with ctx.pose({bell_joint: -0.65}):
        bell_reverse_shell = ctx.part_element_world_aabb(bell, elem="bell_shell")
        assert bell_reverse_shell is not None
        bell_reverse_center_x = 0.5 * (bell_reverse_shell[0][0] + bell_reverse_shell[1][0])
        ctx.check(
            "bell_swings_toward_negative_x",
            bell_reverse_center_x < bell_rest_center_x - 0.42,
            f"expected bell shell center-x to decrease by > 0.42 m, got {bell_reverse_center_x - bell_rest_center_x:.3f} m",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_reverse_bell_swing")

    clapper_rest_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_bulb")
    assert clapper_rest_aabb is not None
    clapper_rest_center_x = 0.5 * (clapper_rest_aabb[0][0] + clapper_rest_aabb[1][0])
    with ctx.pose({clapper_joint: 0.35}):
        clapper_swung_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_bulb")
        assert clapper_swung_aabb is not None
        clapper_swung_center_x = 0.5 * (clapper_swung_aabb[0][0] + clapper_swung_aabb[1][0])
        ctx.check(
            "clapper_swings_inside_bell",
            clapper_swung_center_x > clapper_rest_center_x + 0.16,
            f"expected clapper bulb center-x to increase by > 0.16 m, got {clapper_swung_center_x - clapper_rest_center_x:.3f} m",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_clapper_deflection")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
