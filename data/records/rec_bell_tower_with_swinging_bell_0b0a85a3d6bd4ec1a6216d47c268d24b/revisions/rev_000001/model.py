from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _add_quad(geom: MeshGeometry, a, b, c, d) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _in_arch_window(theta: float, z: float) -> bool:
    """Four round-headed through openings in the belfry tier."""
    base_z = 3.25
    spring_z = 4.10
    top_z = 4.78
    half_angle = 0.33
    if z < base_z or z > top_z:
        return False
    for center in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        d = abs(_angle_delta(theta, center))
        if d > half_angle:
            continue
        if z <= spring_z:
            return True
        arch = ((d / half_angle) ** 2) + (((z - spring_z) / (top_z - spring_z)) ** 2)
        if arch <= 1.0:
            return True
    return False


def _cylindrical_tower_shell(
    *,
    outer_radius: float = 1.15,
    inner_radius: float = 0.92,
    height: float = 5.40,
    radial_segments: int = 144,
    z_segments: int = 150,
) -> MeshGeometry:
    """A hollow round stone shell with gridded arched window cut-outs."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for i in range(radial_segments):
        theta = math.tau * i / radial_segments
        c = math.cos(theta)
        s = math.sin(theta)
        outer_row: list[int] = []
        inner_row: list[int] = []
        for j in range(z_segments + 1):
            z = height * j / z_segments
            outer_row.append(geom.add_vertex(outer_radius * c, outer_radius * s, z))
            inner_row.append(geom.add_vertex(inner_radius * c, inner_radius * s, z))
        outer.append(outer_row)
        inner.append(inner_row)

    solid = [[False for _ in range(z_segments)] for _ in range(radial_segments)]
    for i in range(radial_segments):
        theta_mid = math.tau * (i + 0.5) / radial_segments
        for j in range(z_segments):
            z_mid = height * (j + 0.5) / z_segments
            solid[i][j] = not _in_arch_window(theta_mid, z_mid)

    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        pi = (i - 1) % radial_segments
        for j in range(z_segments):
            if not solid[i][j]:
                continue

            # Outer and inner wall faces.
            _add_quad(geom, outer[i][j], outer[ni][j], outer[ni][j + 1], outer[i][j + 1])
            _add_quad(geom, inner[ni][j], inner[i][j], inner[i][j + 1], inner[ni][j + 1])

            # Radial jamb faces wherever the neighboring angular cell is open.
            if not solid[pi][j]:
                _add_quad(geom, inner[i][j], outer[i][j], outer[i][j + 1], inner[i][j + 1])
            if not solid[ni][j]:
                _add_quad(geom, outer[ni][j], inner[ni][j], inner[ni][j + 1], outer[ni][j + 1])

            # Horizontal sill and arched-head thickness faces.
            if j == 0 or not solid[i][j - 1]:
                _add_quad(geom, outer[i][j], inner[i][j], inner[ni][j], outer[ni][j])
            if j == z_segments - 1 or not solid[i][j + 1]:
                _add_quad(
                    geom,
                    outer[ni][j + 1],
                    inner[ni][j + 1],
                    inner[i][j + 1],
                    outer[i][j + 1],
                )

    # Annular top and bottom caps.
    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        _add_quad(geom, inner[i][0], inner[ni][0], outer[ni][0], outer[i][0])
        _add_quad(geom, outer[i][z_segments], outer[ni][z_segments], inner[ni][z_segments], inner[i][z_segments])

    return geom


def _annular_band(
    outer_radius: float,
    inner_radius: float,
    z_center: float,
    height: float,
    *,
    radial_segments: int = 96,
) -> MeshGeometry:
    geom = MeshGeometry()
    bottom = z_center - height * 0.5
    top = z_center + height * 0.5
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for i in range(radial_segments):
        theta = math.tau * i / radial_segments
        c = math.cos(theta)
        s = math.sin(theta)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, bottom))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, top))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, bottom))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, top))
    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        _add_quad(geom, outer_bottom[i], outer_bottom[ni], outer_top[ni], outer_top[i])
        _add_quad(geom, inner_bottom[ni], inner_bottom[i], inner_top[i], inner_top[ni])
        _add_quad(geom, outer_top[i], outer_top[ni], inner_top[ni], inner_top[i])
        _add_quad(geom, inner_bottom[i], inner_bottom[ni], outer_bottom[ni], outer_bottom[i])
    return geom


def _bell_shell() -> MeshGeometry:
    outer_profile = [
        (0.16, -0.18),
        (0.23, -0.25),
        (0.28, -0.42),
        (0.34, -0.70),
        (0.45, -1.05),
        (0.54, -1.27),
        (0.56, -1.35),
    ]
    inner_profile = [
        (0.08, -0.21),
        (0.16, -0.30),
        (0.22, -0.48),
        (0.29, -0.76),
        (0.39, -1.08),
        (0.49, -1.28),
        (0.52, -1.35),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    shell.merge(TorusGeometry(radius=0.535, tube=0.030, radial_segments=16, tubular_segments=96).translate(0.0, 0.0, -1.33))
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detached_round_belfry")

    stone = model.material("warm_limestone", rgba=(0.62, 0.59, 0.51, 1.0))
    shadow_stone = model.material("shadowed_stone", rgba=(0.42, 0.40, 0.36, 1.0))
    oak = model.material("aged_oak", rgba=(0.45, 0.28, 0.13, 1.0))
    bronze = model.material("cast_bronze", rgba=(0.68, 0.43, 0.18, 1.0))
    dark_iron = model.material("dark_wrought_iron", rgba=(0.08, 0.08, 0.075, 1.0))

    tower = model.part("tower")
    tower.visual(
        _mesh(_cylindrical_tower_shell(), "tower_shell"),
        material=stone,
        name="tower_shell",
    )
    # Raised stone courses and cap rings make the otherwise round shaft read as masonry.
    for idx, z in enumerate((0.18, 1.05, 1.95, 2.85, 3.15, 4.96, 5.28)):
        tower.visual(
            _mesh(_annular_band(1.19, 0.90, z, 0.09 if idx not in (0, 6) else 0.16), f"stone_course_{idx}"),
            material=shadow_stone if idx in (3, 4) else stone,
            name=f"stone_course_{idx}",
        )
    # Two opposed bearing blocks and tie-beam stubs are fixed into the belfry wall.
    tower.visual(
        Box((0.38, 0.95, 0.18)),
        origin=Origin(xyz=(0.78, 0.0, 4.25)),
        material=oak,
        name="tie_beam_pos",
    )
    tower.visual(
        Box((0.38, 0.95, 0.18)),
        origin=Origin(xyz=(-0.78, 0.0, 4.25)),
        material=oak,
        name="tie_beam_neg",
    )
    tower.visual(
        Box((0.22, 0.70, 0.36)),
        origin=Origin(xyz=(0.51, 0.0, 4.25)),
        material=oak,
        name="bearing_pos",
    )
    tower.visual(
        Box((0.22, 0.70, 0.36)),
        origin=Origin(xyz=(-0.51, 0.0, 4.25)),
        material=oak,
        name="bearing_neg",
    )

    bell_yoke = model.part("bell_yoke")
    bell_yoke.visual(
        Cylinder(radius=0.055, length=1.12),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="trunnion_pin",
    )
    bell_yoke.visual(
        Box((0.78, 0.20, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=oak,
        name="wooden_yoke",
    )
    bell_yoke.visual(
        Box((0.08, 0.13, 0.42)),
        origin=Origin(xyz=(0.24, 0.0, -0.25)),
        material=dark_iron,
        name="strap_0",
    )
    bell_yoke.visual(
        Box((0.08, 0.13, 0.42)),
        origin=Origin(xyz=(-0.24, 0.0, -0.25)),
        material=dark_iron,
        name="strap_1",
    )
    bell_yoke.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
        material=bronze,
        name="bell_crown",
    )
    bell_yoke.visual(
        Box((0.08, 0.10, 0.22)),
        origin=Origin(xyz=(0.12, 0.0, -0.39)),
        material=dark_iron,
        name="hanger_pos",
    )
    bell_yoke.visual(
        Box((0.08, 0.10, 0.22)),
        origin=Origin(xyz=(-0.12, 0.0, -0.39)),
        material=dark_iron,
        name="hanger_neg",
    )
    bell_yoke.visual(
        _mesh(_bell_shell(), "bell_shell"),
        material=bronze,
        name="bell_shell",
    )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.025, length=0.28),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="clapper_pin",
    )
    clapper.visual(
        Cylinder(radius=0.020, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, -0.34)),
        material=dark_iron,
        name="clapper_rod",
    )
    clapper.visual(
        Sphere(radius=0.115),
        origin=Origin(xyz=(0.0, 0.0, -0.72)),
        material=dark_iron,
        name="clapper_ball",
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_yoke,
        origin=Origin(xyz=(0.0, 0.0, 4.25)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.3, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "clapper_pin",
        ArticulationType.REVOLUTE,
        parent=bell_yoke,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.42)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell_yoke")
    clapper = object_model.get_part("clapper")
    swing = object_model.get_articulation("bell_swing")
    clapper_joint = object_model.get_articulation("clapper_pin")

    ctx.allow_overlap(
        tower,
        bell,
        elem_a="bearing_pos",
        elem_b="trunnion_pin",
        reason="The trunnion pin is intentionally captured in the positive-side wooden bearing block.",
    )
    ctx.allow_overlap(
        tower,
        bell,
        elem_a="bearing_neg",
        elem_b="trunnion_pin",
        reason="The trunnion pin is intentionally captured in the negative-side wooden bearing block.",
    )
    ctx.allow_overlap(
        bell,
        clapper,
        elem_a="hanger_pos",
        elem_b="clapper_pin",
        reason="The clapper pin is intentionally captured in the positive-side internal hanger lug.",
    )
    ctx.allow_overlap(
        bell,
        clapper,
        elem_a="hanger_neg",
        elem_b="clapper_pin",
        reason="The clapper pin is intentionally captured in the negative-side internal hanger lug.",
    )
    ctx.expect_gap(
        tower,
        bell,
        axis="x",
        positive_elem="bearing_pos",
        negative_elem="trunnion_pin",
        max_gap=0.0,
        max_penetration=0.18,
        name="positive trunnion is seated in bearing",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="x",
        positive_elem="trunnion_pin",
        negative_elem="bearing_neg",
        max_gap=0.0,
        max_penetration=0.18,
        name="negative trunnion is seated in bearing",
    )
    ctx.expect_overlap(
        bell,
        clapper,
        axes="x",
        elem_a="hanger_pos",
        elem_b="clapper_pin",
        min_overlap=0.035,
        name="clapper pin is retained in positive hanger",
    )
    ctx.expect_overlap(
        bell,
        clapper,
        axes="x",
        elem_a="hanger_neg",
        elem_b="clapper_pin",
        min_overlap=0.035,
        name="clapper pin is retained in negative hanger",
    )

    def _elem_center_y(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    bell_shell_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    ctx.check(
        "bell hangs clear above the stone floor",
        bell_shell_aabb is not None and bell_shell_aabb[0][2] > 2.50,
        details=f"bell_shell_aabb={bell_shell_aabb}",
    )

    rest_bell_y = _elem_center_y(bell, "bell_shell")
    with ctx.pose({swing: 0.32}):
        swung_bell_y = _elem_center_y(bell, "bell_shell")
    ctx.check(
        "bell swings laterally on the horizontal tie-beam axis",
        rest_bell_y is not None and swung_bell_y is not None and swung_bell_y > rest_bell_y + 0.20,
        details=f"rest_y={rest_bell_y}, swung_y={swung_bell_y}",
    )

    rest_ball_y = _elem_center_y(clapper, "clapper_ball")
    with ctx.pose({clapper_joint: -0.45}):
        tilted_ball_y = _elem_center_y(clapper, "clapper_ball")
    ctx.check(
        "clapper rod swings independently on its secondary pin",
        rest_ball_y is not None and tilted_ball_y is not None and tilted_ball_y < rest_ball_y - 0.20,
        details=f"rest_y={rest_ball_y}, tilted_y={tilted_ball_y}",
    )

    return ctx.report()


object_model = build_object_model()
