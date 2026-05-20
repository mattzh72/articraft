from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
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


def _arched_loop(width: float, bottom_y: float, spring_y: float, *, segments: int = 18):
    """Closed 2D loop for a round-arched opening profile in local panel XY."""
    radius = width * 0.5
    points: list[tuple[float, float]] = [(-radius, bottom_y), (-radius, spring_y)]
    for index in range(1, segments + 1):
        angle = math.pi - (math.pi * index / segments)
        points.append((radius * math.cos(angle), spring_y + radius * math.sin(angle)))
    points.append((radius, bottom_y))
    return points


def _rect_loop(width: float, height: float):
    half_w = width * 0.5
    half_h = height * 0.5
    return [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]


def _belfry_wall_mesh() -> MeshGeometry:
    """A continuous brick wall panel with a true round-arched through opening."""
    hole = _arched_loop(1.30, -1.05, 0.10, segments=24)
    return ExtrudeWithHolesGeometry(
        _rect_loop(2.46, 2.58),
        [hole],
        height=0.18,
        center=True,
    )


def _arch_trim_mesh() -> MeshGeometry:
    """A raised stone jamb-and-arch surround around a belfry opening."""
    outer = _arched_loop(1.58, -1.18, 0.10, segments=28)
    inner = _arched_loop(1.30, -1.04, 0.10, segments=28)
    return ExtrudeWithHolesGeometry(outer, [inner], height=0.045, center=True)


def _pyramid_roof_mesh(base_width: float, height: float) -> MeshGeometry:
    half = base_width * 0.5
    geom = MeshGeometry()
    v0 = geom.add_vertex(-half, -half, 0.0)
    v1 = geom.add_vertex(half, -half, 0.0)
    v2 = geom.add_vertex(half, half, 0.0)
    v3 = geom.add_vertex(-half, half, 0.0)
    apex = geom.add_vertex(0.0, 0.0, height)
    # Four roof planes plus a hidden underside so the mesh is a closed solid.
    geom.add_face(v0, v1, apex)
    geom.add_face(v1, v2, apex)
    geom.add_face(v2, v3, apex)
    geom.add_face(v3, v0, apex)
    geom.add_face(v0, v3, v2)
    geom.add_face(v0, v2, v1)
    return geom


def _bell_shell_mesh() -> MeshGeometry:
    """Thin-walled cast-iron bell: hollow, flared, and open at the mouth."""
    outer = [
        (0.18, -0.26),
        (0.24, -0.34),
        (0.30, -0.54),
        (0.36, -0.78),
        (0.47, -1.04),
        (0.54, -1.20),
        (0.50, -1.28),
    ]
    inner = [
        (0.08, -0.29),
        (0.15, -0.38),
        (0.22, -0.58),
        (0.29, -0.82),
        (0.40, -1.06),
        (0.47, -1.18),
        (0.47, -1.30),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def _add_campanile_static(
    tower,
    *,
    brick,
    dark_brick,
    stone,
    roof_tile,
    wood,
    iron,
):
    # Main square shaft and stepped brick plinth.
    tower.visual(
        Box((2.60, 2.60, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=brick,
        name="base_plinth",
    )
    tower.visual(
        Box((2.20, 2.20, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=dark_brick,
        name="lower_stringcourse",
    )
    tower.visual(
        Box((1.88, 1.88, 7.55)),
        origin=Origin(xyz=(0.0, 0.0, 4.455)),
        material=brick,
        name="square_shaft",
    )

    # Slightly raised corner pilasters make the shaft read as masonry, not a plain block.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.20, 0.20, 7.70)),
                origin=Origin(xyz=(sx * 0.94, sy * 0.94, 4.53)),
                material=dark_brick,
                name=f"corner_pilaster_{sx:+.0f}_{sy:+.0f}",
            )

    # Thin proud mortar/string courses around the square shaft.
    for index, z in enumerate([0.95 + 0.32 * i for i in range(22)]):
        tower.visual(
            Box((1.94, 0.028, 0.018)),
            origin=Origin(xyz=(0.0, 0.958, z)),
            material=dark_brick,
            name=f"front_mortar_{index}",
        )
        tower.visual(
            Box((1.94, 0.028, 0.018)),
            origin=Origin(xyz=(0.0, -0.958, z)),
            material=dark_brick,
            name=f"rear_mortar_{index}",
        )
        tower.visual(
            Box((0.028, 1.94, 0.018)),
            origin=Origin(xyz=(0.958, 0.0, z)),
            material=dark_brick,
            name=f"side_mortar_{index}_0",
        )
        tower.visual(
            Box((0.028, 1.94, 0.018)),
            origin=Origin(xyz=(-0.958, 0.0, z)),
            material=dark_brick,
            name=f"side_mortar_{index}_1",
        )

    # Belfry floor and top cornices tie all four arched walls into one supported tier.
    tower.visual(
        Box((2.52, 2.52, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 8.38)),
        material=dark_brick,
        name="belfry_floor",
    )
    tower.visual(
        Box((2.78, 2.78, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 8.68)),
        material=brick,
        name="lower_cornice",
    )

    wall_mesh = _save_mesh("arched_belfry_wall", _belfry_wall_mesh())
    trim_mesh = _save_mesh("stone_arch_trim", _arch_trim_mesh())
    panel_z = 9.92
    face_offset = 1.23
    trim_offset = 1.345
    for sign, label in ((1.0, "front"), (-1.0, "rear")):
        tower.visual(
            wall_mesh,
            origin=Origin(xyz=(0.0, sign * face_offset, panel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brick,
            name=f"{label}_arched_wall",
        )
        tower.visual(
            trim_mesh,
            origin=Origin(xyz=(0.0, sign * trim_offset, panel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stone,
            name=f"{label}_arch_trim",
        )
    for sign, label in ((1.0, "side_0"), (-1.0, "side_1")):
        tower.visual(
            wall_mesh,
            origin=Origin(
                xyz=(sign * face_offset, 0.0, panel_z),
                rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
            ),
            material=brick,
            name=f"{label}_arched_wall",
        )
        tower.visual(
            trim_mesh,
            origin=Origin(
                xyz=(sign * trim_offset, 0.0, panel_z),
                rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
            ),
            material=stone,
            name=f"{label}_arch_trim",
        )

    # Stone column shafts, plinths, and capitals sit just outside each arched opening.
    column_specs = []
    for sign in (-1.0, 1.0):
        column_specs.extend(
            [
                (sign * 0.70, 1.36, f"front_col_{sign:+.0f}"),
                (sign * 0.70, -1.36, f"rear_col_{sign:+.0f}"),
                (1.36, sign * 0.70, f"side0_col_{sign:+.0f}"),
                (-1.36, sign * 0.70, f"side1_col_{sign:+.0f}"),
            ]
        )
    for x, y, name in column_specs:
        tower.visual(
            Cylinder(radius=0.075, length=1.22),
            origin=Origin(xyz=(x, y, 9.20)),
            material=stone,
            name=f"{name}_shaft",
        )
        tower.visual(
            Box((0.22, 0.16, 0.10)),
            origin=Origin(xyz=(x, y, 8.54)),
            material=stone,
            name=f"{name}_base",
        )
        tower.visual(
            Box((0.23, 0.18, 0.11)),
            origin=Origin(xyz=(x, y, 9.84)),
            material=stone,
            name=f"{name}_capital",
        )

    tower.visual(
        Box((2.78, 2.78, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 11.22)),
        material=brick,
        name="upper_cornice",
    )
    tower.visual(
        Box((2.46, 2.46, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 11.47)),
        material=dark_brick,
        name="belfry_ceiling",
    )

    # The wooden tie beam at the ceiling carries the bell's pivot saddles.
    tower.visual(
        Box((2.68, 0.22, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 10.90)),
        material=wood,
        name="tie_beam",
    )
    for x, label in ((-0.82, "0"), (0.82, "1")):
        tower.visual(
            Box((0.20, 0.24, 0.08)),
            origin=Origin(xyz=(x, 0.0, 10.61)),
            material=iron,
            name=f"bearing_saddle_{label}",
        )
        tower.visual(
            Box((0.08, 0.18, 0.26)),
            origin=Origin(xyz=(x, 0.0, 10.78)),
            material=iron,
            name=f"bearing_hanger_{label}",
        )

    tower.visual(
        _save_mesh("pyramid_tile_roof", _pyramid_roof_mesh(2.42, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 11.60)),
        material=roof_tile,
        name="tile_roof",
    )


def _add_bell_yoke_visuals(bell_yoke, *, wood, iron, bell_metal):
    # Child frame is the horizontal pivot axis at the center of the yoke.
    bell_yoke.visual(
        Box((1.20, 0.24, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wood,
        name="wooden_yoke",
    )
    bell_yoke.visual(
        Cylinder(radius=0.050, length=1.74),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="pivot_axle",
    )
    for x, label in ((-0.23, "0"), (0.23, "1")):
        bell_yoke.visual(
            Box((0.055, 0.035, 0.34)),
            origin=Origin(xyz=(x, 0.0, -0.28)),
            material=iron,
            name=f"hanger_strap_{label}",
        )
    bell_yoke.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.34)),
        material=bell_metal,
        name="bell_crown",
    )
    bell_yoke.visual(
        _save_mesh("cast_iron_bell_shell", _bell_shell_mesh()),
        origin=Origin(),
        material=bell_metal,
        name="bell_shell",
    )
    bell_yoke.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        material=iron,
        name="clapper_rod",
    )
    bell_yoke.visual(
        Cylinder(radius=0.085, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -1.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="clapper_ball",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_campanile_bell")

    brick = model.material("warm_brick", rgba=(0.55, 0.20, 0.12, 1.0))
    dark_brick = model.material("dark_mortar_brick", rgba=(0.30, 0.12, 0.08, 1.0))
    stone = model.material("weathered_stone", rgba=(0.72, 0.69, 0.61, 1.0))
    roof_tile = model.material("aged_roof_tile", rgba=(0.42, 0.08, 0.05, 1.0))
    wood = model.material("dark_oak", rgba=(0.38, 0.22, 0.10, 1.0))
    iron = model.material("blackened_iron", rgba=(0.06, 0.06, 0.055, 1.0))
    bell_metal = model.material("cast_iron_bell", rgba=(0.12, 0.11, 0.10, 1.0))

    tower = model.part("tower")
    _add_campanile_static(
        tower,
        brick=brick,
        dark_brick=dark_brick,
        stone=stone,
        roof_tile=roof_tile,
        wood=wood,
        iron=iron,
    )

    bell_yoke = model.part("bell_yoke")
    _add_bell_yoke_visuals(bell_yoke, wood=wood, iron=iron, bell_metal=bell_metal)

    model.articulation(
        "bell_pivot",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_yoke,
        origin=Origin(xyz=(0.0, 0.0, 10.70)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-0.42,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell_yoke = object_model.get_part("bell_yoke")
    pivot = object_model.get_articulation("bell_pivot")

    for hanger_name in ("bearing_hanger_0", "bearing_hanger_1"):
        ctx.allow_overlap(
            bell_yoke,
            tower,
            elem_a="pivot_axle",
            elem_b=hanger_name,
            reason="The horizontal axle is intentionally captured through simplified solid iron bearing hangers.",
        )
        ctx.expect_contact(
            bell_yoke,
            tower,
            elem_a="pivot_axle",
            elem_b=hanger_name,
            contact_tol=0.004,
            name=f"axle is captured by {hanger_name}",
        )

    ctx.check(
        "bell uses horizontal revolute pivot",
        pivot.articulation_type == ArticulationType.REVOLUTE and tuple(pivot.axis) == (1.0, 0.0, 0.0),
        details=f"type={pivot.articulation_type}, axis={pivot.axis}",
    )
    ctx.expect_contact(
        bell_yoke,
        tower,
        elem_a="pivot_axle",
        elem_b="bearing_saddle_0",
        contact_tol=0.004,
        name="axle rests in first ceiling saddle",
    )
    ctx.expect_contact(
        bell_yoke,
        tower,
        elem_a="pivot_axle",
        elem_b="bearing_saddle_1",
        contact_tol=0.004,
        name="axle rests in second ceiling saddle",
    )
    ctx.expect_gap(
        tower,
        bell_yoke,
        axis="z",
        positive_elem="tie_beam",
        negative_elem="wooden_yoke",
        min_gap=-0.01,
        max_gap=0.02,
        name="yoke is seated immediately below the tie beam",
    )
    ctx.expect_within(
        bell_yoke,
        tower,
        axes="xy",
        inner_elem="bell_shell",
        outer_elem="belfry_floor",
        margin=0.01,
        name="bell hangs within square belfry footprint",
    )

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    rest_shell = ctx.part_element_world_aabb(bell_yoke, elem="bell_shell")
    with ctx.pose({pivot: 0.30}):
        swung_shell = ctx.part_element_world_aabb(bell_yoke, elem="bell_shell")

    rest_y = _aabb_center_y(rest_shell)
    swung_y = _aabb_center_y(swung_shell)
    ctx.check(
        "bell shell moves when yoke pivots",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.12,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
