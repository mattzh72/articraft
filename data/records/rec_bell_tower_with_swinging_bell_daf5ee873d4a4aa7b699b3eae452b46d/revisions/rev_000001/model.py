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
    CylinderGeometry,
    Inertial,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _square_loop(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _add_quad(
    geom: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
    *,
    flip: bool = False,
) -> None:
    if flip:
        geom.add_face(a, c, b)
        geom.add_face(a, d, c)
    else:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)


def _revolve_open_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 56,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []

    for radius, z_pos in outer_profile:
        ring: list[int] = []
        for idx in range(segments):
            angle = math.tau * idx / segments
            ring.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z_pos))
        outer_rings.append(ring)

    for radius, z_pos in inner_profile:
        ring = []
        for idx in range(segments):
            angle = math.tau * idx / segments
            ring.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z_pos))
        inner_rings.append(ring)

    for ring_a, ring_b in zip(outer_rings[:-1], outer_rings[1:]):
        for idx in range(segments):
            nxt = (idx + 1) % segments
            _add_quad(geom, ring_a[idx], ring_a[nxt], ring_b[nxt], ring_b[idx], flip=False)

    for ring_a, ring_b in zip(inner_rings[:-1], inner_rings[1:]):
        for idx in range(segments):
            nxt = (idx + 1) % segments
            _add_quad(geom, ring_a[idx], ring_b[idx], ring_b[nxt], ring_a[nxt], flip=False)

    top_outer = outer_rings[0]
    top_inner = inner_rings[0]
    for idx in range(segments):
        nxt = (idx + 1) % segments
        _add_quad(geom, top_outer[idx], top_inner[idx], top_inner[nxt], top_outer[nxt], flip=False)

    return geom


def _build_bell_shell_mesh() -> MeshGeometry:
    outer_profile = [
        (0.070, -0.340),
        (0.105, -0.390),
        (0.175, -0.480),
        (0.255, -0.610),
        (0.330, -0.740),
        (0.360, -0.820),
    ]
    inner_profile = [
        (0.030, -0.340),
        (0.072, -0.398),
        (0.136, -0.488),
        (0.208, -0.616),
        (0.272, -0.736),
        (0.316, -0.784),
    ]
    return _revolve_open_shell(outer_profile, inner_profile, segments=64)


def _build_roof_mesh() -> MeshGeometry:
    return section_loft(
        [
            _square_loop(2.06, 0.0),
            _square_loop(0.12, 0.92),
        ]
    )


def _build_pulley_mesh() -> MeshGeometry:
    rim = TorusGeometry(
        radius=0.162,
        tube=0.028,
        radial_segments=18,
        tubular_segments=40,
    ).rotate_y(math.pi * 0.5)
    hub = CylinderGeometry(radius=0.050, height=0.115, radial_segments=32).rotate_y(math.pi * 0.5)
    collar = CylinderGeometry(radius=0.030, height=0.090, radial_segments=28).rotate_y(math.pi * 0.5)

    spoke_template = BoxGeometry((0.016, 0.300, 0.024))
    spokes = MeshGeometry()
    for angle in (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0):
        spokes.merge(spoke_template.copy().rotate_x(angle))

    return _merge_geometries([rim, hub, collar, spokes])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bell_tower")

    masonry = model.material("masonry", rgba=(0.70, 0.69, 0.66, 1.0))
    darker_stone = model.material("darker_stone", rgba=(0.60, 0.59, 0.56, 1.0))
    slate = model.material("slate", rgba=(0.23, 0.24, 0.28, 1.0))
    timber = model.material("timber", rgba=(0.42, 0.28, 0.17, 1.0))
    bronze = model.material("bronze", rgba=(0.67, 0.48, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((2.10, 2.10, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=darker_stone,
        name="plinth",
    )
    tower.visual(
        Box((1.82, 1.82, 3.75)),
        origin=Origin(xyz=(0.0, 0.0, 2.325)),
        material=masonry,
        name="tower_shaft",
    )
    tower.visual(
        Box((1.98, 1.98, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 4.29)),
        material=darker_stone,
        name="belfry_floor",
    )
    for x_pos in (-0.77, 0.77):
        for y_pos in (-0.77, 0.77):
            tower.visual(
                Box((0.30, 0.30, 1.12)),
                origin=Origin(xyz=(x_pos, y_pos, 4.94)),
                material=masonry,
                name=f"belfry_pier_{'r' if x_pos > 0 else 'l'}_{'f' if y_pos > 0 else 'b'}",
            )
    tower.visual(
        Box((1.98, 1.98, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 5.50)),
        material=darker_stone,
        name="belfry_top_ring",
    )
    tower.visual(
        Box((0.22, 0.28, 0.18)),
        origin=Origin(xyz=(-0.63, 0.0, 5.28)),
        material=dark_steel,
        name="bell_bearing_left",
    )
    tower.visual(
        Box((0.22, 0.28, 0.18)),
        origin=Origin(xyz=(0.63, 0.0, 5.28)),
        material=dark_steel,
        name="bell_bearing_right",
    )
    tower.visual(
        Box((0.12, 0.12, 0.94)),
        origin=Origin(xyz=(-0.63, 0.0, 4.85)),
        material=dark_steel,
        name="bell_frame_left",
    )
    tower.visual(
        Box((0.12, 0.12, 0.94)),
        origin=Origin(xyz=(0.63, 0.0, 4.85)),
        material=dark_steel,
        name="bell_frame_right",
    )
    tower.visual(
        mesh_from_geometry(_build_roof_mesh(), "tower_roof"),
        origin=Origin(xyz=(0.0, 0.0, 5.54)),
        material=slate,
        name="roof_shell",
    )
    tower.visual(
        Box((0.035, 0.26, 0.58)),
        origin=Origin(xyz=(-0.075, 1.03, 0.90)),
        material=dark_steel,
        name="pulley_bracket_left",
    )
    tower.visual(
        Box((0.035, 0.26, 0.58)),
        origin=Origin(xyz=(0.075, 1.03, 0.90)),
        material=dark_steel,
        name="pulley_bracket_right",
    )
    tower.visual(
        Box((0.20, 0.18, 0.07)),
        origin=Origin(xyz=(0.0, 1.03, 1.155)),
        material=dark_steel,
        name="pulley_bracket_cap",
    )
    tower.inertial = Inertial.from_geometry(
        Box((2.10, 2.10, 6.46)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 3.23)),
    )

    bell = model.part("bell")
    bell.visual(
        mesh_from_geometry(_build_bell_shell_mesh(), "bell_shell"),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Box((1.04, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=timber,
        name="headstock_beam",
    )
    bell.visual(
        Box((0.10, 0.04, 0.16)),
        origin=Origin(xyz=(-0.21, 0.0, -0.16)),
        material=dark_steel,
        name="headstock_strap_left",
    )
    bell.visual(
        Box((0.10, 0.04, 0.16)),
        origin=Origin(xyz=(0.21, 0.0, -0.16)),
        material=dark_steel,
        name="headstock_strap_right",
    )
    bell.visual(
        Box((0.34, 0.09, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.29)),
        material=timber,
        name="crown_block",
    )
    bell.visual(
        Cylinder(radius=0.012, length=0.39),
        origin=Origin(xyz=(0.0, 0.0, -0.535)),
        material=dark_steel,
        name="clapper_rod",
    )
    bell.visual(
        Cylinder(radius=0.048, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.74)),
        material=dark_steel,
        name="clapper_bob",
    )
    bell.inertial = Inertial.from_geometry(
        Box((1.00, 0.78, 0.95)),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
    )

    pulley = model.part("pulley")
    pulley.visual(
        mesh_from_geometry(_build_pulley_mesh(), "pulley_wheel"),
        material=dark_steel,
        name="pulley_rim",
    )
    pulley.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.09),
        mass=18.0,
        origin=Origin(),
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 5.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.6,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "tower_to_pulley",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=pulley,
        origin=Origin(xyz=(0.0, 1.16, 0.90)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=14.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    pulley = object_model.get_part("pulley")
    bell_joint = object_model.get_articulation("tower_to_bell")
    pulley_joint = object_model.get_articulation("tower_to_pulley")

    bell_limits = bell_joint.motion_limits
    pulley_limits = pulley_joint.motion_limits

    ctx.check(
        "bell pivot is a bounded revolute headstock joint",
        bell_joint.articulation_type == ArticulationType.REVOLUTE
        and bell_joint.axis == (1.0, 0.0, 0.0)
        and bell_limits is not None
        and bell_limits.lower is not None
        and bell_limits.upper is not None
        and bell_limits.lower < 0.0 < bell_limits.upper,
        details=str(
            {
                "type": bell_joint.articulation_type,
                "axis": bell_joint.axis,
                "limits": None
                if bell_limits is None
                else (bell_limits.lower, bell_limits.upper),
            }
        ),
    )
    ctx.check(
        "base pulley uses an independent continuous axle joint",
        pulley_joint.articulation_type == ArticulationType.CONTINUOUS
        and pulley_joint.axis == (1.0, 0.0, 0.0)
        and pulley_limits is not None
        and pulley_limits.lower is None
        and pulley_limits.upper is None
        and bell_joint.child != pulley_joint.child,
        details=str(
            {
                "type": pulley_joint.articulation_type,
                "axis": pulley_joint.axis,
                "limits": None
                if pulley_limits is None
                else (pulley_limits.lower, pulley_limits.upper),
                "bell_child": bell_joint.child,
                "pulley_child": pulley_joint.child,
            }
        ),
    )

    bell_shell = bell.get_visual("bell_shell")
    belfry_floor = tower.get_visual("belfry_floor")
    pulley_rim = pulley.get_visual("pulley_rim")
    tower_shaft = tower.get_visual("tower_shaft")

    ctx.expect_gap(
        bell,
        tower,
        axis="z",
        positive_elem=bell_shell,
        negative_elem=belfry_floor,
        min_gap=0.03,
        name="bell clears the belfry floor at rest",
    )
    if bell_limits is not None and bell_limits.upper is not None:
        with ctx.pose({bell_joint: bell_limits.upper}):
            ctx.expect_gap(
                bell,
                tower,
                axis="z",
                positive_elem=bell_shell,
                negative_elem=belfry_floor,
                min_gap=0.01,
                name="bell still clears the belfry floor at full swing",
            )

    ctx.expect_gap(
        pulley,
        tower,
        axis="y",
        positive_elem=pulley_rim,
        negative_elem=tower_shaft,
        min_gap=0.02,
        max_gap=0.08,
        name="pulley wheel stands just proud of the tower shaft",
    )
    with ctx.pose({pulley_joint: 1.3}):
        ctx.expect_gap(
            pulley,
            tower,
            axis="y",
            positive_elem=pulley_rim,
            negative_elem=tower_shaft,
            min_gap=0.02,
            max_gap=0.08,
            name="pulley wheel stays mounted proud when rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
