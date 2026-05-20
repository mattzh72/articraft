from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _box_geometry(
    size: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    yaw: float = 0.0,
) -> MeshGeometry:
    geom = BoxGeometry(size)
    if abs(yaw) > 0.0:
        geom.rotate_z(yaw)
    geom.translate(*xyz)
    return geom


def _octagon_ring_mesh(
    *,
    apothem: float,
    face_width: float,
    thickness: float,
    height: float,
    z_center: float,
) -> MeshGeometry:
    segments: list[MeshGeometry] = []
    for index in range(8):
        theta = index * math.tau / 8.0
        segments.append(
            _box_geometry(
                (face_width, thickness, height),
                xyz=(apothem * math.cos(theta), apothem * math.sin(theta), z_center),
                yaw=theta + math.pi / 2.0,
            )
        )
    return _merge_geometries(segments)


def _post_cluster_mesh(
    *,
    vertex_radius: float,
    post_size: float,
    height: float,
    z_center: float,
) -> MeshGeometry:
    posts: list[MeshGeometry] = []
    for index in range(8):
        theta = (index + 0.5) * math.tau / 8.0
        posts.append(
            _box_geometry(
                (post_size, post_size, height),
                xyz=(vertex_radius * math.cos(theta), vertex_radius * math.sin(theta), z_center),
            )
        )
    return _merge_geometries(posts)


def _inner_frame_yoke_mesh() -> MeshGeometry:
    inner_apothem = 0.058
    ring = _octagon_ring_mesh(
        apothem=inner_apothem,
        face_width=0.048,
        thickness=0.008,
        height=0.010,
        z_center=0.085,
    )

    braces = [
        _box_geometry((0.026, 0.010, 0.010), xyz=(0.071, 0.0, 0.085)),
        _box_geometry((0.026, 0.010, 0.010), xyz=(-0.071, 0.0, 0.085)),
        _box_geometry((0.026, 0.010, 0.010), xyz=(0.0, 0.071, 0.085), yaw=math.pi / 2.0),
        _box_geometry((0.026, 0.010, 0.010), xyz=(0.0, -0.071, 0.085), yaw=math.pi / 2.0),
        _box_geometry((0.022, 0.050, 0.010), xyz=(0.0, 0.0, 0.071)),
        _box_geometry((0.012, 0.022, 0.026), xyz=(0.032, 0.0, 0.075)),
        _box_geometry((0.012, 0.022, 0.026), xyz=(-0.032, 0.0, 0.075)),
    ]
    return _merge_geometries([ring, *braces])


def _roof_shell_mesh() -> MeshGeometry:
    roof = ConeGeometry(radius=0.112, height=0.056, radial_segments=8)
    roof.rotate_z(math.pi / 8.0)
    roof.translate(0.0, 0.0, 0.128)
    return roof


def _louver_panel_mesh() -> MeshGeometry:
    panel = _grille_panel_geometry(
        panel_size=(0.056, 0.078),
        thickness=0.006,
        frame=0.006,
        slat_pitch=0.012,
        slat_width=0.006,
        slat_angle_deg=30.0,
        corner_radius=0.003,
        center=True,
    )
    panel.rotate_x(math.pi / 2.0)
    return panel


def _bell_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.072),
            (0.033, -0.067),
            (0.030, -0.056),
            (0.026, -0.042),
            (0.021, -0.024),
            (0.015, -0.010),
            (0.010, -0.004),
        ],
        [
            (0.027, -0.068),
            (0.026, -0.062),
            (0.024, -0.051),
            (0.020, -0.037),
            (0.015, -0.020),
            (0.010, -0.009),
            (0.0, -0.003),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_cupola_bell", assets=ASSETS)

    roof_gray = model.material("roof_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    painted_white = model.material("painted_white", rgba=(0.88, 0.88, 0.84, 1.0))
    louver_green = model.material("louver_green", rgba=(0.42, 0.47, 0.39, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.19, 0.20, 0.21, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.46, 0.22, 1.0))

    roof_plate = model.part("roof_plate")
    roof_plate.visual(
        Box((0.45, 0.45, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=roof_gray,
        name="roof_plate",
    )
    roof_plate.visual(
        Box((0.11, 0.11, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=roof_gray,
        name="cupola_curb",
    )
    roof_plate.inertial = Inertial.from_geometry(
        Box((0.45, 0.45, 0.028)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    cupola = model.part("cupola")
    cupola.visual(
        _save_mesh(
            "cupola_sill_ring.obj",
            _octagon_ring_mesh(
                apothem=0.082,
                face_width=0.068,
                thickness=0.012,
                height=0.014,
                z_center=0.007,
            ),
        ),
        material=painted_white,
        name="sill_ring",
    )
    cupola.visual(
        Box((0.154, 0.154, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_iron,
        name="base_deck",
    )
    cupola.visual(
        _save_mesh(
            "cupola_posts.obj",
            _post_cluster_mesh(
                vertex_radius=0.0888,
                post_size=0.012,
                height=0.088,
                z_center=0.051,
            ),
        ),
        material=painted_white,
        name="corner_posts",
    )
    cupola.visual(
        _save_mesh(
            "cupola_upper_ring.obj",
            _octagon_ring_mesh(
                apothem=0.082,
                face_width=0.072,
                thickness=0.012,
                height=0.016,
                z_center=0.092,
            ),
        ),
        material=painted_white,
        name="upper_ring",
    )
    cupola.visual(
        _save_mesh("cupola_inner_frame_yoke.obj", _inner_frame_yoke_mesh()),
        material=dark_iron,
        name="inner_frame_yoke",
    )

    panel_mesh = _save_mesh("cupola_louver_panel.obj", _louver_panel_mesh())
    for index in (0, 2, 4, 6):
        theta = index * math.tau / 8.0
        cupola.visual(
            panel_mesh,
            origin=Origin(
                xyz=(0.076 * math.cos(theta), 0.076 * math.sin(theta), 0.049),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=louver_green,
            name=f"louver_{index}",
        )

    cupola.visual(
        Box((0.010, 0.020, 0.028)),
        origin=Origin(xyz=(-0.049, 0.0, 0.085)),
        material=dark_iron,
        name="left_ear",
    )
    cupola.visual(
        Box((0.010, 0.020, 0.028)),
        origin=Origin(xyz=(0.049, 0.0, 0.085)),
        material=dark_iron,
        name="right_ear",
    )
    cupola.visual(
        Cylinder(radius=0.004, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="pivot_shaft",
    )
    cupola.visual(
        _save_mesh("cupola_roof_shell.obj", _roof_shell_mesh()),
        material=roof_gray,
        name="roof_shell",
    )
    cupola.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=dark_iron,
        name="finial_spire",
    )
    cupola.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=dark_iron,
        name="finial_ball",
    )
    cupola.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.22)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )

    bell = model.part("bell")
    bell.visual(
        _save_mesh("bell_shell.obj", _bell_shell_mesh()),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Box((0.022, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=dark_iron,
        name="crown_block",
    )
    bell.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0005)),
        material=dark_iron,
        name="left_trunnion_boss",
    )
    bell.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.0005)),
        material=dark_iron,
        name="right_trunnion_boss",
    )
    bell.visual(
        Cylinder(radius=0.0065, length=0.026),
        origin=Origin(xyz=(-0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="left_trunnion",
    )
    bell.visual(
        Cylinder(radius=0.0065, length=0.026),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="right_trunnion",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.082),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
    )

    clapper = model.part("clapper")
    clapper.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_iron,
        name="hanger_head",
    )
    clapper.visual(
        Cylinder(radius=0.0018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=dark_iron,
        name="clapper_rod",
    )
    clapper.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=dark_iron,
        name="clapper_ball",
    )
    clapper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.008, length=0.048),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    model.articulation(
        "roof_to_cupola",
        ArticulationType.FIXED,
        parent=roof_plate,
        child=cupola,
        origin=Origin(),
    )
    model.articulation(
        "bell_pivot",
        ArticulationType.REVOLUTE,
        parent=cupola,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.40,
        ),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.FIXED,
        parent=bell,
        child=clapper,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    roof_plate = object_model.get_part("roof_plate")
    cupola = object_model.get_part("cupola")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    bell_pivot = object_model.get_articulation("bell_pivot")

    roof_surface = roof_plate.get_visual("roof_plate")
    sill_ring = cupola.get_visual("sill_ring")
    frame_yoke = cupola.get_visual("inner_frame_yoke")
    left_ear = cupola.get_visual("left_ear")
    right_ear = cupola.get_visual("right_ear")
    finial_ball = cupola.get_visual("finial_ball")
    louvers = [cupola.get_visual(f"louver_{index}") for index in (0, 2, 4, 6)]

    bell_shell = bell.get_visual("bell_shell")
    crown_block = bell.get_visual("crown_block")
    left_trunnion = bell.get_visual("left_trunnion")
    right_trunnion = bell.get_visual("right_trunnion")

    hanger_head = clapper.get_visual("hanger_head")
    clapper_ball = clapper.get_visual("clapper_ball")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        bell,
        cupola,
        reason="bell trunnion sleeves wrap the stationary pivot shaft between the bracket ears",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(cupola, roof_plate, axes="xy", max_dist=0.0015)
    ctx.expect_within(cupola, roof_plate, axes="xy", inner_elem=sill_ring, outer_elem=roof_surface)
    ctx.expect_gap(
        cupola,
        roof_plate,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=sill_ring,
        negative_elem=roof_surface,
    )
    for index, louver in zip((0, 2, 4, 6), louvers):
        ctx.expect_gap(
            cupola,
            roof_plate,
            axis="z",
            min_gap=0.009,
            positive_elem=louver,
            negative_elem=roof_surface,
            name=f"louver_{index}_raised_above_roof_plate",
        )
    ctx.expect_gap(
        cupola,
        bell,
        axis="x",
        min_gap=0.035,
        positive_elem=louvers[0],
        negative_elem=bell_shell,
        name="east_louver_sits_on_its_own_face",
    )
    ctx.expect_gap(
        bell,
        cupola,
        axis="x",
        min_gap=0.035,
        positive_elem=bell_shell,
        negative_elem=louvers[2],
        name="west_louver_sits_on_its_own_face",
    )
    ctx.expect_gap(
        cupola,
        bell,
        axis="y",
        min_gap=0.035,
        positive_elem=louvers[1],
        negative_elem=bell_shell,
        name="north_louver_sits_on_its_own_face",
    )
    ctx.expect_gap(
        bell,
        cupola,
        axis="y",
        min_gap=0.035,
        positive_elem=bell_shell,
        negative_elem=louvers[3],
        name="south_louver_sits_on_its_own_face",
    )
    ctx.expect_gap(
        cupola,
        roof_plate,
        axis="z",
        min_gap=0.175,
        positive_elem=finial_ball,
        negative_elem=roof_surface,
        name="finial_crowns_the_roof",
    )

    ctx.expect_within(bell, cupola, axes="xy", inner_elem=bell_shell, outer_elem=frame_yoke)
    ctx.expect_gap(
        bell,
        roof_plate,
        axis="z",
        min_gap=0.012,
        positive_elem=bell_shell,
        negative_elem=roof_surface,
        name="bell_clears_the_roof_plate",
    )
    ctx.expect_gap(
        bell,
        cupola,
        axis="x",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=left_trunnion,
        negative_elem=left_ear,
        name="left_trunnion_seated_at_left_bracket_ear",
    )
    ctx.expect_gap(
        cupola,
        bell,
        axis="x",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=right_ear,
        negative_elem=right_trunnion,
        name="right_trunnion_seated_at_right_bracket_ear",
    )

    ctx.expect_contact(clapper, bell, elem_a=hanger_head, elem_b=crown_block)
    ctx.expect_within(clapper, bell, axes="xy", inner_elem=clapper_ball, outer_elem=bell_shell)
    ctx.expect_gap(
        clapper,
        roof_plate,
        axis="z",
        min_gap=0.016,
        positive_elem=clapper_ball,
        negative_elem=roof_surface,
        name="clapper_hangs_inside_the_bell",
    )

    with ctx.pose({bell_pivot: 0.32}):
        ctx.expect_within(bell, cupola, axes="xy", inner_elem=bell_shell, outer_elem=frame_yoke)
        ctx.expect_gap(
            bell,
            roof_plate,
            axis="z",
            min_gap=0.005,
            positive_elem=bell_shell,
            negative_elem=roof_surface,
            name="swung_bell_still_clears_the_roof_plate",
        )
        ctx.expect_gap(
            clapper,
            roof_plate,
            axis="z",
            min_gap=0.012,
            positive_elem=clapper_ball,
            negative_elem=roof_surface,
            name="swung_clapper_stays_above_the_roof_plate",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
