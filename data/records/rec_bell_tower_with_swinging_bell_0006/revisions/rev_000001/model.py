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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _pyramid_roof_mesh(size_x: float, size_y: float, height: float) -> MeshGeometry:
    half_x = size_x * 0.5
    half_y = size_y * 0.5
    geometry = MeshGeometry()
    base_sw = geometry.add_vertex(-half_x, -half_y, 0.0)
    base_se = geometry.add_vertex(half_x, -half_y, 0.0)
    base_ne = geometry.add_vertex(half_x, half_y, 0.0)
    base_nw = geometry.add_vertex(-half_x, half_y, 0.0)
    apex = geometry.add_vertex(0.0, 0.0, height)

    _add_quad(geometry, base_sw, base_nw, base_ne, base_se)
    geometry.add_face(base_sw, base_se, apex)
    geometry.add_face(base_se, base_ne, apex)
    geometry.add_face(base_ne, base_nw, apex)
    geometry.add_face(base_nw, base_sw, apex)
    return geometry


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_campanile", assets=ASSETS)

    brick = model.material("brick", rgba=(0.58, 0.24, 0.19, 1.0))
    weathered_brick = model.material("weathered_brick", rgba=(0.66, 0.31, 0.24, 1.0))
    stone = model.material("stone", rgba=(0.80, 0.77, 0.71, 1.0))
    terracotta = model.material("terracotta", rgba=(0.63, 0.27, 0.14, 1.0))
    louver_paint = model.material("louver_paint", rgba=(0.47, 0.54, 0.45, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    bronze = model.material("bronze", rgba=(0.67, 0.49, 0.20, 1.0))

    front_louver_mesh = _save_mesh(
        "campanile_front_louver.obj",
        _grille_panel_geometry(
            panel_size=(0.200, 0.130),
            thickness=0.012,
            frame=0.012,
            slat_pitch=0.024,
            slat_width=0.010,
            slat_angle_deg=28.0,
            corner_radius=0.004,
        ).rotate_x(math.pi / 2.0),
    )
    side_louver_mesh = _save_mesh(
        "campanile_side_louver.obj",
        _grille_panel_geometry(
            panel_size=(0.200, 0.130),
            thickness=0.012,
            frame=0.012,
            slat_pitch=0.024,
            slat_width=0.010,
            slat_angle_deg=28.0,
            corner_radius=0.004,
        )
        .rotate_x(math.pi / 2.0)
        .rotate_z(math.pi / 2.0),
    )
    bell_mesh = _save_mesh(
        "campanile_bell_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.010, 0.000),
                (0.018, -0.008),
                (0.030, -0.024),
                (0.046, -0.055),
                (0.058, -0.090),
                (0.068, -0.118),
            ],
            [
                (0.000, -0.002),
                (0.007, -0.002),
                (0.013, -0.010),
                (0.024, -0.032),
                (0.040, -0.071),
                (0.057, -0.112),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    roof_mesh = _save_mesh("campanile_roof.obj", _pyramid_roof_mesh(0.320, 0.320, 0.105))

    tower = model.part("tower")
    tower.visual(
        Box((0.420, 0.420, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=stone,
        name="base_plinth",
    )
    tower.visual(
        Box((0.300, 0.300, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=brick,
        name="main_shaft",
    )
    tower.visual(
        Box((0.180, 0.050, 0.120)),
        origin=Origin(xyz=(0.0, 0.152, 0.300)),
        material=weathered_brick,
        name="north_recess",
    )
    tower.visual(
        Box((0.180, 0.050, 0.120)),
        origin=Origin(xyz=(0.0, -0.152, 0.300)),
        material=weathered_brick,
        name="south_recess",
    )
    tower.visual(
        Box((0.320, 0.320, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.7725)),
        material=stone,
        name="shaft_cornice",
    )

    belfry_bottom = 0.785
    belfry_height = 0.220
    belfry_center_z = belfry_bottom + belfry_height * 0.5
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.060, 0.060, belfry_height)),
                origin=Origin(
                    xyz=(
                        x_sign * 0.140,
                        y_sign * 0.140,
                        belfry_center_z,
                    )
                ),
                material=brick,
                name=f"corner_pier_{'e' if x_sign > 0 else 'w'}{'n' if y_sign > 0 else 's'}",
            )

    for y_sign, face_name in ((1.0, "north"), (-1.0, "south")):
        tower.visual(
            Box((0.220, 0.028, 0.055)),
            origin=Origin(xyz=(0.0, y_sign * 0.156, 0.8125)),
            material=brick,
            name=f"{face_name}_sill",
        )
        tower.visual(
            Box((0.220, 0.028, 0.055)),
            origin=Origin(xyz=(0.0, y_sign * 0.156, 0.9775)),
            material=brick,
            name=f"{face_name}_lintel",
        )

    for x_sign, face_name in ((1.0, "east"), (-1.0, "west")):
        tower.visual(
            Box((0.028, 0.220, 0.055)),
            origin=Origin(xyz=(x_sign * 0.156, 0.0, 0.8125)),
            material=brick,
            name=f"{face_name}_sill",
        )
        tower.visual(
            Box((0.028, 0.220, 0.055)),
            origin=Origin(xyz=(x_sign * 0.156, 0.0, 0.9775)),
            material=brick,
            name=f"{face_name}_lintel",
        )

    tower.visual(
        front_louver_mesh,
        origin=Origin(xyz=(0.0, 0.152, 0.895)),
        material=louver_paint,
        name="north_louver",
    )
    tower.visual(
        front_louver_mesh,
        origin=Origin(xyz=(0.0, -0.152, 0.895), rpy=(0.0, 0.0, math.pi)),
        material=louver_paint,
        name="south_louver",
    )
    tower.visual(
        side_louver_mesh,
        origin=Origin(xyz=(0.152, 0.0, 0.895)),
        material=louver_paint,
        name="east_louver",
    )
    tower.visual(
        side_louver_mesh,
        origin=Origin(xyz=(-0.152, 0.0, 0.895), rpy=(0.0, 0.0, math.pi)),
        material=louver_paint,
        name="west_louver",
    )
    tower.visual(
        Box((0.040, 0.060, 0.050)),
        origin=Origin(xyz=(-0.100, 0.0, 0.895)),
        material=cast_iron,
        name="left_bearing_block",
    )
    tower.visual(
        Box((0.040, 0.304, 0.018)),
        origin=Origin(xyz=(-0.100, 0.0, 0.895)),
        material=cast_iron,
        name="left_bearing_girt",
    )
    tower.visual(
        Box((0.040, 0.060, 0.050)),
        origin=Origin(xyz=(0.100, 0.0, 0.895)),
        material=cast_iron,
        name="right_bearing_block",
    )
    tower.visual(
        Box((0.040, 0.304, 0.018)),
        origin=Origin(xyz=(0.100, 0.0, 0.895)),
        material=cast_iron,
        name="right_bearing_girt",
    )
    tower.visual(
        Box((0.380, 0.380, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 1.019)),
        material=stone,
        name="roof_cap",
    )
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.033)),
        material=terracotta,
        name="pyramidal_roof",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.420, 0.420, 1.138)),
        mass=6800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.569)),
    )

    bell_assembly = model.part("bell_assembly")
    bell_assembly.visual(
        Box((0.160, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=cast_iron,
        name="yoke_beam",
    )
    bell_assembly.visual(
        Box((0.028, 0.010, 0.076)),
        origin=Origin(xyz=(-0.055, 0.0, -0.042)),
        material=cast_iron,
        name="left_yoke_cheek",
    )
    bell_assembly.visual(
        Box((0.028, 0.010, 0.076)),
        origin=Origin(xyz=(0.055, 0.0, -0.042)),
        material=cast_iron,
        name="right_yoke_cheek",
    )
    bell_assembly.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="left_trunnion",
    )
    bell_assembly.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="right_trunnion",
    )
    bell_assembly.visual(
        Box((0.030, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=cast_iron,
        name="bell_headstock",
    )
    bell_assembly.visual(
        bell_mesh,
        material=bronze,
        name="bell_shell",
    )
    bell_assembly.visual(
        Cylinder(radius=0.004, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=cast_iron,
        name="clapper_rod",
    )
    bell_assembly.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
        material=cast_iron,
        name="clapper_bob",
    )
    bell_assembly.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.160)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=1.6,
            lower=-0.75,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    bell_assembly = object_model.get_part("bell_assembly")
    bell_swing = object_model.get_articulation("bell_swing")
    main_shaft = tower.get_visual("main_shaft")
    roof_cap = tower.get_visual("roof_cap")
    north_louver = tower.get_visual("north_louver")
    east_louver = tower.get_visual("east_louver")
    left_bearing = tower.get_visual("left_bearing_block")
    right_bearing = tower.get_visual("right_bearing_block")
    bell_shell = bell_assembly.get_visual("bell_shell")
    left_trunnion = bell_assembly.get_visual("left_trunnion")
    right_trunnion = bell_assembly.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_origin_distance(bell_assembly, tower, axes="xy", max_dist=0.02)
    ctx.expect_contact(bell_assembly, tower, elem_a=left_trunnion, elem_b=left_bearing)
    ctx.expect_contact(bell_assembly, tower, elem_a=right_trunnion, elem_b=right_bearing)
    ctx.expect_gap(
        bell_assembly,
        tower,
        axis="z",
        min_gap=0.010,
        positive_elem=bell_shell,
        negative_elem=main_shaft,
        name="bell hangs above shaft crown",
    )
    ctx.expect_gap(
        tower,
        bell_assembly,
        axis="z",
        min_gap=0.020,
        positive_elem=roof_cap,
        negative_elem=bell_shell,
        name="roof cap clears bell",
    )
    ctx.expect_gap(
        tower,
        bell_assembly,
        axis="y",
        min_gap=0.025,
        positive_elem=north_louver,
        negative_elem=bell_shell,
        name="bell sits behind north louvers",
    )
    ctx.expect_gap(
        tower,
        bell_assembly,
        axis="x",
        min_gap=0.025,
        positive_elem=east_louver,
        negative_elem=bell_shell,
        name="bell sits behind east louvers",
    )
    ctx.expect_overlap(
        bell_assembly,
        tower,
        axes="xz",
        min_overlap=0.012,
        elem_a=bell_shell,
        elem_b=north_louver,
        name="bell aligns with north belfry opening",
    )
    ctx.expect_overlap(
        bell_assembly,
        tower,
        axes="yz",
        min_overlap=0.012,
        elem_a=bell_shell,
        elem_b=east_louver,
        name="bell aligns with east belfry opening",
    )
    with ctx.pose({bell_swing: 0.55}):
        ctx.expect_contact(bell_assembly, tower, elem_a=left_trunnion, elem_b=left_bearing)
        ctx.expect_contact(bell_assembly, tower, elem_a=right_trunnion, elem_b=right_bearing)
        ctx.expect_gap(
            tower,
            bell_assembly,
            axis="y",
            min_gap=0.010,
            positive_elem=north_louver,
            negative_elem=bell_shell,
            name="swung bell stays behind north louvers",
        )
        ctx.expect_gap(
            bell_assembly,
            tower,
            axis="z",
            min_gap=0.004,
            positive_elem=bell_shell,
            negative_elem=main_shaft,
            name="swung bell clears shaft floor",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
