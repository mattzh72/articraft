from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _octagon_loop(apothem: float, z: float = 0.0) -> list[tuple[float, float, float]]:
    """Regular octagon with face normals on the cardinal/diagonal axes."""
    radius = apothem / math.cos(math.pi / 8.0)
    return [
        (
            radius * math.cos(math.pi / 8.0 + i * math.pi / 4.0),
            radius * math.sin(math.pi / 8.0 + i * math.pi / 4.0),
            z,
        )
        for i in range(8)
    ]


def _octagon_profile(apothem: float) -> list[tuple[float, float]]:
    return [(x, y) for x, y, _ in _octagon_loop(apothem)]


def _octagonal_frame(outer_apothem: float, inner_apothem: float, height: float) -> object:
    """CadQuery octagonal ring with a real open center, extruded from z=0."""
    outer = cq.Workplane("XY").polyline(_octagon_profile(outer_apothem)).close().extrude(height)
    cutter = (
        cq.Workplane("XY")
        .polyline(_octagon_profile(inner_apothem))
        .close()
        .extrude(height + 0.006)
        .translate((0.0, 0.0, -0.003))
    )
    return outer.cut(cutter)


def _panel_origin(face_index: int, apothem: float, z: float) -> Origin:
    theta = face_index * math.pi / 4.0
    # Vent/box local X is horizontal across the panel, local Y becomes vertical,
    # and local Z is the panel thickness/outward normal.
    return Origin(
        xyz=(apothem * math.cos(theta), apothem * math.sin(theta), z),
        rpy=(math.pi / 2.0, 0.0, theta + math.pi / 2.0),
    )


def _box_on_face(face_index: int, apothem: float, z: float) -> Origin:
    theta = face_index * math.pi / 4.0
    return Origin(
        xyz=(apothem * math.cos(theta), apothem * math.sin(theta), z),
        rpy=(0.0, 0.0, theta + math.pi / 2.0),
    )


def _build_yoke_mesh() -> object:
    """A single cast yoke with open saddle ears around the bell shaft."""
    ear_x = 0.215
    ear_thickness = 0.055
    ear_depth = 0.140
    lower_height = 0.139
    upper_height = 0.059
    foot_height = 0.065
    ear_bottom_z = 0.320

    def box(size: tuple[float, float, float], center: tuple[float, float, float]):
        return cq.Workplane("XY").box(*size).translate(center)

    yoke = (
        box((ear_thickness, ear_depth, lower_height), (-ear_x, -0.040, ear_bottom_z + lower_height / 2.0))
        .union(box((ear_thickness, ear_depth, lower_height), (ear_x, -0.040, ear_bottom_z + lower_height / 2.0)))
        .union(box((ear_thickness, ear_depth, upper_height), (-ear_x, -0.040, 0.501 + upper_height / 2.0)))
        .union(box((ear_thickness, ear_depth, upper_height), (ear_x, -0.040, 0.501 + upper_height / 2.0)))
        .union(box((ear_thickness, 0.030, 0.240), (-ear_x, -0.120, ear_bottom_z + 0.120)))
        .union(box((ear_thickness, 0.030, 0.240), (ear_x, -0.120, ear_bottom_z + 0.120)))
        .union(box((0.130, ear_depth + 0.030, foot_height), (-ear_x, -0.040, 0.260 + foot_height / 2.0)))
        .union(box((0.130, ear_depth + 0.030, foot_height), (ear_x, -0.040, 0.260 + foot_height / 2.0)))
        .union(box((0.485, 0.035, 0.075), (0.0, -0.125, 0.322 + 0.075 / 2.0)))
    )
    return yoke


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="octagonal_rooftop_cupola_bell")

    slate = model.material("weathered_slate", rgba=(0.18, 0.20, 0.21, 1.0))
    painted_wood = model.material("painted_wood", rgba=(0.78, 0.72, 0.60, 1.0))
    dark_louver = model.material("shadowed_louvers", rgba=(0.28, 0.25, 0.19, 1.0))
    glass = model.material("pale_glass", rgba=(0.62, 0.82, 0.95, 0.36))
    bronze = model.material("aged_bronze", rgba=(0.70, 0.46, 0.18, 1.0))
    iron = model.material("blackened_cast_iron", rgba=(0.05, 0.05, 0.045, 1.0))
    roof_mat = model.material("dark_copper_roof", rgba=(0.16, 0.31, 0.28, 1.0))

    root = model.part("roof_plate")
    root.visual(
        Box((1.20, 1.20, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=slate,
        name="flat_roof_plate",
    )

    # Octagonal cupola curb, rails, and interior support ring.
    ring_pairs = (
        ("lower_ring", 0.370, 0.270, 0.060, 0.050, painted_wood),
        ("upper_ring", 0.355, 0.265, 0.050, 0.560, painted_wood),
        ("interior_ring", 0.310, 0.170, 0.030, 0.250, iron),
    )
    for name, outer_a, inner_a, height, z0, mat in ring_pairs:
        root.visual(
            mesh_from_cadquery(_octagonal_frame(outer_a, inner_a, height), name, tolerance=0.0008, angular_tolerance=0.08),
            origin=Origin(xyz=(0.0, 0.0, z0)),
            material=mat,
            name=name,
        )

    # Eight vertical corner posts tie the rails and roof into one cupola frame.
    post_radius = 0.018
    post_height = 0.500
    vertex_radius = 0.342 / math.cos(math.pi / 8.0)
    for i in range(8):
        a = math.pi / 8.0 + i * math.pi / 4.0
        root.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(vertex_radius * math.cos(a), vertex_radius * math.sin(a), 0.325)),
            material=painted_wood,
            name=f"corner_post_{i}",
        )

    for i, (x, y) in enumerate(((0.300, 0.0), (-0.300, 0.0), (0.0, 0.300), (0.0, -0.300))):
        root.visual(
            Cylinder(radius=0.012, length=0.235),
            origin=Origin(xyz=(x, y, 0.165)),
            material=iron,
            name=f"inner_post_{i}",
        )

    louver = VentGrilleGeometry(
        (0.230, 0.455),
        frame=0.016,
        face_thickness=0.006,
        duct_depth=0.020,
        duct_wall=0.004,
        slat_pitch=0.043,
        slat_width=0.018,
        slat_angle_deg=38.0,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=1, divider_width=0.005),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
        sleeve=VentGrilleSleeve(style="short", depth=0.018, wall=0.004),
    )
    louver_mesh = mesh_from_geometry(louver, "louver_panel")
    for i in range(8):
        if i % 2 == 0:
            root.visual(
                louver_mesh,
                origin=_panel_origin(i, 0.323, 0.335),
                material=dark_louver,
                name=f"louver_panel_{i}",
            )
        else:
            root.visual(
                Box((0.230, 0.012, 0.455)),
                origin=_box_on_face(i, 0.324, 0.335),
                material=glass,
                name=f"glass_panel_{i}",
            )

    # Shallow octagonal roof with a centered finial.
    roof_geom = LoftGeometry(
        [
            _octagon_loop(0.415, 0.0),
            _octagon_loop(0.105, 0.205),
        ],
        cap=True,
        closed=True,
    )
    root.visual(
        mesh_from_geometry(roof_geom, "octagonal_roof"),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=roof_mat,
        name="octagonal_roof",
    )
    root.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.858)),
        material=roof_mat,
        name="finial_stem",
    )
    root.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        material=roof_mat,
        name="finial_ball",
    )
    root.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.806)),
        material=roof_mat,
        name="finial_base",
    )

    root.visual(
        mesh_from_cadquery(_build_yoke_mesh(), "cast_yoke", tolerance=0.0008, angular_tolerance=0.08),
        material=iron,
        name="cast_yoke",
    )

    bell = model.part("bell")
    bell_shell = LatheGeometry.from_shell_profiles(
        [
            (0.150, -0.310),
            (0.145, -0.285),
            (0.126, -0.235),
            (0.096, -0.165),
            (0.066, -0.095),
            (0.052, -0.045),
        ],
        [
            (0.122, -0.292),
            (0.118, -0.255),
            (0.095, -0.190),
            (0.062, -0.112),
            (0.030, -0.063),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    bell.visual(mesh_from_geometry(bell_shell, "bell_shell"), material=bronze, name="bell_shell")
    bell.visual(
        Cylinder(radius=0.011, length=0.377),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="pivot_shaft",
    )
    bell.visual(
        Cylinder(radius=0.028, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="pivot_hub",
    )
    bell.visual(
        Box((0.075, 0.052, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=bronze,
        name="crown_lug",
    )

    model.articulation(
        "yoke_to_bell",
        ArticulationType.REVOLUTE,
        parent=root,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.36, upper=0.36),
    )

    clapper = model.part("clapper")
    clapper.visual(
        Sphere(radius=0.012),
        material=iron,
        name="hanger_eye",
    )
    clapper.visual(
        Cylinder(radius=0.0055, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=iron,
        name="clapper_rod",
    )
    clapper.visual(
        Sphere(radius=0.029),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=iron,
        name="clapper_ball",
    )

    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("roof_plate")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    bell_joint = object_model.get_articulation("yoke_to_bell")
    clapper_joint = object_model.get_articulation("bell_to_clapper")

    ctx.expect_within(
        bell,
        root,
        axes="xy",
        inner_elem="bell_shell",
        outer_elem="upper_ring",
        margin=0.005,
        name="bell hangs inside octagonal cupola footprint",
    )
    ctx.expect_overlap(
        bell,
        root,
        axes="x",
        elem_a="pivot_shaft",
        elem_b="cast_yoke",
        min_overlap=0.34,
        name="horizontal pivot shaft spans the yoke ears",
    )
    ctx.expect_gap(
        bell,
        root,
        axis="z",
        positive_elem="pivot_shaft",
        negative_elem="interior_ring",
        min_gap=0.17,
        max_gap=0.205,
        name="shaft is seated above the interior frame ring",
    )
    ctx.expect_within(
        clapper,
        bell,
        axes="xy",
        inner_elem="clapper_ball",
        outer_elem="bell_shell",
        margin=0.0,
        name="clapper ball hangs inside bell mouth footprint",
    )

    rest_bell_aabb = ctx.part_world_aabb(bell)
    with ctx.pose({bell_joint: 0.32}):
        swung_bell_aabb = ctx.part_world_aabb(bell)
    if rest_bell_aabb is not None and swung_bell_aabb is not None:
        rest_center_y = (rest_bell_aabb[0][1] + rest_bell_aabb[1][1]) / 2.0
        swung_center_y = (swung_bell_aabb[0][1] + swung_bell_aabb[1][1]) / 2.0
        ctx.check(
            "bell swings on the horizontal shaft",
            abs(swung_center_y - rest_center_y) > 0.040,
            details=f"rest_y={rest_center_y:.3f}, swung_y={swung_center_y:.3f}",
        )

    rest_clapper_aabb = ctx.part_world_aabb(clapper)
    with ctx.pose({clapper_joint: 0.45}):
        swung_clapper_aabb = ctx.part_world_aabb(clapper)
    if rest_clapper_aabb is not None and swung_clapper_aabb is not None:
        rest_center_y = (rest_clapper_aabb[0][1] + rest_clapper_aabb[1][1]) / 2.0
        swung_center_y = (swung_clapper_aabb[0][1] + swung_clapper_aabb[1][1]) / 2.0
        ctx.check(
            "clapper rod swings inside the bell",
            abs(swung_center_y - rest_center_y) > 0.055,
            details=f"rest_y={rest_center_y:.3f}, swung_y={swung_center_y:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
