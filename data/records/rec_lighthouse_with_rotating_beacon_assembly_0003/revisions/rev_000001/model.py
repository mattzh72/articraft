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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_lighthouse", assets=ASSETS)

    pier_concrete = model.material("pier_concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    tower_white = model.material("tower_white", rgba=(0.91, 0.92, 0.88, 1.0))
    roof_red = model.material("roof_red", rgba=(0.70, 0.15, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.21, 0.23, 0.25, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.78, 0.88, 0.94, 0.26))
    lens_amber = model.material("lens_amber", rgba=(0.82, 0.64, 0.20, 0.95))

    ring_profile = _regular_polygon_profile(0.114, sides=8, angle_offset=math.pi / 8.0)
    ring_hole = _regular_polygon_profile(0.092, sides=8, angle_offset=math.pi / 8.0)
    lantern_ring_mesh = _save_mesh(
        "lantern_octagon_ring.obj",
        ExtrudeWithHolesGeometry(
            ring_profile,
            [ring_hole],
            height=0.02,
            center=False,
        ),
    )
    roof_mesh = _save_mesh(
        "lantern_roof.obj",
        section_loft(
            [
                _regular_polygon_loop(0.118, sides=8, z=0.0, angle_offset=math.pi / 8.0),
                _regular_polygon_loop(0.082, sides=8, z=0.04, angle_offset=math.pi / 8.0),
                _regular_polygon_loop(0.026, sides=8, z=0.078, angle_offset=math.pi / 8.0),
            ]
        ),
    )
    tower_shell_mesh = _save_mesh(
        "tower_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.205, 0.0),
                (0.205, 0.035),
                (0.192, 0.11),
                (0.174, 0.29),
                (0.152, 0.50),
                (0.144, 0.58),
                (0.0, 0.58),
            ],
            segments=72,
        ),
    )

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((0.48, 0.48, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=pier_concrete,
        name="foundation_plinth",
    )
    tower_base.visual(
        tower_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=tower_white,
        name="tower_shell",
    )
    tower_base.visual(
        Cylinder(radius=0.155, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.624)),
        material=dark_metal,
        name="gallery_skirt",
    )
    tower_base.visual(
        Cylinder(radius=0.17, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.643)),
        material=dark_metal,
        name="gallery_deck",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 0.656)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.328)),
    )

    lantern_room = model.part("lantern_room")
    lantern_room.visual(
        lantern_ring_mesh,
        material=dark_metal,
        name="sill_ring",
    )
    for index in range(8):
        vertex_angle = math.pi / 8.0 + index * (math.pi / 4.0)
        lantern_room.visual(
            Box((0.02, 0.014, 0.15)),
            origin=Origin(
                xyz=(0.106 * math.cos(vertex_angle), 0.106 * math.sin(vertex_angle), 0.095),
                rpy=(0.0, 0.0, vertex_angle),
            ),
            material=dark_metal,
            name=f"frame_post_{index:02d}",
        )
        face_angle = index * (math.pi / 4.0)
        lantern_room.visual(
            Box((0.006, 0.068, 0.15)),
            origin=Origin(
                xyz=(0.097 * math.cos(face_angle), 0.097 * math.sin(face_angle), 0.095),
                rpy=(0.0, 0.0, face_angle),
            ),
            material=lantern_glass,
            name=f"glazing_panel_{index:02d}",
        )
    lantern_room.visual(
        lantern_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_metal,
        name="top_ring",
    )
    lantern_room.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.19)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    roof_cap = model.part("roof_cap")
    roof_cap.visual(roof_mesh, material=roof_red, name="roof_shell")
    roof_cap.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=dark_metal,
        name="vent_stack",
    )
    roof_cap.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=roof_red,
        name="vent_cap",
    )
    roof_cap.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.12)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="pedestal_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.03, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=tower_white,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.04, 0.032, 0.03)),
        origin=Origin(xyz=(-0.032, 0.0, 0.031)),
        material=dark_metal,
        name="drive_box",
    )
    pedestal.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=dark_metal,
        name="bearing_cap",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.10)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    light_head = model.part("light_head")
    light_head.visual(
        Cylinder(radius=0.033, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_metal,
        name="rotor_table",
    )
    light_head.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_metal,
        name="spindle",
    )
    light_head.visual(
        Box((0.018, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark_metal,
        name="support_pylon",
    )
    light_head.visual(
        Box((0.09, 0.02, 0.01)),
        origin=Origin(xyz=(0.004, 0.0, 0.046)),
        material=dark_metal,
        name="main_arm",
    )
    light_head.visual(
        Box((0.042, 0.06, 0.042)),
        origin=Origin(xyz=(0.05, 0.0, 0.053)),
        material=lens_amber,
        name="lens_housing",
    )
    light_head.visual(
        Box((0.006, 0.054, 0.032)),
        origin=Origin(xyz=(0.074, 0.0, 0.053)),
        material=lantern_glass,
        name="front_lens",
    )
    light_head.visual(
        Box((0.04, 0.016, 0.01)),
        origin=Origin(xyz=(-0.043, 0.0, 0.046)),
        material=dark_metal,
        name="rear_boom",
    )
    light_head.visual(
        Box((0.02, 0.028, 0.024)),
        origin=Origin(xyz=(-0.071, 0.0, 0.051)),
        material=roof_red,
        name="counterweight",
    )
    light_head.visual(
        Box((0.02, 0.03, 0.016)),
        origin=Origin(xyz=(0.05, 0.0, 0.077)),
        material=dark_metal,
        name="top_hood",
    )
    light_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.07, 0.08)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    model.articulation(
        "tower_to_lantern",
        ArticulationType.FIXED,
        parent=tower_base,
        child=lantern_room,
        origin=Origin(xyz=(0.0, 0.0, 0.656)),
    )
    model.articulation(
        "lantern_to_roof",
        ArticulationType.FIXED,
        parent=lantern_room,
        child=roof_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )
    model.articulation(
        "tower_to_pedestal",
        ArticulationType.FIXED,
        parent=tower_base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.656)),
    )
    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower_base = object_model.get_part("tower_base")
    lantern_room = object_model.get_part("lantern_room")
    roof_cap = object_model.get_part("roof_cap")
    pedestal = object_model.get_part("pedestal")
    light_head = object_model.get_part("light_head")

    beacon_spin = object_model.get_articulation("beacon_spin")

    gallery_deck = tower_base.get_visual("gallery_deck")
    sill_ring = lantern_room.get_visual("sill_ring")
    top_ring = lantern_room.get_visual("top_ring")
    roof_shell = roof_cap.get_visual("roof_shell")
    vent_stack = roof_cap.get_visual("vent_stack")
    pedestal_plinth = pedestal.get_visual("pedestal_plinth")
    bearing_cap = pedestal.get_visual("bearing_cap")
    rotor_table = light_head.get_visual("rotor_table")
    lens_housing = light_head.get_visual("lens_housing")
    counterweight = light_head.get_visual("counterweight")
    top_hood = light_head.get_visual("top_hood")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_fixed=True)

    ctx.expect_overlap(lantern_room, tower_base, axes="xy", min_overlap=0.20)
    ctx.expect_origin_distance(lantern_room, tower_base, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        lantern_room,
        tower_base,
        axis="z",
        positive_elem=sill_ring,
        negative_elem=gallery_deck,
        max_gap=0.001,
        max_penetration=0.0,
    )

    ctx.expect_origin_distance(roof_cap, lantern_room, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        roof_cap,
        lantern_room,
        axis="z",
        positive_elem=roof_shell,
        negative_elem=top_ring,
        max_gap=0.001,
        max_penetration=0.0,
    )

    ctx.expect_origin_distance(pedestal, lantern_room, axes="xy", max_dist=0.001)
    ctx.expect_within(pedestal, lantern_room, axes="xy", margin=0.005)
    ctx.expect_gap(
        pedestal,
        tower_base,
        axis="z",
        positive_elem=pedestal_plinth,
        negative_elem=gallery_deck,
        max_gap=0.001,
        max_penetration=0.0,
    )

    ctx.expect_origin_distance(light_head, pedestal, axes="xy", max_dist=0.001)
    ctx.expect_within(light_head, lantern_room, axes="xy", margin=0.005)
    ctx.expect_gap(
        light_head,
        pedestal,
        axis="z",
        positive_elem=rotor_table,
        negative_elem=bearing_cap,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        roof_cap,
        light_head,
        axis="z",
        positive_elem=vent_stack,
        negative_elem=top_hood,
        min_gap=0.07,
        max_gap=0.09,
    )

    lens_rest_aabb = ctx.part_element_world_aabb(light_head, elem=lens_housing)
    assert lens_rest_aabb is not None
    lens_rest_center = _aabb_center(lens_rest_aabb)
    assert lens_rest_center[0] > 0.04
    assert abs(lens_rest_center[1]) < 0.01

    with ctx.pose({beacon_spin: math.pi / 2.0}):
        ctx.expect_within(light_head, lantern_room, axes="xy", margin=0.005)
        ctx.expect_gap(
            roof_cap,
            light_head,
            axis="z",
            positive_elem=vent_stack,
            negative_elem=top_hood,
            min_gap=0.07,
            max_gap=0.09,
        )
        lens_quarter_aabb = ctx.part_element_world_aabb(light_head, elem=lens_housing)
        counter_quarter_aabb = ctx.part_element_world_aabb(light_head, elem=counterweight)
        assert lens_quarter_aabb is not None
        assert counter_quarter_aabb is not None
        lens_quarter_center = _aabb_center(lens_quarter_aabb)
        counter_quarter_center = _aabb_center(counter_quarter_aabb)
        assert abs(lens_quarter_center[0]) < 0.012
        assert lens_quarter_center[1] > 0.04
        assert counter_quarter_center[1] < -0.055

    with ctx.pose({beacon_spin: math.pi}):
        ctx.expect_within(light_head, lantern_room, axes="xy", margin=0.005)
        lens_half_aabb = ctx.part_element_world_aabb(light_head, elem=lens_housing)
        assert lens_half_aabb is not None
        lens_half_center = _aabb_center(lens_half_aabb)
        assert lens_half_center[0] < -0.04
        assert abs(lens_half_center[1]) < 0.01

    return ctx.report()


def _regular_polygon_profile(
    radius: float, *, sides: int, angle_offset: float = 0.0
) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + (2.0 * math.pi * index / sides)),
            radius * math.sin(angle_offset + (2.0 * math.pi * index / sides)),
        )
        for index in range(sides)
    ]


def _regular_polygon_loop(
    radius: float, *, sides: int, z: float, angle_offset: float = 0.0
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in _regular_polygon_profile(radius, sides=sides, angle_offset=angle_offset)]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


# >>> USER_CODE_END

object_model = build_object_model()
