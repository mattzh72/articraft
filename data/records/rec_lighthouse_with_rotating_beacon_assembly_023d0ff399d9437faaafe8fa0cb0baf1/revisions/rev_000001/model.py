from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _facet_origin(theta: float, apothem: float, z_center: float, *, depth_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=((apothem + depth_offset) * math.cos(theta), (apothem + depth_offset) * math.sin(theta), z_center),
        rpy=(0.0, 0.0, theta - math.pi / 2.0),
    )


def _octagon_vertices(apothem: float) -> list[tuple[float, float]]:
    radius = apothem / math.cos(math.pi / 8.0)
    start = math.pi / 2.0 + math.pi / 8.0
    return [
        (radius * math.cos(start - index * math.pi / 4.0), radius * math.sin(start - index * math.pi / 4.0))
        for index in range(8)
    ]


def _ring_shell(outer_radius: float, inner_radius: float, z0: float, z1: float):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _roof_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (1.48, 0.00),
            (1.42, 0.12),
            (1.18, 0.30),
            (0.88, 0.50),
            (0.58, 0.72),
            (0.28, 0.90),
            (0.20, 0.98),
        ],
        [
            (1.34, 0.00),
            (1.29, 0.10),
            (1.08, 0.26),
            (0.82, 0.44),
            (0.55, 0.62),
            (0.28, 0.78),
            (0.18, 0.90),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _lens_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.30, 0.10),
            (0.35, 0.14),
            (0.39, 0.26),
            (0.43, 0.46),
            (0.46, 0.70),
            (0.45, 0.94),
            (0.43, 1.18),
            (0.39, 1.38),
            (0.34, 1.54),
            (0.30, 1.60),
        ],
        [
            (0.24, 0.10),
            (0.27, 0.24),
            (0.31, 0.52),
            (0.32, 0.86),
            (0.31, 1.20),
            (0.28, 1.44),
            (0.24, 1.60),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seaside_lighthouse_lantern")

    painted_iron = model.material("painted_iron", rgba=(0.20, 0.22, 0.24, 1.0))
    weathered_brass = model.material("weathered_brass", rgba=(0.58, 0.48, 0.24, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.78, 0.90, 0.96, 0.34))
    lens_glass = model.material("lens_glass", rgba=(0.70, 0.90, 0.94, 0.44))
    lighthouse_white = model.material("lighthouse_white", rgba=(0.90, 0.91, 0.89, 1.0))
    machinery_dark = model.material("machinery_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    deck_grey = model.material("deck_grey", rgba=(0.48, 0.50, 0.52, 1.0))
    bronze_trim = model.material("bronze_trim", rgba=(0.50, 0.36, 0.16, 1.0))

    apothem = 1.45
    facet_span = 2.0 * apothem * math.tan(math.pi / 8.0)
    wall_height = 2.24
    door_width = 0.62
    door_height = 1.92
    front_plane_y = apothem
    door_plane_y = front_plane_y + 0.015
    hinge_x = -0.36

    structure = model.part("lantern_structure")
    structure.inertial = Inertial.from_geometry(
        Box((3.9, 3.9, 3.7)),
        mass=920.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )
    structure.visual(
        Cylinder(radius=1.82, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, -0.29)),
        material=lighthouse_white,
        name="base_drum",
    )
    structure.visual(
        Cylinder(radius=1.62, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=bronze_trim,
        name="floor_cap",
    )

    for vertex_x, vertex_y in _octagon_vertices(apothem):
        structure.visual(
            Box((0.11, 0.11, wall_height)),
            origin=Origin(xyz=(vertex_x, vertex_y, wall_height * 0.5)),
            material=painted_iron,
        )

    facet_normals = [math.pi / 2.0 - index * math.pi / 4.0 for index in range(8)]
    fixed_glass_facets = [1, 2, 3, 4, 5, 6, 7]
    for index in fixed_glass_facets:
        theta = facet_normals[index]
        structure.visual(
            Box((facet_span + 0.05, 0.12, 0.16)),
            origin=_facet_origin(theta, apothem, 0.28),
            material=painted_iron,
        )
        structure.visual(
            Box((facet_span + 0.05, 0.12, 0.14)),
            origin=_facet_origin(theta, apothem, 2.10),
            material=painted_iron,
        )
        structure.visual(
            Box((facet_span - 0.18, 0.024, 1.74)),
            origin=_facet_origin(theta, apothem, 1.20, depth_offset=-0.01),
            material=glass_clear,
        )

    structure.visual(
        Box((facet_span + 0.10, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, front_plane_y, 2.08)),
        material=painted_iron,
        name="door_transom",
    )
    structure.visual(
        Box((0.08, 0.12, 2.04)),
        origin=Origin(xyz=(-0.40, front_plane_y, 1.02)),
        material=painted_iron,
        name="left_door_jamb",
    )
    structure.visual(
        Box((0.08, 0.12, 2.04)),
        origin=Origin(xyz=(0.40, front_plane_y, 1.02)),
        material=painted_iron,
        name="right_door_jamb",
    )
    structure.visual(
        Box((facet_span + 0.06, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, front_plane_y, 0.025)),
        material=bronze_trim,
        name="door_threshold",
    )
    structure.visual(
        Box((0.18, 0.024, 1.74)),
        origin=Origin(xyz=(-0.62, front_plane_y - 0.01, 1.20)),
        material=glass_clear,
        name="left_sidelight",
    )
    structure.visual(
        Box((0.18, 0.024, 1.74)),
        origin=Origin(xyz=(0.62, front_plane_y - 0.01, 1.20)),
        material=glass_clear,
        name="right_sidelight",
    )

    for theta in facet_normals:
        structure.visual(
            Box((facet_span + 0.10, 0.22, 0.20)),
            origin=_facet_origin(theta, apothem + 0.02, 2.28),
            material=bronze_trim,
        )

    structure.visual(
        _save_mesh("roof_shell", _roof_shell()),
        origin=Origin(xyz=(0.0, 0.0, 2.24)),
        material=painted_iron,
        name="roof_shell",
    )
    structure.visual(
        Cylinder(radius=0.20, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 3.22)),
        material=painted_iron,
        name="vent_cylinder",
    )
    structure.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 3.41)),
        material=bronze_trim,
        name="vent_cap",
    )

    shaft = model.part("central_shaft")
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=2.80),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
    )
    shaft.visual(
        Cylinder(radius=0.22, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=weathered_brass,
        name="shaft_pedestal",
    )
    shaft.visual(
        Cylinder(radius=0.09, length=2.54),
        origin=Origin(xyz=(0.0, 0.0, 1.51)),
        material=machinery_dark,
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.14, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=weathered_brass,
        name="lower_bearing",
    )
    shaft.visual(
        Cylinder(radius=0.12, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.82)),
        material=weathered_brass,
        name="top_bearing_cap",
    )

    carriage = model.part("lens_carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((1.55, 1.55, 1.90)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
    )
    carriage.visual(
        _save_mesh("carriage_sleeve", _ring_shell(0.16, 0.105, 0.00, 1.66)),
        material=weathered_brass,
        name="carriage_sleeve",
    )
    carriage.visual(
        _save_mesh("turntable_ring", _ring_shell(0.68, 0.18, 0.00, 0.12)),
        material=weathered_brass,
        name="turntable_ring",
    )
    carriage.visual(
        _save_mesh("lens_crown_ring", _ring_shell(0.70, 0.20, 1.46, 1.64)),
        material=weathered_brass,
        name="lens_crown_ring",
    )
    carriage.visual(
        _save_mesh("fresnel_lens", _lens_shell()),
        material=lens_glass,
        name="fresnel_lens",
    )
    for index in range(8):
        theta = index * math.pi / 4.0
        carriage.visual(
            Box((0.52, 0.05, 0.06)),
            origin=Origin(
                xyz=(0.38 * math.cos(theta), 0.38 * math.sin(theta), 0.06),
                rpy=(0.0, 0.0, theta),
            ),
            material=weathered_brass,
        )
        carriage.visual(
            Box((0.09, 0.06, 1.48)),
            origin=Origin(
                xyz=(0.36 * math.cos(theta), 0.36 * math.sin(theta), 0.84),
                rpy=(0.0, 0.0, theta),
            ),
            material=weathered_brass,
        )
    carriage.visual(
        Box((0.22, 0.12, 0.18)),
        origin=Origin(xyz=(0.52, 0.0, 0.18)),
        material=machinery_dark,
        name="drive_cabinet",
    )
    carriage.visual(
        Box((0.14, 0.10, 0.16)),
        origin=Origin(xyz=(-0.48, 0.0, 0.18)),
        material=machinery_dark,
        name="counterweight_box",
    )
    door = model.part("lantern_door")
    door_raise = 0.06
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.05, door_height)),
        mass=48.0,
        origin=Origin(xyz=(door_width * 0.5, 0.0, door_height * 0.5 + door_raise)),
    )
    door.visual(
        Box((0.06, 0.045, door_height)),
        origin=Origin(xyz=(0.03, 0.0, door_height * 0.5 + door_raise)),
        material=painted_iron,
        name="hinge_stile",
    )
    door.visual(
        Box((0.06, 0.045, door_height)),
        origin=Origin(xyz=(door_width - 0.03, 0.0, door_height * 0.5 + door_raise)),
        material=painted_iron,
        name="latch_stile",
    )
    door.visual(
        Box((door_width, 0.045, 0.08)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, 0.04 + door_raise)),
        material=painted_iron,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, 0.045, 0.10)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, door_height - 0.05 + door_raise)),
        material=painted_iron,
        name="top_rail",
    )
    door.visual(
        Box((door_width - 0.10, 0.045, 0.46)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, 0.31 + door_raise)),
        material=painted_iron,
        name="kick_panel",
    )
    door.visual(
        Box((door_width - 0.10, 0.022, 1.14)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, 1.28 + door_raise)),
        material=glass_clear,
        name="door_leaf",
    )
    hinge_barrel_offset = 0.063
    for hinge_name, hinge_z in (
        ("lower_hinge", 0.38 + door_raise),
        ("middle_hinge", 1.02 + door_raise),
        ("upper_hinge", 1.66 + door_raise),
    ):
        door.visual(
            Box((0.05, 0.024, 0.16)),
            origin=Origin(xyz=(0.03, 0.0345, hinge_z)),
            material=painted_iron,
            name=f"{hinge_name}_leaf",
        )
        door.visual(
            Cylinder(radius=0.018, length=0.055),
            origin=Origin(xyz=(0.0, hinge_barrel_offset, hinge_z)),
            material=painted_iron,
            name=f"{hinge_name}_knuckle",
        )
    door.visual(
        Cylinder(radius=0.018, length=0.05),
        origin=Origin(xyz=(door_width - 0.08, 0.032, 0.98 + door_raise), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=weathered_brass,
        name="door_handle",
    )

    model.articulation(
        "structure_to_shaft",
        ArticulationType.FIXED,
        parent=structure,
        child=shaft,
        origin=Origin(),
    )
    model.articulation(
        "shaft_to_lens_carriage",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )
    model.articulation(
        "structure_to_door",
        ArticulationType.REVOLUTE,
        parent=structure,
        child=door,
        origin=Origin(xyz=(hinge_x, door_plane_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    structure = object_model.get_part("lantern_structure")
    shaft = object_model.get_part("central_shaft")
    carriage = object_model.get_part("lens_carriage")
    door = object_model.get_part("lantern_door")

    carriage_joint = object_model.get_articulation("shaft_to_lens_carriage")
    door_joint = object_model.get_articulation("structure_to_door")

    ctx.expect_contact(
        shaft,
        structure,
        name="central shaft pedestal seats on lantern deck",
    )
    ctx.expect_origin_distance(
        carriage,
        shaft,
        axes="xy",
        max_dist=0.001,
        name="lens carriage stays centered on the shaft axis",
    )
    ctx.expect_within(
        carriage,
        structure,
        axes="xy",
        margin=0.05,
        name="lens carriage remains inside lantern footprint",
    )

    ctx.check(
        "lens carriage uses continuous vertical rotation",
        carriage_joint.articulation_type == ArticulationType.CONTINUOUS
        and carriage_joint.axis == (0.0, 0.0, 1.0)
        and carriage_joint.motion_limits is not None
        and carriage_joint.motion_limits.lower is None
        and carriage_joint.motion_limits.upper is None,
        details=(
            f"type={carriage_joint.articulation_type}, axis={carriage_joint.axis}, "
            f"limits={carriage_joint.motion_limits}"
        ),
    )
    ctx.check(
        "door hinge is a vertical outward-opening revolute joint",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and door_joint.axis == (0.0, 0.0, 1.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper > 1.2,
        details=(
            f"type={door_joint.articulation_type}, axis={door_joint.axis}, "
            f"limits={door_joint.motion_limits}"
        ),
    )

    drive_rest = _aabb_center(ctx.part_element_world_aabb(carriage, elem="drive_cabinet"))
    door_rest = _aabb_center(ctx.part_element_world_aabb(door, elem="door_leaf"))

    with ctx.pose({carriage_joint: math.pi / 2.0, door_joint: math.radians(70.0)}):
        drive_quarter_turn = _aabb_center(ctx.part_element_world_aabb(carriage, elem="drive_cabinet"))
        door_open = _aabb_center(ctx.part_element_world_aabb(door, elem="door_leaf"))

    ctx.check(
        "drive cabinet swings around the central shaft",
        drive_rest is not None
        and drive_quarter_turn is not None
        and drive_rest[0] > 0.35
        and abs(drive_rest[1]) < 0.12
        and drive_quarter_turn[1] > 0.35
        and abs(drive_quarter_turn[0]) < 0.12,
        details=f"rest={drive_rest}, quarter_turn={drive_quarter_turn}",
    )
    ctx.check(
        "door leaf opens outward from the front facet",
        door_rest is not None
        and door_open is not None
        and door_open[1] > door_rest[1] + 0.18
        and door_open[0] < door_rest[0] - 0.12,
        details=f"closed={door_rest}, open={door_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
