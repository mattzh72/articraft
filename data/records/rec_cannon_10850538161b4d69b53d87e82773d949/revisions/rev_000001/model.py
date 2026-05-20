from __future__ import annotations

from math import cos, pi, sin

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
    wire_from_points,
)


TRUNNION_X = 0.0
TRUNNION_Z = 0.90
FRONT_AXLE_X = 0.78
REAR_AXLE_X = -0.78
WHEEL_Y = 0.63
AXLE_Z = 0.285


def _wedge_geometry(length: float, width: float, low: float, high: float) -> MeshGeometry:
    """A quoin wedge: bottom is flat at z=0, top slopes down toward +X."""
    half_l = length / 2.0
    half_w = width / 2.0
    vertices = [
        (-half_l, -half_w, 0.0),
        (half_l, -half_w, 0.0),
        (half_l, half_w, 0.0),
        (-half_l, half_w, 0.0),
        (-half_l, -half_w, high),
        (half_l, -half_w, low),
        (half_l, half_w, low),
        (-half_l, half_w, high),
    ]
    faces = [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
    return MeshGeometry(vertices, faces)


def _cheek_geometry() -> MeshGeometry:
    """Extruded side-cheek timber with a high rounded trunnion shoulder."""
    # Profile is authored in (x, z) and then remapped into (x, y, z).
    profile = [
        (-1.00, 0.34),
        (0.92, 0.34),
        (0.86, 0.54),
        (0.50, 0.70),
        (0.18, 0.88),
        (0.06, 1.02),
        (-0.12, 1.02),
        (-0.34, 0.86),
        (-0.74, 0.70),
        (-1.00, 0.52),
    ]
    return mesh_extrude_xz(profile, 0.11)


def mesh_extrude_xz(profile: list[tuple[float, float]], width: float) -> MeshGeometry:
    """Extrude an X/Z profile through Y as one connected mesh."""
    half = width / 2.0
    vertices: list[tuple[float, float, float]] = []
    for y in (-half, half):
        for x, z in profile:
            vertices.append((x, y, z))
    n = len(profile)
    faces: list[tuple[int, int, int]] = []
    # End caps.
    for i in range(1, n - 1):
        faces.append((0, i, i + 1))
        faces.append((n, n + i + 1, n + i))
    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))
        faces.append((i, n + j, n + i))
    return MeshGeometry(vertices, faces)


def _barrel_geometry() -> MeshGeometry:
    """Lathed iron cannon tube aligned to local +X after rotation."""
    profile = [
        (0.000, -1.02),
        (0.060, -1.02),
        (0.095, -0.97),
        (0.112, -0.91),
        (0.082, -0.84),
        (0.128, -0.80),
        (0.205, -0.77),
        (0.220, -0.68),
        (0.205, -0.56),
        (0.190, -0.46),
        (0.188, -0.12),
        (0.205, -0.08),
        (0.205, 0.04),
        (0.183, 0.08),
        (0.165, 0.58),
        (0.147, 1.10),
        (0.170, 1.20),
        (0.180, 1.44),
        (0.162, 1.57),
        (0.092, 1.60),
        (0.000, 1.60),
        (0.000, -1.02),
    ]
    return LatheGeometry(profile, segments=80).rotate_y(pi / 2.0)


def _add_wheel(part, prefix: str) -> None:
    """Iron-shod wooden truck wheel, centered on a Y-axis axle."""
    # Wooden felloe/rim and iron shoe are toroidal so the wheel reads as a real rim,
    # not a solid plain cylinder.
    part.visual(
        mesh_from_geometry(TorusGeometry(0.225, 0.034, radial_segments=18, tubular_segments=80).rotate_x(pi / 2.0), f"{prefix}_wood_rim"),
        material="weathered_oak",
        name="wood_rim",
    )
    part.visual(
        mesh_from_geometry(TorusGeometry(0.262, 0.016, radial_segments=14, tubular_segments=80).rotate_x(pi / 2.0), f"{prefix}_iron_tire"),
        material="dark_iron",
        name="iron_tire",
    )
    part.visual(
        Cylinder(radius=0.082, length=0.22),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_oak",
        name="hub",
    )
    for i in range(8):
        angle = 2.0 * pi * i / 8.0
        spoke = wire_from_points(
            [
                (cos(angle) * 0.070, 0.0, sin(angle) * 0.070),
                (cos(angle) * 0.218, 0.0, sin(angle) * 0.218),
            ],
            radius=0.018,
            radial_segments=12,
            cap_ends=True,
        )
        part.visual(
            mesh_from_geometry(spoke, f"{prefix}_spoke_{i}"),
            material="weathered_oak",
            name=f"spoke_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civil_war_garrison_siege_cannon")

    model.material("weathered_oak", rgba=(0.47, 0.29, 0.13, 1.0))
    model.material("dark_oak", rgba=(0.28, 0.16, 0.07, 1.0))
    model.material("dark_iron", rgba=(0.06, 0.065, 0.07, 1.0))
    model.material("gunmetal", rgba=(0.12, 0.13, 0.13, 1.0))
    model.material("worn_iron", rgba=(0.22, 0.22, 0.21, 1.0))
    model.material("bore_black", rgba=(0.005, 0.004, 0.003, 1.0))

    carriage = model.part("carriage")
    cheek_mesh = _cheek_geometry()
    carriage.visual(
        mesh_from_geometry(cheek_mesh.copy().translate(0.0, -0.455, 0.0), "cheek_0"),
        material="weathered_oak",
        name="cheek_0",
    )
    carriage.visual(
        mesh_from_geometry(cheek_mesh.copy().translate(0.0, 0.455, 0.0), "cheek_1"),
        material="weathered_oak",
        name="cheek_1",
    )
    carriage.visual(
        Box((1.55, 0.46, 0.10)),
        origin=Origin(xyz=(-0.14, 0.0, 0.47)),
        material="dark_oak",
        name="bed_plank",
    )
    carriage.visual(
        Box((0.18, 1.02, 0.16)),
        origin=Origin(xyz=(0.72, 0.0, 0.44)),
        material="weathered_oak",
        name="front_transom",
    )
    carriage.visual(
        Box((0.20, 1.02, 0.16)),
        origin=Origin(xyz=(-0.84, 0.0, 0.44)),
        material="weathered_oak",
        name="rear_transom",
    )
    carriage.visual(
        Box((0.16, 1.00, 0.12)),
        origin=Origin(xyz=(-0.12, 0.0, 0.42)),
        material="dark_oak",
        name="center_transom",
    )
    carriage.visual(
        Cylinder(radius=0.043, length=1.42),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_iron",
        name="front_axle",
    )
    carriage.visual(
        Cylinder(radius=0.043, length=1.42),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_iron",
        name="rear_axle",
    )
    for x, name in ((FRONT_AXLE_X, "front_axle_block"), (REAR_AXLE_X, "rear_axle_block")):
        carriage.visual(
            Box((0.20, 1.04, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.335)),
            material="dark_oak",
            name=name,
        )
    for y, name in ((-0.405, "trunnion_plate_0"), (0.405, "trunnion_plate_1")):
        carriage.visual(
            mesh_from_geometry(TorusGeometry(0.112, 0.017, radial_segments=14, tubular_segments=64).rotate_x(pi / 2.0), name),
            origin=Origin(xyz=(TRUNNION_X, y, TRUNNION_Z)),
            material="worn_iron",
            name=name,
        )
        carriage.visual(
            Box((0.22, 0.032, 0.055)),
            origin=Origin(xyz=(TRUNNION_X, y, TRUNNION_Z + 0.125)),
            material="dark_iron",
            name=f"{name}_cap",
        )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_barrel_geometry(), "barrel_body"),
        material="gunmetal",
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.080, length=0.84),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="gunmetal",
        name="trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.074, length=0.014),
        origin=Origin(xyz=(1.604, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="bore_black",
        name="bore_shadow",
    )
    barrel.visual(
        Sphere(radius=0.072),
        origin=Origin(xyz=(-1.035, 0.0, 0.0)),
        material="gunmetal",
        name="cascabel",
    )

    quoin = model.part("quoin")
    quoin.visual(
        mesh_from_geometry(_wedge_geometry(0.52, 0.34, 0.045, 0.145), "quoin_wedge"),
        material="dark_oak",
        name="wedge_body",
    )
    quoin.visual(
        Cylinder(radius=0.018, length=0.40),
        origin=Origin(xyz=(0.18, 0.0, 0.070), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_iron",
        name="cross_handle",
    )

    wheel_specs = [
        ("front_wheel_0", FRONT_AXLE_X, -WHEEL_Y, "front_wheel_0_spin"),
        ("front_wheel_1", FRONT_AXLE_X, WHEEL_Y, "front_wheel_1_spin"),
        ("rear_wheel_0", REAR_AXLE_X, -WHEEL_Y, "rear_wheel_0_spin"),
        ("rear_wheel_1", REAR_AXLE_X, WHEEL_Y, "rear_wheel_1_spin"),
    ]
    for part_name, _x, _y, _joint_name in wheel_specs:
        wheel = model.part(part_name)
        _add_wheel(wheel, part_name)

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.35, lower=-0.10, upper=0.45),
    )
    model.articulation(
        "quoin_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=quoin,
        origin=Origin(xyz=(-0.52, 0.0, 0.52)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.18, lower=-0.16, upper=0.20),
    )
    for part_name, x, y, joint_name in wheel_specs:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=part_name,
            origin=Origin(xyz=(x, y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    quoin = object_model.get_part("quoin")
    barrel_joint = object_model.get_articulation("barrel_elevation")
    quoin_joint = object_model.get_articulation("quoin_slide")

    for cheek_name in ("cheek_0", "cheek_1"):
        ctx.allow_overlap(
            barrel,
            carriage,
            elem_a="trunnion",
            elem_b=cheek_name,
            reason="The barrel trunnion is intentionally captured in a simplified cheek-board bearing pocket.",
        )
        ctx.expect_overlap(
            barrel,
            carriage,
            axes="y",
            elem_a="trunnion",
            elem_b=cheek_name,
            min_overlap=0.010,
            name=f"trunnion is seated in {cheek_name}",
        )

    for wheel_name, axle_elem in (
        ("front_wheel_0", "front_axle"),
        ("front_wheel_1", "front_axle"),
        ("rear_wheel_0", "rear_axle"),
        ("rear_wheel_1", "rear_axle"),
    ):
        ctx.allow_overlap(
            carriage,
            wheel_name,
            elem_a=axle_elem,
            elem_b="hub",
            reason="The fixed iron axle is intentionally captured inside the truck wheel hub bore proxy.",
        )
        ctx.expect_within(
            carriage,
            wheel_name,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="hub",
            margin=0.001,
            name=f"{wheel_name} hub surrounds its fixed axle",
        )
        ctx.expect_overlap(
            carriage,
            wheel_name,
            axes="y",
            elem_a=axle_elem,
            elem_b="hub",
            min_overlap=0.10,
            name=f"{wheel_name} hub remains on axle",
        )

    ctx.expect_gap(
        quoin,
        carriage,
        axis="z",
        positive_elem="wedge_body",
        negative_elem="bed_plank",
        max_gap=0.001,
        max_penetration=0.0,
        name="quoin wedge rides on the carriage bed",
    )
    ctx.expect_overlap(
        quoin,
        carriage,
        axes="xy",
        elem_a="wedge_body",
        elem_b="bed_plank",
        min_overlap=0.20,
        name="quoin stays over the bed plank",
    )

    ctx.check(
        "barrel uses revolute trunnion elevation",
        barrel_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(barrel_joint.articulation_type),
    )
    ctx.check(
        "quoin uses prismatic bed slide",
        quoin_joint.articulation_type == ArticulationType.PRISMATIC,
        details=str(quoin_joint.articulation_type),
    )
    for joint_name in (
        "front_wheel_0_spin",
        "front_wheel_1_spin",
        "rear_wheel_0_spin",
        "rear_wheel_1_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(joint.articulation_type),
        )

    rest_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
    with ctx.pose({barrel_joint: 0.45}):
        raised_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
    ctx.check(
        "positive barrel elevation raises the muzzle",
        rest_barrel_aabb is not None
        and raised_barrel_aabb is not None
        and raised_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.25,
        details=f"rest={rest_barrel_aabb}, raised={raised_barrel_aabb}",
    )

    rest_quoin_pos = ctx.part_world_position(quoin)
    with ctx.pose({quoin_joint: 0.20}):
        forward_quoin_pos = ctx.part_world_position(quoin)
    ctx.check(
        "positive quoin slide moves along the bed",
        rest_quoin_pos is not None
        and forward_quoin_pos is not None
        and forward_quoin_pos[0] > rest_quoin_pos[0] + 0.15,
        details=f"rest={rest_quoin_pos}, forward={forward_quoin_pos}",
    )

    return ctx.report()


object_model = build_object_model()
