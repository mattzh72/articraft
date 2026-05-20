from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _rectangular_tube_x(
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
) -> MeshGeometry:
    """A closed rectangular tube with a through-bore along local X."""

    geom = MeshGeometry()
    hx = length * 0.5
    oy = outer_y * 0.5
    oz = outer_z * 0.5
    iy = inner_y * 0.5
    iz = inner_z * 0.5

    outer_profile = [(-oy, -oz), (oy, -oz), (oy, oz), (-oy, oz)]
    inner_profile = [(-iy, -iz), (iy, -iz), (iy, iz), (-iy, iz)]

    end_ids: list[tuple[list[int], list[int]]] = []
    for x in (-hx, hx):
        outer = [geom.add_vertex(x, y, z) for y, z in outer_profile]
        inner = [geom.add_vertex(x, y, z) for y, z in inner_profile]
        end_ids.append((outer, inner))

    outer_a, inner_a = end_ids[0]
    outer_b, inner_b = end_ids[1]

    for i in range(4):
        j = (i + 1) % 4
        _add_quad(geom, outer_a[i], outer_a[j], outer_b[j], outer_b[i])
        _add_quad(geom, inner_a[j], inner_a[i], inner_b[i], inner_b[j])

    for outer, inner in end_ids:
        _add_quad(geom, outer[0], outer[1], inner[1], inner[0])
        _add_quad(geom, outer[1], outer[2], inner[2], inner[1])
        _add_quad(geom, outer[2], outer[3], inner[3], inner[2])
        _add_quad(geom, outer[3], outer[0], inner[0], inner[3])

    return geom


def _add_socket_bolts(part, *, material, z: float, prefix: str) -> None:
    for index, (x, y) in enumerate(
        (
            (-0.125, -0.070),
            (-0.125, 0.070),
            (0.125, -0.070),
            (0.125, 0.070),
            (-0.040, -0.070),
            (-0.040, 0.070),
            (0.040, -0.070),
            (0.040, 0.070),
        )
    ):
        part.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_jaw_carriage(
    jaw,
    *,
    sign: float,
    steel,
    dark,
    bronze,
    grip,
    prefix: str,
) -> None:
    # Sign is +1 for the carriage whose inward closing motion is +X, and -1
    # for its mirror.  The solid members stay outside the fixed rails while the
    # bearing pads visibly overlap the rail length with running clearance.
    jaw.visual(
        Box((0.220, 0.400, 0.030)),
        origin=Origin(xyz=(sign * 0.020, 0.0, 0.055)),
        material=steel,
        name="upper_carriage_cap",
    )
    jaw.visual(
        Box((0.220, 0.400, 0.030)),
        origin=Origin(xyz=(sign * 0.020, 0.0, -0.055)),
        material=steel,
        name="lower_carriage_cap",
    )
    jaw.visual(
        Box((0.160, 0.090, 0.120)),
        origin=Origin(xyz=(-sign * 0.065, 0.0, 0.0)),
        material=steel,
        name="center_web",
    )
    for y, upper_wear_name, lower_wear_name, upper_bearing_name, lower_bearing_name in (
        (-0.155, "upper_wear_0", "lower_wear_0", "upper_bearing_0", "lower_bearing_0"),
        (0.155, "upper_wear_1", "lower_wear_1", "upper_bearing_1", "lower_bearing_1"),
    ):
        jaw.visual(
            Box((0.205, 0.052, 0.006)),
            origin=Origin(xyz=(sign * 0.040, y, 0.037)),
            material=bronze,
            name=upper_wear_name,
        )
        jaw.visual(
            Box((0.205, 0.052, 0.006)),
            origin=Origin(xyz=(sign * 0.040, y, -0.037)),
            material=bronze,
            name=lower_wear_name,
        )
        jaw.visual(
            Box((0.205, 0.064, 0.018)),
            origin=Origin(xyz=(sign * 0.040, y, 0.029)),
            material=steel,
            name=upper_bearing_name,
        )
        jaw.visual(
            Box((0.205, 0.064, 0.018)),
            origin=Origin(xyz=(sign * 0.040, y, -0.029)),
            material=steel,
            name=lower_bearing_name,
        )

    for cheek_index, y in enumerate((-0.205, 0.205)):
        jaw.visual(
            Box((0.180, 0.020, 0.140)),
            origin=Origin(xyz=(-sign * 0.055, y, 0.0)),
            material=dark,
            name=f"side_cheek_{cheek_index}",
        )

    jaw.visual(
        Box((0.280, 0.050, 0.032)),
        origin=Origin(xyz=(sign * 0.145, 0.0, 0.0)),
        material=steel,
        name="inner_ram",
    )
    jaw.visual(
        Box((0.040, 0.086, 0.050)),
        origin=Origin(xyz=(sign * 0.284, 0.0, -0.012)),
        material=steel,
        name="jaw_lower_step",
    )
    jaw.visual(
        Box((0.050, 0.078, 0.050)),
        origin=Origin(xyz=(sign * 0.280, 0.0, 0.024)),
        material=steel,
        name="jaw_upper_step",
    )
    jaw.visual(
        Box((0.006, 0.082, 0.075)),
        origin=Origin(xyz=(sign * 0.306, 0.0, 0.005)),
        material=grip,
        name="grip_insert",
    )
    for groove_index, z in enumerate((-0.020, 0.000, 0.020)):
        jaw.visual(
            Box((0.003, 0.084, 0.004)),
            origin=Origin(xyz=(sign * 0.309, 0.0, z)),
            material=dark,
            name=f"grip_rib_{groove_index}",
        )

    bolt_positions = (
        (-sign * 0.055, -0.130),
        (-sign * 0.055, 0.130),
        (sign * 0.075, -0.130),
        (sign * 0.075, 0.130),
    )
    for bolt_index, (x, y) in enumerate(bolt_positions):
        jaw.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(x, y, 0.073), rpy=(0.0, 0.0, 0.0)),
            material=dark,
            name=f"{prefix}_cap_screw_{bolt_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_block_gripper")

    cast = model.material("cast_iron_blue_grey", rgba=(0.18, 0.22, 0.25, 1.0))
    dark = model.material("black_oxide", rgba=(0.055, 0.060, 0.065, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    jaw_steel = model.material("blued_jaw_steel", rgba=(0.23, 0.25, 0.26, 1.0))
    bronze = model.material("bronze_wear_strip", rgba=(0.72, 0.48, 0.20, 1.0))
    grip = model.material("hardened_grip_face", rgba=(0.11, 0.12, 0.13, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(
            _rectangular_tube_x(
                length=0.360,
                outer_y=0.220,
                outer_z=0.170,
                inner_y=0.170,
                inner_z=0.110,
            ),
            "gripper_hollow_housing",
        ),
        material=cast,
        name="housing_shell",
    )
    housing.visual(
        Box((0.320, 0.205, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=dark,
        name="top_cover",
    )
    housing.visual(
        Box((0.320, 0.205, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=dark,
        name="bottom_cover",
    )
    housing.visual(
        Box((0.105, 0.380, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=cast,
        name="cross_tie_cover",
    )

    for rail_index, (y, rail_name) in enumerate(((-0.155, "rail_0"), (0.155, "rail_1"))):
        housing.visual(
            Box((1.060, 0.036, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=rail_steel,
            name=rail_name,
        )
        housing.visual(
            Box((0.105, 0.090, 0.060)),
            origin=Origin(xyz=(0.0, y * 0.87, 0.0)),
            material=cast,
            name=f"rail_boss_{rail_index}",
        )
        for stop_index, x in enumerate((-0.500, 0.500)):
            housing.visual(
                Box((0.038, 0.074, 0.090)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=dark,
                name=f"rail_stop_{rail_index}_{stop_index}",
            )

    _add_socket_bolts(housing, material=dark, z=0.105, prefix="housing_bolt")
    for rail_index, y in enumerate((-0.155, 0.155)):
        for bolt_index, x in enumerate((-0.030, 0.030)):
            housing.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=Origin(xyz=(x, y, 0.0225)),
                material=dark,
                name=f"rail_bolt_{rail_index}_{bolt_index}",
            )

    housing.inertial = Inertial.from_geometry(
        Box((1.080, 0.410, 0.210)),
        mass=18.0,
        origin=Origin(),
    )

    jaw_0 = model.part("jaw_carriage_0")
    _add_jaw_carriage(
        jaw_0,
        sign=1.0,
        steel=jaw_steel,
        dark=dark,
        bronze=bronze,
        grip=grip,
        prefix="jaw0",
    )
    jaw_0.inertial = Inertial.from_geometry(
        Box((0.440, 0.430, 0.170)),
        mass=5.5,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    jaw_1 = model.part("jaw_carriage_1")
    _add_jaw_carriage(
        jaw_1,
        sign=-1.0,
        steel=jaw_steel,
        dark=dark,
        bronze=bronze,
        grip=grip,
        prefix="jaw1",
    )
    jaw_1.inertial = Inertial.from_geometry(
        Box((0.440, 0.430, 0.170)),
        mass=5.5,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=jaw_0,
        origin=Origin(xyz=(-0.340, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.025),
    )
    model.articulation(
        "housing_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=jaw_1,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    jaw_0 = object_model.get_part("jaw_carriage_0")
    jaw_1 = object_model.get_part("jaw_carriage_1")
    joint_0 = object_model.get_articulation("housing_to_jaw_0")
    joint_1 = object_model.get_articulation("housing_to_jaw_1")

    ctx.check(
        "jaws use independent prismatic joints",
        joint_0.articulation_type == ArticulationType.PRISMATIC
        and joint_1.articulation_type == ArticulationType.PRISMATIC
        and joint_0.mimic is None
        and joint_1.mimic is None,
        details=f"joint_0={joint_0.articulation_type}, joint_1={joint_1.articulation_type}",
    )

    ctx.expect_gap(
        jaw_1,
        jaw_0,
        axis="x",
        positive_elem="grip_insert",
        negative_elem="grip_insert",
        min_gap=0.055,
        max_gap=0.070,
        name="open jaws have a visible mouth gap",
    )
    ctx.expect_overlap(
        jaw_0,
        housing,
        axes="x",
        elem_a="upper_bearing_0",
        elem_b="rail_0",
        min_overlap=0.180,
        name="jaw 0 bearing overlaps fixed rail at rest",
    )
    ctx.expect_overlap(
        jaw_1,
        housing,
        axes="x",
        elem_a="upper_bearing_1",
        elem_b="rail_1",
        min_overlap=0.180,
        name="jaw 1 bearing overlaps fixed rail at rest",
    )
    ctx.expect_gap(
        jaw_0,
        housing,
        axis="z",
        positive_elem="upper_wear_0",
        negative_elem="rail_0",
        min_gap=0.010,
        max_gap=0.020,
        name="upper bearing strip clears rail",
    )
    ctx.expect_gap(
        housing,
        jaw_0,
        axis="z",
        positive_elem="rail_0",
        negative_elem="lower_wear_0",
        min_gap=0.010,
        max_gap=0.020,
        name="lower bearing strip clears rail",
    )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({joint_0: 0.025, joint_1: 0.025}):
        ctx.expect_gap(
            jaw_1,
            jaw_0,
            axis="x",
            positive_elem="grip_insert",
            negative_elem="grip_insert",
            min_gap=0.010,
            max_gap=0.018,
            name="closed jaws stop before the grip faces collide",
        )
        ctx.expect_gap(
            housing,
            jaw_0,
            axis="x",
            positive_elem="housing_shell",
            negative_elem="upper_carriage_cap",
            min_gap=0.004,
            max_gap=0.020,
            name="jaw 0 carriage clears housing at inward stop",
        )
        ctx.expect_gap(
            jaw_1,
            housing,
            axis="x",
            positive_elem="upper_carriage_cap",
            negative_elem="housing_shell",
            min_gap=0.004,
            max_gap=0.020,
            name="jaw 1 carriage clears housing at inward stop",
        )
        closed_0 = ctx.part_world_position(jaw_0)
        closed_1 = ctx.part_world_position(jaw_1)

    ctx.check(
        "symmetric closing motion reduces jaw spacing",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and (closed_1[0] - closed_0[0]) < (rest_1[0] - rest_0[0]) - 0.045,
        details=f"rest={rest_0},{rest_1}; closed={closed_0},{closed_1}",
    )

    return ctx.report()


object_model = build_object_model()
