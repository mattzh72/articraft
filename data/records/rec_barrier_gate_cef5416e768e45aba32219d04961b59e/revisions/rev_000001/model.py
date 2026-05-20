from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PLATE_LENGTH = 1.10
PLATE_WIDTH = 3.16
PLATE_FRONT_DEPTH = 0.82
PLATE_REAR_DEPTH = 0.06
OPENING_LENGTH = 1.12
OPENING_WIDTH = 3.22
PIT_DEPTH = 0.94


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _extrude_profile_along_y(
    profile_xz: list[tuple[float, float]],
    *,
    width: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    left_ids: list[int] = []
    right_ids: list[int] = []
    half_width = width * 0.5

    for x_pos, z_pos in profile_xz:
        left_ids.append(geom.add_vertex(x_pos, -half_width, z_pos))
        right_ids.append(geom.add_vertex(x_pos, half_width, z_pos))

    count = len(profile_xz)
    for index in range(count):
        next_index = (index + 1) % count
        _add_quad(
            geom,
            left_ids[index],
            right_ids[index],
            right_ids[next_index],
            left_ids[next_index],
        )

    for index in range(1, count - 1):
        geom.add_face(left_ids[0], left_ids[index + 1], left_ids[index])
        geom.add_face(right_ids[0], right_ids[index], right_ids[index + 1])

    return geom


def _build_wedge_plate_mesh() -> MeshGeometry:
    profile = [
        (0.0, 0.0),
        (0.0, -PLATE_REAR_DEPTH),
        (PLATE_LENGTH, -PLATE_FRONT_DEPTH),
        (PLATE_LENGTH, 0.0),
    ]
    return _extrude_profile_along_y(profile, width=PLATE_WIDTH)


def _build_grade_frame_mesh() -> MeshGeometry:
    outer_profile = [
        (-0.95, -1.90),
        (0.95, -1.90),
        (0.95, 1.90),
        (-0.95, 1.90),
    ]
    hole_profile = [
        (-OPENING_LENGTH * 0.5, -OPENING_WIDTH * 0.5),
        (-OPENING_LENGTH * 0.5, OPENING_WIDTH * 0.5),
        (OPENING_LENGTH * 0.5, OPENING_WIDTH * 0.5),
        (OPENING_LENGTH * 0.5, -OPENING_WIDTH * 0.5),
    ]
    return ExtrudeWithHolesGeometry(
        outer_profile,
        [hole_profile],
        0.18,
        center=True,
        cap=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_road_wedge_barrier")

    asphalt = model.material("asphalt", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.28, 0.30, 0.33, 1.0))
    galvanized = model.material("galvanized", rgba=(0.54, 0.56, 0.59, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.73, 0.12, 1.0))
    concrete = model.material("concrete", rgba=(0.55, 0.55, 0.56, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_build_grade_frame_mesh(), "wedge_barrier_grade_frame"),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=asphalt,
        name="grade_frame",
    )
    housing.visual(
        Box((1.48, OPENING_WIDTH + 0.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -PIT_DEPTH + 0.05)),
        material=concrete,
        name="pit_floor",
    )
    housing.visual(
        Box((1.48, 0.12, PIT_DEPTH)),
        origin=Origin(xyz=(0.0, OPENING_WIDTH * 0.5 + 0.06, -PIT_DEPTH * 0.5)),
        material=steel,
        name="left_liner",
    )
    housing.visual(
        Box((1.48, 0.12, PIT_DEPTH)),
        origin=Origin(xyz=(0.0, -OPENING_WIDTH * 0.5 - 0.06, -PIT_DEPTH * 0.5)),
        material=steel,
        name="right_liner",
    )
    housing.visual(
        Box((0.12, OPENING_WIDTH, PIT_DEPTH)),
        origin=Origin(xyz=(OPENING_LENGTH * 0.5 + 0.06, 0.0, -PIT_DEPTH * 0.5)),
        material=steel,
        name="front_liner",
    )
    housing.visual(
        Box((0.18, OPENING_WIDTH, PIT_DEPTH)),
        origin=Origin(xyz=(-0.64, 0.0, -PIT_DEPTH * 0.5)),
        material=steel,
        name="rear_liner",
    )
    housing.visual(
        Box((0.24, OPENING_WIDTH + 0.20, 0.18)),
        origin=Origin(xyz=(-0.72, 0.0, -0.20)),
        material=galvanized,
        name="rear_transom",
    )
    housing.inertial = Inertial.from_geometry(
        Box((1.90, 3.80, PIT_DEPTH)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, -PIT_DEPTH * 0.5)),
    )

    wedge_plate = model.part("wedge_plate")
    wedge_plate.visual(
        mesh_from_geometry(_build_wedge_plate_mesh(), "wedge_barrier_plate_body"),
        material=safety_yellow,
        name="plate_body",
    )
    wedge_plate.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_FRONT_DEPTH)),
        mass=780.0,
        origin=Origin(xyz=(PLATE_LENGTH * 0.5, 0.0, -PLATE_FRONT_DEPTH * 0.4)),
    )

    model.articulation(
        "housing_to_wedge_plate",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=wedge_plate,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40000.0,
            velocity=0.45,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    wedge_plate = object_model.get_part("wedge_plate")
    hinge = object_model.get_articulation("housing_to_wedge_plate")
    hinge_limits = hinge.motion_limits

    ctx.check(
        "wedge barrier parts and hinge exist",
        housing is not None and wedge_plate is not None and hinge is not None,
        details="Expected housing part, wedge plate part, and rear hinge articulation.",
    )
    ctx.check(
        "hinge is a rear-edge revolute lift",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (0.0, -1.0, 0.0)
        and hinge_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and hinge_limits.upper >= 0.9,
        details=(
            f"type={hinge.articulation_type}, axis={hinge.axis}, "
            f"limits={hinge_limits}"
        ),
    )

    closed_box = ctx.part_element_world_aabb(wedge_plate, elem="plate_body")
    ctx.check(
        "plate closes flush to grade",
        closed_box is not None
        and abs(closed_box[1][2]) <= 0.002
        and closed_box[0][2] < -0.70,
        details=f"closed_box={closed_box}",
    )
    ctx.check(
        "plate nests inside the pit opening at rest",
        closed_box is not None
        and closed_box[0][0] >= -0.551
        and closed_box[1][0] <= 0.551
        and closed_box[0][1] >= -(OPENING_WIDTH * 0.5)
        and closed_box[1][1] <= OPENING_WIDTH * 0.5,
        details=f"closed_box={closed_box}",
    )
    ctx.expect_contact(
        wedge_plate,
        housing,
        elem_a="plate_body",
        elem_b="rear_liner",
        contact_tol=0.001,
        name="rear edge stays mounted on the hinge-side liner",
    )

    upper_limit = 0.95 if hinge_limits is None or hinge_limits.upper is None else hinge_limits.upper
    with ctx.pose({hinge: upper_limit}):
        raised_box = ctx.part_element_world_aabb(wedge_plate, elem="plate_body")

    ctx.check(
        "raised pose lifts the front edge into a stopping barrier",
        closed_box is not None
        and raised_box is not None
        and raised_box[1][2] > 0.85
        and raised_box[1][2] > closed_box[1][2] + 0.80,
        details=f"closed_box={closed_box}, raised_box={raised_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
