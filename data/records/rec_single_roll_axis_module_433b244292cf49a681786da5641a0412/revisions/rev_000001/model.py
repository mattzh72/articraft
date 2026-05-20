from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHAFT_CENTER_X = 0.18
SHAFT_CENTER_Z = 0.20
REAR_BEARING_X = 0.10
FRONT_BEARING_X = 0.25
BEARING_LENGTH = 0.05
BEARING_WIDTH = 0.24
BEARING_HEIGHT = 0.16
BORE_RADIUS = 0.037
SHAFT_RADIUS = 0.026
SHAFT_LENGTH = 0.285


def _bearing_face_ring_mesh(name: str):
    geom = TorusGeometry(radius=0.049, tube=0.0075, radial_segments=20, tubular_segments=48)
    geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_roll_spindle")

    painted = Material("blue_powdercoat", rgba=(0.08, 0.18, 0.30, 1.0))
    dark = Material("dark_wall_plate", rgba=(0.045, 0.050, 0.055, 1.0))
    rib = Material("ribbed_cast_metal", rgba=(0.13, 0.15, 0.17, 1.0))
    bearing = Material("matte_bearing_blocks", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = Material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    bronze = Material("bronze_bearing_liner", rgba=(0.74, 0.50, 0.22, 1.0))
    flange_mat = Material("orange_moving_flange", rgba=(0.95, 0.42, 0.08, 1.0))
    black = Material("black_fasteners", rgba=(0.02, 0.02, 0.018, 1.0))
    mark = Material("white_index_mark", rgba=(0.93, 0.92, 0.84, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.035, 0.42, 0.34)),
        origin=Origin(xyz=(0.0175, 0.0, SHAFT_CENTER_Z)),
        material=dark,
        name="wall_backplate",
    )
    support.visual(
        Box((0.24, 0.026, 0.22)),
        origin=Origin(xyz=(0.155, 0.122, SHAFT_CENTER_Z)),
        material=painted,
        name="side_rib_0",
    )
    support.visual(
        Box((0.24, 0.026, 0.22)),
        origin=Origin(xyz=(0.155, -0.122, SHAFT_CENTER_Z)),
        material=painted,
        name="side_rib_1",
    )
    support.visual(
        Box((0.225, 0.25, 0.018)),
        origin=Origin(xyz=(0.152, 0.0, SHAFT_CENTER_Z + 0.095)),
        material=rib,
        name="top_tie_rib",
    )
    support.visual(
        Box((0.225, 0.25, 0.018)),
        origin=Origin(xyz=(0.152, 0.0, SHAFT_CENTER_Z - 0.095)),
        material=rib,
        name="bottom_tie_rib",
    )

    support.visual(
        Box((BEARING_LENGTH, BEARING_WIDTH, 0.035)),
        origin=Origin(xyz=(REAR_BEARING_X, 0.0, SHAFT_CENTER_Z + 0.0625)),
        material=bearing,
        name="rear_bearing_upper",
    )
    support.visual(
        Box((BEARING_LENGTH, BEARING_WIDTH, 0.035)),
        origin=Origin(xyz=(REAR_BEARING_X, 0.0, SHAFT_CENTER_Z - 0.0625)),
        material=bearing,
        name="rear_bearing_lower",
    )
    support.visual(
        Box((BEARING_LENGTH, 0.060, BEARING_HEIGHT)),
        origin=Origin(xyz=(REAR_BEARING_X, 0.090, SHAFT_CENTER_Z)),
        material=bearing,
        name="rear_bearing_side_0",
    )
    support.visual(
        Box((BEARING_LENGTH, 0.060, BEARING_HEIGHT)),
        origin=Origin(xyz=(REAR_BEARING_X, -0.090, SHAFT_CENTER_Z)),
        material=bearing,
        name="rear_bearing_side_1",
    )
    support.visual(
        _bearing_face_ring_mesh("rear_bearing_ring_0"),
        origin=Origin(xyz=(REAR_BEARING_X - BEARING_LENGTH / 2.0 - 0.004, 0.0, SHAFT_CENTER_Z)),
        material=steel,
        name="rear_bearing_ring_0",
    )
    support.visual(
        _bearing_face_ring_mesh("rear_bearing_ring_1"),
        origin=Origin(xyz=(REAR_BEARING_X + BEARING_LENGTH / 2.0 + 0.004, 0.0, SHAFT_CENTER_Z)),
        material=steel,
        name="rear_bearing_ring_1",
    )
    support.visual(
        Box((BEARING_LENGTH, 0.040, 0.019)),
        origin=Origin(xyz=(REAR_BEARING_X, 0.0, SHAFT_CENTER_Z - SHAFT_RADIUS - 0.0095)),
        material=bronze,
        name="rear_bearing_saddle",
    )
    support.visual(
        Box((BEARING_LENGTH, BEARING_WIDTH, 0.035)),
        origin=Origin(xyz=(FRONT_BEARING_X, 0.0, SHAFT_CENTER_Z + 0.0625)),
        material=bearing,
        name="front_bearing_upper",
    )
    support.visual(
        Box((BEARING_LENGTH, BEARING_WIDTH, 0.035)),
        origin=Origin(xyz=(FRONT_BEARING_X, 0.0, SHAFT_CENTER_Z - 0.0625)),
        material=bearing,
        name="front_bearing_lower",
    )
    support.visual(
        Box((BEARING_LENGTH, 0.060, BEARING_HEIGHT)),
        origin=Origin(xyz=(FRONT_BEARING_X, 0.090, SHAFT_CENTER_Z)),
        material=bearing,
        name="front_bearing_side_0",
    )
    support.visual(
        Box((BEARING_LENGTH, 0.060, BEARING_HEIGHT)),
        origin=Origin(xyz=(FRONT_BEARING_X, -0.090, SHAFT_CENTER_Z)),
        material=bearing,
        name="front_bearing_side_1",
    )
    support.visual(
        _bearing_face_ring_mesh("front_bearing_ring_0"),
        origin=Origin(xyz=(FRONT_BEARING_X - BEARING_LENGTH / 2.0 - 0.004, 0.0, SHAFT_CENTER_Z)),
        material=steel,
        name="front_bearing_ring_0",
    )
    support.visual(
        _bearing_face_ring_mesh("front_bearing_ring_1"),
        origin=Origin(xyz=(FRONT_BEARING_X + BEARING_LENGTH / 2.0 + 0.004, 0.0, SHAFT_CENTER_Z)),
        material=steel,
        name="front_bearing_ring_1",
    )
    support.visual(
        Box((BEARING_LENGTH, 0.040, 0.019)),
        origin=Origin(xyz=(FRONT_BEARING_X, 0.0, SHAFT_CENTER_Z - SHAFT_RADIUS - 0.0095)),
        material=bronze,
        name="front_bearing_saddle",
    )

    for screw_index, (y, z) in enumerate(
        (
            (-0.165, SHAFT_CENTER_Z - 0.125),
            (0.165, SHAFT_CENTER_Z - 0.125),
            (-0.165, SHAFT_CENTER_Z + 0.125),
            (0.165, SHAFT_CENTER_Z + 0.125),
        )
    ):
        support.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(xyz=(0.036, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"backplate_bolt_{screw_index}",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.074, length=0.026),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=flange_mat,
        name="flange_disc",
    )
    spindle.visual(
        Cylinder(radius=0.042, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    spindle.visual(
        Box((0.010, 0.030, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.061)),
        material=mark,
        name="index_mark",
    )
    for bolt_index in range(6):
        angle = tau * bolt_index / 6.0
        spindle.visual(
            Cylinder(radius=0.0065, length=0.010),
            origin=Origin(
                xyz=(0.018, 0.046 * cos(angle), 0.046 * sin(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=black,
            name=f"flange_bolt_{bolt_index}",
        )

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=support,
        child=spindle,
        origin=Origin(xyz=(SHAFT_CENTER_X, 0.0, SHAFT_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    spindle = object_model.get_part("spindle")
    roll_joint = object_model.get_articulation("roll_joint")

    ctx.expect_gap(
        spindle,
        support,
        axis="x",
        positive_elem="flange_disc",
        negative_elem="rear_bearing_upper",
        min_gap=0.035,
        name="flange clears rear bearing",
    )
    ctx.expect_gap(
        support,
        spindle,
        axis="x",
        positive_elem="front_bearing_upper",
        negative_elem="flange_disc",
        min_gap=0.030,
        name="flange clears front bearing",
    )
    ctx.expect_gap(
        support,
        spindle,
        axis="z",
        positive_elem="front_bearing_upper",
        negative_elem="shaft",
        min_gap=0.010,
        name="front bearing upper clears shaft",
    )
    ctx.expect_gap(
        spindle,
        support,
        axis="z",
        positive_elem="shaft",
        negative_elem="front_bearing_lower",
        min_gap=0.010,
        name="front bearing lower clears shaft",
    )
    ctx.expect_gap(
        support,
        spindle,
        axis="y",
        positive_elem="front_bearing_side_0",
        negative_elem="shaft",
        min_gap=0.020,
        name="front bearing side clears shaft",
    )
    ctx.expect_gap(
        spindle,
        support,
        axis="y",
        positive_elem="shaft",
        negative_elem="front_bearing_side_1",
        min_gap=0.020,
        name="opposite bearing side clears shaft",
    )
    ctx.expect_overlap(
        spindle,
        support,
        axes="x",
        elem_a="shaft",
        elem_b="rear_bearing_upper",
        min_overlap=0.045,
        name="shaft spans rear bearing station",
    )
    ctx.expect_overlap(
        spindle,
        support,
        axes="x",
        elem_a="shaft",
        elem_b="front_bearing_upper",
        min_overlap=0.045,
        name="shaft spans front bearing station",
    )
    ctx.expect_contact(
        spindle,
        support,
        elem_a="shaft",
        elem_b="front_bearing_saddle",
        contact_tol=0.0005,
        name="front bearing saddle supports shaft",
    )
    ctx.expect_contact(
        spindle,
        support,
        elem_a="shaft",
        elem_b="rear_bearing_saddle",
        contact_tol=0.0005,
        name="rear bearing saddle supports shaft",
    )

    rest_mark = ctx.part_element_world_aabb(spindle, elem="index_mark")
    rest_center = None
    if rest_mark is not None:
        rest_center = tuple((rest_mark[0][i] + rest_mark[1][i]) / 2.0 for i in range(3))
    with ctx.pose({roll_joint: pi / 2.0}):
        turned_mark = ctx.part_element_world_aabb(spindle, elem="index_mark")
        turned_center = None
        if turned_mark is not None:
            turned_center = tuple((turned_mark[0][i] + turned_mark[1][i]) / 2.0 for i in range(3))
    ctx.check(
        "index mark rotates about roll axis",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[2] - (SHAFT_CENTER_Z + 0.061)) < 0.012
        and turned_center[1] < -0.045
        and abs(turned_center[2] - SHAFT_CENTER_Z) < 0.018,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
