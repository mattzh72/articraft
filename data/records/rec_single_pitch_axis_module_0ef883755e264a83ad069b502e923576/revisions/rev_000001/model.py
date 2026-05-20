from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangular_web(
    *,
    x_center: float,
    thickness: float,
    y_back: float,
    y_front: float,
    z_low: float,
    z_high: float,
) -> MeshGeometry:
    """Closed triangular gusset plate, expressed in the support frame."""

    x0 = x_center - thickness / 2.0
    x1 = x_center + thickness / 2.0
    vertices = [
        (x0, y_back, z_low),
        (x0, y_back, z_high),
        (x0, y_front, z_low),
        (x1, y_back, z_low),
        (x1, y_back, z_high),
        (x1, y_front, z_low),
    ]
    faces = [
        (0, 2, 1),
        (3, 4, 5),
        (0, 1, 4),
        (0, 4, 3),
        (0, 3, 5),
        (0, 5, 2),
        (1, 2, 5),
        (1, 5, 4),
    ]
    return MeshGeometry(vertices, faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_trunnion_module")

    paint = model.material("warm_powdercoat", rgba=(0.58, 0.60, 0.58, 1.0))
    dark = model.material("blackened_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    bearing = model.material("brushed_bearing", rgba=(0.72, 0.70, 0.64, 1.0))
    moving = model.material("blue_rotary_housing", rgba=(0.08, 0.20, 0.35, 1.0))
    glass = model.material("smoked_front_glass", rgba=(0.01, 0.015, 0.02, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.42, 0.040, 0.38)),
        origin=Origin(xyz=(0.0, -0.146, 0.0)),
        material=paint,
        name="wall_plate",
    )
    support.visual(
        Box((0.044, 0.180, 0.260)),
        origin=Origin(xyz=(-0.122, -0.042, 0.0)),
        material=paint,
        name="cheek_0",
    )
    support.visual(
        Box((0.044, 0.180, 0.260)),
        origin=Origin(xyz=(0.122, -0.042, 0.0)),
        material=paint,
        name="cheek_1",
    )
    support.visual(
        Box((0.300, 0.180, 0.042)),
        origin=Origin(xyz=(0.0, -0.042, -0.109)),
        material=paint,
        name="lower_bridge",
    )
    for index, x_center in enumerate((-0.074, 0.074)):
        support.visual(
            mesh_from_geometry(
                _triangular_web(
                    x_center=x_center,
                    thickness=0.014,
                    y_back=-0.158,
                    y_front=0.028,
                    z_low=-0.112,
                    z_high=0.112,
                ),
                f"side_rib_{index}",
            ),
            material=paint,
            name=f"side_rib_{index}",
        )

    # Wall bolts and outboard bearing bosses keep the fixed support visually
    # separate from the moving head while making the trunnion axis legible.
    for index, (x, z) in enumerate(
        ((-0.160, -0.140), (0.160, -0.140), (-0.160, 0.140), (0.160, 0.140))
    ):
        support.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(x, -0.120, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"wall_bolt_{index}",
        )
    for index, x in enumerate((-0.094, 0.094)):
        support.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing,
            name=f"inner_bearing_{index}",
        )
    for index, x in enumerate((-0.154, 0.154)):
        support.visual(
            Cylinder(radius=0.047, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing,
            name=f"bearing_boss_{index}",
        )
        support.visual(
            Cylinder(radius=0.018, length=0.021),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"bore_shadow_{index}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.192),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.082),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="inner_hub",
    )
    head.visual(
        Box((0.122, 0.076, 0.058)),
        origin=Origin(xyz=(0.0, 0.037, 0.0)),
        material=moving,
        name="pitch_arm",
    )
    head.visual(
        Cylinder(radius=0.074, length=0.082),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=moving,
        name="rotary_body",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.0, 0.140, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_face",
    )
    head.visual(
        Box((0.090, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.088, 0.074)),
        material=dark,
        name="top_service_cap",
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=support,
        child=head,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=42.0, velocity=1.8, lower=-0.40, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    head = object_model.get_part("head")
    joint = object_model.get_articulation("pitch_trunnion")

    ctx.check(
        "single pitch axis is horizontal",
        tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={joint.axis}",
    )
    ctx.check(
        "pitch travel has useful up and down stops",
        joint.motion_limits is not None
        and joint.motion_limits.lower < -0.30
        and joint.motion_limits.upper > 0.55,
        details=f"limits={joint.motion_limits}",
    )

    for boss in ("inner_bearing_0", "inner_bearing_1"):
        ctx.allow_overlap(
            support,
            head,
            elem_a=boss,
            elem_b="trunnion_shaft",
            reason="The trunnion shaft is intentionally captured a few millimetres inside the cheek bearing.",
        )
        ctx.expect_overlap(
            support,
            head,
            axes="x",
            min_overlap=0.002,
            elem_a=boss,
            elem_b="trunnion_shaft",
            name=f"{boss} captures shaft end",
        )
        ctx.expect_within(
            head,
            support,
            axes="yz",
            margin=0.001,
            inner_elem="trunnion_shaft",
            outer_elem=boss,
            name=f"shaft is centered in {boss}",
        )

    with ctx.pose({joint: 0.0}):
        for cheek in ("cheek_0", "cheek_1"):
            ctx.expect_overlap(
                support,
                head,
                axes="yz",
                min_overlap=0.020,
                elem_a=cheek,
                elem_b="trunnion_shaft",
                name=f"shaft aligns through {cheek}",
            )

    rest_face = ctx.part_element_world_aabb(head, elem="front_face")
    with ctx.pose({joint: joint.motion_limits.upper}):
        raised_face = ctx.part_element_world_aabb(head, elem="front_face")
        ctx.expect_gap(
            head,
            support,
            axis="y",
            min_gap=0.006,
            positive_elem="front_face",
            negative_elem="cheek_1",
            name="raised face stays forward of support",
        )
    with ctx.pose({joint: joint.motion_limits.lower}):
        lowered_face = ctx.part_element_world_aabb(head, elem="front_face")
        ctx.expect_gap(
            head,
            support,
            axis="y",
            min_gap=0.006,
            positive_elem="front_face",
            negative_elem="cheek_1",
            name="lowered face stays forward of support",
        )
    ctx.check(
        "positive pitch lifts the head",
        rest_face is not None
        and raised_face is not None
        and raised_face[1][2] > rest_face[1][2] + 0.035,
        details=f"rest={rest_face}, raised={raised_face}",
    )
    ctx.check(
        "negative pitch lowers the head",
        rest_face is not None
        and lowered_face is not None
        and lowered_face[0][2] < rest_face[0][2] - 0.020,
        details=f"rest={rest_face}, lowered={lowered_face}",
    )

    return ctx.report()


object_model = build_object_model()
