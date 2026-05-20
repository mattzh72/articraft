from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.36
BASE_WIDTH = 0.26
BASE_THICKNESS = 0.026
CHEEK_THICKNESS = 0.028
CHEEK_Y = 0.096
CHEEK_HEIGHT = 0.170
CHEEK_WIDTH_X = 0.176
AXIS_Z = 0.125
TRUNNION_RADIUS = 0.020
BUSHING_INNER_RADIUS = 0.027


def _y_cylinder(length: float, radius: float) -> cq.Workplane:
    """Cylinder centered on the origin with its axis along the model Y axis."""
    return cq.Workplane("XY").cylinder(
        length,
        radius,
        centered=(True, True, True),
    ).rotate((0, 0, 0), (1, 0, 0), 90)


def _base_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    bolt_inset_x = 0.040
    bolt_inset_y = 0.035
    bolt_points = [
        (-BASE_LENGTH / 2 + bolt_inset_x, -BASE_WIDTH / 2 + bolt_inset_y),
        (-BASE_LENGTH / 2 + bolt_inset_x, BASE_WIDTH / 2 - bolt_inset_y),
        (BASE_LENGTH / 2 - bolt_inset_x, -BASE_WIDTH / 2 + bolt_inset_y),
        (BASE_LENGTH / 2 - bolt_inset_x, BASE_WIDTH / 2 - bolt_inset_y),
    ]
    return (
        plate.edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(bolt_points)
        .circle(0.0065)
        .cutThruAll()
    )


def _cheek_shape(y_center: float) -> cq.Workplane:
    embed = 0.002
    cheek = cq.Workplane("XY").box(
        CHEEK_WIDTH_X,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
    ).translate((0.0, y_center, BASE_THICKNESS + CHEEK_HEIGHT / 2 - embed))

    bearing_hole = _y_cylinder(CHEEK_THICKNESS + 0.018, BUSHING_INNER_RADIUS + 0.006)
    bearing_hole = bearing_hole.translate((0.0, y_center, AXIS_Z))

    return cheek.cut(bearing_hole).edges("|Y").fillet(0.004)


def _bushing_shape(y_center: float) -> cq.Workplane:
    outer = _y_cylinder(CHEEK_THICKNESS + 0.016, 0.040).translate(
        (0.0, y_center, AXIS_Z)
    )
    inner = _y_cylinder(CHEEK_THICKNESS + 0.030, BUSHING_INNER_RADIUS).translate(
        (0.0, y_center, AXIS_Z)
    )
    return outer.cut(inner)


def _head_core_shape() -> cq.Workplane:
    core = cq.Workplane("XY").box(0.110, 0.118, 0.092)
    return core.edges("|X or |Y or |Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_trunnion")

    cast_iron = model.material("dark_cast_iron", rgba=(0.14, 0.16, 0.17, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.66, 0.68, 0.68, 1.0))
    bronze = model.material("bronze_bushing", rgba=(0.72, 0.47, 0.20, 1.0))
    face_material = model.material("plain_output_face", rgba=(0.86, 0.87, 0.84, 1.0))
    bolt_black = model.material("black_socket_heads", rgba=(0.03, 0.03, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate", tolerance=0.0008),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_cheek_shape(-CHEEK_Y), "cheek_0", tolerance=0.0008),
        material=cast_iron,
        name="cheek_0",
    )
    base.visual(
        mesh_from_cadquery(
            _bushing_shape(-CHEEK_Y),
            "bushing_0",
            tolerance=0.0008,
            angular_tolerance=0.04,
        ),
        material=bronze,
        name="bushing_0",
    )
    base.visual(
        mesh_from_cadquery(_cheek_shape(CHEEK_Y), "cheek_1", tolerance=0.0008),
        material=cast_iron,
        name="cheek_1",
    )
    base.visual(
        mesh_from_cadquery(
            _bushing_shape(CHEEK_Y),
            "bushing_1",
            tolerance=0.0008,
            angular_tolerance=0.04,
        ),
        material=bronze,
        name="bushing_1",
    )

    bolt_points = [
        (-BASE_LENGTH / 2 + 0.040, -BASE_WIDTH / 2 + 0.035),
        (-BASE_LENGTH / 2 + 0.040, BASE_WIDTH / 2 - 0.035),
        (BASE_LENGTH / 2 - 0.040, -BASE_WIDTH / 2 + 0.035),
        (BASE_LENGTH / 2 - 0.040, BASE_WIDTH / 2 - 0.035),
    ]
    for index, (x_pos, y_pos) in enumerate(bolt_points):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, BASE_THICKNESS + 0.003)),
            material=bolt_black,
            name=f"bolt_{index}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_core_shape(), "head_core", tolerance=0.0008),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=machined_steel,
        name="head_core",
    )
    head.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=0.252),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=machined_steel,
        name="trunnion_shaft",
    )
    for index, y_pos in enumerate((-0.124, 0.124)):
        head.visual(
            Cylinder(radius=0.033, length=0.012),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=machined_steel,
            name=f"shaft_cap_{index}",
        )
    head.visual(
        Box((0.014, 0.108, 0.076)),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=face_material,
        name="output_face",
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=0.55,
            effort=45.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("pitch_trunnion")

    ctx.check(
        "one pitch revolute joint",
        len(object_model.articulations) == 1
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"joints={object_model.articulations}",
    )

    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.040,
        positive_elem="head_core",
        negative_elem="base_plate",
        name="moving head clears the base plate",
    )
    ctx.expect_within(
        head,
        base,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bushing_0",
        margin=0.001,
        name="shaft is centered in the first bearing bushing",
    )
    ctx.expect_within(
        head,
        base,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bushing_1",
        margin=0.001,
        name="shaft is centered in the second bearing bushing",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="y",
        min_overlap=0.020,
        elem_a="trunnion_shaft",
        elem_b="bushing_0",
        name="shaft is retained by the first cheek",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="y",
        min_overlap=0.020,
        elem_a="trunnion_shaft",
        elem_b="bushing_1",
        name="shaft is retained by the second cheek",
    )

    def _face_center_z() -> float | None:
        bounds = ctx.part_element_world_aabb(head, elem="output_face")
        if bounds is None:
            return None
        return (bounds[0][2] + bounds[1][2]) / 2

    with ctx.pose({pitch: -0.55}):
        lower_face_z = _face_center_z()
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.008,
            positive_elem="output_face",
            negative_elem="base_plate",
            name="lower-limit output face clears base",
        )
    with ctx.pose({pitch: 0.55}):
        upper_face_z = _face_center_z()
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.008,
            positive_elem="output_face",
            negative_elem="base_plate",
            name="upper-limit output face clears base",
        )
    ctx.check(
        "plain output face tilts about the trunnion axis",
        lower_face_z is not None
        and upper_face_z is not None
        and abs(lower_face_z - upper_face_z) > 0.070,
        details=f"lower_z={lower_face_z}, upper_z={upper_face_z}",
    )

    return ctx.report()


object_model = build_object_model()
