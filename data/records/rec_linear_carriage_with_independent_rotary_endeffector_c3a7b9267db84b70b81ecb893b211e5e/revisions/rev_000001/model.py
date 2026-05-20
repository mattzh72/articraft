from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_slide_rotary_head")

    cast_iron = Material("painted_cast_iron", color=(0.16, 0.19, 0.22, 1.0))
    carriage_paint = Material("carriage_blue_gray", color=(0.22, 0.30, 0.36, 1.0))
    dark_steel = Material("dark_blued_steel", color=(0.05, 0.055, 0.06, 1.0))
    bright_steel = Material("ground_steel", color=(0.72, 0.74, 0.72, 1.0))
    face_steel = Material("plain_faceplate_steel", color=(0.50, 0.51, 0.49, 1.0))
    marker_black = Material("black_index_mark", color=(0.01, 0.01, 0.01, 1.0))

    guide = model.part("guide")
    guide.visual(
        Box((1.05, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_plate",
    )
    guide.visual(
        Box((0.94, 0.150, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=cast_iron,
        name="center_way",
    )
    for rail_name, y in (("linear_rail_0", -0.052), ("linear_rail_1", 0.052)):
        guide.visual(
            Box((0.92, 0.030, 0.019)),
            origin=Origin(xyz=(0.0, y, 0.0985)),
            material=bright_steel,
            name=rail_name,
        )
    for idx, x in enumerate((-0.485, 0.485)):
        guide.visual(
            Box((0.045, 0.300, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.100)),
            material=cast_iron,
            name=f"end_stop_{idx}",
        )
        guide.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x, -0.095, 0.164)),
            material=dark_steel,
            name=f"stop_screw_{idx}_0",
        )
        guide.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x, 0.095, 0.164)),
            material=dark_steel,
            name=f"stop_screw_{idx}_1",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.240, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_paint,
        name="carriage_block",
    )
    for pad_name, y in (("wear_pad_0", -0.052), ("wear_pad_1", 0.052)):
        carriage.visual(
            Box((0.185, 0.024, 0.013)),
            origin=Origin(xyz=(0.0, y, -0.050)),
            material=bright_steel,
            name=pad_name,
        )
    for idx, y in enumerate((-0.102, 0.102)):
        carriage.visual(
            Box((0.205, 0.034, 0.058)),
            origin=Origin(xyz=(0.0, y, -0.057)),
            material=carriage_paint,
            name=f"side_gib_{idx}",
        )
    carriage.visual(
        Box((0.178, 0.044, 0.178)),
        origin=Origin(xyz=(0.0, -0.142, 0.050)),
        material=carriage_paint,
        name="bearing_plate",
    )
    carriage.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.082, tube=0.012, radial_segments=24, tubular_segments=48).rotate_x(
                math.pi / 2.0
            ),
            "bearing_ring",
        ),
        origin=Origin(xyz=(0.0, -0.166, 0.060)),
        material=bright_steel,
        name="bearing_ring",
    )
    for idx, x in enumerate((-0.075, 0.075)):
        carriage.visual(
            Box((0.026, 0.065, 0.105)),
            origin=Origin(xyz=(x, -0.118, 0.018)),
            material=carriage_paint,
            name=f"bearing_gusset_{idx}",
        )
    for ix, x in enumerate((-0.070, 0.070)):
        for iy, y in enumerate((-0.070, 0.070)):
            carriage.visual(
                Cylinder(radius=0.010, length=0.007),
                origin=Origin(xyz=(x, y, 0.048)),
                material=dark_steel,
                name=f"cap_screw_{ix}_{iy}",
            )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.065, length=0.130),
        origin=Origin(xyz=(0.0, -0.065, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=0.085, length=0.026),
        origin=Origin(xyz=(0.0, -0.143, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=face_steel,
        name="faceplate",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.017),
        origin=Origin(xyz=(0.0, -0.164, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="center_pilot",
    )
    head.visual(
        Box((0.012, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, -0.1585, 0.055)),
        material=marker_black,
        name="index_mark",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(-0.250, 0.0, 0.1645)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=0.500),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, -0.160, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")
    slide = object_model.get_articulation("guide_to_carriage")
    rotary = object_model.get_articulation("carriage_to_head")

    ctx.check(
        "linear slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "head joint is independent revolute",
        rotary.articulation_type == ArticulationType.REVOLUTE and tuple(rotary.axis) == (0.0, -1.0, 0.0),
        details=f"type={rotary.articulation_type}, axis={rotary.axis}",
    )

    ctx.expect_contact(
        carriage,
        guide,
        elem_a="wear_pad_0",
        elem_b="linear_rail_0",
        contact_tol=0.0002,
        name="wear pad rides on fixed rail",
    )
    ctx.expect_contact(
        head,
        carriage,
        elem_a="head_body",
        elem_b="bearing_plate",
        contact_tol=0.0005,
        name="rotary head is supported at bearing plate",
    )
    ctx.expect_within(
        head,
        carriage,
        axes="xz",
        inner_elem="head_body",
        outer_elem="bearing_ring",
        margin=0.001,
        name="head body passes through bearing ring opening",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="wear_pad_0",
            elem_b="linear_rail_0",
            contact_tol=0.0002,
            name="wear pad rides on rail at slide extension",
        )

    ctx.check(
        "carriage translates along guide",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.49
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    marker_rest = ctx.part_element_world_aabb(head, elem="index_mark")
    with ctx.pose({rotary: math.pi / 2.0}):
        marker_quarter_turn = ctx.part_element_world_aabb(head, elem="index_mark")

    if marker_rest is not None and marker_quarter_turn is not None:
        rest_center = tuple((marker_rest[0][i] + marker_rest[1][i]) * 0.5 for i in range(3))
        turned_center = tuple(
            (marker_quarter_turn[0][i] + marker_quarter_turn[1][i]) * 0.5 for i in range(3)
        )
    else:
        rest_center = None
        turned_center = None
    ctx.check(
        "faceplate rotates about its own axis",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] - rest_center[0]) > 0.030
        and abs(turned_center[2] - rest_center[2]) > 0.030,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
