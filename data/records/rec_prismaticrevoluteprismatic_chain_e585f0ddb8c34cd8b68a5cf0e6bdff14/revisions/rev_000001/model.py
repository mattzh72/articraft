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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_wristed_slide")

    painted_steel = Material("painted_steel", color=(0.18, 0.20, 0.22, 1.0))
    rail_steel = Material("ground_steel", color=(0.55, 0.58, 0.60, 1.0))
    carriage_blue = Material("carriage_blue", color=(0.08, 0.20, 0.45, 1.0))
    wrist_black = Material("wrist_black", color=(0.02, 0.025, 0.03, 1.0))
    sleeve_blue = Material("sleeve_blue", color=(0.10, 0.28, 0.62, 1.0))
    bright_handle = Material("bright_handle", color=(0.95, 0.45, 0.08, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.90, 0.040, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=painted_steel,
        name="backing_plate",
    )
    side_plate.visual(
        Box((0.95, 0.20, 0.040)),
        origin=Origin(xyz=(0.0, 0.060, 0.020)),
        material=painted_steel,
        name="base_foot",
    )
    side_plate.visual(
        Box((0.80, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.043, 0.38)),
        material=rail_steel,
        name="upper_rail",
    )
    side_plate.visual(
        Box((0.80, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.043, 0.26)),
        material=rail_steel,
        name="lower_rail",
    )
    for i, (bolt_x, bolt_z) in enumerate(
        ((-0.38, 0.51), (0.38, 0.51), (-0.38, 0.10), (0.38, 0.10))
    ):
        side_plate.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(bolt_x, 0.025, bolt_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wrist_black,
            name=f"wall_bolt_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.20, 0.034, 0.17)),
        origin=Origin(xyz=(0.0, 0.033, 0.0)),
        material=carriage_blue,
        name="slide_face",
    )
    carriage.visual(
        Box((0.22, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=rail_steel,
        name="upper_shoe",
    )
    carriage.visual(
        Box((0.22, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=rail_steel,
        name="lower_shoe",
    )
    for i, (cap_x, cap_z) in enumerate(
        ((-0.070, 0.055), (0.070, 0.055), (-0.070, -0.055), (0.070, -0.055))
    ):
        carriage.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(cap_x, 0.054, cap_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wrist_black,
            name=f"roller_cap_{i}",
        )
    carriage.visual(
        Box((0.10, 0.080, 0.020)),
        origin=Origin(xyz=(0.0, 0.087, 0.065)),
        material=carriage_blue,
        name="upper_clevis",
    )
    carriage.visual(
        Box((0.10, 0.080, 0.020)),
        origin=Origin(xyz=(0.0, 0.087, -0.065)),
        material=carriage_blue,
        name="lower_clevis",
    )
    carriage.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.095, 0.077)),
        material=wrist_black,
        name="upper_pin_cap",
    )
    carriage.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.095, -0.077)),
        material=wrist_black,
        name="lower_pin_cap",
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.026, length=0.110),
        origin=Origin(),
        material=wrist_black,
        name="wrist_barrel",
    )
    body.visual(
        Box((0.070, 0.040, 0.060)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=sleeve_blue,
        name="hub_web",
    )
    body.visual(
        Box((0.300, 0.058, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, 0.032)),
        material=sleeve_blue,
        name="sleeve_top",
    )
    body.visual(
        Box((0.300, 0.058, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, -0.032)),
        material=sleeve_blue,
        name="sleeve_bottom",
    )
    body.visual(
        Box((0.300, 0.012, 0.058)),
        origin=Origin(xyz=(0.205, 0.027, 0.0)),
        material=sleeve_blue,
        name="sleeve_side_0",
    )
    body.visual(
        Box((0.300, 0.012, 0.058)),
        origin=Origin(xyz=(0.205, -0.027, 0.0)),
        material=sleeve_blue,
        name="sleeve_side_1",
    )

    extension = model.part("extension")
    extension.visual(
        Box((0.340, 0.032, 0.030)),
        origin=Origin(xyz=(-0.115, 0.0, 0.0)),
        material=rail_steel,
        name="slide_rod",
    )
    extension.visual(
        Box((0.046, 0.070, 0.060)),
        origin=Origin(xyz=(0.077, 0.0, 0.0)),
        material=bright_handle,
        name="end_pad",
    )

    model.articulation(
        "plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(-0.260, 0.085, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    model.articulation(
        "carriage_to_body",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=body,
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "body_to_extension",
        ArticulationType.PRISMATIC,
        parent=body,
        child=extension,
        origin=Origin(xyz=(0.355, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    body = object_model.get_part("body")
    extension = object_model.get_part("extension")
    side_slide = object_model.get_articulation("plate_to_carriage")
    wrist = object_model.get_articulation("carriage_to_body")
    end_slide = object_model.get_articulation("body_to_extension")

    ctx.check(
        "mechanism order is prismatic revolute prismatic",
        (
            side_slide.articulation_type == ArticulationType.PRISMATIC
            and wrist.articulation_type == ArticulationType.REVOLUTE
            and end_slide.articulation_type == ArticulationType.PRISMATIC
        ),
        details=f"types={[side_slide.articulation_type, wrist.articulation_type, end_slide.articulation_type]}",
    )
    ctx.check(
        "mechanism forms a side plate carriage body extension chain",
        (
            side_slide.parent == "side_plate"
            and side_slide.child == "carriage"
            and wrist.parent == "carriage"
            and wrist.child == "body"
            and end_slide.parent == "body"
            and end_slide.child == "extension"
        ),
        details=f"chain={(side_slide.parent, side_slide.child, wrist.parent, wrist.child, end_slide.parent, end_slide.child)}",
    )

    ctx.expect_contact(
        carriage,
        side_plate,
        elem_a="upper_shoe",
        elem_b="upper_rail",
        contact_tol=0.0005,
        name="upper slide shoe rides on fixed upper rail",
    )
    ctx.expect_contact(
        carriage,
        side_plate,
        elem_a="lower_shoe",
        elem_b="lower_rail",
        contact_tol=0.0005,
        name="lower slide shoe rides on fixed lower rail",
    )
    ctx.expect_overlap(
        carriage,
        side_plate,
        axes="xz",
        elem_a="upper_shoe",
        elem_b="upper_rail",
        min_overlap=0.020,
        name="upper shoe is retained on the rail in length and height",
    )
    ctx.expect_overlap(
        carriage,
        side_plate,
        axes="xz",
        elem_a="lower_shoe",
        elem_b="lower_rail",
        min_overlap=0.020,
        name="lower shoe is retained on the rail in length and height",
    )

    ctx.expect_within(
        extension,
        body,
        axes="yz",
        inner_elem="slide_rod",
        margin=0.0,
        name="linear extension fits inside the wristed sleeve cross section",
    )
    ctx.expect_overlap(
        extension,
        body,
        axes="x",
        elem_a="slide_rod",
        min_overlap=0.20,
        name="collapsed extension remains deeply inserted in sleeve",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({side_slide: side_slide.motion_limits.upper}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="xz",
            elem_a="upper_shoe",
            elem_b="upper_rail",
            min_overlap=0.020,
            name="upper shoe remains on rail at full carriage travel",
        )
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="xz",
            elem_a="lower_shoe",
            elem_b="lower_rail",
            min_overlap=0.020,
            name="lower shoe remains on rail at full carriage travel",
        )
    ctx.check(
        "carriage prismatic joint moves along the side plate",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.30,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_sleeve_aabb = ctx.part_element_world_aabb(body, elem="sleeve_top")
    with ctx.pose({wrist: wrist.motion_limits.upper}):
        swung_sleeve_aabb = ctx.part_element_world_aabb(body, elem="sleeve_top")
    if rest_sleeve_aabb is not None and swung_sleeve_aabb is not None:
        rest_sleeve_y = (rest_sleeve_aabb[0][1] + rest_sleeve_aabb[1][1]) / 2.0
        swung_sleeve_y = (swung_sleeve_aabb[0][1] + swung_sleeve_aabb[1][1]) / 2.0
    else:
        rest_sleeve_y = None
        swung_sleeve_y = None
    ctx.check(
        "wrist revolute joint swings body away from side wall",
        rest_sleeve_y is not None and swung_sleeve_y is not None and swung_sleeve_y > rest_sleeve_y + 0.15,
        details=f"rest_y={rest_sleeve_y}, swung_y={swung_sleeve_y}",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(extension, elem="end_pad")
    with ctx.pose({end_slide: end_slide.motion_limits.upper}):
        extended_pad_aabb = ctx.part_element_world_aabb(extension, elem="end_pad")
        ctx.expect_overlap(
            extension,
            body,
            axes="x",
            elem_a="slide_rod",
            min_overlap=0.08,
            name="extension remains inserted at full travel",
        )
        ctx.expect_within(
            extension,
            body,
            axes="yz",
            inner_elem="slide_rod",
            margin=0.0,
            name="extension stays aligned in sleeve at full travel",
        )
    if rest_pad_aabb is not None and extended_pad_aabb is not None:
        rest_pad_x = (rest_pad_aabb[0][0] + rest_pad_aabb[1][0]) / 2.0
        extended_pad_x = (extended_pad_aabb[0][0] + extended_pad_aabb[1][0]) / 2.0
    else:
        rest_pad_x = None
        extended_pad_x = None
    ctx.check(
        "final prismatic joint extends the small end pad",
        rest_pad_x is not None and extended_pad_x is not None and extended_pad_x > rest_pad_x + 0.12,
        details=f"rest_x={rest_pad_x}, extended_x={extended_pad_x}",
    )

    return ctx.report()


object_model = build_object_model()
