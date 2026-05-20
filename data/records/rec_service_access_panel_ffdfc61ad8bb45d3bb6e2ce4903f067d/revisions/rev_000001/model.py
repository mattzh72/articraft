from __future__ import annotations

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
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    powder_coat = Material("powder_coated_body", color=(0.42, 0.46, 0.48, 1.0))
    dark_trim = Material("dark_recessed_frame", color=(0.08, 0.09, 0.09, 1.0))
    service_blue = Material("painted_service_door", color=(0.12, 0.28, 0.42, 1.0))
    steel = Material("brushed_hinge_steel", color=(0.70, 0.72, 0.70, 1.0))
    latch_black = Material("black_latch_hardware", color=(0.015, 0.015, 0.014, 1.0))

    body_width = 1.35
    body_depth = 0.18
    body_height = 0.50
    opening_width = 1.06
    opening_height = 0.36
    opening_center_z = 0.25
    frame_outer_width = 1.20
    frame_outer_height = 0.45
    frame_depth = 0.026
    frame_center_y = -body_depth / 2.0 - frame_depth / 2.0 + 0.002

    face_body_shape = (
        cq.Workplane("XY")
        .box(body_width, body_depth, body_height)
        .translate((0.0, 0.0, body_height / 2.0))
    )
    opening_cutter = (
        cq.Workplane("XY")
        .box(opening_width, body_depth + 0.08, opening_height)
        .translate((0.0, 0.0, opening_center_z))
    )
    face_body_shape = face_body_shape.cut(opening_cutter)

    face = model.part("equipment_face")
    face.visual(
        mesh_from_cadquery(face_body_shape, "face_body"),
        material=powder_coat,
        name="face_body",
    )

    side_frame_width = (frame_outer_width - opening_width) / 2.0
    top_frame_height = (frame_outer_height - opening_height) / 2.0
    frame_center_z = opening_center_z
    face.visual(
        Box((side_frame_width, frame_depth, frame_outer_height)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 + side_frame_width / 2.0),
                frame_center_y,
                frame_center_z,
            )
        ),
        material=dark_trim,
        name="front_frame_side_0",
    )
    face.visual(
        Box((side_frame_width, frame_depth, frame_outer_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + side_frame_width / 2.0,
                frame_center_y,
                frame_center_z,
            )
        ),
        material=dark_trim,
        name="front_frame_side_1",
    )
    face.visual(
        Box((frame_outer_width, frame_depth, top_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                opening_center_z + opening_height / 2.0 + top_frame_height / 2.0,
            )
        ),
        material=dark_trim,
        name="front_frame_top",
    )
    face.visual(
        Box((frame_outer_width, frame_depth, top_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                opening_center_z - opening_height / 2.0 - top_frame_height / 2.0,
            )
        ),
        material=dark_trim,
        name="front_frame_bottom",
    )

    hinge_x = -opening_width / 2.0 + 0.015
    hinge_y = -0.128
    hinge_z = opening_center_z
    knuckle_radius = 0.012
    knuckle_length = 0.050
    hinge_segments = (-0.135, -0.075, -0.015, 0.045, 0.105)

    for idx in (0, 2, 4):
        segment_z = hinge_z + hinge_segments[idx]
        face.visual(
            Cylinder(radius=knuckle_radius, length=knuckle_length),
            origin=Origin(xyz=(hinge_x, hinge_y, segment_z)),
            material=steel,
            name=f"fixed_knuckle_{idx}",
        )
        face.visual(
            Box((0.060, 0.008, knuckle_length)),
            origin=Origin(xyz=(hinge_x - 0.030, -0.115, segment_z)),
            material=steel,
            name=f"fixed_leaf_{idx}",
        )

    face.visual(
        Cylinder(radius=0.0045, length=0.315),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z - 0.015)),
        material=steel,
        name="hinge_pin",
    )

    face.visual(
        Box((0.032, 0.010, 0.150)),
        origin=Origin(xyz=(opening_width / 2.0 + 0.017, -0.116, opening_center_z)),
        material=steel,
        name="strike_plate",
    )

    door_width = 1.00
    door_height = 0.30
    door_thickness = 0.020
    door_left_gap = 0.015
    door_panel_shape = (
        cq.Workplane("XY")
        .box(door_width, door_thickness, door_height)
        .translate((door_left_gap + door_width / 2.0, 0.0, 0.0))
        .edges()
        .fillet(0.006)
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(door_panel_shape, "door_panel"),
        material=service_blue,
        name="door_panel",
    )
    door.visual(
        Box((door_width - 0.10, 0.006, 0.020)),
        origin=Origin(xyz=(door_left_gap + door_width / 2.0, -0.013, door_height / 2.0 - 0.035)),
        material=dark_trim,
        name="upper_reveal",
    )
    door.visual(
        Box((door_width - 0.10, 0.006, 0.020)),
        origin=Origin(xyz=(door_left_gap + door_width / 2.0, -0.013, -door_height / 2.0 + 0.035)),
        material=dark_trim,
        name="lower_reveal",
    )
    door.visual(
        Box((0.080, 0.012, 0.120)),
        origin=Origin(xyz=(door_left_gap + door_width - 0.060, -0.016, 0.0)),
        material=latch_black,
        name="latch_plate",
    )
    door.visual(
        Box((0.024, 0.018, 0.060)),
        origin=Origin(xyz=(door_left_gap + door_width - 0.025, -0.020, 0.0)),
        material=latch_black,
        name="latch_pull",
    )

    for idx in (1, 3):
        segment_z = hinge_segments[idx]
        door.visual(
            Cylinder(radius=knuckle_radius, length=knuckle_length),
            origin=Origin(xyz=(0.0, 0.0, segment_z)),
            material=steel,
            name=f"door_knuckle_{idx}",
        )
        door.visual(
            Box((0.080, 0.006, knuckle_length)),
            origin=Origin(xyz=(0.040, -0.013, segment_z)),
            material=steel,
            name=f"door_leaf_{idx}",
        )

    model.articulation(
        "face_to_door",
        ArticulationType.REVOLUTE,
        parent=face,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("face_to_door")

    for knuckle_name in ("door_knuckle_1", "door_knuckle_3"):
        ctx.allow_overlap(
            face,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The visible hinge pin is intentionally captured through the moving door knuckle.",
        )
        ctx.expect_within(
            face,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.001,
            name=f"hinge pin is centered in {knuckle_name}",
        )
        ctx.expect_overlap(
            face,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.045,
            name=f"hinge pin passes through {knuckle_name}",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            face,
            door,
            axis="y",
            positive_elem="front_frame_top",
            negative_elem="door_panel",
            min_gap=0.002,
            max_gap=0.012,
            name="closed door sits just proud of the surrounding frame",
        )
        ctx.expect_within(
            door,
            face,
            axes="xz",
            inner_elem="door_panel",
            outer_elem="face_body",
            margin=0.0,
            name="service door is contained within the equipment face outline",
        )

        body_aabb = ctx.part_element_world_aabb(face, elem="face_body")
        door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        if body_aabb is not None and door_aabb is not None:
            body_width = body_aabb[1][0] - body_aabb[0][0]
            door_width = door_aabb[1][0] - door_aabb[0][0]
            closed_min_y = door_aabb[0][1]
        else:
            body_width = door_width = closed_min_y = None
        ctx.check(
            "moving panel spans most of the low wide body",
            body_width is not None and door_width is not None and door_width >= 0.70 * body_width,
            details=f"body_width={body_width}, door_width={door_width}",
        )

    with ctx.pose({hinge: 1.35}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        opened_min_y = opened_aabb[0][1] if opened_aabb is not None else None
    ctx.check(
        "positive hinge motion swings the latch edge outward",
        closed_min_y is not None and opened_min_y is not None and opened_min_y < closed_min_y - 0.35,
        details=f"closed_min_y={closed_min_y}, opened_min_y={opened_min_y}",
    )

    return ctx.report()


object_model = build_object_model()
