from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.31, 0.34, 0.37, 1.0))
    door_paint = model.material("door_paint", rgba=(0.72, 0.74, 0.76, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.61, 0.63, 0.66, 1.0))
    body_width = 0.80
    body_height = 0.62
    body_depth = 0.18
    shell_thickness = 0.028
    rear_panel_thickness = 0.006
    frame_thickness = 0.010
    opening_width = 0.448
    opening_height = 0.362
    front_y = body_depth * 0.5
    frame_center_y = front_y - frame_thickness * 0.5
    rear_y = -body_depth * 0.5 + rear_panel_thickness * 0.5
    door_width = 0.442
    door_height = 0.356
    door_thickness = 0.016

    body = model.part("equipment_face")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=32.0,
        origin=Origin(),
    )

    body.visual(
        Box((body_width, rear_panel_thickness, body_height)),
        origin=Origin(xyz=(0.0, rear_y, 0.0)),
        material=cabinet_paint,
        name="rear_panel",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-(body_width - shell_thickness) * 0.5, 0.0, 0.0)),
        material=cabinet_paint,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(xyz=((body_width - shell_thickness) * 0.5, 0.0, 0.0)),
        material=cabinet_paint,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, (body_height - shell_thickness) * 0.5)),
        material=cabinet_paint,
        name="top_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -(body_height - shell_thickness) * 0.5)),
        material=cabinet_paint,
        name="bottom_wall",
    )

    side_frame_width = (body_width - opening_width) * 0.5
    top_frame_height = (body_height - opening_height) * 0.5
    body.visual(
        Box((side_frame_width, frame_thickness, body_height)),
        origin=Origin(xyz=(-(opening_width + side_frame_width) * 0.5, frame_center_y, 0.0)),
        material=cabinet_paint,
        name="left_stile",
    )
    body.visual(
        Box((side_frame_width, frame_thickness, body_height)),
        origin=Origin(xyz=((opening_width + side_frame_width) * 0.5, frame_center_y, 0.0)),
        material=cabinet_paint,
        name="right_stile",
    )
    body.visual(
        Box((opening_width, frame_thickness, top_frame_height)),
        origin=Origin(xyz=(0.0, frame_center_y, (opening_height + top_frame_height) * 0.5)),
        material=cabinet_paint,
        name="top_rail",
    )
    body.visual(
        Box((opening_width, frame_thickness, top_frame_height)),
        origin=Origin(xyz=(0.0, frame_center_y, -(opening_height + top_frame_height) * 0.5)),
        material=cabinet_paint,
        name="bottom_rail",
    )

    hinge_axis_x = -0.239
    hinge_axis_y = front_y + 0.011
    hinge_leaf_width = 0.022
    body.visual(
        Box((hinge_leaf_width, 0.022, door_height + 0.034)),
        origin=Origin(xyz=(hinge_axis_x - hinge_leaf_width * 0.5, front_y + 0.006, 0.0)),
        material=hinge_metal,
        name="body_hinge_leaf",
    )

    door = model.part("access_door")
    door.inertial = Inertial.from_geometry(
        Box((0.456, 0.024, 0.370)),
        mass=5.4,
        origin=Origin(xyz=(0.238, 0.0, 0.0)),
    )

    door_offset_x = -hinge_axis_x
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_offset_x, 0.0, 0.0)),
        material=door_paint,
        name="outer_skin",
    )
    door.visual(
        Box((door_width - 0.050, 0.010, door_height - 0.050)),
        origin=Origin(xyz=(door_offset_x + 0.004, -0.004, 0.0)),
        material=door_paint,
        name="inner_pan",
    )
    door.visual(
        Box((0.022, 0.012, door_height)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=hinge_metal,
        name="door_hinge_leaf",
    )
    door.visual(
        Box((0.026, 0.026, 0.090)),
        origin=Origin(xyz=(door_offset_x + door_width * 0.5 - 0.040, 0.033, 0.0)),
        material=latch_dark,
        name="latch_pull",
    )
    door.visual(
        Box((0.020, 0.020, 0.090)),
        origin=Origin(xyz=(door_offset_x + door_width * 0.5 - 0.040, 0.015, 0.0)),
        material=latch_dark,
        name="latch_mount",
    )
    door.visual(
        Box((0.012, 0.014, 0.220)),
        origin=Origin(xyz=(door_offset_x + door_width * 0.5 - 0.010, 0.001, 0.0)),
        material=latch_dark,
        name="latch_edge",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=2.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("equipment_face")
    door = object_model.get_part("access_door")
    hinge = object_model.get_articulation("door_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem="right_stile",
            negative_elem="outer_skin",
            min_gap=0.001,
            max_gap=0.008,
            name="latch edge sits just inside the right jamb",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="top_rail",
            negative_elem="outer_skin",
            min_gap=0.001,
            max_gap=0.008,
            name="top reveal stays narrow and even",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="latch_pull",
            negative_elem="right_stile",
            min_gap=0.006,
            max_gap=0.035,
            name="latch pull stands proud of the equipment face",
        )

        closed_pull = ctx.part_element_world_aabb(door, elem="latch_pull")

    with ctx.pose({hinge: 1.65}):
        open_pull = ctx.part_element_world_aabb(door, elem="latch_pull")

    closed_pull_y = None if closed_pull is None else (closed_pull[0][1] + closed_pull[1][1]) * 0.5
    open_pull_y = None if open_pull is None else (open_pull[0][1] + open_pull[1][1]) * 0.5
    ctx.check(
        "door swings outward from the equipment face",
        closed_pull_y is not None and open_pull_y is not None and open_pull_y > closed_pull_y + 0.18,
        details=f"closed_pull_y={closed_pull_y}, open_pull_y={open_pull_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
