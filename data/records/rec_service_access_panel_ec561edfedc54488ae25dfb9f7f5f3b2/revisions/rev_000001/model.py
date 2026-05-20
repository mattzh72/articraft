from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    body_color = model.material("body_paint", rgba=(0.26, 0.28, 0.30, 1.0))
    frame_color = model.material("frame_paint", rgba=(0.33, 0.35, 0.37, 1.0))
    door_color = model.material("door_paint", rgba=(0.74, 0.76, 0.78, 1.0))
    hardware_color = model.material("hardware", rgba=(0.12, 0.13, 0.14, 1.0))

    outer_width = 0.24
    outer_height = 0.50
    outer_depth = 0.16
    shell_thickness = 0.014
    back_thickness = 0.006
    front_frame_depth = 0.020

    hinge_jamb_width = 0.040
    latch_jamb_width = 0.028
    top_rail_height = 0.080
    bottom_rail_height = 0.080
    reveal_thickness = 0.014

    opening_left = -outer_width / 2.0 + hinge_jamb_width
    opening_right = outer_width / 2.0 - latch_jamb_width
    opening_bottom = -outer_height / 2.0 + bottom_rail_height
    opening_top = outer_height / 2.0 - top_rail_height
    opening_width = opening_right - opening_left
    opening_height = opening_top - opening_bottom

    body = model.part("body")
    body.visual(
        Box((outer_width, back_thickness, outer_height)),
        origin=Origin(xyz=(0.0, -outer_depth / 2.0 + back_thickness / 2.0, 0.0)),
        material=body_color,
        name="back_panel",
    )
    body.visual(
        Box((shell_thickness, outer_depth - back_thickness, outer_height)),
        origin=Origin(xyz=(-outer_width / 2.0 + shell_thickness / 2.0, back_thickness / 2.0, 0.0)),
        material=body_color,
        name="left_shell",
    )
    body.visual(
        Box((shell_thickness, outer_depth - back_thickness, outer_height)),
        origin=Origin(xyz=(outer_width / 2.0 - shell_thickness / 2.0, back_thickness / 2.0, 0.0)),
        material=body_color,
        name="right_shell",
    )
    body.visual(
        Box((outer_width - 2.0 * shell_thickness, outer_depth - back_thickness, shell_thickness)),
        origin=Origin(xyz=(0.0, back_thickness / 2.0, outer_height / 2.0 - shell_thickness / 2.0)),
        material=body_color,
        name="top_shell",
    )
    body.visual(
        Box((outer_width - 2.0 * shell_thickness, outer_depth - back_thickness, shell_thickness)),
        origin=Origin(xyz=(0.0, back_thickness / 2.0, -outer_height / 2.0 + shell_thickness / 2.0)),
        material=body_color,
        name="bottom_shell",
    )

    front_frame_y = outer_depth / 2.0 - front_frame_depth / 2.0
    body.visual(
        Box((hinge_jamb_width, front_frame_depth, opening_height)),
        origin=Origin(
            xyz=(
                -outer_width / 2.0 + hinge_jamb_width / 2.0,
                front_frame_y,
                (opening_top + opening_bottom) / 2.0,
            )
        ),
        material=frame_color,
        name="hinge_jamb",
    )
    body.visual(
        Box((latch_jamb_width, front_frame_depth, opening_height)),
        origin=Origin(
            xyz=(
                outer_width / 2.0 - latch_jamb_width / 2.0,
                front_frame_y,
                (opening_top + opening_bottom) / 2.0,
            )
        ),
        material=frame_color,
        name="latch_jamb",
    )
    body.visual(
        Box((opening_width, front_frame_depth, top_rail_height)),
        origin=Origin(
            xyz=(
                (opening_left + opening_right) / 2.0,
                front_frame_y,
                outer_height / 2.0 - top_rail_height / 2.0,
            )
        ),
        material=frame_color,
        name="top_frame",
    )
    body.visual(
        Box((opening_width, front_frame_depth, bottom_rail_height)),
        origin=Origin(
            xyz=(
                (opening_left + opening_right) / 2.0,
                front_frame_y,
                -outer_height / 2.0 + bottom_rail_height / 2.0,
            )
        ),
        material=frame_color,
        name="bottom_frame",
    )

    reveal_depth = outer_depth - back_thickness
    reveal_y = back_thickness / 2.0
    body.visual(
        Box((reveal_thickness, reveal_depth, opening_height)),
        origin=Origin(
            xyz=((opening_left - reveal_thickness / 2.0), reveal_y, (opening_top + opening_bottom) / 2.0)
        ),
        material=frame_color,
        name="hinge_return",
    )
    body.visual(
        Box((reveal_thickness, reveal_depth, opening_height)),
        origin=Origin(
            xyz=((opening_right + reveal_thickness / 2.0), reveal_y, (opening_top + opening_bottom) / 2.0)
        ),
        material=frame_color,
        name="latch_return",
    )
    body.visual(
        Box((opening_width, reveal_depth, reveal_thickness)),
        origin=Origin(
            xyz=((opening_left + opening_right) / 2.0, reveal_y, opening_top + reveal_thickness / 2.0)
        ),
        material=frame_color,
        name="top_return",
    )
    body.visual(
        Box((opening_width, reveal_depth, reveal_thickness)),
        origin=Origin(
            xyz=((opening_left + opening_right) / 2.0, reveal_y, opening_bottom - reveal_thickness / 2.0)
        ),
        material=frame_color,
        name="bottom_return",
    )

    hinge_depth_beam_width = 0.024
    body.visual(
        Box((hinge_depth_beam_width, reveal_depth, opening_height)),
        origin=Origin(
            xyz=(
                opening_left - reveal_thickness - hinge_depth_beam_width / 2.0,
                reveal_y,
                (opening_top + opening_bottom) / 2.0,
            )
        ),
        material=frame_color,
        name="hinge_depth_beam",
    )
    body.inertial = Inertial.from_geometry(Box((outer_width, outer_depth, outer_height)), mass=18.0)

    door = model.part("door")
    door_gap = 0.002
    door_width = opening_width - 2.0 * door_gap
    door_height = opening_height - 2.0 * door_gap
    door_thickness = 0.016
    door_front_gap = 0.001
    hinge_x = opening_left + door_gap
    door_center_y = outer_depth / 2.0 + door_front_gap + door_thickness / 2.0

    body.visual(
        Box((0.008, 0.018, door_height - 0.016)),
        origin=Origin(xyz=(hinge_x - 0.004, door_center_y, 0.0)),
        material=hardware_color,
        name="hinge_leaf",
    )

    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.0)),
        material=door_color,
        name="door_panel",
    )
    door.visual(
        Box((door_width - 0.024, 0.008, door_height - 0.040)),
        origin=Origin(xyz=(door_width / 2.0, -0.004, 0.0)),
        material=frame_color,
        name="rear_stiffener",
    )
    door.visual(
        Box((0.010, 0.012, 0.100)),
        origin=Origin(xyz=(door_width - 0.012, 0.010, 0.0)),
        material=hardware_color,
        name="latch_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=2.4,
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, door_center_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    closed_aabb = ctx.part_world_aabb(door)
    ctx.expect_contact(
        door,
        body,
        elem_a="door_panel",
        elem_b="hinge_leaf",
        name="door is mounted to the body along the hinge side",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a="door_panel",
        elem_b="hinge_return",
        min_overlap=0.25,
        name="door spans the framed opening height",
    )

    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door swings outward from the face",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.08,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
