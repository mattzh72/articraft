from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_broom_cupboard")

    # Real-world cupboard proportions: tall, narrow, and shallow enough for
    # brooms, with a framed open carcass and a nearly full-height front door.
    height = 1.90
    width = 0.54
    depth = 0.42
    panel = 0.024
    face = 0.012
    front_y = -depth / 2.0
    back_y = depth / 2.0

    paint = model.material("warm_painted_wood", rgba=(0.82, 0.79, 0.70, 1.0))
    inner_paint = model.material("shadowed_interior", rgba=(0.62, 0.60, 0.54, 1.0))
    door_paint = model.material("slightly_lighter_door", rgba=(0.88, 0.86, 0.78, 1.0))
    metal = model.material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    dark_gap = model.material("black_reveal", rgba=(0.02, 0.018, 0.015, 1.0))

    case = model.part("case")

    # Carcass panels.  The front is deliberately open so the case reads as a
    # cupboard rather than a solid block; the surrounding face frame supports
    # the door, hinges, and latch keeper.
    case.visual(
        Box((panel, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + panel / 2.0, 0.0, height / 2.0)),
        material=paint,
        name="hinge_side_panel",
    )
    case.visual(
        Box((panel, depth, height)),
        origin=Origin(xyz=(width / 2.0 - panel / 2.0, 0.0, height / 2.0)),
        material=paint,
        name="free_side_panel",
    )
    case.visual(
        Box((width, panel, height)),
        origin=Origin(xyz=(0.0, back_y - panel / 2.0, height / 2.0)),
        material=inner_paint,
        name="back_panel",
    )
    case.visual(
        Box((width, depth, panel)),
        origin=Origin(xyz=(0.0, 0.0, panel / 2.0)),
        material=paint,
        name="bottom_panel",
    )
    case.visual(
        Box((width, depth, panel)),
        origin=Origin(xyz=(0.0, 0.0, height - panel / 2.0)),
        material=paint,
        name="top_panel",
    )

    frame_y = front_y - face / 2.0
    stile_w = 0.052
    rail_h = 0.052
    case.visual(
        Box((stile_w, face, height)),
        origin=Origin(xyz=(-width / 2.0 + stile_w / 2.0, frame_y, height / 2.0)),
        material=paint,
        name="front_hinge_stile",
    )
    case.visual(
        Box((stile_w, face, height)),
        origin=Origin(xyz=(width / 2.0 - stile_w / 2.0, frame_y, height / 2.0)),
        material=paint,
        name="front_free_stile",
    )
    case.visual(
        Box((width, face, rail_h)),
        origin=Origin(xyz=(0.0, frame_y, height - rail_h / 2.0)),
        material=paint,
        name="front_top_rail",
    )
    case.visual(
        Box((width, face, rail_h)),
        origin=Origin(xyz=(0.0, frame_y, rail_h / 2.0)),
        material=paint,
        name="front_bottom_rail",
    )
    case.visual(
        Box((width - 0.08, 0.004, height - 0.14)),
        origin=Origin(xyz=(0.0, front_y - 0.002, height / 2.0)),
        material=dark_gap,
        name="door_reveal_shadow",
    )

    # Fixed halves of two side hinges: leaves are tied into the face frame and
    # cylindrical barrels sit on the common hinge line.
    hinge_x = -width / 2.0 + 0.018
    door_thickness = 0.028
    door_y = front_y - face - door_thickness / 2.0 - 0.004
    hinge_radius = 0.012
    hinge_zs = (0.55, 1.42)
    for i, zc in enumerate(hinge_zs):
        case.visual(
            Box((0.014, 0.060, 0.180)),
            origin=Origin(xyz=(hinge_x - 0.014, front_y - 0.040, zc)),
            material=metal,
            name=f"case_hinge_leaf_{i}",
        )
        case.visual(
            Cylinder(radius=hinge_radius, length=0.180),
            origin=Origin(xyz=(hinge_x, door_y - hinge_radius - 0.002, zc)),
            material=metal,
            name=f"hinge_barrel_{i}",
        )

    # Small fixed keeper/strike at the free edge.  The rotating latch on the
    # door swings in front of this plate to act as a simple hold-open/turn catch.
    case.visual(
        Box((0.012, 0.006, 0.095)),
        origin=Origin(xyz=(width / 2.0 - 0.006, frame_y - 0.004, 1.04)),
        material=metal,
        name="latch_keeper",
    )
    case.visual(
        Box((0.020, 0.004, 0.026)),
        origin=Origin(xyz=(width / 2.0 - 0.012, frame_y - 0.008, 1.04)),
        material=metal,
        name="keeper_lip",
    )

    door = model.part("door")
    door_margin_from_axis = 0.018
    door_width = 0.490
    door_height = 1.860
    door_bottom = 0.020
    door_center_x = door_margin_from_axis + door_width / 2.0
    door_center_z = door_bottom + door_height / 2.0

    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_center_x, 0.0, door_center_z)),
        material=door_paint,
        name="door_panel",
    )
    # Raised front stiles and rails make the single tall door recognizable
    # without splitting it into separate doors.
    trim_depth = 0.006
    trim_y = -door_thickness / 2.0 - trim_depth / 2.0
    stile_width = 0.045
    rail_height = 0.055
    door.visual(
        Box((stile_width, trim_depth, door_height - 0.045)),
        origin=Origin(xyz=(door_margin_from_axis + stile_width / 2.0, trim_y, door_center_z)),
        material=paint,
        name="hinge_stile_trim",
    )
    door.visual(
        Box((stile_width, trim_depth, door_height - 0.045)),
        origin=Origin(xyz=(door_margin_from_axis + door_width - stile_width / 2.0, trim_y, door_center_z)),
        material=paint,
        name="free_stile_trim",
    )
    door.visual(
        Box((door_width, trim_depth, rail_height)),
        origin=Origin(xyz=(door_center_x, trim_y, door_bottom + door_height - rail_height / 2.0)),
        material=paint,
        name="top_rail_trim",
    )
    door.visual(
        Box((door_width, trim_depth, rail_height)),
        origin=Origin(xyz=(door_center_x, trim_y, door_bottom + rail_height / 2.0)),
        material=paint,
        name="bottom_rail_trim",
    )
    door.visual(
        Box((0.010, 0.010, door_height - 0.010)),
        origin=Origin(xyz=(door_margin_from_axis + door_width + 0.004, 0.0, door_center_z)),
        material=paint,
        name="free_edge_lip",
    )
    for i, zc in enumerate(hinge_zs):
        door.visual(
            Box((0.046, 0.006, 0.150)),
            origin=Origin(xyz=(door_margin_from_axis + 0.025, trim_y - 0.001, zc)),
            material=metal,
            name=f"door_hinge_leaf_{i}",
        )

    model.articulation(
        "case_to_door",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(hinge_x, door_y, 0.0)),
        # With the closed door extending along local +X, -Z makes positive
        # motion swing the free edge outward toward the viewer/front (-Y).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    latch = model.part("latch")
    latch_pivot_x = door_margin_from_axis + door_width - 0.053
    latch_pivot_y = -door_thickness / 2.0 - 0.007
    latch_pivot_z = 1.04
    latch.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_pin",
    )
    latch.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="round_hub",
    )
    latch.visual(
        Box((0.075, 0.007, 0.026)),
        origin=Origin(xyz=(0.0375, -0.012, 0.0)),
        material=metal,
        name="catch_tab",
    )

    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(latch_pivot_x, latch_pivot_y, latch_pivot_z)),
        # The short pivot runs through the door thickness; positive motion lifts
        # the turn-catch tab upward from its horizontal locked position.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_joint = object_model.get_articulation("case_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.check(
        "door hinge is vertical and bounded",
        tuple(door_joint.axis) == (0.0, 0.0, -1.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper >= 1.5,
        details=f"axis={door_joint.axis}, limits={door_joint.motion_limits}",
    )
    ctx.check(
        "latch uses a short through-door pivot",
        tuple(latch_joint.axis) == (0.0, -1.0, 0.0)
        and latch_joint.motion_limits is not None
        and latch_joint.motion_limits.upper > 1.0,
        details=f"axis={latch_joint.axis}, limits={latch_joint.motion_limits}",
    )

    ctx.expect_gap(
        case,
        door,
        axis="y",
        positive_elem="front_free_stile",
        negative_elem="door_panel",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door sits just in front of the fixed case",
    )
    ctx.expect_overlap(
        door,
        case,
        axes="z",
        elem_a="door_panel",
        elem_b="front_free_stile",
        min_overlap=1.75,
        name="door is nearly full height",
    )
    ctx.expect_contact(
        latch,
        door,
        elem_a="pivot_pin",
        elem_b="door_panel",
        contact_tol=0.002,
        name="latch pivot is seated on the door face",
    )

    rest_latch_position = ctx.part_world_position(latch)
    rest_tab_aabb = ctx.part_element_world_aabb(latch, elem="catch_tab")
    with ctx.pose({door_joint: 1.20}):
        open_latch_position = ctx.part_world_position(latch)
    with ctx.pose({latch_joint: 1.10}):
        raised_tab_aabb = ctx.part_element_world_aabb(latch, elem="catch_tab")

    ctx.check(
        "door opens outward from the hinge side",
        rest_latch_position is not None
        and open_latch_position is not None
        and open_latch_position[1] < rest_latch_position[1] - 0.20,
        details=f"rest={rest_latch_position}, open={open_latch_position}",
    )
    ctx.check(
        "latch tab rotates upward on its own pivot",
        rest_tab_aabb is not None
        and raised_tab_aabb is not None
        and raised_tab_aabb[1][2] > rest_tab_aabb[1][2] + 0.035,
        details=f"rest_tab_aabb={rest_tab_aabb}, raised_tab_aabb={raised_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
