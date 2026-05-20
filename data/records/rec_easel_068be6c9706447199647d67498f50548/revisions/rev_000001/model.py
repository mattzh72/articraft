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


def _bar_between(part, name, start, end, size, material):
    """Add a rectangular wooden rail whose long local axis runs from start to end."""
    sx, sy = size
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    part.visual(
        Box((sx, sy, length)),
        origin=Origin(xyz=center, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_a_frame_easel")

    wood = model.material("oiled_beech_wood", rgba=(0.72, 0.48, 0.25, 1.0))
    end_grain = model.material("darker_end_grain", rgba=(0.50, 0.31, 0.15, 1.0))
    dark_slot = model.material("shadowed_slot", rgba=(0.035, 0.028, 0.020, 1.0))
    metal = model.material("brushed_dark_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    # The root is the rigid front A-frame: two splayed front legs, cross rails,
    # a central slotted mast, and the fixed half of the top hinge.
    front_frame = model.part("front_frame")
    _bar_between(
        front_frame,
        "front_leg_0",
        (-0.38, 0.0, 0.055),
        (-0.10, 0.0, 1.585),
        (0.046, 0.040),
        wood,
    )
    _bar_between(
        front_frame,
        "front_leg_1",
        (0.38, 0.0, 0.055),
        (0.10, 0.0, 1.585),
        (0.046, 0.040),
        wood,
    )
    front_frame.visual(
        Box((0.72, 0.045, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=wood,
        name="lower_cross_rail",
    )
    front_frame.visual(
        Box((0.56, 0.040, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=wood,
        name="middle_cross_rail",
    )
    front_frame.visual(
        Box((0.31, 0.045, 0.046)),
        origin=Origin(xyz=(0.0, 0.035, 1.555)),
        material=wood,
        name="top_cross_block",
    )
    front_frame.visual(
        Box((0.060, 0.036, 1.465)),
        origin=Origin(xyz=(0.0, 0.001, 0.855)),
        material=wood,
        name="slotted_mast",
    )
    front_frame.visual(
        Box((0.020, 0.002, 1.105)),
        origin=Origin(xyz=(0.0, 0.0190, 0.820)),
        material=dark_slot,
        name="vertical_slot",
    )
    front_frame.visual(
        Box((0.006, 0.003, 1.115)),
        origin=Origin(xyz=(-0.020, 0.0198, 0.820)),
        material=metal,
        name="slot_edge_0",
    )
    front_frame.visual(
        Box((0.006, 0.003, 1.115)),
        origin=Origin(xyz=(0.020, 0.0198, 0.820)),
        material=metal,
        name="slot_edge_1",
    )
    # Small rubber feet keep the wood visibly off the floor.
    front_frame.visual(
        Box((0.115, 0.074, 0.050)),
        origin=Origin(xyz=(-0.385, 0.0, 0.030)),
        material=rubber,
        name="front_foot_0",
    )
    front_frame.visual(
        Box((0.115, 0.074, 0.050)),
        origin=Origin(xyz=(0.385, 0.0, 0.030)),
        material=rubber,
        name="front_foot_1",
    )
    # Hinge brackets and pin, with a gap in the middle for the rear leg barrel.
    front_frame.visual(
        Box((0.165, 0.052, 0.060)),
        origin=Origin(xyz=(-0.132, 0.0, 1.605)),
        material=metal,
        name="hinge_cheek_0",
    )
    front_frame.visual(
        Box((0.165, 0.052, 0.060)),
        origin=Origin(xyz=(0.132, 0.0, 1.605)),
        material=metal,
        name="hinge_cheek_1",
    )
    front_frame.visual(
        Cylinder(radius=0.027, length=0.165),
        origin=Origin(xyz=(-0.132, -0.010, 1.650), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="front_hinge_barrel_0",
    )
    front_frame.visual(
        Cylinder(radius=0.027, length=0.165),
        origin=Origin(xyz=(0.132, -0.010, 1.650), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="front_hinge_barrel_1",
    )
    front_frame.visual(
        Cylinder(radius=0.008, length=0.470),
        origin=Origin(xyz=(0.0, -0.010, 1.650), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )

    # Rear prop leg is a separate link about the same top pin.  At q=0 the easel
    # is standing open; positive motion folds the leg forward for storage.
    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.0265, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rear_hinge_barrel",
    )
    _bar_between(
        rear_leg,
        "rear_yoke_lug",
        (0.0, -0.018, -0.018),
        (0.0, -0.085, -0.065),
        (0.044, 0.036),
        metal,
    )
    _bar_between(
        rear_leg,
        "rear_prop",
        (0.0, -0.080, -0.060),
        (0.0, -0.650, -1.555),
        (0.050, 0.042),
        wood,
    )
    rear_leg.visual(
        Box((0.115, 0.085, 0.028)),
        origin=Origin(xyz=(0.0, -0.662, -1.568), rpy=(0.0, 0.0, 0.0)),
        material=rubber,
        name="rear_foot",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.010, 1.650)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.55),
    )

    # Canvas tray that rides on the central slot.  The child origin is the
    # sliding carriage bolt line; the shelf projects forward to support a canvas.
    canvas_ledge = model.part("canvas_ledge")
    canvas_ledge.visual(
        Box((0.086, 0.012, 0.180)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=metal,
        name="ledge_slider_plate",
    )
    canvas_ledge.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.041, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="ledge_lock_knob",
    )
    canvas_ledge.visual(
        Box((0.760, 0.112, 0.046)),
        origin=Origin(xyz=(0.0, 0.080, -0.066)),
        material=wood,
        name="support_shelf",
    )
    canvas_ledge.visual(
        Box((0.760, 0.020, 0.076)),
        origin=Origin(xyz=(0.0, 0.131, -0.016)),
        material=wood,
        name="front_lip",
    )
    canvas_ledge.visual(
        Box((0.115, 0.055, 0.022)),
        origin=Origin(xyz=(-0.155, 0.074, -0.085), rpy=(0.55, 0.0, 0.0)),
        material=end_grain,
        name="ledge_brace_0",
    )
    canvas_ledge.visual(
        Box((0.115, 0.055, 0.022)),
        origin=Origin(xyz=(0.155, 0.074, -0.085), rpy=(0.55, 0.0, 0.0)),
        material=end_grain,
        name="ledge_brace_1",
    )
    model.articulation(
        "ledge_slide",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=canvas_ledge,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.22, lower=0.0, upper=0.48),
    )

    # Upper canvas clamp: a smaller carriage on the same mast with a broad padded
    # bar that bears on the top edge of a canvas.
    top_clamp = model.part("top_clamp")
    top_clamp.visual(
        Box((0.080, 0.012, 0.150)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=metal,
        name="clamp_slider_plate",
    )
    top_clamp.visual(
        Box((0.090, 0.020, 0.046)),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material=metal,
        name="clamp_neck",
    )
    top_clamp.visual(
        Box((0.460, 0.046, 0.044)),
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        material=wood,
        name="clamp_bar",
    )
    top_clamp.visual(
        Box((0.390, 0.026, 0.035)),
        origin=Origin(xyz=(0.0, 0.096, -0.0375)),
        material=rubber,
        name="clamp_pad",
    )
    top_clamp.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.041, 0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="clamp_lock_knob",
    )
    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=top_clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_leg = object_model.get_part("rear_leg")
    canvas_ledge = object_model.get_part("canvas_ledge")
    top_clamp = object_model.get_part("top_clamp")
    top_hinge = object_model.get_articulation("top_hinge")
    ledge_slide = object_model.get_articulation("ledge_slide")
    clamp_slide = object_model.get_articulation("clamp_slide")

    ctx.allow_overlap(
        front_frame,
        rear_leg,
        elem_a="hinge_pin",
        elem_b="rear_hinge_barrel",
        reason="A real top hinge pin is intentionally captured inside the rear leg barrel.",
    )
    ctx.expect_within(
        front_frame,
        rear_leg,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="rear_hinge_barrel",
        margin=0.0005,
        name="hinge pin is centered inside rear barrel",
    )
    ctx.expect_overlap(
        front_frame,
        rear_leg,
        axes="x",
        elem_a="hinge_pin",
        elem_b="rear_hinge_barrel",
        min_overlap=0.080,
        name="hinge pin spans rear barrel",
    )
    ctx.expect_gap(
        canvas_ledge,
        front_frame,
        axis="y",
        positive_elem="ledge_slider_plate",
        negative_elem="vertical_slot",
        max_penetration=0.000001,
        max_gap=0.002,
        name="ledge carriage rides just in front of slot",
    )
    ctx.expect_overlap(
        canvas_ledge,
        front_frame,
        axes="z",
        elem_a="ledge_slider_plate",
        elem_b="vertical_slot",
        min_overlap=0.150,
        name="ledge carriage is retained in vertical slot",
    )
    with ctx.pose({ledge_slide: 0.48}):
        ctx.expect_overlap(
            canvas_ledge,
            front_frame,
            axes="z",
            elem_a="ledge_slider_plate",
            elem_b="vertical_slot",
            min_overlap=0.150,
            name="raised ledge remains in slot",
        )
    with ctx.pose({top_hinge: 0.45}):
        folded_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    rest_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    ctx.check(
        "rear leg folds forward about top hinge",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][1] > rest_aabb[0][1] + 0.20,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )
    rest_clamp = ctx.part_world_position(top_clamp)
    with ctx.pose({clamp_slide: 0.20}):
        raised_clamp = ctx.part_world_position(top_clamp)
    ctx.check(
        "top clamp slides upward on mast",
        rest_clamp is not None
        and raised_clamp is not None
        and raised_clamp[2] > rest_clamp[2] + 0.15,
        details=f"rest={rest_clamp}, raised={raised_clamp}",
    )

    return ctx.report()


object_model = build_object_model()
