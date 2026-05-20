from __future__ import annotations

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
    model = ArticulatedObject(name="compact_shredder")

    body_color = model.material("body_color", rgba=(0.15, 0.16, 0.18, 1.0))
    trim_color = model.material("trim_color", rgba=(0.10, 0.10, 0.11, 1.0))
    bin_color = model.material("bin_color", rgba=(0.26, 0.28, 0.31, 1.0))
    steel_color = model.material("steel_color", rgba=(0.56, 0.58, 0.62, 1.0))
    button_color = model.material("button_color", rgba=(0.24, 0.60, 0.30, 1.0))

    width = 0.245
    depth = 0.185
    shell = 0.009
    wall_height = 0.315
    top_thickness = 0.010
    foot_height = 0.008
    top_surface_z = foot_height + wall_height + top_thickness

    inner_width = width - 2.0 * shell
    slot_width = 0.165
    slot_depth = 0.012
    front_strip_depth = 0.052
    rear_strip_depth = depth - front_strip_depth - slot_depth
    side_slot_margin = (inner_width - slot_width) / 2.0
    slot_center_y = -depth / 2.0 + front_strip_depth + slot_depth / 2.0
    slot_rear_edge_y = slot_center_y + slot_depth / 2.0

    body = model.part("body")

    for x in (-0.084, 0.084):
        for y in (-0.052, 0.052):
            body.visual(
                Box((0.024, 0.030, foot_height)),
                origin=Origin(xyz=(x, y, foot_height / 2.0)),
                material=trim_color,
                name=f"foot_{0 if x < 0 and y < 0 else 1 if x > 0 and y < 0 else 2 if x < 0 else 3}",
            )

    body.visual(
        Box((inner_width, depth - 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + 0.005)),
        material=trim_color,
        name="base_plate",
    )
    body.visual(
        Box((shell, depth, wall_height)),
        origin=Origin(xyz=(-width / 2.0 + shell / 2.0, 0.0, foot_height + wall_height / 2.0)),
        material=body_color,
        name="side_wall_0",
    )
    body.visual(
        Box((shell, depth, wall_height)),
        origin=Origin(xyz=(width / 2.0 - shell / 2.0, 0.0, foot_height + wall_height / 2.0)),
        material=body_color,
        name="side_wall_1",
    )
    body.visual(
        Box((inner_width, shell, wall_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - shell / 2.0, foot_height + wall_height / 2.0)),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((inner_width, shell, 0.125)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + shell / 2.0, foot_height + 0.2525)),
        material=body_color,
        name="front_head_wall",
    )
    body.visual(
        Box((inner_width, front_strip_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -depth / 2.0 + front_strip_depth / 2.0,
                foot_height + wall_height + top_thickness / 2.0,
            )
        ),
        material=body_color,
        name="top_front_strip",
    )
    body.visual(
        Box((inner_width, rear_strip_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                slot_rear_edge_y + rear_strip_depth / 2.0,
                foot_height + wall_height + top_thickness / 2.0,
            )
        ),
        material=body_color,
        name="top_rear_strip",
    )
    body.visual(
        Box((side_slot_margin, slot_depth + 0.002, top_thickness)),
        origin=Origin(
            xyz=(
                -slot_width / 2.0 - side_slot_margin / 2.0,
                slot_center_y,
                foot_height + wall_height + top_thickness / 2.0,
            )
        ),
        material=body_color,
        name="slot_margin_0",
    )
    body.visual(
        Box((side_slot_margin, slot_depth + 0.002, top_thickness)),
        origin=Origin(
            xyz=(
                slot_width / 2.0 + side_slot_margin / 2.0,
                slot_center_y,
                foot_height + wall_height + top_thickness / 2.0,
            )
        ),
        material=body_color,
        name="slot_margin_1",
    )
    body.visual(
        Box((0.032, 0.024, 0.004)),
        origin=Origin(xyz=(-0.066, 0.038, top_surface_z + 0.002)),
        material=trim_color,
        name="button_bezel",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.003),
        origin=Origin(xyz=(0.046, 0.041, top_surface_z + 0.0015)),
        material=trim_color,
        name="dial_bezel",
    )

    bin_part = model.part("bin")
    bin_width = 0.216
    bin_depth = 0.166
    bin_height = 0.163
    bin_wall = 0.004
    front_wall = 0.006
    bin_part.visual(
        Box((bin_width - 2.0 * bin_wall, bin_depth - front_wall, 0.004)),
        origin=Origin(xyz=(0.0, front_wall + (bin_depth - front_wall) / 2.0, 0.002)),
        material=bin_color,
        name="bin_floor",
    )
    bin_part.visual(
        Box((bin_wall, bin_depth, bin_height)),
        origin=Origin(xyz=(-bin_width / 2.0 + bin_wall / 2.0, bin_depth / 2.0, bin_height / 2.0)),
        material=bin_color,
        name="bin_side_0",
    )
    bin_part.visual(
        Box((bin_wall, bin_depth, bin_height)),
        origin=Origin(xyz=(bin_width / 2.0 - bin_wall / 2.0, bin_depth / 2.0, bin_height / 2.0)),
        material=bin_color,
        name="bin_side_1",
    )
    bin_part.visual(
        Box((bin_width - 2.0 * bin_wall, front_wall, bin_height)),
        origin=Origin(xyz=(0.0, front_wall / 2.0, bin_height / 2.0)),
        material=bin_color,
        name="bin_front",
    )
    bin_part.visual(
        Box((bin_width - 2.0 * bin_wall, bin_wall, bin_height - 0.006)),
        origin=Origin(
            xyz=(0.0, bin_depth - bin_wall / 2.0, (bin_height - 0.006) / 2.0),
        ),
        material=bin_color,
        name="bin_back",
    )
    bin_part.visual(
        Box((0.090, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.005, bin_height - 0.020)),
        material=trim_color,
        name="bin_pull",
    )

    flap = model.part("slot_flap")
    flap.visual(
        Box((0.160, 0.024, 0.0025)),
        origin=Origin(xyz=(0.0, -0.012, 0.00125)),
        material=trim_color,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0024, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0024), rpy=(0.0, 1.57079632679, 0.0)),
        material=trim_color,
        name="flap_hinge_barrel",
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_color,
        name="dial_skirt",
    )
    selector_dial.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=body_color,
        name="dial_cap",
    )
    selector_dial.visual(
        Box((0.003, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.010, 0.017)),
        material=steel_color,
        name="dial_pointer",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.024, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_color,
        name="button_cap",
    )

    for index, y in enumerate((-0.043, -0.024)):
        drum = model.part(f"drum_{index}")
        drum.visual(
            Cylinder(radius=0.009, length=0.172),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=steel_color,
            name="drum_body",
        )
        drum.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(xyz=(-0.083, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=steel_color,
            name="endcap_0",
        )
        drum.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=steel_color,
            name="endcap_1",
        )

        model.articulation(
            f"body_to_drum_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum,
            origin=Origin(xyz=(0.0, y, 0.314)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=30.0),
        )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, -0.089, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.22, lower=0.0, upper=0.082),
    )
    model.articulation(
        "body_to_slot_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, slot_rear_edge_y, top_surface_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(0.046, 0.041, top_surface_z + 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(-0.066, 0.038, top_surface_z + 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    flap = object_model.get_part("slot_flap")
    start_button = object_model.get_part("start_button")

    bin_joint = object_model.get_articulation("body_to_bin")
    flap_joint = object_model.get_articulation("body_to_slot_flap")
    button_joint = object_model.get_articulation("body_to_start_button")
    dial_joint = object_model.get_articulation("body_to_selector_dial")
    drum_joint_0 = object_model.get_articulation("body_to_drum_0")
    drum_joint_1 = object_model.get_articulation("body_to_drum_1")

    ctx.expect_within(
        bin_part,
        body,
        axes="xz",
        margin=0.010,
        name="bin stays nested within the body opening at rest",
    )
    ctx.expect_overlap(
        bin_part,
        body,
        axes="x",
        min_overlap=0.180,
        name="bin spans the body opening width",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="flap_panel",
        negative_elem="top_front_strip",
        name="slot flap sits just above the top panel",
    )

    rest_bin_pos = ctx.part_world_position(bin_part)
    rest_button_pos = ctx.part_world_position(start_button)
    rest_flap_aabb = ctx.part_world_aabb(flap)

    bin_upper = bin_joint.motion_limits.upper if bin_joint.motion_limits is not None else None
    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    flap_upper = flap_joint.motion_limits.upper if flap_joint.motion_limits is not None else None

    extended_bin_pos = None
    pressed_button_pos = None
    open_flap_aabb = None

    if bin_upper is not None:
        with ctx.pose({bin_joint: bin_upper}):
            ctx.expect_within(
                bin_part,
                body,
                axes="x",
                margin=0.010,
                name="bin remains guided side to side at full extension",
            )
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                min_overlap=0.080,
                name="bin keeps retained insertion at full extension",
            )
            extended_bin_pos = ctx.part_world_position(bin_part)

    if button_upper is not None:
        with ctx.pose({button_joint: button_upper}):
            pressed_button_pos = ctx.part_world_position(start_button)

    if flap_upper is not None:
        with ctx.pose({flap_joint: flap_upper}):
            open_flap_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "bin pulls forward along the shredder depth axis",
        rest_bin_pos is not None
        and extended_bin_pos is not None
        and extended_bin_pos[1] < rest_bin_pos[1] - 0.060,
        details=f"rest={rest_bin_pos}, extended={extended_bin_pos}",
    )
    ctx.check(
        "start button depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )
    ctx.check(
        "slot flap opens upward above the housing",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > rest_flap_aabb[1][2] + 0.015,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )
    ctx.check(
        "dial and cutter drums use continuous rotation joints",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and drum_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and drum_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and drum_joint_0.motion_limits is not None
        and drum_joint_1.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None
        and drum_joint_0.motion_limits.lower is None
        and drum_joint_0.motion_limits.upper is None
        and drum_joint_1.motion_limits.lower is None
        and drum_joint_1.motion_limits.upper is None,
        details=(
            f"dial={dial_joint.articulation_type}, "
            f"drum_0={drum_joint_0.articulation_type}, drum_1={drum_joint_1.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
