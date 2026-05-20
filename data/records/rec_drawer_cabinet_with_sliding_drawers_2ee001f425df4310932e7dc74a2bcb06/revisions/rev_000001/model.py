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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_key_cabinet")

    cabinet_blue = model.material("powder_coated_blue", rgba=(0.10, 0.16, 0.22, 1.0))
    inner_gray = model.material("satin_gray", rgba=(0.56, 0.58, 0.57, 1.0))
    drawer_ivory = model.material("ivory_drawer_plastic", rgba=(0.86, 0.82, 0.70, 1.0))
    label_white = model.material("paper_labels", rgba=(0.95, 0.92, 0.82, 1.0))
    dark_label = model.material("printed_numbers", rgba=(0.08, 0.08, 0.07, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    black = model.material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))

    width = 0.620
    height = 0.420
    depth = 0.090
    side_t = 0.016
    back_t = 0.012

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((width, back_t, height)),
        origin=Origin(xyz=(0.0, back_t / 2.0, height / 2.0)),
        material=cabinet_blue,
        name="back_panel",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, depth / 2.0, height / 2.0)),
        material=cabinet_blue,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, depth / 2.0, height / 2.0)),
        material=cabinet_blue,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, depth, side_t)),
        origin=Origin(xyz=(0.0, depth / 2.0, side_t / 2.0)),
        material=cabinet_blue,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((width, depth, side_t)),
        origin=Origin(xyz=(0.0, depth / 2.0, height - side_t / 2.0)),
        material=cabinet_blue,
        name="top_wall",
    )

    # Raised back screw heads make the cabinet read as wall-mounted.
    for idx, x in enumerate((-0.205, 0.205)):
        cabinet.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(x, back_t + 0.0015, height - 0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"wall_screw_{idx}",
        )

    # A full-width rail and twenty small pins support the sliding mini hook drawers.
    drawer_count = 20
    drawer_w = 0.022
    drawer_pitch = 0.028
    drawer_depth = 0.045
    drawer_h = 0.110
    drawer_y0 = 0.047
    drawer_z0 = 0.210
    pin_radius = 0.0025
    pin_z = drawer_z0 - drawer_h / 2.0 - pin_radius
    pin_len = 0.066
    rail_depth = pin_len
    rail_h = 0.006

    cabinet.visual(
        Box((width - 0.050, rail_depth, rail_h)),
        origin=Origin(xyz=(0.0, back_t + rail_depth / 2.0, pin_z - pin_radius - rail_h / 2.0)),
        material=inner_gray,
        name="pin_rail",
    )

    first_x = -drawer_pitch * (drawer_count - 1) / 2.0
    drawer_xs = [first_x + i * drawer_pitch for i in range(drawer_count)]
    for i, x in enumerate(drawer_xs):
        cabinet.visual(
            Cylinder(radius=pin_radius, length=pin_len),
            origin=Origin(xyz=(x, back_t + pin_len / 2.0, pin_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"guide_pin_{i:02d}",
        )

    # Exposed left-edge hinge hardware on the fixed cabinet frame.
    hinge_x = -width / 2.0
    door_t = 0.016
    door_y = depth + 0.004 + door_t / 2.0
    hinge_r = 0.006
    for i, (z, segment_h) in enumerate(((0.080, 0.080), (0.340, 0.080))):
        cabinet.visual(
            Box((0.012, 0.006, segment_h)),
            origin=Origin(xyz=(hinge_x + 0.006, depth + 0.003, z)),
            material=steel,
            name=f"fixed_hinge_leaf_{i}",
        )
        cabinet.visual(
            Cylinder(radius=hinge_r, length=segment_h),
            origin=Origin(xyz=(hinge_x, door_y, z)),
            material=steel,
            name=f"fixed_hinge_barrel_{i}",
        )

    door = model.part("door")
    door_panel_w = width - 0.012
    door.visual(
        Box((door_panel_w, door_t, height)),
        # The door part frame sits on the vertical hinge axis.  The panel starts
        # just to the right of the knuckle line and spans across the cabinet.
        origin=Origin(xyz=(0.012 + door_panel_w / 2.0, 0.0, height / 2.0)),
        material=cabinet_blue,
        name="door_panel",
    )
    door.visual(
        Box((door_panel_w - 0.055, 0.004, height - 0.070)),
        origin=Origin(xyz=(0.040 + (door_panel_w - 0.055) / 2.0, door_t / 2.0 + 0.002, height / 2.0)),
        material=inner_gray,
        name="raised_door_field",
    )
    door.visual(
        Box((0.014, 0.006, 0.280)),
        origin=Origin(xyz=(0.007, 0.005, height / 2.0)),
        material=steel,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=hinge_r, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
        material=steel,
        name="door_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.560, door_t / 2.0 + 0.007, height / 2.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="round_handle",
    )
    door.visual(
        Box((0.030, 0.004, 0.018)),
        origin=Origin(xyz=(0.535, door_t / 2.0 + 0.002, height / 2.0 - 0.045)),
        material=steel,
        name="latch_plate",
    )
    door_joint = model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, door_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # Twenty individual sliding mini hook drawers.  Each rests on its guide pin
    # and carries a protruding J-hook for an individual key tag.
    del door_joint  # The named joint is kept on the model; no local use here.
    for i, x in enumerate(drawer_xs):
        drawer = model.part(f"drawer_{i:02d}")
        drawer.visual(
            Box((drawer_w, drawer_depth, drawer_h)),
            origin=Origin(),
            material=drawer_ivory,
            name="drawer_body",
        )
        drawer.visual(
            Box((drawer_w * 0.92, 0.006, 0.035)),
            origin=Origin(xyz=(0.0, drawer_depth / 2.0 + 0.003, 0.010)),
            material=drawer_ivory,
            name="front_lip",
        )
        drawer.visual(
            Box((drawer_w * 0.72, 0.0012, 0.018)),
            origin=Origin(xyz=(0.0, drawer_depth / 2.0 + 0.0066, 0.028)),
            material=label_white,
            name="label_plate",
        )
        drawer.visual(
            Box((drawer_w * 0.34, 0.0014, 0.004)),
            origin=Origin(xyz=(0.0, drawer_depth / 2.0 + 0.0074, 0.028)),
            material=dark_label,
            name="label_mark",
        )
        drawer.visual(
            Box((0.004, 0.006, 0.011)),
            origin=Origin(xyz=(0.0, drawer_depth / 2.0 + 0.0065, -0.013)),
            material=steel,
            name="hook_mount",
        )
        peg_len = 0.014
        peg_y = drawer_depth / 2.0 + 0.006 + peg_len / 2.0 - 0.0005
        drawer.visual(
            Cylinder(radius=0.0020, length=peg_len),
            origin=Origin(xyz=(0.0, peg_y, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hook_peg",
        )
        drawer.visual(
            Cylinder(radius=0.0020, length=0.020),
            origin=Origin(xyz=(0.0, peg_y + peg_len / 2.0 - 0.0005, -0.010)),
            material=steel,
            name="hook_tip",
        )

        model.articulation(
            f"cabinet_to_drawer_{i:02d}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(x, drawer_y0, drawer_z0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.10, lower=0.0, upper=0.032),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("cabinet_to_door")
    drawer_joints = [
        object_model.get_articulation(f"cabinet_to_drawer_{i:02d}") for i in range(20)
    ]

    ctx.check(
        "twenty sliding hook drawers",
        len(drawer_joints) == 20,
        details=f"found {len(drawer_joints)} drawer slider articulations",
    )

    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=0.002,
        positive_elem="door_panel",
        negative_elem="side_wall_0",
        name="closed outer door clears cabinet front",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="round_handle")
    with ctx.pose({door_joint: 1.25}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="round_handle")
    ctx.check(
        "left-edge hinge swings door outward",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][1] > closed_handle_aabb[0][1] + 0.18,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    # Representative pins prove that the mini drawers are supported by their
    # individual guide pins without interpenetrating them.
    for i in (0, 9, 19):
        drawer = object_model.get_part(f"drawer_{i:02d}")
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="drawer_body",
            negative_elem=f"guide_pin_{i:02d}",
            name=f"drawer_{i:02d} rests on guide pin",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="xy",
            min_overlap=0.004,
            elem_a="drawer_body",
            elem_b=f"guide_pin_{i:02d}",
            name=f"drawer_{i:02d} remains over guide pin",
        )

    drawer_0 = object_model.get_part("drawer_00")
    rest_pos = ctx.part_world_position(drawer_0)
    with ctx.pose({"cabinet_to_door": 1.25, "cabinet_to_drawer_00": 0.032}):
        extended_pos = ctx.part_world_position(drawer_0)
        ctx.expect_overlap(
            drawer_0,
            cabinet,
            axes="y",
            min_overlap=0.012,
            elem_a="drawer_body",
            elem_b="guide_pin_00",
            name="extended drawer stays captured on guide pin",
        )
    ctx.check(
        "drawer slides outward on pins",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.025,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
