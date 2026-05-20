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
    model = ArticulatedObject(name="utility_vehicle_glove_compartment")

    dash_plastic = Material("charcoal_dashboard_plastic", color=(0.10, 0.11, 0.11, 1.0))
    bin_plastic = Material("black_recessed_bin_plastic", color=(0.018, 0.020, 0.022, 1.0))
    door_plastic = Material("slate_gray_door_plastic", color=(0.18, 0.20, 0.21, 1.0))
    trim_black = Material("matte_black_trim", color=(0.005, 0.006, 0.006, 1.0))
    dark_metal = Material("black_zinc_hinge_metal", color=(0.025, 0.025, 0.023, 1.0))
    latch_metal = Material("brushed_lock_metal", color=(0.62, 0.59, 0.52, 1.0))
    key_slot_black = Material("black_key_slot", color=(0.0, 0.0, 0.0, 1.0))

    for material in (
        dash_plastic,
        bin_plastic,
        door_plastic,
        trim_black,
        dark_metal,
        latch_metal,
        key_slot_black,
    ):
        model.materials.append(material)

    dash = model.part("dash_bin")

    # The fixed dashboard fascia is built as a thick frame around an actual
    # rectangular opening, with a black molded storage bin extending rearward.
    opening_w = 0.220
    opening_h = 0.220
    frame_w = 0.360
    frame_h = 0.300
    side_rail = (frame_w - opening_w) / 2.0
    top_rail = (frame_h - opening_h) / 2.0

    dash.visual(
        Box((side_rail, 0.026, frame_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + side_rail / 2.0), 0.0, 0.0)),
        material=dash_plastic,
        name="front_left_rail",
    )
    dash.visual(
        Box((side_rail, 0.026, frame_h)),
        origin=Origin(xyz=(opening_w / 2.0 + side_rail / 2.0, 0.0, 0.0)),
        material=dash_plastic,
        name="front_right_rail",
    )
    dash.visual(
        Box((frame_w, 0.026, top_rail)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + top_rail / 2.0)),
        material=dash_plastic,
        name="front_top_rail",
    )
    dash.visual(
        Box((frame_w, 0.026, top_rail)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h / 2.0 + top_rail / 2.0))),
        material=dash_plastic,
        name="front_bottom_rail",
    )

    # Thin black reveal/gasket just inside the frame emphasizes the open bin.
    dash.visual(
        Box((0.010, 0.004, opening_h)),
        origin=Origin(xyz=(-opening_w / 2.0 + 0.005, -0.015, 0.0)),
        material=trim_black,
        name="left_reveal",
    )
    dash.visual(
        Box((0.010, 0.004, opening_h)),
        origin=Origin(xyz=(opening_w / 2.0 - 0.005, -0.015, 0.0)),
        material=trim_black,
        name="right_reveal",
    )
    dash.visual(
        Box((opening_w, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, opening_h / 2.0 - 0.005)),
        material=trim_black,
        name="top_reveal",
    )
    dash.visual(
        Box((opening_w, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, -opening_h / 2.0 + 0.005)),
        material=trim_black,
        name="bottom_reveal",
    )

    bin_depth = 0.175
    bin_y = 0.096
    dash.visual(
        Box((0.014, bin_depth, opening_h)),
        origin=Origin(xyz=(-opening_w / 2.0 - 0.007, bin_y, 0.0)),
        material=bin_plastic,
        name="bin_left_wall",
    )
    dash.visual(
        Box((0.014, bin_depth, opening_h)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.007, bin_y, 0.0)),
        material=bin_plastic,
        name="bin_right_wall",
    )
    dash.visual(
        Box((opening_w + 0.028, bin_depth, 0.014)),
        origin=Origin(xyz=(0.0, bin_y, opening_h / 2.0 + 0.007)),
        material=bin_plastic,
        name="bin_top_wall",
    )
    dash.visual(
        Box((opening_w + 0.028, bin_depth, 0.014)),
        origin=Origin(xyz=(0.0, bin_y, -opening_h / 2.0 - 0.007)),
        material=bin_plastic,
        name="bin_floor_wall",
    )
    dash.visual(
        Box((opening_w + 0.028, 0.014, opening_h + 0.028)),
        origin=Origin(xyz=(0.0, 0.180, 0.0)),
        material=bin_plastic,
        name="bin_back_wall",
    )

    hinge_x = -0.118
    hinge_y = -0.026

    # Fixed leaves and alternating fixed knuckles mounted to the left side wall.
    for zc, suffix in ((0.065, "upper"), (-0.065, "lower")):
        dash.visual(
            Box((0.017, 0.011, 0.072)),
            origin=Origin(xyz=(-0.114, -0.0175, zc)),
            material=dark_metal,
            name=f"fixed_hinge_leaf_{suffix}",
        )
        for dz, knuckle_name in ((0.023, "outer_top"), (-0.023, "outer_bottom")):
            dash.visual(
                Cylinder(radius=0.006, length=0.020),
                origin=Origin(xyz=(hinge_x, hinge_y, zc + dz)),
                material=dark_metal,
                name=f"fixed_hinge_knuckle_{suffix}_{knuckle_name}",
            )
        # Screw heads are slightly proud and embedded in the leaf, not floating.
        for dz, screw_name in ((0.018, "top"), (-0.018, "bottom")):
            dash.visual(
                Cylinder(radius=0.0032, length=0.002),
                origin=Origin(xyz=(-0.112, -0.0235, zc + dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=latch_metal,
                name=f"fixed_hinge_screw_{suffix}_{screw_name}",
            )

    # A small keeper lip on the latch edge: the internal cam swings behind it.
    dash.visual(
        Box((0.011, 0.018, 0.060)),
        origin=Origin(xyz=(0.108, 0.020, 0.016)),
        material=dark_metal,
        name="keeper_lip",
    )

    door = model.part("door")
    door_w = 0.205
    door_h = 0.210

    door.visual(
        Box((door_w, 0.018, door_h)),
        origin=Origin(xyz=(0.008 + door_w / 2.0, 0.0, 0.0)),
        material=door_plastic,
        name="door_panel",
    )
    # Raised front border and a shallow central pad make the narrow plastic door
    # read like a molded utility-vehicle glove box lid.
    door.visual(
        Box((door_w - 0.020, 0.004, 0.014)),
        origin=Origin(xyz=(0.008 + door_w / 2.0, -0.011, door_h / 2.0 - 0.015)),
        material=door_plastic,
        name="door_top_rib",
    )
    door.visual(
        Box((door_w - 0.020, 0.004, 0.014)),
        origin=Origin(xyz=(0.008 + door_w / 2.0, -0.011, -(door_h / 2.0 - 0.015))),
        material=door_plastic,
        name="door_bottom_rib",
    )
    door.visual(
        Box((0.014, 0.004, door_h - 0.020)),
        origin=Origin(xyz=(0.020, -0.011, 0.0)),
        material=door_plastic,
        name="door_hinge_rib",
    )
    door.visual(
        Box((0.014, 0.004, door_h - 0.020)),
        origin=Origin(xyz=(door_w, -0.011, 0.0)),
        material=door_plastic,
        name="door_latch_rib",
    )
    door.visual(
        Box((0.122, 0.003, 0.126)),
        origin=Origin(xyz=(0.104, -0.0095, -0.004)),
        material=Material("slightly_lighter_center_pad", color=(0.22, 0.24, 0.245, 1.0)),
        name="recessed_center_pad",
    )

    for zc, suffix in ((0.065, "upper"), (-0.065, "lower")):
        door.visual(
            Box((0.020, 0.008, 0.072)),
            origin=Origin(xyz=(0.010, -0.006, zc)),
            material=dark_metal,
            name=f"moving_hinge_leaf_{suffix}",
        )
        door.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=dark_metal,
            name=f"moving_hinge_knuckle_{suffix}",
        )
        for dz, screw_name in ((0.018, "top"), (-0.018, "bottom")):
            door.visual(
                Cylinder(radius=0.0030, length=0.002),
                origin=Origin(xyz=(0.014, -0.012, zc + dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=latch_metal,
                name=f"moving_hinge_screw_{suffix}_{screw_name}",
            )

    model.articulation(
        "dash_to_door",
        ArticulationType.REVOLUTE,
        parent=dash,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        # Closed geometry extends along local +X; -Z swings the latch edge out
        # toward the occupant side (-Y) as the joint opens.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.35),
    )

    key_cylinder = model.part("key_cylinder")
    key_cylinder.visual(
        Cylinder(radius=0.014, length=0.035),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="lock_barrel",
    )
    key_cylinder.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="lock_face_ring",
    )
    key_cylinder.visual(
        Box((0.020, 0.002, 0.0035)),
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
        material=key_slot_black,
        name="key_slot",
    )

    model.articulation(
        "door_to_key_cylinder",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cylinder,
        origin=Origin(xyz=(0.165, 0.0, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    cam = model.part("cam")
    cam.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="cam_shaft",
    )
    cam.visual(
        Box((0.055, 0.006, 0.014)),
        origin=Origin(xyz=(0.026, 0.045, 0.0)),
        material=latch_metal,
        name="cam_bar",
    )
    cam.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="cam_hub",
    )

    model.articulation(
        "key_cylinder_to_cam",
        ArticulationType.FIXED,
        parent=key_cylinder,
        child=cam,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    dash = object_model.get_part("dash_bin")
    door = object_model.get_part("door")
    key_cylinder = object_model.get_part("key_cylinder")
    cam = object_model.get_part("cam")
    door_hinge = object_model.get_articulation("dash_to_door")
    lock_joint = object_model.get_articulation("door_to_key_cylinder")

    ctx.allow_overlap(
        door,
        key_cylinder,
        elem_a="door_panel",
        elem_b="lock_barrel",
        reason="The keyed cylinder is intentionally shown passing through a drilled bore in the molded door.",
    )
    ctx.allow_overlap(
        key_cylinder,
        cam,
        elem_a="lock_barrel",
        elem_b="cam_shaft",
        reason="The square cam shaft is intentionally seated into the keyed cylinder tailpiece so the cam rotates with the lock.",
    )
    ctx.expect_overlap(
        door,
        key_cylinder,
        axes="y",
        elem_a="door_panel",
        elem_b="lock_barrel",
        min_overlap=0.010,
        name="lock barrel passes through the door thickness",
    )
    ctx.expect_within(
        key_cylinder,
        door,
        axes="xz",
        inner_elem="lock_barrel",
        outer_elem="door_panel",
        margin=0.001,
        name="lock bore is inside the door panel face",
    )
    ctx.expect_gap(
        cam,
        key_cylinder,
        axis="y",
        positive_elem="cam_shaft",
        negative_elem="lock_barrel",
        max_gap=0.001,
        max_penetration=0.001,
        name="cam shaft is seated in the lock tailpiece",
    )

    ctx.expect_gap(
        dash,
        door,
        axis="y",
        positive_elem="front_right_rail",
        negative_elem="door_panel",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door sits just proud of dashboard face",
    )
    ctx.expect_within(
        door,
        dash,
        axes="xz",
        inner_elem="door_panel",
        margin=0.015,
        name="narrow door fits within the rectangular dash frame",
    )
    ctx.expect_gap(
        dash,
        cam,
        axis="x",
        positive_elem="keeper_lip",
        negative_elem="cam_bar",
        min_gap=0.001,
        max_gap=0.015,
        name="locked cam rests near the keeper lip without colliding",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_latch_rib")
    with ctx.pose({door_hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_latch_rib")
    ctx.check(
        "door swings outward on vertical left hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.060,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    locked_cam = ctx.part_element_world_aabb(cam, elem="cam_bar")
    locked_slot = ctx.part_element_world_aabb(key_cylinder, elem="key_slot")
    with ctx.pose({lock_joint: math.pi / 2.0}):
        unlocked_cam = ctx.part_element_world_aabb(cam, elem="cam_bar")
        unlocked_slot = ctx.part_element_world_aabb(key_cylinder, elem="key_slot")
    ctx.check(
        "key slot rotates with cylinder",
        locked_slot is not None
        and unlocked_slot is not None
        and (locked_slot[1][0] - locked_slot[0][0]) > (locked_slot[1][2] - locked_slot[0][2])
        and (unlocked_slot[1][2] - unlocked_slot[0][2]) > (unlocked_slot[1][0] - unlocked_slot[0][0]),
        details=f"locked_slot={locked_slot}, unlocked_slot={unlocked_slot}",
    )
    ctx.check(
        "cam turns from latchwise horizontal to releasewise vertical",
        locked_cam is not None
        and unlocked_cam is not None
        and (locked_cam[1][0] - locked_cam[0][0]) > 0.045
        and (unlocked_cam[1][2] - unlocked_cam[0][2]) > 0.045,
        details=f"locked_cam={locked_cam}, unlocked_cam={unlocked_cam}",
    )

    return ctx.report()


object_model = build_object_model()
