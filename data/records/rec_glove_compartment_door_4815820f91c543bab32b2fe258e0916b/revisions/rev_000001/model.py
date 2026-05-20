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
    model = ArticulatedObject(name="rugged_offroad_glove_compartment")

    dash_plastic = Material("charcoal_grained_plastic", color=(0.055, 0.060, 0.058, 1.0))
    inner_black = Material("deep_shadow_black", color=(0.012, 0.012, 0.011, 1.0))
    door_plastic = Material("slightly_lighter_rugged_plastic", color=(0.105, 0.110, 0.105, 1.0))
    rubber = Material("matte_black_rubber_gasket", color=(0.004, 0.004, 0.004, 1.0))
    hinge_steel = Material("dark_phosphate_hinge_steel", color=(0.030, 0.032, 0.033, 1.0))
    worn_edge = Material("brushed_worn_edges", color=(0.42, 0.42, 0.39, 1.0))

    frame = model.part("dashboard_frame")

    # Thick dashboard flange around the glove-box opening.
    frame.visual(Box((0.76, 0.080, 0.080)), origin=Origin(xyz=(0.0, 0.0, 0.210)), material=dash_plastic, name="top_rail")
    frame.visual(Box((0.76, 0.080, 0.080)), origin=Origin(xyz=(0.0, 0.0, -0.210)), material=dash_plastic, name="bottom_rail")
    frame.visual(Box((0.080, 0.080, 0.500)), origin=Origin(xyz=(-0.340, 0.0, 0.0)), material=dash_plastic, name="hinge_jamb")
    frame.visual(Box((0.080, 0.080, 0.500)), origin=Origin(xyz=(0.340, 0.0, 0.0)), material=dash_plastic, name="latch_jamb")

    # A fixed dark bin behind the frame, built as separate thick walls so the
    # compartment reads as hollow rather than a solid block.
    frame.visual(Box((0.030, 0.225, 0.345)), origin=Origin(xyz=(-0.315, 0.148, 0.0)), material=inner_black, name="left_bin_wall")
    frame.visual(Box((0.030, 0.225, 0.345)), origin=Origin(xyz=(0.315, 0.148, 0.0)), material=inner_black, name="right_bin_wall")
    frame.visual(Box((0.600, 0.225, 0.030)), origin=Origin(xyz=(0.0, 0.148, 0.185)), material=inner_black, name="upper_bin_wall")
    frame.visual(Box((0.600, 0.225, 0.030)), origin=Origin(xyz=(0.0, 0.148, -0.185)), material=inner_black, name="lower_bin_wall")
    frame.visual(Box((0.600, 0.030, 0.345)), origin=Origin(xyz=(0.0, 0.258, 0.0)), material=inner_black, name="back_bin_wall")

    # Rubber gasket just proud of the inner lip.
    frame.visual(Box((0.640, 0.018, 0.024)), origin=Origin(xyz=(0.0, -0.049, 0.181)), material=rubber, name="upper_gasket")
    frame.visual(Box((0.640, 0.018, 0.024)), origin=Origin(xyz=(0.0, -0.049, -0.181)), material=rubber, name="lower_gasket")
    frame.visual(Box((0.024, 0.018, 0.356)), origin=Origin(xyz=(0.312, -0.049, 0.0)), material=rubber, name="latch_gasket")

    # Exposed fixed hinge knuckles on the jamb: two hinge assemblies, each with
    # top and bottom fixed knuckles that capture a moving door knuckle between.
    for prefix, zc in (("upper", 0.105), ("lower", -0.105)):
        for suffix, zoff in (("top", 0.050), ("bottom", -0.050)):
            name = f"{prefix}_hinge_{suffix}_knuckle"
            frame.visual(
                Cylinder(radius=0.018, length=0.030),
                origin=Origin(xyz=(-0.325, -0.065, zc + zoff)),
                material=hinge_steel,
                name=name,
            )
            frame.visual(
                Box((0.066, 0.012, 0.028)),
                origin=Origin(xyz=(-0.350, -0.065, zc + zoff)),
                material=hinge_steel,
                name=f"{prefix}_{suffix}_hinge_leaf",
            )
            frame.visual(
                Box((0.046, 0.032, 0.028)),
                origin=Origin(xyz=(-0.362, -0.050, zc + zoff)),
                material=hinge_steel,
                name=f"{prefix}_{suffix}_hinge_standoff",
            )

    # Opposite-side latch keeper on the fixed jamb.
    frame.visual(Box((0.038, 0.026, 0.155)), origin=Origin(xyz=(0.315, -0.056, 0.0)), material=hinge_steel, name="latch_strike_plate")
    frame.visual(Box((0.022, 0.035, 0.060)), origin=Origin(xyz=(0.322, -0.061, 0.0)), material=worn_edge, name="latch_catch_lip")

    door = model.part("door_panel")

    # The door part frame is the vertical hinge axis.  The closed panel extends
    # along local +X, with its front face slightly in front of the dashboard.
    door.visual(Box((0.590, 0.045, 0.345)), origin=Origin(xyz=(0.325, 0.015, 0.0)), material=door_plastic, name="main_panel")
    door.visual(Box((0.545, 0.014, 0.026)), origin=Origin(xyz=(0.335, -0.010, 0.142)), material=door_plastic, name="upper_raised_rib")
    door.visual(Box((0.545, 0.014, 0.026)), origin=Origin(xyz=(0.335, -0.010, -0.142)), material=door_plastic, name="lower_raised_rib")
    door.visual(Box((0.026, 0.014, 0.285)), origin=Origin(xyz=(0.065, -0.010, 0.0)), material=door_plastic, name="hinge_side_rib")
    door.visual(Box((0.026, 0.014, 0.285)), origin=Origin(xyz=(0.595, -0.010, 0.0)), material=door_plastic, name="latch_side_rib")
    door.visual(Box((0.390, 0.010, 0.030)), origin=Origin(xyz=(0.365, -0.011, 0.075)), material=door_plastic, name="upper_grip_bead")
    door.visual(Box((0.390, 0.010, 0.030)), origin=Origin(xyz=(0.365, -0.011, -0.075)), material=door_plastic, name="lower_grip_bead")

    for name, x, z in (
        ("screw_0", 0.080, 0.145),
        ("screw_1", 0.570, 0.145),
        ("screw_2", 0.080, -0.145),
        ("screw_3", 0.570, -0.145),
    ):
        door.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.020, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_edge,
            name=name,
        )

    for prefix, zc in (("upper", 0.105), ("lower", -0.105)):
        door.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_steel,
            name=f"{prefix}_hinge_barrel",
        )
        door.visual(
            Box((0.135, 0.014, 0.052)),
            origin=Origin(xyz=(0.066, 0.0, zc)),
            material=hinge_steel,
            name=f"{prefix}_hinge_leaf",
        )

    latch = model.part("quarter_latch")
    # The latch is a separate revolute part on the door face, not the door
    # support.  It carries a large quarter-turn wing and a round escutcheon.
    latch.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="round_escutcheon",
    )
    latch.visual(Box((0.185, 0.020, 0.036)), origin=Origin(xyz=(0.0, -0.020, 0.0)), material=hinge_steel, name="wing_handle")
    latch.visual(Box((0.105, 0.012, 0.016)), origin=Origin(xyz=(0.105, -0.012, 0.0)), material=worn_edge, name="locking_cam")
    latch.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="center_plug",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-0.325, -0.065, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.3, lower=0.0, upper=1.45),
    )

    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.325, -0.0075, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("dashboard_frame")
    door = object_model.get_part("door_panel")
    latch = object_model.get_part("quarter_latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    # The moving door knuckles are coaxial with and vertically trapped between
    # fixed hinge knuckles on the jamb, so the hinges visibly carry the panel.
    for prefix in ("upper", "lower"):
        ctx.expect_overlap(
            door,
            frame,
            axes="xy",
            elem_a=f"{prefix}_hinge_barrel",
            elem_b=f"{prefix}_hinge_top_knuckle",
            min_overlap=0.030,
            name=f"{prefix} hinge knuckles share a vertical pin line",
        )
        ctx.expect_gap(
            frame,
            door,
            axis="z",
            positive_elem=f"{prefix}_hinge_top_knuckle",
            negative_elem=f"{prefix}_hinge_barrel",
            max_gap=0.008,
            max_penetration=0.0,
            name=f"{prefix} hinge upper clearance is small",
        )
        ctx.expect_gap(
            door,
            frame,
            axis="z",
            positive_elem=f"{prefix}_hinge_barrel",
            negative_elem=f"{prefix}_hinge_bottom_knuckle",
            max_gap=0.008,
            max_penetration=0.0,
            name=f"{prefix} hinge lower clearance is small",
        )

    ctx.expect_gap(
        door,
        latch,
        axis="y",
        positive_elem="main_panel",
        negative_elem="round_escutcheon",
        max_gap=0.001,
        max_penetration=0.0,
        name="latch escutcheon seats on the door face",
    )

    rest_latch_pos = ctx.part_world_position(latch)
    with ctx.pose({door_hinge: 1.10}):
        open_latch_pos = ctx.part_world_position(latch)
    ctx.check(
        "door swings outward on the vertical hinge",
        rest_latch_pos is not None
        and open_latch_pos is not None
        and open_latch_pos[1] < rest_latch_pos[1] - 0.18,
        details=f"rest={rest_latch_pos}, open={open_latch_pos}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(latch, elem="wing_handle")
    with ctx.pose({latch_turn: math.pi / 2.0}):
        turned_handle_aabb = ctx.part_element_world_aabb(latch, elem="wing_handle")
    if rest_handle_aabb is not None and turned_handle_aabb is not None:
        rest_width = rest_handle_aabb[1][0] - rest_handle_aabb[0][0]
        rest_height = rest_handle_aabb[1][2] - rest_handle_aabb[0][2]
        turned_width = turned_handle_aabb[1][0] - turned_handle_aabb[0][0]
        turned_height = turned_handle_aabb[1][2] - turned_handle_aabb[0][2]
        ctx.check(
            "quarter-turn latch rotates its wing ninety degrees",
            rest_width > rest_height * 3.0 and turned_height > turned_width * 3.0,
            details=(
                f"rest_width={rest_width:.3f}, rest_height={rest_height:.3f}, "
                f"turned_width={turned_width:.3f}, turned_height={turned_height:.3f}"
            ),
        )
    else:
        ctx.fail("quarter-turn latch rotates its wing ninety degrees", "wing_handle AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
