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
    model = ArticulatedObject(name="surface_mount_distribution_board")

    powder_steel = Material("powder_coated_steel", color=(0.62, 0.66, 0.68, 1.0))
    darker_steel = Material("dark_galvanized_steel", color=(0.36, 0.39, 0.40, 1.0))
    inner_plate = Material("inner_off_white_steel", color=(0.82, 0.84, 0.80, 1.0))
    black = Material("black_polymer", color=(0.02, 0.022, 0.022, 1.0))
    copper = Material("copper_bus", color=(0.80, 0.43, 0.18, 1.0))
    blue = Material("neutral_blue", color=(0.05, 0.18, 0.55, 1.0))
    label = Material("paper_labels", color=(0.94, 0.92, 0.84, 1.0))

    body = model.part("body")

    # A deep surface-mount steel box: back sheet plus four welded side walls.
    body.visual(Box((0.012, 0.540, 0.720)), origin=Origin(xyz=(0.006, 0.0, 0.0)), material=darker_steel, name="back_sheet")
    body.visual(Box((0.184, 0.022, 0.722)), origin=Origin(xyz=(0.092, -0.261, 0.0)), material=powder_steel, name="left_wall")
    body.visual(Box((0.184, 0.022, 0.722)), origin=Origin(xyz=(0.092, 0.261, 0.0)), material=powder_steel, name="right_wall")
    body.visual(Box((0.184, 0.542, 0.022)), origin=Origin(xyz=(0.092, 0.0, 0.349)), material=powder_steel, name="top_wall")
    body.visual(Box((0.184, 0.542, 0.022)), origin=Origin(xyz=(0.092, 0.0, -0.349)), material=powder_steel, name="bottom_wall")

    # Shallow return lips around the open front make the enclosure read as a formed steel body.
    body.visual(Box((0.016, 0.038, 0.710)), origin=Origin(xyz=(0.171, -0.252, 0.0)), material=powder_steel, name="front_lip_left")
    body.visual(Box((0.016, 0.038, 0.710)), origin=Origin(xyz=(0.171, 0.252, 0.0)), material=powder_steel, name="front_lip_right")
    body.visual(Box((0.016, 0.505, 0.038)), origin=Origin(xyz=(0.171, 0.0, 0.334)), material=powder_steel, name="front_lip_top")
    body.visual(Box((0.016, 0.505, 0.038)), origin=Origin(xyz=(0.171, 0.0, -0.334)), material=powder_steel, name="front_lip_bottom")

    # Two fixed hinge mounting pads beside the cover knuckles.  They are tangent to,
    # not intersecting, the moving hinge barrels.
    for z, hinge_name in ((0.225, "upper"), (-0.225, "lower")):
        body.visual(Box((0.030, 0.020, 0.150)), origin=Origin(xyz=(0.166, -0.292, z)), material=darker_steel, name=f"{hinge_name}_cover_hinge_pad")
        body.visual(Box((0.020, 0.034, 0.150)), origin=Origin(xyz=(0.162, -0.276, z)), material=darker_steel, name=f"{hinge_name}_cover_hinge_root")

    body.visual(Box((0.020, 0.014, 0.600)), origin=Origin(xyz=(0.134, -0.250, 0.0)), material=darker_steel, name="subpanel_hinge_mount")

    # Rail-mounted internals are fixed to the back sheet by standoffs so the body
    # remains one supported assembly instead of loose components floating inside.
    for z, row_name in ((0.125, "upper"), (-0.115, "lower")):
        body.visual(Box((0.018, 0.420, 0.026)), origin=Origin(xyz=(0.068, 0.020, z)), material=darker_steel, name=f"{row_name}_din_rail")
        for y in (-0.160, 0.200):
            body.visual(Box((0.070, 0.020, 0.032)), origin=Origin(xyz=(0.035, y, z)), material=darker_steel, name=f"{row_name}_standoff_{y:+.1f}")
        for index, y in enumerate((-0.135, -0.075, -0.015, 0.045, 0.105, 0.165)):
            body.visual(Box((0.046, 0.052, 0.070)), origin=Origin(xyz=(0.098, y, z + 0.006)), material=black, name=f"{row_name}_breaker_{index}")
            body.visual(Box((0.010, 0.026, 0.014)), origin=Origin(xyz=(0.126, y, z + 0.024)), material=blue, name=f"{row_name}_toggle_{index}")
        body.visual(Box((0.014, 0.370, 0.014)), origin=Origin(xyz=(0.050, 0.030, z - 0.050)), material=copper, name=f"{row_name}_busbar")
        for y, support_name in ((-0.135, "left"), (0.165, "right")):
            body.visual(Box((0.050, 0.012, 0.014)), origin=Origin(xyz=(0.026, y, z - 0.050)), material=darker_steel, name=f"{row_name}_{support_name}_bus_support")

    cover_door = model.part("cover_door")
    # The cover part frame is the vertical hinge axis; the steel door spans local +Y.
    cover_door.visual(Box((0.016, 0.565, 0.748)), origin=Origin(xyz=(0.0, 0.292, 0.0)), material=powder_steel, name="door_skin")
    cover_door.visual(Box((0.010, 0.525, 0.034)), origin=Origin(xyz=(-0.010, 0.302, 0.387)), material=darker_steel, name="door_top_return")
    cover_door.visual(Box((0.010, 0.525, 0.034)), origin=Origin(xyz=(-0.010, 0.302, -0.387)), material=darker_steel, name="door_bottom_return")
    cover_door.visual(Box((0.010, 0.035, 0.690)), origin=Origin(xyz=(-0.010, 0.592, 0.0)), material=darker_steel, name="door_strike_return")
    cover_door.visual(Box((0.006, 0.030, 0.088)), origin=Origin(xyz=(0.010, 0.525, 0.0)), material=black, name="quarter_turn_latch")
    cover_door.visual(Cylinder(radius=0.018, length=0.006), origin=Origin(xyz=(0.010, 0.525, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=black, name="latch_escutcheon")
    cover_door.visual(Box((0.004, 0.410, 0.010)), origin=Origin(xyz=(-0.007, 0.300, 0.276)), material=label, name="circuit_label_strip")

    # Two exposed hinge knuckles carried by the moving cover.
    for z, name in ((0.225, "upper"), (-0.225, "lower")):
        cover_door.visual(Cylinder(radius=0.011, length=0.140), origin=Origin(xyz=(0.0, 0.0, z)), material=darker_steel, name=f"{name}_cover_knuckle")
        cover_door.visual(Box((0.006, 0.040, 0.118)), origin=Origin(xyz=(0.0, 0.016, z)), material=darker_steel, name=f"{name}_hinge_leaf")

    sub_panel = model.part("sub_panel")
    # The internal dead-front/sub-panel is a connected frame with two access windows
    # over the breaker rows instead of a solid slab hiding the internals.
    sub_panel.visual(Box((0.008, 0.450, 0.030)), origin=Origin(xyz=(0.0, 0.240, 0.305)), material=inner_plate, name="panel_top_rail")
    sub_panel.visual(Box((0.008, 0.450, 0.030)), origin=Origin(xyz=(0.0, 0.240, -0.305)), material=inner_plate, name="panel_bottom_rail")
    sub_panel.visual(Box((0.008, 0.450, 0.030)), origin=Origin(xyz=(0.0, 0.240, 0.0)), material=inner_plate, name="panel_mid_rail")
    sub_panel.visual(Box((0.008, 0.030, 0.640)), origin=Origin(xyz=(0.0, 0.015, 0.0)), material=inner_plate, name="panel_hinge_stile")
    sub_panel.visual(Box((0.008, 0.026, 0.640)), origin=Origin(xyz=(0.0, 0.465, 0.0)), material=inner_plate, name="panel_latch_stile")
    sub_panel.visual(Box((0.004, 0.370, 0.018)), origin=Origin(xyz=(0.004, 0.240, 0.286)), material=label, name="upper_label_strip")
    sub_panel.visual(Box((0.004, 0.370, 0.018)), origin=Origin(xyz=(0.004, 0.240, -0.286)), material=label, name="lower_label_strip")
    sub_panel.visual(Cylinder(radius=0.008, length=0.560), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=darker_steel, name="subpanel_hinge_barrel")
    sub_panel.visual(Box((0.006, 0.030, 0.560)), origin=Origin(xyz=(0.0, 0.012, 0.0)), material=darker_steel, name="subpanel_hinge_leaf")
    sub_panel.visual(Box((0.004, 0.030, 0.080)), origin=Origin(xyz=(0.004, 0.450, 0.0)), material=black, name="subpanel_pull_tab")

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover_door,
        origin=Origin(xyz=(0.192, -0.292, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=0.0, upper=1.85),
    )

    model.articulation(
        "body_to_sub_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=sub_panel,
        origin=Origin(xyz=(0.134, -0.235, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover_door")
    sub_panel = object_model.get_part("sub_panel")
    cover_hinge = object_model.get_articulation("body_to_cover")
    sub_hinge = object_model.get_articulation("body_to_sub_panel")

    ctx.check(
        "front cover uses two visible hinge knuckles",
        len([v for v in cover.visuals if v.name and v.name.endswith("_cover_knuckle")]) == 2,
        details="expected exactly two named cover hinge-barrel knuckles",
    )

    with ctx.pose({cover_hinge: 0.0, sub_hinge: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="door_skin",
            negative_elem="front_lip_right",
            min_gap=0.002,
            max_gap=0.012,
            name="closed cover sits just proud of the front rim",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="yz",
            elem_a="door_skin",
            elem_b="back_sheet",
            min_overlap=0.45,
            name="cover spans the distribution board opening",
        )
        ctx.expect_gap(
            cover,
            sub_panel,
            axis="x",
            positive_elem="door_skin",
            negative_elem="panel_mid_rail",
            min_gap=0.030,
            name="closed cover stands in front of inner sub-panel",
        )
        ctx.expect_within(
            sub_panel,
            body,
            axes="yz",
            inner_elem="panel_latch_stile",
            outer_elem="back_sheet",
            margin=0.010,
            name="sub-panel frame fits inside the steel body",
        )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.20}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover hinge opens outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][0] > closed_cover_aabb[1][0] + 0.25,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    closed_sub_aabb = ctx.part_world_aabb(sub_panel)
    with ctx.pose({sub_hinge: 1.00}):
        open_sub_aabb = ctx.part_world_aabb(sub_panel)
    ctx.check(
        "internal sub-panel opens forward for inner-row access",
        closed_sub_aabb is not None
        and open_sub_aabb is not None
        and open_sub_aabb[1][0] > closed_sub_aabb[1][0] + 0.18,
        details=f"closed={closed_sub_aabb}, open={open_sub_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
