from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="condenser_tumble_dryer")

    white = model.material("warm_white_plastic", rgba=(0.94, 0.94, 0.90, 1.0))
    satin_white = model.material("slightly_satin_white", rgba=(0.86, 0.87, 0.84, 1.0))
    dark = model.material("dark_recess", rgba=(0.03, 0.035, 0.04, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.58, 0.60, 0.60, 1.0))
    hinge_metal = model.material("hinge_pin_metal", rgba=(0.45, 0.46, 0.46, 1.0))
    glass = model.material("smoked_clear_glass", rgba=(0.10, 0.16, 0.18, 0.38))
    water_blue = model.material("pale_water_tint", rgba=(0.32, 0.60, 0.90, 0.42))

    # Overall appliance frame: front is +X, vertical is +Z, and +Y is the
    # left edge as viewed from the front.  Dimensions match a domestic dryer.
    body = model.part("body")
    body.visual(Box((0.62, 0.025, 0.85)), origin=Origin(xyz=(0.0, 0.2875, 0.425)), material=white, name="left_side")
    body.visual(Box((0.62, 0.025, 0.85)), origin=Origin(xyz=(0.0, -0.2875, 0.425)), material=white, name="right_side")
    body.visual(Box((0.62, 0.60, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.835)), material=white, name="top_panel")
    body.visual(Box((0.62, 0.60, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=white, name="base_floor")
    body.visual(Box((0.025, 0.270, 0.80)), origin=Origin(xyz=(-0.2975, 0.165, 0.425)), material=satin_white, name="rear_left_panel")
    body.visual(Box((0.025, 0.270, 0.80)), origin=Origin(xyz=(-0.2975, -0.165, 0.425)), material=satin_white, name="rear_right_panel")
    body.visual(Box((0.025, 0.060, 0.295)), origin=Origin(xyz=(-0.2975, 0.0, 0.6775)), material=satin_white, name="rear_top_panel")
    body.visual(Box((0.025, 0.060, 0.445)), origin=Origin(xyz=(-0.2975, 0.0, 0.2475)), material=satin_white, name="rear_bottom_panel")

    # Front sheet-metal areas leave a real circular service opening rather than
    # hiding the drum behind a solid slab.
    body.visual(Box((0.030, 0.60, 0.130)), origin=Origin(xyz=(0.295, 0.0, 0.785)), material=white, name="top_fascia")
    body.visual(Box((0.030, 0.60, 0.120)), origin=Origin(xyz=(0.295, 0.0, 0.210)), material=white, name="lower_front")
    body.visual(Box((0.030, 0.050, 0.460)), origin=Origin(xyz=(0.295, 0.275, 0.500)), material=white, name="left_front_cheek")
    body.visual(Box((0.030, 0.050, 0.460)), origin=Origin(xyz=(0.295, -0.275, 0.500)), material=white, name="right_front_cheek")

    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.400, 0.400),
                (0.500, 0.500),
                0.026,
                opening_shape="circle",
                outer_shape="circle",
                center=True,
            ),
            "body_porthole_bezel",
        ),
        origin=Origin(xyz=(0.323, 0.0, 0.500), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_white,
        name="porthole_bezel",
    )

    # Base plinth and the sliding water drawer pocket.
    body.visual(Box((0.030, 0.090, 0.115)), origin=Origin(xyz=(0.295, 0.255, 0.092)), material=white, name="left_plinth_post")
    body.visual(Box((0.030, 0.090, 0.115)), origin=Origin(xyz=(0.295, -0.255, 0.092)), material=white, name="right_plinth_post")
    body.visual(Box((0.030, 0.60, 0.026)), origin=Origin(xyz=(0.295, 0.0, 0.145)), material=white, name="plinth_top_rail")
    body.visual(Box((0.245, 0.015, 0.018)), origin=Origin(xyz=(0.180, 0.212, 0.086)), material=dark, name="drawer_left_rail")
    body.visual(Box((0.245, 0.015, 0.018)), origin=Origin(xyz=(0.180, -0.212, 0.086)), material=dark, name="drawer_right_rail")
    body.visual(Box((0.230, 0.430, 0.010)), origin=Origin(xyz=(0.175, 0.0, 0.128)), material=dark, name="drawer_tunnel_shadow")

    # Small exposed leaves next to the two porthole hinge barrels.
    for zc, name in ((0.620, "upper"), (0.380, "lower")):
        body.visual(
            Box((0.032, 0.042, 0.074)),
            origin=Origin(xyz=(0.319, 0.268, zc)),
            material=hinge_metal,
            name=f"{name}_fixed_leaf",
        )
    body.visual(
        Cylinder(radius=0.035, length=0.035),
        origin=Origin(xyz=(-0.282, 0.0, 0.500), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="rear_bearing",
    )

    # The stainless drum sits behind the porthole and rotates around a horizontal
    # axle.  The three raised lifters make rotation visible.
    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.220, length=0.420),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.330, 0.330),
                (0.435, 0.435),
                0.020,
                opening_shape="circle",
                outer_shape="circle",
                center=True,
            ),
            "drum_front_lip",
        ),
        origin=Origin(xyz=(0.195, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="front_lip",
    )
    drum.visual(
        Cylinder(radius=0.016, length=0.160),
        origin=Origin(xyz=(-0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="rear_axle",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = 0.195
        drum.visual(
            Box((0.340, 0.030, 0.045)),
            origin=Origin(
                xyz=(-0.020, -math.sin(angle) * radial, math.cos(angle) * radial),
                rpy=(angle, 0.0, 0.0),
            ),
            material=stainless,
            name=f"baffle_{i}",
        )

    # Round porthole door.  Its part frame is the hinge axis on the left edge;
    # the circular panel extends toward local -Y in the closed pose.
    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.300, 0.300),
                (0.490, 0.490),
                0.046,
                opening_shape="circle",
                outer_shape="circle",
                center=True,
            ),
            "door_outer_ring",
        ),
        origin=Origin(xyz=(0.023, -0.245, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="outer_ring",
    )
    door.visual(
        Cylinder(radius=0.153, length=0.014),
        origin=Origin(xyz=(0.026, -0.245, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Box((0.018, 0.055, 0.070)),
        origin=Origin(xyz=(0.030, -0.470, 0.0)),
        material=satin_white,
        name="pull_grip",
    )
    for zoff, name in ((0.120, "upper"), (-0.120, "lower")):
        door.visual(
            Cylinder(radius=0.012, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, zoff)),
            material=hinge_metal,
            name=f"{name}_barrel",
        )
        door.visual(
            Box((0.024, 0.065, 0.060)),
            origin=Origin(xyz=(0.012, -0.026, zoff)),
            material=hinge_metal,
            name=f"{name}_moving_leaf",
        )

    # Condenser water collection drawer in the base plinth.
    drawer = model.part("drawer")
    drawer.visual(Box((0.024, 0.440, 0.088)), origin=Origin(xyz=(0.012, 0.0, 0.0)), material=white, name="front_face")
    drawer.visual(Box((0.290, 0.411, 0.064)), origin=Origin(xyz=(-0.135, 0.0, 0.000)), material=satin_white, name="tray")
    drawer.visual(Box((0.165, 0.330, 0.044)), origin=Origin(xyz=(-0.155, 0.0, 0.010)), material=water_blue, name="water_tank")
    drawer.visual(Box((0.006, 0.320, 0.018)), origin=Origin(xyz=(0.025, 0.0, 0.018)), material=dark, name="finger_recess")

    model.articulation(
        "body_to_drum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.020, 0.0, 0.500)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.345, 0.245, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.315, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_drawer")

    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing",
        elem_b="rear_axle",
        reason="The drum axle is intentionally captured inside the rear bearing boss.",
    )
    for name in ("upper", "lower"):
        ctx.allow_overlap(
            body,
            door,
            elem_a=f"{name}_fixed_leaf",
            elem_b=f"{name}_barrel",
            reason="The fixed hinge leaf locally captures the porthole-door barrel on the shared hinge pin.",
        )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="drawer_left_rail",
        elem_b="tray",
        reason="The sliding water drawer tray is intentionally represented seated in the left guide rail.",
    )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="drawer_right_rail",
        elem_b="tray",
        reason="The sliding water drawer tray is intentionally represented seated in the right guide rail.",
    )

    ctx.check("drum has horizontal revolute axle", drum_joint.articulation_type == ArticulationType.REVOLUTE and tuple(drum_joint.axis) == (1.0, 0.0, 0.0))
    ctx.check("door has vertical hinge axis", door_joint.articulation_type == ArticulationType.REVOLUTE and tuple(door_joint.axis) == (0.0, 0.0, 1.0))
    ctx.check("drawer has forward prismatic slide", drawer_joint.articulation_type == ArticulationType.PRISMATIC and tuple(drawer_joint.axis) == (1.0, 0.0, 0.0))

    ctx.expect_within(drum, body, axes="yz", inner_elem="rear_axle", outer_elem="rear_bearing", margin=0.002, name="drum axle is centered in rear bearing")
    ctx.expect_overlap(drum, body, axes="x", elem_a="rear_axle", elem_b="rear_bearing", min_overlap=0.020, name="drum axle remains captured by rear bearing")
    for name in ("upper", "lower"):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            max_penetration=0.004,
            positive_elem=f"{name}_barrel",
            negative_elem=f"{name}_fixed_leaf",
            name=f"{name} hinge barrel is seated in fixed leaf",
        )
    ctx.expect_gap(
        body,
        drawer,
        axis="y",
        max_penetration=0.002,
        positive_elem="drawer_left_rail",
        negative_elem="tray",
        name="left drawer rail lightly captures tray",
    )
    ctx.expect_gap(
        drawer,
        body,
        axis="y",
        max_penetration=0.002,
        positive_elem="tray",
        negative_elem="drawer_right_rail",
        name="right drawer rail lightly captures tray",
    )

    # The closed porthole door sits just proud of the front bezel and overlaps it
    # in Y/Z projection, proving it covers the round opening without being fused.
    ctx.expect_gap(
        door,
        body,
        axis="x",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem="outer_ring",
        negative_elem="porthole_bezel",
        name="closed door is proud of front bezel",
    )
    ctx.expect_overlap(door, body, axes="yz", elem_a="outer_ring", elem_b="porthole_bezel", min_overlap=0.35, name="door covers the body porthole")

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from left hinge",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > rest_door_aabb[1][0] + 0.10,
        details=f"closed={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.200}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(drawer, body, axes="x", elem_a="tray", elem_b="drawer_tunnel_shadow", min_overlap=0.030, name="extended drawer remains retained in plinth")
    ctx.check(
        "water drawer slides forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.18,
        details=f"closed={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_baffle = ctx.part_element_world_aabb(drum, elem="baffle_0")
    with ctx.pose({drum_joint: 0.75}):
        rotated_baffle = ctx.part_element_world_aabb(drum, elem="baffle_0")
    ctx.check(
        "drum baffle rotates with drum",
        rest_baffle is not None
        and rotated_baffle is not None
        and abs(rotated_baffle[1][1] - rest_baffle[1][1]) > 0.04,
        details=f"rest={rest_baffle}, rotated={rotated_baffle}",
    )

    return ctx.report()


object_model = build_object_model()
