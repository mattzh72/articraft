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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_dryer")

    # Real compact-apartment dryer proportions, in meters.
    width = 0.62
    depth = 0.56
    height = 0.76
    wall = 0.025
    front_y = -depth / 2.0
    drum_z = 0.38
    hinge_x = -0.255
    door_center_offset = 0.255
    door_y = front_y - 0.046

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    trim = model.material("soft_gray_trim", rgba=(0.72, 0.74, 0.74, 1.0))
    rubber = model.material("black_rubber_gasket", rgba=(0.02, 0.022, 0.024, 1.0))
    drum_metal = model.material("brushed_drum_metal", rgba=(0.66, 0.68, 0.67, 1.0))
    dark = model.material("shadowed_interior", rgba=(0.09, 0.095, 0.10, 1.0))
    glass = model.material("smoked_porthole_glass", rgba=(0.45, 0.62, 0.72, 0.38))
    handle_mat = model.material("charcoal_handle", rgba=(0.035, 0.035, 0.038, 1.0))

    body = model.part("body")

    # Cabinet shell: open front center for the drum/porthole, with connected
    # sheet-metal panels rather than a solid block hiding the mechanism.
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=white,
        name="rear_panel",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_0",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_1",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=white,
        name="top_panel",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=white,
        name="bottom_panel",
    )
    body.visual(
        Box((0.065, wall, height)),
        origin=Origin(xyz=(-width / 2.0 + 0.0325, front_y + wall / 2.0, height / 2.0)),
        material=white,
        name="front_stile_0",
    )
    body.visual(
        Box((0.065, wall, height)),
        origin=Origin(xyz=(width / 2.0 - 0.0325, front_y + wall / 2.0, height / 2.0)),
        material=white,
        name="front_stile_1",
    )
    body.visual(
        Box((width, wall, 0.155)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, height - 0.0775)),
        material=white,
        name="front_top_rail",
    )
    body.visual(
        Box((width, wall, 0.155)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, 0.0775)),
        material=white,
        name="front_bottom_rail",
    )

    # Rounded porthole surround and a dark recessed cavity behind it.
    body.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.235, tube=0.012, radial_segments=24, tubular_segments=72),
            "body_porthole_bezel",
        ),
        origin=Origin(xyz=(0.0, front_y - 0.004, drum_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="porthole_bezel",
    )
    body.visual(
        Box((0.450, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, front_y + 0.010, drum_z)),
        material=trim,
        name="axle_spoke_x",
    )
    body.visual(
        Box((0.012, 0.006, 0.450)),
        origin=Origin(xyz=(0.0, front_y + 0.010, drum_z)),
        material=trim,
        name="axle_spoke_z",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.0, front_y + 0.018, drum_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="axle_bearing",
    )
    for i, z in enumerate((drum_z - 0.145, drum_z + 0.145)):
        body.visual(
            Box((0.032, 0.033, 0.115)),
            origin=Origin(xyz=(hinge_x - 0.020, front_y - 0.0145, z)),
            material=trim,
            name=f"hinge_leaf_{i}",
        )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.205, length=0.34),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, -0.182, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="drum_hub",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        x = 0.122 * math.cos(angle)
        z = 0.122 * math.sin(angle)
        drum.visual(
            Box((0.030, 0.285, 0.018)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -angle, 0.0)),
            material=trim,
            name=f"tumbling_fin_{i}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.217, tube=0.024, radial_segments=24, tubular_segments=72),
            "door_outer_ring",
        ),
        origin=Origin(xyz=(door_center_offset, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="door_ring",
    )
    door.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.184, tube=0.010, radial_segments=16, tubular_segments=64),
            "door_inner_gasket",
        ),
        origin=Origin(xyz=(door_center_offset, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_gasket",
    )
    door.visual(
        Cylinder(radius=0.177, length=0.010),
        origin=Origin(xyz=(door_center_offset, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=trim,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=trim,
        name="hinge_barrel_1",
    )
    door.visual(
        Box((0.040, 0.018, 0.265)),
        origin=Origin(xyz=(0.012, 0.002, 0.0)),
        material=white,
        name="hinge_bridge",
    )

    latch = model.part("latch_handle")
    latch.visual(
        Cylinder(radius=0.025, length=0.026),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="latch_base",
    )
    latch.visual(
        Box((0.036, 0.026, 0.145)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=handle_mat,
        name="latch_grip",
    )
    latch.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="latch_neck",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.070, drum_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=10.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, door_y, drum_z)),
        # Closed door geometry extends along local +X.  Negative Z makes
        # positive motion swing the porthole outward toward the user/front.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.80),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.493, -0.026, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("body_to_door")
    latch_pivot = object_model.get_articulation("door_to_latch")

    ctx.allow_overlap(
        latch,
        door,
        elem_a="latch_base",
        elem_b="door_ring",
        reason="The latch base is a small seated spindle captured in the door rim.",
    )
    ctx.allow_overlap(
        body,
        drum,
        elem_a="axle_bearing",
        elem_b="drum_hub",
        reason="The drum hub is intentionally captured inside the front bearing bushing on the axle.",
    )

    ctx.expect_within(
        drum,
        body,
        axes="xz",
        elem_a="drum_shell",
        elem_b="porthole_bezel",
        margin=0.035,
        name="drum sits concentrically behind the round porthole",
    )
    ctx.expect_gap(
        drum,
        door,
        axis="y",
        min_gap=0.020,
        positive_elem="drum_shell",
        negative_elem="glass_window",
        name="closed glass remains in front of the drum",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.003,
        max_gap=0.018,
        positive_elem="porthole_bezel",
        negative_elem="door_ring",
        name="closed door is proud of the cabinet front",
    )
    ctx.expect_overlap(
        body,
        drum,
        axes="xz",
        elem_a="axle_bearing",
        elem_b="drum_hub",
        min_overlap=0.025,
        name="drum hub is centered in the front axle bearing",
    )
    ctx.expect_gap(
        drum,
        body,
        axis="y",
        max_penetration=0.014,
        positive_elem="drum_hub",
        negative_elem="axle_bearing",
        name="drum bearing insertion is shallow",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        elem_a="latch_base",
        elem_b="door_ring",
        min_overlap=0.010,
        name="right-side latch base lands on the door rim",
    )
    ctx.expect_gap(
        door,
        latch,
        axis="y",
        max_penetration=0.030,
        positive_elem="door_ring",
        negative_elem="latch_base",
        name="latch spindle embed is shallow and local",
    )

    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_hinge: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    ctx.check(
        "door hinge opens outward from the left",
        open_aabb is not None
        and closed_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    with ctx.pose({latch_pivot: 0.30}):
        turned_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_pivot: 0.0}):
        rest_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "latch handle has a small twisting motion",
        turned_aabb is not None
        and rest_aabb is not None
        and (turned_aabb[1][0] - turned_aabb[0][0]) > (rest_aabb[1][0] - rest_aabb[0][0]) + 0.004,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
