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
    model = ArticulatedObject(name="server_tower_chassis")

    steel = model.material("zinc_plated_steel", color=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_powder_coat", color=(0.08, 0.09, 0.10, 1.0))
    black = model.material("black_plastic", color=(0.015, 0.016, 0.018, 1.0))
    blue = model.material("blue_release_tabs", color=(0.05, 0.22, 0.75, 1.0))
    vent = model.material("dark_perforated_vent", color=(0.02, 0.025, 0.03, 0.82))

    depth = 0.72
    width = 0.46
    height = 1.20
    t = 0.012
    front_x = -depth / 2.0
    rear_x = depth / 2.0
    side_y = width / 2.0

    chassis = model.part("chassis")
    # One connected sheet-metal chassis: top/bottom trays, rear plate, and the
    # front bay frame all touch each other so the root reads as a single steel
    # tower body rather than loose rails.
    chassis.visual(
        Box((depth, width, t)),
        origin=Origin(xyz=(0.0, 0.0, t / 2.0)),
        material=steel,
        name="bottom_tray",
    )
    chassis.visual(
        Box((depth, width, t)),
        origin=Origin(xyz=(0.0, 0.0, height - t / 2.0)),
        material=steel,
        name="top_tray",
    )
    chassis.visual(
        Box((t, width, height)),
        origin=Origin(xyz=(rear_x - t / 2.0, 0.0, height / 2.0)),
        material=dark_steel,
        name="rear_plate",
    )
    chassis.visual(
        Box((t, 0.045, height)),
        origin=Origin(xyz=(front_x + t / 2.0, -side_y + 0.0225, height / 2.0)),
        material=steel,
        name="front_post_0",
    )
    chassis.visual(
        Box((t, 0.045, height)),
        origin=Origin(xyz=(front_x + t / 2.0, side_y - 0.0225, height / 2.0)),
        material=steel,
        name="front_post_1",
    )
    chassis.visual(
        Box((t, width, 0.05)),
        origin=Origin(xyz=(front_x + t / 2.0, 0.0, 0.025)),
        material=steel,
        name="front_sill",
    )
    chassis.visual(
        Box((t, width, 0.05)),
        origin=Origin(xyz=(front_x + t / 2.0, 0.0, height - 0.025)),
        material=steel,
        name="front_header",
    )
    chassis.visual(
        Box((0.008, width - 0.07, 0.84)),
        origin=Origin(xyz=(front_x + 0.012, 0.0, 0.57)),
        material=black,
        name="drive_bay_shadow",
    )

    # Four fixed internal rails per side support the hot-swap sleds.  Each rail
    # reaches from the front frame back to the rear plate, so the root chassis
    # remains one connected assembly.
    sled_z = [0.27, 0.38, 0.49, 0.60]
    rail_len = depth - 2.0 * t
    for i, z in enumerate(sled_z):
        for sign, suffix in ((-1.0, "a"), (1.0, "b")):
            chassis.visual(
                Box((rail_len, 0.012, 0.014)),
                origin=Origin(xyz=(0.0, sign * 0.180, z - 0.048)),
                material=steel,
                name=f"rail_{i}_{suffix}",
            )

    # Fixed hinge knuckles on both rear side edges.
    hinge_r = 0.014
    hinge_x = rear_x + hinge_r
    side_hinge_y = side_y + 0.035
    neg_side_hinge_y = -side_hinge_y
    fixed_side_segments = ((0.09, 0.18), (0.48, 0.18), (0.87, 0.18))
    for side_idx, hy in enumerate((side_hinge_y, neg_side_hinge_y)):
        sign = 1.0 if hy > 0 else -1.0
        for j, (zc, length) in enumerate(fixed_side_segments):
            chassis.visual(
                Box((0.028, 0.035, length)),
                origin=Origin(xyz=(rear_x + 0.014, sign * (side_y + 0.0175), zc)),
                material=dark_steel,
                name=f"side{side_idx}_hinge_leaf_{j}",
            )
            chassis.visual(
                Cylinder(radius=hinge_r, length=length),
                origin=Origin(xyz=(hinge_x, hy, zc)),
                material=dark_steel,
                name=f"side{side_idx}_fixed_knuckle_{j}",
            )

    # Fixed knuckles for the front door hinge along the left edge of the front.
    front_hinge_r = 0.012
    front_hinge_x = front_x - t / 2.0
    front_hinge_y = -side_y - front_hinge_r - 0.012
    fixed_front_segments = ()
    for j, (zc, length) in enumerate(fixed_front_segments):
        chassis.visual(
            Box((0.030, 0.025, length)),
            origin=Origin(xyz=(front_x - 0.003, -side_y - 0.020, zc)),
            material=dark_steel,
            name=f"front_hinge_leaf_{j}",
        )
        chassis.visual(
            Cylinder(radius=front_hinge_r, length=length),
            origin=Origin(xyz=(front_hinge_x, front_hinge_y, zc)),
            material=dark_steel,
            name=f"front_fixed_knuckle_{j}",
        )

    # Rear cable-management bracket fixed hinge tabs.
    cable_hinge_x = rear_x + 0.028
    cable_hinge_y = -0.185
    for j, (zc, length) in enumerate(((0.34, 0.12), (0.56, 0.12))):
        chassis.visual(
            Box((0.036, 0.040, length)),
            origin=Origin(xyz=(rear_x + 0.010, cable_hinge_y + 0.012, zc)),
            material=dark_steel,
            name=f"cable_hinge_tab_{j}",
        )
        chassis.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(xyz=(cable_hinge_x, cable_hinge_y, zc)),
            material=dark_steel,
            name=f"cable_fixed_knuckle_{j}",
        )

    # Two hinged side panels.  Numeric suffixes avoid arbitrary left/right names
    # for an otherwise symmetric pair.
    panel_thick = 0.010
    for side_idx, (hy, sign, axis_z) in enumerate(
        ((side_hinge_y, 1.0, -1.0), (neg_side_hinge_y, -1.0, 1.0))
    ):
        side_panel = model.part(f"side_panel_{side_idx}")
        panel_y_local = sign * (side_y + panel_thick / 2.0) - hy
        panel_x_local = -(depth / 2.0 + hinge_r)
        side_panel.visual(
            Box((depth, panel_thick, height - 0.065)),
            origin=Origin(xyz=(panel_x_local, panel_y_local, height / 2.0)),
            material=steel,
            name="panel_sheet",
        )
        for j, (zc, length) in enumerate(((0.285, 0.17), (0.675, 0.17), (1.065, 0.16))):
            side_panel.visual(
                Cylinder(radius=hinge_r, length=length),
                origin=Origin(xyz=(0.0, 0.0, zc)),
                material=dark_steel,
                name=f"hinge_knuckle_{j}",
            )
            side_panel.visual(
                Box((0.040, 0.032, length)),
                origin=Origin(xyz=(-0.024, sign * -0.016, zc)),
                material=dark_steel,
                name=f"knuckle_leaf_{j}",
            )
        side_panel.visual(
            Box((0.030, 0.016, 0.12)),
            origin=Origin(xyz=(-depth + 0.055, panel_y_local - sign * 0.010, 0.62)),
            material=black,
            name="front_latch",
        )
        model.articulation(
            f"chassis_to_side_panel_{side_idx}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=side_panel,
            origin=Origin(xyz=(hinge_x, hy, 0.0)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.85),
        )

    # Hinged front security door, with a dark vent panel and a right-side pull.
    front_door = model.part("front_door")
    door_t = 0.014
    door_z = height / 2.0
    door_center_y = -front_hinge_y
    front_door.visual(
        Box((door_t, width, height - 0.12)),
        origin=Origin(xyz=(0.0, door_center_y, door_z)),
        material=dark_steel,
        name="door_panel",
    )
    front_door.visual(
        Box((0.006, width - 0.09, 0.78)),
        origin=Origin(xyz=(-door_t / 2.0 - 0.003, door_center_y, 0.54)),
        material=vent,
        name="vent_insert",
    )
    front_door.visual(
        Box((0.018, 0.026, 0.20)),
        origin=Origin(xyz=(-door_t / 2.0 - 0.010, door_center_y + width / 2.0 - 0.045, 0.65)),
        material=black,
        name="door_pull",
    )
    for j, (zc, length) in enumerate(((0.32, 0.16), (0.72, 0.16), (1.10, 0.13))):
        front_door.visual(
            Cylinder(radius=front_hinge_r, length=length),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=dark_steel,
            name=f"hinge_knuckle_{j}",
        )
        front_door.visual(
            Box((0.016, 0.036, length)),
            origin=Origin(xyz=(0.0, 0.018, zc)),
            material=dark_steel,
            name=f"knuckle_leaf_{j}",
        )
    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=2.05),
    )

    # Hot-swap drive sleds on individual prismatic rails.  The caddy geometry
    # extends into the chassis at rest and retains insertion when pulled out.
    sled_len = 0.44
    sled_front_x = front_x + 0.040
    for i, z in enumerate(sled_z):
        sled = model.part(f"drive_sled_{i}")
        sled.visual(
            Box((sled_len, 0.320, 0.070)),
            origin=Origin(xyz=(sled_len / 2.0, 0.0, 0.0)),
            material=steel,
            name="sled_body",
        )
        sled.visual(
            Box((0.018, 0.342, 0.090)),
            origin=Origin(xyz=(0.009, 0.0, 0.0)),
            material=dark_steel,
            name="front_face",
        )
        sled.visual(
            Box((0.025, 0.090, 0.018)),
            origin=Origin(xyz=(-0.006, 0.0, 0.018)),
            material=black,
            name="pull_handle",
        )
        sled.visual(
            Box((0.010, 0.050, 0.016)),
            origin=Origin(xyz=(-0.002, -0.135, 0.020)),
            material=blue,
            name="release_tab",
        )
        sled.visual(
            Box((sled_len, 0.016, 0.014)),
            origin=Origin(xyz=(sled_len / 2.0, -0.166, -0.048)),
            material=dark_steel,
            name="side_runner_a",
        )
        sled.visual(
            Box((sled_len, 0.016, 0.014)),
            origin=Origin(xyz=(sled_len / 2.0, 0.166, -0.048)),
            material=dark_steel,
            name="side_runner_b",
        )
        model.articulation(
            f"chassis_to_drive_sled_{i}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=sled,
            origin=Origin(xyz=(sled_front_x, 0.0, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.28),
        )

    # Hinged rear cable-management bracket with a comb-like set of cable fingers.
    cable = model.part("cable_bracket")
    cable.visual(
        Cylinder(radius=0.011, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=dark_steel,
        name="hinge_knuckle_0",
    )
    cable.visual(
        Box((0.020, 0.040, 0.10)),
        origin=Origin(xyz=(0.0, 0.025, 0.45)),
        material=dark_steel,
        name="hinge_leaf_0",
    )
    cable.visual(
        Box((0.014, 0.290, 0.030)),
        origin=Origin(xyz=(0.0, 0.205, 0.31)),
        material=black,
        name="lower_bar",
    )
    cable.visual(
        Box((0.014, 0.290, 0.030)),
        origin=Origin(xyz=(0.0, 0.205, 0.68)),
        material=black,
        name="upper_bar",
    )
    cable.visual(
        Box((0.014, 0.024, 0.390)),
        origin=Origin(xyz=(0.0, 0.055, 0.495)),
        material=black,
        name="inner_spine",
    )
    cable.visual(
        Box((0.014, 0.030, 0.390)),
        origin=Origin(xyz=(0.0, 0.325, 0.495)),
        material=black,
        name="outer_spine",
    )
    for j, z in enumerate((0.39, 0.48, 0.57)):
        cable.visual(
            Box((0.012, 0.240, 0.018)),
            origin=Origin(xyz=(0.0, 0.205, z)),
            material=black,
            name=f"cable_finger_{j}",
        )
    model.articulation(
        "chassis_to_cable_bracket",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=cable,
        origin=Origin(xyz=(cable_hinge_x, cable_hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    front_door = object_model.get_part("front_door")
    side_panel_0 = object_model.get_part("side_panel_0")
    side_panel_1 = object_model.get_part("side_panel_1")
    cable = object_model.get_part("cable_bracket")

    def aabb_center(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    # The closed front door must actually cover the hot-swap bay fronts, while
    # each sled remains supported by, and retained on, its fixed chassis rail.
    for i in range(4):
        sled = object_model.get_part(f"drive_sled_{i}")
        slide = object_model.get_articulation(f"chassis_to_drive_sled_{i}")

        ctx.expect_overlap(
            front_door,
            sled,
            axes="yz",
            elem_a="door_panel",
            elem_b="front_face",
            min_overlap=0.08,
            name=f"front door covers drive sled {i}",
        )
        ctx.expect_within(
            sled,
            chassis,
            axes="y",
            inner_elem="sled_body",
            outer_elem="drive_bay_shadow",
            margin=0.0,
            name=f"drive sled {i} is centered in the bay",
        )
        ctx.expect_contact(
            sled,
            chassis,
            elem_a="side_runner_b",
            elem_b=f"rail_{i}_b",
            contact_tol=1e-5,
            name=f"drive sled {i} rides on its rail",
        )
        ctx.expect_overlap(
            sled,
            chassis,
            axes="x",
            elem_a="side_runner_b",
            elem_b=f"rail_{i}_b",
            min_overlap=0.30,
            name=f"drive sled {i} is inserted at rest",
        )

        rest_pos = ctx.part_world_position(sled)
        with ctx.pose({slide: 0.28}):
            ctx.expect_overlap(
                sled,
                chassis,
                axes="x",
                elem_a="side_runner_b",
                elem_b=f"rail_{i}_b",
                min_overlap=0.15,
                name=f"drive sled {i} retains rail engagement",
            )
            extended_pos = ctx.part_world_position(sled)
        ctx.check(
            f"drive sled {i} pulls out forward",
            rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.25,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    # Revolute mechanisms should move in the real-world outward direction from
    # the closed tower: side skins swing away from the chassis, the front door
    # swings toward the user, and the rear cable bracket swings rearward.
    side_joint_0 = object_model.get_articulation("chassis_to_side_panel_0")
    side_joint_1 = object_model.get_articulation("chassis_to_side_panel_1")
    front_joint = object_model.get_articulation("chassis_to_front_door")
    cable_joint = object_model.get_articulation("chassis_to_cable_bracket")

    rest_side_0 = aabb_center(side_panel_0)
    with ctx.pose({side_joint_0: 1.20}):
        open_side_0 = aabb_center(side_panel_0)
    ctx.check(
        "side panel 0 opens outward",
        rest_side_0 is not None and open_side_0 is not None and open_side_0[1] > rest_side_0[1] + 0.20,
        details=f"rest={rest_side_0}, open={open_side_0}",
    )

    rest_side_1 = aabb_center(side_panel_1)
    with ctx.pose({side_joint_1: 1.20}):
        open_side_1 = aabb_center(side_panel_1)
    ctx.check(
        "side panel 1 opens outward",
        rest_side_1 is not None and open_side_1 is not None and open_side_1[1] < rest_side_1[1] - 0.20,
        details=f"rest={rest_side_1}, open={open_side_1}",
    )

    rest_front = aabb_center(front_door)
    with ctx.pose({front_joint: 1.20}):
        open_front = aabb_center(front_door)
    ctx.check(
        "front door opens forward",
        rest_front is not None and open_front is not None and open_front[0] < rest_front[0] - 0.20,
        details=f"rest={rest_front}, open={open_front}",
    )

    rest_cable = aabb_center(cable)
    with ctx.pose({cable_joint: 1.00}):
        open_cable = aabb_center(cable)
    ctx.check(
        "rear cable bracket swings rearward",
        rest_cable is not None and open_cable is not None and open_cable[0] > rest_cable[0] + 0.10,
        details=f"rest={rest_cable}, open={open_cable}",
    )

    return ctx.report()


object_model = build_object_model()
