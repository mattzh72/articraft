import cadquery as cq
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
    model = ArticulatedObject(name="passenger_elevator")

    # Dimensions
    travel_height = 8.0
    cab_w = 2.6
    cab_d = 1.8
    cab_h = 2.6
    wall_t = 0.05
    door_w = 0.61
    door_h = 2.1
    door_t = 0.04
    rail_h = travel_height + cab_h + 1.0

    # 1. Guide Structure
    guide = model.part("guide_structure")
    
    # Base pad
    guide.visual(
        Box((3.5, 3.0, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        name="base_pad",
        color=(0.3, 0.3, 0.3)
    )
    
    # Left rail (T-shape)
    guide.visual(
        Box((0.02, 0.1, rail_h)),
        origin=Origin(xyz=(-cab_w/2 - 0.08, 0.0, rail_h/2 + 0.2)),
        name="left_rail_base",
        color=(0.3, 0.3, 0.3)
    )
    guide.visual(
        Box((0.06, 0.02, rail_h)),
        origin=Origin(xyz=(-cab_w/2 - 0.04, 0.0, rail_h/2 + 0.2)),
        name="left_rail_stem",
        color=(0.7, 0.7, 0.7)
    )
    
    # Right rail (T-shape)
    guide.visual(
        Box((0.02, 0.1, rail_h)),
        origin=Origin(xyz=(cab_w/2 + 0.08, 0.0, rail_h/2 + 0.2)),
        name="right_rail_base",
        color=(0.3, 0.3, 0.3)
    )
    guide.visual(
        Box((0.06, 0.02, rail_h)),
        origin=Origin(xyz=(cab_w/2 + 0.04, 0.0, rail_h/2 + 0.2)),
        name="right_rail_stem",
        color=(0.7, 0.7, 0.7)
    )

    # Back vertical beams
    guide.visual(
        Box((0.1, 0.1, rail_h)),
        origin=Origin(xyz=(-cab_w/2, -cab_d/2 - 0.2, rail_h/2 + 0.2)),
        name="back_left_beam",
        color=(0.4, 0.4, 0.4)
    )
    guide.visual(
        Box((0.1, 0.1, rail_h)),
        origin=Origin(xyz=(cab_w/2, -cab_d/2 - 0.2, rail_h/2 + 0.2)),
        name="back_right_beam",
        color=(0.4, 0.4, 0.4)
    )

    # Top crossbeam / machine room floor
    guide.visual(
        Box((3.5, 3.0, 0.4)),
        origin=Origin(xyz=(0.0, 0.0, rail_h + 0.4)),
        name="top_beam",
        color=(0.2, 0.2, 0.2)
    )

    # Hoist machine / pulley
    guide.visual(
        Cylinder(radius=0.3, length=0.4),
        origin=Origin(xyz=(0.0, 0.0, rail_h + 0.6 + 0.3), rpy=(0.0, 1.5708, 0.0)),
        name="hoist_pulley",
        color=(0.8, 0.5, 0.1)
    )

    # 2. Cab
    cab = model.part("cab")
    
    # Floor
    cab.visual(
        Box((cab_w, cab_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t/2)),
        name="floor",
        color=(0.2, 0.2, 0.2)
    )
    # Floor finish
    cab.visual(
        Box((cab_w - 2*wall_t, cab_d - 2*wall_t, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, wall_t + 0.005)),
        name="floor_finish",
        color=(0.15, 0.15, 0.15)
    )
    # Ceiling
    cab.visual(
        Box((cab_w, cab_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, cab_h - wall_t/2)),
        name="ceiling",
        color=(0.9, 0.9, 0.9)
    )
    # Back wall
    cab.visual(
        Box((cab_w, wall_t, cab_h - 2*wall_t)),
        origin=Origin(xyz=(0.0, -cab_d/2 + wall_t/2, cab_h/2)),
        name="back_wall",
        color=(0.8, 0.8, 0.85)
    )
    # Left wall
    cab.visual(
        Box((wall_t, cab_d - wall_t, cab_h - 2*wall_t)),
        origin=Origin(xyz=(-cab_w/2 + wall_t/2, 0.0, cab_h/2)),
        name="left_wall",
        color=(0.8, 0.8, 0.85)
    )
    # Right wall
    cab.visual(
        Box((wall_t, cab_d - wall_t, cab_h - 2*wall_t)),
        origin=Origin(xyz=(cab_w/2 - wall_t/2, 0.0, cab_h/2)),
        name="right_wall",
        color=(0.8, 0.8, 0.85)
    )
    
    # Front wall panels
    front_panel_w = (cab_w - 1.2) / 2  # 0.7
    cab.visual(
        Box((front_panel_w, wall_t, cab_h - 2*wall_t)),
        origin=Origin(xyz=(-cab_w/2 + front_panel_w/2, cab_d/2 - wall_t/2, cab_h/2)),
        name="front_left_wall",
        color=(0.7, 0.7, 0.7)
    )
    cab.visual(
        Box((front_panel_w, wall_t, cab_h - 2*wall_t)),
        origin=Origin(xyz=(cab_w/2 - front_panel_w/2, cab_d/2 - wall_t/2, cab_h/2)),
        name="front_right_wall",
        color=(0.7, 0.7, 0.7)
    )
    top_panel_h = cab_h - 2*wall_t - door_h
    cab.visual(
        Box((1.2, wall_t, top_panel_h)),
        origin=Origin(xyz=(0.0, cab_d/2 - wall_t/2, cab_h - wall_t - top_panel_h/2)),
        name="front_top_wall",
        color=(0.7, 0.7, 0.7)
    )

    # Door frame (outside of front wall)
    cab.visual(
        Box((0.05, 0.02, door_h)),
        origin=Origin(xyz=(-0.625, cab_d/2 + 0.01, wall_t + door_h/2)),
        name="frame_left",
        color=(0.8, 0.8, 0.8)
    )
    cab.visual(
        Box((0.05, 0.02, door_h)),
        origin=Origin(xyz=(0.625, cab_d/2 + 0.01, wall_t + door_h/2)),
        name="frame_right",
        color=(0.8, 0.8, 0.8)
    )
    cab.visual(
        Box((1.3, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, cab_d/2 + 0.01, wall_t + door_h + 0.025)),
        name="frame_top",
        color=(0.8, 0.8, 0.8)
    )

    # Interior details
    cab.visual(
        Box((1.8, 0.01, 1.2)),
        origin=Origin(xyz=(0.0, -cab_d/2 + wall_t + 0.005, 1.4)),
        name="mirror",
        color=(0.9, 0.9, 0.95)
    )
    cab.visual(
        Cylinder(radius=0.02, length=cab_w - 0.4),
        origin=Origin(xyz=(0.0, -cab_d/2 + wall_t + 0.05, 0.9), rpy=(0.0, 1.5708, 0.0)),
        name="handrail",
        color=(0.8, 0.8, 0.8)
    )
    cab.visual(
        Cylinder(radius=0.015, length=0.05),
        origin=Origin(xyz=(-cab_w/2 + 0.4, -cab_d/2 + wall_t + 0.025, 0.9), rpy=(1.5708, 0.0, 0.0)),
        name="handrail_mount_left",
        color=(0.8, 0.8, 0.8)
    )
    cab.visual(
        Cylinder(radius=0.015, length=0.05),
        origin=Origin(xyz=(cab_w/2 - 0.4, -cab_d/2 + wall_t + 0.025, 0.9), rpy=(1.5708, 0.0, 0.0)),
        name="handrail_mount_right",
        color=(0.8, 0.8, 0.8)
    )
    cab.visual(
        Box((0.02, 0.2, 0.6)),
        origin=Origin(xyz=(cab_w/2 - wall_t - 0.01, 0.0, 1.2)),
        name="control_panel",
        color=(0.1, 0.1, 0.1)
    )

    # Cables
    cab.visual(
        Cylinder(radius=0.02, length=rail_h),
        origin=Origin(xyz=(-0.1, 0.0, cab_h + rail_h/2)),
        name="hoist_cable_1",
        color=(0.1, 0.1, 0.1)
    )
    cab.visual(
        Cylinder(radius=0.02, length=rail_h),
        origin=Origin(xyz=(0.1, 0.0, cab_h + rail_h/2)),
        name="hoist_cable_2",
        color=(0.1, 0.1, 0.1)
    )

    # Guide shoes
    def add_guide_shoe(z_pos: float, is_left: bool):
        sign = -1 if is_left else 1
        prefix = "left" if is_left else "right"
        cab.visual(
            Box((0.01, 0.12, 0.15)),
            origin=Origin(xyz=(sign * (cab_w/2 + 0.005), 0.0, z_pos)),
            name=f"{prefix}_shoe_mount_{z_pos}",
            color=(0.2, 0.2, 0.2)
        )
        cab.visual(
            Box((0.06, 0.02, 0.15)),
            origin=Origin(xyz=(sign * (cab_w/2 + 0.04), 0.02, z_pos)),
            name=f"{prefix}_shoe_front_{z_pos}",
            color=(0.2, 0.2, 0.2)
        )
        cab.visual(
            Box((0.06, 0.02, 0.15)),
            origin=Origin(xyz=(sign * (cab_w/2 + 0.04), -0.02, z_pos)),
            name=f"{prefix}_shoe_back_{z_pos}",
            color=(0.2, 0.2, 0.2)
        )

    add_guide_shoe(0.4, True)
    add_guide_shoe(cab_h - 0.4, True)
    add_guide_shoe(0.4, False)
    add_guide_shoe(cab_h - 0.4, False)

    # Cab Articulation
    model.articulation(
        "cab_lift",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=cab,
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10000.0, velocity=2.0, lower=0.0, upper=travel_height)
    )

    # 3. Doors
    left_door = model.part("left_door")
    left_door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="panel",
        color=(0.6, 0.6, 0.65)
    )
    model.articulation(
        "left_door_slide",
        ArticulationType.PRISMATIC,
        parent=cab,
        child=left_door,
        origin=Origin(xyz=(-0.31, cab_d/2 - wall_t - door_t/2 - 0.01, wall_t + 0.01 + door_h/2)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.6)
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="panel",
        color=(0.6, 0.6, 0.65)
    )
    model.articulation(
        "right_door_slide",
        ArticulationType.PRISMATIC,
        parent=cab,
        child=right_door,
        origin=Origin(xyz=(0.31, cab_d/2 - wall_t - door_t/2 - 0.01, wall_t + 0.01 + door_h/2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.6)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cab")
    guide = object_model.get_part("guide_structure")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    # Allow hoist cables to pass through the machine room floor
    ctx.allow_overlap(cab, guide, elem_a="hoist_cable_1", elem_b="top_beam", reason="Cables pass through floor.")
    ctx.allow_overlap(cab, guide, elem_a="hoist_cable_2", elem_b="top_beam", reason="Cables pass through floor.")
    ctx.allow_overlap(cab, guide, elem_a="hoist_cable_1", elem_b="hoist_pulley", reason="Cables wrap around pulley.")
    ctx.allow_overlap(cab, guide, elem_a="hoist_cable_2", elem_b="hoist_pulley", reason="Cables wrap around pulley.")

    ctx.expect_within(cab, guide, axes="xy", name="cab stays within guide rails footprint")
    ctx.expect_contact(cab, guide, elem_a="floor", elem_b="base_pad", name="cab rests on base pad")
    ctx.expect_gap(right_door, left_door, axis="x", min_gap=0.005, max_gap=0.02, name="doors meet in the middle")
    
    cab_lift = object_model.get_articulation("cab_lift")
    l_door_slide = object_model.get_articulation("left_door_slide")
    r_door_slide = object_model.get_articulation("right_door_slide")

    with ctx.pose({cab_lift: 8.0, l_door_slide: 0.6, r_door_slide: 0.6}):
        ctx.expect_gap(right_door, left_door, axis="x", min_gap=1.0, name="doors open wide")
        cab_pos = ctx.part_world_position(cab)
        ctx.check("cab moves up", cab_pos is not None and cab_pos[2] > 5.0)

    return ctx.report()


object_model = build_object_model()
