from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="wall_mounted_mailbox")

    painted_metal = model.material("painted_dark_green", rgba=(0.06, 0.20, 0.14, 1.0))
    plate_metal = model.material("slightly_darker_green", rgba=(0.04, 0.14, 0.10, 1.0))
    hinge_metal = model.material("blackened_hinge_metal", rgba=(0.015, 0.017, 0.018, 1.0))
    lock_metal = model.material("brushed_lock_metal", rgba=(0.72, 0.67, 0.55, 1.0))
    slot_dark = model.material("keyway_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    housing = model.part("housing")
    # Overall mailbox body: 0.42 m wide, 0.18 m deep, and 0.30 m tall.
    # The larger rear plate is the wall-mounting plate visible around the box.
    housing.visual(
        Box((0.50, 0.012, 0.38)),
        origin=Origin(xyz=(0.0, -0.096, 0.150)),
        material=plate_metal,
        name="back_plate",
    )
    housing.visual(
        Box((0.42, 0.014, 0.30)),
        origin=Origin(xyz=(0.0, -0.083, 0.150)),
        material=painted_metal,
        name="back_wall",
    )
    housing.visual(
        Box((0.014, 0.18, 0.30)),
        origin=Origin(xyz=(-0.203, 0.0, 0.150)),
        material=painted_metal,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.014, 0.18, 0.30)),
        origin=Origin(xyz=(0.203, 0.0, 0.150)),
        material=painted_metal,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.42, 0.18, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.293)),
        material=painted_metal,
        name="top_wall",
    )
    housing.visual(
        Box((0.42, 0.18, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=painted_metal,
        name="bottom_wall",
    )
    # Exposed screw heads on the wall plate make the mounting purpose explicit.
    for name, x, z in (
        ("mount_screw_0", -0.205, 0.305),
        ("mount_screw_1", 0.205, 0.305),
        ("mount_screw_2", -0.205, -0.005),
        ("mount_screw_3", 0.205, -0.005),
    ):
        housing.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(x, -0.094, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=lock_metal,
            name=name,
        )

    hinge_y = 0.105
    hinge_z = 0.006
    for name, x in (("hinge_barrel_0", -0.125), ("hinge_barrel_1", 0.125)):
        housing.visual(
            Cylinder(radius=0.010, length=0.130),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=name,
        )
    for name, x in (("hinge_leaf_0", -0.125), ("hinge_leaf_1", 0.125)):
        housing.visual(
            Box((0.130, 0.010, 0.018)),
            origin=Origin(xyz=(x, 0.095, 0.014)),
            material=hinge_metal,
            name=name,
        )

    door = model.part("front_door")
    # The door part frame is the lower hinge line. At q=0 the panel stands
    # vertically in front of the open box and extends upward from the hinge.
    door.visual(
        Box((0.390, 0.018, 0.280)),
        origin=Origin(xyz=(0.0, 0.006, 0.152)),
        material=painted_metal,
        name="door_panel",
    )
    door.visual(
        Box((0.366, 0.005, 0.018)),
        origin=Origin(xyz=(0.0, 0.017, 0.272)),
        material=plate_metal,
        name="top_raised_rim",
    )
    door.visual(
        Box((0.366, 0.005, 0.016)),
        origin=Origin(xyz=(0.0, 0.017, 0.037)),
        material=plate_metal,
        name="bottom_raised_rim",
    )
    door.visual(
        Box((0.018, 0.005, 0.235)),
        origin=Origin(xyz=(-0.174, 0.017, 0.154)),
        material=plate_metal,
        name="side_raised_rim_0",
    )
    door.visual(
        Box((0.018, 0.005, 0.235)),
        origin=Origin(xyz=(0.174, 0.017, 0.154)),
        material=plate_metal,
        name="side_raised_rim_1",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="center_hinge_barrel",
    )
    door.visual(
        Box((0.100, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.014)),
        material=hinge_metal,
        name="center_hinge_leaf",
    )

    door_hinge = model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    lock_cam = model.part("lock_cam")
    # Child frame is the lock rotation axis. Local +Y is front-to-back through
    # the door, so the cam blade rotates like a real quarter-turn cam lock.
    lock_cam.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=lock_metal,
        name="lock_barrel",
    )
    lock_cam.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=lock_metal,
        name="lock_face",
    )
    lock_cam.visual(
        Box((0.004, 0.0015, 0.020)),
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=slot_dark,
        name="key_slot",
    )
    lock_cam.visual(
        Box((0.018, 0.006, 0.076)),
        origin=Origin(xyz=(0.0, -0.030, -0.038)),
        material=lock_metal,
        name="cam_blade",
    )

    model.articulation(
        "door_to_lock_cam",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock_cam,
        origin=Origin(xyz=(0.145, 0.006, 0.235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=pi / 2.0),
    )

    # Keep local references alive for static analysis/lint friendliness when the
    # SDK returns articulation objects.
    _ = door_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("front_door")
    lock_cam = object_model.get_part("lock_cam")
    door_hinge = object_model.get_articulation("housing_to_door")
    lock_joint = object_model.get_articulation("door_to_lock_cam")

    ctx.allow_overlap(
        door,
        lock_cam,
        elem_a="door_panel",
        elem_b="lock_barrel",
        reason="The cam-lock barrel is intentionally captured through the flat door panel.",
    )
    ctx.expect_overlap(
        lock_cam,
        door,
        axes="xz",
        elem_a="lock_barrel",
        elem_b="door_panel",
        min_overlap=0.010,
        name="lock barrel passes through the door panel footprint",
    )
    ctx.expect_contact(
        lock_cam,
        door,
        elem_a="lock_face",
        elem_b="door_panel",
        contact_tol=0.001,
        name="lock face sits against the front door skin",
    )

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        negative_elem="top_wall",
        min_gap=0.005,
        max_gap=0.020,
        name="closed door stands just proud of the box front",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="door_panel",
        elem_b="back_wall",
        min_overlap=0.25,
        name="flat front door covers the rectangular mailbox opening",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.70}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "lower-edge hinge swings the door outward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.10
        and open_aabb[1][2] < closed_aabb[1][2] - 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    blade_closed = ctx.part_element_world_aabb(lock_cam, elem="cam_blade")
    with ctx.pose({lock_joint: pi / 2.0}):
        blade_rotated = ctx.part_element_world_aabb(lock_cam, elem="cam_blade")
    ctx.check(
        "cam blade quarter-turns about a front-to-back axis",
        blade_closed is not None
        and blade_rotated is not None
        and (blade_rotated[1][0] - blade_rotated[0][0])
        > (blade_closed[1][0] - blade_closed[0][0]) + 0.040
        and (blade_closed[1][2] - blade_closed[0][2])
        > (blade_rotated[1][2] - blade_rotated[0][2]) + 0.040,
        details=f"closed_blade={blade_closed}, rotated_blade={blade_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
