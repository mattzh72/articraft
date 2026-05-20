from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_truck_glove_compartment")

    textured_black = model.material("grained_black_plastic", color=(0.035, 0.038, 0.040, 1.0))
    dark_insert = model.material("dark_recess", color=(0.010, 0.011, 0.012, 1.0))
    rubber = model.material("rubber_stops", color=(0.015, 0.015, 0.013, 1.0))
    hinge_metal = model.material("blackened_steel", color=(0.08, 0.075, 0.065, 1.0))
    link_metal = model.material("zinc_limiter_links", color=(0.62, 0.58, 0.50, 1.0))

    # Object frame: X is storage-bin depth into the dashboard, Y is the hinge
    # axis across the truck, and Z is up.  The top hinge line is the origin.
    bin_part = model.part("storage_bin")

    # Shallow open box, assembled from visibly connected wall and flange members.
    bin_part.visual(
        Box((0.232, 0.525, 0.018)),
        origin=Origin(xyz=(0.139, 0.0, 0.009)),
        material=textured_black,
        name="top_wall",
    )
    bin_part.visual(
        Box((0.255, 0.525, 0.020)),
        origin=Origin(xyz=(0.1275, 0.0, -0.230)),
        material=textured_black,
        name="bottom_wall",
    )
    for name, y in (("side_wall_0", -0.270), ("side_wall_1", 0.270)):
        bin_part.visual(
            Box((0.255, 0.020, 0.235)),
            origin=Origin(xyz=(0.1275, y, -0.1125)),
            material=textured_black,
            name=name,
        )
    bin_part.visual(
        Box((0.024, 0.525, 0.235)),
        origin=Origin(xyz=(0.267, 0.0, -0.1125)),
        material=textured_black,
        name="rear_wall",
    )

    # Raised dashboard surround: a boxy utility-truck front frame around the
    # storage opening.
    bin_part.visual(
        Box((0.018, 0.630, 0.033)),
        origin=Origin(xyz=(0.022, 0.0, 0.0155)),
        material=textured_black,
        name="upper_bezel",
    )
    bin_part.visual(
        Box((0.018, 0.630, 0.030)),
        origin=Origin(xyz=(0.003, 0.0, -0.254)),
        material=textured_black,
        name="lower_bezel",
    )
    for idx, y in enumerate((-0.304, 0.304)):
        bin_part.visual(
            Box((0.018, 0.028, 0.272)),
            origin=Origin(xyz=(0.003, y, -0.119)),
            material=textured_black,
            name=f"side_bezel_{idx}",
        )

    # Fixed hinge knuckles on the top lip.  The door knuckles occupy the gaps.
    for idx, (y, length) in enumerate(((-0.245, 0.090), (0.0, 0.155), (0.245, 0.090))):
        bin_part.visual(
            Cylinder(radius=0.012, length=length),
            origin=Origin(xyz=(-0.004, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"fixed_hinge_{idx}",
        )
        bin_part.visual(
            Box((0.020, length, 0.014)),
            origin=Origin(xyz=(0.006, y, 0.003)),
            material=textured_black,
            name=f"hinge_mount_{idx}",
        )

    # Rubber bumpers at the lower sill keep the door from reading as embedded.
    for idx, y in enumerate((-0.205, 0.205)):
        bin_part.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.008, y, -0.236), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"bumper_{idx}",
        )

    # Wall-side pivot pins for the twin limiter links.
    box_pivot_x = 0.075
    box_pivot_z = -0.155
    link_offset_y = 0.329
    for name, sign in (("wall_pivot_0", -1.0), ("wall_pivot_1", 1.0)):
        bin_part.visual(
            Cylinder(radius=0.016, length=0.044),
            origin=Origin(
                xyz=(box_pivot_x, sign * 0.302, box_pivot_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=name,
        )

    door = model.part("door")
    door.visual(
        Box((0.026, 0.574, 0.216)),
        origin=Origin(xyz=(-0.015, 0.0, -0.128)),
        material=textured_black,
        name="door_panel",
    )
    # Slightly inset molded pull pocket on the front face.
    door.visual(
        Box((0.004, 0.190, 0.040)),
        origin=Origin(xyz=(-0.030, 0.0, -0.143)),
        material=dark_insert,
        name="pull_recess",
    )
    door.visual(
        Box((0.005, 0.150, 0.009)),
        origin=Origin(xyz=(-0.030, 0.0, -0.119)),
        material=textured_black,
        name="pull_lip",
    )

    # Moving hinge knuckles attached to the upper edge of the panel.
    for idx, (y, length) in enumerate(((-0.145, 0.095), (0.145, 0.095))):
        door.visual(
            Cylinder(radius=0.0115, length=length),
            origin=Origin(xyz=(-0.006, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"door_hinge_{idx}",
        )
        door.visual(
            Box((0.016, 0.050, 0.018)),
            origin=Origin(xyz=(-0.012, y, -0.019)),
            material=textured_black,
            name=f"hinge_tab_{idx}",
        )

    # Door-side pivot studs, just inboard of the lower corners, for the limiter
    # link eyes to bear against when the door is shut.
    door_pivot_x = -0.050
    door_pivot_z = -0.190
    for name, sign in (("pivot_stud_0", -1.0), ("pivot_stud_1", 1.0)):
        door.visual(
            Box((0.026, 0.029, 0.030)),
            origin=Origin(xyz=(-0.039, sign * 0.3005, door_pivot_z)),
            material=textured_black,
            name=f"pivot_ear_{0 if sign < 0 else 1}",
        )
        door.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(
                xyz=(door_pivot_x, sign * 0.319, door_pivot_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=name,
        )

    door_hinge = model.articulation(
        "bin_to_door",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    # The limiter links are short strap links with round eyes.  They are parented
    # to the fixed bin pivots and mimic the door hinge enough to show the
    # coupled swing without trying to model an impossible closed-loop URDF.
    dx = door_pivot_x - box_pivot_x
    dz = door_pivot_z - box_pivot_z
    link_length = math.sqrt(dx * dx + dz * dz)
    link_angle = math.atan2(-dz, dx)

    for idx, sign in enumerate((-1.0, 1.0)):
        link = model.part(f"limiter_{idx}")
        link.visual(
            Box((link_length, 0.010, 0.008)),
            origin=Origin(xyz=(link_length / 2.0, 0.0, 0.0)),
            material=link_metal,
            name="strap",
        )
        link.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=link_metal,
            name="wall_eye",
        )
        link.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(link_length, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=link_metal,
            name="door_eye",
        )
        model.articulation(
            f"bin_to_limiter_{idx}",
            ArticulationType.REVOLUTE,
            parent=bin_part,
            child=link,
            origin=Origin(xyz=(box_pivot_x, sign * link_offset_y, box_pivot_z), rpy=(0.0, link_angle, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=0.65),
            mimic=Mimic(joint=door_hinge.name, multiplier=0.55, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    storage_bin = object_model.get_part("storage_bin")
    door = object_model.get_part("door")
    limiter_0 = object_model.get_part("limiter_0")
    limiter_1 = object_model.get_part("limiter_1")
    door_hinge = object_model.get_articulation("bin_to_door")
    link_joint_0 = object_model.get_articulation("bin_to_limiter_0")
    link_joint_1 = object_model.get_articulation("bin_to_limiter_1")

    ctx.check(
        "top hinge has horizontal axis",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "limiter links mimic the door swing",
        link_joint_0.mimic is not None and link_joint_1.mimic is not None,
        details=f"mimics={link_joint_0.mimic}, {link_joint_1.mimic}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            storage_bin,
            axes="yz",
            elem_a="door_panel",
            elem_b="rear_wall",
            min_overlap=0.20,
            name="closed door covers the shallow bin opening",
        )
        ctx.expect_gap(
            storage_bin,
            door,
            axis="x",
            positive_elem="side_wall_0",
            negative_elem="door_panel",
            min_gap=0.001,
            max_gap=0.006,
            name="closed door sits just proud of bin front",
        )
        ctx.expect_contact(
            limiter_0,
            storage_bin,
            elem_a="wall_eye",
            elem_b="wall_pivot_0",
            contact_tol=0.0015,
            name="first limiter bears on box wall pivot",
        )
        ctx.expect_contact(
            limiter_1,
            storage_bin,
            elem_a="wall_eye",
            elem_b="wall_pivot_1",
            contact_tol=0.0015,
            name="second limiter bears on box wall pivot",
        )
        ctx.expect_contact(
            limiter_0,
            door,
            elem_a="door_eye",
            elem_b="pivot_stud_0",
            contact_tol=0.0015,
            name="first limiter reaches lower door pivot",
        )
        ctx.expect_contact(
            limiter_1,
            door,
            elem_a="door_eye",
            elem_b="pivot_stud_1",
            contact_tol=0.0015,
            name="second limiter reaches lower door pivot",
        )
        closed_door_box = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_link_box = ctx.part_element_world_aabb(limiter_0, elem="door_eye")

    with ctx.pose({door_hinge: 1.05}):
        opened_door_box = ctx.part_element_world_aabb(door, elem="door_panel")
        opened_link_box = ctx.part_element_world_aabb(limiter_0, elem="door_eye")

    if closed_door_box is not None and opened_door_box is not None:
        closed_min, _closed_max = closed_door_box
        opened_min, _opened_max = opened_door_box
        ctx.check(
            "door rotates upward and outward",
            opened_min[2] > closed_min[2] + 0.070 and opened_min[0] < closed_min[0] - 0.080,
            details=f"closed_min={closed_min}, opened_min={opened_min}",
        )
    else:
        ctx.fail("door pose aabbs available", "door panel AABBs were not available")

    if closed_link_box is not None and opened_link_box is not None:
        cmin, cmax = closed_link_box
        omin, omax = opened_link_box
        closed_center_z = 0.5 * (cmin[2] + cmax[2])
        opened_center_z = 0.5 * (omin[2] + omax[2])
        ctx.check(
            "limiter link rotates upward with door",
            opened_center_z > closed_center_z + 0.015,
            details=f"closed_z={closed_center_z}, opened_z={opened_center_z}",
        )
    else:
        ctx.fail("limiter pose aabbs available", "limiter link AABBs were not available")

    return ctx.report()


object_model = build_object_model()
