from __future__ import annotations

from math import isclose

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
    model = ArticulatedObject(name="tall_pantry_cabinet")

    oak = model.material("warm_oak", color=(0.72, 0.46, 0.24, 1.0))
    oak_dark = model.material("end_grain", color=(0.45, 0.25, 0.12, 1.0))
    interior = model.material("shadowed_interior", color=(0.30, 0.23, 0.18, 1.0))
    brass = model.material("aged_brass", color=(0.78, 0.60, 0.27, 1.0))
    steel = model.material("brushed_steel", color=(0.72, 0.70, 0.65, 1.0))

    width = 0.80
    depth = 0.45
    height = 2.10
    wall = 0.04
    front_y = -depth / 2.0
    back_y = depth / 2.0
    hinge_x = -width / 2.0 - 0.015
    hinge_y = front_y - 0.060

    carcass = model.part("carcass")
    # Full-height rectangular carcass: sides, back, top/bottom, and the
    # horizontal divider rail/shelf that visibly separates the two compartments.
    carcass.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=oak_dark,
        name="side_wall_0",
    )
    carcass.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=oak_dark,
        name="side_wall_1",
    )
    carcass.visual(
        Box((width, wall * 0.70, height)),
        origin=Origin(xyz=(0.0, back_y - wall * 0.35, height / 2.0)),
        material=interior,
        name="back_panel",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=oak_dark,
        name="bottom_panel",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=oak_dark,
        name="top_panel",
    )
    carcass.visual(
        Box((width, depth, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
        material=oak_dark,
        name="divider_rail",
    )
    # Face-frame strips make the carcass read as a cabinet rather than a plain
    # box and give the closed doors a reveal to sit proud of.
    carcass.visual(
        Box((0.055, 0.035, height)),
        origin=Origin(xyz=(-width / 2.0 + 0.0275, front_y - 0.0175, height / 2.0)),
        material=oak_dark,
        name="front_stile_0",
    )
    carcass.visual(
        Box((0.055, 0.035, height)),
        origin=Origin(xyz=(width / 2.0 - 0.0275, front_y - 0.0175, height / 2.0)),
        material=oak_dark,
        name="front_stile_1",
    )
    carcass.visual(
        Box((width, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, front_y - 0.0175, height / 2.0)),
        material=oak_dark,
        name="front_divider_rail",
    )
    carcass.visual(
        Box((width, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, front_y - 0.0175, 0.025)),
        material=oak_dark,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((width, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, front_y - 0.0175, height - 0.025)),
        material=oak_dark,
        name="front_top_rail",
    )

    def add_stationary_hinge_set(prefix: str, z_center: float) -> None:
        for idx, hinge_z in enumerate((z_center - 0.315, z_center + 0.315)):
            for segment_name, dz in (("lower", -0.053), ("upper", 0.053)):
                carcass.visual(
                    Cylinder(radius=0.012, length=0.048),
                    origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + dz)),
                    material=brass,
                    name=f"{prefix}_hinge_{idx}_{segment_name}_knuckle",
                )
                carcass.visual(
                    Box((0.008, 0.058, 0.048)),
                    origin=Origin(xyz=(hinge_x + 0.012, hinge_y + 0.031, hinge_z + dz)),
                    material=brass,
                    name=f"{prefix}_hinge_{idx}_{segment_name}_side_leaf",
                )

    lower_center = 0.535
    upper_center = 1.565
    add_stationary_hinge_set("lower", lower_center)
    add_stationary_hinge_set("upper", upper_center)

    door_width = 0.755
    door_thick = 0.030
    lower_height = 0.980
    upper_height = 0.980

    def build_door(part_name: str, door_height: float) -> object:
        door = model.part(part_name)
        door.visual(
            Box((door_width, door_thick, door_height)),
            origin=Origin(xyz=(0.018 + door_width / 2.0, 0.0, 0.0)),
            material=oak,
            name="door_panel",
        )
        # Raised frame on the proud face with a darker inset field.
        face_y = -door_thick / 2.0 - 0.004
        door.visual(
            Box((door_width - 0.060, 0.010, door_height - 0.180)),
            origin=Origin(xyz=(0.018 + door_width / 2.0, face_y - 0.001, 0.0)),
            material=oak_dark,
            name="recessed_field",
        )
        for side_name, x in (("hinge_stile", 0.018 + 0.038), ("latch_stile", 0.018 + door_width - 0.038)):
            door.visual(
                Box((0.076, 0.018, door_height)),
                origin=Origin(xyz=(x, face_y, 0.0)),
                material=oak,
                name=side_name,
            )
        for rail_name, z in (("top_rail", door_height / 2.0 - 0.045), ("bottom_rail", -door_height / 2.0 + 0.045)):
            door.visual(
                Box((door_width, 0.018, 0.090)),
                origin=Origin(xyz=(0.018 + door_width / 2.0, face_y, z)),
                material=oak,
                name=rail_name,
            )
        for i, x in enumerate((0.22, 0.39, 0.56)):
            door.visual(
                Box((0.010, 0.006, door_height - 0.250)),
                origin=Origin(xyz=(x, face_y - 0.008, 0.0)),
                material=oak_dark,
                name=f"grain_line_{i}",
            )

        # Door-side hinge leaves and middle knuckles.  They sit between the
        # carcass-side upper/lower knuckles with visible air gaps, so the door is
        # clipped into the hardware while still rotating around its own axis.
        for idx, hinge_z in enumerate((-0.315, 0.315)):
            door.visual(
                Cylinder(radius=0.012, length=0.058),
                origin=Origin(xyz=(0.0, 0.0, hinge_z)),
                material=brass,
                name=f"hinge_{idx}_door_knuckle",
            )
            door.visual(
                Box((0.066, 0.014, 0.058)),
                origin=Origin(xyz=(0.033, -0.007, hinge_z)),
                material=brass,
                name=f"hinge_{idx}_door_leaf",
            )

        handle_x = 0.018 + door_width - 0.135
        for mount_name, z in (("handle_mount_0", -0.180), ("handle_mount_1", 0.180)):
            door.visual(
                Box((0.075, 0.014, 0.070)),
                origin=Origin(xyz=(handle_x, -door_thick / 2.0 - 0.007, z)),
                material=steel,
                name=mount_name,
            )
        return door

    lower_door = build_door("lower_door", lower_height)
    upper_door = build_door("upper_door", upper_height)

    def build_handle(part_name: str) -> object:
        handle = model.part(part_name)
        handle.visual(
            Cylinder(radius=0.012, length=0.420),
            origin=Origin(xyz=(0.0, -0.058, 0.0)),
            material=steel,
            name="grip",
        )
        for arm_name, z in (("arm_0", -0.180), ("arm_1", 0.180)):
            handle.visual(
                Box((0.018, 0.058, 0.018)),
                origin=Origin(xyz=(0.0, -0.029, z)),
                material=steel,
                name=arm_name,
            )
        return handle

    lower_handle = build_handle("lower_handle")
    upper_handle = build_handle("upper_handle")

    door_limits = MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75)
    model.articulation(
        "carcass_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lower_door,
        origin=Origin(xyz=(hinge_x, hinge_y, lower_center)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=door_limits,
    )
    model.articulation(
        "carcass_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=upper_door,
        origin=Origin(xyz=(hinge_x, hinge_y, upper_center)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=door_limits,
    )

    handle_x = 0.018 + door_width - 0.135
    # Put the handle part frame exactly on the front face of the two fixed
    # mount pads so the articulated pull is supported but not embedded.
    handle_y = -door_thick / 2.0 - 0.014
    handle_limits = MotionLimits(effort=1.0, velocity=1.0, lower=-0.25, upper=0.25)
    model.articulation(
        "lower_door_to_handle",
        ArticulationType.REVOLUTE,
        parent=lower_door,
        child=lower_handle,
        origin=Origin(xyz=(handle_x, handle_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=handle_limits,
    )
    model.articulation(
        "upper_door_to_handle",
        ArticulationType.REVOLUTE,
        parent=upper_door,
        child=upper_handle,
        origin=Origin(xyz=(handle_x, handle_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=handle_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    lower_door = object_model.get_part("lower_door")
    upper_door = object_model.get_part("upper_door")
    lower_handle = object_model.get_part("lower_handle")
    upper_handle = object_model.get_part("upper_handle")
    lower_hinge = object_model.get_articulation("carcass_to_lower_door")
    upper_hinge = object_model.get_articulation("carcass_to_upper_door")
    lower_handle_joint = object_model.get_articulation("lower_door_to_handle")
    upper_handle_joint = object_model.get_articulation("upper_door_to_handle")

    ctx.expect_overlap(
        lower_door,
        carcass,
        axes="x",
        min_overlap=0.70,
        elem_a="door_panel",
        elem_b="back_panel",
        name="lower door spans the cabinet width",
    )
    ctx.expect_overlap(
        upper_door,
        carcass,
        axes="x",
        min_overlap=0.70,
        elem_a="door_panel",
        elem_b="back_panel",
        name="upper door spans the cabinet width",
    )
    for door, label in ((lower_door, "lower"), (upper_door, "upper")):
        for idx in (0, 1):
            ctx.expect_overlap(
                door,
                carcass,
                axes="xy",
                min_overlap=0.016,
                elem_a=f"hinge_{idx}_door_knuckle",
                elem_b=f"{label}_hinge_{idx}_lower_knuckle",
                name=f"{label} door hinge {idx} shares the fixed pin line",
            )

    lower_closed = ctx.part_element_world_aabb(lower_door, elem="door_panel")
    upper_closed = ctx.part_element_world_aabb(upper_door, elem="door_panel")
    lower_handle_closed = ctx.part_element_world_aabb(lower_handle, elem="grip")
    upper_handle_closed = ctx.part_element_world_aabb(upper_handle, elem="grip")
    with ctx.pose({upper_hinge: 1.20}):
        upper_open = ctx.part_element_world_aabb(upper_door, elem="door_panel")
        lower_still = ctx.part_element_world_aabb(lower_door, elem="door_panel")
    with ctx.pose({lower_hinge: 1.20}):
        lower_open = ctx.part_element_world_aabb(lower_door, elem="door_panel")
    with ctx.pose({upper_handle_joint: 0.22}):
        upper_handle_rotated = ctx.part_element_world_aabb(upper_handle, elem="grip")
    with ctx.pose({lower_handle_joint: -0.22}):
        lower_handle_rotated = ctx.part_element_world_aabb(lower_handle, elem="grip")

    ctx.check(
        "upper door swings outward independently",
        upper_closed is not None
        and upper_open is not None
        and lower_closed is not None
        and lower_still is not None
        and upper_open[0][1] < upper_closed[0][1] - 0.35
        and isclose(lower_still[0][1], lower_closed[0][1], abs_tol=1e-5),
        details=f"upper_closed={upper_closed}, upper_open={upper_open}, lower_closed={lower_closed}, lower_still={lower_still}",
    )
    ctx.check(
        "lower door swings on its own hinge axis",
        lower_closed is not None
        and lower_open is not None
        and lower_open[0][1] < lower_closed[0][1] - 0.35,
        details=f"lower_closed={lower_closed}, lower_open={lower_open}",
    )
    ctx.check(
        "upper pull handle rotates slightly on its mounts",
        upper_handle_closed is not None
        and upper_handle_rotated is not None
        and upper_handle_rotated[1][0] > upper_handle_closed[1][0] + 0.008,
        details=f"closed={upper_handle_closed}, rotated={upper_handle_rotated}",
    )
    ctx.check(
        "lower pull handle rotates slightly on its mounts",
        lower_handle_closed is not None
        and lower_handle_rotated is not None
        and lower_handle_rotated[0][0] < lower_handle_closed[0][0] - 0.008,
        details=f"closed={lower_handle_closed}, rotated={lower_handle_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
