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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_door_sideboard")

    walnut = model.material("oiled_walnut", rgba=(0.34, 0.18, 0.08, 1.0))
    dark_walnut = model.material("recessed_walnut", rgba=(0.20, 0.10, 0.045, 1.0))
    shadow = model.material("shadowed_interior", rgba=(0.055, 0.045, 0.035, 1.0))
    brass = model.material("brushed_brass", rgba=(0.90, 0.62, 0.26, 1.0))
    black = model.material("black_shadow_gap", rgba=(0.015, 0.012, 0.010, 1.0))

    carcass = model.part("carcass")

    width = 1.56
    depth = 0.42
    body_bottom = 0.09
    top_z = 0.69
    body_height = top_z - body_bottom
    front_y = -depth / 2.0
    back_y = depth / 2.0

    # Long low cabinet shell: continuous panels, rails, and a recessed dark back.
    carcass.visual(
        Box((width, depth, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, top_z - 0.0225)),
        material=walnut,
        name="top_panel",
    )
    carcass.visual(
        Box((width, depth, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + 0.020)),
        material=walnut,
        name="bottom_panel",
    )
    carcass.visual(
        Box((0.040, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + 0.020, 0.0, body_bottom + body_height / 2.0)),
        material=walnut,
        name="side_panel_0",
    )
    carcass.visual(
        Box((0.040, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - 0.020, 0.0, body_bottom + body_height / 2.0)),
        material=walnut,
        name="side_panel_1",
    )
    carcass.visual(
        Box((width - 0.055, 0.026, body_height - 0.020)),
        origin=Origin(xyz=(0.0, back_y - 0.013, body_bottom + body_height / 2.0)),
        material=shadow,
        name="back_panel",
    )
    carcass.visual(
        Box((width, 0.040, 0.058)),
        origin=Origin(xyz=(0.0, front_y + 0.020, top_z - 0.085)),
        material=walnut,
        name="front_top_rail",
    )
    carcass.visual(
        Box((width, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, front_y + 0.020, body_bottom + 0.055)),
        material=walnut,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((width, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, front_y + 0.025, 0.055)),
        material=dark_walnut,
        name="recessed_plinth",
    )

    stile_xs = (-0.725, -0.240, 0.245, 0.730)
    for i, x in enumerate(stile_xs):
        carcass.visual(
            Box((0.036, 0.046, 0.540)),
            origin=Origin(xyz=(x, front_y + 0.023, 0.380)),
            material=walnut,
            name=f"hinge_stile_{i}",
        )

    # Three dark reveal strips between the door leaves and around the apertures.
    for i, x in enumerate((-0.482, 0.0025, 0.4875)):
        carcass.visual(
            Box((0.010, 0.006, 0.505)),
            origin=Origin(xyz=(x, front_y - 0.001, 0.385)),
            material=black,
            name=f"vertical_reveal_{i}",
        )

    hinge_y = front_y - 0.015
    door_center_z = 0.385
    door_width = 0.445
    hinge_offset = 0.018
    door_height = 0.505
    door_panel_y = -0.021
    door_front_y = door_panel_y - 0.012
    trim_y = door_front_y - 0.003
    handle_pivot_y = trim_y - 0.010

    door_hinges = (-0.725, -0.240, 0.245)
    for i, hinge_x in enumerate(door_hinges):
        # Fixed hinge clip mounted to the vertical stile. The two fixed pieces
        # interleave with the moving door knuckles so the leaf reads as clipped
        # to a stout stile without broad hidden overlap.
        for j, z_rel in enumerate((-0.070, 0.070)):
            carcass.visual(
                Box((0.035, 0.009, 0.065)),
                origin=Origin(xyz=(hinge_x - 0.010, front_y - 0.0045, door_center_z + z_rel)),
                material=brass,
                name=f"fixed_hinge_leaf_{i}_{j}",
            )
            carcass.visual(
                Cylinder(radius=0.0085, length=0.065),
                origin=Origin(xyz=(hinge_x, hinge_y, door_center_z + z_rel)),
                material=brass,
                name=f"fixed_hinge_knuckle_{i}_{j}",
            )

        door = model.part(f"door_{i}")
        door.visual(
            Box((door_width, 0.024, door_height)),
            origin=Origin(xyz=(hinge_offset + door_width / 2.0, door_panel_y, 0.0)),
            material=walnut,
            name="panel",
        )
        # Raised frame and a darker inset keep each narrow leaf from reading as
        # a flat placeholder slab.
        door.visual(
            Box((door_width - 0.060, 0.006, door_height - 0.120)),
            origin=Origin(xyz=(hinge_offset + door_width / 2.0, trim_y, 0.0)),
            material=dark_walnut,
            name="inset_field",
        )
        door.visual(
            Box((door_width - 0.028, 0.007, 0.030)),
            origin=Origin(xyz=(hinge_offset + door_width / 2.0, trim_y - 0.0005, door_height / 2.0 - 0.036)),
            material=walnut,
            name="top_rail",
        )
        door.visual(
            Box((door_width - 0.028, 0.007, 0.030)),
            origin=Origin(xyz=(hinge_offset + door_width / 2.0, trim_y - 0.0005, -door_height / 2.0 + 0.036)),
            material=walnut,
            name="bottom_rail",
        )
        door.visual(
            Box((0.030, 0.007, door_height - 0.050)),
            origin=Origin(xyz=(hinge_offset + 0.029, trim_y - 0.0005, 0.0)),
            material=walnut,
            name="hinge_side_rail",
        )
        door.visual(
            Box((0.030, 0.007, door_height - 0.050)),
            origin=Origin(xyz=(hinge_offset + door_width - 0.029, trim_y - 0.0005, 0.0)),
            material=walnut,
            name="pull_side_rail",
        )

        for j, z_rel in enumerate((-0.140, 0.140)):
            door.visual(
                Box((0.048, 0.008, 0.040)),
                origin=Origin(xyz=(0.020, -0.012, z_rel)),
                material=brass,
                name=f"moving_hinge_leaf_{j}",
            )
            door.visual(
                Cylinder(radius=0.0085, length=0.075),
                origin=Origin(xyz=(0.0, 0.0, z_rel)),
                material=brass,
                name=f"moving_hinge_knuckle_{j}",
            )

        handle_x = hinge_offset + door_width - 0.068
        handle_z = 0.010
        # A small round witness plate is part of the door leaf and shows the
        # handle pivot mount even when the brass pull is rotated.
        door.visual(
            Cylinder(radius=0.023, length=0.004),
            origin=Origin(
                xyz=(handle_x, trim_y - 0.002, handle_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name="handle_rosette",
        )

        model.articulation(
            f"door_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, door_center_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
        )

        handle = model.part(f"pull_{i}")
        handle.visual(
            Cylinder(radius=0.006, length=0.052),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="pivot_barrel",
        )
        handle.visual(
            Box((0.020, 0.008, 0.036)),
            origin=Origin(xyz=(0.0, -0.001, -0.010)),
            material=brass,
            name="mounting_stem",
        )
        handle.visual(
            Box((0.082, 0.010, 0.070)),
            origin=Origin(xyz=(0.0, -0.004, -0.046)),
            material=brass,
            name="drop_pull_leaf",
        )
        handle.visual(
            Cylinder(radius=0.007, length=0.086),
            origin=Origin(xyz=(0.0, -0.010, -0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="rounded_grip_edge",
        )

        model.articulation(
            f"pull_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=handle,
            origin=Origin(xyz=(handle_x, handle_pivot_y, handle_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door_joints = [object_model.get_articulation(f"door_hinge_{i}") for i in range(3)]
    pull_joints = [object_model.get_articulation(f"pull_pivot_{i}") for i in range(3)]
    ctx.check(
        "three independent door hinges",
        len(door_joints) == 3 and all(j is not None for j in door_joints),
        details=f"door_joints={door_joints}",
    )
    ctx.check(
        "three independent pull pivots",
        len(pull_joints) == 3 and all(j is not None for j in pull_joints),
        details=f"pull_joints={pull_joints}",
    )

    carcass = object_model.get_part("carcass")
    for i, hinge in enumerate(door_joints):
        door = object_model.get_part(f"door_{i}")
        handle = object_model.get_part(f"pull_{i}")
        pull = pull_joints[i]

        ctx.expect_overlap(
            door,
            carcass,
            axes="z",
            min_overlap=0.45,
            elem_a="panel",
            elem_b=f"hinge_stile_{i}",
            name=f"door {i} remains vertically carried by its stile",
        )
        ctx.expect_contact(
            carcass,
            door,
            elem_a=f"fixed_hinge_knuckle_{i}_0",
            elem_b="moving_hinge_knuckle_0",
            contact_tol=0.001,
            name=f"door {i} hinge knuckles are clipped close to the stile",
        )

        closed_aabb = ctx.part_element_world_aabb(door, elem="pull_side_rail")
        with ctx.pose({hinge: 0.85}):
            open_aabb = ctx.part_element_world_aabb(door, elem="pull_side_rail")
            ctx.expect_contact(
                carcass,
                door,
                elem_a=f"fixed_hinge_knuckle_{i}_0",
                elem_b="moving_hinge_knuckle_0",
                contact_tol=0.001,
                name=f"door {i} stays supported while opening",
            )
        ctx.check(
            f"door {i} opens outward from the front",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

        flat_aabb = ctx.part_element_world_aabb(handle, elem="drop_pull_leaf")
        with ctx.pose({pull: 0.35}):
            swung_aabb = ctx.part_element_world_aabb(handle, elem="drop_pull_leaf")
        ctx.check(
            f"pull {i} swings on its small pivot",
            flat_aabb is not None
            and swung_aabb is not None
            and swung_aabb[0][1] < flat_aabb[0][1] - 0.012,
            details=f"flat={flat_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
