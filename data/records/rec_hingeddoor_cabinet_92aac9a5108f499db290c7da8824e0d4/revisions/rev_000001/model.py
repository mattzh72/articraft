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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_door_medicine_cabinet")

    # Overall proportions: a small shallow wall-mounted medicine cabinet.
    cabinet_w = 0.420
    cabinet_h = 0.620
    cabinet_d = 0.120
    panel_t = 0.014

    door_w = 0.436
    door_h = 0.636
    door_slab_t = 0.012
    door_border = 0.030

    hinge_x = -cabinet_w / 2.0 - 0.012
    hinge_y = -cabinet_d / 2.0 - door_slab_t
    hinge_len = 0.086
    hinge_centers_z = (-0.215, 0.215)

    carcass_mat = model.material("warm_white_painted_steel", rgba=(0.86, 0.88, 0.84, 1.0))
    inner_mat = model.material("matte_off_white_interior", rgba=(0.93, 0.94, 0.91, 1.0))
    wall_mat = model.material("painted_wall_backer", rgba=(0.78, 0.80, 0.76, 1.0))
    mirror_mat = model.material("cool_blue_mirror_glass", rgba=(0.62, 0.78, 0.88, 0.55))
    dark_back_mat = model.material("dark_door_shadow_gap", rgba=(0.08, 0.09, 0.09, 1.0))
    metal_mat = model.material("brushed_nickel_hardware", rgba=(0.72, 0.72, 0.68, 1.0))
    pull_mat = model.material("satin_chrome_finger_pull", rgba=(0.80, 0.78, 0.72, 1.0))

    carcass = model.part("carcass")

    # A thin wall backer plate behind the cabinet makes the mounting intent clear.
    carcass.visual(
        Box((cabinet_w + 0.070, 0.006, cabinet_h + 0.070)),
        origin=Origin(xyz=(0.0, cabinet_d / 2.0 + 0.003, 0.0)),
        material=wall_mat,
        name="wall_mount_plate",
    )
    carcass.visual(
        Box((cabinet_w, panel_t, cabinet_h)),
        origin=Origin(xyz=(0.0, cabinet_d / 2.0 - panel_t / 2.0, 0.0)),
        material=carcass_mat,
        name="back_panel",
    )
    carcass.visual(
        Box((panel_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + panel_t / 2.0, 0.0, 0.0)),
        material=carcass_mat,
        name="hinge_side_panel",
    )
    carcass.visual(
        Box((panel_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(cabinet_w / 2.0 - panel_t / 2.0, 0.0, 0.0)),
        material=carcass_mat,
        name="free_side_panel",
    )
    carcass.visual(
        Box((cabinet_w, cabinet_d, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h / 2.0 - panel_t / 2.0)),
        material=carcass_mat,
        name="top_panel",
    )
    carcass.visual(
        Box((cabinet_w, cabinet_d, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, -cabinet_h / 2.0 + panel_t / 2.0)),
        material=carcass_mat,
        name="bottom_panel",
    )
    # Narrow shelves are fixed into the side panels, emphasizing the hollow cabinet.
    for idx, z in enumerate((-0.105, 0.105)):
        carcass.visual(
            Box((cabinet_w - 2.0 * panel_t, cabinet_d - 0.028, 0.006)),
            origin=Origin(xyz=(0.0, 0.004, z)),
            material=inner_mat,
            name=f"fixed_shelf_{idx}",
        )

    # Two visible hinge assemblies: each has a carcass leaf, interleaved knuckles,
    # and a fixed pin that captures the door knuckle on the hinge axis.
    for label, z, leaf_name, knuckle_0_name, knuckle_1_name, pin_name in (
        ("lower", hinge_centers_z[0], "lower_hinge_leaf", "lower_hinge_knuckle_0", "lower_hinge_knuckle_1", "lower_hinge_pin"),
        ("upper", hinge_centers_z[1], "upper_hinge_leaf", "upper_hinge_knuckle_0", "upper_hinge_knuckle_1", "upper_hinge_pin"),
    ):
        carcass.visual(
            Box((0.026, 0.012, hinge_len)),
            origin=Origin(xyz=(hinge_x + 0.012, hinge_y + 0.012, z)),
            material=metal_mat,
            name=leaf_name,
        )
        carcass.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(xyz=(hinge_x, hinge_y, z - 0.028)),
            material=metal_mat,
            name=knuckle_0_name,
        )
        carcass.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(xyz=(hinge_x, hinge_y, z + 0.028)),
            material=metal_mat,
            name=knuckle_1_name,
        )
        carcass.visual(
            Cylinder(radius=0.0022, length=hinge_len),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=metal_mat,
            name=pin_name,
        )

    door = model.part("door")
    door.visual(
        Box((door_w, door_slab_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0 + 0.002, 0.0, 0.0)),
        material=dark_back_mat,
        name="door_backing",
    )
    # Raised front frame around the mirror.
    door.visual(
        Box((door_w, 0.006, door_border)),
        origin=Origin(xyz=(door_w / 2.0 + 0.002, -door_slab_t / 2.0 - 0.003, door_h / 2.0 - door_border / 2.0)),
        material=carcass_mat,
        name="top_door_rail",
    )
    door.visual(
        Box((door_w, 0.006, door_border)),
        origin=Origin(xyz=(door_w / 2.0 + 0.002, -door_slab_t / 2.0 - 0.003, -door_h / 2.0 + door_border / 2.0)),
        material=carcass_mat,
        name="bottom_door_rail",
    )
    door.visual(
        Box((door_border, 0.006, door_h)),
        origin=Origin(xyz=(door_border / 2.0 + 0.002, -door_slab_t / 2.0 - 0.003, 0.0)),
        material=carcass_mat,
        name="hinge_door_stile",
    )
    door.visual(
        Box((door_border, 0.006, door_h)),
        origin=Origin(xyz=(door_w - door_border / 2.0 + 0.002, -door_slab_t / 2.0 - 0.003, 0.0)),
        material=carcass_mat,
        name="free_door_stile",
    )
    door.visual(
        Box((door_w - 2.0 * door_border, 0.004, door_h - 2.0 * door_border)),
        origin=Origin(xyz=(door_w / 2.0 + 0.002, -door_slab_t / 2.0 - 0.004, 0.0)),
        material=mirror_mat,
        name="mirror_glass",
    )
    # Door-side hinge leaves and center knuckles share the child frame hinge axis.
    for label, z, leaf_name, knuckle_name in (
        ("lower", hinge_centers_z[0], "lower_door_hinge_leaf", "lower_hinge_knuckle"),
        ("upper", hinge_centers_z[1], "upper_door_hinge_leaf", "upper_hinge_knuckle"),
    ):
        door.visual(
            Box((0.036, 0.006, hinge_len - 0.006)),
            origin=Origin(xyz=(0.016, -0.007, z)),
            material=metal_mat,
            name=leaf_name,
        )
        door.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=metal_mat,
            name=knuckle_name,
        )

    # Fixed pivot boss for the separate rotating finger pull, placed on the free-edge stile.
    pull_x = door_w - 0.017
    door.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(pull_x, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pull_mat,
        name="pull_pivot_mount",
    )

    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    pull = model.part("finger_pull")
    pull.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pull_mat,
        name="pull_spindle",
    )
    pull.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pull_mat,
        name="pull_hub",
    )
    pull.visual(
        Box((0.014, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, -0.011, -0.042)),
        material=pull_mat,
        name="pull_tab",
    )
    pull.visual(
        Sphere(radius=0.0085),
        origin=Origin(xyz=(0.0, -0.011, -0.077)),
        material=pull_mat,
        name="rounded_pull_tip",
    )

    model.articulation(
        "door_to_finger_pull",
        ArticulationType.REVOLUTE,
        parent=door,
        child=pull,
        origin=Origin(xyz=(pull_x, -0.015, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=1.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    pull = object_model.get_part("finger_pull")
    door_joint = object_model.get_articulation("carcass_to_door")
    pull_joint = object_model.get_articulation("door_to_finger_pull")

    for label in ("lower", "upper"):
        ctx.allow_overlap(
            carcass,
            door,
            elem_a=f"{label}_hinge_pin",
            elem_b=f"{label}_hinge_knuckle",
            reason="The fixed hinge pin intentionally passes through the door-side barrel so the mirrored door is captured.",
        )
        ctx.expect_within(
            carcass,
            door,
            axes="xy",
            inner_elem=f"{label}_hinge_pin",
            outer_elem=f"{label}_hinge_knuckle",
            margin=0.0005,
            name=f"{label} hinge pin is centered in door barrel",
        )
        ctx.expect_overlap(
            carcass,
            door,
            axes="z",
            elem_a=f"{label}_hinge_pin",
            elem_b=f"{label}_hinge_knuckle",
            min_overlap=0.025,
            name=f"{label} hinge pin passes through the door barrel",
        )

    ctx.allow_overlap(
        door,
        pull,
        elem_a="pull_pivot_mount",
        elem_b="pull_spindle",
        reason="The small finger-pull spindle is intentionally seated inside the fixed pivot boss.",
    )
    ctx.expect_within(
        pull,
        door,
        axes="xz",
        inner_elem="pull_spindle",
        outer_elem="pull_pivot_mount",
        margin=0.0005,
        name="finger pull spindle is captured in the pivot mount",
    )
    ctx.expect_overlap(
        door,
        pull,
        axes="y",
        elem_a="pull_pivot_mount",
        elem_b="pull_spindle",
        min_overlap=0.006,
        name="finger pull spindle remains inserted in pivot mount",
    )

    ctx.check(
        "door hinge axis is vertical",
        tuple(round(v, 3) for v in door_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "finger pull has small pivot travel",
        pull_joint.motion_limits is not None
        and pull_joint.motion_limits.lower is not None
        and pull_joint.motion_limits.upper is not None
        and pull_joint.motion_limits.lower < 0.0
        and pull_joint.motion_limits.upper > 0.0
        and pull_joint.motion_limits.upper <= 0.35,
        details=f"limits={pull_joint.motion_limits}",
    )
    ctx.expect_overlap(
        door,
        carcass,
        axes="xz",
        elem_a="mirror_glass",
        elem_b="back_panel",
        min_overlap=0.34,
        name="mirrored door covers the cabinet opening",
    )
    ctx.expect_gap(
        carcass,
        door,
        axis="y",
        positive_elem="hinge_side_panel",
        negative_elem="door_backing",
        min_gap=0.004,
        max_gap=0.010,
        name="closed door sits just in front of the carcass",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_backing")
    with ctx.pose({door_joint: 1.20}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_backing")
        ctx.expect_overlap(
            carcass,
            door,
            axes="z",
            elem_a="upper_hinge_pin",
            elem_b="upper_hinge_knuckle",
            min_overlap=0.025,
            name="upper hinge stays captured while door swings",
        )
    ctx.check(
        "door swings outward from the cabinet face",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.20,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    pull_closed_aabb = ctx.part_element_world_aabb(pull, elem="pull_tab")
    with ctx.pose({pull_joint: 0.28}):
        pull_turned_aabb = ctx.part_element_world_aabb(pull, elem="pull_tab")
    ctx.check(
        "finger pull rotates on its pivot",
        pull_closed_aabb is not None
        and pull_turned_aabb is not None
        and pull_turned_aabb[1][0] > pull_closed_aabb[1][0] + 0.015,
        details=f"closed={pull_closed_aabb}, turned={pull_turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
