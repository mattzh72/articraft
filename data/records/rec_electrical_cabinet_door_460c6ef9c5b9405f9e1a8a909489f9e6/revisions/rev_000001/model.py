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
    model = ArticulatedObject(name="double_door_switchgear_panel")

    cabinet_grey = model.material("powder_coated_grey", rgba=(0.62, 0.66, 0.66, 1.0))
    darker_grey = model.material("recessed_grey", rgba=(0.38, 0.42, 0.43, 1.0))
    gasket_black = model.material("black_rubber_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    hinge_metal = model.material("brushed_steel", rgba=(0.68, 0.69, 0.66, 1.0))
    handle_black = model.material("black_handle", rgba=(0.03, 0.032, 0.034, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    warning_red = model.material("warning_red", rgba=(0.75, 0.06, 0.04, 1.0))

    width = 1.64
    height = 2.10
    depth = 0.34
    door_width = 0.795
    door_height = 2.02
    door_thickness = 0.040
    hinge_x = 0.845
    door_y = -0.235

    carcass = model.part("carcass")
    carcass.visual(
        Box((width, 0.035, height)),
        origin=Origin(xyz=(0.0, 0.1525, height / 2.0)),
        material=cabinet_grey,
        name="back_panel",
    )
    carcass.visual(
        Box((0.040, depth, height)),
        origin=Origin(xyz=(-0.800, 0.0, height / 2.0)),
        material=cabinet_grey,
        name="side_wall_0",
    )
    carcass.visual(
        Box((0.040, depth, height)),
        origin=Origin(xyz=(0.800, 0.0, height / 2.0)),
        material=cabinet_grey,
        name="side_wall_1",
    )
    carcass.visual(
        Box((width, depth, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cabinet_grey,
        name="bottom_wall",
    )
    carcass.visual(
        Box((width, depth, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, height - 0.0225)),
        material=cabinet_grey,
        name="top_wall",
    )
    carcass.visual(
        Box((0.060, 0.040, height)),
        origin=Origin(xyz=(-0.790, -0.185, height / 2.0)),
        material=darker_grey,
        name="front_frame_0",
    )
    carcass.visual(
        Box((0.060, 0.040, height)),
        origin=Origin(xyz=(0.790, -0.185, height / 2.0)),
        material=darker_grey,
        name="front_frame_1",
    )
    carcass.visual(
        Box((width, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, -0.185, 0.0275)),
        material=darker_grey,
        name="front_sill",
    )
    carcass.visual(
        Box((width, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, -0.185, height - 0.0275)),
        material=darker_grey,
        name="front_header",
    )
    carcass.visual(
        Box((0.035, depth, height)),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
        material=darker_grey,
        name="center_mullion",
    )
    carcass.visual(
        Box((0.024, 0.070, 2.02)),
        origin=Origin(xyz=(0.0, -0.200, height / 2.0)),
        material=gasket_black,
        name="center_gasket",
    )
    # External hinge pillars are tied back into the face frame and side walls.
    for x, suffix in ((-0.845, "0"), (0.845, "1")):
        carcass.visual(
            Box((0.050, 0.065, height)),
            origin=Origin(xyz=(x, -0.200, height / 2.0)),
            material=hinge_metal,
            name=f"hinge_pillar_{suffix}",
        )
        for zc in (0.42, 1.05, 1.68):
            carcass.visual(
                Cylinder(radius=0.025, length=0.210),
                origin=Origin(xyz=(x, -0.238, zc)),
                material=hinge_metal,
                name=f"hinge_knuckle_{suffix}_{int(zc * 100)}",
            )

    left_door = model.part("door_0")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.4275, 0.0, height / 2.0)),
        material=cabinet_grey,
        name="door_panel",
    )
    left_door.visual(
        Box((0.020, door_thickness, door_height)),
        origin=Origin(xyz=(0.035, 0.0, height / 2.0)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    _add_door_details(
        left_door,
        panel_center_x=0.4275,
        inner_x=0.805,
        outer_x=0.050,
        material=darker_grey,
        label_material=warning_yellow,
        accent_material=warning_red,
    )

    right_door = model.part("door_1")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-0.4275, 0.0, height / 2.0)),
        material=cabinet_grey,
        name="door_panel",
    )
    right_door.visual(
        Box((0.020, door_thickness, door_height)),
        origin=Origin(xyz=(-0.035, 0.0, height / 2.0)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    _add_door_details(
        right_door,
        panel_center_x=-0.4275,
        inner_x=-0.805,
        outer_x=-0.050,
        material=darker_grey,
        label_material=warning_yellow,
        accent_material=warning_red,
    )

    lock_bar = model.part("locking_bar")
    lock_bar.visual(
        Box((0.055, 0.022, 1.38)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=handle_black,
        name="locking_bar",
    )
    lock_bar.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="hub_disk",
    )
    lock_bar.visual(
        Cylinder(radius=0.018, length=0.088),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="turn_stem",
    )
    lock_bar.visual(
        Cylinder(radius=0.020, length=0.200),
        origin=Origin(xyz=(0.0, -0.095, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="turn_handle",
    )
    lock_bar.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(-0.103, -0.095, 0.0)),
        material=handle_black,
        name="handle_end_0",
    )
    lock_bar.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.103, -0.095, 0.0)),
        material=handle_black,
        name="handle_end_1",
    )

    model.articulation(
        "carcass_to_door_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(-hinge_x, door_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "carcass_to_door_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(hinge_x, door_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_1_to_locking_bar",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=lock_bar,
        origin=Origin(xyz=(-0.310, -0.028, height / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def _add_door_details(
    door,
    *,
    panel_center_x: float,
    inner_x: float,
    outer_x: float,
    material: Material,
    label_material: Material,
    accent_material: Material,
) -> None:
    """Raised ribs, vent louvers, and warning/nameplate details fused to a leaf."""

    # Perimeter stiffening ribs sit proud of the sheet-metal leaf.
    for x, name in ((outer_x, "outer_stile"), (inner_x, "inner_stile")):
        door.visual(
            Box((0.030, 0.008, 1.82)),
            origin=Origin(xyz=(x, -0.023, 1.05)),
            material=material,
            name=name,
        )
    door.visual(
        Box((0.650, 0.008, 0.030)),
        origin=Origin(xyz=(panel_center_x, -0.023, 1.935)),
        material=material,
        name="top_rail",
    )
    door.visual(
        Box((0.650, 0.008, 0.030)),
        origin=Origin(xyz=(panel_center_x, -0.023, 0.165)),
        material=material,
        name="bottom_rail",
    )

    door.visual(
        Box((0.430, 0.006, 0.190)),
        origin=Origin(xyz=(panel_center_x, -0.022, 0.390)),
        material=Material("dark_vent_shadow", rgba=(0.06, 0.065, 0.065, 1.0)),
        name="vent_recess",
    )
    for i in range(5):
        door.visual(
            Box((0.395, 0.008, 0.017)),
            origin=Origin(xyz=(panel_center_x, -0.028, 0.325 + i * 0.032)),
            material=material,
            name=f"vent_louver_{i}",
        )

    door.visual(
        Box((0.155, 0.006, 0.110)),
        origin=Origin(xyz=(panel_center_x, -0.022, 1.560)),
        material=label_material,
        name="warning_label",
    )
    door.visual(
        Box((0.055, 0.008, 0.055)),
        origin=Origin(xyz=(panel_center_x, -0.025, 1.560)),
        material=accent_material,
        name="warning_flash",
    )
    door.visual(
        Box((0.220, 0.006, 0.055)),
        origin=Origin(xyz=(panel_center_x, -0.022, 1.375)),
        material=material,
        name="nameplate",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("door_0")
    right_door = object_model.get_part("door_1")
    lock_bar = object_model.get_part("locking_bar")
    left_hinge = object_model.get_articulation("carcass_to_door_0")
    right_hinge = object_model.get_articulation("carcass_to_door_1")
    handle_joint = object_model.get_articulation("door_1_to_locking_bar")

    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem="door_panel",
        negative_elem="door_panel",
        min_gap=0.035,
        max_gap=0.050,
        name="leaf doors meet with a narrow center gap",
    )
    ctx.expect_gap(
        carcass,
        left_door,
        axis="x",
        positive_elem="center_gasket",
        negative_elem="door_panel",
        min_gap=0.004,
        max_gap=0.014,
        name="left leaf closes against center gasket",
    )
    ctx.expect_gap(
        right_door,
        carcass,
        axis="x",
        positive_elem="door_panel",
        negative_elem="center_gasket",
        min_gap=0.004,
        max_gap=0.014,
        name="right leaf closes against center gasket",
    )
    ctx.expect_contact(
        lock_bar,
        right_door,
        elem_a="turn_stem",
        elem_b="door_panel",
        contact_tol=0.003,
        name="turn handle shaft seats on the right door",
    )

    closed_left = ctx.part_element_world_aabb(left_door, elem="door_panel")
    closed_right = ctx.part_element_world_aabb(right_door, elem="door_panel")
    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0}):
        open_left = ctx.part_element_world_aabb(left_door, elem="door_panel")
        open_right = ctx.part_element_world_aabb(right_door, elem="door_panel")
    ctx.check(
        "both doors swing outward on outer vertical hinges",
        closed_left is not None
        and closed_right is not None
        and open_left is not None
        and open_right is not None
        and open_left[0][1] < closed_left[0][1] - 0.20
        and open_right[0][1] < closed_right[0][1] - 0.20,
        details=f"closed_left={closed_left}, open_left={open_left}, closed_right={closed_right}, open_right={open_right}",
    )

    locked_aabb = ctx.part_element_world_aabb(lock_bar, elem="locking_bar")
    with ctx.pose({handle_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(lock_bar, elem="locking_bar")
    if locked_aabb is not None and turned_aabb is not None:
        locked_dx = locked_aabb[1][0] - locked_aabb[0][0]
        locked_dz = locked_aabb[1][2] - locked_aabb[0][2]
        turned_dx = turned_aabb[1][0] - turned_aabb[0][0]
        turned_dz = turned_aabb[1][2] - turned_aabb[0][2]
    else:
        locked_dx = locked_dz = turned_dx = turned_dz = 0.0
    ctx.check(
        "turn handle rotates the locking bar ninety degrees",
        locked_dz > locked_dx * 8.0 and turned_dx > turned_dz * 8.0,
        details=f"locked_dx={locked_dx:.3f}, locked_dz={locked_dz:.3f}, turned_dx={turned_dx:.3f}, turned_dz={turned_dz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
