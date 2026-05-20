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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_mcc_panel")

    painted_steel = model.material("painted_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_seal = model.material("dark_gasket", rgba=(0.025, 0.027, 0.028, 1.0))
    hinge_dark = model.material("dark_hinge", rgba=(0.09, 0.095, 0.10, 1.0))
    handle_black = model.material("black_latch", rgba=(0.01, 0.012, 0.014, 1.0))
    label_yellow = model.material("warning_label", rgba=(1.0, 0.80, 0.10, 1.0))
    label_red = model.material("red_warning", rgba=(0.75, 0.04, 0.02, 1.0))

    cabinet_w = 1.00
    cabinet_d = 0.45
    cabinet_h = 2.10
    steel_t = 0.035
    front_y = -cabinet_d / 2.0
    door_t = 0.026
    # The closed doors just kiss the front frame, as compressed gasketed steel
    # doors do, so the assembly is visibly supported without interpenetration.
    door_y = front_y - steel_t / 2.0 - door_t / 2.0
    hinge_y = door_y - 0.020
    panel_y = door_y - hinge_y
    door_h = 1.90
    door_z = 1.08
    hinge_offset = 0.025
    center_gap = 0.012
    door_w = cabinet_w / 2.0 - hinge_offset - center_gap / 2.0
    hinge_r = 0.014

    body = model.part("body")
    # A sheet-steel enclosure: individual panels leave the front as a real door
    # opening rather than a solid block.
    body.visual(
        Box((cabinet_w, steel_t, cabinet_h)),
        origin=Origin(xyz=(0.0, cabinet_d / 2.0 - steel_t / 2.0, cabinet_h / 2.0)),
        material=painted_steel,
        name="rear_panel",
    )
    body.visual(
        Box((steel_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(-(cabinet_w / 2.0 - steel_t / 2.0), 0.0, cabinet_h / 2.0)),
        material=painted_steel,
        name="side_panel_0",
    )
    body.visual(
        Box((steel_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=((cabinet_w / 2.0 - steel_t / 2.0), 0.0, cabinet_h / 2.0)),
        material=painted_steel,
        name="side_panel_1",
    )
    body.visual(
        Box((cabinet_w, cabinet_d, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - steel_t / 2.0)),
        material=painted_steel,
        name="top_panel",
    )
    body.visual(
        Box((cabinet_w, cabinet_d, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, steel_t / 2.0)),
        material=painted_steel,
        name="bottom_panel",
    )
    body.visual(
        Box((cabinet_w + 0.06, cabinet_d + 0.04, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=hinge_dark,
        name="floor_plinth",
    )
    body.visual(
        Box((0.055, steel_t, cabinet_h - 0.13)),
        origin=Origin(xyz=(-(cabinet_w / 2.0 - 0.0275), front_y, cabinet_h / 2.0)),
        material=painted_steel,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.055, steel_t, cabinet_h - 0.13)),
        origin=Origin(xyz=((cabinet_w / 2.0 - 0.0275), front_y, cabinet_h / 2.0)),
        material=painted_steel,
        name="front_jamb_1",
    )
    body.visual(
        Box((cabinet_w, steel_t, 0.065)),
        origin=Origin(xyz=(0.0, front_y, cabinet_h - 0.0325)),
        material=painted_steel,
        name="front_rail_top",
    )
    body.visual(
        Box((cabinet_w, steel_t, 0.095)),
        origin=Origin(xyz=(0.0, front_y, 0.0875)),
        material=painted_steel,
        name="front_rail_bottom",
    )
    body.visual(
        Box((0.045, 0.016, cabinet_h - 0.20)),
        origin=Origin(xyz=(-cabinet_w / 2.0 - 0.006, front_y - 0.023, cabinet_h / 2.0)),
        material=hinge_dark,
        name="hinge_reinforcement_0",
    )
    body.visual(
        Box((0.045, 0.016, cabinet_h - 0.20)),
        origin=Origin(xyz=(cabinet_w / 2.0 + 0.006, front_y - 0.023, cabinet_h / 2.0)),
        material=hinge_dark,
        name="hinge_reinforcement_1",
    )
    for side_index, side in enumerate((-1, 1)):
        hinge_x = side * cabinet_w / 2.0
        bracket_x = hinge_x + side * 0.011
        for knuckle_index, local_z in enumerate((-0.31, 0.31)):
            body.visual(
                Cylinder(radius=0.012, length=0.20),
                origin=Origin(xyz=(hinge_x, hinge_y, door_z + local_z)),
                material=hinge_dark,
                name=f"body_hinge_knuckle_{side_index}_{knuckle_index}",
            )
            body.visual(
                Box((0.035, 0.024, 0.18)),
                origin=Origin(xyz=(bracket_x, hinge_y + 0.012, door_z + local_z)),
                material=hinge_dark,
                name=f"body_hinge_leaf_{side_index}_{knuckle_index}",
            )

    def add_door(
        part_name: str,
        *,
        side: int,
        material: Material,
    ):
        door = model.part(part_name)
        panel_center_x = side * (hinge_offset + door_w / 2.0)
        front_strip_y = panel_y - (door_t / 2.0 + 0.001)
        door.visual(
            Box((door_w, door_t, door_h)),
            origin=Origin(xyz=(panel_center_x, panel_y, 0.0)),
            material=material,
            name="door_leaf",
        )
        door.visual(
            Box((door_w + 0.006, 0.010, 0.026)),
            origin=Origin(xyz=(panel_center_x, front_strip_y, door_h / 2.0 - 0.05)),
            material=painted_steel,
            name="top_flange",
        )
        door.visual(
            Box((door_w + 0.006, 0.010, 0.026)),
            origin=Origin(xyz=(panel_center_x, front_strip_y, -(door_h / 2.0 - 0.05))),
            material=painted_steel,
            name="bottom_flange",
        )
        door.visual(
            Box((0.026, 0.010, door_h - 0.08)),
            origin=Origin(xyz=(side * hinge_offset, front_strip_y, 0.0)),
            material=painted_steel,
            name="outer_stile",
        )
        door.visual(
            Box((0.038, 0.011, door_h - 0.11)),
            origin=Origin(
                xyz=(side * (hinge_offset + door_w - 0.019), front_strip_y - 0.001, 0.0)
            ),
            material=painted_steel,
            name="center_stile",
        )
        door.visual(
            Box((door_w - 0.08, 0.007, 0.018)),
            origin=Origin(xyz=(panel_center_x, front_strip_y, 0.30)),
            material=painted_steel,
            name="mid_rail",
        )
        # Three exposed hinge knuckles ride on the outer-edge revolute axis.
        for i, z in enumerate((-0.62, 0.0, 0.62)):
            door.visual(
                Cylinder(radius=hinge_r, length=0.34),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=hinge_dark,
                name=f"hinge_knuckle_{i}",
            )
            door.visual(
                Box((0.048, 0.008, 0.25)),
                origin=Origin(xyz=(side * 0.024, 0.004, z)),
                material=hinge_dark,
                name=f"hinge_leaf_{i}",
            )
        # Raised lower louver bars and a gasket outline make the door read as a
        # manufactured MCC cabinet door without adding extra controls.
        for i, z in enumerate((-0.52, -0.47, -0.42, -0.37)):
            door.visual(
                Box((door_w - 0.16, 0.007, 0.014)),
                origin=Origin(xyz=(panel_center_x, front_strip_y, z)),
                material=hinge_dark,
                name=f"louver_{i}",
            )
        door.visual(
            Box((0.16, 0.006, 0.095)),
            origin=Origin(xyz=(panel_center_x, front_strip_y, 0.66)),
            material=label_yellow,
            name="arc_flash_label",
        )
        door.visual(
            Box((0.055, 0.007, 0.055)),
            origin=Origin(xyz=(panel_center_x, front_strip_y - 0.001, 0.66)),
            material=label_red,
            name="warning_symbol",
        )
        return door

    door_0 = add_door("door_0", side=1, material=painted_steel)
    door_1 = add_door("door_1", side=-1, material=painted_steel)

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="escutcheon",
    )
    latch_handle.visual(
        Box((0.026, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=handle_black,
        name="vertical_grip",
    )
    latch_handle.visual(
        Box((0.16, 0.010, 0.028)),
        origin=Origin(xyz=(-0.070, -0.014, 0.0)),
        material=hinge_dark,
        name="latch_cam",
    )

    # A fixed keeper on the opposite center stile completes the visual latch.
    door_0.visual(
        Box((0.035, 0.007, 0.22)),
        origin=Origin(
            xyz=(hinge_offset + door_w - 0.030, panel_y - (door_t / 2.0 + 0.004), 0.03)
        ),
        material=handle_black,
        name="latch_keeper",
    )

    model.articulation(
        "body_to_door_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_0,
        origin=Origin(xyz=(-cabinet_w / 2.0, hinge_y, door_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_door_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_1,
        origin=Origin(xyz=(cabinet_w / 2.0, hinge_y, door_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_1_to_latch",
        ArticulationType.REVOLUTE,
        parent=door_1,
        child=latch_handle,
        origin=Origin(
            xyz=(
                -(hinge_offset + door_w - 0.055),
                panel_y - (door_t / 2.0 + 0.0135),
                0.08,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    latch = object_model.get_part("latch_handle")
    hinge_0 = object_model.get_articulation("body_to_door_0")
    hinge_1 = object_model.get_articulation("body_to_door_1")
    latch_turn = object_model.get_articulation("door_1_to_latch")

    ctx.check(
        "door hinges sit on outer cabinet edges",
        hinge_0.origin.xyz[0] < -0.49 and hinge_1.origin.xyz[0] > 0.49,
        details=f"hinge_0={hinge_0.origin.xyz}, hinge_1={hinge_1.origin.xyz}",
    )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, latch_turn: 0.0}):
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            min_gap=0.004,
            max_gap=0.025,
            name="closed doors meet at a narrow center stile gap",
        )
        ctx.expect_gap(
            body,
            door_0,
            axis="y",
            positive_elem="front_jamb_0",
            negative_elem="door_leaf",
            max_gap=0.002,
            max_penetration=0.0005,
            name="door_0 seats on the front frame",
        )
        ctx.expect_gap(
            body,
            door_1,
            axis="y",
            positive_elem="front_jamb_1",
            negative_elem="door_leaf",
            max_gap=0.002,
            max_penetration=0.0005,
            name="door_1 seats on the front frame",
        )
        ctx.expect_gap(
            door_1,
            latch,
            axis="y",
            positive_elem="center_stile",
            negative_elem="escutcheon",
            max_gap=0.002,
            max_penetration=0.0005,
            name="latch escutcheon is seated on the center stile",
        )

    def _extent(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return float(hi[axis_index] - lo[axis_index])

    def _min_y(aabb) -> float | None:
        if aabb is None:
            return None
        return float(aabb[0][1])

    closed_0 = ctx.part_element_world_aabb(door_0, elem="door_leaf")
    closed_1 = ctx.part_element_world_aabb(door_1, elem="door_leaf")
    with ctx.pose({hinge_0: hinge_0.motion_limits.upper}):
        opened_0 = ctx.part_element_world_aabb(door_0, elem="door_leaf")
    with ctx.pose({hinge_1: hinge_1.motion_limits.upper}):
        opened_1 = ctx.part_element_world_aabb(door_1, elem="door_leaf")
    ctx.check(
        "door_0 swings outward from its outer hinge",
        _min_y(opened_0) is not None
        and _min_y(closed_0) is not None
        and _min_y(opened_0) < _min_y(closed_0) - 0.18,
        details=f"closed={closed_0}, opened={opened_0}",
    )
    ctx.check(
        "door_1 swings outward from its outer hinge",
        _min_y(opened_1) is not None
        and _min_y(closed_1) is not None
        and _min_y(opened_1) < _min_y(closed_1) - 0.18,
        details=f"closed={closed_1}, opened={opened_1}",
    )

    closed_grip = ctx.part_element_world_aabb(latch, elem="vertical_grip")
    with ctx.pose({latch_turn: latch_turn.motion_limits.upper}):
        turned_grip = ctx.part_element_world_aabb(latch, elem="vertical_grip")
    ctx.check(
        "latch handle quarter-turns on the center stile",
        _extent(closed_grip, 2) is not None
        and _extent(turned_grip, 0) is not None
        and _extent(closed_grip, 2) > 0.15
        and _extent(turned_grip, 0) > 0.15,
        details=f"closed={closed_grip}, turned={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
