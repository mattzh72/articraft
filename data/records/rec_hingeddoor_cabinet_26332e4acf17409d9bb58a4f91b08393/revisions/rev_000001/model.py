from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_door_shaker_base_cabinet")

    maple = model.material("warm_maple", rgba=(0.72, 0.52, 0.34, 1.0))
    painted = model.material("painted_ivory", rgba=(0.86, 0.84, 0.76, 1.0))
    recessed = model.material("recessed_panel_shadow", rgba=(0.78, 0.76, 0.68, 1.0))
    brass = model.material("aged_brass_hinges", rgba=(0.72, 0.52, 0.18, 1.0))
    dark_gap = model.material("dark_reveal", rgba=(0.08, 0.07, 0.055, 1.0))
    knob_mat = model.material("brushed_dark_knob", rgba=(0.18, 0.16, 0.14, 1.0))

    cab_w = 1.00
    cab_d = 0.55
    cab_h = 0.80
    panel_t = 0.035

    carcass = model.part("carcass")
    carcass.visual(
        Box((panel_t, cab_d, cab_h)),
        origin=Origin(xyz=(-cab_w / 2 + panel_t / 2, 0.0, cab_h / 2)),
        material=maple,
        name="side_panel_0",
    )
    carcass.visual(
        Box((panel_t, cab_d, cab_h)),
        origin=Origin(xyz=(cab_w / 2 - panel_t / 2, 0.0, cab_h / 2)),
        material=maple,
        name="side_panel_1",
    )
    carcass.visual(
        Box((cab_w, cab_d, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t / 2)),
        material=maple,
        name="bottom_panel",
    )
    carcass.visual(
        Box((cab_w, cab_d, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, cab_h - panel_t / 2)),
        material=maple,
        name="top_panel",
    )
    carcass.visual(
        Box((cab_w, 0.025, cab_h)),
        origin=Origin(xyz=(0.0, cab_d / 2 - 0.0125, cab_h / 2)),
        material=maple,
        name="back_panel",
    )
    # A dark interior set back from the two doors makes the simple carcass read
    # as a box rather than a solid block.
    carcass.visual(
        Box((cab_w - 2 * panel_t, 0.006, cab_h - 2 * panel_t)),
        origin=Origin(xyz=(0.0, -cab_d / 2 + 0.006, cab_h / 2)),
        material=dark_gap,
        name="open_shadow",
    )

    hinge_x = 0.493
    hinge_y = -cab_d / 2 - 0.016
    door_h = 0.72
    door_body_w = 0.472
    door_offset = 0.014
    door_t = 0.022
    rail_w = 0.070
    stile_w = 0.062
    hinge_centers = (0.22, 0.58)

    def add_fixed_hinge_set(sign: float, suffix: str) -> None:
        x_axis = sign * hinge_x
        side_plate_x = sign * (hinge_x + 0.009)
        for idx, zc in enumerate(hinge_centers):
            carcass.visual(
                Box((0.006, 0.064, 0.126)),
                origin=Origin(xyz=(side_plate_x, hinge_y + 0.030, zc)),
                material=brass,
                name=f"hinge_side_plate_{suffix}_{idx}",
            )
            for seg, dz in enumerate((-0.0525, 0.0, 0.0525)):
                carcass.visual(
                    Cylinder(radius=0.008, length=0.025),
                    origin=Origin(xyz=(x_axis, hinge_y, zc + dz)),
                    material=brass,
                    name=f"hinge_fixed_knuckle_{suffix}_{idx}_{seg}",
                )

    add_fixed_hinge_set(-1.0, "0")
    add_fixed_hinge_set(1.0, "1")

    def build_door(name: str, sign: float):
        """Build a shaker door in a frame whose origin is its hinge pin line."""

        door = model.part(name)
        x_dir = -sign  # sign=-1 (door_0) extends +X; sign=+1 (door_1) extends -X.
        body_center_x = x_dir * (door_offset + door_body_w / 2)
        inner_stile_x = x_dir * (door_offset + door_body_w - stile_w / 2)
        outer_stile_x = x_dir * (door_offset + stile_w / 2)
        rail_x = x_dir * (door_offset + door_body_w / 2)
        panel_x = x_dir * (door_offset + door_body_w / 2)

        door.visual(
            Box((door_body_w, door_t, door_h)),
            origin=Origin(xyz=(body_center_x, 0.0, cab_h / 2)),
            material=painted,
            name="door_slab",
        )
        # Raised shaker frame on the front face, proud of a recessed center panel.
        for vname, x in (("outer_stile", outer_stile_x), ("inner_stile", inner_stile_x)):
            door.visual(
                Box((stile_w, 0.011, door_h)),
                origin=Origin(xyz=(x, -door_t / 2 - 0.0035, cab_h / 2)),
                material=painted,
                name=vname,
            )
        for vname, z in (("top_rail", cab_h / 2 + door_h / 2 - rail_w / 2), ("bottom_rail", cab_h / 2 - door_h / 2 + rail_w / 2)):
            door.visual(
                Box((door_body_w, 0.011, rail_w)),
                origin=Origin(xyz=(rail_x, -door_t / 2 - 0.0035, z)),
                material=painted,
                name=vname,
            )
        door.visual(
            Box((door_body_w - 2 * stile_w, 0.006, door_h - 2 * rail_w)),
            origin=Origin(xyz=(panel_x, -door_t / 2 - 0.001, cab_h / 2)),
            material=recessed,
            name="recessed_panel",
        )

        # Two moving hinge leaves and interleaved knuckles.  The leaf starts just
        # inside the fixed hinge barrel, so the door remains visibly clipped to
        # the side hinge without the leaves colliding at the closed pose.
        leaf_center_x = x_dir * 0.041
        for idx, zc in enumerate(hinge_centers):
            door.visual(
                Box((0.065, 0.005, 0.118)),
                origin=Origin(xyz=(leaf_center_x, -0.010, zc)),
                material=brass,
                name=f"hinge_leaf_{idx}",
            )
            for seg, dz in enumerate((-0.0265, 0.0265)):
                door.visual(
                    Cylinder(radius=0.009, length=0.022),
                    origin=Origin(xyz=(0.0, 0.0, zc + dz)),
                    material=brass,
                    name=f"hinge_door_knuckle_{idx}_{seg}",
                )
                door.visual(
                    Box((0.014, 0.008, 0.018)),
                    origin=Origin(xyz=(x_dir * 0.012, -0.0075, zc + dz)),
                    material=brass,
                    name=f"hinge_knuckle_bridge_{idx}_{seg}",
                )

        return door

    door_0 = build_door("door_0", -1.0)
    door_1 = build_door("door_1", 1.0)

    model.articulation(
        "carcass_to_door_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=0.0, upper=1.70),
    )
    model.articulation(
        "carcass_to_door_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door_1,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=0.0, upper=1.70),
    )

    def build_knob(name: str):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.010, length=0.019),
            origin=Origin(xyz=(0.0, -0.0095, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=knob_mat,
            name="stem",
        )
        knob.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=(0.0, -0.034, 0.0)),
            material=knob_mat,
            name="round_pull",
        )
        knob.visual(
            Box((0.032, 0.003, 0.006)),
            origin=Origin(xyz=(0.0, -0.056, 0.0)),
            material=brass,
            name="turn_marker",
        )
        return knob

    knob_0 = build_knob("knob_0")
    knob_1 = build_knob("knob_1")

    model.articulation(
        "door_0_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=door_0,
        child=knob_0,
        origin=Origin(xyz=(door_offset + door_body_w - 0.060, -door_t / 2 - 0.009, cab_h / 2 + 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0),
    )
    model.articulation(
        "door_1_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=door_1,
        child=knob_1,
        origin=Origin(xyz=(-(door_offset + door_body_w - 0.060), -door_t / 2 - 0.009, cab_h / 2 + 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    door_joint_0 = object_model.get_articulation("carcass_to_door_0")
    door_joint_1 = object_model.get_articulation("carcass_to_door_1")
    knob_joint_0 = object_model.get_articulation("door_0_to_knob_0")

    ctx.expect_gap(
        carcass,
        door_0,
        axis="y",
        positive_elem="side_panel_0",
        negative_elem="door_slab",
        min_gap=0.002,
        max_gap=0.012,
        name="door_0 closes just in front of carcass",
    )
    ctx.expect_gap(
        carcass,
        door_1,
        axis="y",
        positive_elem="side_panel_1",
        negative_elem="door_slab",
        min_gap=0.002,
        max_gap=0.012,
        name="door_1 closes just in front of carcass",
    )
    ctx.expect_overlap(door_0, carcass, axes="z", elem_a="door_slab", elem_b="side_panel_0", min_overlap=0.65)
    ctx.expect_overlap(door_1, carcass, axes="z", elem_a="door_slab", elem_b="side_panel_1", min_overlap=0.65)
    ctx.expect_gap(
        door_0,
        knob_0,
        axis="y",
        positive_elem="door_slab",
        negative_elem="stem",
        min_gap=0.008,
        max_gap=0.012,
        name="knob_0 stem seats on door face",
    )
    ctx.expect_gap(
        door_1,
        knob_1,
        axis="y",
        positive_elem="door_slab",
        negative_elem="stem",
        min_gap=0.008,
        max_gap=0.012,
        name="knob_1 stem seats on door face",
    )

    hinge_0_rest = ctx.part_world_position(door_0)
    hinge_1_rest = ctx.part_world_position(door_1)
    closed_aabb_0 = ctx.part_element_world_aabb(door_0, elem="door_slab")
    closed_aabb_1 = ctx.part_element_world_aabb(door_1, elem="door_slab")
    with ctx.pose({door_joint_0: 1.25, door_joint_1: 1.25}):
        open_aabb_0 = ctx.part_element_world_aabb(door_0, elem="door_slab")
        open_aabb_1 = ctx.part_element_world_aabb(door_1, elem="door_slab")
        hinge_0_open = ctx.part_world_position(door_0)
        hinge_1_open = ctx.part_world_position(door_1)

    ctx.check(
        "door_0 opens outward from its clipped hinge",
        closed_aabb_0 is not None
        and open_aabb_0 is not None
        and open_aabb_0[0][1] < closed_aabb_0[0][1] - 0.22,
        details=f"closed={closed_aabb_0}, open={open_aabb_0}",
    )
    ctx.check(
        "door_1 opens outward from its clipped hinge",
        closed_aabb_1 is not None
        and open_aabb_1 is not None
        and open_aabb_1[0][1] < closed_aabb_1[0][1] - 0.22,
        details=f"closed={closed_aabb_1}, open={open_aabb_1}",
    )
    ctx.check(
        "door hinge origins stay seated on carcass",
        hinge_0_rest == hinge_0_open and hinge_1_rest == hinge_1_open,
        details=f"rest=({hinge_0_rest}, {hinge_1_rest}) open=({hinge_0_open}, {hinge_1_open})",
    )

    marker_closed = ctx.part_element_world_aabb(knob_0, elem="turn_marker")
    with ctx.pose({knob_joint_0: math.pi / 2}):
        marker_rotated = ctx.part_element_world_aabb(knob_0, elem="turn_marker")
    ctx.check(
        "knob_0 has its own rotary mounting axis",
        marker_closed is not None
        and marker_rotated is not None
        and (marker_rotated[1][2] - marker_rotated[0][2]) > (marker_closed[1][2] - marker_closed[0][2]) + 0.015,
        details=f"closed={marker_closed}, rotated={marker_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
