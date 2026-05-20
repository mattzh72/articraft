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
    model = ArticulatedObject(name="open_cage_goods_elevator")

    painted_steel = Material("safety_yellow_painted_steel", color=(0.95, 0.66, 0.08, 1.0))
    dark_steel = Material("dark_blued_steel", color=(0.12, 0.13, 0.13, 1.0))
    galvanized = Material("galvanized_wire_mesh", color=(0.62, 0.66, 0.66, 1.0))
    worn_plate = Material("worn_checkered_floor_plate", color=(0.32, 0.34, 0.34, 1.0))
    rubber = Material("black_rubber_rollers", color=(0.02, 0.02, 0.018, 1.0))

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    tower = model.part("tower")

    # Fixed tower: one broad foot plate, four posts, cross ties, and two inside guide rails.
    add_box(tower, "base_plate", (1.36, 1.14, 0.08), (0.0, 0.0, 0.04), dark_steel)
    for sx in (-1, 1):
        for sy in (-1, 1):
            add_box(
                tower,
                f"tower_post_{sx}_{sy}",
                (0.065, 0.065, 2.80),
                (sx * 0.57, sy * 0.48, 1.44),
                painted_steel,
            )

    for sy in (-1, 1):
        add_box(tower, f"top_crossbeam_{sy}", (1.20, 0.07, 0.07), (0.0, sy * 0.48, 2.82), painted_steel)
        add_box(tower, f"lower_crossbeam_{sy}", (1.20, 0.055, 0.055), (0.0, sy * 0.48, 0.33), painted_steel)
        add_box(tower, f"mid_crossbeam_{sy}", (1.20, 0.05, 0.05), (0.0, sy * 0.48, 1.45), painted_steel)
    for sx in (-1, 1):
        add_box(tower, f"top_sidebeam_{sx}", (0.07, 1.02, 0.07), (sx * 0.57, 0.0, 2.82), painted_steel)
        add_box(tower, f"lower_sidebeam_{sx}", (0.055, 1.02, 0.055), (sx * 0.57, 0.0, 0.33), painted_steel)

    tower.visual(
        Box((0.035, 0.055, 2.50)),
        origin=Origin(xyz=(-0.50, 0.0, 1.36)),
        material=galvanized,
        name="guide_rail_-1",
    )
    tower.visual(
        Box((0.035, 0.055, 2.50)),
        origin=Origin(xyz=(0.50, 0.0, 1.36)),
        material=galvanized,
        name="guide_rail_1",
    )
    for sx in (-1, 1):
        for z in (0.62, 1.36, 2.10):
            add_box(
                tower,
                f"rail_tie_{sx}_{int(z * 100)}",
                (0.090, 1.02, 0.040),
                (sx * 0.535, 0.0, z),
                dark_steel,
            )
            add_box(
                tower,
                f"rail_bracket_{sx}_{int(z * 100)}",
                (0.145, 0.045, 0.045),
                (sx * 0.535, 0.0, z),
                dark_steel,
            )

    cage = model.part("cage")

    # Moving cage car: a real open cage, not a solid box.  The front is left
    # open for the hinged gate; side and rear faces are welded wire grids.
    add_box(cage, "floor_plate", (0.82, 0.72, 0.06), (0.0, 0.0, 0.03), worn_plate)
    for sx in (-1, 1):
        for sy in (-1, 1):
            add_box(cage, f"corner_post_{sx}_{sy}", (0.045, 0.045, 0.94), (sx * 0.41, sy * 0.36, 0.52), painted_steel)
    for sy in (-1, 1):
        add_box(cage, f"top_rail_frontback_{sy}", (0.86, 0.04, 0.045), (0.0, sy * 0.36, 0.965), painted_steel)
        add_box(cage, f"bottom_rail_frontback_{sy}", (0.86, 0.04, 0.045), (0.0, sy * 0.36, 0.115), painted_steel)
    for sx in (-1, 1):
        add_box(cage, f"top_side_rail_{sx}", (0.045, 0.76, 0.045), (sx * 0.41, 0.0, 0.965), painted_steel)
        add_box(cage, f"bottom_side_rail_{sx}", (0.045, 0.76, 0.045), (sx * 0.41, 0.0, 0.115), painted_steel)

    # Side wire mesh grids.
    for sx in (-1, 1):
        x = sx * 0.432
        for i, y in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
            add_box(cage, f"side_vertical_wire_{sx}_{i}", (0.010, 0.010, 0.70), (x, y, 0.50), galvanized)
        for i, z in enumerate((0.23, 0.35, 0.47, 0.59, 0.71, 0.83)):
            add_box(cage, f"side_horizontal_wire_{sx}_{i}", (0.010, 0.70, 0.010), (x, 0.0, z), galvanized)

    # Rear wire mesh grid.
    y_back = 0.382
    for i, x in enumerate((-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)):
        add_box(cage, f"rear_vertical_wire_{i}", (0.010, 0.010, 0.70), (x, y_back, 0.50), galvanized)
    for i, z in enumerate((0.23, 0.35, 0.47, 0.59, 0.71, 0.83)):
        add_box(cage, f"rear_horizontal_wire_{i}", (0.84, 0.010, 0.010), (0.0, y_back, z), galvanized)

    # Sliding guide shoes and small rollers next to the tower rails.
    for sx in (-1, 1):
        for iz, z in enumerate((0.30, 0.76)):
            add_box(cage, f"guide_shoe_{sx}_{iz}", (0.060, 0.18, 0.060), (sx * 0.449, 0.0, z), dark_steel)
            add_box(cage, f"shoe_web_{sx}_{iz}", (0.045, 0.19, 0.020), (sx * 0.423, 0.0, z), dark_steel)
    cage.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(-0.4655, 0.0, 0.30), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="guide_roller_-1_0",
    )
    cage.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(-0.4655, 0.0, 0.76), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="guide_roller_-1_1",
    )
    cage.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.4655, 0.0, 0.30), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="guide_roller_1_0",
    )
    cage.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.4655, 0.0, 0.76), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="guide_roller_1_1",
    )

    # Fixed hinge knuckles and a mounting strap on the cage front edge.
    hinge_x = -0.405
    hinge_y = -0.425
    add_box(cage, "hinge_mount_strip", (0.050, 0.045, 0.84), (hinge_x - 0.020, hinge_y + 0.055, 0.50), dark_steel)
    for i, (zc, length) in enumerate(((0.18, 0.16), (0.48, 0.16), (0.78, 0.16))):
        add_cylinder(cage, f"fixed_hinge_knuckle_{i}", 0.022, length, (hinge_x, hinge_y, zc), dark_steel)
        add_box(cage, f"fixed_hinge_lug_{i}", (0.050, 0.070, length), (hinge_x - 0.045, hinge_y + 0.030, zc), dark_steel)

    gate = model.part("gate")

    # Gate frame is authored in a hinge-line frame: local +X spans across the
    # entrance; local +Z rises up the vertical hinge pin.
    add_box(gate, "hinge_stile", (0.030, 0.025, 0.78), (0.055, 0.0, 0.39), painted_steel)
    add_box(gate, "latch_stile", (0.030, 0.025, 0.78), (0.755, 0.0, 0.39), painted_steel)
    add_box(gate, "top_gate_rail", (0.73, 0.025, 0.035), (0.405, 0.0, 0.755), painted_steel)
    add_box(gate, "bottom_gate_rail", (0.73, 0.025, 0.035), (0.405, 0.0, 0.035), painted_steel)

    for i, x in enumerate((0.17, 0.29, 0.41, 0.53, 0.65)):
        add_box(gate, f"gate_vertical_wire_{i}", (0.010, 0.010, 0.64), (x, 0.0, 0.39), galvanized)
    for i, z in enumerate((0.16, 0.28, 0.40, 0.52, 0.64)):
        add_box(gate, f"gate_horizontal_wire_{i}", (0.70, 0.010, 0.010), (0.405, 0.0, z), galvanized)

    for i, (zc, length) in enumerate(((0.23, 0.10), (0.53, 0.10))):
        add_cylinder(gate, f"gate_hinge_knuckle_{i}", 0.020, length, (0.0, 0.0, zc), dark_steel)
        add_box(gate, f"gate_hinge_lug_{i}", (0.060, 0.010, length), (0.030, 0.0, zc), dark_steel)
    gate.visual(
        Cylinder(radius=0.008, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=dark_steel,
        name="hinge_pin",
    )

    add_box(gate, "latch_plate", (0.060, 0.018, 0.090), (0.725, -0.018, 0.45), dark_steel)
    add_box(gate, "pull_handle", (0.028, 0.025, 0.220), (0.675, -0.048, 0.45), dark_steel)
    add_box(gate, "handle_standoff_low", (0.070, 0.055, 0.020), (0.695, -0.025, 0.40), dark_steel)
    add_box(gate, "handle_standoff_high", (0.070, 0.055, 0.020), (0.695, -0.025, 0.52), dark_steel)

    model.articulation(
        "tower_to_cage",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.45, lower=0.0, upper=1.40),
    )

    model.articulation(
        "cage_to_gate",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=gate,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.10)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cage = object_model.get_part("cage")
    gate = object_model.get_part("gate")
    lift = object_model.get_articulation("tower_to_cage")
    gate_hinge = object_model.get_articulation("cage_to_gate")

    # The hinge is represented like the real hardware: one continuous hinge pin
    # carried by the gate passes through the fixed cage knuckles.  The knuckles
    # themselves are not hollow mesh geometry, so the captured-pin fit is the
    # one intentional local overlap in the model.
    for i in range(3):
        fixed_knuckle = f"fixed_hinge_knuckle_{i}"
        ctx.allow_overlap(
            cage,
            gate,
            elem_a=fixed_knuckle,
            elem_b="hinge_pin",
            reason="The gate hinge pin is intentionally captured inside the fixed cage knuckle.",
        )
        ctx.expect_within(
            gate,
            cage,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=fixed_knuckle,
            margin=0.0,
            name=f"hinge pin is centered in fixed knuckle {i}",
        )
        ctx.expect_overlap(
            gate,
            cage,
            axes="z",
            elem_a="hinge_pin",
            elem_b=fixed_knuckle,
            min_overlap=0.12,
            name=f"hinge pin passes through fixed knuckle {i}",
        )

    ctx.expect_contact(
        cage,
        tower,
        elem_a="guide_roller_1_0",
        elem_b="guide_rail_1",
        contact_tol=0.001,
        name="right lower guide roller rides on the guide rail",
    )
    ctx.expect_contact(
        cage,
        tower,
        elem_a="guide_roller_-1_0",
        elem_b="guide_rail_-1",
        contact_tol=0.001,
        name="left lower guide roller rides on the guide rail",
    )

    closed_cage_position = ctx.part_world_position(cage)
    with ctx.pose({lift: 1.40}):
        raised_cage_position = ctx.part_world_position(cage)
        ctx.expect_overlap(
            cage,
            tower,
            axes="z",
            elem_a="guide_roller_1_1",
            elem_b="guide_rail_1",
            min_overlap=0.02,
            name="raised cage remains engaged with the right guide rail",
        )
        ctx.expect_overlap(
            cage,
            tower,
            axes="z",
            elem_a="guide_roller_-1_1",
            elem_b="guide_rail_-1",
            min_overlap=0.02,
            name="raised cage remains engaged with the left guide rail",
        )
    ctx.check(
        "cage moves vertically on the prismatic guide",
        closed_cage_position is not None
        and raised_cage_position is not None
        and raised_cage_position[2] > closed_cage_position[2] + 1.35,
        details=f"closed={closed_cage_position}, raised={raised_cage_position}",
    )

    closed_gate_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
    with ctx.pose({gate_hinge: 1.20}):
        open_gate_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
    ctx.check(
        "gate swings outward from the cage entrance",
        closed_gate_aabb is not None
        and open_gate_aabb is not None
        and open_gate_aabb[0][1] < closed_gate_aabb[0][1] - 0.45,
        details=f"closed_latch_aabb={closed_gate_aabb}, open_latch_aabb={open_gate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
