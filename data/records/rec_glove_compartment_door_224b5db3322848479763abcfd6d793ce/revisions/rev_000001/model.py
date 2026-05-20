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
    model = ArticulatedObject(name="offroad_dashboard_glove_box")

    dash_plastic = model.material("textured_charcoal_plastic", rgba=(0.05, 0.055, 0.052, 1.0))
    inner_plastic = model.material("dark_storage_cavity", rgba=(0.018, 0.019, 0.018, 1.0))
    door_plastic = model.material("olive_black_rugged_plastic", rgba=(0.12, 0.135, 0.115, 1.0))
    raised_plastic = model.material("raised_black_ribs", rgba=(0.028, 0.031, 0.028, 1.0))
    rubber = model.material("matte_black_gasket", rgba=(0.006, 0.006, 0.005, 1.0))
    hinge_steel = model.material("blackened_steel", rgba=(0.015, 0.014, 0.013, 1.0))
    latch_plastic = model.material("satin_black_latch", rgba=(0.02, 0.022, 0.021, 1.0))

    dashboard = model.part("dashboard")

    # Coordinate convention: the viewer/driver is at -Y, the fixed storage bin
    # extends into the dashboard along +Y, X is horizontal, and Z is vertical.
    # The large dash face is built from four connected rails so the rectangular
    # glove-box opening remains truly open when the door swings away.
    panel_t = 0.035
    outer_w = 0.78
    outer_h = 0.48
    opening_w = 0.48
    opening_h = 0.30
    side_w = (outer_w - opening_w) / 2.0
    rail_h = (outer_h - opening_h) / 2.0

    dashboard.visual(
        Box((outer_w, panel_t, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + rail_h / 2.0)),
        material=dash_plastic,
        name="upper_fascia",
    )
    dashboard.visual(
        Box((outer_w, panel_t, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, -opening_h / 2.0 - rail_h / 2.0)),
        material=dash_plastic,
        name="lower_fascia",
    )
    dashboard.visual(
        Box((side_w, panel_t, opening_h)),
        origin=Origin(xyz=(-opening_w / 2.0 - side_w / 2.0, 0.0, 0.0)),
        material=dash_plastic,
        name="hinge_fascia",
    )
    dashboard.visual(
        Box((side_w, panel_t, opening_h)),
        origin=Origin(xyz=(opening_w / 2.0 + side_w / 2.0, 0.0, 0.0)),
        material=dash_plastic,
        name="free_fascia",
    )

    # A black rubber lip around the opening and a hollow fixed bin behind it.
    lip_t = 0.010
    lip_y = -0.0205
    dashboard.visual(
        Box((opening_w + 2.0 * lip_t, 0.008, lip_t)),
        origin=Origin(xyz=(0.0, lip_y, opening_h / 2.0 + lip_t / 2.0)),
        material=rubber,
        name="upper_gasket",
    )
    dashboard.visual(
        Box((opening_w + 2.0 * lip_t, 0.008, lip_t)),
        origin=Origin(xyz=(0.0, lip_y, -opening_h / 2.0 - lip_t / 2.0)),
        material=rubber,
        name="lower_gasket",
    )
    dashboard.visual(
        Box((lip_t, 0.008, opening_h)),
        origin=Origin(xyz=(-opening_w / 2.0 - lip_t / 2.0, lip_y, 0.0)),
        material=rubber,
        name="hinge_gasket",
    )
    dashboard.visual(
        Box((lip_t, 0.008, opening_h)),
        origin=Origin(xyz=(opening_w / 2.0 + lip_t / 2.0, lip_y, 0.0)),
        material=rubber,
        name="free_gasket",
    )

    cavity_depth = 0.30
    cavity_w = 0.45
    cavity_h = 0.265
    wall_t = 0.022
    cavity_center_y = panel_t / 2.0 + cavity_depth / 2.0
    dashboard.visual(
        Box((cavity_w, wall_t, cavity_h)),
        origin=Origin(xyz=(0.0, panel_t / 2.0 + cavity_depth + wall_t / 2.0, 0.0)),
        material=inner_plastic,
        name="cavity_back",
    )
    dashboard.visual(
        Box((cavity_w + 2.0 * wall_t, cavity_depth, wall_t)),
        origin=Origin(xyz=(0.0, cavity_center_y, cavity_h / 2.0 + wall_t / 2.0)),
        material=inner_plastic,
        name="cavity_top",
    )
    dashboard.visual(
        Box((cavity_w + 2.0 * wall_t, cavity_depth, wall_t)),
        origin=Origin(xyz=(0.0, cavity_center_y, -cavity_h / 2.0 - wall_t / 2.0)),
        material=inner_plastic,
        name="cavity_bottom",
    )
    dashboard.visual(
        Box((wall_t, cavity_depth, cavity_h + 2.0 * wall_t)),
        origin=Origin(xyz=(-cavity_w / 2.0 - wall_t / 2.0, cavity_center_y, 0.0)),
        material=inner_plastic,
        name="cavity_hinge_wall",
    )
    dashboard.visual(
        Box((wall_t, cavity_depth, cavity_h + 2.0 * wall_t)),
        origin=Origin(xyz=(cavity_w / 2.0 + wall_t / 2.0, cavity_center_y, 0.0)),
        material=inner_plastic,
        name="cavity_free_wall",
    )

    # Fixed halves of the exposed side hinges: one middle knuckle and leaf per
    # upper/lower hinge. The moving door provides the alternating knuckles.
    hinge_x = -0.265
    hinge_y = -0.061
    hinge_centers = (-0.105, 0.105)
    for i, zc in enumerate(hinge_centers):
        dashboard.visual(
            Box((0.067, 0.040, 0.036)),
            origin=Origin(xyz=(hinge_x - 0.042, -0.034, zc)),
            material=hinge_steel,
            name=f"fixed_leaf_{i}",
        )
        dashboard.visual(
            Cylinder(radius=0.012, length=0.034),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=hinge_steel,
            name=f"fixed_knuckle_{i}",
        )
        # Two visible screw heads on the fixed hinge leaf.
        for j, dz in enumerate((-0.012, 0.012)):
            dashboard.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(
                    xyz=(hinge_x - 0.044, -0.056, zc + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=hinge_steel,
                name=f"fixed_screw_{i}_{j}",
            )

    door = model.part("door")
    # Door frame is the hinge axis. The slab begins just to the free side of the
    # hinge barrel, covers the opening, and leaves a realistic air gap to the dash.
    door.visual(
        Box((0.530, 0.032, 0.320)),
        origin=Origin(xyz=(0.280, 0.018, 0.0)),
        material=door_plastic,
        name="door_panel",
    )
    # Raised rugged perimeter and diagonal bracing.
    door.visual(
        Box((0.500, 0.008, 0.026)),
        origin=Origin(xyz=(0.285, -0.002, 0.133)),
        material=raised_plastic,
        name="top_rib",
    )
    door.visual(
        Box((0.500, 0.008, 0.026)),
        origin=Origin(xyz=(0.285, -0.002, -0.133)),
        material=raised_plastic,
        name="bottom_rib",
    )
    door.visual(
        Box((0.026, 0.008, 0.285)),
        origin=Origin(xyz=(0.045, -0.002, 0.0)),
        material=raised_plastic,
        name="hinge_rib",
    )
    door.visual(
        Box((0.026, 0.008, 0.285)),
        origin=Origin(xyz=(0.525, -0.002, 0.0)),
        material=raised_plastic,
        name="free_rib",
    )
    door.visual(
        Box((0.420, 0.008, 0.018)),
        origin=Origin(xyz=(0.285, -0.006, 0.0), rpy=(0.0, 0.54, 0.0)),
        material=raised_plastic,
        name="diagonal_rib_0",
    )
    door.visual(
        Box((0.420, 0.008, 0.018)),
        origin=Origin(xyz=(0.285, -0.006, 0.0), rpy=(0.0, -0.54, 0.0)),
        material=raised_plastic,
        name="diagonal_rib_1",
    )
    door.visual(
        Box((0.140, 0.006, 0.095)),
        origin=Origin(xyz=(0.440, -0.007, 0.005)),
        material=rubber,
        name="latch_recess",
    )

    for i, (x, z) in enumerate(((0.065, 0.125), (0.065, -0.125), (0.505, 0.125), (0.505, -0.125))):
        door.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(x, -0.006, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=f"door_bolt_{i}",
        )

    for i, zc in enumerate(hinge_centers):
        for j, dz in enumerate((-0.027, 0.027)):
            door.visual(
                Box((0.060, 0.010, 0.026)),
                origin=Origin(xyz=(0.040, 0.002, zc + dz)),
                material=hinge_steel,
                name=f"moving_leaf_{i}_{j}",
            )
            door.visual(
                Cylinder(radius=0.012, length=0.024),
                origin=Origin(xyz=(0.0, 0.0, zc + dz)),
                material=hinge_steel,
                name=f"moving_knuckle_{i}_{j}",
            )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=18.0, velocity=1.5),
    )

    paddle_latch = model.part("paddle_latch")
    # Local frame is the latch's short horizontal pivot pin. The paddle hangs
    # below it and rotates outward from the door when pulled.
    paddle_latch.visual(
        Cylinder(radius=0.007, length=0.112),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="pivot_pin",
    )
    paddle_latch.visual(
        Box((0.112, 0.012, 0.058)),
        origin=Origin(xyz=(0.0, -0.008, -0.034)),
        material=latch_plastic,
        name="paddle_plate",
    )
    paddle_latch.visual(
        Box((0.090, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.017, -0.060)),
        material=raised_plastic,
        name="finger_lip",
    )

    # Small clevis pads on the door show where the latch pivot is supported.
    door.visual(
        Box((0.020, 0.010, 0.030)),
        origin=Origin(xyz=(0.378, -0.010, 0.039)),
        material=hinge_steel,
        name="latch_ear_0",
    )
    door.visual(
        Box((0.020, 0.010, 0.030)),
        origin=Origin(xyz=(0.502, -0.010, 0.039)),
        material=hinge_steel,
        name="latch_ear_1",
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=paddle_latch,
        origin=Origin(xyz=(0.440, -0.014, 0.040)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=1.5, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard = object_model.get_part("dashboard")
    door = object_model.get_part("door")
    latch = object_model.get_part("paddle_latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.expect_overlap(
        door,
        dashboard,
        axes="xz",
        elem_a="door_panel",
        elem_b="cavity_back",
        min_overlap=0.20,
        name="closed door covers the glove-box opening",
    )
    ctx.expect_gap(
        dashboard,
        door,
        axis="y",
        positive_elem="free_fascia",
        negative_elem="door_panel",
        min_gap=0.006,
        max_gap=0.018,
        name="door sits proud of dash without collision",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        elem_a="paddle_plate",
        elem_b="latch_recess",
        min_overlap=0.040,
        name="paddle latch is centered in its recess",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    closed_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5 if closed_aabb else None
    open_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5 if open_aabb else None
    ctx.check(
        "door hinge swings outward from the dashboard",
        closed_y is not None and open_y is not None and open_y < closed_y - 0.10,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    latch_closed = ctx.part_element_world_aabb(latch, elem="paddle_plate")
    with ctx.pose({latch_pivot: 0.45}):
        latch_open = ctx.part_element_world_aabb(latch, elem="paddle_plate")
    latch_closed_y = (latch_closed[0][1] + latch_closed[1][1]) * 0.5 if latch_closed else None
    latch_open_y = (latch_open[0][1] + latch_open[1][1]) * 0.5 if latch_open else None
    ctx.check(
        "paddle latch rotates outward on its short pivot",
        latch_closed_y is not None and latch_open_y is not None and latch_open_y < latch_closed_y - 0.010,
        details=f"closed_y={latch_closed_y}, open_y={latch_open_y}",
    )

    return ctx.report()


object_model = build_object_model()
