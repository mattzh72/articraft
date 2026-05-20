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
    model = ArticulatedObject(name="childrens_three_drawer_dresser")

    paint = model.material("soft_white_paint", color=(0.92, 0.88, 0.78, 1.0))
    drawer_paint = model.material("pastel_blue_paint", color=(0.47, 0.68, 0.88, 1.0))
    drawer_wood = model.material("drawer_side_wood", color=(0.72, 0.50, 0.28, 1.0))
    guide_wood = model.material("polished_wood_guides", color=(0.58, 0.34, 0.16, 1.0))
    knob_red = model.material("red_knobs", color=(0.86, 0.22, 0.20, 1.0))

    width = 0.82
    depth = 0.46
    height = 0.76
    panel_t = 0.026
    separator_t = 0.018
    rail_h = 0.018

    slot_h = (height - 2.0 * panel_t - 2.0 * separator_t) / 3.0
    slot_bottoms = [
        panel_t,
        panel_t + slot_h + separator_t,
        panel_t + 2.0 * slot_h + 2.0 * separator_t,
    ]
    slot_centers = [bottom + slot_h / 2.0 for bottom in slot_bottoms]

    carcass = model.part("carcass")

    # A rectangular painted carcass: side walls, top/bottom, full back, and two
    # full-depth separator shelves that create three wide drawer openings.
    carcass.visual(
        Box((depth, panel_t, height)),
        origin=Origin(xyz=(-depth / 2.0, -width / 2.0 + panel_t / 2.0, height / 2.0)),
        material=paint,
        name="side_panel_0",
    )
    carcass.visual(
        Box((depth, panel_t, height)),
        origin=Origin(xyz=(-depth / 2.0, width / 2.0 - panel_t / 2.0, height / 2.0)),
        material=paint,
        name="side_panel_1",
    )
    carcass.visual(
        Box((depth, width, panel_t)),
        origin=Origin(xyz=(-depth / 2.0, 0.0, panel_t / 2.0)),
        material=paint,
        name="bottom_panel",
    )
    carcass.visual(
        Box((depth, width, panel_t)),
        origin=Origin(xyz=(-depth / 2.0, 0.0, height - panel_t / 2.0)),
        material=paint,
        name="top_panel",
    )
    carcass.visual(
        Box((panel_t, width, height)),
        origin=Origin(xyz=(-depth + panel_t / 2.0, 0.0, height / 2.0)),
        material=paint,
        name="back_panel",
    )
    for i, z in enumerate((slot_bottoms[1] - separator_t / 2.0, slot_bottoms[2] - separator_t / 2.0)):
        carcass.visual(
            Box((depth, width, separator_t)),
            origin=Origin(xyz=(-depth / 2.0, 0.0, z)),
            material=paint,
            name=f"separator_shelf_{i}",
        )

    # Front stiles/rails make the dresser read as a framed cabinet even when the
    # drawer faces are slightly proud of the opening.
    face_rail_x = -panel_t / 2.0
    carcass.visual(
        Box((panel_t, panel_t, height)),
        origin=Origin(xyz=(face_rail_x, -width / 2.0 + panel_t / 2.0, height / 2.0)),
        material=paint,
        name="front_stile_0",
    )
    carcass.visual(
        Box((panel_t, panel_t, height)),
        origin=Origin(xyz=(face_rail_x, width / 2.0 - panel_t / 2.0, height / 2.0)),
        material=paint,
        name="front_stile_1",
    )
    carcass.visual(
        Box((panel_t, width, panel_t)),
        origin=Origin(xyz=(face_rail_x, 0.0, panel_t / 2.0)),
        material=paint,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((panel_t, width, panel_t)),
        origin=Origin(xyz=(face_rail_x, 0.0, height - panel_t / 2.0)),
        material=paint,
        name="front_top_rail",
    )
    for i, z in enumerate((slot_bottoms[1] - separator_t / 2.0, slot_bottoms[2] - separator_t / 2.0)):
        carcass.visual(
            Box((panel_t, width, separator_t)),
            origin=Origin(xyz=(face_rail_x, 0.0, z)),
            material=paint,
            name=f"front_separator_{i}",
        )

    # Short block feet keep the children's dresser low and sturdy.
    for ix, x in enumerate((-0.065, -depth + 0.065)):
        for iy, y in enumerate((-width / 2.0 + 0.075, width / 2.0 - 0.075)):
            carcass.visual(
                Box((0.07, 0.065, 0.035)),
                origin=Origin(xyz=(x, y, -0.0175)),
                material=guide_wood,
                name=f"foot_{ix}_{iy}",
            )

    # Fixed wooden bottom-edge guide rails, two under each drawer.
    guide_length = 0.36
    guide_w = 0.035
    guide_y = 0.305
    for row, bottom in enumerate(slot_bottoms):
        for side, y in enumerate((-guide_y, guide_y)):
            carcass.visual(
                Box((guide_length, guide_w, rail_h)),
                origin=Origin(xyz=(-0.22, y, bottom + rail_h / 2.0)),
                material=guide_wood,
                name=f"guide_{row}_{side}",
            )

    # Three separate moving drawers.  Each is an open wooden tray with a broad
    # painted front, two round knobs, and wooden runners on its bottom edges.
    drawer_w = 0.68
    drawer_depth = 0.40
    side_t = 0.012
    bottom_t = 0.012
    runner_h = 0.012
    runner_w = 0.028
    runner_y = guide_y
    runner_z = -slot_h / 2.0 + rail_h + runner_h / 2.0
    bottom_z = runner_z + runner_h / 2.0 + bottom_t / 2.0
    box_bottom = bottom_z + bottom_t / 2.0
    side_h = 0.135
    side_z = box_bottom + side_h / 2.0
    face_t = 0.030
    face_h = slot_h - 0.018
    face_w = width - 2.0 * panel_t - 0.016

    for row, center_z in enumerate(slot_centers):
        drawer = model.part(f"drawer_{row}")
        drawer.visual(
            Box((face_t, face_w, face_h)),
            origin=Origin(xyz=(0.007, 0.0, 0.0)),
            material=drawer_paint,
            name="front",
        )
        drawer.visual(
            Box((drawer_depth, drawer_w, bottom_t)),
            origin=Origin(xyz=(-0.205, 0.0, bottom_z)),
            material=drawer_wood,
            name="bottom",
        )
        drawer.visual(
            Box((drawer_depth, side_t, side_h)),
            origin=Origin(xyz=(-0.205, -drawer_w / 2.0 + side_t / 2.0, side_z)),
            material=drawer_wood,
            name="side_0",
        )
        drawer.visual(
            Box((drawer_depth, side_t, side_h)),
            origin=Origin(xyz=(-0.205, drawer_w / 2.0 - side_t / 2.0, side_z)),
            material=drawer_wood,
            name="side_1",
        )
        drawer.visual(
            Box((side_t, drawer_w, side_h)),
            origin=Origin(xyz=(-0.405 + side_t / 2.0, 0.0, side_z)),
            material=drawer_wood,
            name="back",
        )
        for side, y in enumerate((-runner_y, runner_y)):
            drawer.visual(
                Box((drawer_depth, runner_w, runner_h)),
                origin=Origin(xyz=(-0.205, y, runner_z)),
                material=guide_wood,
                name=f"runner_{side}",
            )
        for knob_i, y in enumerate((-0.18, 0.18)):
            drawer.visual(
                Cylinder(radius=0.024, length=0.032),
                origin=Origin(xyz=(0.038, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=knob_red,
                name=f"knob_{knob_i}",
            )

        model.articulation(
            f"carcass_to_drawer_{row}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.28),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    for row in range(3):
        drawer = object_model.get_part(f"drawer_{row}")
        joint = object_model.get_articulation(f"carcass_to_drawer_{row}")
        ctx.expect_contact(
            drawer,
            carcass,
            elem_a="runner_0",
            elem_b=f"guide_{row}_0",
            contact_tol=0.0005,
            name=f"drawer {row} left runner rests on guide",
        )
        ctx.expect_contact(
            drawer,
            carcass,
            elem_a="runner_1",
            elem_b=f"guide_{row}_1",
            contact_tol=0.0005,
            name=f"drawer {row} right runner rests on guide",
        )
        ctx.expect_within(
            drawer,
            carcass,
            axes="y",
            inner_elem="front",
            outer_elem="back_panel",
            margin=0.0,
            name=f"drawer {row} front is wide but inside side panels",
        )

        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.28}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                elem_a="runner_0",
                elem_b=f"guide_{row}_0",
                min_overlap=0.08,
                name=f"drawer {row} stays retained on left guide",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                elem_a="runner_1",
                elem_b=f"guide_{row}_1",
                min_overlap=0.08,
                name=f"drawer {row} stays retained on right guide",
            )
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="runner_0",
                elem_b=f"guide_{row}_0",
                contact_tol=0.0005,
                name=f"drawer {row} left guide still supports when extended",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer {row} slides outward",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[0] > closed_pos[0] + 0.25,
            details=f"closed={closed_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
