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


WOOD = Material("oiled_hardwood", rgba=(0.55, 0.33, 0.16, 1.0))
DARK_WOOD = Material("dark_end_grain", rgba=(0.30, 0.16, 0.07, 1.0))
DRAWER_WOOD = Material("drawer_face_maple", rgba=(0.64, 0.40, 0.19, 1.0))
SHADOW = Material("drawer_shadow", rgba=(0.13, 0.08, 0.04, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hardwood_tabletop_tool_caddy",
        materials=[WOOD, DARK_WOOD, DRAWER_WOOD, SHADOW],
    )

    width = 0.620
    depth = 0.360
    base_t = 0.025
    wall_t = 0.022
    divider_t = 0.018
    wall_h = 0.205
    shelf_t = 0.018
    shelf_z = 0.116

    caddy = model.part("caddy")

    # One low, rectangular hardwood base with the fixed carcass, two divider
    # walls, an upper tray shelf, and the wooden bottom guides for the drawers.
    caddy.visual(
        Box((width, depth, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=WOOD,
        name="base_board",
    )
    side_x = width / 2.0 - wall_t / 2.0
    for i, x in enumerate((-side_x, side_x)):
        caddy.visual(
            Box((wall_t, depth, wall_h)),
            origin=Origin(xyz=(x, 0.0, base_t + wall_h / 2.0)),
            material=WOOD,
            name=f"side_wall_{i}",
        )
    caddy.visual(
        Box((width, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall_t / 2.0, base_t + wall_h / 2.0)),
        material=WOOD,
        name="rear_wall",
    )

    # The two fixed divider walls create three front drawer bays and continue
    # upward as tabletop tool compartments.
    divider_centers = (-0.099, 0.099)
    for i, x in enumerate(divider_centers):
        caddy.visual(
            Box((divider_t, depth, wall_h)),
            origin=Origin(xyz=(x, 0.0, base_t + wall_h / 2.0)),
            material=WOOD,
            name=f"divider_{i}",
        )

    caddy.visual(
        Box((width, depth, shelf_t)),
        origin=Origin(xyz=(0.0, 0.0, shelf_z)),
        material=WOOD,
        name="upper_shelf",
    )
    caddy.visual(
        Box((width, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.001, shelf_z - 0.001)),
        material=DARK_WOOD,
        name="shelf_front_edge",
    )
    caddy.visual(
        Box((width, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.001, base_t + 0.010)),
        material=DARK_WOOD,
        name="base_front_edge",
    )

    drawer_centers = (-0.198, 0.0, 0.198)
    guide_span = 0.245
    guide_w = 0.014
    guide_h = 0.010
    guide_offset = 0.057
    for drawer_i, x_center in enumerate(drawer_centers):
        for runner_i, dx in enumerate((-guide_offset, guide_offset)):
            caddy.visual(
                Box((guide_w, guide_span, guide_h)),
                origin=Origin(
                    xyz=(x_center + dx, -0.024, base_t + guide_h / 2.0)
                ),
                material=DARK_WOOD,
                name=f"guide_{drawer_i}_{runner_i}",
            )

    drawer_depth = 0.270
    drawer_body_w = 0.152
    drawer_front_w = 0.170
    front_t = 0.018
    side_t = 0.010
    bottom_t = 0.008
    drawer_side_h = 0.060
    runner_h = 0.006
    runner_w = 0.014
    runner_depth = 0.230
    frame_y = -0.035
    frame_z = 0.070

    for drawer_i, x_center in enumerate(drawer_centers):
        drawer = model.part(f"drawer_{drawer_i}")

        # A shallow open wooden drawer: bottom, two side boards, back, taller
        # face, attached wooden runners, and a round wooden pull.
        drawer.visual(
            Box((drawer_body_w, drawer_depth, bottom_t)),
            origin=Origin(xyz=(0.0, 0.0, 0.045 - frame_z)),
            material=WOOD,
            name="bottom",
        )
        side_x_local = drawer_body_w / 2.0 - side_t / 2.0
        for side_i, sx in enumerate((-side_x_local, side_x_local)):
            drawer.visual(
                Box((side_t, drawer_depth, drawer_side_h)),
                origin=Origin(xyz=(sx, 0.0, 0.070 - frame_z)),
                material=WOOD,
                name=f"side_{side_i}",
            )
        drawer.visual(
            Box((drawer_body_w, side_t, drawer_side_h)),
            origin=Origin(
                xyz=(0.0, drawer_depth / 2.0 - side_t / 2.0, 0.070 - frame_z)
            ),
            material=WOOD,
            name="back",
        )
        drawer.visual(
            Box((drawer_front_w, front_t, 0.078)),
            origin=Origin(
                xyz=(
                    0.0,
                    -drawer_depth / 2.0 - front_t / 2.0,
                    0.064 - frame_z,
                )
            ),
            material=DRAWER_WOOD,
            name="front",
        )
        for line_i, z in enumerate((0.055, 0.090)):
            drawer.visual(
                Box((drawer_front_w * 0.78, 0.003, 0.004)),
                origin=Origin(
                    xyz=(
                        0.0,
                        -drawer_depth / 2.0 - front_t - 0.001,
                        z - frame_z,
                    )
                ),
                material=DARK_WOOD,
                name=f"front_grain_{line_i}",
            )
        drawer.visual(
            Box((runner_w, runner_depth, runner_h)),
            origin=Origin(xyz=(-guide_offset, -0.005, 0.038 - frame_z)),
            material=DARK_WOOD,
            name="runner_0",
        )
        drawer.visual(
            Box((runner_w, runner_depth, runner_h)),
            origin=Origin(xyz=(guide_offset, -0.005, 0.038 - frame_z)),
            material=DARK_WOOD,
            name="runner_1",
        )

        drawer.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(
                xyz=(0.0, -drawer_depth / 2.0 - front_t - 0.009, 0.070 - frame_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=DARK_WOOD,
            name="pull",
        )

        model.articulation(
            f"caddy_to_drawer_{drawer_i}",
            ArticulationType.PRISMATIC,
            parent=caddy,
            child=drawer,
            origin=Origin(xyz=(x_center, frame_y, frame_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=0.25,
                lower=0.0,
                upper=0.155,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    caddy = object_model.get_part("caddy")

    for i in range(3):
        drawer = object_model.get_part(f"drawer_{i}")
        joint = object_model.get_articulation(f"caddy_to_drawer_{i}")

        ctx.check(
            f"drawer {i} uses a prismatic slide",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=str(joint.articulation_type),
        )
        ctx.expect_contact(
            drawer,
            caddy,
            elem_a="runner_0",
            elem_b=f"guide_{i}_0",
            name=f"drawer {i} left runner sits on wooden guide",
        )
        ctx.expect_contact(
            drawer,
            caddy,
            elem_a="runner_1",
            elem_b=f"guide_{i}_1",
            name=f"drawer {i} right runner sits on wooden guide",
        )
        ctx.expect_overlap(
            drawer,
            caddy,
            axes="y",
            elem_a="runner_0",
            elem_b=f"guide_{i}_0",
            min_overlap=0.18,
            name=f"drawer {i} is deeply seated on guide at rest",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.155}):
            ctx.expect_contact(
                drawer,
                caddy,
                elem_a="runner_0",
                elem_b=f"guide_{i}_0",
                name=f"drawer {i} left runner remains supported when pulled",
            )
            ctx.expect_overlap(
                drawer,
                caddy,
                axes="y",
                elem_a="runner_0",
                elem_b=f"guide_{i}_0",
                min_overlap=0.065,
                name=f"drawer {i} retains insertion at full pull",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer {i} slides outward from the front face",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.14,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
