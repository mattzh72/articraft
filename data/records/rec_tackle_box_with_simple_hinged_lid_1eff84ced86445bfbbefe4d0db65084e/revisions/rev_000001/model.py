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
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    body_mat = model.material("dark_teal_plastic", rgba=(0.02, 0.18, 0.22, 1.0))
    rim_mat = model.material("slightly_worn_lip", rgba=(0.04, 0.25, 0.30, 1.0))
    tray_mat = model.material("cream_tray_plastic", rgba=(0.88, 0.78, 0.55, 1.0))
    lid_mat = model.material("blue_gray_lid", rgba=(0.10, 0.28, 0.36, 0.92))
    hinge_mat = model.material("black_hinge_pin_plastic", rgba=(0.01, 0.012, 0.012, 1.0))

    # Object frame: +X is toward the front latch side, +Z is upward.  The rear
    # hinge sits just behind the -X edge of the box body.
    length = 0.48
    width = 0.24
    wall_t = 0.018
    bottom_t = 0.025
    wall_h = 0.140
    wall_center_z = bottom_t + wall_h / 2.0

    body = model.part("body")
    body.visual(
        Box((length, width, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_mat,
        name="bottom_panel",
    )
    body.visual(
        Box((length, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall_t / 2.0, wall_center_z)),
        material=body_mat,
        name="side_wall_0",
    )
    body.visual(
        Box((length, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall_t / 2.0, wall_center_z)),
        material=body_mat,
        name="side_wall_1",
    )
    body.visual(
        Box((wall_t, width, wall_h)),
        origin=Origin(xyz=(length / 2.0 - wall_t / 2.0, 0.0, wall_center_z)),
        material=body_mat,
        name="front_wall",
    )
    body.visual(
        Box((wall_t, width, wall_h)),
        origin=Origin(xyz=(-length / 2.0 + wall_t / 2.0, 0.0, wall_center_z)),
        material=body_mat,
        name="rear_wall",
    )

    # A fixed stop flange / secondary lip runs around the opening.  It is a
    # separate visible tier from the walls, protruding inward for the lid to
    # settle on when closed.
    lip_z = 0.161
    lip_h = 0.018
    body.visual(
        Box((length + 0.010, 0.032, lip_h)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall_t - 0.016 + 0.003, lip_z)),
        material=rim_mat,
        name="side_lip_0",
    )
    body.visual(
        Box((length + 0.010, 0.032, lip_h)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall_t + 0.016 - 0.003, lip_z)),
        material=rim_mat,
        name="side_lip_1",
    )
    body.visual(
        Box((0.032, width - 2.0 * wall_t + 0.010, lip_h)),
        origin=Origin(xyz=(length / 2.0 - wall_t - 0.016 + 0.003, 0.0, lip_z)),
        material=rim_mat,
        name="front_lip",
    )
    body.visual(
        Box((0.032, width - 2.0 * wall_t + 0.010, lip_h)),
        origin=Origin(xyz=(-length / 2.0 + wall_t + 0.016 - 0.003, 0.0, lip_z)),
        material=rim_mat,
        name="rear_lip",
    )

    # Fixed internal tray details: a shallow molded tray and dividers are part
    # of the body assembly, not articulated.  The tray floor slightly enters the
    # side walls so it reads as supported by the box rather than floating.
    body.visual(
        Box((0.450, 0.210, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.062)),
        material=tray_mat,
        name="tray_floor",
    )
    body.visual(
        Box((0.420, 0.006, 0.033)),
        origin=Origin(xyz=(0.012, 0.0, 0.0835)),
        material=tray_mat,
        name="long_divider",
    )
    for idx, x in enumerate((-0.090, 0.075)):
        body.visual(
            Box((0.006, 0.180, 0.033)),
            origin=Origin(xyz=(x, 0.0, 0.0835)),
            material=tray_mat,
            name=f"cross_divider_{idx}",
        )
    body.visual(
        Box((0.420, 0.008, 0.024)),
        origin=Origin(xyz=(0.012, 0.092, 0.079)),
        material=tray_mat,
        name="tray_side_0",
    )
    body.visual(
        Box((0.420, 0.008, 0.024)),
        origin=Origin(xyz=(0.012, -0.092, 0.079)),
        material=tray_mat,
        name="tray_side_1",
    )

    # Exposed rear hinge knuckles are fixed to the body at both sides.  They are
    # separated along Y from the lid knuckle, so the parts meet as an actual
    # hinge instead of interpenetrating.
    hinge_x = -0.253
    hinge_z = 0.176
    for idx, y in enumerate((-0.080, 0.080)):
        body.visual(
            Box((0.018, 0.070, 0.026)),
            origin=Origin(xyz=(hinge_x + 0.009, y, hinge_z - 0.014)),
            material=hinge_mat,
            name=f"hinge_mount_{idx}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.070),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"body_hinge_knuckle_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.485, 0.254, 0.022)),
        # The panel starts forward of the hinge barrel and overhangs the front
        # wall slightly, matching a simple molded tackle-box lid.
        origin=Origin(xyz=(0.2715, 0.0, 0.007)),
        material=lid_mat,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, 0.090, 0.010)),
        origin=Origin(xyz=(0.019, 0.0, 0.001)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="lid_hinge_knuckle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_lip",
            min_gap=0.0,
            max_gap=0.004,
            name="closed lid settles just above the stop flange",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="front_lip",
            min_overlap=0.030,
            name="closed lid covers the front stop lip",
        )
        ctx.expect_within(
            body,
            body,
            axes="xy",
            inner_elem="tray_floor",
            outer_elem="bottom_panel",
            margin=0.002,
            name="fixed tray stays inside the box footprint",
        )

    rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "positive lid rotation opens upward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.20,
        details=f"closed={rest_aabb}, opened={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
