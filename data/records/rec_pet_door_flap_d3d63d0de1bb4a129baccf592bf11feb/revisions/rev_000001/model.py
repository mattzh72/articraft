from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_pet_flap")

    warm_plastic = Material("warm_white_insulated_plastic", rgba=(0.86, 0.82, 0.72, 1.0))
    dark_rubber = Material("dark_rubber_gasket", rgba=(0.025, 0.026, 0.025, 1.0))
    clear_polycarbonate = Material("clear_smoked_polycarbonate", rgba=(0.58, 0.82, 0.95, 0.38))
    grey_hardware = Material("brushed_grey_hardware", rgba=(0.38, 0.39, 0.39, 1.0))
    amber_latch = Material("amber_latch_tab", rgba=(0.95, 0.58, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.58, 0.74),
                outer_size=(0.82, 1.02),
                depth=0.10,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.035,
                outer_corner_radius=0.055,
            ),
            "outer_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.51), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_plastic,
        name="outer_frame",
    )
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.54, 0.70),
                outer_size=(0.64, 0.80),
                depth=0.014,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.030,
                outer_corner_radius=0.040,
            ),
            "inner_gasket",
        ),
        origin=Origin(xyz=(0.0, 0.035, 0.51), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="inner_gasket",
    )
    frame.visual(
        Box((0.78, 0.16, 0.14)),
        origin=Origin(xyz=(0.0, 0.025, 0.89)),
        material=warm_plastic,
        name="hood",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(xyz=(-0.3305, 0.065, 0.800), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_hardware,
        name="socket_0",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(xyz=(0.3305, 0.065, 0.800), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_hardware,
        name="socket_1",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.46, 0.010, 0.58)),
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        material=clear_polycarbonate,
        name="clear_panel",
    )
    flap.visual(
        Box((0.54, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_rubber,
        name="top_rail",
    )
    for x, name in ((-0.2525, "side_rail_0"), (0.2525, "side_rail_1")):
        flap.visual(
            Box((0.035, 0.022, 0.620)),
            origin=Origin(xyz=(x, 0.0, -0.340)),
            material=dark_rubber,
            name=name,
        )
    flap.visual(
        Box((0.54, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.660)),
        material=dark_rubber,
        name="bottom_rail",
    )
    flap.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(-0.294, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_hardware,
        name="pin_0",
    )
    flap.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(0.294, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_hardware,
        name="pin_1",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.065, 0.800)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.95, upper=1.15),
    )

    latch_tab = model.part("latch_tab")
    latch_tab.visual(
        Cylinder(radius=0.012, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_hardware,
        name="pivot_barrel",
    )
    latch_tab.visual(
        Box((0.105, 0.012, 0.082)),
        origin=Origin(xyz=(0.0, 0.018, -0.047)),
        material=amber_latch,
        name="tab_plate",
    )
    latch_tab.visual(
        Box((0.070, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.010, -0.018)),
        material=amber_latch,
        name="tab_neck",
    )

    model.articulation(
        "flap_to_latch",
        ArticulationType.REVOLUTE,
        parent=flap,
        child=latch_tab,
        origin=Origin(xyz=(0.0, 0.020, -0.685)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.65, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    latch = object_model.get_part("latch_tab")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    latch_hinge = object_model.get_articulation("flap_to_latch")

    ctx.allow_overlap(
        flap,
        latch,
        elem_a="bottom_rail",
        elem_b="pivot_barrel",
        reason="The latch pivot barrel is intentionally seated into the lower rail as a captured hinge knuckle.",
    )

    with ctx.pose({flap_hinge: 0.0, latch_hinge: 0.0}):
        ctx.expect_within(
            flap,
            frame,
            axes="xz",
            margin=0.0,
            name="closed flap stays inside the outer frame silhouette",
        )
        ctx.expect_gap(
            latch,
            flap,
            axis="y",
            max_gap=0.001,
            max_penetration=0.004,
            positive_elem="pivot_barrel",
            negative_elem="bottom_rail",
            name="latch barrel is captured at the flap edge",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="x",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="socket_1",
            negative_elem="pin_1",
            name="upper pin seats against the frame socket",
        )

    rest_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 0.85}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap opens outward on the top horizontal hinge",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > rest_flap_aabb[1][1] + 0.20,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_hinge: 0.90}):
        turned_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "latch tab rotates on its local lower-edge pivot",
        rest_latch_aabb is not None
        and turned_latch_aabb is not None
        and turned_latch_aabb[1][1] > rest_latch_aabb[1][1] + 0.025,
        details=f"rest={rest_latch_aabb}, turned={turned_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
