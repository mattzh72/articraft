from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
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
    model = ArticulatedObject(name="wall_service_hatch")

    wall_paint = Material("painted_wall", rgba=(0.78, 0.77, 0.72, 1.0))
    frame_paint = Material("powder_coated_frame", rgba=(0.52, 0.55, 0.54, 1.0))
    panel_paint = Material("warm_gray_panel", rgba=(0.64, 0.66, 0.64, 1.0))
    rubber = Material("dark_rubber_gasket", rgba=(0.035, 0.035, 0.032, 1.0))
    zinc = Material("brushed_zinc", rgba=(0.82, 0.80, 0.72, 1.0))
    latch_black = Material("black_latch", rgba=(0.025, 0.026, 0.025, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.86, 1.00, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=wall_paint,
        name="wall_patch",
    )
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.49, 0.63),
                outer_size=(0.64, 0.78),
                depth=0.035,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.030,
                face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
            ),
            "shallow_perimeter_frame",
        ),
        origin=Origin(),
        material=frame_paint,
        name="perimeter_frame",
    )
    frame.visual(
        Box((0.47, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.328, 0.021)),
        material=rubber,
        name="bottom_gasket",
    )
    frame.visual(
        Box((0.16, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, -0.329, 0.020)),
        material=zinc,
        name="latch_keeper",
    )

    hinge_y = 0.315
    hinge_z = 0.030
    for i, x in enumerate((-0.195, 0.0, 0.195)):
        frame.visual(
            Box((0.092, 0.024, 0.008)),
            origin=Origin(xyz=(x, hinge_y + 0.014, hinge_z - 0.005)),
            material=zinc,
            name=f"frame_hinge_leaf_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.090),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"frame_knuckle_{i}",
        )

    panel = model.part("panel")
    panel.visual(
        Box((0.46, 0.60, 0.018)),
        origin=Origin(xyz=(0.0, -0.310, -0.006)),
        material=panel_paint,
        name="flat_panel",
    )
    panel.visual(
        Box((0.40, 0.50, 0.003)),
        origin=Origin(xyz=(0.0, -0.325, 0.004)),
        material=panel_paint,
        name="pressed_center",
    )
    panel.visual(
        Box((0.37, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.570, 0.004)),
        material=rubber,
        name="lower_seal_line",
    )
    for i, x in enumerate((-0.100, 0.100)):
        panel.visual(
            Box((0.086, 0.024, 0.008)),
            origin=Origin(xyz=(x, -0.016, -0.003)),
            material=zinc,
            name=f"panel_hinge_leaf_{i}",
        )
        panel.visual(
            Cylinder(radius=0.012, length=0.085),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"panel_knuckle_{i}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=zinc,
        name="latch_shaft",
    )
    latch.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=latch_black,
        name="round_latch_hub",
    )
    latch.visual(
        Box((0.082, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=latch_black,
        name="turn_wing",
    )
    latch.visual(
        Box((0.095, 0.016, 0.006)),
        origin=Origin(xyz=(0.030, 0.0, -0.030)),
        material=zinc,
        name="rear_cam",
    )

    panel_hinge = model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(0.0, -0.535, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    panel_hinge.meta["description"] = "Horizontal top-edge hinge; positive travel opens the hatch panel upward and outward."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    latch = object_model.get_part("latch")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    latch_joint = object_model.get_articulation("panel_to_latch")

    def extent(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.allow_overlap(
        panel,
        latch,
        elem_a="flat_panel",
        elem_b="latch_shaft",
        reason="The latch shaft intentionally passes through a small drilled hole in the hatch panel.",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="xy",
        elem_a="flat_panel",
        elem_b="perimeter_frame",
        min_overlap=0.40,
        name="closed panel sits inside the fixed perimeter frame footprint",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="flat_panel",
        negative_elem="perimeter_frame",
        min_gap=-0.006,
        max_gap=0.018,
        name="closed panel is shallow and nearly flush with frame",
    )
    ctx.expect_overlap(
        latch,
        panel,
        axes="xy",
        elem_a="latch_shaft",
        elem_b="flat_panel",
        min_overlap=0.004,
        name="latch shaft is centered through the panel",
    )

    closed_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: 1.0}):
        open_panel_aabb = ctx.part_world_aabb(panel)
        ctx.expect_gap(
            latch,
            frame,
            axis="z",
            positive_elem="turn_wing",
            negative_elem="perimeter_frame",
            min_gap=0.10,
            name="opened hatch lifts the lower latch outward from the wall",
        )
    ctx.check(
        "panel hinge opens upward/outward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_panel_aabb}, open_aabb={open_panel_aabb}",
    )

    latch_rest_aabb = ctx.part_element_world_aabb(latch, elem="turn_wing")
    with ctx.pose({latch_joint: math.pi / 2.0}):
        latch_turned_aabb = ctx.part_element_world_aabb(latch, elem="turn_wing")
        ctx.expect_origin_distance(
            latch,
            panel,
            axes="xy",
            max_dist=0.60,
            name="rotary latch remains mounted to the panel",
        )
    rest_x = extent(latch_rest_aabb, 0)
    turned_y = extent(latch_turned_aabb, 1)
    ctx.check(
        "latch wing rotates a quarter turn",
        rest_x is not None and turned_y is not None and turned_y > rest_x * 0.75,
        details=f"rest_turn_wing_aabb={latch_rest_aabb}, turned_turn_wing_aabb={latch_turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
