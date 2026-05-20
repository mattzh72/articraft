from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_louvered_shutter")

    painted_frame = model.material("painted_frame", rgba=(0.86, 0.88, 0.84, 1.0))
    shutter_paint = model.material("shutter_paint", rgba=(0.16, 0.34, 0.25, 1.0))
    recessed_panel = model.material("recessed_panel", rgba=(0.12, 0.27, 0.20, 1.0))
    dark_shadow = model.material("dark_shadow", rgba=(0.035, 0.045, 0.040, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.06, 0.055, 0.050, 1.0))
    pin_metal = model.material("aged_brass_pivots", rgba=(0.68, 0.54, 0.29, 1.0))

    panel_width = 0.58
    panel_height = 1.20
    panel_depth = 0.042
    panel_left = 0.020
    panel_mid_x = panel_left + panel_width / 2.0
    stile_w = 0.065
    top_rail_h = 0.075
    bottom_rail_h = 0.085
    divider_z = -0.090
    divider_h = 0.075
    inner_left = panel_left + stile_w
    inner_right = panel_left + panel_width - stile_w

    # Fixed wall frame / jamb.  It sits just behind the movable shutter and gives
    # the side hinges a believable support path.
    frame = model.part("frame")
    frame.visual(
        Box((0.075, 0.045, 1.36)),
        origin=Origin(xyz=(-0.040, -0.052, 0.0)),
        material=painted_frame,
        name="hinge_jamb",
    )
    frame.visual(
        Box((0.700, 0.045, 0.065)),
        origin=Origin(xyz=(0.285, -0.052, 0.655)),
        material=painted_frame,
        name="top_jamb",
    )
    frame.visual(
        Box((0.700, 0.045, 0.065)),
        origin=Origin(xyz=(0.285, -0.052, -0.655)),
        material=painted_frame,
        name="bottom_jamb",
    )
    frame.visual(
        Box((0.075, 0.045, 1.36)),
        origin=Origin(xyz=(0.640, -0.052, 0.0)),
        material=painted_frame,
        name="outer_jamb",
    )
    for i, z in enumerate((0.38, -0.38)):
        frame.visual(
            Box((0.050, 0.009, 0.165)),
            origin=Origin(xyz=(0.000, -0.0255, z)),
            material=hinge_metal,
            name=f"fixed_hinge_leaf_{i}",
        )

    # Movable shutter panel: an upper louver bay over a solid lower panel,
    # separated by a divider rail.
    panel = model.part("panel")
    panel.visual(
        Box((stile_w, panel_depth, panel_height)),
        origin=Origin(xyz=(panel_left + stile_w / 2.0, 0.0, 0.0)),
        material=shutter_paint,
        name="hinge_stile",
    )
    panel.visual(
        Box((stile_w, panel_depth, panel_height)),
        origin=Origin(xyz=(panel_left + panel_width - stile_w / 2.0, 0.0, 0.0)),
        material=shutter_paint,
        name="outer_stile",
    )
    panel.visual(
        Box((panel_width, panel_depth, top_rail_h)),
        origin=Origin(xyz=(panel_mid_x, 0.0, panel_height / 2.0 - top_rail_h / 2.0)),
        material=shutter_paint,
        name="top_rail",
    )
    panel.visual(
        Box((panel_width, panel_depth, bottom_rail_h)),
        origin=Origin(xyz=(panel_mid_x, 0.0, -panel_height / 2.0 + bottom_rail_h / 2.0)),
        material=shutter_paint,
        name="bottom_rail",
    )
    panel.visual(
        Box((panel_width, panel_depth, divider_h)),
        origin=Origin(xyz=(panel_mid_x, 0.0, divider_z)),
        material=shutter_paint,
        name="divider_rail",
    )

    lower_panel_center_z = (
        (-panel_height / 2.0 + bottom_rail_h) + (divider_z - divider_h / 2.0)
    ) / 2.0
    lower_panel_h = (divider_z - divider_h / 2.0) - (
        -panel_height / 2.0 + bottom_rail_h
    )
    panel.visual(
        Box((inner_right - inner_left, 0.018, lower_panel_h)),
        origin=Origin(xyz=(panel_mid_x, -0.002, lower_panel_center_z)),
        material=recessed_panel,
        name="lower_solid_panel",
    )
    panel.visual(
        Box((0.350, 0.012, lower_panel_h - 0.115)),
        origin=Origin(xyz=(panel_mid_x, 0.011, lower_panel_center_z)),
        material=shutter_paint,
        name="raised_lower_field",
    )
    panel.visual(
        Box((inner_right - inner_left, 0.004, 0.010)),
        origin=Origin(xyz=(panel_mid_x, 0.018, lower_panel_center_z + lower_panel_h / 2.0 - 0.030)),
        material=dark_shadow,
        name="upper_reveal_line",
    )
    panel.visual(
        Box((inner_right - inner_left, 0.004, 0.010)),
        origin=Origin(xyz=(panel_mid_x, 0.018, lower_panel_center_z - lower_panel_h / 2.0 + 0.030)),
        material=dark_shadow,
        name="lower_reveal_line",
    )

    for i, z in enumerate((0.38, -0.38)):
        panel.visual(
            Box((0.050, 0.006, 0.150)),
            origin=Origin(xyz=(0.025, 0.0, z)),
            material=hinge_metal,
            name=f"panel_hinge_leaf_{i}",
        )
        panel.visual(
            Cylinder(radius=0.012, length=0.165),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"hinge_barrel_{i}",
        )
    panel.visual(
        Cylinder(radius=0.004, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_metal,
        name="hinge_pin",
    )

    louver_zs = [0.455, 0.365, 0.275, 0.185, 0.095, 0.005]
    pivot_left_x = inner_left + 0.010
    pivot_right_x = inner_right - 0.010
    for i, z in enumerate(louver_zs):
        panel.visual(
            Cylinder(radius=0.009, length=0.028),
            origin=Origin(xyz=(pivot_left_x, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_metal,
            name=f"pivot_left_{i}",
        )
        panel.visual(
            Cylinder(radius=0.009, length=0.028),
            origin=Origin(xyz=(pivot_right_x, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_metal,
            name=f"pivot_right_{i}",
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    louver_len = 0.380
    louver_chord = 0.064
    louver_thickness = 0.014
    louver_angle = -0.55
    pin_len = 0.034
    pin_center_x = louver_len / 2.0 + pin_len / 2.0
    slat_shape = (
        cq.Workplane("XY")
        .box(louver_len, louver_chord, louver_thickness)
        .edges("|X")
        .fillet(0.004)
    )
    slat_mesh = mesh_from_cadquery(slat_shape, "rounded_louver_slat")

    for i, z in enumerate(louver_zs):
        louver = model.part(f"louver_{i}")
        louver.visual(
            slat_mesh,
            origin=Origin(rpy=(louver_angle, 0.0, 0.0)),
            material=shutter_paint,
            name="slat_body",
        )
        louver.visual(
            Cylinder(radius=0.005, length=pin_len),
            origin=Origin(xyz=(-pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_metal,
            name="pin_left",
        )
        louver.visual(
            Cylinder(radius=0.005, length=pin_len),
            origin=Origin(xyz=(pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_metal,
            name="pin_right",
        )
        model.articulation(
            f"panel_to_louver_{i}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(panel_mid_x, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.70, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("panel")
    frame = object_model.get_part("frame")
    panel_hinge = object_model.get_articulation("frame_to_panel")

    ctx.expect_gap(
        panel,
        frame,
        axis="y",
        positive_elem="hinge_stile",
        negative_elem="hinge_jamb",
        min_gap=0.005,
        name="closed shutter board stands proud of the fixed frame",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        min_overlap=1.0,
        name="panel height is carried by the side frame",
    )

    for i in range(6):
        louver = object_model.get_part(f"louver_{i}")
        for side in ("left", "right"):
            ctx.allow_overlap(
                panel,
                louver,
                elem_a=f"pivot_{side}_{i}",
                elem_b=f"pin_{side}",
                reason="Each louver pivot pin is intentionally seated inside its brass side bushing.",
            )
            ctx.expect_within(
                louver,
                panel,
                axes="yz",
                inner_elem=f"pin_{side}",
                outer_elem=f"pivot_{side}_{i}",
                margin=0.001,
                name=f"louver {i} {side} pin is centered in its bushing",
            )
            ctx.expect_overlap(
                louver,
                panel,
                axes="x",
                elem_a=f"pin_{side}",
                elem_b=f"pivot_{side}_{i}",
                min_overlap=0.010,
                name=f"louver {i} {side} pin remains inserted",
            )

    ctx.expect_gap(
        panel,
        object_model.get_part("louver_0"),
        axis="z",
        positive_elem="top_rail",
        negative_elem="slat_body",
        min_gap=0.030,
        name="top louver clears the upper rail",
    )
    ctx.expect_gap(
        object_model.get_part("louver_5"),
        panel,
        axis="z",
        positive_elem="slat_body",
        negative_elem="divider_rail",
        min_gap=0.020,
        name="lowest upper louver sits above the divider rail",
    )

    rest_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: 1.20}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "panel swings outward on a vertical side hinge",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > rest_panel_aabb[1][1] + 0.30,
        details=f"rest={rest_panel_aabb}, open={open_panel_aabb}",
    )

    louver_joint = object_model.get_articulation("panel_to_louver_2")
    louver = object_model.get_part("louver_2")
    rest_louver_aabb = ctx.part_element_world_aabb(louver, elem="slat_body")
    with ctx.pose({louver_joint: 0.60}):
        rotated_louver_aabb = ctx.part_element_world_aabb(louver, elem="slat_body")
    rest_z = None
    rotated_z = None
    if rest_louver_aabb is not None and rotated_louver_aabb is not None:
        rest_z = rest_louver_aabb[1][2] - rest_louver_aabb[0][2]
        rotated_z = rotated_louver_aabb[1][2] - rotated_louver_aabb[0][2]
    ctx.check(
        "upper louver rotates on its long horizontal pivot",
        rest_z is not None and rotated_z is not None and abs(rotated_z - rest_z) > 0.015,
        details=f"rest_z_extent={rest_z}, rotated_z_extent={rotated_z}",
    )

    return ctx.report()


object_model = build_object_model()
