from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A robust rounded-rectangle appliance casing primitive, authored in meters."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_sandwich_press")

    warm_plastic = model.material("warm_grey_plastic", rgba=(0.70, 0.68, 0.62, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.06, 0.065, 0.07, 1.0))
    dark_metal = model.material("dark_nonstick_plate", rgba=(0.015, 0.015, 0.014, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    label_white = model.material("white_marking", rgba=(0.92, 0.90, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_box((0.340, 0.270, 0.060), 0.024), "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=warm_plastic,
        name="base_shell",
    )
    base.visual(
        Box((0.284, 0.198, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, 0.068)),
        material=dark_metal,
        name="lower_plate",
    )
    lower_ridge_names = ("lower_ridge_0", "lower_ridge_1", "lower_ridge_2", "lower_ridge_3", "lower_ridge_4")
    for ridge_name, x in zip(lower_ridge_names, (-0.100, -0.050, 0.000, 0.050, 0.100)):
        base.visual(
            Box((0.012, 0.184, 0.008)),
            origin=Origin(xyz=(x, -0.018, 0.075)),
            material=dark_metal,
            name=ridge_name,
        )

    for i, (x, y) in enumerate(((-0.120, -0.092), (0.120, -0.092), (-0.120, 0.092), (0.120, 0.092))):
        base.visual(
            Box((0.050, 0.030, 0.008)),
            origin=Origin(xyz=(x, y, 0.001)),
            material=dark_plastic,
            name=f"foot_{i}",
        )

    # Back-edge hinge supports mounted to the base shell.  The top platen carries
    # the center knuckle between these two base knuckles.
    for i, x in enumerate((-0.105, 0.105)):
        base.visual(
            Box((0.086, 0.022, 0.028)),
            origin=Origin(xyz=(x, 0.142, 0.079)),
            material=warm_plastic,
            name=f"hinge_saddle_{i}",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.080),
            origin=Origin(xyz=(x, 0.151, 0.096), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"base_knuckle_{i}",
        )

    # Height-stop pivot boss and selector knob socket on the right side.
    base.visual(
        Cylinder(radius=0.015, length=0.011),
        origin=Origin(xyz=(0.1745, 0.040, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="lever_boss",
    )
    base.visual(
        Box((0.006, 0.076, 0.068)),
        origin=Origin(xyz=(0.169, 0.080, 0.094)),
        material=warm_plastic,
        name="height_sector",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.170, -0.076, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_socket",
    )
    base.visual(
        Box((0.006, 0.110, 0.060)),
        origin=Origin(xyz=(0.169, -0.076, 0.072)),
        material=dark_plastic,
        name="selector_label_panel",
    )
    for i, (dy, dz) in enumerate(((-0.040, 0.030), (0.000, 0.040), (0.040, 0.030))):
        base.visual(
            Box((0.004, 0.004, 0.016)),
            origin=Origin(xyz=(0.169, -0.076 + dy, 0.050 + dz)),
            material=label_white,
            name=f"selector_tick_{i}",
        )
    for i, (dy, dz) in enumerate(((0.008, 0.020), (0.025, 0.033), (0.046, 0.038))):
        base.visual(
            Box((0.004, 0.018, 0.006)),
            origin=Origin(xyz=(0.169, 0.040 + dy, 0.086 + dz)),
            material=dark_plastic,
            name=f"height_notch_{i}",
        )

    top = model.part("top_platen")
    top.visual(
        mesh_from_cadquery(_rounded_box((0.332, 0.270, 0.058), 0.023), "top_shell"),
        origin=Origin(xyz=(0.0, -0.150, 0.029)),
        material=warm_plastic,
        name="top_shell",
    )
    top.visual(
        Box((0.278, 0.196, 0.006)),
        origin=Origin(xyz=(0.0, -0.169, -0.0025)),
        material=dark_metal,
        name="top_plate",
    )
    top_ridge_names = ("top_ridge_0", "top_ridge_1", "top_ridge_2", "top_ridge_3", "top_ridge_4")
    for ridge_name, x in zip(top_ridge_names, (-0.100, -0.050, 0.000, 0.050, 0.100)):
        top.visual(
            Box((0.012, 0.180, 0.006)),
            origin=Origin(xyz=(x, -0.169, -0.0080)),
            material=dark_metal,
            name=ridge_name,
        )
    top.visual(
        Box((0.120, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, 0.005)),
        material=hinge_metal,
        name="top_hinge_leaf",
    )
    top.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="top_knuckle",
    )
    for i, x in enumerate((-0.065, 0.065)):
        top.visual(
            Box((0.020, 0.026, 0.026)),
            origin=Origin(xyz=(x, -0.260, 0.066)),
            material=dark_plastic,
            name=f"handle_post_{i}",
        )
    top.visual(
        Cylinder(radius=0.011, length=0.160),
        origin=Origin(xyz=(0.0, -0.270, 0.079), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="front_handle",
    )

    lever = model.part("height_lever")
    lever.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="pivot_cap",
    )
    lever.visual(
        Box((0.008, 0.090, 0.012)),
        origin=Origin(xyz=(0.006, -0.045, 0.0)),
        material=dark_plastic,
        name="stop_arm",
    )
    lever.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.006, -0.089, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="lever_paddle",
    )

    knob = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.024,
            body_style="skirted",
            top_diameter=0.040,
            edge_radius=0.0012,
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "selector_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_cap",
    )

    model.articulation(
        "base_to_top",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.0, 0.151, 0.096)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.20),
    )
    model.articulation(
        "base_to_height_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(0.180, 0.040, 0.086)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "base_to_selector",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.174, -0.076, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top = object_model.get_part("top_platen")
    lever = object_model.get_part("height_lever")
    knob = object_model.get_part("selector_knob")
    top_hinge = object_model.get_articulation("base_to_top")
    lever_pivot = object_model.get_articulation("base_to_height_lever")
    selector = object_model.get_articulation("base_to_selector")

    ctx.check(
        "top hinge has domestic press range",
        top_hinge.motion_limits is not None
        and top_hinge.motion_limits.lower == 0.0
        and top_hinge.motion_limits.upper is not None
        and 1.0 <= top_hinge.motion_limits.upper <= 1.4,
        details=f"limits={top_hinge.motion_limits}",
    )
    ctx.check(
        "selector knob is continuous",
        selector.articulation_type == ArticulationType.CONTINUOUS
        and selector.motion_limits is not None
        and selector.motion_limits.lower is None
        and selector.motion_limits.upper is None,
        details=f"type={selector.articulation_type}, limits={selector.motion_limits}",
    )

    with ctx.pose({top_hinge: 0.0, lever_pivot: 0.0}):
        ctx.expect_gap(
            top,
            base,
            axis="z",
            positive_elem="top_ridge_2",
            negative_elem="lower_ridge_2",
            min_gap=0.002,
            max_gap=0.012,
            name="closed nonstick ridges leave sandwich clearance",
        )
        ctx.expect_overlap(
            top,
            base,
            axes="xy",
            elem_a="top_plate",
            elem_b="lower_plate",
            min_overlap=0.18,
            name="opposing platens align over the sandwich area",
        )
        ctx.expect_contact(
            lever,
            base,
            elem_a="pivot_cap",
            elem_b="lever_boss",
            contact_tol=0.0015,
            name="height lever is mounted on side pivot boss",
        )
        ctx.expect_contact(
            knob,
            base,
            elem_a="knob_cap",
            elem_b="knob_socket",
            contact_tol=0.0015,
            name="selector knob seats on side socket",
        )
        closed_handle = ctx.part_element_world_aabb(top, elem="front_handle")
        closed_paddle = ctx.part_element_world_aabb(lever, elem="lever_paddle")

    with ctx.pose({top_hinge: 1.0}):
        open_handle = ctx.part_element_world_aabb(top, elem="front_handle")

    ctx.check(
        "top platen rotates upward from back hinge",
        closed_handle is not None
        and open_handle is not None
        and open_handle[1][2] > closed_handle[1][2] + 0.10,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    with ctx.pose({lever_pivot: 0.9}):
        raised_paddle = ctx.part_element_world_aabb(lever, elem="lever_paddle")

    ctx.check(
        "height-stop lever rotates upward on side pivot",
        closed_paddle is not None
        and raised_paddle is not None
        and raised_paddle[1][2] > closed_paddle[1][2] + 0.035,
        details=f"closed={closed_paddle}, raised={raised_paddle}",
    )

    return ctx.report()


object_model = build_object_model()
