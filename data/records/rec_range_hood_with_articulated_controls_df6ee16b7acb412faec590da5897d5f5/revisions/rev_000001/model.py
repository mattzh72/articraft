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
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BOTTOM_WIDTH = 0.92
BOTTOM_DEPTH = 0.54
CANOPY_HEIGHT = 0.36
TOP_WIDTH = 0.42
TOP_DEPTH = 0.26
WALL = 0.028
CHIMNEY_WIDTH = 0.34
CHIMNEY_DEPTH = 0.22
CHIMNEY_HEIGHT = 0.78
FRONT_Y = -BOTTOM_DEPTH / 2.0 - 0.038
CONTROL_Z = 0.17
FILTER_WIDTH = 0.39
FILTER_DEPTH = 0.34
FILTER_REAR_Y = 0.18
FILTER_HINGE_Z = -0.008


def _hood_body_shape() -> cq.Workplane:
    """Tall pro-canopy body with a broad flared canopy and rectangular chimney."""
    outer_canopy = (
        cq.Workplane("XY")
        .rect(BOTTOM_WIDTH, BOTTOM_DEPTH)
        .workplane(offset=CANOPY_HEIGHT)
        .rect(TOP_WIDTH, TOP_DEPTH)
        .loft(combine=True)
    )

    chimney_outer = (
        cq.Workplane("XY")
        .box(CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)
        .translate((0.0, 0.035, CANOPY_HEIGHT + CHIMNEY_HEIGHT / 2.0 - 0.015))
    )

    control_apron = (
        cq.Workplane("XY")
        .box(0.86, 0.145, 0.135)
        .translate((0.0, FRONT_Y + 0.0725, CONTROL_Z))
        .edges("|Z")
        .fillet(0.006)
    )
    lower_rolled_lip = (
        cq.Workplane("XY")
        .box(BOTTOM_WIDTH, 0.030, 0.035)
        .translate((0.0, -BOTTOM_DEPTH / 2.0 - 0.004, 0.030))
        .edges("|X")
        .fillet(0.004)
    )

    intake_recess = (
        cq.Workplane("XY")
        .box(0.82, 0.42, 0.085)
        .translate((0.0, -0.005, 0.005))
    )
    return (
        outer_canopy.union(chimney_outer)
        .union(control_apron)
        .union(lower_rolled_lip)
        .cut(intake_recess)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pro_canopy_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.73, 1.0))
    dark = model.material("shadow_black", rgba=(0.015, 0.014, 0.013, 1.0))
    filter_metal = model.material("warm_aluminum", rgba=(0.62, 0.64, 0.61, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    indicator_white = model.material("white_marking", rgba=(0.92, 0.92, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_hood_body_shape(), "hood_body", tolerance=0.0015),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        Box((0.80, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.001, CONTROL_Z)),
        material=dark,
        name="control_band",
    )
    body.visual(
        Box((0.83, 0.43, 0.004)),
        origin=Origin(xyz=(0.0, -0.005, 0.040)),
        material=dark,
        name="intake_shadow",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.86),
        origin=Origin(
            xyz=(0.0, FILTER_REAR_Y, FILTER_HINGE_Z + 0.0095),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="rear_hinge_pin",
    )

    knob = model.part("center_knob")
    knob_shape = KnobGeometry(
        0.090,
        0.050,
        body_style="skirted",
        top_diameter=0.066,
        skirt=KnobSkirt(0.104, 0.010, flare=0.08, chamfer=0.002),
        grip=KnobGrip(style="fluted", count=24, depth=0.0024),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_shape, "center_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    for idx, x in enumerate((-0.22, 0.22)):
        button = model.part(f"side_button_{idx}")
        button.visual(
            Box((0.072, 0.020, 0.046)),
            origin=Origin(xyz=(0.0, -0.010, 0.0)),
            material=black_plastic,
            name="button_cap",
        )
        button.visual(
            Box((0.052, 0.0025, 0.030)),
            origin=Origin(xyz=(0.0, -0.02075, 0.0)),
            material=indicator_white,
            name="button_highlight",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.0065, CONTROL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.012),
        )

    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0065, CONTROL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    filter_shape = SlotPatternPanelGeometry(
        (FILTER_WIDTH, FILTER_DEPTH),
        0.008,
        slot_size=(0.054, 0.010),
        pitch=(0.074, 0.028),
        frame=0.018,
        corner_radius=0.006,
        slot_angle_deg=14.0,
        stagger=True,
    )

    for idx, x in enumerate((-0.205, 0.205)):
        grease_filter = model.part(f"grease_filter_{idx}")
        grease_filter.visual(
            mesh_from_geometry(filter_shape, f"grease_filter_{idx}"),
            origin=Origin(xyz=(0.0, -FILTER_DEPTH / 2.0, -0.006)),
            material=filter_metal,
            name="filter_panel",
        )
        grease_filter.visual(
            Cylinder(radius=0.006, length=FILTER_WIDTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="hinge_barrel",
        )
        grease_filter.visual(
            Box((0.026, 0.020, 0.012)),
            origin=Origin(xyz=(0.0, -FILTER_DEPTH + 0.020, -0.006)),
            material=stainless,
            name="front_pull_tab",
        )
        model.articulation(
            f"body_to_filter_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=grease_filter,
            origin=Origin(xyz=(x, FILTER_REAR_Y, FILTER_HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    knob = object_model.get_part("center_knob")
    knob_joint = object_model.get_articulation("body_to_knob")

    ctx.check(
        "center knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.expect_gap(
        body,
        knob,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem="control_band",
        negative_elem="knob_cap",
        name="knob seats on front control band",
    )

    for idx in range(2):
        button = object_model.get_part(f"side_button_{idx}")
        button_joint = object_model.get_articulation(f"body_to_button_{idx}")
        ctx.check(
            f"side button {idx} is prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={button_joint.articulation_type}",
        )
        ctx.expect_gap(
            body,
            button,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            positive_elem="control_band",
            negative_elem="button_cap",
            name=f"side button {idx} sits proud of panel",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.012}):
            depressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"side button {idx} depresses inward",
            rest_pos is not None
            and depressed_pos is not None
            and depressed_pos[1] > rest_pos[1] + 0.009,
            details=f"rest={rest_pos}, depressed={depressed_pos}",
        )

    for idx in range(2):
        grease_filter = object_model.get_part(f"grease_filter_{idx}")
        filter_joint = object_model.get_articulation(f"body_to_filter_{idx}")
        ctx.check(
            f"grease filter {idx} is rear hinged",
            filter_joint.articulation_type == ArticulationType.REVOLUTE
            and abs(filter_joint.axis[0] - 1.0) < 1e-6,
            details=f"type={filter_joint.articulation_type}, axis={filter_joint.axis}",
        )
        ctx.expect_gap(
            body,
            grease_filter,
            axis="z",
            min_gap=0.0,
            max_gap=0.020,
            positive_elem="body_shell",
            negative_elem="filter_panel",
            name=f"grease filter {idx} hangs just below intake",
        )
        ctx.expect_overlap(
            grease_filter,
            body,
            axes="xy",
            min_overlap=0.28,
            elem_a="filter_panel",
            elem_b="intake_shadow",
            name=f"grease filter {idx} covers intake footprint",
        )
        rest_aabb = ctx.part_world_aabb(grease_filter)
        with ctx.pose({filter_joint: 1.0}):
            open_aabb = ctx.part_world_aabb(grease_filter)
        ctx.check(
            f"grease filter {idx} rotates downward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] < rest_aabb[0][2] - 0.15,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
