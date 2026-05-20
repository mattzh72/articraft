from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_DEPTH = 0.36
BASE_WIDTH = 0.44
BASE_HEIGHT = 0.080
FRONT_X = -BASE_DEPTH / 2.0
REAR_X = BASE_DEPTH / 2.0
RIGHT_Y = -BASE_WIDTH / 2.0


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """Small CadQuery helper for molded appliance panels."""
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.12)


def _base_housing_mesh():
    """Rounded lower housing with a real front channel for the grease drawer."""
    housing = cq.Workplane("XY").box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT)
    drawer_channel = (
        cq.Workplane("XY")
        .box(0.235, 0.360, 0.030)
        .translate((FRONT_X + 0.118, 0.0, -BASE_HEIGHT / 2.0 + 0.036))
    )
    housing = housing.cut(drawer_channel)
    housing = housing.edges("|Z").fillet(0.018)
    return mesh_from_cadquery(housing, "lower_housing", tolerance=0.0008, angular_tolerance=0.12)


def _handle_mesh():
    """One continuous bent stainless tube mounted to the front of the upper platen."""
    handle_path = [
        (-0.255, -0.165, 0.055),
        (-0.295, -0.165, 0.074),
        (-0.326, -0.115, 0.092),
        (-0.334, 0.000, 0.098),
        (-0.326, 0.115, 0.092),
        (-0.295, 0.165, 0.074),
        (-0.255, 0.165, 0.055),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            handle_path,
            radius=0.009,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        "front_handle_tube",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_panini_press")

    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.027, 0.028, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.012, 1.0))
    cast_iron = model.material("seasoned_cast_iron", rgba=(0.035, 0.034, 0.032, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.66, 0.63, 0.57, 1.0))
    warm_grey = model.material("warm_grey_label", rgba=(0.36, 0.34, 0.31, 1.0))
    drawer_mat = model.material("grease_tray_metal", rgba=(0.48, 0.46, 0.40, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_DEPTH, BASE_WIDTH, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=dark_plastic,
        name="lower_housing",
    )
    base.visual(
        Box((BASE_DEPTH, 0.035, 0.059)),
        origin=Origin(xyz=(0.0, -0.2025, 0.0505)),
        material=dark_plastic,
        name="side_wall_0",
    )
    base.visual(
        Box((BASE_DEPTH, 0.035, 0.059)),
        origin=Origin(xyz=(0.0, 0.2025, 0.0505)),
        material=dark_plastic,
        name="side_wall_1",
    )
    base.visual(
        Box((0.055, 0.370, 0.059)),
        origin=Origin(xyz=(REAR_X - 0.0275, 0.0, 0.0505)),
        material=dark_plastic,
        name="rear_wall",
    )
    base.visual(
        Box((0.330, 0.400, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.071)),
        material=dark_plastic,
        name="top_deck",
    )
    base.visual(
        Box((0.286, 0.352, 0.009)),
        origin=Origin(xyz=(-0.024, 0.0, BASE_HEIGHT + 0.003)),
        material=cast_iron,
        name="lower_cooking_plate",
    )
    lower_ridges = [
        ("lower_ridge_0", -0.140),
        ("lower_ridge_1", -0.112),
        ("lower_ridge_2", -0.084),
        ("lower_ridge_3", -0.056),
        ("lower_ridge_4", -0.028),
        ("lower_ridge_5", 0.000),
        ("lower_ridge_6", 0.028),
        ("lower_ridge_7", 0.056),
        ("lower_ridge_8", 0.084),
        ("lower_ridge_9", 0.112),
        ("lower_ridge_10", 0.140),
    ]
    for ridge_name, y in lower_ridges:
        base.visual(
            Box((0.246, 0.010, 0.007)),
            origin=Origin(xyz=(-0.026, y, BASE_HEIGHT + 0.011)),
            material=satin_black,
            name=ridge_name,
        )

    # Slightly raised feet make it read as a countertop appliance and keep the drawer off the table.
    for idx, (x, y) in enumerate(
        [
            (FRONT_X + 0.045, -0.165),
            (FRONT_X + 0.045, 0.165),
            (REAR_X - 0.045, -0.165),
            (REAR_X - 0.045, 0.165),
        ]
    ):
        base.visual(
            Box((0.060, 0.045, 0.012)),
            origin=Origin(xyz=(x, y, -0.003)),
            material=satin_black,
            name=f"rubber_foot_{idx}",
        )

    # Rear hinge supports and a visible horizontal hinge pin.
    base.visual(
        Box((0.034, 0.034, 0.062)),
        origin=Origin(xyz=(REAR_X - 0.008, -0.236, BASE_HEIGHT + 0.030)),
        material=dark_plastic,
        name="hinge_cheek_0",
    )
    base.visual(
        Box((0.034, 0.034, 0.062)),
        origin=Origin(xyz=(REAR_X - 0.008, 0.236, BASE_HEIGHT + 0.030)),
        material=dark_plastic,
        name="hinge_cheek_1",
    )
    base.visual(
        Box((0.026, 0.500, 0.020)),
        origin=Origin(xyz=(REAR_X - 0.009, 0.0, BASE_HEIGHT + 0.060)),
        material=stainless,
        name="hinge_pin_cover",
    )

    # A side control escutcheon is fixed to the right wall; the knob itself is articulated.
    base.visual(
        Box((0.115, 0.006, 0.055)),
        origin=Origin(xyz=(-0.030, RIGHT_Y - 0.002, 0.055)),
        material=warm_grey,
        name="control_panel",
    )
    for idx, x in enumerate([-0.072, -0.052, -0.008, 0.012]):
        base.visual(
            Box((0.003, 0.004, 0.013)),
            origin=Origin(xyz=(x, RIGHT_Y - 0.006, 0.081)),
            material=stainless,
            name=f"control_tick_{idx}",
        )

    upper = model.part("upper_platen")
    upper.visual(
        _rounded_box_mesh((0.305, 0.430, 0.088), 0.017, "upper_platen_shell"),
        origin=Origin(xyz=(-0.156, 0.0, 0.040)),
        material=stainless,
        name="upper_shell",
    )
    upper.visual(
        Box((0.020, 0.360, 0.018)),
        origin=Origin(xyz=(-0.007, 0.0, 0.000)),
        material=stainless,
        name="upper_hinge_leaf",
    )
    upper.visual(
        Box((0.268, 0.344, 0.008)),
        origin=Origin(xyz=(-0.150, 0.0, -0.017)),
        material=cast_iron,
        name="upper_cooking_plate",
    )
    for idx, y in enumerate([-0.178, 0.178]):
        upper.visual(
            Box((0.278, 0.012, 0.010)),
            origin=Origin(xyz=(-0.150, y, -0.009)),
            material=satin_black,
            name=f"upper_plate_carrier_{idx}",
        )
    upper_ridges = [
        ("upper_ridge_0", -0.140),
        ("upper_ridge_1", -0.112),
        ("upper_ridge_2", -0.084),
        ("upper_ridge_3", -0.056),
        ("upper_ridge_4", -0.028),
        ("upper_ridge_5", 0.000),
        ("upper_ridge_6", 0.028),
        ("upper_ridge_7", 0.056),
        ("upper_ridge_8", 0.084),
        ("upper_ridge_9", 0.112),
        ("upper_ridge_10", 0.140),
    ]
    for ridge_name, y in upper_ridges:
        upper.visual(
            Box((0.230, 0.010, 0.006)),
            origin=Origin(xyz=(-0.150, y, -0.024)),
            material=satin_black,
            name=ridge_name,
        )
    for idx, y in enumerate([-0.165, 0.165]):
        upper.visual(
            Box((0.028, 0.040, 0.030)),
            origin=Origin(xyz=(-0.255, y, 0.046)),
            material=stainless,
            name=f"handle_mount_{idx}",
        )
    upper.visual(
        _handle_mesh(),
        material=stainless,
        name="front_handle",
    )

    drawer = model.part("grease_drawer")
    drawer.visual(
        Box((0.160, 0.318, 0.014)),
        origin=Origin(xyz=(0.082, 0.0, 0.000)),
        material=drawer_mat,
        name="drawer_pan",
    )
    drawer.visual(
        Box((0.014, 0.350, 0.032)),
        origin=Origin(xyz=(-0.005, 0.0, 0.000)),
        material=dark_plastic,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.007, 0.155, 0.010)),
        origin=Origin(xyz=(-0.0155, 0.0, -0.002)),
        material=satin_black,
        name="drawer_pull_groove",
    )

    knob = model.part("browning_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.024,
            body_style="skirted",
            top_diameter=0.044,
            edge_radius=0.0015,
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "browning_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="knob_cap",
    )

    model.articulation(
        "base_to_upper_platen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(REAR_X - 0.025, 0.0, BASE_HEIGHT + 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "base_to_grease_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(FRONT_X - 0.002, 0.0, 0.028)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "base_to_browning_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(-0.030, RIGHT_Y - 0.005, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
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

    base = object_model.get_part("base")
    upper = object_model.get_part("upper_platen")
    drawer = object_model.get_part("grease_drawer")
    knob = object_model.get_part("browning_knob")
    hinge = object_model.get_articulation("base_to_upper_platen")
    drawer_slide = object_model.get_articulation("base_to_grease_drawer")
    knob_spin = object_model.get_articulation("base_to_browning_knob")

    ctx.check(
        "upper platen uses rear revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "browning knob is a continuous rotary control",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )
    ctx.check(
        "grease drawer is a prismatic slider",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_slide.articulation_type}",
    )

    with ctx.pose({hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_overlap(
            upper,
            base,
            axes="xy",
            elem_a="upper_cooking_plate",
            elem_b="lower_cooking_plate",
            min_overlap=0.22,
            name="closed cooking plates align over each other",
        )
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem="upper_ridge_5",
            negative_elem="lower_ridge_5",
            min_gap=0.002,
            max_gap=0.020,
            name="closed grill ribs have a small cooking gap",
        )
        ctx.expect_within(
            drawer,
            base,
            axes="y",
            inner_elem="drawer_pan",
            outer_elem="lower_housing",
            margin=0.010,
            name="drawer pan is centered in the front channel",
        )

    rest_handle_aabb = ctx.part_element_world_aabb(upper, elem="front_handle")
    with ctx.pose({hinge: 1.05}):
        open_handle_aabb = ctx.part_element_world_aabb(upper, elem="front_handle")
    ctx.check(
        "positive hinge angle lifts the front platen",
        rest_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][2] > rest_handle_aabb[0][2] + 0.080,
        details=f"rest={rest_handle_aabb}, open={open_handle_aabb}",
    )

    rest_drawer_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_slide: 0.120}):
        extended_drawer_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            elem_a="drawer_pan",
            elem_b="lower_housing",
            min_overlap=0.035,
            name="extended grease drawer remains retained in base",
        )
    ctx.check(
        "positive drawer travel slides out from the front",
        rest_drawer_aabb is not None
        and extended_drawer_aabb is not None
        and extended_drawer_aabb[0][0] < rest_drawer_aabb[0][0] - 0.090,
        details=f"rest={rest_drawer_aabb}, extended={extended_drawer_aabb}",
    )

    ctx.expect_contact(
        knob,
        base,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.003,
        name="rotary knob sits on right side control panel",
    )

    return ctx.report()


object_model = build_object_model()
