from __future__ import annotations

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


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _bolt(part, name, xyz, material):
    part.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_stacker_carriage")

    painted_steel = Material("painted_steel", rgba=(0.86, 0.29, 0.08, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    base_coat = Material("base_coat", rgba=(0.18, 0.20, 0.22, 1.0))
    clear_poly = Material("clear_polycarbonate", rgba=(0.50, 0.82, 1.00, 0.36))
    zinc = Material("zinc_bolt_heads", rgba=(0.70, 0.70, 0.66, 1.0))
    graphite = Material("graphite_slide_pads", rgba=(0.04, 0.045, 0.045, 1.0))

    uprights = model.part("uprights")
    _box(uprights, "base_plate", (1.10, 0.60, 0.050), (0.0, 0.0, 0.025), base_coat)
    _box(uprights, "lower_tie", (0.94, 0.075, 0.080), (0.0, 0.0775, 0.220), dark_steel)
    _box(uprights, "top_bridge", (0.94, 0.130, 0.080), (0.0, 0.0, 1.740), dark_steel)

    column_x = (-0.36, 0.36)
    for i, x in enumerate(column_x):
        _box(uprights, f"foot_plate_{i}", (0.18, 0.16, 0.020), (x, 0.0, 0.060), dark_steel)
        _box(uprights, f"column_{i}", (0.080, 0.080, 1.630), (x, 0.0, 0.885), dark_steel)
        _box(uprights, f"front_wear_strip_{i}", (0.050, 0.010, 1.500), (x, -0.035, 0.865), zinc)
        for j, bx in enumerate((-0.055, 0.055)):
            _bolt(uprights, f"bolt_{i}_{j}_front", (x + bx, -0.050, 0.075), zinc)
            _bolt(uprights, f"bolt_{i}_{j}_rear", (x + bx, 0.050, 0.075), zinc)

    carriage = model.part("carriage")
    _box(carriage, "front_plate", (0.88, 0.050, 0.420), (0.0, -0.220, 0.0), painted_steel)
    _box(carriage, "top_flange", (0.95, 0.070, 0.050), (0.0, -0.220, 0.235), painted_steel)
    _box(carriage, "bottom_flange", (0.95, 0.070, 0.050), (0.0, -0.220, -0.235), painted_steel)
    _box(carriage, "center_rib", (0.055, 0.085, 0.420), (0.0, -0.2375, 0.0), painted_steel)
    _box(carriage, "side_rib_0", (0.050, 0.080, 0.420), (-0.450, -0.225, 0.0), painted_steel)
    _box(carriage, "side_rib_1", (0.050, 0.080, 0.420), (0.450, -0.225, 0.0), painted_steel)

    guide_height = 0.520
    guide_hole = 0.086
    guide_wall = 0.025
    guide_side_offset = guide_hole * 0.5 + guide_wall * 0.5
    guide_outer = guide_hole + 2.0 * guide_wall
    guide_cross = guide_hole + 0.006

    for i, x in enumerate(column_x):
        for z in (-0.155, 0.155):
            _box(
                carriage,
                f"guide_arm_{i}_{'lower' if z < 0 else 'upper'}",
                (0.170, 0.130, 0.060),
                (x, -0.1325, z),
                painted_steel,
            )
        _box(
            carriage,
            f"guide_{i}_front_bar",
            (guide_cross, guide_wall, guide_height),
            (x, -guide_side_offset, 0.0),
            clear_poly,
        )
        _box(
            carriage,
            f"guide_{i}_rear_bar",
            (guide_cross, guide_wall, guide_height),
            (x, guide_side_offset, 0.0),
            clear_poly,
        )
        _box(
            carriage,
            f"guide_{i}_xneg_bar",
            (guide_wall, guide_outer, guide_height),
            (x - guide_side_offset, 0.0, 0.0),
            clear_poly,
        )
        _box(
            carriage,
            f"guide_{i}_xpos_bar",
            (guide_wall, guide_outer, guide_height),
            (x + guide_side_offset, 0.0, 0.0),
            clear_poly,
        )
        _box(carriage, f"front_pad_{i}", (0.038, 0.0035, 0.110), (x, -0.04175, 0.0), graphite)
        _box(carriage, f"rear_pad_{i}", (0.038, 0.0035, 0.110), (x, 0.04175, 0.0), graphite)
        _box(carriage, f"xneg_pad_{i}", (0.0035, 0.038, 0.110), (x - 0.04175, 0.0, 0.0), graphite)
        _box(carriage, f"xpos_pad_{i}", (0.0035, 0.038, 0.110), (x + 0.04175, 0.0, 0.0), graphite)

    model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=uprights,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.85),
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

    uprights = object_model.get_part("uprights")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("lift_slide")

    ctx.check(
        "one vertical prismatic carriage joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"joint_type={slide.articulation_type}, axis={slide.axis}",
    )

    for i in range(2):
        ctx.expect_gap(
            uprights,
            carriage,
            axis="x",
            positive_elem=f"column_{i}",
            negative_elem=f"guide_{i}_xneg_bar",
            min_gap=0.002,
            max_gap=0.0045,
            name=f"column {i} clears negative-x guide face",
        )
        ctx.expect_gap(
            carriage,
            uprights,
            axis="x",
            positive_elem=f"guide_{i}_xpos_bar",
            negative_elem=f"column_{i}",
            min_gap=0.002,
            max_gap=0.0045,
            name=f"column {i} clears positive-x guide face",
        )
        ctx.expect_gap(
            uprights,
            carriage,
            axis="y",
            positive_elem=f"column_{i}",
            negative_elem=f"guide_{i}_front_bar",
            min_gap=0.002,
            max_gap=0.0045,
            name=f"column {i} clears front guide face",
        )
        ctx.expect_gap(
            carriage,
            uprights,
            axis="y",
            positive_elem=f"guide_{i}_rear_bar",
            negative_elem=f"column_{i}",
            min_gap=0.002,
            max_gap=0.0045,
            name=f"column {i} clears rear guide face",
        )

    ctx.expect_contact(
        carriage,
        uprights,
        contact_tol=1e-6,
        name="slide pads bear on the grounded columns",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.85}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            uprights,
            contact_tol=1e-6,
            name="raised slide pads remain on the columns",
        )
        ctx.expect_gap(
            uprights,
            carriage,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="guide_0_xpos_bar",
            min_gap=0.025,
            name="raised carriage stays below top bridge",
        )

    ctx.check(
        "carriage rises along the grounded columns",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.80,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
