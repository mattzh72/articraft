from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.46
BASE_DEPTH = 0.34
COLUMN_X = 0.205
REAR_Y = 0.145
SLEEVE_TOP_Z = 0.150
COLUMN_TRAVEL = 0.100
COLUMN_TOP_LOCAL_Z = 0.100


def _box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    anodized = Material("dark anodized aluminum", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed = Material("brushed aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    black_rubber = Material("matte black rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    tray_finish = Material("satin graphite tray", rgba=(0.15, 0.16, 0.17, 1.0))

    base = model.part("base_frame")
    rail_h = 0.018
    rail_w = 0.035
    _box(base, "front_rail", (BASE_WIDTH, rail_w, rail_h), (0.0, -0.165, rail_h / 2), anodized)
    _box(base, "rear_rail", (BASE_WIDTH, rail_w, rail_h), (0.0, REAR_Y, rail_h / 2), anodized)
    _box(base, "side_rail_0", (rail_w, BASE_DEPTH, rail_h), (-COLUMN_X, -0.010, rail_h / 2), anodized)
    _box(base, "side_rail_1", (rail_w, BASE_DEPTH, rail_h), (COLUMN_X, -0.010, rail_h / 2), anodized)

    # Small rubber pads make the frame read as resting on a desk rather than as a
    # floating metal outline.
    for i, x in enumerate((-0.190, 0.190)):
        for j, y in enumerate((-0.165, REAR_Y)):
            _box(base, f"foot_{i}_{j}", (0.050, 0.028, 0.008), (x, y, -0.004), black_rubber)

    # Each rear corner carries a short hollow outer sleeve.  The inner columns
    # slide through the clear central opening rather than through a solid proxy.
    sleeve_outer = 0.052
    sleeve_wall = 0.007
    sleeve_h = SLEEVE_TOP_Z - rail_h + 0.002
    sleeve_z = rail_h - 0.001 + sleeve_h / 2
    for side_name, x in (("left", -COLUMN_X), ("right", COLUMN_X)):
        _box(
            base,
            f"{side_name}_sleeve_front_wall",
            (sleeve_outer, sleeve_wall, sleeve_h),
            (x, REAR_Y - sleeve_outer / 2 + sleeve_wall / 2, sleeve_z),
            anodized,
        )
        _box(
            base,
            f"{side_name}_sleeve_rear_wall",
            (sleeve_outer, sleeve_wall, sleeve_h),
            (x, REAR_Y + sleeve_outer / 2 - sleeve_wall / 2, sleeve_z),
            anodized,
        )
        _box(
            base,
            f"{side_name}_sleeve_side_wall_0",
            (sleeve_wall, sleeve_outer, sleeve_h),
            (x - sleeve_outer / 2 + sleeve_wall / 2, REAR_Y, sleeve_z),
            anodized,
        )
        _box(
            base,
            f"{side_name}_sleeve_side_wall_1",
            (sleeve_wall, sleeve_outer, sleeve_h),
            (x + sleeve_outer / 2 - sleeve_wall / 2, REAR_Y, sleeve_z),
            anodized,
        )

    column_size = 0.026
    cap_size = 0.034
    for part_name in ("left_column", "right_column"):
        column = model.part(part_name)
        _box(
            column,
            "sliding_tube",
            (column_size, column_size, 0.230),
            (0.0, 0.0, -0.015),
            brushed,
        )
        _box(
            column,
            "top_cap",
            (cap_size, cap_size, 0.010),
            (0.0, 0.0, COLUMN_TOP_LOCAL_Z - 0.005),
            brushed,
        )
        _box(
            column,
            "stop_collar",
            (0.044, 0.044, 0.008),
            (0.0, 0.0, 0.005),
            brushed,
        )
        _box(
            column,
            "height_mark",
            (0.004, 0.002, 0.120),
            (0.0, -column_size / 2 - 0.001, -0.035),
            black_rubber,
        )

    left_column = model.get_part("left_column")
    right_column = model.get_part("right_column")

    slide_limits = MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=COLUMN_TRAVEL)
    slide_friction = MotionProperties(damping=8.0, friction=3.0)
    model.articulation(
        "left_column_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_column,
        origin=Origin(xyz=(-COLUMN_X, REAR_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=slide_limits,
        motion_properties=slide_friction,
    )
    model.articulation(
        "right_column_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_column,
        origin=Origin(xyz=(COLUMN_X, REAR_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=slide_limits,
        motion_properties=slide_friction,
        mimic=Mimic(joint="left_column_slide", multiplier=1.0, offset=0.0),
    )

    tray = model.part("tray")
    # The tray frame sits at the top of the left column; all tray geometry is a
    # single rigid part that spans across to the right column.
    _box(tray, "tray_panel", (0.460, 0.310, 0.018), (0.205, -0.130, 0.009), tray_finish)
    _box(tray, "rear_beam", (0.460, 0.050, 0.032), (0.205, 0.000, 0.016), anodized)
    _box(tray, "front_lip", (0.420, 0.018, 0.055), (0.205, -0.285, 0.0455), tray_finish)
    _box(tray, "side_lip_0", (0.018, 0.270, 0.035), (-0.016, -0.130, 0.0355), tray_finish)
    _box(tray, "side_lip_1", (0.018, 0.270, 0.035), (0.426, -0.130, 0.0355), tray_finish)
    for i, x in enumerate((0.095, 0.315)):
        _box(tray, f"rubber_strip_{i}", (0.110, 0.018, 0.006), (x, -0.190, 0.021), black_rubber)
        _box(tray, f"rubber_strip_{i + 2}", (0.110, 0.018, 0.006), (x, -0.060, 0.021), black_rubber)

    model.articulation(
        "left_column_to_tray",
        ArticulationType.FIXED,
        parent=left_column,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_LOCAL_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    left_column = object_model.get_part("left_column")
    right_column = object_model.get_part("right_column")
    tray = object_model.get_part("tray")
    left_slide = object_model.get_articulation("left_column_slide")
    right_slide = object_model.get_articulation("right_column_slide")

    ctx.check(
        "right column is mechanically matched to left column",
        getattr(right_slide, "mimic", None) is not None
        and right_slide.mimic.joint == "left_column_slide"
        and abs(right_slide.mimic.multiplier - 1.0) < 1e-9,
        details="The two rear column sliders should travel together with identical displacement.",
    )

    ctx.expect_origin_gap(
        right_column,
        left_column,
        axis="x",
        min_gap=0.409,
        max_gap=0.411,
        name="columns keep fixed width spacing",
    )
    ctx.expect_origin_distance(
        left_column,
        right_column,
        axes="yz",
        max_dist=0.001,
        name="columns start level and co-linear",
    )
    ctx.expect_overlap(
        left_column,
        base,
        axes="z",
        elem_a="sliding_tube",
        elem_b="left_sleeve_front_wall",
        min_overlap=0.030,
        name="left column remains inserted in sleeve at rest",
    )
    ctx.expect_overlap(
        right_column,
        base,
        axes="z",
        elem_a="sliding_tube",
        elem_b="right_sleeve_front_wall",
        min_overlap=0.030,
        name="right column remains inserted in sleeve at rest",
    )
    ctx.expect_gap(
        left_column,
        base,
        axis="z",
        positive_elem="stop_collar",
        negative_elem="left_sleeve_front_wall",
        max_gap=0.001,
        max_penetration=0.00001,
        name="left stop collar rests on sleeve",
    )
    ctx.expect_gap(
        right_column,
        base,
        axis="z",
        positive_elem="stop_collar",
        negative_elem="right_sleeve_front_wall",
        max_gap=0.001,
        max_penetration=0.00001,
        name="right stop collar rests on sleeve",
    )
    ctx.expect_overlap(
        tray,
        right_column,
        axes="xy",
        elem_a="rear_beam",
        elem_b="top_cap",
        min_overlap=0.025,
        name="rigid tray spans over right column top",
    )
    ctx.expect_gap(
        tray,
        right_column,
        axis="z",
        positive_elem="rear_beam",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="right column top seats against tray beam",
    )

    left_rest = ctx.part_world_position(left_column)
    right_rest = ctx.part_world_position(right_column)
    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({left_slide: COLUMN_TRAVEL}):
        ctx.expect_origin_gap(
            right_column,
            left_column,
            axis="x",
            min_gap=0.409,
            max_gap=0.411,
            name="extended columns keep fixed width spacing",
        )
        ctx.expect_origin_distance(
            left_column,
            right_column,
            axes="yz",
            max_dist=0.001,
            name="extended columns remain level",
        )
        ctx.expect_overlap(
            left_column,
            base,
            axes="z",
            elem_a="sliding_tube",
            elem_b="left_sleeve_front_wall",
            min_overlap=0.025,
            name="left column remains inserted when raised",
        )
        ctx.expect_overlap(
            right_column,
            base,
            axes="z",
            elem_a="sliding_tube",
            elem_b="right_sleeve_front_wall",
            min_overlap=0.025,
            name="right column remains inserted when raised",
        )
        ctx.expect_gap(
            tray,
            right_column,
            axis="z",
            positive_elem="rear_beam",
            negative_elem="top_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name="raised right column still supports tray beam",
        )
        left_extended = ctx.part_world_position(left_column)
        right_extended = ctx.part_world_position(right_column)
        tray_extended = ctx.part_world_position(tray)

    ctx.check(
        "left column translates upward",
        left_rest is not None
        and left_extended is not None
        and left_extended[2] > left_rest[2] + COLUMN_TRAVEL - 0.002,
        details=f"rest={left_rest}, extended={left_extended}",
    )
    ctx.check(
        "right column follows upward",
        right_rest is not None
        and right_extended is not None
        and right_extended[2] > right_rest[2] + COLUMN_TRAVEL - 0.002,
        details=f"rest={right_rest}, extended={right_extended}",
    )
    ctx.check(
        "tray rides rigidly with columns",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[2] > tray_rest[2] + COLUMN_TRAVEL - 0.002,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
