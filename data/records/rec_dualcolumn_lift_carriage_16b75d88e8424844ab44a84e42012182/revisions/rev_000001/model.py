from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_lift_carriage")

    frame_steel = model.material("welded_dark_steel", rgba=(0.16, 0.18, 0.19, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    carriage_yellow = model.material("painted_carriage_yellow", rgba=(0.95, 0.66, 0.12, 1.0))
    bronze = model.material("bronze_liners", rgba=(0.74, 0.48, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    bolt_dark = model.material("black_oxide_bolts", rgba=(0.04, 0.045, 0.05, 1.0))

    frame = model.part("frame")

    def frame_box(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material: Material = frame_steel) -> None:
        frame.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Heavy welded base: two broad feet tied by front/rear cross tubes.
    for i, x in enumerate((-0.45, 0.45)):
        frame_box(f"sole_foot_{i}", (0.32, 0.78, 0.045), (x, 0.0, 0.0225))
        frame_box(f"base_runner_{i}", (0.22, 0.66, 0.10), (x, 0.0, 0.080))
        frame_box(f"column_socket_{i}", (0.28, 0.30, 0.11), (x, 0.0, 0.145))
        for y in (-0.27, 0.27):
            frame.visual(
                Cylinder(radius=0.027, length=0.016),
                origin=Origin(xyz=(x - 0.075, y, 0.107)),
                material=bolt_dark,
                name=f"anchor_bolt_{i}_{'front' if y < 0 else 'rear'}_0",
            )
            frame.visual(
                Cylinder(radius=0.027, length=0.016),
                origin=Origin(xyz=(x + 0.075, y, 0.107)),
                material=bolt_dark,
                name=f"anchor_bolt_{i}_{'front' if y < 0 else 'rear'}_1",
            )

    frame_box("front_base_tube", (1.16, 0.12, 0.12), (0.0, -0.31, 0.105))
    frame_box("rear_base_tube", (1.16, 0.12, 0.12), (0.0, 0.31, 0.105))
    frame_box("center_tie_tube", (1.02, 0.12, 0.08), (0.0, 0.0, 0.13))

    # Rectangular guide columns with bolted-on machined wear strips.
    column_height = 1.48
    column_center_z = 0.86
    for i, x in enumerate((-0.45, 0.45)):
        column_name = "guide_column_0" if i == 0 else "guide_column_1"
        front_way_name = "front_way_0" if i == 0 else "front_way_1"
        rear_way_name = "rear_way_0" if i == 0 else "rear_way_1"
        inner_way_name = "inner_side_way_0" if i == 0 else "inner_side_way_1"
        outer_way_name = "outer_side_way_0" if i == 0 else "outer_side_way_1"
        frame_box(column_name, (0.11, 0.14, column_height), (x, 0.0, column_center_z))
        frame_box(front_way_name, (0.088, 0.010, 1.30), (x, -0.075, 0.88), machined_steel)
        frame_box(rear_way_name, (0.088, 0.010, 1.30), (x, 0.075, 0.88), machined_steel)
        frame_box(inner_way_name, (0.010, 0.100, 1.30), (x + (0.060 if x < 0 else -0.060), 0.0, 0.88), machined_steel)
        frame_box(outer_way_name, (0.010, 0.100, 1.30), (x - (0.060 if x < 0 else -0.060), 0.0, 0.88), machined_steel)

        # Web gussets make the load path from the upright into the foot visible.
        frame_box(f"front_gusset_{i}", (0.22, 0.026, 0.34), (x, -0.158, 0.275))
        frame_box(f"rear_gusset_{i}", (0.22, 0.026, 0.34), (x, 0.158, 0.275))
        frame_box(f"lower_stop_pad_{i}", (0.16, 0.030, 0.080), (x, -0.085, 0.480), rubber)
        upper_stop_name = "upper_stop_pad_0" if i == 0 else "upper_stop_pad_1"
        frame_box(upper_stop_name, (0.16, 0.030, 0.080), (x, -0.085, 1.610), rubber)

    # Upper tie beam clamps the two columns into one portal frame.
    frame_box("upper_tie_beam", (1.15, 0.22, 0.16), (0.0, 0.0, 1.66))
    frame_box("upper_front_lip", (1.03, 0.045, 0.08), (0.0, -0.132, 1.575))
    frame_box("upper_rear_lip", (1.03, 0.045, 0.08), (0.0, 0.132, 1.575))

    carriage = model.part("carriage")

    def carriage_box(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material: Material = carriage_yellow) -> None:
        carriage.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Broad front carriage plate.  It is in front of the columns, while the
    # guide shoes wrap around each column so the sliding part reads as captured.
    carriage_box("front_plate", (1.18, 0.060, 0.50), (0.0, -0.175, 0.0))
    carriage_box("top_plate_cap", (1.00, 0.080, 0.055), (0.0, -0.172, 0.255))
    carriage_box("bottom_plate_cap", (1.00, 0.080, 0.055), (0.0, -0.172, -0.255))
    carriage_box("center_horizontal_rib", (0.96, 0.035, 0.055), (0.0, -0.219, 0.0))
    carriage_box("left_vertical_rib", (0.055, 0.035, 0.45), (-0.25, -0.219, 0.0))
    carriage_box("right_vertical_rib", (0.055, 0.035, 0.45), (0.25, -0.219, 0.0))
    carriage_box("center_vertical_rib", (0.050, 0.035, 0.44), (0.0, -0.219, 0.0))

    shoe_outer_x = 0.28
    shoe_outer_y = 0.30
    side_t = 0.045
    face_t = 0.045
    shoe_h = 0.18
    side_offset_x = shoe_outer_x * 0.5 - side_t * 0.5
    face_offset_y = shoe_outer_y * 0.5 - face_t * 0.5

    for col_i, x in enumerate((-0.45, 0.45)):
        for level_i, z in enumerate((-0.16, 0.16)):
            prefix = f"shoe_{col_i}_{level_i}"
            front_bar_name = "shoe_0_1_front_bar" if col_i == 0 and level_i == 1 else f"{prefix}_front_bar"
            carriage_box(front_bar_name, (shoe_outer_x, face_t, shoe_h), (x, -face_offset_y, z))
            carriage_box(f"{prefix}_rear_bar", (shoe_outer_x, face_t, shoe_h), (x, face_offset_y, z))
            carriage_box(f"{prefix}_inner_cheek", (side_t, shoe_outer_y, shoe_h), (x + (side_offset_x if x < 0 else -side_offset_x), 0.0, z))
            carriage_box(f"{prefix}_outer_cheek", (side_t, shoe_outer_y, shoe_h), (x - (side_offset_x if x < 0 else -side_offset_x), 0.0, z))

            # Replaceable machined liner pads set inside the boxed shoe.
            front_liner_name = "shoe_0_0_front_liner" if col_i == 0 and level_i == 0 else f"{prefix}_front_liner"
            rear_liner_name = "shoe_0_0_rear_liner" if col_i == 0 and level_i == 0 else f"{prefix}_rear_liner"
            inner_liner_name = "shoe_0_0_inner_liner" if col_i == 0 and level_i == 0 else f"{prefix}_inner_liner"
            outer_liner_name = "shoe_0_0_outer_liner" if col_i == 0 and level_i == 0 else f"{prefix}_outer_liner"
            carriage_box(front_liner_name, (0.18, 0.025, 0.145), (x, -0.0925, z), bronze)
            carriage_box(rear_liner_name, (0.18, 0.025, 0.145), (x, 0.0925, z), bronze)
            carriage_box(
                inner_liner_name,
                (0.030, 0.18, 0.145),
                (x + (0.080 if x < 0 else -0.080), 0.0, z),
                bronze,
            )
            carriage_box(
                outer_liner_name,
                (0.030, 0.18, 0.145),
                (x - (0.080 if x < 0 else -0.080), 0.0, z),
                bronze,
            )

        # Deep welded pads join the boxed shoes to the plate so the shoes do
        # not read as isolated collars.
        carriage_box(f"shoe_web_{col_i}", (0.30, 0.070, 0.42), (x, -0.158, 0.0))
        carriage_box(f"side_load_rib_{col_i}", (0.045, 0.11, 0.42), (x * 0.78, -0.190, 0.0))

    slide = model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.18, lower=0.0, upper=0.52),
        motion_properties=MotionProperties(damping=180.0, friction=60.0),
    )
    slide.meta["qc_samples"] = [0.0, 0.26, 0.52]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("lift_slide")

    ctx.check(
        "single vertical prismatic lift axis",
        slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.52,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    # Representative guide-shoe clearances prove the carriage is captured
    # around the columns without clipping through the fixed ways.
    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem="shoe_0_0_rear_liner",
        negative_elem="rear_way_0",
        min_gap=0.0,
        max_gap=0.001,
        name="rear liner rides on rear way",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem="front_way_0",
        negative_elem="shoe_0_0_front_liner",
        min_gap=0.0,
        max_gap=0.001,
        name="front liner rides on front way",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem="shoe_0_0_inner_liner",
        negative_elem="inner_side_way_0",
        min_gap=0.0,
        max_gap=0.001,
        name="inner side liner rides on way",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem="outer_side_way_0",
        negative_elem="shoe_0_0_outer_liner",
        min_gap=0.0,
        max_gap=0.001,
        name="outer side liner rides on way",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="shoe_0_0_front_liner",
        elem_b="guide_column_0",
        min_overlap=0.12,
        name="lower guide shoe remains engaged",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.52}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="upper_stop_pad_0",
            negative_elem="shoe_0_1_front_bar",
            min_gap=0.015,
            max_gap=0.055,
            name="upper stop has running clearance",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="shoe_0_0_front_liner",
            elem_b="guide_column_0",
            min_overlap=0.12,
            name="extended guide shoe remains engaged",
        )

    ctx.check(
        "carriage translates upward on lift slide",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.50,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
