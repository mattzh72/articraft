from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_swivel_usb_drive")

    dark_polymer = Material("overmolded_dark_polymer", rgba=(0.035, 0.04, 0.045, 1.0))
    safety_rubber = Material("replaceable_safety_rubber", rgba=(0.95, 0.34, 0.06, 1.0))
    gunmetal = Material("brushed_gunmetal_cover", rgba=(0.32, 0.34, 0.35, 1.0))
    stainless = Material("stainless_steel_hardware", rgba=(0.80, 0.78, 0.72, 1.0))
    black_insert = Material("black_connector_insert", rgba=(0.01, 0.01, 0.012, 1.0))
    label = Material("etched_service_label", rgba=(0.14, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.075, 0.028, 0.012)),
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
        material=dark_polymer,
        name="main_overmold",
    )
    body.visual(
        Box((0.008, 0.030, 0.014)),
        origin=Origin(xyz=(0.054, 0.0, 0.0)),
        material=safety_rubber,
        name="rear_bumper",
    )

    # Open USB-A shell built from four connected plates rather than a solid block.
    body.visual(
        Box((0.024, 0.012, 0.0010)),
        origin=Origin(xyz=(-0.037, 0.0, 0.00255)),
        material=stainless,
        name="connector_top_plate",
    )
    body.visual(
        Box((0.024, 0.012, 0.0010)),
        origin=Origin(xyz=(-0.037, 0.0, -0.00255)),
        material=stainless,
        name="connector_bottom_plate",
    )
    body.visual(
        Box((0.024, 0.0012, 0.0050)),
        origin=Origin(xyz=(-0.037, 0.0054, 0.0)),
        material=stainless,
        name="connector_side_0",
    )
    body.visual(
        Box((0.024, 0.0012, 0.0050)),
        origin=Origin(xyz=(-0.037, -0.0054, 0.0)),
        material=stainless,
        name="connector_side_1",
    )
    body.visual(
        Box((0.019, 0.0070, 0.0012)),
        origin=Origin(xyz=(-0.0345, 0.0, -0.0008)),
        material=black_insert,
        name="connector_tongue",
    )

    # Raised service hatch and screw heads: visible maintenance access without
    # making unsupported islands.
    body.visual(
        Box((0.040, 0.018, 0.0012)),
        origin=Origin(xyz=(0.018, 0.0, 0.0066)),
        material=label,
        name="service_hatch",
    )
    for idx, x in enumerate((0.002, 0.034)):
        body.visual(
            Cylinder(radius=0.0022, length=0.0010),
            origin=Origin(xyz=(x, 0.0062, 0.0077)),
            material=stainless,
            name=f"hatch_screw_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0022, length=0.0010),
            origin=Origin(xyz=(x, -0.0062, 0.0077)),
            material=stainless,
            name=f"hatch_screw_{idx + 2}",
        )

    # Replaceable side wear rails are proud of the overmold but backed directly
    # against it, leaving clearance for the closed swivel cover.
    body.visual(
        Box((0.054, 0.004, 0.006)),
        origin=Origin(xyz=(0.016, 0.016, 0.001)),
        material=safety_rubber,
        name="side_wear_rail_0",
    )
    body.visual(
        Box((0.054, 0.004, 0.006)),
        origin=Origin(xyz=(0.016, -0.016, 0.001)),
        material=safety_rubber,
        name="side_wear_rail_1",
    )

    # Field-service tether loop welded/molded into the rear bumper.
    body.visual(
        Box((0.014, 0.004, 0.004)),
        origin=Origin(xyz=(0.065, 0.008, 0.0)),
        material=stainless,
        name="lanyard_loop_0",
    )
    body.visual(
        Box((0.014, 0.004, 0.004)),
        origin=Origin(xyz=(0.065, -0.008, 0.0)),
        material=stainless,
        name="lanyard_loop_1",
    )
    body.visual(
        Box((0.004, 0.020, 0.004)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=stainless,
        name="lanyard_end_bar",
    )

    # Raised hinge saddle and visible captured pin.  Cylinders are rotated so
    # their length runs along the drive width (local/world Y).
    hinge_rpy = (pi / 2.0, 0.0, 0.0)
    body.visual(
        Box((0.014, 0.024, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.009)),
        material=dark_polymer,
        name="hinge_saddle",
    )
    body.visual(
        Cylinder(radius=0.0050, length=0.024),
        origin=Origin(xyz=(-0.020, 0.0, 0.017), rpy=hinge_rpy),
        material=dark_polymer,
        name="center_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.056),
        origin=Origin(xyz=(-0.020, 0.0, 0.017), rpy=hinge_rpy),
        material=stainless,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0030),
        origin=Origin(xyz=(-0.020, 0.0295, 0.017), rpy=hinge_rpy),
        material=stainless,
        name="pivot_pin_head_0",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0030),
        origin=Origin(xyz=(-0.020, -0.0295, 0.017), rpy=hinge_rpy),
        material=stainless,
        name="pivot_pin_head_1",
    )

    cover = model.part("swivel_cover")
    # Child frame is the pin axis.  In the closed pose the cover extends along
    # local -X over the connector and its side cheeks clear the body rails.
    cover.visual(
        Box((0.044, 0.036, 0.004)),
        origin=Origin(xyz=(-0.034, 0.0, -0.007)),
        material=gunmetal,
        name="cover_top",
    )
    cover.visual(
        Box((0.056, 0.004, 0.021)),
        origin=Origin(xyz=(-0.028, 0.018, -0.0155)),
        material=gunmetal,
        name="cover_side_0",
    )
    cover.visual(
        Box((0.056, 0.004, 0.021)),
        origin=Origin(xyz=(-0.028, -0.018, -0.0155)),
        material=gunmetal,
        name="cover_side_1",
    )
    cover.visual(
        Box((0.006, 0.036, 0.018)),
        origin=Origin(xyz=(-0.059, 0.0, -0.016)),
        material=safety_rubber,
        name="cover_front_bumper",
    )
    cover.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=hinge_rpy),
        material=gunmetal,
        name="cover_collar_0",
    )
    cover.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=hinge_rpy),
        material=gunmetal,
        name="cover_collar_1",
    )
    cover.visual(
        Box((0.026, 0.026, 0.0025)),
        origin=Origin(xyz=(-0.046, 0.0, -0.00375)),
        material=safety_rubber,
        name="cover_wear_pad",
    )
    for idx, y in enumerate((-0.010, 0.010)):
        cover.visual(
            Cylinder(radius=0.0020, length=0.0010),
            origin=Origin(xyz=(-0.046, y, -0.0020)),
            material=stainless,
            name=f"cover_pad_screw_{idx}",
        )

    model.articulation(
        "cover_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.020, 0.0, 0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=2.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("swivel_cover")
    pivot = object_model.get_articulation("cover_pivot")

    ctx.allow_overlap(
        body,
        cover,
        elem_a="pivot_pin",
        elem_b="cover_collar_0",
        reason="The visible steel pivot pin is intentionally captured inside the replaceable cover collar.",
    )
    ctx.allow_overlap(
        body,
        cover,
        elem_a="pivot_pin",
        elem_b="cover_collar_1",
        reason="The visible steel pivot pin is intentionally captured inside the opposite cover collar.",
    )

    ctx.expect_within(
        body,
        cover,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="cover_collar_0",
        margin=0.0,
        name="pin is radially captured in collar 0",
    )
    ctx.expect_within(
        body,
        cover,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="cover_collar_1",
        margin=0.0,
        name="pin is radially captured in collar 1",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="y",
        elem_a="pivot_pin",
        elem_b="cover_collar_0",
        min_overlap=0.006,
        name="collar 0 has retained pin engagement",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="y",
        elem_a="pivot_pin",
        elem_b="cover_collar_1",
        min_overlap=0.006,
        name="collar 1 has retained pin engagement",
    )
    ctx.expect_within(
        body,
        cover,
        axes="xyz",
        inner_elem="connector_tongue",
        margin=0.001,
        name="closed cover surrounds connector tongue",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_top",
        negative_elem="connector_top_plate",
        min_gap=0.0035,
        name="closed cover clears connector shell",
    )

    closed_front = ctx.part_element_world_aabb(cover, elem="cover_front_bumper")
    with ctx.pose({pivot: 2.20}):
        open_front = ctx.part_element_world_aabb(cover, elem="cover_front_bumper")
    ctx.check(
        "swivel cover opens upward around pin",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[1][2] + 0.010,
        details=f"closed={closed_front}, open={open_front}",
    )

    return ctx.report()


object_model = build_object_model()
