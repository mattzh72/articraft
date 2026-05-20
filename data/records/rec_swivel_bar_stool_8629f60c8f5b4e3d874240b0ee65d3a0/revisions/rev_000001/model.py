from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    vinyl = model.material("vinyl", rgba=(0.18, 0.15, 0.13, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.22, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=powder_black,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.19, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=powder_black,
        name="base_riser",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=satin_metal,
        name="lower_sleeve",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=chrome,
        name="center_post",
    )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=satin_metal,
        name="top_cap",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.090, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=satin_metal,
        name="swivel_hub",
    )
    seat.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=powder_black,
        name="seat_pan",
    )
    seat.visual(
        Cylinder(radius=0.180, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0555)),
        material=vinyl,
        name="cushion_lower",
    )
    seat.visual(
        Cylinder(radius=0.165, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=vinyl,
        name="cushion_upper",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
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
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.check(
        "seat uses a continuous vertical swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.expect_origin_gap(
        seat,
        pedestal,
        axis="z",
        min_gap=0.68,
        max_gap=0.72,
        name="seat is at bar-stool height above the pedestal root",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="swivel_hub",
            negative_elem="top_cap",
            max_gap=0.002,
            max_penetration=0.0,
            name="swivel hub seats on the pedestal cap without penetration",
        )
        ctx.expect_overlap(
            seat,
            pedestal,
            axes="xy",
            elem_a="swivel_hub",
            elem_b="top_cap",
            min_overlap=0.11,
            name="seat hub stays centered over the pedestal cap",
        )
        ctx.expect_overlap(
            seat,
            pedestal,
            axes="xy",
            elem_a="cushion_lower",
            elem_b="base_plate",
            min_overlap=0.34,
            name="seat remains centered over the round base footprint",
        )

    return ctx.report()


object_model = build_object_model()
