from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_swivel_bar_stool")

    powder_coat = model.material("powder_coat", rgba=(0.17, 0.18, 0.20, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    upholstery = model.material("upholstery", rgba=(0.39, 0.27, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    footrest_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.105, tube=0.012, radial_segments=18, tubular_segments=64),
        "footrest_ring",
    )
    seat_cushion_geom = DomeGeometry(
        radius=0.155,
        radial_segments=40,
        height_segments=16,
        closed=True,
    )
    seat_cushion_geom.scale(1.0, 1.0, 0.28)
    seat_cushion_mesh = mesh_from_geometry(seat_cushion_geom, "seat_cushion")

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.145, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="floor_glide",
    )
    pedestal.visual(
        Cylinder(radius=0.150, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=powder_coat,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.085, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=powder_coat,
        name="base_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.030, length=0.490),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=powder_coat,
        name="central_column",
    )
    pedestal.visual(
        footrest_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=satin_steel,
        name="footrest_ring",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        pedestal.visual(
            Box((0.070, 0.016, 0.016)),
            origin=Origin(xyz=(0.060, 0.0, 0.245), rpy=(0.0, 0.0, angle)),
            material=powder_coat,
            name=f"footrest_strut_{index}",
        )
    pedestal.visual(
        Cylinder(radius=0.040, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.5175)),
        material=powder_coat,
        name="upper_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.054, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.539)),
        material=satin_steel,
        name="lower_bearing",
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_steel,
        name="upper_bearing",
    )
    seat_stage.visual(
        Cylinder(radius=0.120, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_coat,
        name="seat_pan",
    )
    seat_stage.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=upholstery,
        name="seat_cushion",
    )

    model.articulation(
        "pedestal_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.548)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_stage = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("pedestal_to_seat_swivel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "seat swivel uses continuous articulation",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        f"expected continuous articulation, got {swivel.articulation_type!r}",
    )
    ctx.check(
        "seat swivel axis is vertical",
        swivel.axis == (0.0, 0.0, 1.0),
        f"expected vertical axis, got {swivel.axis!r}",
    )
    ctx.expect_origin_distance(
        seat_stage,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="seat stays concentric with pedestal",
    )
    ctx.expect_contact(
        seat_stage,
        pedestal,
        elem_a="upper_bearing",
        elem_b="lower_bearing",
        name="bearing stack is supported in rest pose",
    )
    ctx.expect_overlap(
        seat_stage,
        pedestal,
        axes="xy",
        min_overlap=0.10,
        elem_a="upper_bearing",
        elem_b="lower_bearing",
        name="bearing stack remains centered",
    )
    ctx.expect_gap(
        seat_stage,
        pedestal,
        axis="z",
        min_gap=0.020,
        max_gap=0.060,
        positive_elem="seat_cushion",
        negative_elem="lower_bearing",
        name="seat cushion clears swivel hardware",
    )

    with ctx.pose({swivel: pi / 2.0}):
        ctx.expect_contact(
            seat_stage,
            pedestal,
            elem_a="upper_bearing",
            elem_b="lower_bearing",
            name="bearing stack stays supported after swivel",
        )
        ctx.expect_origin_distance(
            seat_stage,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="seat remains concentric after swivel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
