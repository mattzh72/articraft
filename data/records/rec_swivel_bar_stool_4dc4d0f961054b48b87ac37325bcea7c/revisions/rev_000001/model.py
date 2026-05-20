from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("satin_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("vinyl_seat", rgba=(0.22, 0.11, 0.08, 1.0))
    model.material("dark_trim", rgba=(0.08, 0.08, 0.09, 1.0))

    base_profile = [
        (0.0, 0.000),
        (0.100, 0.000),
        (0.175, 0.002),
        (0.205, 0.008),
        (0.210, 0.018),
        (0.205, 0.028),
        (0.180, 0.038),
        (0.080, 0.045),
        (0.0, 0.045),
    ]
    seat_profile = [
        (0.0, 0.000),
        (0.090, 0.000),
        (0.145, 0.004),
        (0.170, 0.012),
        (0.180, 0.026),
        (0.178, 0.046),
        (0.162, 0.056),
        (0.095, 0.062),
        (0.0, 0.060),
    ]
    base_dish_mesh = _mesh("stool_base_dish", LatheGeometry(base_profile, segments=72))
    foot_ring_mesh = _mesh(
        "stool_foot_ring",
        TorusGeometry(radius=0.140, tube=0.012, radial_segments=18, tubular_segments=72),
    )
    seat_mesh = _mesh("stool_seat_cushion", LatheGeometry(seat_profile, segments=72))

    pedestal = model.part("pedestal")
    pedestal.visual(
        base_dish_mesh,
        material="powder_black",
        name="base_dish",
    )
    pedestal.visual(
        Cylinder(radius=0.030, length=0.490),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material="satin_steel",
        name="main_post",
    )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material="satin_steel",
        name="foot_ring",
    )
    for axis, center in (
        ("x", (0.079, 0.0, 0.300)),
        ("x", (-0.079, 0.0, 0.300)),
        ("y", (0.0, 0.079, 0.300)),
        ("y", (0.0, -0.079, 0.300)),
    ):
        pedestal.visual(
            Cylinder(radius=0.010, length=0.098),
            origin=Origin(
                xyz=center,
                rpy=(0.0, math.pi / 2.0, 0.0) if axis == "x" else (math.pi / 2.0, 0.0, 0.0),
            ),
            material="satin_steel",
            name=f"foot_ring_strut_{axis}_{'pos' if center[0] + center[1] > 0 else 'neg'}",
        )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
        material="powder_black",
        name="bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        material="satin_steel",
        name="top_stub",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.420, 0.420, 0.640)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material="dark_trim",
        name="swivel_sleeve",
    )
    seat.visual(
        Cylinder(radius=0.095, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material="powder_black",
        name="seat_support_plate",
    )
    seat.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="dark_trim",
        name="seat_pan",
    )
    seat.visual(
        seat_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="vinyl_seat",
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.125),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
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

    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_origin_distance(
        pedestal,
        seat,
        axes="xy",
        max_dist=0.001,
        name="seat axis stays centered on pedestal",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_sleeve",
        negative_elem="top_stub",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel sleeve seats on pedestal stub",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="swivel_sleeve",
        elem_b="top_stub",
        min_overlap=0.040,
        name="swivel sleeve remains coaxial with pedestal stub",
    )

    with ctx.pose({swivel: 1.7}):
        ctx.expect_origin_distance(
            pedestal,
            seat,
            axes="xy",
            max_dist=0.001,
            name="seat stays centered while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
