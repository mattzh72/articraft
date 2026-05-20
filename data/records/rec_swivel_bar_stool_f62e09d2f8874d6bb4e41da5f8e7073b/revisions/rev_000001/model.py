from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.88, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.18, 0.16, 0.14, 1.0))

    foot_ring_mesh = _mesh(
        "foot_ring",
        TorusGeometry(radius=0.165, tube=0.014, radial_segments=18, tubular_segments=56),
    )
    seat_cushion_mesh = _mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.140, 0.0),
                (0.173, 0.006),
                (0.186, 0.020),
                (0.184, 0.039),
                (0.168, 0.054),
                (0.130, 0.061),
                (0.0, 0.062),
                (0.0, 0.0),
            ],
            segments=64,
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.235, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_black,
        name="base_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.190, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=polished_steel,
        name="base_cap",
    )
    pedestal.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=polished_steel,
        name="post_shroud",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.356)),
        material=chrome,
        name="main_post",
    )
    pedestal.visual(
        Cylinder(radius=0.050, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=polished_steel,
        name="foot_ring_collar",
    )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=chrome,
        name="foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.132),
        origin=Origin(xyz=(0.104, 0.0, 0.325), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="foot_ring_spoke_pos_x",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.132),
        origin=Origin(xyz=(-0.104, 0.0, 0.325), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="foot_ring_spoke_neg_x",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.132),
        origin=Origin(xyz=(0.0, 0.104, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="foot_ring_spoke_pos_y",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.132),
        origin=Origin(xyz=(0.0, -0.104, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="foot_ring_spoke_neg_y",
    )
    pedestal.visual(
        Cylinder(radius=0.064, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.651)),
        material=polished_steel,
        name="bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.076, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.671)),
        material=satin_black,
        name="top_thrust_plate",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=0.686),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.343)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.075, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_black,
        name="seat_swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=polished_steel,
        name="seat_hub",
    )
    seat.visual(
        Cylinder(radius=0.145, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=satin_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.188, length=0.082),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    model.articulation(
        "pedestal_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.679)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("pedestal_to_seat_swivel")

    ctx.check(
        "swivel is a continuous vertical joint",
        swivel.articulation_type == ArticulationType.CONTINUOUS and swivel.axis == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.expect_origin_gap(
        seat,
        pedestal,
        axis="z",
        min_gap=0.66,
        max_gap=0.70,
        name="seat stage sits at realistic bar-stool height",
    )
    ctx.expect_origin_distance(
        seat,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="seat stays centered over the pedestal axis",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="seat_swivel_plate",
        negative_elem="top_thrust_plate",
        max_gap=0.005,
        max_penetration=0.0,
        name="rotating stage stays compact above the support plate",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="seat_swivel_plate",
        elem_b="top_thrust_plate",
        min_overlap=0.11,
        name="swivel plate remains well supported in plan",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="seat_swivel_plate",
            negative_elem="top_thrust_plate",
            max_gap=0.005,
            max_penetration=0.0,
            name="support clearance holds after a quarter turn",
        )
        ctx.expect_origin_distance(
            seat,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="seat remains coaxial after a quarter turn",
        )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
