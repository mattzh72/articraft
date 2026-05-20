from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.89, 1.0))
    vinyl = model.material("vinyl", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.17, tube=0.012, radial_segments=18, tubular_segments=64),
        "foot_ring",
    )

    seat_cushion_geom = CylinderGeometry(
        radius=0.175,
        height=0.030,
        radial_segments=48,
        closed=True,
    ).translate(0.0, 0.0, 0.015)
    seat_cushion_geom.merge(
        DomeGeometry(
            radius=0.175,
            radial_segments=48,
            height_segments=14,
            closed=True,
        ).scale(1.0, 1.0, 0.18).translate(0.0, 0.0, 0.026)
    )
    seat_cushion_mesh = mesh_from_geometry(seat_cushion_geom, "seat_cushion")

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.22, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.19, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber,
        name="base_trim_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=chrome,
        name="center_post",
    )
    pedestal.visual(
        Cylinder(radius=0.048, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=brushed_steel,
        name="foot_ring_collar",
    )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=brushed_steel,
        name="foot_ring",
    )
    pedestal.visual(
        Box((0.120, 0.016, 0.014)),
        origin=Origin(xyz=(0.102, 0.0, 0.27)),
        material=brushed_steel,
        name="foot_ring_spoke_pos_x",
    )
    pedestal.visual(
        Box((0.120, 0.016, 0.014)),
        origin=Origin(xyz=(-0.102, 0.0, 0.27)),
        material=brushed_steel,
        name="foot_ring_spoke_neg_x",
    )
    pedestal.visual(
        Box((0.016, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, 0.102, 0.27)),
        material=brushed_steel,
        name="foot_ring_spoke_pos_y",
    )
    pedestal.visual(
        Box((0.016, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, -0.102, 0.27)),
        material=brushed_steel,
        name="foot_ring_spoke_neg_y",
    )
    pedestal.visual(
        Cylinder(radius=0.06, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.6475)),
        material=brushed_steel,
        name="bearing_housing",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.665),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.3325)),
    )

    seat = model.part("seat_assembly")
    seat.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brushed_steel,
        name="swivel_stage",
    )
    seat.visual(
        Cylinder(radius=0.024, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=chrome,
        name="swivel_spindle",
    )
    seat.visual(
        Cylinder(radius=0.10, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=satin_black,
        name="underseat_mount",
    )
    seat.visual(
        Cylinder(radius=0.155, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=satin_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.135),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat_assembly")
    swivel = object_model.get_articulation("seat_swivel")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_contact(
            seat,
            pedestal,
            elem_a="swivel_stage",
            elem_b="bearing_housing",
            name="swivel stage sits on the central bearing housing",
        )
        ctx.expect_origin_distance(
            seat,
            pedestal,
            axes="xy",
            max_dist=1e-6,
            name="seat stays centered over the pedestal axis",
        )

    with ctx.pose({swivel: 1.8}):
        ctx.expect_origin_distance(
            seat,
            pedestal,
            axes="xy",
            max_dist=1e-6,
            name="seat remains centered while rotated",
        )

    seat_cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    foot_ring_aabb = ctx.part_element_world_aabb(pedestal, elem="foot_ring")
    seat_top = seat_cushion_aabb[1][2] if seat_cushion_aabb is not None else None
    foot_ring_top = foot_ring_aabb[1][2] if foot_ring_aabb is not None else None

    ctx.check(
        "seat height reads as bar stool scale",
        seat_top is not None and 0.76 <= seat_top <= 0.82,
        details=f"seat_top={seat_top}",
    )
    ctx.check(
        "foot ring sits below the seat at usable height",
        seat_top is not None
        and foot_ring_top is not None
        and 0.24 <= foot_ring_top <= 0.30
        and foot_ring_top < seat_top - 0.45,
        details=f"seat_top={seat_top}, foot_ring_top={foot_ring_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
