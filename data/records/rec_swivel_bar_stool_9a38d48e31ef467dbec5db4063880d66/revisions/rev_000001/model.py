from __future__ import annotations

from math import pi, cos, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(geometry, name: str):
    """Convert a procedural geometry object into a managed visual mesh."""
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.035, 0.038, 0.040, 1.0))
    worn_black = model.material("worn_black", rgba=(0.070, 0.075, 0.075, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.24, 0.24, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.68, 0.69, 0.68, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.80, 0.78, 0.70, 1.0))
    molded_seat = model.material("molded_seat", rgba=(0.13, 0.14, 0.13, 1.0))
    edge_wear = model.material("edge_wear", rgba=(0.36, 0.37, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.012, 0.012, 0.012, 1.0))

    pedestal = model.part("pedestal")

    base_profile = [
        (0.000, 0.000),
        (0.305, 0.000),
        (0.335, 0.012),
        (0.325, 0.046),
        (0.268, 0.064),
        (0.135, 0.070),
        (0.000, 0.070),
    ]
    pedestal.visual(
        _mesh(LatheGeometry(base_profile, segments=96), "dished_floor_base"),
        material=powder_black,
        name="dished_floor_base",
    )
    pedestal.visual(
        _mesh(TorusGeometry(0.315, 0.012, radial_segments=18, tubular_segments=96), "rubber_floor_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=rubber,
        name="rubber_floor_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.118, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=gunmetal,
        name="base_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.068, length=0.552),
        origin=Origin(xyz=(0.0, 0.0, 0.346)),
        material=gunmetal,
        name="center_column",
    )
    pedestal.visual(
        Cylinder(radius=0.090, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.212)),
        material=powder_black,
        name="lower_sleeve",
    )
    pedestal.visual(
        Cylinder(radius=0.100, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.602)),
        material=powder_black,
        name="top_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.108, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=gunmetal,
        name="bearing_shoulder",
    )
    pedestal.visual(
        Cylinder(radius=0.250, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=powder_black,
        name="stationary_flange",
    )
    pedestal.visual(
        _mesh(
            LatheGeometry(
                [(0.085, 0.638), (0.190, 0.638), (0.190, 0.660), (0.085, 0.660)],
                segments=96,
            ),
            "lower_race",
        ),
        material=bearing_steel,
        name="lower_race",
    )
    pedestal.visual(
        Cylinder(radius=0.072, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.651)),
        material=edge_wear,
        name="kingpin_nose",
    )

    # Heavy welded base gussets and the serviceable foot rail make the
    # pedestal read as utility-grade rather than a light decorative stool.
    for angle in (0.0, pi / 2.0, pi, 3.0 * pi / 2.0):
        x = cos(angle) * 0.105
        y = sin(angle) * 0.105
        pedestal.visual(
            Box((0.150, 0.018, 0.150)),
            origin=Origin(xyz=(x, y, 0.132), rpy=(0.0, 0.0, angle)),
            material=powder_black,
            name=f"base_gusset_{round(angle, 3)}",
        )

    pedestal.visual(
        _mesh(TorusGeometry(0.335, 0.021, radial_segments=18, tubular_segments=112), "foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=worn_black,
        name="foot_ring",
    )
    for angle in (0.0, pi / 2.0, pi, 3.0 * pi / 2.0):
        horizontal_rpy = (0.0, pi / 2.0, 0.0) if abs(sin(angle)) < 0.5 else (pi / 2.0, 0.0, 0.0)
        pedestal.visual(
            Cylinder(radius=0.018, length=0.282),
            origin=Origin(
                xyz=(cos(angle) * 0.205, sin(angle) * 0.205, 0.330),
                rpy=horizontal_rpy,
            ),
            material=powder_black,
            name=f"foot_ring_spoke_{round(angle, 3)}",
        )
        pedestal.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(xyz=(cos(angle) * 0.335, sin(angle) * 0.335, 0.330), rpy=(0.0, pi / 2.0, 0.0)),
            material=edge_wear,
            name=f"weld_pad_{round(angle, 3)}",
        )

    for i in range(8):
        angle = 2.0 * pi * i / 8.0
        pedestal.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(cos(angle) * 0.245, sin(angle) * 0.245, 0.063)),
            material=fastener_zinc,
            name=f"base_bolt_{i}",
        )

    for i in range(6):
        angle = 2.0 * pi * i / 6.0 + pi / 6.0
        pedestal.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(cos(angle) * 0.222, sin(angle) * 0.222, 0.636)),
            material=fastener_zinc,
            name=f"flange_bolt_{i}",
        )

    seat = model.part("seat")
    seat.visual(
        _mesh(
            LatheGeometry(
                [(0.086, 0.000), (0.202, 0.000), (0.202, 0.024), (0.086, 0.024)],
                segments=96,
            ),
            "upper_race",
        ),
        material=bearing_steel,
        name="upper_race",
    )
    seat.visual(
        _mesh(TorusGeometry(0.202, 0.004, radial_segments=10, tubular_segments=96), "bearing_shadow_line"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=worn_black,
        name="bearing_shadow_line",
    )
    seat.visual(
        Cylinder(radius=0.108, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=gunmetal,
        name="swivel_hub",
    )
    seat.visual(
        Cylinder(radius=0.225, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=powder_black,
        name="seat_support_plate",
    )

    for angle in (0.0, pi / 2.0, pi, 3.0 * pi / 2.0):
        seat.visual(
            Box((0.180, 0.022, 0.060)),
            origin=Origin(xyz=(cos(angle) * 0.095, sin(angle) * 0.095, 0.083), rpy=(0.0, 0.0, angle)),
            material=gunmetal,
            name=f"seat_gusset_{round(angle, 3)}",
        )

    seat_pan_profile = [
        (0.000, 0.073),
        (0.180, 0.073),
        (0.229, 0.083),
        (0.238, 0.101),
        (0.220, 0.113),
        (0.045, 0.109),
        (0.000, 0.105),
    ]
    seat.visual(
        _mesh(LatheGeometry(seat_pan_profile, segments=96), "pressed_steel_pan"),
        material=powder_black,
        name="pressed_steel_pan",
    )
    cushion_profile = [
        (0.000, 0.106),
        (0.185, 0.106),
        (0.238, 0.116),
        (0.252, 0.136),
        (0.240, 0.160),
        (0.195, 0.176),
        (0.000, 0.178),
    ]
    seat.visual(
        _mesh(LatheGeometry(cushion_profile, segments=112), "molded_round_seat"),
        material=molded_seat,
        name="molded_round_seat",
    )
    seat.visual(
        _mesh(TorusGeometry(0.247, 0.006, radial_segments=12, tubular_segments=96), "seat_edge_wear_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=edge_wear,
        name="seat_edge_wear_band",
    )

    for i in range(8):
        angle = 2.0 * pi * i / 8.0
        seat.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=Origin(xyz=(cos(angle) * 0.185, sin(angle) * 0.185, 0.072)),
            material=fastener_zinc,
            name=f"support_bolt_{i}",
        )
    for i in range(6):
        angle = 2.0 * pi * i / 6.0 + pi / 6.0
        seat.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(cos(angle) * 0.162, sin(angle) * 0.162, 0.027)),
            material=fastener_zinc,
            name=f"race_cap_screw_{i}",
        )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_contact(
        seat,
        pedestal,
        elem_a="upper_race",
        elem_b="lower_race",
        contact_tol=0.0005,
        name="swivel races are visibly seated",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.35,
        name="upper and lower race share a full circular footprint",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="upper_race",
        negative_elem="lower_race",
        name="swivel stack has no vertical float",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(seat)
        ctx.expect_contact(
            seat,
            pedestal,
            elem_a="upper_race",
            elem_b="lower_race",
            contact_tol=0.0005,
            name="swivel remains seated after quarter turn",
        )
    ctx.check(
        "swivel rotates about the central vertical column",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 0.0005
        and abs(rest_pos[1] - turned_pos[1]) < 0.0005
        and abs(rest_pos[2] - turned_pos[2]) < 0.0005,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
