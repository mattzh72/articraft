from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_swivel_bar_stool")

    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    molded_black = model.material("molded_black", rgba=(0.055, 0.058, 0.062, 1.0))
    textured_vinyl = model.material("textured_vinyl", rgba=(0.095, 0.095, 0.10, 1.0))
    zinc = model.material("zinc_plated", rgba=(0.78, 0.77, 0.70, 1.0))

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.75),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    # One stamped conical steel base with the return lip revolved as a single
    # cheap-to-tool profile; no separate feet are needed.
    base_dish = LatheGeometry(
        [
            (0.050, 0.018),
            (0.275, 0.018),
            (0.335, 0.028),
            (0.320, 0.053),
            (0.140, 0.096),
            (0.058, 0.096),
            (0.050, 0.018),
        ],
        segments=96,
    )
    pedestal.visual(
        mesh_from_geometry(base_dish, "stamped_base_dish"),
        material=satin_steel,
        name="stamped_base_dish",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.305, tube=0.009, radial_segments=18, tubular_segments=96), "rubber_floor_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=black_rubber,
        name="rubber_floor_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=satin_steel,
        name="base_weld_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.386)),
        material=satin_steel,
        name="central_tube",
    )

    # Four base bolt heads are represented as simple cold-headed fasteners,
    # seated into the stamped dish rather than left as unattached decoration.
    for i in range(4):
        angle = i * pi / 2.0 + pi / 4.0
        pedestal.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.250 * cos(angle), 0.250 * sin(angle), 0.024)),
            material=zinc,
            name=f"base_bolt_{i}",
        )

    # A single bent hoop footrest and four short radial weld tubes keep the
    # component count low while giving the user's foot a real supported ring.
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.240, tube=0.014, radial_segments=18, tubular_segments=112), "footrest_hoop"),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=satin_steel,
        name="footrest_hoop",
    )
    for i, (x, y, rpy) in enumerate(
        [
            (0.152, 0.0, (0.0, pi / 2.0, 0.0)),
            (-0.152, 0.0, (0.0, pi / 2.0, 0.0)),
            (0.0, 0.152, (-pi / 2.0, 0.0, 0.0)),
            (0.0, -0.152, (-pi / 2.0, 0.0, 0.0)),
        ]
    ):
        pedestal.visual(
            Cylinder(radius=0.011, length=0.202),
            origin=Origin(xyz=(x, y, 0.430), rpy=rpy),
            material=satin_steel,
            name=f"footrest_spoke_{i}",
        )
    pedestal.visual(
        Cylinder(radius=0.064, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=satin_steel,
        name="footrest_weld_collar",
    )

    # Low-cost split clamp and stacked bearing support at the top of the post.
    pedestal.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.696)),
        material=dark_steel,
        name="split_clamp_collar",
    )
    for y in (-0.042, 0.042):
        pedestal.visual(
            Box((0.044, 0.020, 0.038)),
            origin=Origin(xyz=(0.072, y, 0.699)),
            material=dark_steel,
            name=f"clamp_ear_{0 if y < 0 else 1}",
        )
    pedestal.visual(
        Cylinder(radius=0.007, length=0.110),
        origin=Origin(xyz=(0.086, 0.0, 0.699), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="clamp_bolt",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=dark_steel,
        name="bearing_cup",
    )
    pedestal.visual(
        Cylinder(radius=0.120, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.744)),
        material=zinc,
        name="lower_race",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.45, 0.46, 0.14)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    seat_shell = superellipse_side_loft(
        [
            (-0.235, 0.063, 0.112, 0.330),
            (-0.110, 0.050, 0.089, 0.430),
            (0.075, 0.050, 0.088, 0.455),
            (0.235, 0.066, 0.118, 0.365),
        ],
        exponents=2.7,
        segments=64,
    )
    seat.visual(
        mesh_from_geometry(seat_shell, "molded_seat_shell"),
        material=molded_black,
        name="molded_seat_shell",
    )
    pad = superellipse_side_loft(
        [
            (-0.205, 0.094, 0.121, 0.285),
            (-0.085, 0.087, 0.112, 0.372),
            (0.080, 0.087, 0.112, 0.390),
            (0.205, 0.096, 0.124, 0.320),
        ],
        exponents=2.4,
        segments=56,
    )
    seat.visual(
        mesh_from_geometry(pad, "textured_seat_pad"),
        material=textured_vinyl,
        name="textured_seat_pad",
    )
    seat.visual(
        Cylinder(radius=0.105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=zinc,
        name="upper_race",
    )
    seat.visual(
        Cylinder(radius=0.158, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_steel,
        name="turntable_plate",
    )
    seat.visual(
        Cylinder(radius=0.078, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_steel,
        name="central_boss",
    )
    seat.visual(
        Box((0.355, 0.036, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_steel,
        name="cross_rib_x",
    )
    seat.visual(
        Box((0.036, 0.355, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_steel,
        name="cross_rib_y",
    )
    for i in range(4):
        angle = i * pi / 2.0 + pi / 4.0
        seat.visual(
            Cylinder(radius=0.010, length=0.042),
            origin=Origin(xyz=(0.110 * cos(angle), 0.110 * sin(angle), 0.043)),
            material=zinc,
            name=f"seat_bolt_{i}",
        )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.750)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.015),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.check(
        "one centered continuous seat swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(swivel.origin.xyz) == (0.0, 0.0, 0.750),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}, origin={swivel.origin}",
    )
    ctx.expect_within(
        seat,
        pedestal,
        axes="xy",
        inner_elem="upper_race",
        outer_elem="lower_race",
        margin=0.001,
        name="upper race is centered inside lower race",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.20,
        name="bearing races have broad supported overlap",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="upper_race",
        negative_elem="lower_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper race sits on lower race without penetration",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_origin = ctx.part_world_position(seat)
    rest_bolt = _center_from_aabb(ctx.part_element_world_aabb(seat, elem="seat_bolt_0"))
    with ctx.pose({swivel: pi / 2.0}):
        spun_origin = ctx.part_world_position(seat)
        spun_bolt = _center_from_aabb(ctx.part_element_world_aabb(seat, elem="seat_bolt_0"))

    ctx.check(
        "swivel keeps seat centered on post",
        rest_origin is not None
        and spun_origin is not None
        and abs(rest_origin[0] - spun_origin[0]) < 1e-6
        and abs(rest_origin[1] - spun_origin[1]) < 1e-6
        and abs(rest_origin[2] - spun_origin[2]) < 1e-6,
        details=f"rest={rest_origin}, spun={spun_origin}",
    )
    ctx.check(
        "visible bolt pattern rotates with seat stage",
        rest_bolt is not None
        and spun_bolt is not None
        and abs(rest_bolt[0] - spun_bolt[0]) > 0.10
        and abs(rest_bolt[1] - spun_bolt[1]) < 0.002,
        details=f"rest_bolt={rest_bolt}, spun_bolt={spun_bolt}",
    )

    return ctx.report()


object_model = build_object_model()
