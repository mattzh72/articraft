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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    matte_black = model.material("matte_black", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.26, 0.28, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.22, 0.14, 0.10, 1.0))

    seat_pan_mesh = _mesh(
        "seat_pan",
        ExtrudeGeometry(
            superellipse_profile(0.42, 0.39, exponent=2.35, segments=56),
            0.016,
            center=True,
        ),
    )
    seat_cushion_mesh = _mesh(
        "seat_cushion",
        ExtrudeGeometry(
            superellipse_profile(0.40, 0.37, exponent=2.0, segments=56),
            0.050,
            center=True,
        ),
    )
    foot_ring_mesh = _mesh(
        "foot_ring",
        TorusGeometry(radius=0.17, tube=0.012, radial_segments=18, tubular_segments=56),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.22, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=matte_black,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.10, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_steel,
        name="base_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.262)),
        material=satin_steel,
        name="seat_post",
    )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=satin_steel,
        name="foot_ring",
    )
    for index, (xyz, rpy) in enumerate(
        [
            ((0.102, 0.0, 0.325), (0.0, 0.0, 0.0)),
            ((-0.102, 0.0, 0.325), (0.0, 0.0, 0.0)),
            ((0.0, 0.102, 0.325), (0.0, 0.0, math.pi / 2.0)),
            ((0.0, -0.102, 0.325), (0.0, 0.0, math.pi / 2.0)),
        ]
    ):
        pedestal.visual(
            Box((0.118, 0.024, 0.016)),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=dark_steel,
            name=f"foot_ring_spoke_{index}",
        )
    pedestal.visual(
        Cylinder(radius=0.068, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=dark_steel,
        name="post_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=matte_black,
        name="bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.125, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.634)),
        material=dark_steel,
        name="bearing_top_flange",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.641),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.3205)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.112, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_steel,
        name="swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.082, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=matte_black,
        name="upper_bearing_hub",
    )
    seat.visual(
        Box((0.280, 0.058, 0.115)),
        origin=Origin(xyz=(0.0, 0.139, 0.0575)),
        material=dark_steel,
        name="left_support",
    )
    seat.visual(
        Box((0.280, 0.058, 0.115)),
        origin=Origin(xyz=(0.0, -0.139, 0.0575)),
        material=dark_steel,
        name="right_support",
    )
    seat.visual(
        Box((0.082, 0.286, 0.030)),
        origin=Origin(xyz=(0.092, 0.0, 0.100)),
        material=dark_steel,
        name="front_crossmember",
    )
    seat.visual(
        Box((0.082, 0.286, 0.030)),
        origin=Origin(xyz=(-0.092, 0.0, 0.100)),
        material=dark_steel,
        name="rear_crossmember",
    )
    seat.visual(
        seat_pan_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=matte_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.42, 0.39, 0.178)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.641)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_plate",
        negative_elem="bearing_top_flange",
        min_gap=0.0,
        max_gap=0.001,
        name="swivel plate seats on the fixed top flange",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="swivel_plate",
        elem_b="bearing_housing",
        min_overlap=0.18,
        name="swivel plate stays centered over the bearing housing",
    )

    cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    cushion_top = None if cushion_aabb is None else cushion_aabb[1][2]
    ctx.check(
        "seat height reads as a bar stool",
        cushion_top is not None and 0.76 <= cushion_top <= 0.84,
        details=f"seat cushion top z={cushion_top}",
    )

    rest_support_center = _aabb_center(ctx.part_element_world_aabb(seat, elem="left_support"))
    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="swivel_plate",
            negative_elem="bearing_top_flange",
            min_gap=0.0,
            max_gap=0.001,
            name="swivel stage stays seated after a quarter turn",
        )
        turned_support_center = _aabb_center(ctx.part_element_world_aabb(seat, elem="left_support"))

    ctx.check(
        "seat support rotates around the vertical pedestal axis",
        rest_support_center is not None
        and turned_support_center is not None
        and abs(rest_support_center[0]) < 0.03
        and rest_support_center[1] > 0.10
        and turned_support_center[0] < -0.10
        and abs(turned_support_center[1]) < 0.03,
        details=f"rest={rest_support_center}, turned={turned_support_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
