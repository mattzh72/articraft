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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.79, 0.80, 0.82, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.16, 0.16, 0.17, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    base = model.part("pedestal_base")
    base.inertial = Inertial.from_geometry(
        Box((0.70, 0.70, 0.74)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    leg_mesh = save_mesh(
        "base_leg",
        tube_from_spline_points(
            [
                (0.054, 0.0, 0.050),
                (0.140, 0.0, 0.046),
                (0.235, 0.0, 0.031),
                (0.312, 0.0, 0.017),
            ],
            radius=0.018,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    foot_ring_mesh = save_mesh(
        "foot_ring",
        TorusGeometry(radius=0.185, tube=0.012, radial_segments=18, tubular_segments=72),
    )

    base.visual(
        Cylinder(radius=0.078, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=powder_black,
        name="hub_shell",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=powder_black,
        name="post_socket",
    )

    for idx, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        base.visual(
            leg_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=powder_black,
            name=f"base_leg_{idx}",
        )
        base.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(0.312, 0.0, 0.006), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"foot_pad_{idx}",
        )

    base.visual(
        Cylinder(radius=0.023, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=satin_steel,
        name="seat_post",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=powder_black,
        name="foot_ring_collar",
    )
    for idx, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        base.visual(
            Cylinder(radius=0.008, length=0.136),
            origin=Origin(xyz=(0.117, 0.0, 0.275), rpy=(0.0, pi / 2.0, angle)),
            material=polished_steel,
            name=f"foot_ring_spoke_{idx}",
        )
    base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=polished_steel,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=powder_black,
        name="bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.716)),
        material=polished_steel,
        name="bearing_cap",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.095),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    seat_cushion_mesh = save_mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.038),
                (0.055, 0.038),
                (0.110, 0.041),
                (0.156, 0.049),
                (0.176, 0.062),
                (0.170, 0.078),
                (0.128, 0.086),
                (0.052, 0.089),
                (0.0, 0.087),
                (0.0, 0.038),
            ],
            segments=72,
        ),
    )

    seat.visual(
        Cylinder(radius=0.098, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=polished_steel,
        name="rotary_plate",
    )
    seat.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=powder_black,
        name="turntable_drum",
    )
    seat.visual(
        Cylinder(radius=0.148, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=powder_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        material=vinyl_black,
        name="seat_cushion",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.721)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_base")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="rotary_plate",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="rotary plate sits directly over the exposed bearing cap",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="rotary_plate",
        elem_b="bearing_housing",
        min_overlap=0.10,
        name="seat remains centered over the pedestal axis",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="seat_pan",
        negative_elem="foot_ring",
        min_gap=0.40,
        name="foot ring stays well below the seat pan",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({swivel: pi / 2.0}):
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="rotary_plate",
            negative_elem="bearing_cap",
            min_gap=0.0,
            max_gap=0.001,
            name="swiveled seat keeps the same bearing clearance",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            elem_a="rotary_plate",
            elem_b="bearing_housing",
            min_overlap=0.10,
            name="swiveled seat stays on the pedestal axis",
        )
        rotated_pos = ctx.part_world_position(seat)

    centered = (
        rest_pos is not None
        and rotated_pos is not None
        and abs(rest_pos[0] - rotated_pos[0]) <= 1e-6
        and abs(rest_pos[1] - rotated_pos[1]) <= 1e-6
        and abs(rest_pos[2] - rotated_pos[2]) <= 1e-6
    )
    ctx.check(
        "seat origin does not drift while swiveling",
        centered,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
