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

    base_black = model.material("base_black", rgba=(0.13, 0.13, 0.14, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.23, 0.20, 0.17, 1.0))

    pedestal_shell = _mesh(
        "pedestal_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.110, 0.0),
                (0.106, 0.010),
                (0.094, 0.045),
                (0.078, 0.180),
                (0.067, 0.370),
                (0.060, 0.520),
                (0.056, 0.560),
                (0.0, 0.560),
            ],
            segments=56,
        ),
    )
    foot_ring_mesh = _mesh(
        "foot_ring",
        TorusGeometry(radius=0.176, tube=0.013, radial_segments=18, tubular_segments=60),
    )
    guard_hoop_mesh = _mesh(
        "guard_hoop",
        TorusGeometry(radius=0.108, tube=0.012, radial_segments=16, tubular_segments=48),
    )
    seat_cushion_mesh = _mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.085, 0.0),
                (0.155, 0.006),
                (0.182, 0.016),
                (0.190, 0.038),
                (0.184, 0.056),
                (0.146, 0.066),
                (0.0, 0.064),
            ],
            segments=64,
        ),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.240, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=base_black,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.138, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=base_black,
        name="base_skirt",
    )
    base.visual(
        pedestal_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=polished_steel,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=satin_steel,
        name="center_post",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=base_black,
        name="upper_housing",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=satin_steel,
        name="bearing_race",
    )
    base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=polished_steel,
        name="foot_ring",
    )
    for sign in (-1.0, 1.0):
        base.visual(
            Box((0.136, 0.016, 0.016)),
            origin=Origin(xyz=(sign * 0.104, 0.0, 0.330)),
            material=polished_steel,
            name=f"foot_spoke_x_{'pos' if sign > 0 else 'neg'}",
        )
        base.visual(
            Box((0.016, 0.136, 0.016)),
            origin=Origin(xyz=(0.0, sign * 0.104, 0.330)),
            material=polished_steel,
            name=f"foot_spoke_y_{'pos' if sign > 0 else 'neg'}",
        )
        base.visual(
            Box((0.050, 0.012, 0.012)),
            origin=Origin(xyz=(sign * 0.082, 0.0, 0.630)),
            material=base_black,
            name=f"guard_spoke_x_{'pos' if sign > 0 else 'neg'}",
        )
        base.visual(
            Box((0.012, 0.050, 0.012)),
            origin=Origin(xyz=(0.0, sign * 0.082, 0.630)),
            material=base_black,
            name=f"guard_spoke_y_{'pos' if sign > 0 else 'neg'}",
        )
    base.visual(
        guard_hoop_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        material=base_black,
        name="guard_hoop",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.240, length=0.640),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_steel,
        name="seat_bearing_collar",
    )
    seat.visual(
        Cylinder(radius=0.112, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=base_black,
        name="seat_support_plate",
    )
    seat.visual(
        Cylinder(radius=0.182, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=base_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.390, 0.390, 0.150)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.648)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="seat_bearing_collar",
        negative_elem="guard_hoop",
        min_gap=0.001,
        max_gap=0.020,
        name="seat bearing clears fixed guard hoop",
    )
    ctx.expect_contact(
        seat,
        base,
        elem_a="seat_bearing_collar",
        elem_b="bearing_race",
        name="seat bearing rests on the fixed bearing race",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="seat_pan",
        elem_b="base_disk",
        min_overlap=0.300,
        name="seat remains centered over the pedestal base",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(seat)
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="seat_bearing_collar",
            negative_elem="guard_hoop",
            min_gap=0.001,
            max_gap=0.020,
            name="seat bearing still clears guard after swivel",
        )
        ctx.expect_contact(
            seat,
            base,
            elem_a="seat_bearing_collar",
            elem_b="bearing_race",
            name="seat bearing remains seated on the race after swivel",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            elem_a="seat_pan",
            elem_b="base_disk",
            min_overlap=0.300,
            name="seat stays over base after swivel",
        )
    ctx.check(
        "swivel keeps seat centered on vertical axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
