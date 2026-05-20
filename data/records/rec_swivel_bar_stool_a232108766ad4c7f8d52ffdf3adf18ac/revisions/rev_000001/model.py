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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def saved_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    base_cover = saved_mesh(
        "pedestal_base_cover",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.195, 0.0),
                (0.215, 0.004),
                (0.222, 0.012),
                (0.222, 0.028),
                (0.090, 0.030),
                (0.0, 0.030),
                (0.0, 0.0),
            ],
            segments=56,
        ),
    )
    foot_ring = saved_mesh(
        "foot_ring",
        TorusGeometry(radius=0.175, tube=0.013, radial_segments=18, tubular_segments=64),
    )
    seat_cushion = saved_mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.105, 0.0),
                (0.155, 0.006),
                (0.182, 0.018),
                (0.190, 0.036),
                (0.182, 0.057),
                (0.148, 0.072),
                (0.080, 0.079),
                (0.0, 0.078),
                (0.0, 0.0),
            ],
            segments=64,
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(base_cover, material=powder_black, name="base_cover")
    pedestal.visual(
        Cylinder(radius=0.085, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_steel,
        name="lower_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.031, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=brushed_steel,
        name="seat_post",
    )
    pedestal.visual(
        Cylinder(radius=0.056, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=dark_steel,
        name="foot_ring_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=dark_steel,
        name="swivel_base_cap",
    )
    pedestal.visual(
        foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=brushed_steel,
        name="foot_ring",
    )

    spoke_length = 0.175 - 0.013 - 0.056
    spoke_center = 0.056 + (spoke_length * 0.5)
    pedestal.visual(
        Cylinder(radius=0.011, length=spoke_length),
        origin=Origin(xyz=(spoke_center, 0.0, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="foot_spoke_pos_x",
    )
    pedestal.visual(
        Cylinder(radius=0.011, length=spoke_length),
        origin=Origin(xyz=(-spoke_center, 0.0, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="foot_spoke_neg_x",
    )
    pedestal.visual(
        Cylinder(radius=0.011, length=spoke_length),
        origin=Origin(xyz=(0.0, spoke_center, 0.290), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="foot_spoke_pos_y",
    )
    pedestal.visual(
        Cylinder(radius=0.011, length=spoke_length),
        origin=Origin(xyz=(0.0, -spoke_center, 0.290), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="foot_spoke_neg_y",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.222, length=0.660),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.074, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="swivel_housing",
    )
    seat.visual(
        Cylinder(radius=0.128, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_steel,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=vinyl_black,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.151),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0755)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
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
        min_gap=0.0,
        max_gap=0.001,
        name="seat assembly seats on pedestal cap",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        min_overlap=0.10,
        name="seat stays centered over pedestal footprint",
    )

    rest_position = ctx.part_world_position(seat)
    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            name="seat remains seated while rotated",
        )
        ctx.expect_overlap(
            seat,
            pedestal,
            axes="xy",
            min_overlap=0.10,
            name="seat remains centered while rotated",
        )
        rotated_position = ctx.part_world_position(seat)

    same_center = (
        rest_position is not None
        and rotated_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, rotated_position)) <= 1e-6
    )
    ctx.check(
        "swivel rotation keeps seat origin fixed in space",
        same_center,
        details=f"rest={rest_position}, rotated={rotated_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
