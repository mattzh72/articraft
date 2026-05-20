from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


def _seat_cushion_geometry() -> LatheGeometry:
    return LatheGeometry(
        [
            (0.0, 0.014),
            (0.050, 0.008),
            (0.145, 0.006),
            (0.176, 0.015),
            (0.185, 0.034),
            (0.171, 0.060),
            (0.108, 0.078),
            (0.0, 0.074),
        ],
        segments=72,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    dark_shell = model.material("dark_shell", rgba=(0.20, 0.20, 0.22, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.08, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.230, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.205, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=polished_steel,
        name="base_cap",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.362)),
        material=polished_steel,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=brushed_steel,
        name="top_cap",
    )
    pedestal.visual(
        Cylinder(radius=0.043, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=brushed_steel,
        name="ring_collar",
    )
    pedestal.visual(
        _mesh("foot_ring", TorusGeometry(radius=0.165, tube=0.012, radial_segments=20, tubular_segments=56)),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=brushed_steel,
        name="foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.138),
        origin=Origin(xyz=(0.103, 0.0, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="ring_spoke_x_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.138),
        origin=Origin(xyz=(-0.103, 0.0, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="ring_spoke_x_neg",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.138),
        origin=Origin(xyz=(0.0, 0.103, 0.315), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="ring_spoke_y_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.138),
        origin=Origin(xyz=(0.0, -0.103, 0.315), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="ring_spoke_y_neg",
    )

    seat = model.part("seat")
    seat.visual(
        _mesh("seat_cushion", _seat_cushion_geometry()),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Cylinder(radius=0.155, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_shell,
        name="seat_shell",
    )
    seat.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_steel,
        name="seat_bearing",
    )
    seat.visual(
        Box((0.140, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, 0.177, 0.020)),
        material=dark_shell,
        name="hinge_mount",
    )
    seat.visual(
        Box((0.094, 0.032, 0.034)),
        origin=Origin(xyz=(0.0, 0.169, 0.038)),
        material=dark_shell,
        name="hinge_gusset",
    )
    seat.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(-0.034, 0.204, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_knuckle_0",
    )
    seat.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.034, 0.204, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_knuckle_1",
    )
    seat.visual(
        Box((0.032, 0.030, 0.030)),
        origin=Origin(xyz=(-0.034, 0.191, 0.044)),
        material=dark_shell,
        name="hinge_cheek_0",
    )
    seat.visual(
        Box((0.032, 0.030, 0.030)),
        origin=Origin(xyz=(0.034, 0.191, 0.044)),
        material=dark_shell,
        name="hinge_cheek_1",
    )

    arm_support = model.part("arm_support")
    arm_support.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    arm_support.visual(
        Box((0.028, 0.080, 0.090)),
        origin=Origin(xyz=(0.0, 0.040, 0.045)),
        material=dark_shell,
        name="arm_spine",
    )
    arm_support.visual(
        Box((0.170, 0.070, 0.028)),
        origin=Origin(xyz=(0.0, 0.102, 0.088)),
        material=dark_shell,
        name="arm_bridge",
    )
    arm_support.visual(
        Cylinder(radius=0.013, length=0.154),
        origin=Origin(xyz=(-0.078, 0.145, 0.090)),
        material=brushed_steel,
        name="arm_post_0",
    )
    arm_support.visual(
        Cylinder(radius=0.013, length=0.154),
        origin=Origin(xyz=(0.078, 0.145, 0.090)),
        material=brushed_steel,
        name="arm_post_1",
    )
    arm_support.visual(
        Cylinder(radius=0.020, length=0.230),
        origin=Origin(xyz=(0.0, 0.155, 0.125), rpy=(0.0, pi / 2.0, 0.0)),
        material=matte_black,
        name="arm_pad",
    )

    seat_spin = model.articulation(
        "pedestal_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.712)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    arm_fold = model.articulation(
        "seat_to_arm_support",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=arm_support,
        origin=Origin(xyz=(0.0, 0.204, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.15),
    )

    pedestal.meta["primary_joint"] = seat_spin.name
    arm_support.meta["fold_joint"] = arm_fold.name

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    arm_support = object_model.get_part("arm_support")
    seat_spin = object_model.get_articulation("pedestal_to_seat")
    arm_fold = object_model.get_articulation("seat_to_arm_support")

    with ctx.pose({seat_spin: 0.0, arm_fold: 0.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="seat_bearing",
            negative_elem="top_cap",
            max_gap=0.0015,
            max_penetration=0.0,
            name="seat bearing seats on pedestal top cap",
        )
        ctx.expect_overlap(
            seat,
            pedestal,
            axes="xy",
            elem_a="seat_bearing",
            elem_b="top_cap",
            min_overlap=0.100,
            name="seat bearing remains centered over pedestal",
        )
        ctx.expect_origin_gap(
            arm_support,
            seat,
            axis="y",
            min_gap=0.15,
            max_gap=0.21,
            name="arm support hinge sits at one side of the seat",
        )

        rest_pad_aabb = ctx.part_element_world_aabb(arm_support, elem="arm_pad")

    with ctx.pose({seat_spin: pi / 2.0, arm_fold: 0.0}):
        spun_arm_position = ctx.part_world_position(arm_support)

    with ctx.pose({seat_spin: 0.0, arm_fold: 1.15}):
        raised_pad_aabb = ctx.part_element_world_aabb(arm_support, elem="arm_pad")

    ctx.check(
        "seat spin carries side arm around pedestal axis",
        spun_arm_position is not None
        and abs(spun_arm_position[0]) > 0.15
        and abs(spun_arm_position[1]) < 0.04,
        details=f"spun_arm_position={spun_arm_position}",
    )
    ctx.check(
        "arm support folds upward",
        rest_pad_aabb is not None
        and raised_pad_aabb is not None
        and raised_pad_aabb[1][2] > rest_pad_aabb[1][2] + 0.06,
        details=f"rest_pad_aabb={rest_pad_aabb}, raised_pad_aabb={raised_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
