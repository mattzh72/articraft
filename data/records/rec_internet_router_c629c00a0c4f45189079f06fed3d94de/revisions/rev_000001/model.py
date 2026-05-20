from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WIDTH = 0.26
DEPTH = 0.16
BODY_HEIGHT = 0.035
FOOT_HEIGHT = 0.008
BODY_BOTTOM_Z = 0.006
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT

ANTENNA_RANGE_RAD = math.radians(120.0)
ANTENNA_HALF_RANGE_RAD = ANTENNA_RANGE_RAD / 2.0
ANTENNA_MOUNT_XS = (-0.108, -0.034, 0.034, 0.108)
ANTENNA_MOUNT_Y = DEPTH / 2.0 - 0.006
HINGE_Z = BODY_TOP_Z + 0.018


def _rounded_router_body() -> cq.Workplane:
    """Low router housing with softened vertical edges and a slight top radius."""
    return (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.020)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )


def _add_antenna_socket(housing, index: int, x: float, y: float, z_top: float, material) -> None:
    """Fixed clevis-style hinge support on the router housing."""
    housing.visual(
        Box((0.046, 0.026, 0.007)),
        origin=Origin(xyz=(x, y, z_top + 0.0035)),
        material=material,
        name=f"socket_pad_{index}",
    )
    for side, dx in enumerate((-0.014, 0.014)):
        housing.visual(
            Box((0.006, 0.016, 0.022)),
            origin=Origin(xyz=(x + dx, y, z_top + 0.018)),
            material=material,
            name=f"socket_cheek_{index}_{side}",
        )
    housing.visual(
        Cylinder(radius=0.003, length=0.040),
        origin=Origin(xyz=(x, y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=f"hinge_pin_{index}",
    )


def _add_antenna_geometry(antenna, material) -> None:
    """Moving antenna knuckle and vertical whip, in the child frame at the hinge."""
    antenna.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="hinge_barrel",
    )
    antenna.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=material,
        name="root_collar",
    )
    antenna.visual(
        Cylinder(radius=0.0038, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=material,
        name="stalk",
    )
    antenna.visual(
        Sphere(radius=0.0048),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=material,
        name="tip_cap",
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wifi_router")

    shell_mat = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    trim_mat = model.material("charcoal_trim", rgba=(0.09, 0.095, 0.10, 1.0))
    rubber_mat = model.material("soft_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    green_mat = model.material("green_status_lens", rgba=(0.0, 0.85, 0.20, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_rounded_router_body(), "rounded_router_housing", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=shell_mat,
        name="rounded_shell",
    )

    housing.visual(
        Box((0.190, 0.095, 0.002)),
        origin=Origin(xyz=(0.0, -0.005, BODY_TOP_Z + 0.001)),
        material=trim_mat,
        name="recessed_top_panel",
    )

    # A small front status strip and separate green LEDs make the object read as a router.
    housing.visual(
        Box((0.095, 0.004, 0.004)),
        origin=Origin(xyz=(-0.030, -DEPTH / 2.0 - 0.001, BODY_BOTTOM_Z + 0.018)),
        material=trim_mat,
        name="front_status_strip",
    )
    for i, x in enumerate((-0.066, -0.047, -0.028, -0.009, 0.010)):
        housing.visual(
            Box((0.007, 0.002, 0.003)),
            origin=Origin(xyz=(x, -DEPTH / 2.0 - 0.003, BODY_BOTTOM_Z + 0.019)),
            material=green_mat,
            name=f"status_led_{i}",
        )

    # Shallow dark vent insets on both side faces.
    for side, x in enumerate((-WIDTH / 2.0 - 0.0003, WIDTH / 2.0 + 0.0003)):
        for i, y in enumerate((-0.045, -0.026, -0.007, 0.012, 0.031, 0.050)):
            housing.visual(
                Box((0.0012, 0.012, 0.005)),
                origin=Origin(xyz=(x, y, BODY_BOTTOM_Z + 0.018)),
                material=trim_mat,
                name=f"side_vent_{side}_{i}",
            )

    # Four small rubber feet protrude below the base.
    for i, (x, y) in enumerate(
        ((-0.100, -0.055), (0.100, -0.055), (-0.100, 0.055), (0.100, 0.055))
    ):
        housing.visual(
            Cylinder(radius=0.014, length=FOOT_HEIGHT),
            origin=Origin(xyz=(x, y, FOOT_HEIGHT / 2.0)),
            material=rubber_mat,
            name=f"foot_{i}",
        )

    for i, x in enumerate(ANTENNA_MOUNT_XS):
        _add_antenna_socket(housing, i, x, ANTENNA_MOUNT_Y, BODY_TOP_Z, shell_mat)

        antenna = model.part(f"antenna_{i}")
        _add_antenna_geometry(antenna, shell_mat)
        model.articulation(
            f"housing_to_antenna_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x, ANTENNA_MOUNT_Y, HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-ANTENNA_HALF_RANGE_RAD,
                upper=ANTENNA_HALF_RANGE_RAD,
                effort=1.2,
                velocity=2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    antennas = [object_model.get_part(f"antenna_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"housing_to_antenna_{i}") for i in range(4)]

    ctx.check("four external antennas", len(antennas) == 4)
    ctx.check("four root hinges", len(joints) == 4)

    for i, (antenna, joint) in enumerate(zip(antennas, joints)):
        limits = joint.motion_limits
        travel = None if limits is None else limits.upper - limits.lower
        ctx.check(
            f"antenna_{i} tilts about 120 degrees",
            travel is not None and abs(travel - ANTENNA_RANGE_RAD) < math.radians(3.0),
            details=f"travel={travel}",
        )
        ctx.expect_overlap(
            antenna,
            housing,
            axes="xy",
            elem_a="hinge_barrel",
            elem_b=f"socket_pad_{i}",
            min_overlap=0.012,
            name=f"antenna_{i} hinge sits over its rear socket",
        )
        ctx.expect_gap(
            antenna,
            housing,
            axis="z",
            positive_elem="hinge_barrel",
            negative_elem=f"socket_pad_{i}",
            min_gap=0.002,
            max_gap=0.008,
            name=f"antenna_{i} hinge clears socket pad without floating far above it",
        )
        ctx.allow_overlap(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"hinge_pin_{i}",
            reason="The antenna hinge barrel is intentionally captured around the fixed hinge pin.",
        )
        ctx.expect_within(
            housing,
            antenna,
            axes="yz",
            inner_elem=f"hinge_pin_{i}",
            outer_elem="hinge_barrel",
            margin=0.001,
            name=f"antenna_{i} hinge pin is captured inside barrel",
        )
        ctx.expect_overlap(
            antenna,
            housing,
            axes="x",
            elem_a="hinge_barrel",
            elem_b=f"hinge_pin_{i}",
            min_overlap=0.018,
            name=f"antenna_{i} hinge pin spans through barrel",
        )

    # At the upper stop the hinge axis tilts the whip forward in its vertical plane.
    first_joint = joints[0]
    first_antenna = antennas[0]
    rest_tip_aabb = ctx.part_element_world_aabb(first_antenna, elem="tip_cap")
    with ctx.pose({first_joint: ANTENNA_HALF_RANGE_RAD}):
        tilted_tip_aabb = ctx.part_element_world_aabb(first_antenna, elem="tip_cap")
    ctx.check(
        "antenna hinge tilts forward at upper limit",
        rest_tip_aabb is not None
        and tilted_tip_aabb is not None
        and tilted_tip_aabb[0][1] < rest_tip_aabb[0][1] - 0.08
        and tilted_tip_aabb[0][2] < rest_tip_aabb[0][2] - 0.05,
        details=f"rest={rest_tip_aabb}, tilted={tilted_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
