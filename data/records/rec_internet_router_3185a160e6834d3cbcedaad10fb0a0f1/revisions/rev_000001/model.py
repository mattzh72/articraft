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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_X = 0.240
BODY_Y = 0.160
BODY_Z = 0.034
HINGE_Y = 0.046
HINGE_Z = 0.023
HINGE_X = BODY_X / 2.0 + 0.009
ANTENNA_LIMIT = 1.25


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mesh_office_router")

    warm_gray = model.material("warm_gray_plastic", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_gray = model.material("dark_gray_rubber", rgba=(0.045, 0.048, 0.052, 1.0))
    vent_black = model.material("black_vent_shadow", rgba=(0.010, 0.011, 0.012, 1.0))
    led_green = model.material("soft_green_led", rgba=(0.15, 0.85, 0.28, 1.0))
    port_black = model.material("port_black", rgba=(0.0, 0.0, 0.0, 1.0))

    housing = model.part("housing")

    body_shape = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_Z)
        .edges("|Z")
        .fillet(0.010)
    )
    housing.visual(
        mesh_from_cadquery(body_shape, "rounded_router_body", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z / 2.0)),
        material=warm_gray,
        name="body_shell",
    )

    # A dark cavity just under the slotted top makes the through slots read as
    # open ventilation instead of a printed decoration.
    housing.visual(
        Box((0.180, 0.104, 0.0012)),
        origin=Origin(xyz=(0.0, -0.006, BODY_Z + 0.0006)),
        material=vent_black,
        name="vent_shadow",
    )
    top_vent = SlotPatternPanelGeometry(
        (0.176, 0.100),
        0.0020,
        slot_size=(0.030, 0.0045),
        pitch=(0.040, 0.014),
        frame=0.012,
        corner_radius=0.007,
        stagger=True,
    )
    housing.visual(
        mesh_from_geometry(top_vent, "slotted_top_vent"),
        origin=Origin(xyz=(0.0, -0.006, BODY_Z + 0.0012)),
        material=dark_gray,
        name="top_vent",
    )

    # Subtle front status lights and rear port recesses make the shallow box
    # read as an office network appliance while remaining part of the housing.
    for i, x in enumerate((-0.034, -0.014, 0.006, 0.026)):
        housing.visual(
            Box((0.006, 0.0022, 0.0030)),
            origin=Origin(xyz=(x, -BODY_Y / 2.0 - 0.0009, 0.018)),
            material=led_green if i == 0 else dark_gray,
            name=f"status_light_{i}",
        )
    for i, x in enumerate((-0.046, -0.018, 0.018, 0.048)):
        housing.visual(
            Box((0.018, 0.0025, 0.010)),
            origin=Origin(xyz=(x, BODY_Y / 2.0 + 0.0008, 0.016)),
            material=port_black,
            name=f"rear_port_{i}",
        )

    # Side hinge pods are a fixed part of the case: each pod has two outboard
    # lugs with a gap that captures the rotating antenna collar.
    for idx, side in enumerate((-1.0, 1.0)):
        x = side * HINGE_X
        for lug_name, y_offset in (("front", -0.020), ("rear", 0.020)):
            housing.visual(
                Cylinder(radius=0.011, length=0.012),
                origin=Origin(
                    xyz=(x, HINGE_Y + y_offset, HINGE_Z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_gray,
                name=f"hinge_lug_{idx}_{lug_name}",
            )

        antenna = model.part(f"antenna_{idx}")
        _add_paddle_antenna(antenna, side, dark_gray, warm_gray)

        model.articulation(
            f"housing_to_antenna_{idx}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z)),
            axis=(0.0, side, 0.0),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=1.3,
                lower=0.0,
                upper=ANTENNA_LIMIT,
            ),
        )

    return model


def _add_paddle_antenna(part, side: float, dark_gray, warm_gray) -> None:
    """Add one side-specific paddle antenna in the part frame at its hinge axis."""

    # The collar sits between the two fixed lugs on the housing pod.  Its axis
    # is the joint axis, so it stays clipped as the paddle folds.
    part.visual(
        Cylinder(radius=0.0072, length=0.028),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="hinge_collar",
    )

    outward_x = side * 0.0145
    neck_x = side * 0.0080
    part.visual(
        Box((0.009, 0.012, 0.030)),
        origin=Origin(xyz=(neck_x, 0.0, 0.021)),
        material=dark_gray,
        name="neck",
    )

    paddle_shape = (
        cq.Workplane("XY")
        .box(0.007, 0.042, 0.108)
        .edges("|Z")
        .fillet(0.0025)
    )
    part.visual(
        mesh_from_cadquery(
            paddle_shape,
            f"paddle_antenna_{0 if side < 0 else 1}",
            tolerance=0.0006,
        ),
        origin=Origin(xyz=(outward_x, 0.0, 0.088)),
        material=dark_gray,
        name="paddle",
    )
    part.visual(
        Box((0.004, 0.026, 0.070)),
        origin=Origin(xyz=(outward_x + side * 0.0039, 0.0, 0.090)),
        material=warm_gray,
        name="raised_center_strip",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    for idx in (0, 1):
        antenna = object_model.get_part(f"antenna_{idx}")
        hinge = object_model.get_articulation(f"housing_to_antenna_{idx}")

        ctx.expect_within(
            antenna,
            housing,
            axes="xz",
            inner_elem="hinge_collar",
            outer_elem=f"hinge_lug_{idx}_front",
            margin=0.001,
            name=f"antenna_{idx} collar is coaxial with fixed pod",
        )
        ctx.expect_gap(
            antenna,
            housing,
            axis="y",
            positive_elem="hinge_collar",
            negative_elem=f"hinge_lug_{idx}_front",
            min_gap=-0.0002,
            max_gap=0.001,
            name=f"antenna_{idx} front clip clearance",
        )
        ctx.expect_gap(
            housing,
            antenna,
            axis="y",
            positive_elem=f"hinge_lug_{idx}_rear",
            negative_elem="hinge_collar",
            min_gap=-0.0002,
            max_gap=0.001,
            name=f"antenna_{idx} rear clip clearance",
        )

        rest_aabb = ctx.part_world_aabb(antenna)
        with ctx.pose({hinge: ANTENNA_LIMIT}):
            folded_aabb = ctx.part_world_aabb(antenna)

        if idx == 0:
            moves_outward = (
                rest_aabb is not None
                and folded_aabb is not None
                and folded_aabb[0][0] < rest_aabb[0][0] - 0.030
                and folded_aabb[1][2] < rest_aabb[1][2] - 0.015
            )
        else:
            moves_outward = (
                rest_aabb is not None
                and folded_aabb is not None
                and folded_aabb[1][0] > rest_aabb[1][0] + 0.030
                and folded_aabb[1][2] < rest_aabb[1][2] - 0.015
            )
        ctx.check(
            f"antenna_{idx} folds outward on side hinge",
            moves_outward,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
