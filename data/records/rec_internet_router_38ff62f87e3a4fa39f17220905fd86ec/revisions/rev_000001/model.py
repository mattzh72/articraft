from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_W = 0.300
HOUSING_D = 0.170
HOUSING_H = 0.036
REAR_Y = HOUSING_D / 2.0

BUTTON_X = -0.095
BUTTON_Y = -0.038
BUTTON_RECESS_R = 0.013
BUTTON_RECESS_DEPTH = 0.020
BUTTON_TRAVEL = 0.004
BUTTON_THICKNESS = 0.003
BUTTON_CENTER_Z = HOUSING_H - 0.006

ANTENNA_XS = (-0.108, -0.036, 0.036, 0.108)
ANTENNA_HINGE_Y = REAR_Y + 0.006
ANTENNA_HINGE_Z = HOUSING_H + 0.019
ANTENNA_UPPER = 1.25


def _router_body_shape() -> cq.Workplane:
    """Rounded low router body with a real blind cylindrical button recess."""
    return (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H)
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .workplane()
        .center(BUTTON_X, BUTTON_Y)
        .circle(BUTTON_RECESS_R)
        .cutBlind(-BUTTON_RECESS_DEPTH)
    )


def _annular_ring_shape(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)


def _antenna_blade_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.012, 0.006, 0.165)
        .edges("|Z")
        .fillet(0.0028)
    )


def _add_hinge_base(
    housing,
    *,
    x: float,
    index: int,
    base_material: Material,
    cap_material: Material,
) -> None:
    """Add the fixed rear-edge yoke and mounting foot for one antenna."""
    housing.visual(
        Box((0.042, 0.026, 0.009)),
        origin=Origin(xyz=(x, REAR_Y + 0.003, HOUSING_H + 0.0035)),
        material=base_material,
        name=f"hinge_foot_{index}",
    )
    for side, sx in (("a", -0.016), ("b", 0.016)):
        housing.visual(
            Box((0.004, 0.014, 0.025)),
            origin=Origin(xyz=(x + sx, ANTENNA_HINGE_Y, ANTENNA_HINGE_Z)),
            material=base_material,
            name=f"hinge_ear_{index}_{side}",
        )
    for side, sx in (("a", -0.019), ("b", 0.019)):
        housing.visual(
            Cylinder(radius=0.006, length=0.002),
            origin=Origin(
                xyz=(x + sx, ANTENNA_HINGE_Y, ANTENNA_HINGE_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cap_material,
            name=f"hinge_cap_{index}_{side}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wifi_router")

    matte_charcoal = model.material("matte_charcoal", rgba=(0.030, 0.034, 0.038, 1.0))
    satin_black = model.material("satin_black", rgba=(0.006, 0.007, 0.008, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.010, 0.010, 0.011, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.45, 0.46, 0.47, 1.0))
    led_green = model.material("led_green", rgba=(0.05, 0.90, 0.32, 1.0))
    port_black = model.material("port_black", rgba=(0.0, 0.0, 0.0, 1.0))

    body_mesh = mesh_from_cadquery(
        _router_body_shape(),
        "router_body_shell",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    button_ring_mesh = mesh_from_cadquery(
        _annular_ring_shape(0.014, 0.0105, 0.001),
        "button_recess_ring",
        tolerance=0.0004,
        angular_tolerance=0.08,
    )
    blade_mesh = mesh_from_cadquery(
        _antenna_blade_shape(),
        "antenna_blade",
        tolerance=0.0006,
        angular_tolerance=0.08,
    )

    housing = model.part("housing")
    housing.visual(
        body_mesh,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_H / 2.0)),
        material=matte_charcoal,
        name="body_shell",
    )
    housing.visual(
        Box((0.220, 0.086, 0.0012)),
        origin=Origin(xyz=(0.0, -0.010, HOUSING_H + 0.0004)),
        material=Material("top_sheen", rgba=(0.055, 0.060, 0.065, 1.0)),
        name="top_inset",
    )
    housing.visual(
        button_ring_mesh,
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, HOUSING_H - 0.00015)),
        material=port_black,
        name="button_well_lip",
    )
    for i, x in enumerate((-0.046, -0.023, 0.0, 0.023, 0.046)):
        housing.visual(
            Box((0.010, 0.002, 0.0035)),
            origin=Origin(xyz=(x, -REAR_Y - 0.0002, 0.020)),
            material=led_green,
            name=f"front_led_{i}",
        )
    for i, x in enumerate((-0.045, -0.015, 0.015, 0.045)):
        housing.visual(
            Box((0.023, 0.0025, 0.012)),
            origin=Origin(xyz=(x, REAR_Y + 0.0005, 0.017)),
            material=port_black,
            name=f"rear_port_{i}",
        )
    for i, x in enumerate(ANTENNA_XS):
        _add_hinge_base(
            housing,
            x=x,
            index=i,
            base_material=satin_black,
            cap_material=dark_rubber,
        )

    for i, x in enumerate(ANTENNA_XS):
        antenna = model.part(f"antenna_{i}")
        antenna.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=dark_rubber,
            name="root_collar",
        )
        antenna.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.095)),
            material=satin_black,
            name="blade",
        )
        antenna.visual(
            Sphere(radius=0.0055),
            origin=Origin(xyz=(0.0, 0.0, 0.181)),
            material=satin_black,
            name="rounded_tip",
        )
        model.articulation(
            f"housing_to_antenna_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x, ANTENNA_HINGE_Y, ANTENNA_HINGE_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-0.25,
                upper=ANTENNA_UPPER,
                effort=0.25,
                velocity=2.2,
            ),
        )

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.008, length=BUTTON_THICKNESS),
        origin=Origin(),
        material=soft_gray,
        name="plunger",
    )
    button.visual(
        Cylinder(radius=0.0133, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.0044)),
        material=soft_gray,
        name="guide_sleeve",
    )
    model.articulation(
        "housing_to_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, BUTTON_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BUTTON_TRAVEL,
            effort=1.2,
            velocity=0.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    button = object_model.get_part("button")
    button_joint = object_model.get_articulation("housing_to_button")

    ctx.allow_overlap(
        button,
        housing,
        elem_a="guide_sleeve",
        elem_b="body_shell",
        reason=(
            "The reset plunger guide is intentionally nested into the recessed "
            "cylindrical bore so the button is retained while it slides."
        ),
    )

    ctx.check(
        "four independent antenna hinges",
        all(
            object_model.get_articulation(f"housing_to_antenna_{i}") is not None
            for i in range(4)
        ),
    )

    ctx.expect_within(
        button,
        housing,
        axes="xy",
        inner_elem="plunger",
        outer_elem="body_shell",
        margin=0.0,
        name="button lies inside the recessed housing footprint",
    )
    ctx.expect_within(
        button,
        housing,
        axes="xy",
        inner_elem="guide_sleeve",
        outer_elem="body_shell",
        margin=0.001,
        name="button guide stays captured by the body bore",
    )
    ctx.expect_overlap(
        button,
        housing,
        axes="z",
        elem_a="guide_sleeve",
        elem_b="body_shell",
        min_overlap=0.004,
        name="button guide remains inserted in the bore",
    )
    plunger_aabb = ctx.part_element_world_aabb(button, elem="plunger")
    if plunger_aabb is not None:
        ctx.check(
            "button face is visibly recessed",
            plunger_aabb[1][2] < HOUSING_H - 0.002,
            details=f"button_top={plunger_aabb[1][2]:.4f}, housing_top={HOUSING_H:.4f}",
        )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        depressed_button = ctx.part_world_position(button)
        depressed_aabb = ctx.part_element_world_aabb(button, elem="plunger")
        if depressed_aabb is not None:
            ctx.check(
                "depressed button remains above recess floor",
                depressed_aabb[0][2] > HOUSING_H - BUTTON_RECESS_DEPTH + 0.001,
                details=f"depressed_bottom={depressed_aabb[0][2]:.4f}",
            )
    ctx.check(
        "button plunges inward",
        rest_button is not None
        and depressed_button is not None
        and depressed_button[2] < rest_button[2] - 0.003,
        details=f"rest={rest_button}, depressed={depressed_button}",
    )

    for i in range(4):
        antenna = object_model.get_part(f"antenna_{i}")
        hinge = object_model.get_articulation(f"housing_to_antenna_{i}")
        rest_blade = ctx.part_element_world_aabb(antenna, elem="blade")
        with ctx.pose({hinge: ANTENNA_UPPER}):
            tilted_blade = ctx.part_element_world_aabb(antenna, elem="blade")
        if rest_blade is not None and tilted_blade is not None:
            rest_center = tuple((rest_blade[0][j] + rest_blade[1][j]) * 0.5 for j in range(3))
            tilted_center = tuple(
                (tilted_blade[0][j] + tilted_blade[1][j]) * 0.5 for j in range(3)
            )
            ctx.check(
                f"antenna {i} tilts rearward",
                tilted_center[1] > rest_center[1] + 0.070
                and tilted_center[2] < rest_center[2] - 0.020,
                details=f"rest_center={rest_center}, tilted_center={tilted_center}",
            )

    return ctx.report()


object_model = build_object_model()
