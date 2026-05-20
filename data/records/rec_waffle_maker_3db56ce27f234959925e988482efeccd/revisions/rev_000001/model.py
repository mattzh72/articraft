from __future__ import annotations

import math

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
)


BODY_WIDTH = 0.285
BODY_DEPTH = 0.245
HINGE_X = -0.104
HINGE_Z = 0.086
LID_CENTER_X = 0.112
PLATE_DEPTH = 0.188
PLATE_WIDTH = 0.222
PLATE_THICKNESS = 0.010
BUBBLE_RADIUS = 0.014


def cyl_x(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def cyl_y(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def bubble_points() -> list[tuple[float, float]]:
    rows = (
        (-0.060, (-0.082, -0.041, 0.000, 0.041, 0.082)),
        (-0.030, (-0.061, -0.020, 0.020, 0.061)),
        (0.000, (-0.082, -0.041, 0.000, 0.041, 0.082)),
        (0.030, (-0.061, -0.020, 0.020, 0.061)),
        (0.060, (-0.082, -0.041, 0.000, 0.041, 0.082)),
    )
    return [(x, y) for x, ys in rows for y in ys]


def add_bubble_plate(part, *, upper: bool, material) -> None:
    slab_center_z = PLATE_THICKNESS / 2.0 if upper else -PLATE_THICKNESS / 2.0
    slab_name = "upper_plate" if upper else "lower_plate"
    part.visual(
        Box((PLATE_DEPTH, PLATE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, slab_center_z)),
        material=material,
        name=slab_name,
    )
    for idx, (x, y) in enumerate(bubble_points()):
        part.visual(
            Sphere(BUBBLE_RADIUS),
            origin=Origin(xyz=(x, y, 0.0)),
            material=material,
            name=f"{slab_name}_bubble_{idx}",
        )


def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bubble_waffle_maker")

    shell_green = model.material("shell_green", rgba=(0.42, 0.60, 0.52, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.10, 1.0))
    plate_black = model.material("plate_black", rgba=(0.16, 0.16, 0.17, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.62, 0.64, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.214, 0.246, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=shell_green,
        name="body_core",
    )
    body.visual(
        Box((0.192, 0.228, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=shell_green,
        name="body_crown",
    )
    body.visual(
        Box((0.034, 0.228, 0.010)),
        origin=Origin(xyz=(0.105, 0.0, 0.079)),
        material=shell_green,
        name="body_shell",
    )
    body.visual(
        Box((0.194, 0.228, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=shell_green,
        name="plate_pad",
    )
    body.visual(
        Box((0.020, 0.010, 0.020)),
        origin=Origin(xyz=(0.028, 0.140, 0.048)),
        material=shell_green,
        name="dial_mount",
    )
    geom, origin = cyl_x(0.022, 0.214, (0.0, -0.123, 0.022))
    body.visual(geom, origin=origin, material=shell_green, name="body_side_0")
    geom, origin = cyl_x(0.022, 0.214, (0.0, 0.123, 0.022))
    body.visual(geom, origin=origin, material=shell_green, name="body_side_1")
    geom, origin = cyl_y(0.022, 0.246, (-0.107, 0.0, 0.022))
    body.visual(geom, origin=origin, material=shell_green, name="body_end_0")
    geom, origin = cyl_y(0.022, 0.246, (0.107, 0.0, 0.022))
    body.visual(geom, origin=origin, material=shell_green, name="body_end_1")
    geom, origin = cyl_y(0.008, 0.095, (HINGE_X, -0.080, HINGE_Z))
    body.visual(geom, origin=origin, material=hinge_metal, name="body_hinge_0")
    geom, origin = cyl_y(0.008, 0.095, (HINGE_X, 0.080, HINGE_Z))
    body.visual(geom, origin=origin, material=hinge_metal, name="body_hinge_1")
    body.visual(
        Box((0.022, 0.095, 0.014)),
        origin=Origin(xyz=(HINGE_X, -0.080, 0.079)),
        material=hinge_metal,
        name="body_hinge_support_0",
    )
    body.visual(
        Box((0.022, 0.095, 0.014)),
        origin=Origin(xyz=(HINGE_X, 0.080, 0.079)),
        material=hinge_metal,
        name="body_hinge_support_1",
    )

    lower_plate = model.part("lower_plate")
    add_bubble_plate(lower_plate, upper=False, material=plate_black)

    lid = model.part("lid")
    lid.visual(
        Box((0.038, 0.234, 0.010)),
        origin=Origin(xyz=(0.214, 0.0, 0.009)),
        material=shell_green,
        name="lid_shell",
    )
    lid.visual(
        Box((0.186, 0.236, 0.044)),
        origin=Origin(xyz=(0.122, 0.0, 0.035)),
        material=shell_green,
        name="lid_dome_box",
    )
    lid.visual(
        Sphere(0.044),
        origin=Origin(xyz=(0.128, 0.0, 0.058)),
        material=shell_green,
        name="lid_dome",
    )
    lid.visual(
        Box((0.194, 0.228, 0.004)),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, 0.008)),
        material=shell_green,
        name="underside_pad",
    )
    geom, origin = cyl_y(0.008, 0.056, (0.0, 0.0, 0.0))
    lid.visual(geom, origin=origin, material=hinge_metal, name="lid_hinge")
    lid.visual(
        Box((0.020, 0.056, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.009)),
        material=hinge_metal,
        name="lid_hinge_web",
    )
    lid.visual(
        Box((0.014, 0.078, 0.016)),
        origin=Origin(xyz=(0.223, 0.0, 0.020)),
        material=shell_green,
        name="handle_mount",
    )
    lid.visual(
        Box((0.016, 0.022, 0.020)),
        origin=Origin(xyz=(0.232, -0.028, 0.020)),
        material=handle_black,
        name="handle_post_0",
    )
    lid.visual(
        Box((0.016, 0.022, 0.020)),
        origin=Origin(xyz=(0.232, 0.028, 0.020)),
        material=handle_black,
        name="handle_post_1",
    )
    geom, origin = cyl_y(0.009, 0.076, (0.244, 0.0, 0.030))
    lid.visual(geom, origin=origin, material=handle_black, name="handle_bar")

    upper_plate = model.part("upper_plate")
    add_bubble_plate(upper_plate, upper=True, material=plate_black)

    thermostat_dial = model.part("thermostat_dial")
    geom, origin = cyl_y(0.005, 0.012, (0.0, 0.006, 0.0))
    thermostat_dial.visual(geom, origin=origin, material=handle_black, name="shaft")
    geom, origin = cyl_y(0.019, 0.018, (0.0, 0.009, 0.0))
    thermostat_dial.visual(geom, origin=origin, material=handle_black, name="dial_cap")
    geom, origin = cyl_y(0.015, 0.004, (0.0, 0.018, 0.0))
    thermostat_dial.visual(geom, origin=origin, material=handle_black, name="dial_front")
    thermostat_dial.visual(
        Box((0.003, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, 0.019, 0.010)),
        material=hinge_metal,
        name="indicator",
    )

    model.articulation(
        "body_to_lower_plate",
        ArticulationType.FIXED,
        parent=body,
        child=lower_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "lid_to_upper_plate",
        ArticulationType.FIXED,
        parent=lid,
        child=upper_plate,
        origin=Origin(xyz=(LID_CENTER_X, 0.0, -0.004)),
    )
    model.articulation(
        "body_to_thermostat_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=thermostat_dial,
        origin=Origin(xyz=(0.028, 0.145, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("thermostat_dial")
    lower_plate = object_model.get_part("lower_plate")
    upper_plate = object_model.get_part("upper_plate")
    lid_hinge = object_model.get_articulation("body_to_lid")
    dial_joint = object_model.get_articulation("body_to_thermostat_dial")
    limits = lid_hinge.motion_limits

    ctx.allow_overlap(
        lower_plate,
        upper_plate,
        reason="Full spheres are used as simplified proxies for opposing hemispherical bubble molds that meet at closure.",
    )
    ctx.allow_overlap(
        body,
        lower_plate,
        reason="The lower housing stays visually solid while the bubble plate is simplified as a mounted insert rather than a fully recessed cavity.",
    )
    ctx.allow_overlap(
        body,
        upper_plate,
        reason="The closed appliance uses a simplified solid lower housing proxy while the upper bubble mold closes into that envelope.",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.lower}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="lid_shell",
                negative_elem="body_shell",
                min_gap=0.004,
                max_gap=0.012,
                name="lid shell stays visibly separate from lower shell",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="y",
                elem_a="lid_shell",
                elem_b="body_shell",
                min_overlap=0.18,
                name="lid front seam spans the body width",
            )
            ctx.expect_contact(
                dial,
                body,
                elem_a="shaft",
                elem_b="dial_mount",
                name="dial shaft meets the side wall",
            )
            closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle_bar")

        with ctx.pose({lid_hinge: limits.upper}):
            open_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle_bar")

        closed_handle_center = aabb_center(closed_handle_aabb)
        open_handle_center = aabb_center(open_handle_aabb)
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_handle_center is not None
            and open_handle_center is not None
            and open_handle_center[2] > closed_handle_center[2] + 0.08
            and open_handle_center[0] < closed_handle_center[0] - 0.02,
            details=f"closed={closed_handle_center}, open={open_handle_center}",
        )

    with ctx.pose({dial_joint: math.pi / 2.0}):
        ctx.expect_contact(
            dial,
            body,
            elem_a="shaft",
            elem_b="dial_mount",
            name="dial stays seated while rotating",
        )

    return ctx.report()


object_model = build_object_model()
