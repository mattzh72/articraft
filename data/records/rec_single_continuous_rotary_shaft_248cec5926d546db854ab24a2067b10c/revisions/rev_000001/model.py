from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.05
BASE_WIDTH = 0.32
BASE_THICKNESS = 0.045
RAIL_WIDTH = 0.040
RAIL_HEIGHT = 0.026

SHAFT_Z = 0.220
SHAFT_RADIUS = 0.025
SHAFT_LENGTH = 0.98

BLOCK_X_WIDTH = 0.140
BLOCK_SPACING = 0.52
FOOT_X = 0.220
FOOT_Y = 0.295
FOOT_HEIGHT = 0.055
BODY_Y = 0.220
BODY_RADIUS = BODY_Y / 2.0
BODY_BOTTOM_Z = BASE_THICKNESS + FOOT_HEIGHT - 0.001

BEARING_OUTER_RADIUS = 0.066
BEARING_INNER_RADIUS = 0.033
BEARING_LENGTH = BLOCK_X_WIDTH + 0.018
BODY_BORE_RADIUS = 0.061
BALL_RADIUS = 0.006
BALL_CENTER_RADIUS = SHAFT_RADIUS + BALL_RADIUS

FLANGE_X = -0.430
FLANGE_RADIUS = 0.140
FLANGE_THICKNESS = 0.036
HUB_RADIUS = 0.060
HUB_THICKNESS = 0.072
BOLT_CIRCLE_RADIUS = 0.095


def _cylinder_x_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _build_pillow_body(x: float) -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_X, FOOT_Y, FOOT_HEIGHT)
        .translate((x, 0.0, BASE_THICKNESS + FOOT_HEIGHT / 2.0))
    )
    arched_housing = (
        cq.Workplane("YZ")
        .moveTo(-BODY_RADIUS, BODY_BOTTOM_Z)
        .lineTo(BODY_RADIUS, BODY_BOTTOM_Z)
        .lineTo(BODY_RADIUS, SHAFT_Z)
        .threePointArc((0.0, SHAFT_Z + BODY_RADIUS), (-BODY_RADIUS, SHAFT_Z))
        .close()
        .extrude(BLOCK_X_WIDTH)
        .translate((x - BLOCK_X_WIDTH / 2.0, 0.0, 0.0))
    )
    bore_cutter = (
        cq.Workplane("YZ")
        .circle(BODY_BORE_RADIUS)
        .extrude(BLOCK_X_WIDTH + 0.030)
        .translate((x - BLOCK_X_WIDTH / 2.0 - 0.015, 0.0, SHAFT_Z))
    )
    return foot.union(arched_housing).cut(bore_cutter)


def _build_frame_body() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    frame = base
    for y in (-BASE_WIDTH / 2.0 + 0.045, BASE_WIDTH / 2.0 - 0.045):
        rail = (
            cq.Workplane("XY")
            .box(BASE_LENGTH * 0.92, RAIL_WIDTH, RAIL_HEIGHT)
            .translate((0.0, y, BASE_THICKNESS + RAIL_HEIGHT / 2.0 - 0.001))
        )
        frame = frame.union(rail)

    for x in (-BLOCK_SPACING / 2.0, BLOCK_SPACING / 2.0):
        frame = frame.union(_build_pillow_body(x))

    return frame.edges("|Z").fillet(0.006)


def _build_bearing_sleeve(x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(BEARING_OUTER_RADIUS)
        .circle(BEARING_INNER_RADIUS)
        .extrude(BEARING_LENGTH)
        .translate((x - BEARING_LENGTH / 2.0, 0.0, SHAFT_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_spin_fixture")

    model.material("painted_frame", rgba=(0.08, 0.11, 0.14, 1.0))
    model.material("bearing_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    model.material("dark_fastener", rgba=(0.015, 0.016, 0.018, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_body(), "frame_body", tolerance=0.0015),
        material="painted_frame",
        name="frame_body",
    )
    for index, x in enumerate((-BLOCK_SPACING / 2.0, BLOCK_SPACING / 2.0)):
        frame.visual(
            mesh_from_cadquery(
                _build_bearing_sleeve(x),
                f"bearing_{index}",
                tolerance=0.001,
                angular_tolerance=0.08,
            ),
            material="bearing_steel",
            name=f"bearing_{index}",
        )
        for ball_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            frame.visual(
                Sphere(radius=BALL_RADIUS),
                origin=Origin(
                    xyz=(
                        x,
                        BALL_CENTER_RADIUS * math.cos(angle),
                        SHAFT_Z + BALL_CENTER_RADIUS * math.sin(angle),
                    )
                ),
                material="bearing_steel",
                name=f"bearing_ball_{index}_{ball_index}",
            )
        for bx in (-0.062, 0.062):
            for by in (-0.108, 0.108):
                frame.visual(
                    Cylinder(radius=0.012, length=0.008),
                    origin=Origin(
                        xyz=(
                            x + bx,
                            by,
                            BASE_THICKNESS + FOOT_HEIGHT + 0.003,
                        )
                    ),
                    material="dark_fastener",
                    name=f"mount_bolt_{index}_{0 if bx < 0 else 1}_{0 if by < 0 else 1}",
                )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=_cylinder_x_origin((0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_THICKNESS),
        origin=_cylinder_x_origin((FLANGE_X, 0.0, 0.0)),
        material="brushed_steel",
        name="flange",
    )
    rotor.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=_cylinder_x_origin((FLANGE_X, 0.0, 0.0)),
        material="brushed_steel",
        name="hub",
    )
    for index in range(6):
        angle = index * math.tau / 6.0
        rotor.visual(
            Cylinder(radius=0.010, length=0.009),
            origin=_cylinder_x_origin(
                (
                    FLANGE_X - FLANGE_THICKNESS / 2.0 - 0.004,
                    BOLT_CIRCLE_RADIUS * math.cos(angle),
                    BOLT_CIRCLE_RADIUS * math.sin(angle),
                )
            ),
            material="dark_fastener",
            name=f"flange_bolt_{index}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "single continuous spin joint",
        len(object_model.articulations) == 1
        and joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )
    for bearing_name in ("bearing_0", "bearing_1"):
        ctx.expect_within(
            rotor,
            frame,
            axes="yz",
            inner_elem="shaft",
            outer_elem=bearing_name,
            margin=0.0,
            name=f"shaft centered within {bearing_name}",
        )
        ctx.expect_overlap(
            rotor,
            frame,
            axes="x",
            elem_a="shaft",
            elem_b=bearing_name,
            min_overlap=0.12,
            name=f"shaft spans through {bearing_name}",
        )
        ctx.expect_contact(
            rotor,
            frame,
            elem_a="shaft",
            elem_b=bearing_name.replace("bearing_", "bearing_ball_") + "_0",
            contact_tol=0.001,
            name=f"shaft has rolling contact at {bearing_name}",
        )

    flange_aabb = ctx.part_element_world_aabb(rotor, elem="flange")
    shaft_aabb = ctx.part_element_world_aabb(rotor, elem="shaft")
    ctx.check(
        "flange mounted near one shaft end",
        flange_aabb is not None
        and shaft_aabb is not None
        and flange_aabb[0][0] < shaft_aabb[0][0] + 0.10,
        details=f"flange_aabb={flange_aabb}, shaft_aabb={shaft_aabb}",
    )

    rest_bolt = ctx.part_element_world_aabb(rotor, elem="flange_bolt_0")
    with ctx.pose({joint: math.pi / 2.0}):
        spun_bolt = ctx.part_element_world_aabb(rotor, elem="flange_bolt_0")

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "flange bolts spin about shaft axis",
        rest_bolt is not None
        and spun_bolt is not None
        and _center_z(spun_bolt) is not None
        and _center_z(rest_bolt) is not None
        and _center_z(spun_bolt) > _center_z(rest_bolt) + 0.07,
        details=f"rest={rest_bolt}, spun={spun_bolt}",
    )

    return ctx.report()


object_model = build_object_model()
