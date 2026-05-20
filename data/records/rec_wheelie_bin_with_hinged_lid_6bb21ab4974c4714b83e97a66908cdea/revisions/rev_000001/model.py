from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_BOTTOM_Z = 0.17
BODY_TOP_Z = 0.78
BODY_HEIGHT = BODY_TOP_Z - BODY_BOTTOM_Z
BODY_BOTTOM_DEPTH = 0.44
BODY_TOP_DEPTH = 0.56
BODY_BOTTOM_WIDTH = 0.58
BODY_TOP_WIDTH = 0.74
WALL = 0.030
BASE_THICKNESS = 0.045

HINGE_X = -BODY_TOP_DEPTH / 2.0 - 0.018
HINGE_Z = BODY_TOP_Z + 0.055
LID_DEPTH = BODY_TOP_DEPTH + 0.055
LID_WIDTH = BODY_TOP_WIDTH + 0.045
LID_THICKNESS = 0.032

AXLE_X = -BODY_BOTTOM_DEPTH / 2.0 - 0.010
AXLE_Z = 0.135
WHEEL_RADIUS = 0.125
WHEEL_WIDTH = 0.070
WHEEL_Y = BODY_TOP_WIDTH / 2.0 + WHEEL_WIDTH / 2.0 + 0.020


def _bin_body_shell() -> cq.Workplane:
    """Open, tapered plastic tub with real wall thickness and a closed floor."""

    outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BOTTOM_Z)
        .rect(BODY_BOTTOM_DEPTH, BODY_BOTTOM_WIDTH)
        .workplane(offset=BODY_HEIGHT)
        .rect(BODY_TOP_DEPTH, BODY_TOP_WIDTH)
        .loft(combine=True)
    )

    inner_height = BODY_TOP_Z + 0.080 - (BODY_BOTTOM_Z + BASE_THICKNESS)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BOTTOM_Z + BASE_THICKNESS)
        .rect(BODY_BOTTOM_DEPTH - 2.0 * WALL, BODY_BOTTOM_WIDTH - 2.0 * WALL)
        .workplane(offset=inner_height)
        .rect(BODY_TOP_DEPTH - 2.0 * WALL, BODY_TOP_WIDTH - 2.0 * WALL)
        .loft(combine=True)
    )

    shell = outer.cut(inner)

    rim_outer = cq.Workplane("XY").box(BODY_TOP_DEPTH + 0.030, BODY_TOP_WIDTH + 0.030, 0.030)
    rim_inner = cq.Workplane("XY").box(BODY_TOP_DEPTH - 0.055, BODY_TOP_WIDTH - 0.055, 0.038)
    rim = rim_outer.cut(rim_inner).translate((0.0, 0.0, BODY_TOP_Z + 0.014))

    return shell.union(rim)


def _lid_panel() -> cq.Workplane:
    """Slightly crowned, full-width lid panel whose frame origin is the hinge line."""

    slab = cq.Workplane("XY").box(LID_DEPTH, LID_WIDTH, LID_THICKNESS).translate(
        (LID_DEPTH / 2.0 + 0.022, 0.0, LID_THICKNESS / 2.0 + 0.003)
    )

    front_lip = cq.Workplane("XY").box(0.032, LID_WIDTH, 0.046).translate(
        (LID_DEPTH + 0.006, 0.0, -0.002)
    )
    side_lip_0 = cq.Workplane("XY").box(LID_DEPTH * 0.86, 0.012, 0.035).translate(
        (LID_DEPTH * 0.48, -(BODY_TOP_WIDTH / 2.0 + 0.024), -0.002)
    )
    side_lip_1 = cq.Workplane("XY").box(LID_DEPTH * 0.86, 0.012, 0.035).translate(
        (LID_DEPTH * 0.48, BODY_TOP_WIDTH / 2.0 + 0.024, -0.002)
    )
    raised_center = cq.Workplane("XY").box(LID_DEPTH * 0.64, LID_WIDTH * 0.68, 0.012).translate(
        (LID_DEPTH * 0.50, 0.0, LID_THICKNESS + 0.006)
    )

    return slab.union(front_lip).union(side_lip_0).union(side_lip_1).union(raised_center)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_wheelie_bin")

    bin_green = model.material("molded_green", color=(0.08, 0.34, 0.16, 1.0))
    dark_green = model.material("dark_green", color=(0.04, 0.22, 0.10, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    grey_plastic = model.material("grey_plastic", color=(0.28, 0.30, 0.30, 1.0))
    steel = model.material("galvanized_steel", color=(0.62, 0.64, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_bin_body_shell(), "tapered_open_body"),
        material=bin_green,
        name="tapered_body",
    )
    # Subtle front ribs and a lower kick plate make the molded tub read as a bin.
    for i, y in enumerate((-0.245, 0.0, 0.245)):
        body.visual(
            Box((0.070, 0.030, 0.460)),
            origin=Origin(xyz=(BODY_TOP_DEPTH / 2.0 - 0.010, y, BODY_BOTTOM_Z + 0.285)),
            material=dark_green,
            name=f"front_rib_{i}",
        )
    body.visual(
        Box((0.095, BODY_TOP_WIDTH * 0.74, 0.055)),
        origin=Origin(xyz=(BODY_TOP_DEPTH / 2.0 - 0.030, 0.0, BODY_BOTTOM_Z + 0.075)),
        material=dark_green,
        name="front_kick_plate",
    )

    # Rear handle and hinge support, physically tied into the rear rim.
    body.visual(
        Cylinder(radius=0.022, length=BODY_TOP_WIDTH * 0.90),
        origin=Origin(xyz=(HINGE_X - 0.030, 0.0, BODY_TOP_Z - 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_green,
        name="rear_grip_bar",
    )
    for i, y in enumerate((-BODY_TOP_WIDTH * 0.34, BODY_TOP_WIDTH * 0.34)):
        body.visual(
            Box((0.105, 0.044, 0.135)),
            origin=Origin(xyz=(HINGE_X + 0.006, y, BODY_TOP_Z - 0.070)),
            material=dark_green,
            name=f"handle_stanchion_{i}",
        )

    for i, y in enumerate((-(LID_WIDTH / 2.0 + 0.052), LID_WIDTH / 2.0 + 0.052)):
        body.visual(
            Cylinder(radius=0.023, length=0.075),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=f"body_hinge_knuckle_{i}",
        )
        body.visual(
            Box((0.045, 0.080, 0.060)),
            origin=Origin(xyz=(HINGE_X + 0.010, y, BODY_TOP_Z + 0.017)),
            material=dark_green,
            name=f"hinge_lug_{i}",
        )
        bridge_y = -BODY_TOP_WIDTH / 2.0 - 0.030 if y < 0.0 else BODY_TOP_WIDTH / 2.0 + 0.030
        body.visual(
            Box((0.045, 0.125, 0.026)),
            origin=Origin(xyz=(HINGE_X + 0.010, bridge_y, BODY_TOP_Z - 0.020)),
            material=dark_green,
            name=f"hinge_bridge_{i}",
        )
    body.visual(
        Cylinder(radius=0.010, length=2.0 * (LID_WIDTH / 2.0 + 0.095)),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    body.visual(
        Cylinder(radius=0.016, length=2.0 * WHEEL_Y + 0.020),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    for i, y in enumerate((-(WHEEL_Y - WHEEL_WIDTH / 2.0 - 0.008), WHEEL_Y - WHEEL_WIDTH / 2.0 - 0.008)):
        body.visual(
            Cylinder(radius=0.038, length=0.016),
            origin=Origin(xyz=(AXLE_X, y, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"axle_collar_{i}",
        )
    for i, y in enumerate((-BODY_BOTTOM_WIDTH * 0.38, BODY_BOTTOM_WIDTH * 0.38)):
        body.visual(
            Box((0.070, 0.065, 0.095)),
            origin=Origin(xyz=(AXLE_X + 0.020, y, AXLE_Z + 0.030)),
            material=dark_green,
            name=f"axle_boss_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel(), "full_width_lid"),
        material=bin_green,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.020, length=BODY_TOP_WIDTH * 0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_green,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.055, BODY_TOP_WIDTH * 0.52, 0.028)),
        origin=Origin(xyz=(0.043, 0.0, 0.020)),
        material=dark_green,
        name="rear_lid_web",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            radius=0.086,
            width=WHEEL_WIDTH,
            rim=WheelRim(inner_radius=0.055, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.033,
                width=0.052,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.040, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.045),
        ),
        "utility_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            outer_radius=WHEEL_RADIUS,
            width=WHEEL_WIDTH,
            inner_radius=0.086,
            tread=TireTread(style="block", depth=0.006, count=20, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.003),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "utility_tire",
    )
    wheel_visual_origin = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    wheels = []
    for i in range(2):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, origin=wheel_visual_origin, material=black_rubber, name="tire")
        wheel.visual(wheel_mesh, origin=wheel_visual_origin, material=grey_plastic, name="rim")
        wheel.visual(
            Cylinder(radius=0.038, length=WHEEL_WIDTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grey_plastic,
            name="hub_sleeve",
        )
        wheels.append(wheel)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.85),
    )
    for i, y in enumerate((-WHEEL_Y, WHEEL_Y)):
        model.articulation(
            f"axle_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheels[i],
            origin=Origin(xyz=(AXLE_X, y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_joint = object_model.get_articulation("body_to_lid")
    wheel_joint_0 = object_model.get_articulation("axle_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("axle_to_wheel_1")

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="hub_sleeve",
            reason="The steel axle is intentionally captured through the simplified spinning hub sleeve.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xyz",
            min_overlap=0.030,
            elem_a="rear_axle",
            elem_b="hub_sleeve",
            name=f"{wheel.name}_axle_captured_in_hub",
        )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_hinge_barrel",
        reason="The hinge pin is intentionally represented as captured inside the lid barrel.",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="xyz",
        min_overlap=0.018,
        elem_a="hinge_pin",
        elem_b="lid_hinge_barrel",
        name="lid_barrel_captured_on_hinge_pin",
    )

    ctx.check("one_full_width_lid_part", lid is not None, "The bin should have one moving full-width lid.")
    ctx.check(
        "lid_joint_is_rear_hinge",
        lid_joint is not None
        and lid_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_joint.axis) == (0.0, -1.0, 0.0),
        details=f"joint={lid_joint}",
    )
    ctx.check(
        "wheels_are_continuous",
        wheel_joint_0 is not None
        and wheel_joint_1 is not None
        and wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"wheel joints={wheel_joint_0}, {wheel_joint_1}",
    )

    ctx.expect_overlap(lid, body, axes="y", min_overlap=BODY_TOP_WIDTH * 0.85, elem_a="lid_panel", elem_b="tapered_body")
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_penetration=0.030,
        max_gap=0.055,
        elem_a="lid_panel",
        elem_b="tapered_body",
        name="closed_lid_skirt_sits_around_rim",
    )

    closed_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_joint: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)
        body_aabb = ctx.part_world_aabb(body)
        open_pos = ctx.part_world_position(lid)
        ctx.check(
            "lid_opens_up_from_front_edge",
            open_aabb is not None
            and body_aabb is not None
            and open_pos is not None
            and closed_pos is not None
            and open_aabb[1][2] > body_aabb[1][2] + 0.18,
            details=f"open_aabb={open_aabb}, body_aabb={body_aabb}, closed={closed_pos}, open={open_pos}",
        )

    ctx.expect_origin_gap(wheel_1, wheel_0, axis="y", min_gap=2.0 * WHEEL_Y - 0.002, max_gap=2.0 * WHEEL_Y + 0.002)
    ctx.expect_origin_gap(wheel_0, body, axis="z", min_gap=0.0, max_gap=AXLE_Z + 0.002)

    return ctx.report()


object_model = build_object_model()
