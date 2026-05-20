from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def _build_body_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.34, 0.22, 0.045)
        .translate((0.0, 0.0, 0.0225))
        .edges("|Z")
        .fillet(0.018)
    )

    rear_column = (
        cq.Workplane("XY")
        .box(0.082, 0.100, 0.280)
        .translate((-0.125, 0.0, 0.185))
        .edges("|Z")
        .fillet(0.020)
    )

    sloped_neck = (
        cq.Workplane("XY")
        .box(0.120, 0.088, 0.075)
        .translate((-0.100, 0.0, 0.095))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 22.0)
    )

    bowl_plinth = (
        cq.Workplane("XY")
        .circle(0.078)
        .extrude(0.015)
        .translate((0.100, 0.0, 0.045))
    )

    bowl_collar = (
        cq.Workplane("XY")
        .circle(0.066)
        .circle(0.053)
        .extrude(0.010)
        .translate((0.100, 0.0, 0.060))
    )

    hinge_support_0 = (
        cq.Workplane("XY")
        .box(0.022, 0.024, 0.040)
        .translate((-0.086, 0.059, 0.308))
    )
    hinge_support_1 = (
        cq.Workplane("XY")
        .box(0.022, 0.024, 0.040)
        .translate((-0.086, -0.059, 0.308))
    )

    hinge_ear_0 = _cylinder_y(0.012, 0.022, (-0.065, 0.059, 0.314))
    hinge_ear_1 = _cylinder_y(0.012, 0.022, (-0.065, -0.059, 0.314))

    return (
        base.union(rear_column)
        .union(sloped_neck)
        .union(bowl_plinth)
        .union(bowl_collar)
        .union(hinge_support_0)
        .union(hinge_support_1)
        .union(hinge_ear_0)
        .union(hinge_ear_1)
    )


def _build_head_shape() -> cq.Workplane:
    hinge_barrel = _cylinder_y(0.011, 0.096, (0.0, 0.0, 0.0))

    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.050, 0.054, 0.060)
        .translate((0.034, 0.0, 0.008))
        .edges("|Z")
        .fillet(0.010)
    )

    rear_motor = (
        cq.Workplane("YZ")
        .ellipse(0.056, 0.066)
        .extrude(0.170)
        .translate((0.030, 0.0, 0.016))
    )

    front_nose = (
        cq.Workplane("XY")
        .box(0.135, 0.094, 0.078)
        .translate((0.210, 0.0, -0.008))
        .edges("|Z")
        .fillet(0.016)
    )

    crown = (
        cq.Workplane("XY")
        .box(0.125, 0.082, 0.040)
        .translate((0.190, 0.0, 0.030))
        .edges("|Z")
        .fillet(0.010)
    )

    drive_boss = (
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(0.060)
        .translate((0.170, 0.0, -0.100))
    )

    head_shape = (
        hinge_barrel.union(hinge_bridge)
        .union(rear_motor)
        .union(front_nose)
        .union(crown)
        .union(drive_boss)
    )

    slot_cut = (
        cq.Workplane("XY")
        .box(0.042, 0.024, 0.010)
        .translate((0.100, 0.058, 0.010))
    )
    recess_cut = (
        cq.Workplane("XY")
        .box(0.058, 0.010, 0.022)
        .translate((0.100, 0.052, 0.012))
    )
    return head_shape.cut(slot_cut).cut(recess_cut)


def _build_beater_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.007).extrude(0.045).translate((0.0, 0.0, -0.045))
    collar = cq.Workplane("XY").circle(0.011).extrude(0.018).translate((0.0, 0.0, -0.018))

    frame_outer = (
        cq.Workplane("XZ")
        .rect(0.067, 0.086)
        .extrude(0.010)
        .translate((0.0, -0.005, -0.087))
    )
    frame_inner = (
        cq.Workplane("XZ")
        .rect(0.045, 0.060)
        .extrude(0.014)
        .translate((0.0, -0.007, -0.087))
    )
    frame_bar = (
        cq.Workplane("XZ")
        .rect(0.046, 0.014)
        .extrude(0.010)
        .translate((0.0, -0.005, -0.066))
    )

    return shaft.union(collar).union(frame_outer.cut(frame_inner)).union(frame_bar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    enamel = model.material("enamel", rgba=(0.82, 0.13, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=enamel,
        name="body_shell",
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.044, 0.000),
            (0.054, 0.008),
            (0.086, 0.024),
            (0.110, 0.072),
            (0.116, 0.146),
            (0.118, 0.154),
        ],
        [
            (0.000, 0.005),
            (0.048, 0.014),
            (0.080, 0.028),
            (0.101, 0.072),
            (0.106, 0.143),
            (0.108, 0.148),
        ],
        segments=56,
    )
    bowl.visual(mesh_from_geometry(bowl_shell, "bowl_shell"), material=steel, name="bowl_shell")
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        bowl.visual(
            Box((0.018, 0.010, 0.004)),
            origin=Origin(
                xyz=(0.053 * math.cos(angle), 0.053 * math.sin(angle), 0.007),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"bowl_lug_{idx}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shape(), "head_shell"),
        material=enamel,
        name="head_shell",
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Box((0.030, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=dark_trim,
        name="lever_body",
    )
    speed_lever.visual(
        Box((0.010, 0.018, 0.018)),
        origin=Origin(xyz=(0.008, 0.013, 0.004)),
        material=dark_trim,
        name="lever_grip",
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_cadquery(_build_beater_shape(), "beater_frame"),
        material=steel,
        name="beater_frame",
    )

    model.articulation(
        "body_to_bowl",
        ArticulationType.FIXED,
        parent=body,
        child=bowl,
        origin=Origin(xyz=(0.100, 0.0, 0.060)),
    )

    model.articulation(
        "body_to_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(-0.055, 0.0, 0.314)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(63.0),
        ),
    )

    model.articulation(
        "head_to_speed_lever",
        ArticulationType.PRISMATIC,
        parent=head,
        child=speed_lever,
        origin=Origin(xyz=(0.085, 0.053, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=-0.014,
            upper=0.014,
        ),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.155, 0.0, -0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_lever = object_model.get_part("speed_lever")

    head_tilt = object_model.get_articulation("body_to_head")
    lever_slide = object_model.get_articulation("head_to_speed_lever")

    ctx.allow_overlap(
        body,
        bowl,
        elem_a="body_shell",
        elem_b="bowl_shell",
        reason="The low bayonet plinth sits inside the bowl's simplified closed-shell proxy rather than a separate collision cavity.",
    )
    ctx.allow_overlap(
        beater,
        bowl,
        elem_a="beater_frame",
        elem_b="bowl_shell",
        reason="The beater works inside the mixing bowl, which is represented as a closed shell mesh rather than an open collision cavity.",
    )
    ctx.allow_overlap(
        body,
        head,
        elem_a="body_shell",
        elem_b="head_shell",
        reason="The rear hinge is represented as a compact nested barrel-and-yoke pivot, so the closed-pose hinge geometry shares the same local envelope.",
    )

    ctx.expect_overlap(
        bowl,
        body,
        axes="xy",
        min_overlap=0.10,
        name="bowl stays centered over the mixer base",
    )
    ctx.expect_within(
        beater,
        bowl,
        axes="xy",
        margin=0.03,
        name="beater stays within the bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        max_penetration=0.0,
        max_gap=0.030,
        name="closed head clears the bowl rim",
    )

    closed_beater_pos = ctx.part_world_position(beater)
    with ctx.pose({head_tilt: math.radians(63.0)}):
        opened_beater_pos = ctx.part_world_position(beater)

    ctx.check(
        "head tilt lifts the beater upward",
        closed_beater_pos is not None
        and opened_beater_pos is not None
        and opened_beater_pos[2] > closed_beater_pos[2] + 0.10,
        details=f"closed={closed_beater_pos}, opened={opened_beater_pos}",
    )

    lower = lever_slide.motion_limits.lower if lever_slide.motion_limits is not None else -0.014
    upper = lever_slide.motion_limits.upper if lever_slide.motion_limits is not None else 0.014
    with ctx.pose({lever_slide: lower}):
        rear_lever_pos = ctx.part_world_position(speed_lever)
    with ctx.pose({lever_slide: upper}):
        front_lever_pos = ctx.part_world_position(speed_lever)

    ctx.check(
        "speed lever slides forward along its slot",
        rear_lever_pos is not None
        and front_lever_pos is not None
        and front_lever_pos[0] > rear_lever_pos[0] + 0.020,
        details=f"rear={rear_lever_pos}, front={front_lever_pos}",
    )

    return ctx.report()


object_model = build_object_model()
