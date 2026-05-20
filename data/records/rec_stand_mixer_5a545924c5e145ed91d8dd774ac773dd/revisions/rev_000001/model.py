from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _safe_fillet(shape: cq.Workplane, radius: float, selector: str | None = None) -> cq.Workplane:
    try:
        return (shape.edges(selector).fillet(radius) if selector else shape.edges().fillet(radius))
    except Exception:
        return shape


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _flared_base_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .rect(0.76, 0.46)
        .workplane(offset=0.055)
        .rect(0.68, 0.39)
        .workplane(offset=0.065)
        .rect(0.56, 0.31)
        .loft(combine=True)
    )
    shell = _safe_fillet(shell, 0.026, "|Z")
    shell = _safe_fillet(shell, 0.014)

    column = _safe_fillet(_cq_box((0.18, 0.28, 0.52), (-0.25, 0.0, 0.36)), 0.025, "|Z")
    neck = _safe_fillet(_cq_box((0.22, 0.30, 0.075), (-0.23, 0.0, 0.585)), 0.018, "|Z")

    base = shell.union(column).union(neck)

    for y in (-0.15, 0.15):
        lug = _safe_fillet(_cq_box((0.10, 0.055, 0.13), (-0.25, y, 0.66)), 0.010)
        hole = (
            cq.Workplane("XY")
            .cylinder(0.080, 0.038)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((-0.25, y, 0.66))
        )
        base = base.union(lug.cut(hole))

    return base


def _bowl_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.083)
        .workplane(offset=0.060)
        .circle(0.112)
        .workplane(offset=0.170)
        .circle(0.178)
        .workplane(offset=0.090)
        .circle(0.202)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.030)
        .circle(0.052)
        .workplane(offset=0.060)
        .circle(0.092)
        .workplane(offset=0.155)
        .circle(0.157)
        .workplane(offset=0.080)
        .circle(0.183)
        .workplane(offset=0.040)
        .circle(0.190)
        .loft(combine=True)
    )
    bowl = outer.cut(inner)

    rim_outer = cq.Workplane("XY").cylinder(0.018, 0.212).translate((0.0, 0.0, 0.320))
    rim_inner = cq.Workplane("XY").cylinder(0.024, 0.181).translate((0.0, 0.0, 0.320))
    foot_outer = cq.Workplane("XY").cylinder(0.026, 0.096).translate((0.0, 0.0, 0.013))
    foot_inner = cq.Workplane("XY").cylinder(0.032, 0.058).translate((0.0, 0.0, 0.013))

    bowl = bowl.union(rim_outer.cut(rim_inner)).union(foot_outer.cut(foot_inner))
    return _safe_fillet(bowl, 0.004)


def _cradle_socket() -> cq.Workplane:
    ring_outer = cq.Workplane("XY").cylinder(0.026, 0.128).translate((0.0, 0.0, 0.044))
    ring_inner = cq.Workplane("XY").cylinder(0.034, 0.090).translate((0.0, 0.0, 0.044))
    yoke = ring_outer.cut(ring_inner)
    bridge = _safe_fillet(_cq_box((0.240, 0.060, 0.026), (0.0, -0.095, 0.030)), 0.008)
    return yoke.union(bridge)


def _head_shell() -> cq.Workplane:
    rear_hub = _safe_fillet(_cq_box((0.15, 0.20, 0.12), (0.055, 0.0, 0.055)), 0.026)
    main = _safe_fillet(_cq_box((0.62, 0.28, 0.17), (0.36, 0.0, 0.095)), 0.045)
    nose = _safe_fillet(_cq_box((0.20, 0.22, 0.13), (0.62, 0.0, 0.060)), 0.040)
    shell = rear_hub.union(main).union(nose)
    drive = cq.Workplane("XY").cylinder(0.120, 0.050).translate((0.45, 0.0, -0.048))
    return shell.union(drive)


def _paddle_attachment() -> cq.Workplane:
    shaft = cq.Workplane("XY").cylinder(0.160, 0.012).translate((0.0, 0.0, -0.080))
    hub = cq.Workplane("XY").cylinder(0.035, 0.030).translate((0.0, 0.0, -0.0175))
    upper_bar = _safe_fillet(_cq_box((0.150, 0.024, 0.018), (0.0, 0.0, -0.130)), 0.006)
    lower_bar = _safe_fillet(_cq_box((0.105, 0.022, 0.018), (0.0, 0.0, -0.205)), 0.006)
    left_side = _safe_fillet(_cq_box((0.024, 0.022, 0.095), (-0.063, 0.0, -0.168)), 0.006)
    right_side = _safe_fillet(_cq_box((0.024, 0.022, 0.095), (0.063, 0.0, -0.168)), 0.006)
    center_blade = _safe_fillet(_cq_box((0.034, 0.018, 0.090), (0.0, 0.0, -0.165)), 0.005)
    return shaft.union(hub).union(upper_bar).union(lower_bar).union(left_side).union(right_side).union(center_blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_end_stand_mixer")

    enamel = Material("warm_ivory_enamel", rgba=(0.86, 0.80, 0.67, 1.0))
    dark_enamel = Material("charcoal_enamel", rgba=(0.05, 0.055, 0.060, 1.0))
    stainless = Material("brushed_stainless", rgba=(0.72, 0.72, 0.70, 1.0))
    chrome = Material("polished_chrome", rgba=(0.92, 0.90, 0.86, 1.0))
    black = Material("black_bakelite", rgba=(0.015, 0.014, 0.013, 1.0))
    red = Material("red_lock_marker", rgba=(0.72, 0.04, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_flared_base_shell(), "flared_base_shell", tolerance=0.0015),
        material=enamel,
        name="flared_base_shell",
    )
    base.visual(
        Box((0.380, 0.034, 0.025)),
        origin=Origin(xyz=(0.17, -0.125, 0.1225)),
        material=chrome,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.380, 0.034, 0.025)),
        origin=Origin(xyz=(0.17, 0.125, 0.1225)),
        material=chrome,
        name="slide_rail_1",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(-0.25, -0.145, 0.270), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="speed_boss",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(xyz=(-0.150, 0.105, 0.430), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lock_boss",
    )

    cradle = model.part("bowl_cradle")
    cradle.visual(
        Box((0.360, 0.280, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_enamel,
        name="slider_plate",
    )
    cradle.visual(
        mesh_from_cadquery(_cradle_socket(), "bowl_socket", tolerance=0.0012),
        material=chrome,
        name="bowl_socket",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell(), "deep_bowl_shell", tolerance=0.0012),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.105, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=stainless,
        name="bowl_foot_pad",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell(), "long_head_shell", tolerance=0.0015),
        material=enamel,
        name="long_head_shell",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.185),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.355),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )
    head.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.0, -0.1805, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_cap_0",
    )
    head.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.0, 0.1805, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_cap_1",
    )
    head.visual(
        Box((0.028, 0.286, 0.145)),
        origin=Origin(xyz=(0.145, 0.0, 0.040)),
        material=chrome,
        name="trim_band",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.45, 0.0, -0.103)),
        material=chrome,
        name="drive_socket_face",
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_paddle_attachment(), "flat_paddle_attachment", tolerance=0.0009),
        material=chrome,
        name="paddle_frame",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.0, -0.0225, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_disk",
    )
    speed_knob.visual(
        Box((0.014, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, -0.049, 0.030)),
        material=chrome,
        name="knob_pointer",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(xyz=(0.0225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="plunger_stem",
    )
    head_lock.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="plunger_cap",
    )

    model.articulation(
        "base_to_cradle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.17, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.12, lower=-0.030, upper=0.030),
    )
    model.articulation(
        "cradle_to_bowl",
        ArticulationType.FIXED,
        parent=cradle,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.25, 0.0, 0.660)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=0.72),
    )
    model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.45, 0.0, -0.103)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=20.0),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(-0.25, -0.155, 0.270)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.139, 0.105, 0.430)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.024),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cradle = object_model.get_part("bowl_cradle")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    paddle = object_model.get_part("paddle")
    speed_knob = object_model.get_part("speed_knob")
    head_lock = object_model.get_part("head_lock")

    slide = object_model.get_articulation("base_to_cradle")
    head_hinge = object_model.get_articulation("base_to_head")
    paddle_drive = object_model.get_articulation("head_to_paddle")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.allow_overlap(
        head,
        paddle,
        elem_a="drive_socket_face",
        elem_b="paddle_frame",
        reason="The paddle's keyed shaft is intentionally captured slightly inside the mixer drive socket.",
    )
    ctx.allow_overlap(
        head,
        paddle,
        elem_a="long_head_shell",
        elem_b="paddle_frame",
        reason="The paddle's keyed shaft runs up into the modeled drive boss so the attachment is physically captured.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="flared_base_shell",
        elem_b="hinge_cap_0",
        reason="The hinge end cap is intentionally seated into the rear lug with a small hidden press fit.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="flared_base_shell",
        elem_b="hinge_cap_1",
        reason="The opposite hinge end cap is intentionally seated into the rear lug with a small hidden press fit.",
    )
    ctx.expect_gap(
        head,
        paddle,
        axis="z",
        positive_elem="drive_socket_face",
        negative_elem="paddle_frame",
        max_gap=0.004,
        max_penetration=0.010,
        name="paddle shaft is seated in the drive socket",
    )

    ctx.expect_gap(
        cradle,
        base,
        axis="z",
        positive_elem="slider_plate",
        negative_elem="slide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="cradle rides on the short base slide rail",
    )
    ctx.expect_overlap(
        cradle,
        base,
        axes="xy",
        elem_a="slider_plate",
        elem_b="slide_rail_0",
        min_overlap=0.020,
        name="cradle remains seated over a rail",
    )
    ctx.expect_gap(
        bowl,
        cradle,
        axis="z",
        positive_elem="bowl_foot_pad",
        negative_elem="bowl_socket",
        max_gap=0.002,
        max_penetration=0.0,
        name="bowl foot rests on its cradle socket",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="xz",
        elem_a="hinge_cap_0",
        elem_b="flared_base_shell",
        min_overlap=0.020,
        name="rear hinge cap is seated in its support lug",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="xz",
        elem_a="hinge_cap_1",
        elem_b="flared_base_shell",
        min_overlap=0.020,
        name="opposite rear hinge cap is seated in its support lug",
    )
    ctx.expect_overlap(
        head,
        paddle,
        axes="xy",
        elem_a="long_head_shell",
        elem_b="paddle_frame",
        min_overlap=0.030,
        name="paddle shaft aligns inside the drive boss",
    )
    ctx.expect_within(
        paddle,
        bowl,
        axes="xy",
        inner_elem="paddle_frame",
        outer_elem="bowl_shell",
        margin=0.006,
        name="paddle sits within the bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="long_head_shell",
        negative_elem="bowl_shell",
        min_gap=0.020,
        name="mixer head clears the bowl rim",
    )
    ctx.expect_contact(
        speed_knob,
        base,
        elem_a="knob_disk",
        elem_b="speed_boss",
        contact_tol=0.0015,
        name="speed knob is mounted on the base boss",
    )
    ctx.expect_contact(
        head_lock,
        base,
        elem_a="plunger_stem",
        elem_b="lock_boss",
        contact_tol=0.0015,
        name="head lock plunger is mounted in its base boss",
    )

    rest_cradle_pos = ctx.part_world_position(cradle)
    with ctx.pose({slide: 0.030}):
        extended_cradle_pos = ctx.part_world_position(cradle)
        ctx.expect_overlap(
            cradle,
            base,
            axes="x",
            elem_a="slider_plate",
            elem_b="slide_rail_0",
            min_overlap=0.250,
            name="bowl slide keeps substantial rail engagement",
        )
    ctx.check(
        "bowl cradle has short forward prismatic travel",
        rest_cradle_pos is not None
        and extended_cradle_pos is not None
        and extended_cradle_pos[0] > rest_cradle_pos[0] + 0.025,
        details=f"rest={rest_cradle_pos}, extended={extended_cradle_pos}",
    )

    rest_paddle_pos = ctx.part_world_position(paddle)
    with ctx.pose({head_hinge: 0.60}):
        raised_paddle_pos = ctx.part_world_position(paddle)
    ctx.check(
        "rear horizontal hinge lifts the nose and paddle",
        rest_paddle_pos is not None
        and raised_paddle_pos is not None
        and raised_paddle_pos[2] > rest_paddle_pos[2] + 0.12,
        details=f"rest={rest_paddle_pos}, raised={raised_paddle_pos}",
    )

    rest_lock_pos = ctx.part_world_position(head_lock)
    with ctx.pose({lock_slide: 0.020}):
        extended_lock_pos = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock plunger slides out from the base",
        rest_lock_pos is not None
        and extended_lock_pos is not None
        and extended_lock_pos[0] > rest_lock_pos[0] + 0.015,
        details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
    )

    rest_knob_aabb = ctx.part_world_aabb(speed_knob)
    with ctx.pose({knob_joint: 1.2, paddle_drive: math.pi / 2.0}):
        turned_knob_aabb = ctx.part_world_aabb(speed_knob)
        spun_paddle_pos = ctx.part_world_position(paddle)
    ctx.check(
        "speed knob rotates on the side boss",
        rest_knob_aabb is not None
        and turned_knob_aabb is not None
        and abs(rest_knob_aabb[0][2] - turned_knob_aabb[0][2]) > 0.006,
        details=f"rest_aabb={rest_knob_aabb}, turned_aabb={turned_knob_aabb}",
    )
    ctx.check(
        "paddle drive spins without shifting its vertical drive origin",
        rest_paddle_pos is not None
        and spun_paddle_pos is not None
        and abs(spun_paddle_pos[0] - rest_paddle_pos[0]) < 0.002
        and abs(spun_paddle_pos[1] - rest_paddle_pos[1]) < 0.002
        and abs(spun_paddle_pos[2] - rest_paddle_pos[2]) < 0.002,
        details=f"rest={rest_paddle_pos}, spun={spun_paddle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
