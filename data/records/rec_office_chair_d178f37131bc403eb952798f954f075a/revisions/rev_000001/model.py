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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges().fillet(radius)
    return shape.translate(center)


def _tube(outer_radius: float, inner_radius: float, z_min: float, z_max: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _make_base_frame() -> cq.Workplane:
    """One connected five-star pedestal frame with a clear center bore."""

    base = _tube(0.092, 0.052, 0.080, 0.150)
    arm_len = 0.430
    arm_center = 0.282
    for i in range(5):
        angle = i * 72.0
        arm = _rounded_box((arm_len, 0.060, 0.034), (arm_center, 0.0, 0.116), 0.012)
        end_boss = cq.Workplane("XY").circle(0.046).extrude(0.028).translate((0.500, 0.0, 0.091))
        spoke = arm.union(end_boss)
        spoke = spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        base = base.union(spoke)
    return base


def _make_seat_support() -> cq.Workplane:
    plate = _rounded_box((0.420, 0.285, 0.040), (-0.075, 0.0, 0.035), 0.008)
    socket = _tube(0.060, 0.038, -0.080, 0.030)
    bridge = _rounded_box((0.150, 0.330, 0.050), (-0.275, 0.0, 0.060), 0.006)
    yoke_0 = _rounded_box((0.090, 0.035, 0.140), (-0.305, 0.170, 0.115), 0.006)
    yoke_1 = _rounded_box((0.090, 0.035, 0.140), (-0.305, -0.170, 0.115), 0.006)
    return plate.union(socket).union(bridge).union(yoke_0).union(yoke_1)


def _make_backrest() -> cq.Workplane:
    panel = _rounded_box((0.060, 0.430, 0.500), (-0.050, 0.0, 0.380), 0.0)
    lower_tab = _rounded_box((0.050, 0.205, 0.170), (-0.020, 0.0, 0.075), 0.0)
    lumbar_ridge = _rounded_box((0.026, 0.300, 0.120), (-0.087, 0.0, 0.310), 0.0)
    return panel.union(lower_tab).union(lumbar_ridge)


def _make_caster_wheel() -> cq.Workplane:
    """A single connected caster wheel disk; the axle is represented as captured through it."""

    wheel = cq.Workplane("YZ").circle(0.033).extrude(0.030).translate((-0.015, 0.0, 0.0))
    side_hub = cq.Workplane("YZ").circle(0.016).extrude(0.038).translate((-0.019, 0.0, 0.0))
    return wheel.union(side_hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="armless_task_chair")

    fabric = Material("charcoal_fabric", color=(0.075, 0.080, 0.085, 1.0))
    black = Material("satin_black_plastic", color=(0.015, 0.015, 0.014, 1.0))
    rubber = Material("matte_black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    chrome = Material("polished_chrome", color=(0.78, 0.78, 0.74, 1.0))
    dark_metal = Material("dark_steel", color=(0.12, 0.125, 0.13, 1.0))
    hub_gray = Material("caster_gray_hub", color=(0.42, 0.43, 0.42, 1.0))

    base = model.part("five_star_base")
    base.visual(mesh_from_cadquery(_make_base_frame(), "five_star_base_frame"), material=black, name="star_frame")
    base.visual(mesh_from_cadquery(_tube(0.070, 0.047, 0.105, 0.285), "base_lower_socket"), material=black, name="lower_socket")

    gas = model.part("gas_column")
    gas.visual(Cylinder(radius=0.025, length=0.340), origin=Origin(xyz=(0.0, 0.0, 0.080)), material=chrome, name="piston")
    gas.visual(Cylinder(radius=0.049, length=0.180), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=dark_metal, name="outer_shroud")
    gas.visual(Cylinder(radius=0.040, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.235)), material=chrome, name="top_plug")

    seat = model.part("seat")
    seat.visual(mesh_from_cadquery(_make_seat_support(), "seat_mechanism"), material=dark_metal, name="mechanism")
    seat.visual(mesh_from_cadquery(_rounded_box((0.500, 0.460, 0.080), (0.055, 0.0, 0.095), 0.035), "seat_cushion"), material=fabric, name="cushion")

    back = model.part("backrest")
    back.visual(mesh_from_cadquery(_make_backrest(), "backrest_shell"), material=fabric, name="back_shell")
    back.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )

    gas_slide = model.articulation(
        "gas_height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gas,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.18, lower=0.0, upper=0.120),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=gas,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(-0.305, 0.0, 0.140)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.12, upper=0.48),
    )

    caster_radius = 0.500
    pivot_z = 0.090
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        fork = model.part(f"caster_fork_{i}")
        fork.visual(Cylinder(radius=0.011, length=0.055), origin=Origin(xyz=(0.0, 0.0, -0.020)), material=dark_metal, name="swivel_stem")
        fork.visual(Cylinder(radius=0.025, length=0.008), origin=Origin(xyz=(0.0, 0.0, -0.004)), material=black, name="top_washer")
        fork.visual(Box((0.032, 0.060, 0.030)), origin=Origin(xyz=(0.0, -0.035, -0.020)), material=black, name="neck")
        fork.visual(Box((0.009, 0.030, 0.070)), origin=Origin(xyz=(0.027, -0.070, -0.075)), material=black, name="fork_cheek_0")
        fork.visual(Box((0.009, 0.030, 0.070)), origin=Origin(xyz=(-0.027, -0.070, -0.075)), material=black, name="fork_cheek_1")
        fork.visual(Box((0.062, 0.018, 0.012)), origin=Origin(xyz=(0.0, -0.070, -0.037)), material=black, name="crown_bridge")
        fork.visual(
            Cylinder(radius=0.0045, length=0.064),
            origin=Origin(xyz=(0.0, -0.070, -0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="axle",
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(mesh_from_cadquery(_make_caster_wheel(), f"caster_wheel_{i}_body"), material=rubber, name="wheel_body")

        x = caster_radius * math.cos(angle)
        y = caster_radius * math.sin(angle)
        yaw = angle + math.pi / 2.0
        model.articulation(
            f"caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=(x, y, pivot_z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=5.0),
        )
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.070, -0.075)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    # Named local samples make the compiler's pose sampler exercise the retained gas-column insertion.
    gas_slide.meta["qc_samples"] = [0.0, 0.060, 0.120]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("five_star_base")
    gas = object_model.get_part("gas_column")
    seat = object_model.get_part("seat")
    back = object_model.get_part("backrest")
    gas_slide = object_model.get_articulation("gas_height_slide")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_recline = object_model.get_articulation("back_recline")

    ctx.allow_overlap(
        base,
        gas,
        elem_a="lower_socket",
        elem_b="outer_shroud",
        reason="The lower gas-cylinder shroud is intentionally clipped into the pedestal socket.",
    )
    ctx.allow_overlap(
        gas,
        seat,
        elem_a="top_plug",
        elem_b="mechanism",
        reason="The upper gas-column plug is intentionally captured inside the seat support socket.",
    )
    ctx.allow_overlap(
        gas,
        seat,
        elem_a="piston",
        elem_b="mechanism",
        reason="The polished lift piston intentionally continues through the seat mechanism socket.",
    )
    ctx.allow_overlap(
        seat,
        back,
        elem_a="mechanism",
        elem_b="pivot_barrel",
        reason="The recline pivot barrel is intentionally captured in the rear support yoke.",
    )

    ctx.expect_within(
        gas,
        base,
        axes="xy",
        inner_elem="outer_shroud",
        outer_elem="lower_socket",
        margin=0.002,
        name="gas shroud stays centered in lower socket",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="outer_shroud",
        elem_b="lower_socket",
        min_overlap=0.060,
        name="gas column remains clipped into the five-star base",
    )
    ctx.expect_within(
        gas,
        seat,
        axes="xy",
        inner_elem="top_plug",
        outer_elem="mechanism",
        margin=0.002,
        name="gas column top remains captured by seat mechanism",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="top_plug",
        elem_b="mechanism",
        min_overlap=0.025,
        name="column top is inserted into the seat support",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="piston",
        elem_b="mechanism",
        min_overlap=0.080,
        name="lift piston continues through the seat socket",
    )

    rest_seat = ctx.part_world_position(seat)
    with ctx.pose({gas_slide: 0.120, seat_swivel: math.pi / 2.0}):
        ctx.expect_overlap(
            gas,
            base,
            axes="z",
            elem_a="outer_shroud",
            elem_b="lower_socket",
            min_overlap=0.055,
            name="raised gas column still retained in base socket",
        )
        ctx.expect_overlap(
            gas,
            seat,
            axes="z",
            elem_a="top_plug",
            elem_b="mechanism",
            min_overlap=0.025,
            name="raised swivel seat stays clipped to gas column",
        )
        raised_seat = ctx.part_world_position(seat)
    ctx.check(
        "height slide raises the carried seat",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.10,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    rest_back = ctx.part_world_position(back)
    rest_back_aabb = ctx.part_world_aabb(back)
    with ctx.pose({back_recline: 0.45}):
        reclined_back_aabb = ctx.part_world_aabb(back)
        ctx.expect_gap(
            seat,
            back,
            axis="x",
            positive_elem="cushion",
            negative_elem="back_shell",
            max_penetration=0.0,
            name="reclined back cushion remains behind the seat cushion",
        )
        ctx.expect_overlap(
            back,
            seat,
            axes="y",
            elem_a="pivot_barrel",
            elem_b="mechanism",
            min_overlap=0.28,
            name="backrest pivot spans the rear yoke",
        )
        ctx.expect_overlap(
            back,
            seat,
            axes="z",
            elem_a="pivot_barrel",
            elem_b="mechanism",
            min_overlap=0.030,
            name="backrest pivot is vertically captured by the yoke",
        )
    ctx.check(
        "backrest reclines about rear horizontal pivot",
        rest_back is not None
        and rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.150,
        details=f"rest_pos={rest_back}, rest_aabb={rest_back_aabb}, reclined_aabb={reclined_back_aabb}",
    )

    for i in range(5):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle",
            elem_b="wheel_body",
            reason="The caster axle is intentionally represented as passing through the wheel hub.",
        )
        ctx.allow_overlap(
            base,
            fork,
            elem_a="star_frame",
            elem_b="swivel_stem",
            reason="The caster swivel stem is intentionally captured in the star-base end boss.",
        )
        ctx.expect_overlap(
            wheel,
            fork,
            axes="xyz",
            elem_a="wheel_body",
            elem_b="axle",
            min_overlap=0.008,
            name=f"caster wheel {i} rides on its axle",
        )

    return ctx.report()


object_model = build_object_model()
