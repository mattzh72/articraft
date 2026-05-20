from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


HANDLE_L = 0.180
HANDLE_W = 0.028
HANDLE_H = 0.038
CARRIER_TRAVEL = 0.035


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _side_plate_shape(y_center: float) -> cq.Workplane:
    # A heavy utility-knife handle is a hollow metal/plastic shell: two shaped
    # side plates tied by webs with a straight guide running down the middle.
    # The central channel is deliberately open so the separate blade carrier can
    # slide without occupying solid shell volume.
    side_profile = [
        (-0.092, -0.017),
        (0.052, -0.017),
        (0.090, -0.006),
        (0.088, 0.010),
        (0.036, 0.020),
        (-0.074, 0.018),
        (-0.094, 0.004),
    ]

    thickness = 0.004
    return (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(thickness)
        .translate((0.0, y_center + thickness / 2.0, 0.0))
    )


def _blade_shape() -> cq.Workplane:
    blade_profile = [
        (0.074, -0.006),
        (0.115, -0.006),
        (0.132, 0.000),
        (0.115, 0.010),
        (0.074, 0.010),
    ]
    blade = (
        cq.Workplane("XZ")
        .polyline(blade_profile)
        .close()
        .extrude(0.002)
        .translate((0.0, -0.001, 0.0))
    )
    return blade


def _hatch_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.052, 0.0024, 0.024).translate((0.029, 0.0014, 0.0))
    inset = cq.Workplane("XY").box(0.040, 0.0010, 0.015).translate((0.031, 0.0030, 0.0))
    rib_0 = cq.Workplane("XY").box(0.030, 0.0012, 0.0016).translate((0.032, 0.0036, -0.0045))
    rib_1 = cq.Workplane("XY").box(0.030, 0.0012, 0.0016).translate((0.032, 0.0036, 0.0045))
    hinge_leaf = cq.Workplane("XY").box(0.007, 0.0020, 0.024).translate((0.003, 0.0010, 0.0))
    return panel.union(inset).union(rib_0).union(rib_1).union(hinge_leaf)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_utility_knife")

    model.material("gunmetal_shell", rgba=(0.28, 0.30, 0.32, 1.0))
    model.material("black_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("safety_orange", rgba=(0.95, 0.36, 0.04, 1.0))
    model.material("dark_cavity", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("brushed_steel", rgba=(0.74, 0.76, 0.76, 1.0))

    shell = model.part("handle_shell")
    shell.visual(
        mesh_from_cadquery(_side_plate_shape(-0.012), "side_plate_negative"),
        material="gunmetal_shell",
        name="side_plate_0",
    )
    shell.visual(
        mesh_from_cadquery(_side_plate_shape(0.012), "side_plate_positive"),
        material="gunmetal_shell",
        name="side_plate_1",
    )
    shell.visual(
        Box((0.162, HANDLE_W, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, -0.015)),
        material="gunmetal_shell",
        name="bottom_web",
    )
    shell.visual(
        Box((0.028, HANDLE_W, 0.012)),
        origin=Origin(xyz=(-0.080, 0.0, -0.002)),
        material="gunmetal_shell",
        name="rear_web",
    )
    shell.visual(
        Box((0.030, HANDLE_W, 0.006)),
        origin=Origin(xyz=(0.071, 0.0, -0.013)),
        material="gunmetal_shell",
        name="front_web",
    )
    shell.visual(
        Box((0.145, 0.0041, 0.004)),
        origin=Origin(xyz=(0.005, 0.00795, 0.002)),
        material="gunmetal_shell",
        name="guide_strip_0",
    )
    shell.visual(
        Box((0.145, 0.0041, 0.004)),
        origin=Origin(xyz=(0.005, -0.00795, 0.002)),
        material="gunmetal_shell",
        name="guide_strip_1",
    )
    # Rubber scale inserts are lightly proud/embedded so they read as molded grip
    # panels but remain physically part of the shell.
    shell.visual(
        Box((0.100, 0.0020, 0.020)),
        origin=Origin(xyz=(-0.017, -0.0148, -0.001)),
        material="black_rubber",
        name="grip_scale",
    )
    shell.visual(
        Box((0.038, 0.0016, 0.014)),
        origin=Origin(xyz=(0.050, -0.0146, 0.002)),
        material="black_rubber",
        name="front_grip_scale",
    )
    # Dark top rails frame the slider slot and make the straight internal guide
    # legible from above.
    shell.visual(
        Box((0.112, 0.0030, 0.0020)),
        origin=Origin(xyz=(0.002, -0.0120, 0.0178)),
        material="black_rubber",
        name="top_rail_0",
    )
    shell.visual(
        Box((0.112, 0.0030, 0.0020)),
        origin=Origin(xyz=(0.002, 0.0125, 0.0178)),
        material="black_rubber",
        name="top_rail_1",
    )
    # Spare blades visible in the side pocket when the hatch is opened.
    shell.visual(
        Box((0.044, 0.0010, 0.010)),
        origin=Origin(xyz=(-0.035, 0.0104, -0.001)),
        material="brushed_steel",
        name="spare_blades",
    )
    # Interleaved shell-side hinge knuckles and small bosses keep the side hatch
    # visibly clipped to the handle wall.
    for z, name in ((0.0090, "shell_hinge_upper"), (-0.0090, "shell_hinge_lower")):
        shell.visual(
            Cylinder(radius=0.0026, length=0.0076),
            origin=Origin(xyz=(-0.066, 0.0170, z)),
            material="gunmetal_shell",
            name=name,
        )
        shell.visual(
            Box((0.0065, 0.0040, 0.0076)),
            origin=Origin(xyz=(-0.066, 0.0150, z)),
            material="gunmetal_shell",
            name=f"{name}_boss",
        )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Box((0.090, 0.0120, 0.0060)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material="brushed_steel",
        name="carrier_rail",
    )
    carrier.visual(
        Box((0.012, 0.0090, 0.0090)),
        origin=Origin(xyz=(0.076, 0.0, 0.001)),
        material="brushed_steel",
        name="blade_clamp",
    )
    carrier.visual(
        mesh_from_cadquery(_blade_shape(), "utility_blade"),
        material="brushed_steel",
        name="blade",
    )

    top_slider = model.part("top_slider")
    top_slider.visual(
        Box((0.030, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="safety_orange",
        name="thumb_pad",
    )
    top_slider.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material="safety_orange",
        name="slider_stem",
    )
    for dx, name in ((-0.008, "slider_rib_0"), (0.0, "slider_rib_1"), (0.008, "slider_rib_2")):
        top_slider.visual(
            Box((0.0030, 0.020, 0.0020)),
            origin=Origin(xyz=(dx, 0.0, 0.0120)),
            material="black_rubber",
            name=name,
        )

    side_hatch = model.part("side_hatch")
    side_hatch.visual(
        mesh_from_cadquery(_hatch_panel_shape(), "side_hatch"),
        material="gunmetal_shell",
        name="hatch_panel",
    )
    side_hatch.visual(
        Cylinder(radius=0.00245, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="gunmetal_shell",
        name="hatch_hinge_barrel",
    )

    model.articulation(
        "carrier_slide",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=carrier,
        origin=Origin(xyz=(-0.055, 0.0, 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=CARRIER_TRAVEL, effort=70.0, velocity=0.22),
    )
    model.articulation(
        "slider_mount",
        ArticulationType.FIXED,
        parent=carrier,
        child=top_slider,
        origin=Origin(xyz=(0.055, 0.0, 0.018)),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=side_hatch,
        origin=Origin(xyz=(-0.066, 0.0170, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(82.0), effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("handle_shell")
    carrier = object_model.get_part("blade_carrier")
    slider = object_model.get_part("top_slider")
    hatch = object_model.get_part("side_hatch")
    carrier_slide = object_model.get_articulation("carrier_slide")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.expect_within(
        carrier,
        shell,
        axes="z",
        inner_elem="carrier_rail",
        outer_elem="guide_strip_0",
        margin=0.001,
        name="carrier rail height matches guide strip",
    )
    ctx.expect_gap(
        shell,
        carrier,
        axis="y",
        positive_elem="guide_strip_0",
        negative_elem="carrier_rail",
        max_gap=0.0003,
        max_penetration=0.0003,
        name="positive guide strip captures carrier side",
    )
    ctx.expect_gap(
        carrier,
        shell,
        axis="y",
        positive_elem="carrier_rail",
        negative_elem="guide_strip_1",
        max_gap=0.0003,
        max_penetration=0.0003,
        name="negative guide strip captures carrier side",
    )
    ctx.expect_overlap(
        carrier,
        shell,
        axes="x",
        elem_a="carrier_rail",
        elem_b="guide_strip_0",
        min_overlap=0.055,
        name="carrier remains retained in handle at rest",
    )
    ctx.expect_gap(
        slider,
        shell,
        axis="z",
        positive_elem="thumb_pad",
        negative_elem="top_rail_0",
        min_gap=0.001,
        max_gap=0.010,
        name="thumb pad rides above shell top",
    )

    slider_rest = ctx.part_world_position(slider)
    carrier_rest = ctx.part_world_position(carrier)
    with ctx.pose({carrier_slide: CARRIER_TRAVEL}):
        ctx.expect_within(
            carrier,
            shell,
            axes="z",
            inner_elem="carrier_rail",
            outer_elem="guide_strip_0",
            margin=0.001,
            name="extended carrier stays at guide height",
        )
        ctx.expect_overlap(
            carrier,
            shell,
            axes="x",
            elem_a="carrier_rail",
            elem_b="guide_strip_0",
            min_overlap=0.050,
            name="extended carrier still has retained insertion",
        )
        slider_extended = ctx.part_world_position(slider)
        carrier_extended = ctx.part_world_position(carrier)

    ctx.check(
        "carrier and top slider translate forward together",
        slider_rest is not None
        and slider_extended is not None
        and carrier_rest is not None
        and carrier_extended is not None
        and slider_extended[0] > slider_rest[0] + 0.030
        and abs((slider_extended[0] - slider_rest[0]) - (carrier_extended[0] - carrier_rest[0])) < 1e-6,
        details=(
            f"slider_rest={slider_rest}, slider_extended={slider_extended}, "
            f"carrier_rest={carrier_rest}, carrier_extended={carrier_extended}"
        ),
    )

    hatch_rest = ctx.part_world_position(hatch)
    with ctx.pose({hatch_hinge: math.radians(82.0)}):
        hatch_open = ctx.part_world_position(hatch)
        hatch_aabb = ctx.part_world_aabb(hatch)
    ctx.check(
        "side hatch hinge line stays clipped to shell",
        hatch_rest is not None
        and hatch_open is not None
        and abs(hatch_rest[0] - hatch_open[0]) < 1e-6
        and abs(hatch_rest[1] - hatch_open[1]) < 1e-6
        and abs(hatch_rest[2] - hatch_open[2]) < 1e-6,
        details=f"rest={hatch_rest}, open={hatch_open}",
    )
    ctx.check(
        "side hatch opens outward from side wall",
        hatch_aabb is not None and hatch_aabb[1][1] > 0.050,
        details=f"open_hatch_aabb={hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
