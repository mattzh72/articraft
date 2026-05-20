from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell(length: float, outer_radius: float, inner_radius: float, *, segments: int = 56):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, 0.0),
            (outer_radius, length),
        ],
        [
            (inner_radius, 0.0),
            (inner_radius, length),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_station_pump")

    painted_red = model.material("painted_red", rgba=(0.70, 0.12, 0.10, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.73, 0.77, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))
    gauge_gray = model.material("gauge_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    dial_white = model.material("dial_white", rgba=(0.95, 0.96, 0.94, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.94, 0.42, 0.10, 1.0))

    barrel_shell = _mesh(
        "pump_barrel_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.078, 0.0),
                (0.074, 0.020),
                (0.067, 0.055),
                (0.066, 0.540),
                (0.070, 0.585),
                (0.074, 0.610),
            ],
            [
                (0.058, 0.0),
                (0.055, 0.020),
                (0.052, 0.055),
                (0.052, 0.540),
                (0.054, 0.585),
                (0.058, 0.610),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    top_guide = _mesh("pump_top_guide", _ring_shell(0.060, 0.060, 0.0165))
    gauge_shell = _mesh(
        "pump_gauge_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.018, 0.0),
                (0.060, 0.004),
                (0.068, 0.020),
                (0.072, 0.046),
            ],
            [
                (0.0, 0.004),
                (0.055, 0.008),
                (0.060, 0.020),
                (0.060, 0.044),
            ],
            segments=56,
        ),
    )
    gauge_bezel = _mesh("pump_gauge_bezel", _ring_shell(0.008, 0.076, 0.060))
    grip_sleeve = _mesh("pump_grip_sleeve", _ring_shell(0.055, 0.029, 0.015))
    hose_geom = tube_from_spline_points(
        [
            (0.000, -0.085, 0.228),
            (0.000, -0.110, 0.188),
            (0.026, -0.112, 0.130),
            (0.062, -0.103, 0.086),
            (0.102, -0.084, 0.056),
            (0.136, -0.064, 0.050),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    hose_mesh = _mesh("pump_hose", hose_geom)

    base = model.part("base")
    base.visual(
        Box((0.340, 0.110, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_steel,
        name="bridge",
    )
    base.visual(
        Box((0.200, 0.170, 0.036)),
        origin=Origin(xyz=(-0.165, 0.0, 0.018)),
        material=rubber_black,
        name="foot_pad_0",
    )
    base.visual(
        Box((0.200, 0.170, 0.036)),
        origin=Origin(xyz=(0.165, 0.0, 0.018)),
        material=rubber_black,
        name="foot_pad_1",
    )
    base.visual(
        barrel_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=painted_red,
        name="barrel_shell",
    )
    base.visual(
        top_guide,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=dark_steel,
        name="top_guide",
    )
    base.visual(
        Box((0.028, 0.034, 0.180)),
        origin=Origin(xyz=(0.0, 0.065, 0.160)),
        material=dark_steel,
        name="gauge_stem",
    )
    base.visual(
        Box((0.028, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.094, 0.240)),
        material=dark_steel,
        name="gauge_arm",
    )
    base.visual(
        gauge_shell,
        origin=Origin(xyz=(0.0, 0.098, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gauge_gray,
        name="gauge_housing",
    )
    base.visual(
        gauge_bezel,
        origin=Origin(xyz=(0.0, 0.136, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="gauge_bezel",
    )
    base.visual(
        Cylinder(radius=0.056, length=0.002),
        origin=Origin(xyz=(0.0, 0.112, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_white,
        name="dial_face",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.0, -0.077, 0.228), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hose_outlet",
    )
    base.visual(hose_mesh, material=rubber_black, name="hose")
    base.visual(
        Box((0.030, 0.026, 0.030)),
        origin=Origin(xyz=(0.000, -0.087, 0.184)),
        material=dark_steel,
        name="hose_clip_0",
    )
    base.visual(
        Box((0.044, 0.028, 0.026)),
        origin=Origin(xyz=(0.058, -0.095, 0.090)),
        material=dark_steel,
        name="hose_clip_1",
    )
    base.visual(
        Box((0.052, 0.028, 0.022)),
        origin=Origin(xyz=(0.112, -0.072, 0.051)),
        material=dark_steel,
        name="hose_clip_2",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.164, -0.064, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hose_nozzle",
    )
    base.visual(
        Box((0.020, 0.024, 0.018)),
        origin=Origin(xyz=(0.195, -0.064, 0.050)),
        material=dark_steel,
        name="nozzle_head",
    )

    slider = model.part("slider")
    slider.visual(
        Cylinder(radius=0.0165, length=0.880),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=satin_steel,
        name="piston_rod",
    )
    slider.visual(
        Box((0.048, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.247)),
        material=dark_steel,
        name="handle_block",
    )
    slider.visual(
        Cylinder(radius=0.018, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="crossbar",
    )
    slider.visual(
        Cylinder(radius=0.022, length=0.110),
        origin=Origin(xyz=(-0.175, 0.0, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="bar_grip_0",
    )
    slider.visual(
        Cylinder(radius=0.022, length=0.110),
        origin=Origin(xyz=(0.175, 0.0, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="bar_grip_1",
    )

    grip = model.part("grip")
    grip.visual(
        grip_sleeve,
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="sleeve_0",
    )
    grip.visual(
        grip_sleeve,
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="sleeve_1",
    )
    grip.visual(
        Box((0.030, 0.036, 0.092)),
        origin=Origin(xyz=(-0.067, 0.040, -0.060)),
        material=rubber_black,
        name="strap_0",
    )
    grip.visual(
        Box((0.030, 0.036, 0.092)),
        origin=Origin(xyz=(0.067, 0.040, -0.060)),
        material=rubber_black,
        name="strap_1",
    )
    grip.visual(
        Box((0.115, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.068, -0.102)),
        material=rubber_black,
        name="yoke",
    )
    grip.visual(
        Cylinder(radius=0.013, length=0.120),
        origin=Origin(xyz=(0.0, 0.095, -0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="grip_bar",
    )

    needle = model.part("needle")
    needle.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    needle.visual(
        Box((0.004, 0.002, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=warning_orange,
        name="pointer_tip",
    )
    needle.visual(
        Box((0.010, 0.002, 0.012)),
        origin=Origin(xyz=(-0.006, 0.0, -0.004)),
        material=warning_orange,
        name="counterweight",
    )

    model.articulation(
        "pump_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.4,
            lower=0.0,
            upper=0.280,
        ),
    )
    model.articulation(
        "grip_roll",
        ArticulationType.CONTINUOUS,
        parent=slider,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "gauge_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=needle,
        origin=Origin(xyz=(0.0, 0.116, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    slider = object_model.get_part("slider")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")

    pump_slide = object_model.get_articulation("pump_slide")
    grip_roll = object_model.get_articulation("grip_roll")
    gauge_spin = object_model.get_articulation("gauge_spin")

    slide_limits = pump_slide.motion_limits

    ctx.allow_overlap(
        base,
        slider,
        elem_a="top_guide",
        elem_b="piston_rod",
        reason="The piston rod is intentionally represented as sliding through the pump head guide sleeve.",
    )
    ctx.allow_overlap(
        grip,
        slider,
        elem_a="sleeve_0",
        elem_b="crossbar",
        reason="The folding grip is intentionally represented as riding on snug sleeve bearings around the top crossbar.",
    )
    ctx.allow_overlap(
        grip,
        slider,
        elem_a="sleeve_1",
        elem_b="crossbar",
        reason="The folding grip is intentionally represented as riding on snug sleeve bearings around the top crossbar.",
    )

    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({pump_slide: slide_limits.lower}):
            ctx.expect_within(
                slider,
                base,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rod stays centered in barrel at rest",
            )
            ctx.expect_overlap(
                slider,
                base,
                axes="z",
                elem_a="piston_rod",
                elem_b="barrel_shell",
                min_overlap=0.360,
                name="collapsed rod remains inserted in barrel",
            )
            ctx.expect_gap(
                slider,
                base,
                axis="z",
                positive_elem="crossbar",
                negative_elem="top_guide",
                min_gap=0.230,
                max_gap=0.320,
                name="handle sits above the pump head at rest",
            )
            rest_slider_pos = ctx.part_world_position(slider)

        with ctx.pose({pump_slide: slide_limits.upper}):
            ctx.expect_within(
                slider,
                base,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rod stays centered in barrel when extended",
            )
            ctx.expect_overlap(
                slider,
                base,
                axes="z",
                elem_a="piston_rod",
                elem_b="barrel_shell",
                min_overlap=0.180,
                name="extended rod still retains insertion",
            )
            extended_slider_pos = ctx.part_world_position(slider)

        ctx.check(
            "handle extends upward",
            rest_slider_pos is not None
            and extended_slider_pos is not None
            and extended_slider_pos[2] > rest_slider_pos[2] + 0.200,
            details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
        )

    with ctx.pose({grip_roll: 0.0}):
        ctx.expect_within(
            slider,
            grip,
            axes="yz",
            inner_elem="crossbar",
            outer_elem="sleeve_0",
            margin=0.0,
            name="first grip sleeve remains concentric on crossbar",
        )
        ctx.expect_overlap(
            grip,
            slider,
            axes="x",
            elem_a="sleeve_0",
            elem_b="crossbar",
            min_overlap=0.050,
            name="first grip sleeve remains carried by crossbar",
        )
        ctx.expect_within(
            slider,
            grip,
            axes="yz",
            inner_elem="crossbar",
            outer_elem="sleeve_1",
            margin=0.0,
            name="second grip sleeve remains concentric on crossbar",
        )
        ctx.expect_overlap(
            grip,
            slider,
            axes="x",
            elem_a="sleeve_1",
            elem_b="crossbar",
            min_overlap=0.050,
            name="second grip sleeve remains carried by crossbar",
        )
        grip_bar_rest = _aabb_center(ctx.part_element_world_aabb(grip, elem="grip_bar"))

    with ctx.pose({grip_roll: math.pi / 2.0}):
        ctx.expect_within(
            slider,
            grip,
            axes="yz",
            inner_elem="crossbar",
            outer_elem="sleeve_0",
            margin=0.0,
            name="first grip sleeve stays concentric when folded",
        )
        ctx.expect_overlap(
            grip,
            slider,
            axes="x",
            elem_a="sleeve_0",
            elem_b="crossbar",
            min_overlap=0.050,
            name="first grip sleeve stays carried when folded",
        )
        ctx.expect_within(
            slider,
            grip,
            axes="yz",
            inner_elem="crossbar",
            outer_elem="sleeve_1",
            margin=0.0,
            name="second grip sleeve stays concentric when folded",
        )
        ctx.expect_overlap(
            grip,
            slider,
            axes="x",
            elem_a="sleeve_1",
            elem_b="crossbar",
            min_overlap=0.050,
            name="second grip sleeve stays carried when folded",
        )
        grip_bar_folded = _aabb_center(ctx.part_element_world_aabb(grip, elem="grip_bar"))

    ctx.check(
        "grip folds around the top bar",
        grip_bar_rest is not None
        and grip_bar_folded is not None
        and (
            (
                (grip_bar_folded[1] - grip_bar_rest[1]) ** 2
                + (grip_bar_folded[2] - grip_bar_rest[2]) ** 2
            )
            ** 0.5
            > 0.120
        )
        and abs(grip_bar_folded[2] - grip_bar_rest[2]) > 0.080,
        details=f"rest={grip_bar_rest}, folded={grip_bar_folded}",
    )

    with ctx.pose({gauge_spin: 0.0}):
        needle_tip_rest = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer_tip"))
    with ctx.pose({gauge_spin: math.pi / 2.0}):
        needle_tip_turned = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer_tip"))

    ctx.check(
        "gauge needle sweeps across the dial",
        needle_tip_rest is not None
        and needle_tip_turned is not None
        and abs(needle_tip_turned[0] - needle_tip_rest[0]) > 0.015
        and abs(needle_tip_turned[2] - needle_tip_rest[2]) > 0.015,
        details=f"rest={needle_tip_rest}, turned={needle_tip_turned}",
    )

    return ctx.report()


object_model = build_object_model()
