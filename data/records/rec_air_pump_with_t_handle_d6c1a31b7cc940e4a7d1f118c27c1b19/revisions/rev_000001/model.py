from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Thin-walled hollow tube along local Z, with annular end faces."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            inner_profile=[(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_floor_air_pump")

    safety_yellow = model.material("powder_coated_yellow", rgba=(0.95, 0.66, 0.08, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    gauge_face = model.material("gauge_face", rgba=(0.92, 0.92, 0.86, 1.0))
    red_mark = model.material("red_mark", rgba=(0.85, 0.05, 0.03, 1.0))

    frame = model.part("frame")

    # Heavy floor-pump base and hollow lower slide guide.
    frame.visual(
        Box((0.38, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    guide_z = 0.068
    guide_len = 0.62
    guide_inner_y = 0.054
    guide_inner_z = 0.038
    guide_wall = 0.014
    guide_outer_y = guide_inner_y + 2.0 * guide_wall
    guide_outer_z = guide_inner_z + 2.0 * guide_wall
    frame.visual(
        Box((guide_len, guide_outer_y, guide_wall)),
        origin=Origin(xyz=(0.0, 0.0, guide_z + guide_inner_z / 2.0 + guide_wall / 2.0)),
        material=dark_steel,
        name="guide_top",
    )
    frame.visual(
        Box((guide_len, guide_outer_y, guide_wall)),
        origin=Origin(xyz=(0.0, 0.0, guide_z - guide_inner_z / 2.0 - guide_wall / 2.0)),
        material=dark_steel,
        name="guide_bottom",
    )
    frame.visual(
        Box((guide_len, guide_wall, guide_outer_z)),
        origin=Origin(xyz=(0.0, guide_inner_y / 2.0 + guide_wall / 2.0, guide_z)),
        material=dark_steel,
        name="guide_side_pos",
    )
    frame.visual(
        Box((guide_len, guide_wall, guide_outer_z)),
        origin=Origin(xyz=(0.0, -guide_inner_y / 2.0 - guide_wall / 2.0, guide_z)),
        material=dark_steel,
        name="guide_side_neg",
    )

    # Tall pump cylinder is a real annular shell so the piston rod passes through
    # an open bore rather than a hidden solid placeholder.
    pump_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.062, 0.000),
            (0.062, 0.030),
            (0.055, 0.045),
            (0.055, 0.805),
            (0.066, 0.820),
        ],
        inner_profile=[
            (0.039, 0.000),
            (0.039, 0.820),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    frame.visual(
        mesh_from_geometry(pump_shell, "pump_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=safety_yellow,
        name="pump_shell",
    )
    frame.visual(
        _tube_mesh(0.071, 0.028, 0.045, "top_gland"),
        origin=Origin(xyz=(0.0, 0.0, 0.916)),
        material=dark_steel,
        name="top_gland",
    )
    frame.visual(
        _tube_mesh(0.069, 0.038, 0.052, "base_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=dark_steel,
        name="base_collar",
    )

    # Gauge, valve boss, and a short coiled hose make the model read as an
    # industrial air pump while staying fused to the stationary frame.
    frame.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, -0.062, 0.705), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="gauge_stem",
    )
    frame.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(0.0, -0.096, 0.705), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="gauge_bezel",
    )
    frame.visual(
        Cylinder(radius=0.041, length=0.020),
        origin=Origin(xyz=(0.0, -0.107, 0.705), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )
    frame.visual(
        Box((0.004, 0.004, 0.056)),
        origin=Origin(xyz=(0.0, -0.116, 0.718), rpy=(0.0, pi / 8.0, 0.0)),
        material=red_mark,
        name="gauge_needle",
    )
    hose = tube_from_spline_points(
        [
            (0.051, -0.020, 0.245),
            (0.120, -0.145, 0.205),
            (0.205, -0.145, 0.090),
            (0.235, -0.045, 0.047),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(hose, "side_hose"),
        material=black_rubber,
        name="side_hose",
    )

    stabilizer = model.part("stabilizer")
    stabilizer.visual(
        Box((1.20, 0.038, 0.026)),
        origin=Origin(),
        material=brushed_steel,
        name="inner_bar",
    )
    stabilizer.visual(
        Box((0.560, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=black_rubber,
        name="lower_wear_strip",
    )
    stabilizer.visual(
        Box((0.085, 0.155, 0.026)),
        origin=Origin(xyz=(-0.610, 0.0, -0.023)),
        material=black_rubber,
        name="foot_pad_0",
    )
    stabilizer.visual(
        Box((0.085, 0.155, 0.026)),
        origin=Origin(xyz=(0.610, 0.0, -0.023)),
        material=black_rubber,
        name="foot_pad_1",
    )
    stabilizer.visual(
        Box((0.020, 0.060, 0.038)),
        origin=Origin(xyz=(-0.560, 0.0, -0.006)),
        material=brushed_steel,
        name="end_web_0",
    )
    stabilizer.visual(
        Box((0.020, 0.060, 0.038)),
        origin=Origin(xyz=(0.560, 0.0, -0.006)),
        material=brushed_steel,
        name="end_web_1",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.013, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        material=brushed_steel,
        name="piston_rod",
    )
    handle.visual(
        Cylinder(radius=0.039, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.610)),
        material=black_rubber,
        name="piston_cup",
    )
    handle.visual(
        Cylinder(radius=0.020, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=dark_steel,
        name="handle_stem",
    )
    handle.visual(
        Cylinder(radius=0.017, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="top_bar",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(-0.385, 0.0, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="end_bumper_0",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.385, 0.0, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="end_bumper_1",
    )

    grip = model.part("grip")
    grip.visual(
        _tube_mesh(0.035, 0.017, 0.295, "grip_sleeve"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="sleeve",
    )
    grip.visual(
        Cylinder(radius=0.024, length=0.270),
        origin=Origin(xyz=(0.0, -0.070, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="folding_pad",
    )
    grip.visual(
        Box((0.036, 0.030, 0.052)),
        origin=Origin(xyz=(-0.105, -0.040, -0.034)),
        material=black_rubber,
        name="pad_web_0",
    )
    grip.visual(
        Box((0.036, 0.030, 0.052)),
        origin=Origin(xyz=(0.105, -0.040, -0.034)),
        material=black_rubber,
        name="pad_web_1",
    )

    model.articulation(
        "frame_to_stabilizer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stabilizer,
        origin=Origin(xyz=(0.0, 0.0, guide_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=-0.24, upper=0.24),
    )
    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.916)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=0.0, upper=0.30),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.180, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    stabilizer = object_model.get_part("stabilizer")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    stabilizer_slide = object_model.get_articulation("frame_to_stabilizer")
    handle_slide = object_model.get_articulation("frame_to_handle")
    grip_spin = object_model.get_articulation("handle_to_grip")

    ctx.allow_overlap(
        frame,
        handle,
        elem_a="pump_shell",
        elem_b="piston_cup",
        reason=(
            "The hidden rubber piston cup is intentionally represented as a "
            "compressed sealing plunger inside the pump cylinder bore."
        ),
    )
    ctx.allow_overlap(
        grip,
        handle,
        elem_a="sleeve",
        elem_b="top_bar",
        reason=(
            "The folding grip sleeve is intentionally captured on the top "
            "crossbar; the overlap represents the bearing fit that supports its continuous rotation."
        ),
    )
    ctx.expect_within(
        handle,
        frame,
        axes="xy",
        inner_elem="piston_cup",
        outer_elem="pump_shell",
        margin=0.001,
        name="piston cup stays captured inside cylinder diameter",
    )
    ctx.expect_overlap(
        handle,
        frame,
        axes="z",
        elem_a="piston_cup",
        elem_b="pump_shell",
        min_overlap=0.040,
        name="piston cup remains axially inside pump cylinder",
    )
    ctx.expect_within(
        handle,
        frame,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="pump_shell",
        margin=0.001,
        name="piston rod stays centered in cylinder bore",
    )
    ctx.expect_overlap(
        handle,
        frame,
        axes="z",
        elem_a="piston_rod",
        elem_b="pump_shell",
        min_overlap=0.55,
        name="collapsed piston remains deeply inside cylinder",
    )
    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.30}):
        ctx.expect_overlap(
            handle,
            frame,
            axes="z",
            elem_a="piston_rod",
            elem_b="pump_shell",
            min_overlap=0.35,
            name="raised piston remains retained in cylinder",
        )
        raised_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "T handle translates upward along cylinder axis",
        rest_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] > rest_handle_pos[2] + 0.29,
        details=f"rest={rest_handle_pos}, raised={raised_handle_pos}",
    )

    ctx.expect_gap(
        frame,
        stabilizer,
        axis="z",
        positive_elem="guide_top",
        negative_elem="inner_bar",
        min_gap=0.005,
        max_gap=0.010,
        name="stabilizer clears upper guide wall",
    )
    ctx.expect_gap(
        stabilizer,
        frame,
        axis="z",
        positive_elem="inner_bar",
        negative_elem="guide_bottom",
        min_gap=0.005,
        max_gap=0.010,
        name="stabilizer clears lower guide wall",
    )
    ctx.expect_gap(
        frame,
        stabilizer,
        axis="y",
        positive_elem="guide_side_pos",
        negative_elem="inner_bar",
        min_gap=0.007,
        max_gap=0.010,
        name="stabilizer clears positive guide cheek",
    )
    ctx.expect_gap(
        stabilizer,
        frame,
        axis="y",
        positive_elem="inner_bar",
        negative_elem="guide_side_neg",
        min_gap=0.007,
        max_gap=0.010,
        name="stabilizer clears negative guide cheek",
    )
    ctx.expect_overlap(
        stabilizer,
        frame,
        axes="x",
        elem_a="inner_bar",
        elem_b="guide_top",
        min_overlap=0.60,
        name="centered stabilizer is nested across lower guide",
    )
    with ctx.pose({stabilizer_slide: 0.24}):
        ctx.expect_overlap(
            stabilizer,
            frame,
            axes="x",
            elem_a="inner_bar",
            elem_b="guide_top",
            min_overlap=0.60,
            name="extended stabilizer remains retained in lower guide",
        )

    ctx.expect_within(
        handle,
        grip,
        axes="yz",
        inner_elem="top_bar",
        outer_elem="sleeve",
        margin=0.002,
        name="top bar stays inside rotating grip sleeve envelope",
    )
    ctx.expect_overlap(
        grip,
        handle,
        axes="x",
        elem_a="sleeve",
        elem_b="top_bar",
        min_overlap=0.26,
        name="rotating sleeve is supported along top crossbar",
    )
    with ctx.pose({grip_spin: pi / 2.0}):
        ctx.expect_overlap(
            grip,
            handle,
            axes="x",
            elem_a="sleeve",
            elem_b="top_bar",
            min_overlap=0.26,
            name="folded grip remains around crossbar after rotation",
        )

    return ctx.report()


object_model = build_object_model()
