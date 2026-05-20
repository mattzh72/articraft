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


def _shell_mesh(
    name: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    rotate_y: float = 0.0,
    rotate_x: float = 0.0,
):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )
    if rotate_x:
        geom.rotate_x(rotate_x)
    if rotate_y:
        geom.rotate_y(rotate_y)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_floor_pump")

    body_red = model.material("body_red", rgba=(0.78, 0.17, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    gauge_white = model.material("gauge_white", rgba=(0.96, 0.96, 0.95, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.85, 0.92, 0.98, 0.35))
    needle_red = model.material("needle_red", rgba=(0.86, 0.18, 0.15, 1.0))

    base_frame = model.part("base_frame")

    barrel_shell = _shell_mesh(
        "barrel_shell",
        outer_profile=[(0.040, 0.000), (0.038, 0.170), (0.0365, 0.560)],
        inner_profile=[(0.0315, 0.008), (0.0305, 0.170), (0.0290, 0.552)],
    )
    gauge_shell = _shell_mesh(
        "gauge_shell",
        outer_profile=[(0.060, 0.000), (0.058, 0.032)],
        inner_profile=[(0.052, 0.004), (0.050, 0.028)],
        rotate_x=math.pi / 2.0,
    )
    top_bushing = _shell_mesh(
        "top_bushing",
        outer_profile=[(0.031, 0.000), (0.031, 0.016)],
        inner_profile=[(0.011, 0.002), (0.011, 0.014)],
    )

    hose_geom = tube_from_spline_points(
        [
            (0.086, 0.028, 0.118),
            (0.095, 0.018, 0.200),
            (0.090, 0.000, 0.320),
            (0.076, 0.000, 0.430),
            (0.060, 0.012, 0.570),
            (0.038, 0.036, 0.710),
            (0.018, 0.052, 0.792),
        ],
        radius=0.007,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    hose_mesh = _mesh("hose", hose_geom)

    base_frame.visual(
        Box((0.360, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="base_spine",
    )
    base_frame.visual(
        Box((0.120, 0.120, 0.016)),
        origin=Origin(xyz=(-0.108, 0.0, 0.012)),
        material=black_rubber,
        name="left_tread",
    )
    base_frame.visual(
        Box((0.120, 0.120, 0.016)),
        origin=Origin(xyz=(0.108, 0.0, 0.012)),
        material=black_rubber,
        name="right_tread",
    )
    base_frame.visual(
        Box((0.072, 0.090, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=dark_steel,
        name="pedestal",
    )
    base_frame.visual(
        Cylinder(radius=0.046, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=dark_steel,
        name="barrel_collar",
    )
    base_frame.visual(
        barrel_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=body_red,
        name="barrel_shell",
    )
    base_frame.visual(
        top_bushing,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=dark_steel,
        name="top_bushing",
    )
    base_frame.visual(
        Box((0.010, 0.028, 0.004)),
        origin=Origin(xyz=(-0.017, 0.0, 0.664)),
        material=dark_steel,
        name="stop_pad_0",
    )
    base_frame.visual(
        Box((0.010, 0.028, 0.004)),
        origin=Origin(xyz=(0.017, 0.0, 0.664)),
        material=dark_steel,
        name="stop_pad_1",
    )

    base_frame.visual(
        Box((0.034, 0.036, 0.090)),
        origin=Origin(xyz=(0.0, 0.020, 0.136)),
        material=dark_steel,
        name="gauge_bracket",
    )
    base_frame.visual(
        gauge_shell,
        origin=Origin(xyz=(0.0, 0.056, 0.175)),
        material=dark_steel,
        name="gauge_shell",
    )
    base_frame.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.041, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="gauge_back",
    )
    base_frame.visual(
        Cylinder(radius=0.047, length=0.008),
        origin=Origin(xyz=(0.0, 0.046, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_white,
        name="gauge_face",
    )
    base_frame.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.0, 0.047, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_post",
    )
    base_frame.visual(
        Cylinder(radius=0.052, length=0.004),
        origin=Origin(xyz=(0.0, 0.054, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_clear,
        name="gauge_lens",
    )

    base_frame.visual(
        Box((0.020, 0.018, 0.030)),
        origin=Origin(xyz=(0.054, 0.028, 0.118)),
        material=dark_steel,
        name="hose_outlet_block",
    )
    base_frame.visual(
        Box((0.048, 0.016, 0.022)),
        origin=Origin(xyz=(0.032, 0.026, 0.118)),
        material=dark_steel,
        name="hose_support",
    )
    base_frame.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.075, 0.028, 0.118), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hose_coupler",
    )
    base_frame.visual(
        hose_mesh,
        material=black_rubber,
        name="hose",
    )
    base_frame.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.018, 0.052, 0.812)),
        material=dark_steel,
        name="hose_head",
    )

    base_frame.visual(
        Box((0.010, 0.032, 0.074)),
        origin=Origin(xyz=(0.041, 0.0, 0.430)),
        material=dark_steel,
        name="clip_back",
    )
    base_frame.visual(
        Box((0.024, 0.022, 0.010)),
        origin=Origin(xyz=(0.053, 0.0, 0.456)),
        material=dark_steel,
        name="clip_upper",
    )
    base_frame.visual(
        Box((0.024, 0.022, 0.010)),
        origin=Origin(xyz=(0.053, 0.0, 0.404)),
        material=dark_steel,
        name="clip_lower",
    )

    handle_assembly = model.part("handle_assembly")
    handle_assembly.visual(
        Cylinder(radius=0.0085, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=steel,
        name="rod",
    )
    handle_assembly.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_steel,
        name="guide_collar",
    )
    handle_assembly.visual(
        Box((0.034, 0.034, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, 0.404)),
        material=dark_steel,
        name="handle_block",
    )
    handle_assembly.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.430), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="crossbar",
    )
    handle_assembly.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.386)),
        material=dark_steel,
        name="rod_collar",
    )

    grip = model.part("grip")
    grip_sleeve = _shell_mesh(
        "grip_sleeve",
        outer_profile=[(0.025, -0.060), (0.025, 0.060)],
        inner_profile=[(0.0135, -0.058), (0.0135, 0.058)],
        rotate_y=math.pi / 2.0,
    )
    grip.visual(
        grip_sleeve,
        material=black_rubber,
        name="grip_sleeve",
    )
    grip.visual(
        Box((0.108, 0.028, 0.064)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=black_rubber,
        name="grip_body",
    )

    needle = model.part("needle")
    needle.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot",
    )
    needle.visual(
        Box((0.036, 0.002, 0.006)),
        origin=Origin(xyz=(0.018, -0.001, 0.0)),
        material=needle_red,
        name="pointer",
    )
    needle.visual(
        Box((0.010, 0.002, 0.004)),
        origin=Origin(xyz=(-0.007, -0.001, 0.0)),
        material=needle_red,
        name="tail",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=handle_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.662)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=0.260,
        ),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle_assembly,
        child=grip,
        origin=Origin(xyz=(0.075, 0.0, 0.430)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=8.0,
        ),
    )
    model.articulation(
        "base_to_needle",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=needle,
        origin=Origin(xyz=(0.0, 0.051, 0.175)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=-1.05,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_frame = object_model.get_part("base_frame")
    handle_assembly = object_model.get_part("handle_assembly")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")

    handle_joint = object_model.get_articulation("base_to_handle")
    grip_joint = object_model.get_articulation("handle_to_grip")
    needle_joint = object_model.get_articulation("base_to_needle")

    handle_limits = handle_joint.motion_limits
    needle_limits = needle_joint.motion_limits

    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: 0.0}):
            ctx.expect_within(
                handle_assembly,
                base_frame,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rod stays centered in barrel at rest",
            )
            ctx.expect_overlap(
                handle_assembly,
                base_frame,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.300,
                name="rod remains deeply inserted at rest",
            )
            rest_handle_pos = ctx.part_world_position(handle_assembly)

        with ctx.pose({handle_joint: handle_limits.upper}):
            ctx.expect_within(
                handle_assembly,
                base_frame,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rod stays centered in barrel at full stroke",
            )
            ctx.expect_overlap(
                handle_assembly,
                base_frame,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.070,
                name="rod retains insertion at full stroke",
            )
            extended_handle_pos = ctx.part_world_position(handle_assembly)

        ctx.check(
            "handle travels upward",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > rest_handle_pos[2] + 0.22,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    ctx.expect_overlap(
        grip,
        handle_assembly,
        axes="x",
        elem_a="grip_sleeve",
        elem_b="crossbar",
        min_overlap=0.100,
        name="grip sleeve spans the handle crossbar",
    )
    ctx.expect_within(
        handle_assembly,
        grip,
        axes="yz",
        inner_elem="crossbar",
        outer_elem="grip_sleeve",
        margin=0.014,
        name="crossbar stays inside the grip sleeve envelope",
    )

    rest_grip_box = ctx.part_element_world_aabb(grip, elem="grip_body")
    with ctx.pose({grip_joint: math.pi / 2.0}):
        turned_grip_box = ctx.part_element_world_aabb(grip, elem="grip_body")
    if rest_grip_box is not None and turned_grip_box is not None:
        rest_center = tuple((a + b) * 0.5 for a, b in zip(rest_grip_box[0], rest_grip_box[1]))
        turned_center = tuple((a + b) * 0.5 for a, b in zip(turned_grip_box[0], turned_grip_box[1]))
    else:
        rest_center = None
        turned_center = None
    ctx.check(
        "grip rotates around crossbar axis",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[1] - rest_center[1]) > 0.030
        and turned_center[2] > rest_center[2] + 0.030,
        details=f"rest={rest_grip_box}, turned={turned_grip_box}",
    )

    if needle_limits is not None:
        rest_pointer_box = ctx.part_element_world_aabb(needle, elem="pointer")
        with ctx.pose({needle_joint: 0.75}):
            raised_pointer_box = ctx.part_element_world_aabb(needle, elem="pointer")
        ctx.check(
            "needle swings upward from its pivot",
            rest_pointer_box is not None
            and raised_pointer_box is not None
            and raised_pointer_box[1][2] > rest_pointer_box[1][2] + 0.012,
            details=f"rest={rest_pointer_box}, raised={raised_pointer_box}",
        )

    return ctx.report()


object_model = build_object_model()
