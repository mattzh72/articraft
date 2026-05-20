from __future__ import annotations

import math

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
    rounded_rect_profile,
    ExtrudeGeometry,
    tube_from_spline_points,
)


def _hollow_cylinder_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    radial_segments: int = 56,
):
    """Return a managed mesh for a straight, open-bore tubular sleeve."""
    half = length * 0.5
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=radial_segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    return mesh_from_geometry(shell, name)


def _vertical_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    radial_segments: int = 64,
):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=radial_segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_workshop_floor_pump")

    powder_blue = model.material("powder_blue", rgba=(0.06, 0.22, 0.42, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.01, 0.011, 0.012, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.025, 0.03, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    white_face = model.material("white_gauge_face", rgba=(0.94, 0.92, 0.86, 1.0))
    red = model.material("red_needle", rgba=(0.86, 0.04, 0.03, 1.0))

    frame = model.part("frame")

    base_plate = ExtrudeGeometry(
        rounded_rect_profile(0.54, 0.24, 0.045, corner_segments=10),
        0.038,
        cap=True,
        center=True,
    )
    frame.visual(
        mesh_from_geometry(base_plate, "wide_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=black_plastic,
        name="wide_base",
    )
    frame.visual(
        Box((0.22, 0.10, 0.012)),
        origin=Origin(xyz=(-0.14, -0.018, 0.044)),
        material=dark_rubber,
        name="foot_pad_0",
    )
    frame.visual(
        Box((0.22, 0.10, 0.012)),
        origin=Origin(xyz=(0.14, -0.018, 0.044)),
        material=dark_rubber,
        name="foot_pad_1",
    )

    frame.visual(
        _vertical_tube_mesh(
            "barrel_shell_mesh",
            outer_radius=0.034,
            inner_radius=0.014,
            z_min=0.038,
            z_max=0.790,
        ),
        material=powder_blue,
        name="barrel_shell",
    )
    frame.visual(
        _vertical_tube_mesh(
            "bottom_collar_mesh",
            outer_radius=0.046,
            inner_radius=0.014,
            z_min=0.040,
            z_max=0.095,
        ),
        material=satin_steel,
        name="bottom_collar",
    )
    frame.visual(
        _vertical_tube_mesh(
            "top_collar_mesh",
            outer_radius=0.045,
            inner_radius=0.014,
            z_min=0.750,
            z_max=0.825,
        ),
        material=satin_steel,
        name="top_collar",
    )

    for index, x in enumerate((-0.175, 0.175)):
        support = tube_from_spline_points(
            [
                (x, 0.075, 0.040),
                (x * 0.72, 0.060, 0.260),
                (x * 0.42, 0.030, 0.545),
                (x * 0.20, 0.008, 0.765),
            ],
            radius=0.0085,
            samples_per_segment=14,
            radial_segments=20,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(support, f"tubular_support_{index}_mesh"),
            material=powder_blue,
            name=f"tubular_support_{index}",
        )

    rear_bridge = tube_from_spline_points(
        [(-0.190, 0.083, 0.052), (-0.070, 0.086, 0.070), (0.070, 0.086, 0.070), (0.190, 0.083, 0.052)],
        radius=0.0075,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(rear_bridge, "rear_bridge_mesh"),
        material=powder_blue,
        name="rear_bridge",
    )

    hose_hook = tube_from_spline_points(
        [
            (0.030, -0.004, 0.530),
            (0.064, -0.018, 0.532),
            (0.090, -0.026, 0.498),
            (0.086, -0.026, 0.454),
            (0.058, -0.018, 0.438),
        ],
        radius=0.0065,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(hose_hook, "hose_hook_mesh"),
        material=black_plastic,
        name="hose_hook",
    )
    hose = tube_from_spline_points(
        [
            (0.024, -0.036, 0.150),
            (0.080, -0.066, 0.235),
            (0.118, -0.052, 0.360),
            (0.098, -0.030, 0.485),
            (0.066, -0.023, 0.505),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(hose, "short_hose_mesh"),
        material=dark_rubber,
        name="short_hose",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.020, -0.020, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hose_outlet",
    )

    frame.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.0, -0.058, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="gauge_case",
    )
    frame.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, -0.071, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white_face,
        name="gauge_face",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.058),
        origin=Origin(xyz=(0.0, -0.032, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="gauge_stem",
    )
    for index, angle in enumerate((-1.15, -0.58, 0.0, 0.58, 1.15)):
        x = math.sin(angle) * 0.038
        z = 0.205 + math.cos(angle) * 0.038
        frame.visual(
            Box((0.004, 0.004, 0.014)),
            origin=Origin(
                xyz=(x, -0.074, z),
                rpy=(0.0, angle, 0.0),
            ),
            material=black_plastic,
            name=f"gauge_tick_{index}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0065, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material=satin_steel,
        name="piston_rod",
    )
    plunger.visual(
        Cylinder(radius=0.010, length=0.385),
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="crossbar",
    )
    plunger.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(-0.148, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="stop_collar_0",
    )
    plunger.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.148, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="stop_collar_1",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(-0.198, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="bar_end_0",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.198, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="bar_end_1",
    )

    grip = model.part("grip")
    grip.visual(
        _hollow_cylinder_mesh(
            "grip_sleeve_0_mesh",
            outer_radius=0.027,
            inner_radius=0.010,
            length=0.110,
            radial_segments=56,
        ),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="grip_sleeve_0",
    )
    grip.visual(
        _hollow_cylinder_mesh(
            "grip_sleeve_1_mesh",
            outer_radius=0.027,
            inner_radius=0.010,
            length=0.110,
            radial_segments=56,
        ),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="grip_sleeve_1",
    )
    grip.visual(
        Box((0.078, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_rubber,
        name="grip_bridge",
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.005, 0.002, 0.070)),
        origin=Origin(xyz=(0.0, -0.002, 0.035)),
        material=red,
        name="needle_pointer",
    )
    needle.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="needle_axle",
    )

    model.articulation(
        "frame_to_plunger",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.32, lower=0.0, upper=0.250),
    )
    model.articulation(
        "plunger_to_grip",
        ArticulationType.CONTINUOUS,
        parent=plunger,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_needle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=needle,
        origin=Origin(xyz=(0.0, -0.071, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0, lower=-2.25, upper=2.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    plunger = object_model.get_part("plunger")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")
    pump_slide = object_model.get_articulation("frame_to_plunger")
    needle_joint = object_model.get_articulation("frame_to_needle")

    for sleeve_name in ("grip_sleeve_0", "grip_sleeve_1"):
        ctx.allow_overlap(
            grip,
            plunger,
            elem_a=sleeve_name,
            elem_b="crossbar",
            reason="The rotating rubber grip sleeve is intentionally captured around the steel crossbar axle.",
        )
        ctx.expect_within(
            plunger,
            grip,
            axes="yz",
            inner_elem="crossbar",
            outer_elem=sleeve_name,
            margin=0.0,
            name=f"{sleeve_name} surrounds the crossbar axle",
        )
        ctx.expect_overlap(
            grip,
            plunger,
            axes="x",
            elem_a=sleeve_name,
            elem_b="crossbar",
            min_overlap=0.10,
            name=f"{sleeve_name} has retained length on crossbar",
        )

    ctx.expect_within(
        "plunger",
        "frame",
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        margin=0.002,
        name="piston rod stays centered in the barrel bore",
    )
    ctx.expect_overlap(
        "plunger",
        "frame",
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.50,
        name="collapsed piston rod remains deeply inserted",
    )
    with ctx.pose({pump_slide: 0.250}):
        ctx.expect_within(
            plunger,
            frame,
            axes="xy",
            inner_elem="piston_rod",
            outer_elem="barrel_shell",
            margin=0.002,
            name="extended piston rod remains centered",
        )
        ctx.expect_overlap(
            plunger,
            frame,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel_shell",
            min_overlap=0.30,
            name="extended piston rod keeps retained insertion",
        )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({pump_slide: 0.250}):
        raised_pos = ctx.part_world_position(plunger)
    ctx.check(
        "T handle slides upward on barrel axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.22,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_within(
        grip,
        plunger,
        axes="x",
        outer_elem="crossbar",
        margin=0.0,
        name="grip sleeve sits between crossbar ends",
    )
    ctx.expect_overlap(
        grip,
        plunger,
        axes="x",
        elem_b="crossbar",
        min_overlap=0.24,
        name="grip sleeve is carried by the T bar",
    )

    at_zero = ctx.part_element_world_aabb(needle, elem="needle_pointer")
    with ctx.pose({needle_joint: 1.20}):
        swept = ctx.part_element_world_aabb(needle, elem="needle_pointer")
    ctx.check(
        "gauge needle rotates about central axle",
        at_zero is not None
        and swept is not None
        and (swept[1][0] - swept[0][0]) > (at_zero[1][0] - at_zero[0][0]) + 0.03,
        details=f"zero={at_zero}, swept={swept}",
    )

    return ctx.report()


object_model = build_object_model()
