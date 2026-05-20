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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=14,
            cap_ends=True,
            corner_mode="miter",
        ),
        name,
    )


def _bell_shell_mesh():
    outer = [
        (0.280, -0.760),
        (0.292, -0.720),
        (0.270, -0.620),
        (0.224, -0.455),
        (0.164, -0.320),
        (0.106, -0.235),
        (0.070, -0.205),
    ]
    inner = [
        (0.220, -0.710),
        (0.210, -0.620),
        (0.178, -0.455),
        (0.126, -0.320),
        (0.072, -0.235),
        (0.038, -0.205),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=72,
            start_cap="round",
            end_cap="flat",
            lip_samples=8,
        ),
        "bell_shell",
    )


def _bell_rim_mesh():
    return mesh_from_geometry(
        TorusGeometry(radius=0.255, tube=0.028, radial_segments=16, tubular_segments=72).translate(
            0.0,
            0.0,
            -0.742,
        ),
        "bell_mouth_rim",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_timber_belfry")

    aged_oak = model.material("aged_oak", rgba=(0.50, 0.31, 0.16, 1.0))
    end_grain = model.material("dark_end_grain", rgba=(0.32, 0.19, 0.10, 1.0))
    roof_wood = model.material("cedar_shakes", rgba=(0.40, 0.26, 0.17, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.08, 0.075, 0.065, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.58, 0.39, 0.16, 1.0))

    frame = model.part("timber_frame")

    post_radius = 0.065
    post_height = 2.68
    for index, (x, y) in enumerate(
        [(-0.62, -0.62), (0.62, -0.62), (0.62, 0.62), (-0.62, 0.62)]
    ):
        frame.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(x, y, post_height / 2.0)),
            material=aged_oak,
            name=f"post_{index}",
        )
        frame.visual(
            Cylinder(radius=0.088, length=0.030),
            origin=Origin(xyz=(x, y, 0.015)),
            material=end_grain,
            name=f"foot_pad_{index}",
        )

    frame.visual(
        Box((1.48, 1.48, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=aged_oak,
        name="platform_deck",
    )
    for i, x in enumerate((-0.48, -0.16, 0.16, 0.48)):
        frame.visual(
            Box((0.035, 1.50, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.836)),
            material=end_grain,
            name=f"plank_gap_{i}",
        )

    for y, name in [(-0.69, "rail_y_0"), (0.69, "rail_y_1")]:
        frame.visual(
            Box((1.52, 0.080, 0.100)),
            origin=Origin(xyz=(0.0, y, 1.020)),
            material=aged_oak,
            name=name,
        )
    for x, name in [(-0.69, "rail_x_0"), (0.69, "rail_x_1")]:
        frame.visual(
            Box((0.080, 1.52, 0.100)),
            origin=Origin(xyz=(x, 0.0, 1.020)),
            material=aged_oak,
            name=name,
        )

    for y, name in [(-0.66, "top_beam_y_0"), (0.66, "top_beam_y_1")]:
        frame.visual(
            Box((1.55, 0.120, 0.140)),
            origin=Origin(xyz=(0.0, y, 2.560)),
            material=aged_oak,
            name=name,
        )
    for x, name in [(-0.66, "top_beam_x_0"), (0.66, "top_beam_x_1")]:
        frame.visual(
            Box((0.120, 1.55, 0.140)),
            origin=Origin(xyz=(x, 0.0, 2.560)),
            material=aged_oak,
            name=name,
        )
    for y, name in [(-0.675, "roof_plate_0"), (0.675, "roof_plate_1")]:
        frame.visual(
            Box((1.42, 0.070, 0.120)),
            origin=Origin(xyz=(0.0, y, 2.650)),
            material=aged_oak,
            name=name,
        )

    brace_specs = [
        ((-0.62, -0.66, 0.96), (0.62, -0.66, 2.48), "brace_0"),
        ((0.62, -0.66, 0.96), (-0.62, -0.66, 2.48), "brace_1"),
        ((-0.62, 0.66, 0.96), (0.62, 0.66, 2.48), "brace_2"),
        ((0.62, 0.66, 0.96), (-0.62, 0.66, 2.48), "brace_3"),
    ]
    for p0, p1, name in brace_specs:
        frame.visual(_tube_mesh([p0, p1], 0.030, name), material=aged_oak, name=name)

    for x, name in [(-0.535, "headstock_bearer_0"), (0.535, "headstock_bearer_1")]:
        frame.visual(
            Box((0.115, 1.36, 0.100)),
            origin=Origin(xyz=(x, 0.0, 2.125)),
            material=aged_oak,
            name=name,
        )
        frame.visual(
            Box((0.110, 0.260, 0.080)),
            origin=Origin(xyz=(x, 0.0, 2.176)),
            material=dark_iron,
            name=f"bearing_block_{0 if x < 0 else 1}",
        )

    roof_angle = math.atan2(0.35, 0.85)
    roof_slope = math.hypot(0.35, 0.85)
    frame.visual(
        Box((1.68, roof_slope, 0.080)),
        origin=Origin(xyz=(0.0, -0.425, 2.840), rpy=(roof_angle, 0.0, 0.0)),
        material=roof_wood,
        name="roof_panel_0",
    )
    frame.visual(
        Box((1.68, roof_slope, 0.080)),
        origin=Origin(xyz=(0.0, 0.425, 2.840), rpy=(-roof_angle, 0.0, 0.0)),
        material=roof_wood,
        name="roof_panel_1",
    )
    frame.visual(
        Cylinder(radius=0.035, length=1.76),
        origin=Origin(xyz=(0.0, 0.0, 3.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=end_grain,
        name="ridge_pole",
    )
    for x, name in [(-0.86, "gable_tie_0"), (0.86, "gable_tie_1")]:
        frame.visual(
            Box((0.055, 1.38, 0.080)),
            origin=Origin(xyz=(x, 0.0, 2.665)),
            material=aged_oak,
            name=name,
        )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.034, length=1.100),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="trunnion_shaft",
    )
    bell.visual(
        Box((0.820, 0.120, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aged_oak,
        name="timber_headstock",
    )
    bell.visual(
        Box((0.040, 0.052, 0.330)),
        origin=Origin(xyz=(-0.135, 0.0, -0.190)),
        material=dark_iron,
        name="hanger_0",
    )
    bell.visual(
        Box((0.040, 0.052, 0.330)),
        origin=Origin(xyz=(0.135, 0.0, -0.190)),
        material=dark_iron,
        name="hanger_1",
    )
    bell.visual(
        Cylinder(radius=0.083, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=bronze,
        name="crown_lug",
    )
    bell.visual(_bell_shell_mesh(), material=bronze, name="bell_shell")
    bell.visual(_bell_rim_mesh(), material=bronze, name="bell_mouth_rim")
    bell.visual(
        Cylinder(radius=0.012, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=dark_iron,
        name="clapper_rod",
    )
    bell.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.640)),
        material=dark_iron,
        name="clapper_ball",
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 2.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.8, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("timber_frame")
    bell = object_model.get_part("bell")
    joint = object_model.get_articulation("bell_swing")

    ctx.check(
        "bell has horizontal swing axis",
        tuple(round(v, 3) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_gap(
        bell,
        frame,
        axis="z",
        positive_elem="trunnion_shaft",
        negative_elem="bearing_block_1",
        min_gap=-0.001,
        max_gap=0.004,
        name="trunnion rests in bearing saddle",
    )
    ctx.expect_gap(
        bell,
        frame,
        axis="z",
        positive_elem="bell_shell",
        negative_elem="platform_deck",
        min_gap=0.45,
        name="bell clears the square platform",
    )

    rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    rest_center_y = None
    if rest_aabb is not None:
        rest_center_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])

    with ctx.pose({joint: 0.55}):
        swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
        swung_center_y = None
        if swung_aabb is not None:
            swung_center_y = 0.5 * (swung_aabb[0][1] + swung_aabb[1][1])
        ctx.check(
            "bell swings about the headstock",
            rest_center_y is not None
            and swung_center_y is not None
            and swung_center_y > rest_center_y + 0.20,
            details=f"rest_y={rest_center_y}, swung_y={swung_center_y}",
        )

    return ctx.report()


object_model = build_object_model()
