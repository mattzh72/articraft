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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHOULDER_HEIGHT = 0.30
UPPER_LENGTH = 0.32
FOREARM_LENGTH = 0.54
SLEEVE_CENTER_X = 0.58
SLEEVE_LENGTH = 0.18
SLEEVE_FRONT_X = SLEEVE_CENTER_X + SLEEVE_LENGTH / 2.0
TOOL_TRAVEL = 0.12


def _x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder primitives are local-Z aligned; rotate them onto local +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder primitives are local-Z aligned; rotate them onto local +Y."""
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _slide_sleeve_mesh():
    """Thin-walled tube with an open bore for the telescoping inspection shaft."""
    outer = [0.034, 0.034]
    inner = [0.024, 0.024]
    half = SLEEVE_LENGTH / 2.0
    sleeve = LatheGeometry.from_shell_profiles(
        [(outer[0], -half), (outer[1], half)],
        [(inner[0], -half), (inner[1], half)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    sleeve.rotate_y(math.pi / 2.0)
    sleeve.translate(SLEEVE_CENTER_X, 0.0, 0.0)
    return sleeve


def _x_oriented_tube_mesh(
    *,
    center_x: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
    name_segments: int = 48,
):
    half = length / 2.0
    tube = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=name_segments,
        start_cap="flat",
        end_cap="flat",
    )
    tube.rotate_y(math.pi / 2.0)
    tube.translate(center_x, 0.0, 0.0)
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_telescoping_robot_arm")

    dark_cast = model.material("dark_cast", rgba=(0.09, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.017, 0.020, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.36, 0.05, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    lens = model.material("smoked_lens", rgba=(0.02, 0.04, 0.05, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.16, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_cast,
        name="round_base",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=satin_black,
        name="base_recess",
    )
    pedestal.visual(
        Cylinder(radius=0.042, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
        material=dark_cast,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.112, 0.128, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT - 0.060)),
        material=dark_cast,
        name="shoulder_block",
    )
    pedestal.visual(
        Box((0.082, 0.018, 0.142)),
        origin=Origin(xyz=(0.0, -0.071, SHOULDER_HEIGHT)),
        material=dark_cast,
        name="shoulder_cheek_0",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=_y_cylinder_origin((0.0, -0.071 * 1.12, SHOULDER_HEIGHT)),
        material=steel,
        name="shoulder_bearing_0",
    )
    pedestal.visual(
        Box((0.082, 0.018, 0.142)),
        origin=Origin(xyz=(0.0, 0.071, SHOULDER_HEIGHT)),
        material=dark_cast,
        name="shoulder_cheek_1",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=_y_cylinder_origin((0.0, 0.071 * 1.12, SHOULDER_HEIGHT)),
        material=steel,
        name="shoulder_bearing_1",
    )
    for index, (x, y) in enumerate(
        ((0.095, 0.095), (-0.095, 0.095), (-0.095, -0.095), (0.095, -0.095))
    ):
        pedestal.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x, y, 0.041)),
            material=steel,
            name=f"base_bolt_{index}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.028, length=0.124),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_barrel",
    )
    for index, y in enumerate((-0.048, 0.048)):
        upper_link.visual(
            Box((0.306, 0.016, 0.046)),
            origin=Origin(xyz=(UPPER_LENGTH / 2.0, y, 0.0)),
            material=orange,
            name=f"side_plate_{index}",
        )
    upper_link.visual(
        Box((0.180, 0.080, 0.018)),
        origin=Origin(xyz=(0.155, 0.0, 0.025)),
        material=graphite,
        name="top_rib",
    )
    upper_link.visual(
        Box((0.180, 0.080, 0.014)),
        origin=Origin(xyz=(0.155, 0.0, -0.026)),
        material=graphite,
        name="lower_rib",
    )
    for index, y in enumerate((-0.048, 0.048)):
        upper_link.visual(
            Box((0.068, 0.016, 0.102)),
            origin=Origin(xyz=(UPPER_LENGTH, y, 0.0)),
            material=orange,
            name=f"elbow_cheek_{index}",
        )
        upper_link.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=_y_cylinder_origin((UPPER_LENGTH, y * 1.18, 0.0)),
            material=steel,
            name=f"elbow_bearing_{index}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.026, length=0.080),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.490, 0.054, 0.050)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=graphite,
        name="forearm_box",
    )
    forearm.visual(
        Box((0.435, 0.072, 0.016)),
        origin=Origin(xyz=(0.260, 0.0, 0.032)),
        material=orange,
        name="upper_cover",
    )
    forearm.visual(
        Box((0.435, 0.072, 0.016)),
        origin=Origin(xyz=(0.260, 0.0, -0.032)),
        material=orange,
        name="lower_cover",
    )
    forearm.visual(
        Box((0.075, 0.090, 0.090)),
        origin=Origin(xyz=(0.462, 0.0, 0.0)),
        material=dark_cast,
        name="nose_receiver",
    )
    forearm.visual(
        mesh_from_geometry(_slide_sleeve_mesh(), "cylindrical_slide_sleeve"),
        material=steel,
        name="slide_sleeve",
    )
    forearm.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=_x_cylinder_origin((SLEEVE_CENTER_X - 0.098, 0.0, 0.0)),
        material=dark_cast,
        name="rear_collar",
    )
    forearm.visual(
        mesh_from_geometry(
            _x_oriented_tube_mesh(
                center_x=SLEEVE_FRONT_X,
                length=0.020,
                outer_radius=0.038,
                inner_radius=0.024,
            ),
            "slide_front_lip",
        ),
        material=steel,
        name="front_lip",
    )

    tool_tip = model.part("tool_tip")
    tool_tip.visual(
        Cylinder(radius=0.017, length=0.270),
        origin=_x_cylinder_origin((-0.015, 0.0, 0.0)),
        material=steel,
        name="probe_shaft",
    )
    tool_tip.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=_x_cylinder_origin((-0.115, 0.0, 0.0)),
        material=steel,
        name="slider_bushing",
    )
    tool_tip.visual(
        Box((0.030, 0.086, 0.086)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=graphite,
        name="square_flange",
    )
    tool_tip.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=_x_cylinder_origin((0.149, 0.0, 0.0)),
        material=lens,
        name="inspection_window",
    )
    for index, (y, z) in enumerate(
        ((0.031, 0.031), (-0.031, 0.031), (-0.031, -0.031), (0.031, -0.031))
    ):
        tool_tip.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=_x_cylinder_origin((0.148, y, z)),
            material=black_rubber,
            name=f"flange_screw_{index}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-0.55, upper=1.10),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.0, lower=-1.15, upper=1.35),
    )
    model.articulation(
        "tool_slide",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=tool_tip,
        origin=Origin(xyz=(SLEEVE_FRONT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.18, lower=0.0, upper=TOOL_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    tool_tip = object_model.get_part("tool_tip")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    tool_slide = object_model.get_articulation("tool_slide")

    ctx.allow_overlap(
        forearm,
        tool_tip,
        elem_a="slide_sleeve",
        elem_b="slider_bushing",
        reason=(
            "The hidden slider bushing is intentionally represented as a captured "
            "bearing inside the cylindrical sleeve proxy for the prismatic inspection tip."
        ),
    )

    ctx.check(
        "compact arm has requested joint chain",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tool_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={[shoulder.articulation_type, elbow.articulation_type, tool_slide.articulation_type]}",
    )
    ctx.check(
        "forearm is longer than upper link",
        FOREARM_LENGTH > UPPER_LENGTH * 1.4,
        details=f"upper={UPPER_LENGTH}, forearm={FOREARM_LENGTH}",
    )

    ctx.expect_overlap(
        tool_tip,
        forearm,
        axes="x",
        elem_a="probe_shaft",
        elem_b="slide_sleeve",
        min_overlap=0.12,
        name="collapsed probe remains deeply inserted in sleeve",
    )
    ctx.expect_within(
        tool_tip,
        forearm,
        axes="yz",
        inner_elem="probe_shaft",
        outer_elem="slide_sleeve",
        margin=0.001,
        name="probe shaft stays centered in cylindrical sleeve",
    )
    ctx.expect_overlap(
        tool_tip,
        forearm,
        axes="x",
        elem_a="slider_bushing",
        elem_b="slide_sleeve",
        min_overlap=0.035,
        name="hidden bushing is captured in sleeve at rest",
    )
    ctx.expect_within(
        tool_tip,
        forearm,
        axes="yz",
        inner_elem="slider_bushing",
        outer_elem="slide_sleeve",
        margin=0.001,
        name="hidden bushing stays centered in sleeve",
    )
    ctx.expect_gap(
        tool_tip,
        forearm,
        axis="x",
        positive_elem="square_flange",
        negative_elem="slide_sleeve",
        min_gap=0.09,
        name="square flange sits beyond the slide nose",
    )

    rest_tip = ctx.part_world_position(tool_tip)
    with ctx.pose({tool_slide: TOOL_TRAVEL}):
        ctx.expect_overlap(
            tool_tip,
            forearm,
            axes="x",
            elem_a="probe_shaft",
            elem_b="slide_sleeve",
            min_overlap=0.025,
            name="extended probe retains insertion in sleeve",
        )
        ctx.expect_within(
            tool_tip,
            forearm,
            axes="yz",
            inner_elem="probe_shaft",
            outer_elem="slide_sleeve",
            margin=0.001,
            name="extended probe stays coaxial with sleeve",
        )
        ctx.expect_overlap(
            tool_tip,
            forearm,
            axes="x",
            elem_a="slider_bushing",
            elem_b="slide_sleeve",
            min_overlap=0.010,
            name="extended bushing remains retained in sleeve",
        )
        extended_tip = ctx.part_world_position(tool_tip)
    ctx.check(
        "tool slide extends along forearm axis",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.10,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    def center_z(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    upper_rest_z = center_z(upper_link)
    with ctx.pose({shoulder: 0.70}):
        upper_raised_z = center_z(upper_link)
    ctx.check(
        "positive shoulder motion lifts short upper link",
        upper_rest_z is not None and upper_raised_z is not None and upper_raised_z > upper_rest_z + 0.06,
        details=f"rest_z={upper_rest_z}, raised_z={upper_raised_z}",
    )

    forearm_rest_z = center_z(forearm)
    with ctx.pose({elbow: 0.80}):
        forearm_raised_z = center_z(forearm)
    ctx.check(
        "positive elbow motion lifts longer forearm",
        forearm_rest_z is not None and forearm_raised_z is not None and forearm_raised_z > forearm_rest_z + 0.10,
        details=f"rest_z={forearm_rest_z}, raised_z={forearm_raised_z}",
    )

    ctx.expect_overlap(
        upper_link,
        pedestal,
        axes="y",
        elem_a="shoulder_barrel",
        elem_b="shoulder_cheek_0",
        min_overlap=0.0,
        name="shoulder barrel sits between supported cheeks",
    )

    return ctx.report()


object_model = build_object_model()
