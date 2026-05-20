from __future__ import annotations

import math

import cadquery as cq

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


def _centered_cylinder_z(radius: float, length: float) -> cq.Workplane:
    """CadQuery cylinder centered on the local origin and aligned to +Z."""
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))


def _ring_z(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Centered hollow cylindrical ring aligned to +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _cylinder_between_x(radius: float, length: float, z: float = 0.0) -> cq.Workplane:
    """Centered horizontal cylinder along local +X."""
    return _centered_cylinder_z(radius, length).rotate((0, 0, 0), (0, 1, 0), 90).translate((0.0, 0.0, z))


def _base_housing_shape() -> cq.Workplane:
    """Fixed pedestal, hollow yaw bearing housing, ribs, and machined side bosses."""
    base = cq.Workplane("XY").box(0.42, 0.34, 0.026).translate((0.0, 0.0, 0.013))
    base = base.union(_ring_z(0.165, 0.112, 0.050).translate((0.0, 0.0, 0.052)))
    base = base.union(_ring_z(0.146, 0.108, 0.060).translate((0.0, 0.0, 0.096)))
    base = base.union(_ring_z(0.174, 0.112, 0.014).translate((0.0, 0.0, 0.128)))

    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        x = math.cos(angle) * 0.154
        y = math.sin(angle) * 0.154
        rib = cq.Workplane("XY").box(0.108, 0.026, 0.060).translate((0.115, 0.0, 0.056))
        rib = rib.rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))
        base = base.union(rib)
        lug = cq.Workplane("XY").box(0.054, 0.044, 0.020).translate((x, y, 0.029))
        lug = lug.rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))
        base = base.union(lug)

    return base


def _yaw_stage_shape() -> cq.Workplane:
    """Rotating yaw carrier with a bored pitch trunnion yoke."""
    body = _centered_cylinder_z(0.101, 0.042).translate((0.0, 0.0, 0.000))
    body = body.union(_centered_cylinder_z(0.132, 0.022).translate((0.0, 0.0, 0.027)))
    body = body.union(cq.Workplane("XY").box(0.210, 0.140, 0.024).translate((0.0, 0.0, 0.044)))

    cheek_thickness = 0.032
    cheek_center_x = 0.105
    cheek_depth = 0.140
    cheek_height = 0.132
    cheek_center_z = 0.044 + cheek_height / 2.0
    for side in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .box(cheek_thickness, cheek_depth, cheek_height)
            .translate((side * cheek_center_x, 0.0, cheek_center_z))
        )
        body = body.union(cheek)

    rear_bridge = cq.Workplane("XY").box(0.222, 0.024, 0.096).translate((0.0, -0.080, 0.103))
    body = body.union(rear_bridge)

    bore = _cylinder_between_x(0.034, 0.260, z=0.135)
    body = body.cut(bore)
    return body


def _roll_bearing_shape() -> cq.Workplane:
    """Pitch-stage roll bearing ring, oriented in SDK later along the local Y axis."""
    ring = _ring_z(0.053, 0.034, 0.070)
    ring = ring.union(_ring_z(0.062, 0.034, 0.010).translate((0.0, 0.0, 0.040)))
    ring = ring.union(_ring_z(0.062, 0.034, 0.010).translate((0.0, 0.0, -0.040)))
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_yaw_pitch_roll_wrist")

    iron = model.material("blasted_iron", rgba=(0.26, 0.28, 0.30, 1.0))
    steel = model.material("bearing_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    dark = model.material("oxide_black", rgba=(0.05, 0.055, 0.06, 1.0))
    cover = model.material("cover_plate", rgba=(0.18, 0.20, 0.22, 1.0))
    brass = model.material("brass_bushing", rgba=(0.78, 0.62, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_housing_shape(), "base_housing", tolerance=0.0007, angular_tolerance=0.05),
        material=iron,
        name="bearing_housing",
    )
    for index, (x, y, yaw) in enumerate(
        (
            (0.0, 0.162, 0.0),
            (0.0, -0.162, 0.0),
            (0.162, 0.0, math.pi / 2.0),
            (-0.162, 0.0, math.pi / 2.0),
        )
    ):
        base.visual(
            Box((0.086, 0.007, 0.038)),
            origin=Origin(xyz=(x, y, 0.075), rpy=(0.0, 0.0, yaw)),
            material=cover,
            name=f"access_cover_{index}",
        )
    for index, (x, y) in enumerate(
        (
            (0.160, 0.120),
            (-0.160, 0.120),
            (0.160, -0.120),
            (-0.160, -0.120),
        )
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(x, y, 0.0285)),
            material=dark,
            name=f"anchor_bolt_{index}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage", tolerance=0.0007, angular_tolerance=0.05),
        material=iron,
        name="pitch_yoke",
    )
    yaw_stage.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=brass,
        name="yaw_retaining_collar",
    )
    yaw_stage.visual(
        Box((0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.120, 0.040)),
        material=dark,
        name="yaw_index_tab",
    )
    for index in range(10):
        angle = 2.0 * math.pi * index / 10.0
        yaw_stage.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.122 * math.cos(angle), 0.122 * math.sin(angle), 0.015)),
            material=steel,
            name=f"yaw_bearing_roller_{index}",
        )

    pitch_stage = model.part("pitch_stage")
    pitch_stage.visual(
        Cylinder(radius=0.025, length=0.080),
        origin=Origin(xyz=(-0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pitch_trunnion_0",
    )
    pitch_stage.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(-0.136, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="trunnion_collar_0",
    )
    pitch_stage.visual(
        Cylinder(radius=0.025, length=0.080),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pitch_trunnion_1",
    )
    pitch_stage.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.136, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="trunnion_collar_1",
    )
    pitch_stage.visual(
        Box((0.118, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=iron,
        name="saddle_top_web",
    )
    pitch_stage.visual(
        Box((0.118, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=iron,
        name="saddle_lower_web",
    )
    pitch_stage.visual(
        Box((0.018, 0.030, 0.080)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=iron,
        name="saddle_side_web_0",
    )
    pitch_stage.visual(
        Box((0.018, 0.030, 0.080)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=iron,
        name="saddle_side_web_1",
    )
    pitch_stage.visual(
        Box((0.012, 0.024, 0.020)),
        origin=Origin(xyz=(-0.127, 0.052, 0.0)),
        material=brass,
        name="thrust_pad_0",
    )
    pitch_stage.visual(
        Box((0.012, 0.024, 0.020)),
        origin=Origin(xyz=(0.127, 0.052, 0.0)),
        material=brass,
        name="thrust_pad_1",
    )
    pitch_stage.visual(
        mesh_from_cadquery(_roll_bearing_shape(), "roll_bearing", tolerance=0.0006, angular_tolerance=0.05),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="roll_bearing_ring",
    )
    pitch_stage.visual(
        Box((0.060, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.038, 0.050)),
        material=cover,
        name="roll_cover_top",
    )
    pitch_stage.visual(
        Box((0.060, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.038, -0.050)),
        material=cover,
        name="roll_cover_lower",
    )

    roll_stage = model.part("roll_stage")
    roll_stage.visual(
        Cylinder(radius=0.027, length=0.205),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="roll_spindle",
    )
    roll_stage.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, -0.0485, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rear_retaining_collar",
    )
    roll_stage.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_retaining_collar",
    )
    roll_stage.visual(
        Cylinder(radius=0.057, length=0.040),
        origin=Origin(xyz=(0.0, 0.100, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="stepped_output_hub",
    )
    roll_stage.visual(
        Cylinder(radius=0.078, length=0.024),
        origin=Origin(xyz=(0.0, 0.142, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_flange",
    )
    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        roll_stage.visual(
            Cylinder(radius=0.0048, length=0.006),
            origin=Origin(
                xyz=(0.055 * math.cos(angle), 0.157, 0.055 * math.sin(angle)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"flange_bolt_{index}",
        )
    roll_stage.visual(
        Box((0.020, 0.009, 0.012)),
        origin=Origin(xyz=(0.066, 0.158, 0.0)),
        material=dark,
        name="flange_index_lug",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=1.2, lower=-2.75, upper=2.75),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=190.0, velocity=1.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pitch_stage,
        child=roll_stage,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-3.05, upper=3.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    yaw = object_model.get_articulation("yaw_axis")
    pitch = object_model.get_articulation("pitch_axis")
    roll = object_model.get_articulation("roll_axis")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_stage = object_model.get_part("pitch_stage")
    roll_stage = object_model.get_part("roll_stage")

    def axis_close(actual, expected) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))

    ctx.check("three nested revolute axes", len(object_model.articulations) == 3)
    ctx.check("yaw axis is vertical", axis_close(yaw.axis, (0.0, 0.0, 1.0)))
    ctx.check("pitch axis crosses yaw carrier", axis_close(pitch.axis, (1.0, 0.0, 0.0)))
    ctx.check("roll axis is coaxial with flange", axis_close(roll.axis, (0.0, 1.0, 0.0)))
    ctx.expect_origin_distance(yaw_stage, "base", axes="xy", max_dist=0.001, name="yaw stage centered over bearing")
    ctx.expect_overlap(
        pitch_stage,
        yaw_stage,
        axes="x",
        elem_a="pitch_trunnion_0",
        elem_b="pitch_yoke",
        min_overlap=0.025,
        name="negative pitch trunnion enters yoke cheek",
    )
    ctx.expect_overlap(
        pitch_stage,
        yaw_stage,
        axes="x",
        elem_a="pitch_trunnion_1",
        elem_b="pitch_yoke",
        min_overlap=0.025,
        name="positive pitch trunnion enters yoke cheek",
    )
    ctx.expect_within(
        roll_stage,
        pitch_stage,
        axes="xz",
        inner_elem="roll_spindle",
        outer_elem="roll_bearing_ring",
        margin=0.001,
        name="roll spindle fits through bearing bore projection",
    )
    ctx.allow_overlap(
        pitch_stage,
        yaw_stage,
        elem_a="thrust_pad_0",
        elem_b="pitch_yoke",
        reason="The bronze thrust pad is intentionally seated in the pitch-yoke cheek pocket to represent a preloaded bearing contact.",
    )
    ctx.allow_overlap(
        pitch_stage,
        yaw_stage,
        elem_a="thrust_pad_1",
        elem_b="pitch_yoke",
        reason="The opposite bronze thrust pad is intentionally seated in the pitch-yoke cheek pocket to represent a preloaded bearing contact.",
    )
    ctx.expect_gap(
        yaw_stage,
        pitch_stage,
        axis="x",
        positive_elem="pitch_yoke",
        negative_elem="thrust_pad_0",
        max_penetration=0.012,
        name="negative thrust pad is only locally seated",
    )
    ctx.expect_gap(
        pitch_stage,
        yaw_stage,
        axis="x",
        positive_elem="thrust_pad_1",
        negative_elem="pitch_yoke",
        max_penetration=0.012,
        name="positive thrust pad is only locally seated",
    )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        low, high = bounds
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    yaw_tab_rest = element_center(yaw_stage, "yaw_index_tab")
    with ctx.pose({yaw: 0.80}):
        yaw_tab_turned = element_center(yaw_stage, "yaw_index_tab")
    ctx.check(
        "yaw joint visibly slews carrier",
        yaw_tab_rest is not None
        and yaw_tab_turned is not None
        and abs(yaw_tab_turned[0] - yaw_tab_rest[0]) > 0.050,
        details=f"rest={yaw_tab_rest}, turned={yaw_tab_turned}",
    )

    flange_rest = element_center(roll_stage, "wrist_flange")
    with ctx.pose({pitch: 0.75}):
        flange_pitched = element_center(roll_stage, "wrist_flange")
    ctx.check(
        "pitch joint raises output flange",
        flange_rest is not None
        and flange_pitched is not None
        and flange_pitched[2] > flange_rest[2] + 0.050,
        details=f"rest={flange_rest}, pitched={flange_pitched}",
    )

    lug_rest = element_center(roll_stage, "flange_index_lug")
    with ctx.pose({roll: 1.0}):
        lug_rolled = element_center(roll_stage, "flange_index_lug")
    ctx.check(
        "roll joint spins flange index lug",
        lug_rest is not None and lug_rolled is not None and lug_rolled[2] < lug_rest[2] - 0.030,
        details=f"rest={lug_rest}, rolled={lug_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
