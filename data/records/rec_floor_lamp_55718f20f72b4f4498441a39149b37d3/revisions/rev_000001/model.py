from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _weighted_base_geometry() -> LatheGeometry:
    """Low, heavy, slightly bevelled disk like a cast floor-lamp base."""

    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.178, 0.000),
            (0.214, 0.006),
            (0.232, 0.024),
            (0.226, 0.044),
            (0.196, 0.056),
            (0.000, 0.056),
        ],
        segments=96,
    )


def _shade_shell_geometry() -> LatheGeometry:
    """Thin spun-metal conical shade, open at both ends and thick at the lip."""

    shell = LatheGeometry.from_shell_profiles(
        [
            (0.044, 0.000),
            (0.057, 0.030),
            (0.078, 0.150),
            (0.094, 0.252),
        ],
        [
            (0.030, 0.006),
            (0.044, 0.034),
            (0.064, 0.150),
            (0.079, 0.246),
        ],
        segments=88,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    # The lathe axis is local Z; rotate it so the shade points along +X.
    return shell.rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_adjustable_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.025, 0.023, 0.021, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.74, 0.55, 0.30, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.95, 0.73, 0.42, 1.0))
    soft_white = model.material("soft_white", rgba=(0.92, 0.88, 0.78, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.86, 0.48, 0.55))
    warm_light = model.material("warm_light", rgba=(1.0, 0.76, 0.30, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(_weighted_base_geometry(), "weighted_base"),
        material=matte_black,
        name="weighted_base",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(radius=0.206, tube=0.006, radial_segments=18, tubular_segments=96), "rubber_foot"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_rubber,
        name="rubber_foot",
    )
    stand.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=aged_brass,
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.016, length=1.265),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=aged_brass,
        name="upright_stem",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 1.255)),
        material=matte_black,
        name="upper_sleeve",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 1.317)),
        material=aged_brass,
        name="top_socket",
    )
    stand.visual(
        Cylinder(radius=0.0045, length=0.054),
        origin=Origin(xyz=(0.033, 0.0, 1.255), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="set_screw",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=1.33),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )

    arm = model.part("swing_arm")
    arm.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=aged_brass,
        name="swivel_collar",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.402),
        origin=Origin(xyz=(0.231, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="arm_tube",
    )
    arm.visual(
        Box((0.030, 0.126, 0.032)),
        origin=Origin(xyz=(0.405, 0.0, 0.026)),
        material=aged_brass,
        name="yoke_bridge",
    )
    for sign, name in ((-1.0, "yoke_cheek_0"), (1.0, "yoke_cheek_1")):
        arm.visual(
            Box((0.040, 0.016, 0.074)),
            origin=Origin(xyz=(0.440, sign * 0.045, 0.026)),
            material=aged_brass,
            name=name,
        )
    arm.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.456, -0.060, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="hinge_washer_0",
    )
    arm.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.456, 0.060, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="hinge_washer_1",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.50, 0.14, 0.09)),
        mass=0.7,
        origin=Origin(xyz=(0.25, 0.0, 0.025)),
    )

    head = model.part("lamp_head")
    head.visual(
        Cylinder(radius=0.013, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="trunnion_pin",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="shade_neck",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="rear_socket",
    )
    head.visual(
        mesh_from_geometry(_shade_shell_geometry(), "shade_shell"),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=matte_black,
        name="shade_shell",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.087, tube=0.0045, radial_segments=18, tubular_segments=96).rotate_y(math.pi / 2.0), "front_rim"),
        origin=Origin(xyz=(0.342, 0.0, 0.0)),
        material=warm_brass,
        name="front_rim",
    )
    head.visual(
        Cylinder(radius=0.080, length=0.006),
        origin=Origin(xyz=(0.336, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_glass,
        name="diffuser_lens",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.130),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="bulb_socket",
    )
    head.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(0.238, 0.0, 0.0)),
        material=warm_light,
        name="lamp_bulb",
    )
    head.visual(
        Cylinder(radius=0.0135, length=0.022),
        origin=Origin(xyz=(0.188, 0.071, 0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="switch_boss",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.44, 0.20, 0.18)),
        mass=0.9,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
    )

    switch = model.part("switch_knob")
    switch.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.018,
                body_style="faceted",
                top_diameter=0.024,
                base_diameter=0.030,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0007, width=0.0015),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "switch_knob",
        ),
        material=matte_black,
        name="switch_knob",
    )
    switch.inertial = Inertial.from_geometry(Cylinder(radius=0.015, length=0.018), mass=0.03)

    model.articulation(
        "stand_to_arm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=-math.radians(85), upper=math.radians(85)),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        # The child shade points along +X; this pitch makes the rest pose aim
        # slightly downward like a reading lamp.
        origin=Origin(xyz=(0.456, 0.0, 0.026), rpy=(0.0, math.radians(24), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.4, lower=-math.radians(32), upper=math.radians(42)),
    )
    model.articulation(
        "head_to_switch",
        ArticulationType.REVOLUTE,
        parent=head,
        child=switch,
        origin=Origin(xyz=(0.188, 0.082, 0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=math.radians(100)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    arm = object_model.get_part("swing_arm")
    head = object_model.get_part("lamp_head")
    switch = object_model.get_part("switch_knob")
    yaw = object_model.get_articulation("stand_to_arm")
    pitch = object_model.get_articulation("arm_to_head")
    switch_turn = object_model.get_articulation("head_to_switch")

    ctx.expect_gap(
        arm,
        stand,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="swivel_collar",
        negative_elem="top_socket",
        name="swivel collar is seated on top socket",
    )
    ctx.expect_within(
        head,
        arm,
        axes="y",
        inner_elem="trunnion_pin",
        outer_elem="yoke_bridge",
        margin=0.0,
        name="head trunnion sits between yoke sides",
    )
    ctx.expect_gap(
        switch,
        head,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="switch_knob",
        negative_elem="switch_boss",
        name="rotary switch is flush on its boss",
    )

    def elem_center(part, elem_name: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(part, elem=elem_name)
        if box is None:
            return None
        lo, hi = box
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    with ctx.pose({pitch: pitch.motion_limits.lower}):
        raised_lens = elem_center(head, "diffuser_lens")
    with ctx.pose({pitch: pitch.motion_limits.upper}):
        lowered_lens = elem_center(head, "diffuser_lens")
    ctx.check(
        "head pitch aims lamp up and down",
        raised_lens is not None
        and lowered_lens is not None
        and raised_lens[2] > lowered_lens[2] + 0.12,
        details=f"raised={raised_lens}, lowered={lowered_lens}",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({yaw: yaw.motion_limits.upper}):
        yawed_head = ctx.part_world_position(head)
    ctx.check(
        "arm yaw swings the head around the stand",
        rest_head is not None and yawed_head is not None and yawed_head[1] > rest_head[1] + 0.35,
        details=f"rest={rest_head}, yawed={yawed_head}",
    )

    ctx.check(
        "rotary switch has a short residential-lamp throw",
        switch_turn.motion_limits.lower == 0.0
        and switch_turn.motion_limits.upper is not None
        and math.radians(70) < switch_turn.motion_limits.upper < math.radians(130),
        details=str(switch_turn.motion_limits),
    )

    return ctx.report()


object_model = build_object_model()
