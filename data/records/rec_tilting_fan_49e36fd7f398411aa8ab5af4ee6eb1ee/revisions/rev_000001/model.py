from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_points(
    radius: float,
    *,
    x: float = 0.0,
    count: int = 48,
) -> list[tuple[float, float, float]]:
    """Circle in the local YZ plane, used for fan guard wire hoops."""
    return [
        (
            x,
            radius * math.cos(2.0 * math.pi * i / count),
            radius * math.sin(2.0 * math.pi * i / count),
        )
        for i in range(count)
    ]


def _guard_ring(radius: float, *, x: float, wire_radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            _circle_points(radius, x=x),
            radius=wire_radius,
            samples_per_segment=4,
            closed_spline=True,
            radial_segments=12,
            cap_ends=False,
        ),
        name,
    )


def _guard_spoke(angle: float, *, x: float, inner: float, outer: float, name: str):
    points = [
        (x, inner * math.cos(angle), inner * math.sin(angle)),
        (x, outer * math.cos(angle), outer * math.sin(angle)),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=0.0032,
            samples_per_segment=2,
            closed_spline=False,
            radial_segments=10,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    dark_plastic = Material("charcoal_plastic", rgba=(0.035, 0.038, 0.045, 1.0))
    soft_black = Material("rubber_feet", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_metal = Material("brushed_guard_wire", rgba=(0.72, 0.74, 0.76, 1.0))
    blade_blue = Material("translucent_smoke_blade", rgba=(0.28, 0.45, 0.68, 0.78))
    motor_gray = Material("warm_motor_gray", rgba=(0.48, 0.50, 0.52, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.185, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_plastic,
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=0.150, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=soft_black,
        name="foot_ring",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.355),
        origin=Origin(xyz=(0.02, 0.0, 0.2125)),
        material=dark_plastic,
        name="pedestal",
    )
    stand.visual(
        Cylinder(radius=0.025, length=0.56),
        origin=Origin(xyz=(0.02, 0.0, 0.390), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="yoke_bridge",
    )
    for side, y in (("side_0", 0.2925), ("side_1", -0.2925)):
        stand.visual(
            Box((0.038, 0.030, 0.300)),
            origin=Origin(xyz=(0.02, y, 0.520)),
            material=dark_plastic,
            name=f"{side}_bracket",
        )
        stand.visual(
            Cylinder(radius=0.048, length=0.035),
            origin=Origin(xyz=(0.02, y, 0.670), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name=f"{side}_pivot_pad",
        )
        stand.visual(
            Cylinder(radius=0.018, length=0.042),
            origin=Origin(xyz=(0.02, y, 0.670), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=f"{side}_pivot_bushing",
        )

    head = model.part("head")
    # Fixed wire cage: much broader than the spinning rotor, with front and rear hoops.
    for radius, wire, label in (
        (0.250, 0.0062, "outer"),
        (0.190, 0.0040, "middle"),
        (0.118, 0.0035, "inner"),
    ):
        head.visual(
            _guard_ring(radius, x=-0.055, wire_radius=wire, name=f"front_{label}_ring_mesh"),
            material=satin_metal,
            name=f"front_{label}_ring",
        )
    head.visual(
        _guard_ring(0.250, x=0.050, wire_radius=0.0062, name="rear_outer_ring_mesh"),
        material=satin_metal,
        name="rear_outer_ring",
    )
    head.visual(
        _guard_ring(0.145, x=0.048, wire_radius=0.0040, name="rear_motor_ring_mesh"),
        material=satin_metal,
        name="rear_motor_ring",
    )
    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        head.visual(
            _guard_spoke(angle, x=-0.055, inner=0.052, outer=0.247, name=f"front_spoke_{i}_mesh"),
            material=satin_metal,
            name=f"front_spoke_{i}",
        )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        y = 0.248 * math.cos(angle)
        z = 0.248 * math.sin(angle)
        head.visual(
            Cylinder(radius=0.0050, length=0.105),
            origin=Origin(xyz=(-0.0025, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"cage_depth_stay_{i}",
        )
    for i in range(6):
        angle = math.pi / 6.0 + 2.0 * math.pi * i / 6.0
        head.visual(
            _guard_spoke(angle, x=0.050, inner=0.078, outer=0.247, name=f"rear_motor_strut_{i}_mesh"),
            material=satin_metal,
            name=f"rear_motor_strut_{i}",
        )

    head.visual(
        Cylinder(radius=0.083, length=0.130),
        origin=Origin(xyz=(0.118, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.045),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="front_bearing_nose",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.020, 0.262, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="side_pivot_0",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.020, -0.262, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="side_pivot_1",
    )
    head.visual(
        Cylinder(radius=0.053, length=0.018),
        origin=Origin(xyz=(-0.057, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="front_badge_ring",
    )

    rotor = model.part("rotor")
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=0.155,
            hub_radius=0.040,
            blade_count=5,
            thickness=0.024,
            blade_pitch_deg=32.0,
            blade_sweep_deg=26.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.14),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.012, rear_collar_radius=0.030),
        ),
        "five_blade_rotor_mesh",
    )
    rotor.visual(
        rotor_mesh,
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_blue,
        name="blade_wheel",
    )
    rotor.visual(
        Cylinder(radius=0.014, length=0.054),
        origin=Origin(xyz=(0.0255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="axle_stub",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.02, 0.0, 0.670)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("head_tilt")
    spin = object_model.get_articulation("rotor_spin")

    ctx.allow_overlap(
        rotor,
        head,
        elem_a="axle_stub",
        elem_b="front_bearing_nose",
        reason="The rotating axle stub is intentionally captured inside the fixed front bearing nose.",
    )

    ctx.check(
        "head tilts on horizontal bracket axis",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in tilt.axis) == (0.0, 1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.0,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
    )
    ctx.check(
        "rotor is continuous about fan axle",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        margin=0.010,
        inner_elem="blade_wheel",
        outer_elem="front_outer_ring",
        name="rotor sits inside broad guard circle",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        min_gap=0.012,
        positive_elem="blade_wheel",
        negative_elem="front_outer_ring",
        name="rotor clears front guard wires",
    )
    ctx.expect_contact(
        stand,
        head,
        elem_a="side_0_pivot_pad",
        elem_b="side_pivot_0",
        contact_tol=0.002,
        name="head is carried by side bracket pivot",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="axle_stub",
        outer_elem="front_bearing_nose",
        margin=0.0,
        name="rotor axle is centered in bearing",
    )
    ctx.expect_overlap(
        rotor,
        head,
        axes="x",
        elem_a="axle_stub",
        elem_b="front_bearing_nose",
        min_overlap=0.006,
        name="rotor axle remains inserted in bearing",
    )

    head_box = ctx.part_world_aabb(head)
    rotor_box = ctx.part_element_world_aabb(rotor, elem="blade_wheel")
    if head_box is not None and rotor_box is not None:
        head_y = head_box[1][1] - head_box[0][1]
        rotor_y = rotor_box[1][1] - rotor_box[0][1]
        ctx.check(
            "rotating member smaller than fixed support",
            rotor_y < 0.72 * head_y,
            details=f"rotor_y={rotor_y:.3f}, head_y={head_y:.3f}",
        )

    front_rest = ctx.part_element_world_aabb(head, elem="front_outer_ring")
    with ctx.pose({tilt: 0.45}):
        front_tilted = ctx.part_element_world_aabb(head, elem="front_outer_ring")
    if front_rest is not None and front_tilted is not None:
        rest_center_z = 0.5 * (front_rest[0][2] + front_rest[1][2])
        tilted_center_z = 0.5 * (front_tilted[0][2] + front_tilted[1][2])
        ctx.check(
            "positive tilt raises fan face",
            tilted_center_z > rest_center_z + 0.010,
            details=f"rest_z={rest_center_z:.3f}, tilted_z={tilted_center_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
