from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """A cylinder descriptor and origin rotation for a barrel running along Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_bar(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    y: float,
    width: float,
    thickness: float,
    material: Material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(
            xyz=((start[0] + end[0]) / 2.0, y, (start[2] + end[2]) / 2.0),
            rpy=(0.0, -angle, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_table_scissor_lamp")

    black = Material("satin_black_powdercoat", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = Material("dark_graphite_arms", rgba=(0.055, 0.060, 0.064, 1.0))
    steel = Material("brushed_steel_pivots", rgba=(0.58, 0.60, 0.60, 1.0))
    rubber = Material("black_rubber_pads", rgba=(0.006, 0.006, 0.006, 1.0))
    warm = Material("warm_diffuser_glow", rgba=(1.0, 0.78, 0.34, 0.72))
    white = Material("matte_white_reflector", rgba=(0.92, 0.90, 0.84, 1.0))
    wood = Material("drafting_table_wood", rgba=(0.50, 0.34, 0.18, 1.0))

    # Root: a table-edge clamp, vertical riser, and the fixed hinge pins.
    base = model.part("table_clamp")
    base.visual(
        Box((1.30, 0.62, 0.035)),
        origin=Origin(xyz=(0.40, 0.0, -0.222)),
        material=wood,
        name="table_edge",
    )
    base.visual(
        Box((0.13, 0.12, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.1955)),
        material=black,
        name="top_clamp_plate",
    )
    base.visual(
        Box((0.025, 0.12, 0.17)),
        origin=Origin(xyz=(-0.075, 0.0, -0.285)),
        material=black,
        name="clamp_spine",
    )
    base.visual(
        Box((0.11, 0.10, 0.018)),
        origin=Origin(xyz=(-0.020, 0.0, -0.351)),
        material=black,
        name="lower_clamp_jaw",
    )
    base.visual(
        Box((0.080, 0.080, 0.010)),
        origin=Origin(xyz=(-0.005, 0.0, -0.347)),
        material=rubber,
        name="rubber_pressure_pad",
    )
    screw_geom, screw_rot = _cyl_y(0.012, 0.090)
    base.visual(
        screw_geom,
        origin=Origin(xyz=(-0.020, 0.0, -0.335), rpy=screw_rot.rpy),
        material=steel,
        name="clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=black,
        name="vertical_riser",
    )
    lower_pin_geom, lower_pin_rot = _cyl_y(0.030, 0.124)
    base.visual(
        lower_pin_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=lower_pin_rot.rpy),
        material=steel,
        name="lower_base_pin",
    )
    upper_pin_geom, upper_pin_rot = _cyl_y(0.028, 0.104)
    base.visual(
        upper_pin_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.115), rpy=upper_pin_rot.rpy),
        material=steel,
        name="upper_parked_pin",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=black,
        name="upper_pin_stanchion",
    )

    lower_len = 0.56
    lower_angle = math.radians(62.0)
    elbow = (
        lower_len * math.cos(lower_angle),
        0.0,
        lower_len * math.sin(lower_angle),
    )

    # The lower assembly is a pair of flat side links joined by hinge barrels and
    # a visible extension spring.  Its part frame sits on the lower base pivot.
    lower_arm = model.part("lower_arm")
    for bar_name, bushing_name, y in (
        ("lower_side_bar_0", "base_bushing_0", -0.070),
        ("lower_side_bar_1", "base_bushing_1", 0.070),
    ):
        _add_bar(
            lower_arm,
            start=(0.0, y, 0.0),
            end=(elbow[0], y, elbow[2]),
            y=y,
            width=0.016,
            thickness=0.020,
            material=graphite,
            name=bar_name,
        )
        bushing_geom, bushing_rot = _cyl_y(0.032, 0.016)
        lower_arm.visual(
            bushing_geom,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=bushing_rot.rpy),
            material=graphite,
            name=bushing_name,
        )
    elbow_pin_geom, elbow_pin_rot = _cyl_y(0.030, 0.164)
    lower_arm.visual(
        elbow_pin_geom,
        origin=Origin(xyz=elbow, rpy=elbow_pin_rot.rpy),
        material=steel,
        name="elbow_pin",
    )
    lower_arm.visual(
        Cylinder(radius=0.008, length=lower_len * 0.80),
        origin=Origin(
            xyz=(elbow[0] * 0.50, 0.080, elbow[2] * 0.50 + 0.012),
            rpy=(0.0, math.pi / 2.0 - lower_angle, 0.0),
        ),
        material=steel,
        name="lower_tension_spring",
    )

    upper_len = 0.52
    upper_angle = math.radians(-28.0)
    head_pivot = (
        upper_len * math.cos(upper_angle),
        0.0,
        upper_len * math.sin(upper_angle),
    )

    upper_arm = model.part("upper_arm")
    for bar_name, bushing_name, y in (
        ("upper_side_bar_0", "elbow_bushing_0", -0.090),
        ("upper_side_bar_1", "elbow_bushing_1", 0.090),
    ):
        _add_bar(
            upper_arm,
            start=(0.0, y, 0.0),
            end=(head_pivot[0], y, head_pivot[2]),
            y=y,
            width=0.016,
            thickness=0.020,
            material=graphite,
            name=bar_name,
        )
        elbow_bushing_geom, elbow_bushing_rot = _cyl_y(0.032, 0.016)
        upper_arm.visual(
            elbow_bushing_geom,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=elbow_bushing_rot.rpy),
            material=graphite,
            name=bushing_name,
        )
    head_pin_geom, head_pin_rot = _cyl_y(0.028, 0.214)
    upper_arm.visual(
        head_pin_geom,
        origin=Origin(xyz=head_pivot, rpy=head_pin_rot.rpy),
        material=steel,
        name="head_pin",
    )
    upper_arm.visual(
        Cylinder(radius=0.007, length=upper_len * 0.72),
        origin=Origin(
            xyz=(head_pivot[0] * 0.50, -0.104, head_pivot[2] * 0.50 - 0.010),
            rpy=(0.0, math.pi / 2.0 - upper_angle, 0.0),
        ),
        material=steel,
        name="upper_tension_spring",
    )

    neck = model.part("shade_yoke")
    for lug_name, cheek_name, y in (
        ("head_lug_0", "yoke_cheek_0", -0.115),
        ("head_lug_1", "yoke_cheek_1", 0.115),
    ):
        lug_geom, lug_rot = _cyl_y(0.030, 0.016)
        neck.visual(
            lug_geom,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=lug_rot.rpy),
            material=graphite,
            name=lug_name,
        )
        neck.visual(
            Box((0.020, 0.016, 0.085)),
            origin=Origin(xyz=(0.0, y, -0.042)),
            material=graphite,
            name=cheek_name,
        )
    neck.visual(
        Box((0.020, 0.246, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=graphite,
        name="yoke_cross_bridge",
    )
    neck.visual(
        Cylinder(radius=0.010, length=0.051),
        origin=Origin(xyz=(0.0, 0.0, -0.0655)),
        material=steel,
        name="swivel_stem",
    )
    neck.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=steel,
        name="swivel_socket",
    )

    shade = model.part("shade_head")
    shade.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="top_collar",
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=black,
        name="collar_post",
    )
    shade.visual(
        Box((0.280, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=black,
        name="shade_top_panel",
    )
    shade.visual(
        Box((0.280, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.079, -0.089)),
        material=black,
        name="shade_side_wall_0",
    )
    shade.visual(
        Box((0.280, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, -0.079, -0.089)),
        material=black,
        name="shade_side_wall_1",
    )
    shade.visual(
        Box((0.012, 0.170, 0.070)),
        origin=Origin(xyz=(0.134, 0.0, -0.089)),
        material=black,
        name="shade_end_wall_0",
    )
    shade.visual(
        Box((0.012, 0.170, 0.070)),
        origin=Origin(xyz=(-0.134, 0.0, -0.089)),
        material=black,
        name="shade_end_wall_1",
    )
    shade.visual(
        Box((0.268, 0.158, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=warm,
        name="warm_diffuser",
    )
    shade.visual(
        Box((0.250, 0.140, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=white,
        name="white_reflector_lip",
    )

    base_joint = model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.45, upper=0.45),
    )
    elbow_joint = model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=elbow),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-0.70, upper=0.95),
    )
    head_tilt = model.articulation(
        "upper_arm_to_yoke",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=neck,
        origin=Origin(xyz=head_pivot),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.6, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "yoke_to_shade_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.109)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-1.35, upper=1.35),
    )

    model.meta["nominal_pose"] = {
        base_joint.name: 0.0,
        elbow_joint.name: 0.0,
        head_tilt.name: 0.0,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade_head = object_model.get_part("shade_head")
    table_clamp = object_model.get_part("table_clamp")
    shade_yoke = object_model.get_part("shade_yoke")
    base_joint = object_model.get_articulation("base_to_lower_arm")
    elbow_joint = object_model.get_articulation("lower_to_upper_arm")
    yoke_joint = object_model.get_articulation("upper_arm_to_yoke")
    swivel_joint = object_model.get_articulation("yoke_to_shade_head")

    ctx.check(
        "four user facing revolute adjustments",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (base_joint, elbow_joint, yoke_joint, swivel_joint)
        ),
        details="The arm should use paired pivots and a swiveling shade head.",
    )

    ctx.expect_contact(
        table_clamp,
        lower_arm,
        elem_a="lower_base_pin",
        elem_b="base_bushing_0",
        name="lower arm bushing seats on fixed base pin",
    )
    ctx.expect_contact(
        lower_arm,
        upper_arm,
        elem_a="elbow_pin",
        elem_b="elbow_bushing_0",
        name="upper arm bushing seats on elbow pin",
    )
    ctx.expect_contact(
        upper_arm,
        shade_yoke,
        elem_a="head_pin",
        elem_b="head_lug_0",
        name="shade yoke lug seats on head pin",
    )
    ctx.expect_contact(
        shade_yoke,
        shade_head,
        elem_a="swivel_socket",
        elem_b="top_collar",
        name="shade collar seats below swivel socket",
    )

    ctx.expect_overlap(
        shade_head,
        table_clamp,
        axes="x",
        min_overlap=0.10,
        name="rectangular head reaches out over drafting table",
    )

    rest_pos = ctx.part_world_position(shade_head)
    with ctx.pose({base_joint: 0.32, elbow_joint: -0.38, yoke_joint: 0.25, swivel_joint: 0.75}):
        posed_pos = ctx.part_world_position(shade_head)
        ctx.expect_origin_gap(
            shade_head,
            table_clamp,
            axis="z",
            min_gap=0.25,
            name="posed shade remains above table clamp",
        )

    ctx.check(
        "arm pose changes shade position",
        rest_pos is not None
        and posed_pos is not None
        and abs(posed_pos[0] - rest_pos[0]) > 0.04,
        details=f"rest={rest_pos}, posed={posed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
