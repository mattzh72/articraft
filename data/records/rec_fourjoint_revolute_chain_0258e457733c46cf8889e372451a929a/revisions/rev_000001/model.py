from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


AXIS_Y = (0.0, -1.0, 0.0)


def _y_cyl(x: float, y: float, z: float) -> Origin:
    """Cylinder origin with the primitive's local Z axis laid along the arm hinge axis."""
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _x_cyl(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _add_side_washer_stack(
    part,
    *,
    x: float,
    z: float,
    pin_radius: float,
    pin_length: float,
    washer_radius: float,
    washer_thickness: float,
    cap_radius: float,
    cap_thickness: float,
    name_prefix: str,
    metal,
    screw,
) -> None:
    """Visible captured hinge pin with outer washers and cap-screw heads."""
    part.visual(
        Cylinder(radius=pin_radius, length=pin_length),
        origin=_y_cyl(x, 0.0, z),
        material=metal,
        name=f"{name_prefix}_pin",
    )
    side_y = pin_length / 2.0 + washer_thickness / 2.0 - 0.001
    cap_y = side_y + washer_thickness / 2.0 + cap_thickness / 2.0 - 0.001
    for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
        part.visual(
            Cylinder(radius=washer_radius, length=washer_thickness),
            origin=_y_cyl(x, sign * side_y, z),
            material=metal,
            name=f"{name_prefix}_{suffix}_washer",
        )
        part.visual(
            Cylinder(radius=cap_radius, length=cap_thickness),
            origin=_y_cyl(x, sign * cap_y, z),
            material=screw,
            name=f"{name_prefix}_{suffix}_cap_screw",
        )


def _add_link(
    part,
    *,
    length: float,
    beam_size: tuple[float, float],
    hub_radius: float,
    hub_length: float,
    name_prefix: str,
    material,
    metal,
    has_distal_fork: bool,
    fork_gap: float = 0.0,
    fork_plate_thickness: float = 0.0,
    fork_x: float = 0.0,
    fork_z: float = 0.0,
    distal_pin_radius: float = 0.0,
    distal_pin_length: float = 0.0,
    washer_radius: float = 0.0,
    washer_thickness: float = 0.0,
    screw,
) -> None:
    beam_y, beam_z = beam_size
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=_y_cyl(0.0, 0.0, 0.0),
        material=metal,
        name=f"{name_prefix}_hub",
    )

    beam_start = hub_radius + 0.018
    beam_end = length - (fork_x / 2.0 + 0.012 if has_distal_fork else 0.030)
    beam_length = beam_end - beam_start
    part.visual(
        Box((beam_length, beam_y, beam_z)),
        origin=Origin(xyz=(beam_start + beam_length / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_beam",
    )
    strap_z = hub_radius * 0.32
    for sign, suffix in ((1.0, "top"), (-1.0, "bottom")):
        part.visual(
            Box((beam_start + 0.025, min(beam_y, hub_length * 0.82), strap_z)),
            origin=Origin(xyz=(beam_start / 2.0, 0.0, sign * hub_radius * 0.72)),
            material=metal,
            name=f"{name_prefix}_{suffix}_hub_neck",
        )
    part.visual(
        Box((beam_length * 0.92, beam_y * 0.32, beam_z * 0.28)),
        origin=Origin(xyz=(beam_start + beam_length / 2.0, 0.0, beam_z * 0.45)),
        material=metal,
        name=f"{name_prefix}_top_rib",
    )
    part.visual(
        Box((0.030, beam_y * 1.06, beam_z * 1.08)),
        origin=Origin(xyz=(beam_start + 0.015, 0.0, 0.0)),
        material=metal,
        name=f"{name_prefix}_root_end_cap",
    )

    if has_distal_fork:
        y_center = fork_gap / 2.0 + fork_plate_thickness / 2.0
        for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
            part.visual(
                Box((fork_x, fork_plate_thickness, fork_z)),
                origin=Origin(xyz=(length, sign * y_center, 0.0)),
                material=metal,
                name=f"{name_prefix}_{suffix}_fork_cheek",
            )
        part.visual(
            Box((0.026, beam_y * 1.05, beam_z * 1.04)),
            origin=Origin(xyz=(length - fork_x / 2.0 - 0.013, 0.0, 0.0)),
            material=metal,
            name=f"{name_prefix}_fork_web",
        )
        _add_side_washer_stack(
            part,
            x=length,
            z=0.0,
            pin_radius=distal_pin_radius,
            pin_length=distal_pin_length,
            washer_radius=washer_radius,
            washer_thickness=washer_thickness,
            cap_radius=washer_radius * 0.43,
            cap_thickness=washer_thickness * 1.20,
            name_prefix=f"{name_prefix}_distal",
            metal=metal,
            screw=screw,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_arm_chain")

    dark_steel = model.material("dark_burnished_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.26, 0.27, 1.0))
    root_paint = model.material("root_blue_finish", rgba=(0.12, 0.22, 0.34, 1.0))
    middle_paint = model.material("middle_slate_finish", rgba=(0.20, 0.30, 0.36, 1.0))
    outer_paint = model.material("outer_gray_finish", rgba=(0.36, 0.42, 0.44, 1.0))
    wrist_paint = model.material("wrist_light_finish", rgba=(0.55, 0.58, 0.57, 1.0))
    screw_black = model.material("black_socket_screws", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    root_axis_z = 0.180

    base = model.part("base")
    base.visual(
        Box((0.46, 0.32, 0.035)),
        origin=Origin(xyz=(0.040, 0.0, 0.0175)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.24, 0.19, 0.078)),
        origin=Origin(xyz=(0.015, 0.0, 0.073)),
        material=gunmetal,
        name="raised_plinth",
    )
    base.visual(
        Box((0.035, 0.118, 0.125)),
        origin=Origin(xyz=(-0.072, 0.0, 0.165)),
        material=gunmetal,
        name="clevis_rear_web",
    )
    for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
        base.visual(
            Box((0.165, 0.036, 0.160)),
            origin=Origin(xyz=(0.025, sign * 0.076, root_axis_z)),
            material=gunmetal,
            name=f"base_{suffix}_clevis_cheek",
        )
    _add_side_washer_stack(
        base,
        x=0.0,
        z=root_axis_z,
        pin_radius=0.016,
        pin_length=0.190,
        washer_radius=0.044,
        washer_thickness=0.010,
        cap_radius=0.018,
        cap_thickness=0.012,
        name_prefix="joint_1",
        metal=dark_steel,
        screw=screw_black,
    )
    for sx in (-0.135, 0.205):
        for sy in (-0.115, 0.115):
            base.visual(
                Cylinder(radius=0.013, length=0.007),
                origin=Origin(xyz=(sx, sy, 0.0375)),
                material=screw_black,
                name=f"base_anchor_{sx:+.3f}_{sy:+.3f}",
            )

    root_link = model.part("root_link")
    _add_link(
        root_link,
        length=0.620,
        beam_size=(0.070, 0.055),
        hub_radius=0.035,
        hub_length=0.090,
        name_prefix="root",
        material=root_paint,
        metal=dark_steel,
        has_distal_fork=True,
        fork_gap=0.065,
        fork_plate_thickness=0.026,
        fork_x=0.120,
        fork_z=0.104,
        distal_pin_radius=0.013,
        distal_pin_length=0.116,
        washer_radius=0.036,
        washer_thickness=0.008,
        screw=screw_black,
    )

    middle_link = model.part("middle_link")
    _add_link(
        middle_link,
        length=0.500,
        beam_size=(0.052, 0.045),
        hub_radius=0.028,
        hub_length=0.052,
        name_prefix="middle",
        material=middle_paint,
        metal=dark_steel,
        has_distal_fork=True,
        fork_gap=0.052,
        fork_plate_thickness=0.020,
        fork_x=0.100,
        fork_z=0.086,
        distal_pin_radius=0.010,
        distal_pin_length=0.092,
        washer_radius=0.030,
        washer_thickness=0.007,
        screw=screw_black,
    )

    outer_link = model.part("outer_link")
    _add_link(
        outer_link,
        length=0.380,
        beam_size=(0.044, 0.038),
        hub_radius=0.024,
        hub_length=0.040,
        name_prefix="outer",
        material=outer_paint,
        metal=dark_steel,
        has_distal_fork=True,
        fork_gap=0.044,
        fork_plate_thickness=0.018,
        fork_x=0.084,
        fork_z=0.074,
        distal_pin_radius=0.008,
        distal_pin_length=0.080,
        washer_radius=0.025,
        washer_thickness=0.006,
        screw=screw_black,
    )

    wrist_link = model.part("wrist_link")
    wrist_link.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=_y_cyl(0.0, 0.0, 0.0),
        material=dark_steel,
        name="wrist_hub",
    )
    wrist_link.visual(
        Box((0.250, 0.032, 0.030)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=wrist_paint,
        name="wrist_beam",
    )
    for sign, suffix in ((1.0, "top"), (-1.0, "bottom")):
        wrist_link.visual(
            Box((0.054, 0.030, 0.007)),
            origin=Origin(xyz=(0.021, 0.0, sign * 0.015)),
            material=dark_steel,
            name=f"wrist_{suffix}_hub_neck",
        )
    wrist_link.visual(
        Box((0.052, 0.064, 0.052)),
        origin=Origin(xyz=(0.283, 0.0, 0.0)),
        material=dark_steel,
        name="pad_backing_lug",
    )
    wrist_link.visual(
        Box((0.026, 0.200, 0.140)),
        origin=Origin(xyz=(0.314, 0.0, 0.0)),
        material=gunmetal,
        name="rectangular_pad_plate",
    )
    wrist_link.visual(
        Box((0.008, 0.184, 0.124)),
        origin=Origin(xyz=(0.331, 0.0, 0.0)),
        material=rubber,
        name="pad_face",
    )
    for sy in (-0.068, 0.068):
        for sz in (-0.045, 0.045):
            wrist_link.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=_x_cyl(0.337, sy, sz),
                material=screw_black,
                name=f"pad_screw_{sy:+.3f}_{sz:+.3f}",
            )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=base,
        child=root_link,
        origin=Origin(xyz=(0.0, 0.0, root_axis_z)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(effort=160.0, velocity=1.2, lower=0.0, upper=0.65),
    )
    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=root_link,
        child=middle_link,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(effort=110.0, velocity=1.4, lower=0.0, upper=0.65),
    )
    model.articulation(
        "middle_to_outer",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=outer_link,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(effort=70.0, velocity=1.6, lower=0.0, upper=0.65),
    )
    model.articulation(
        "outer_to_wrist",
        ArticulationType.REVOLUTE,
        parent=outer_link,
        child=wrist_link,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pin_pairs = (
        ("base", "root_link", "joint_1_pin", "root_hub", 0.080),
        ("root_link", "middle_link", "root_distal_pin", "middle_hub", 0.044),
        ("middle_link", "outer_link", "middle_distal_pin", "outer_hub", 0.034),
        ("outer_link", "wrist_link", "outer_distal_pin", "wrist_hub", 0.028),
    )
    for parent, child, pin, hub, retained_y in pin_pairs:
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin,
            elem_b=hub,
            reason="The visible hinge pin is intentionally captured through the rotating hub bore proxy.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin,
            outer_elem=hub,
            margin=0.002,
            name=f"{pin} sits inside {hub} in the hinge bore",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin,
            elem_b=hub,
            min_overlap=retained_y,
            name=f"{hub} has retained pin engagement",
        )

    joints = [
        object_model.get_articulation("base_to_root"),
        object_model.get_articulation("root_to_middle"),
        object_model.get_articulation("middle_to_outer"),
        object_model.get_articulation("outer_to_wrist"),
    ]
    ctx.check("serial four revolute joints", len(joints) == 4, details=f"found={len(joints)}")
    ctx.check(
        "parallel hinge axes",
        all(tuple(round(v, 6) for v in joint.axis) == AXIS_Y for joint in joints),
        details=str([joint.axis for joint in joints]),
    )

    with ctx.pose(
        {
            joints[0]: 0.55,
            joints[1]: 0.55,
            joints[2]: 0.55,
            joints[3]: 0.55,
        }
    ):
        ctx.expect_gap(
            "wrist_link",
            "base",
            axis="z",
            min_gap=0.18,
            name="raised terminal pad clears the grounded base",
        )
        wrist_pos = ctx.part_world_position("wrist_link")
        root_pos = ctx.part_world_position("root_link")
    ctx.check(
        "folded chain moves upward from the base clevis",
        wrist_pos is not None and root_pos is not None and wrist_pos[2] > root_pos[2] + 0.45,
        details=f"root={root_pos}, wrist={wrist_pos}",
    )

    return ctx.report()


object_model = build_object_model()
