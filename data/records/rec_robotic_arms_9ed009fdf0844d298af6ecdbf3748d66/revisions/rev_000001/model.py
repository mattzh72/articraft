from __future__ import annotations

from math import cos, pi, sin

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


def _cyl_axis(axis: str) -> tuple[float, float, float]:
    """Rotation that turns a SDK cylinder's local +Z into the requested axis."""
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (-pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _bolt_circle(
    part,
    *,
    prefix: str,
    center: tuple[float, float, float],
    radius: float,
    count: int,
    face_axis: str,
    head_radius: float,
    head_length: float,
    material,
) -> None:
    """Small proud bolt heads on a circular cartridge face."""
    cx, cy, cz = center
    for i in range(count):
        a = 2.0 * pi * i / count
        if face_axis == "y":
            xyz = (cx + radius * cos(a), cy, cz + radius * sin(a))
            rpy = _cyl_axis("y")
        elif face_axis == "x":
            xyz = (cx, cy + radius * cos(a), cz + radius * sin(a))
            rpy = _cyl_axis("x")
        else:
            xyz = (cx + radius * cos(a), cy + radius * sin(a), cz)
            rpy = _cyl_axis("z")
        part.visual(
            Cylinder(radius=head_radius, length=head_length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=f"{prefix}_bolt_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_robot_arm")

    cast = model.material("aged_olive_casting", rgba=(0.30, 0.35, 0.31, 1.0))
    dark = model.material("blackened_iron", rgba=(0.055, 0.060, 0.060, 1.0))
    steel = model.material("worn_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    hatch = model.material("painted_service_hatch", rgba=(0.17, 0.23, 0.25, 1.0))
    brass = model.material("aged_brass_labels", rgba=(0.72, 0.54, 0.23, 1.0))
    rubber = model.material("black_rubber_gasket", rgba=(0.015, 0.015, 0.014, 1.0))

    # Root pedestal: heavy retrofit base, cast column, service access, and a
    # large visible yaw socket that the turret nests into.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.78, 0.62, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="floor_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.36, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=cast,
        name="round_base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=cast,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.205, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.5825)),
        material=dark,
        name="yaw_socket",
    )
    pedestal.visual(
        Box((0.18, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, -0.178, 0.33)),
        material=hatch,
        name="front_service_hatch",
    )
    pedestal.visual(
        Box((0.19, 0.010, 0.215)),
        origin=Origin(xyz=(0.0, -0.190, 0.33)),
        material=rubber,
        name="hatch_gasket",
    )
    pedestal.visual(
        Box((0.13, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.196, 0.42)),
        material=brass,
        name="legacy_nameplate",
    )
    for sx, sy, yaw in (
        (0.205, 0.0, 0.0),
        (-0.205, 0.0, 0.0),
        (0.0, 0.205, pi / 2.0),
        (0.0, -0.205, pi / 2.0),
    ):
        pedestal.visual(
            Box((0.25, 0.045, 0.115)),
            origin=Origin(xyz=(sx, sy, 0.155), rpy=(0.0, 0.0, yaw)),
            material=cast,
            name=f"pedestal_rib_{len(pedestal.visuals)}",
        )
    for x in (-0.30, 0.30):
        for y in (-0.23, 0.23):
            pedestal.visual(
                Cylinder(radius=0.022, length=0.015),
                origin=Origin(xyz=(x, y, 0.107)),
                material=steel,
                name=f"anchor_bolt_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )
    for x in (-0.070, 0.070):
        for z in (0.260, 0.400):
            pedestal.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(x, -0.197, z), rpy=_cyl_axis("y")),
                material=steel,
                name=f"hatch_screw_{x}_{z}",
            )

    # Yaw turret: rotating drum, bolted deck, and a shoulder yoke with
    # visible old-school plates rather than a seamless plastic shell.
    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.165, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="yaw_drum",
    )
    turret.visual(
        Cylinder(radius=0.255, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=cast,
        name="turret_deck",
    )
    turret.visual(
        Box((0.29, 0.20, 0.060)),
        origin=Origin(xyz=(0.115, 0.0, 0.095)),
        material=cast,
        name="shoulder_adapter",
    )
    for y, idx in ((0.165, 0), (-0.165, 1)):
        turret.visual(
            Box((0.145, 0.055, 0.290)),
            origin=Origin(xyz=(0.185, y, 0.235)),
            material=cast,
            name=f"shoulder_yoke_{idx}",
        )
    turret.visual(
        Cylinder(radius=0.038, length=0.430),
        origin=Origin(xyz=(0.185, 0.0, 0.250), rpy=_cyl_axis("y")),
        material=steel,
        name="shoulder_pin",
    )
    turret.visual(
        Box((0.018, 0.23, 0.18)),
        origin=Origin(xyz=(0.050, 0.0, 0.185), rpy=(0.0, -0.22, 0.0)),
        material=dark,
        name="shoulder_rear_web",
    )
    turret.visual(
        Box((0.145, 0.025, 0.075)),
        origin=Origin(xyz=(0.0, -0.252, 0.105)),
        material=hatch,
        name="turret_service_hatch",
    )
    _bolt_circle(
        turret,
        prefix="yaw",
        center=(0.0, 0.0, 0.093),
        radius=0.205,
        count=10,
        face_axis="z",
        head_radius=0.010,
        head_length=0.012,
        material=steel,
    )
    _bolt_circle(
        turret,
        prefix="shoulder_face",
        center=(0.185, 0.196, 0.250),
        radius=0.060,
        count=6,
        face_axis="y",
        head_radius=0.007,
        head_length=0.010,
        material=steel,
    )

    # Upper arm: ribbed rectangular link with a large shoulder cartridge and an
    # offset elbow cartridge so the axis order reads from left to right.
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.118, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_cyl_axis("y")),
        material=dark,
        name="shoulder_bearing",
    )
    upper_arm.visual(
        Box((0.430, 0.180, 0.120)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=cast,
        name="upper_box_beam",
    )
    upper_arm.visual(
        Box((0.535, 0.030, 0.150)),
        origin=Origin(xyz=(0.335, 0.105, 0.0)),
        material=dark,
        name="upper_side_strap_0",
    )
    upper_arm.visual(
        Box((0.535, 0.030, 0.150)),
        origin=Origin(xyz=(0.335, -0.105, 0.0)),
        material=dark,
        name="upper_side_strap_1",
    )
    upper_arm.visual(
        Box((0.250, 0.160, 0.030)),
        origin=Origin(xyz=(0.300, 0.0, 0.074)),
        material=steel,
        name="upper_top_reinforcement",
    )
    upper_arm.visual(
        Box((0.180, 0.022, 0.060)),
        origin=Origin(xyz=(0.315, 0.118, 0.0)),
        material=hatch,
        name="upper_service_hatch",
    )
    for y, idx in ((0.128, 0), (-0.128, 1)):
        upper_arm.visual(
            Box((0.150, 0.046, 0.245)),
            origin=Origin(xyz=(0.625, y, 0.0)),
            material=cast,
            name=f"elbow_fork_{idx}",
        )
    upper_arm.visual(
        Cylinder(radius=0.030, length=0.350),
        origin=Origin(xyz=(0.625, 0.0, 0.0), rpy=_cyl_axis("y")),
        material=steel,
        name="elbow_pin",
    )
    _bolt_circle(
        upper_arm,
        prefix="shoulder",
        center=(0.0, 0.112, 0.0),
        radius=0.080,
        count=8,
        face_axis="y",
        head_radius=0.0065,
        head_length=0.010,
        material=steel,
    )
    _bolt_circle(
        upper_arm,
        prefix="elbow_cartridge",
        center=(0.625, 0.154, 0.0),
        radius=0.074,
        count=6,
        face_axis="y",
        head_radius=0.006,
        head_length=0.008,
        material=steel,
    )

    # Forearm: slimmer replacement link, visibly bolted into the older elbow.
    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.092, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_cyl_axis("y")),
        material=dark,
        name="elbow_bearing",
    )
    forearm.visual(
        Box((0.360, 0.135, 0.105)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=cast,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.360, 0.026, 0.130)),
        origin=Origin(xyz=(0.260, 0.080, 0.0)),
        material=dark,
        name="forearm_side_rail_0",
    )
    forearm.visual(
        Box((0.360, 0.026, 0.130)),
        origin=Origin(xyz=(0.260, -0.080, 0.0)),
        material=dark,
        name="forearm_side_rail_1",
    )
    forearm.visual(
        Box((0.125, 0.017, 0.055)),
        origin=Origin(xyz=(0.235, -0.090, 0.0)),
        material=hatch,
        name="forearm_service_hatch",
    )
    for y, idx in ((0.098, 0), (-0.098, 1)):
        forearm.visual(
            Box((0.130, 0.040, 0.195)),
            origin=Origin(xyz=(0.500, y, 0.0)),
            material=cast,
            name=f"wrist_fork_{idx}",
        )
    forearm.visual(
        Cylinder(radius=0.024, length=0.270),
        origin=Origin(xyz=(0.500, 0.0, 0.0), rpy=_cyl_axis("y")),
        material=steel,
        name="wrist_pin",
    )
    _bolt_circle(
        forearm,
        prefix="elbow",
        center=(0.0, 0.0875, 0.0),
        radius=0.064,
        count=6,
        face_axis="y",
        head_radius=0.0055,
        head_length=0.010,
        material=steel,
    )
    _bolt_circle(
        forearm,
        prefix="wrist_pitch",
        center=(0.500, 0.120, 0.0),
        radius=0.055,
        count=6,
        face_axis="y",
        head_radius=0.005,
        head_length=0.007,
        material=steel,
    )

    # Wrist: compact modern sealed cartridge added to the legacy arm; pitch axis
    # is horizontal, roll axis is along the outgoing tool flange.
    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.072, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_cyl_axis("y")),
        material=dark,
        name="wrist_bearing",
    )
    wrist.visual(
        Box((0.140, 0.105, 0.105)),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material=cast,
        name="wrist_gearbox",
    )
    wrist.visual(
        Cylinder(radius=0.066, length=0.175),
        origin=Origin(xyz=(0.185, 0.0, 0.0), rpy=_cyl_axis("x")),
        material=dark,
        name="roll_hub",
    )
    wrist.visual(
        Box((0.080, 0.035, 0.120)),
        origin=Origin(xyz=(0.088, 0.055, 0.0)),
        material=steel,
        name="wrist_side_reinforcement_0",
    )
    wrist.visual(
        Box((0.080, 0.035, 0.120)),
        origin=Origin(xyz=(0.088, -0.055, 0.0)),
        material=steel,
        name="wrist_side_reinforcement_1",
    )
    _bolt_circle(
        wrist,
        prefix="roll",
        center=(0.274, 0.0, 0.0),
        radius=0.045,
        count=6,
        face_axis="x",
        head_radius=0.0047,
        head_length=0.006,
        material=steel,
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.045, length=0.095),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=_cyl_axis("x")),
        material=steel,
        name="flange_coupler",
    )
    tool_flange.visual(
        Cylinder(radius=0.090, length=0.035),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=_cyl_axis("x")),
        material=dark,
        name="tool_faceplate",
    )
    tool_flange.visual(
        Box((0.018, 0.110, 0.030)),
        origin=Origin(xyz=(0.127, 0.0, 0.0)),
        material=brass,
        name="axis_mark_plate",
    )
    _bolt_circle(
        tool_flange,
        prefix="tool",
        center=(0.124, 0.0, 0.0),
        radius=0.063,
        count=6,
        face_axis="x",
        head_radius=0.005,
        head_length=0.006,
        material=steel,
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.5825)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.0),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.185, 0.0, 0.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.4, lower=-0.75, upper=1.05),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.625, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.6, lower=-1.20, upper=1.35),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=2.4, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.CONTINUOUS,
        parent=wrist,
        child=tool_flange,
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=3.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    tool_flange = object_model.get_part("tool_flange")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.allow_overlap(
        pedestal,
        turret,
        elem_a="yaw_socket",
        elem_b="yaw_drum",
        reason="The yaw drum is intentionally seated inside the legacy pedestal bearing socket.",
    )
    ctx.expect_within(
        turret,
        pedestal,
        axes="xy",
        inner_elem="yaw_drum",
        outer_elem="yaw_socket",
        margin=0.004,
        name="yaw drum is captured inside socket footprint",
    )
    ctx.expect_overlap(
        turret,
        pedestal,
        axes="z",
        elem_a="yaw_drum",
        elem_b="yaw_socket",
        min_overlap=0.070,
        name="yaw cartridge has retained vertical insertion",
    )

    for parent, child, pin, bearing, check_name in (
        (turret, upper_arm, "shoulder_pin", "shoulder_bearing", "shoulder pin runs through bearing"),
        (upper_arm, forearm, "elbow_pin", "elbow_bearing", "elbow pin runs through bearing"),
        (forearm, wrist, "wrist_pin", "wrist_bearing", "wrist pitch pin runs through bearing"),
    ):
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin,
            elem_b=bearing,
            reason="Captured steel pin inside a separate rotating cartridge is the intended joint construction.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin,
            outer_elem=bearing,
            margin=0.002,
            name=f"{check_name} cross-section",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin,
            elem_b=bearing,
            min_overlap=0.10,
            name=check_name,
        )

    ctx.allow_overlap(
        wrist,
        tool_flange,
        elem_a="roll_hub",
        elem_b="flange_coupler",
        reason="The output coupler is captured in the wrist roll hub as a compact bearing stack.",
    )
    ctx.expect_within(
        tool_flange,
        wrist,
        axes="yz",
        inner_elem="flange_coupler",
        outer_elem="roll_hub",
        margin=0.002,
        name="roll coupler is concentric with hub",
    )
    ctx.expect_overlap(
        wrist,
        tool_flange,
        axes="x",
        elem_a="roll_hub",
        elem_b="flange_coupler",
        min_overlap=0.035,
        name="roll coupler remains inserted in hub",
    )

    rest_elbow_aabb = ctx.part_element_world_aabb(upper_arm, elem="elbow_pin")
    with ctx.pose({shoulder: 0.55}):
        raised_elbow_aabb = ctx.part_element_world_aabb(upper_arm, elem="elbow_pin")
    ctx.check(
        "positive shoulder pitch raises elbow cartridge",
        rest_elbow_aabb is not None
        and raised_elbow_aabb is not None
        and raised_elbow_aabb[0][2] > rest_elbow_aabb[0][2] + 0.12,
        details=f"rest={rest_elbow_aabb}, raised={raised_elbow_aabb}",
    )

    with ctx.pose({base_yaw: 0.65, shoulder: 0.25, elbow: 0.65, wrist_pitch: -0.45}):
        ctx.expect_overlap(
            wrist,
            forearm,
            axes="y",
            elem_a="wrist_bearing",
            elem_b="wrist_pin",
            min_overlap=0.10,
            name="articulated wrist pin remains captured",
        )

    return ctx.report()


object_model = build_object_model()
