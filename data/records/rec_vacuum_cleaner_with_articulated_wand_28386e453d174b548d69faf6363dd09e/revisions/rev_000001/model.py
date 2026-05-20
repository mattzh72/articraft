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


def _cyl_along_x(length: float, radius: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_along_y(length: float, radius: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_screw_head(
    part,
    name: str,
    *,
    xyz: tuple[float, float, float],
    axis: str,
    material: Material,
    radius: float = 0.012,
    thickness: float = 0.006,
) -> None:
    """Add a low-profile round fastener head, slightly seated into its support."""
    if axis == "x":
        geom, base = _cyl_along_x(thickness, radius)
    elif axis == "y":
        geom, base = _cyl_along_y(thickness, radius)
    else:
        geom, base = Cylinder(radius=radius, length=thickness), Origin()
    part.visual(geom, origin=Origin(xyz=xyz, rpy=base.rpy), material=material, name=name)


def _tube_pose(length: float, down_angle: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    direction = (math.cos(down_angle), 0.0, -math.sin(down_angle))
    center = (0.5 * length * direction[0], 0.0, 0.5 * length * direction[2])
    return direction, center


def _add_reinforced_wand(
    part,
    *,
    name: str,
    length: float,
    down_angle: float,
    tube_material: Material,
    collar_material: Material,
    brace_material: Material,
    bolt_material: Material,
) -> tuple[float, float, float]:
    """Build a connected heavy wand member and return its distal joint position."""
    direction, center = _tube_pose(length, down_angle)
    cyl_rpy = (0.0, math.pi / 2.0 + down_angle, 0.0)
    box_rpy = (0.0, down_angle, 0.0)
    tube_length = length - 0.080
    _, tube_center = _tube_pose(tube_length, down_angle)

    part.visual(
        Cylinder(radius=0.032, length=tube_length),
        origin=Origin(xyz=tube_center, rpy=cyl_rpy),
        material=tube_material,
        name=f"{name}_tube",
    )
    part.visual(
        Cylinder(radius=0.047, length=0.090),
        origin=Origin(xyz=(0.045 * direction[0], 0.0, 0.045 * direction[2]), rpy=cyl_rpy),
        material=collar_material,
        name=f"{name}_near_collar",
    )
    part.visual(
        Cylinder(radius=0.047, length=0.070),
        origin=Origin(
            xyz=((length - 0.105) * direction[0], 0.0, (length - 0.105) * direction[2]),
            rpy=cyl_rpy,
        ),
        material=collar_material,
        name=f"{name}_far_collar",
    )

    # Long welded rib and periodic clamp blocks create a visible load path instead
    # of a decorative smooth tube.
    rib_center = (center[0], 0.0, center[2] + 0.034)
    part.visual(
        Box((length * 0.84, 0.026, 0.018)),
        origin=Origin(xyz=rib_center, rpy=box_rpy),
        material=brace_material,
        name=f"{name}_spine",
    )
    for idx, t in enumerate((0.24, 0.50, 0.76)):
        x = t * length * direction[0]
        z = t * length * direction[2]
        part.visual(
            Box((0.050, 0.118, 0.030)),
            origin=Origin(xyz=(x, 0.0, z + 0.004), rpy=box_rpy),
            material=brace_material,
            name=f"{name}_clamp_{idx}",
        )
        _add_screw_head(
            part,
            f"{name}_clamp_bolt_{idx}_0",
            xyz=(x, 0.061, z + 0.006),
            axis="y",
            material=bolt_material,
            radius=0.007,
            thickness=0.006,
        )
        _add_screw_head(
            part,
            f"{name}_clamp_bolt_{idx}_1",
            xyz=(x, -0.061, z + 0.006),
            axis="y",
            material=bolt_material,
            radius=0.007,
            thickness=0.006,
        )

    distal = (length * direction[0], 0.0, length * direction[2])

    # Fork cheeks for the next pivot.  The central child knuckle sits in the gap.
    for side, y in (("pos", 0.058), ("neg", -0.058)):
        part.visual(
            Box((0.135, 0.022, 0.116)),
            origin=Origin(xyz=(distal[0] - 0.010 * direction[0], y, distal[2] + 0.008), rpy=box_rpy),
            material=brace_material,
            name=f"{name}_fork_{side}",
        )
        _add_screw_head(
            part,
            f"{name}_fork_bolt_{side}",
            xyz=(distal[0] + 0.005 * direction[0], y + (0.009 if y > 0 else -0.009), distal[2] + 0.010),
            axis="y",
            material=bolt_material,
            radius=0.010,
            thickness=0.006,
        )

    # Two mechanical over-travel stop blocks bear against the child lug before
    # the tube or hose could become the hard stop.
    part.visual(
        Box((0.038, 0.132, 0.040)),
        origin=Origin(xyz=(distal[0] - 0.040 * direction[0], 0.0, distal[2] + 0.072), rpy=box_rpy),
        material=brace_material,
        name=f"{name}_upper_stop",
    )
    part.visual(
        Box((0.038, 0.132, 0.040)),
        origin=Origin(xyz=(distal[0] - 0.040 * direction[0], 0.0, distal[2] - 0.050), rpy=box_rpy),
        material=brace_material,
        name=f"{name}_lower_stop",
    )

    return distal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_vacuum")

    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.74, 0.05, 1.0))
    safety_red = model.material("safety_red", rgba=(0.85, 0.04, 0.02, 1.0))
    tank_metal = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.02, 0.023, 0.025, 1.0))
    frame_black = model.material("powdercoat_black", rgba=(0.08, 0.09, 0.10, 1.0))
    tube_blue = model.material("reinforced_blue_tube", rgba=(0.05, 0.18, 0.38, 1.0))
    collar_steel = model.material("collar_steel", rgba=(0.46, 0.48, 0.49, 1.0))
    bolt_dark = model.material("dark_bolts", rgba=(0.015, 0.015, 0.014, 1.0))
    warning_label = model.material("warning_label", rgba=(1.0, 0.92, 0.10, 1.0))
    translucent_guard = model.material("amber_guard", rgba=(1.0, 0.48, 0.05, 0.55))

    body = model.part("body")
    body.visual(
        Box((0.78, 0.70, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=frame_black,
        name="skid_frame",
    )
    body.visual(
        Cylinder(radius=0.310, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=tank_metal,
        name="collection_tank",
    )
    body.visual(
        Cylinder(radius=0.285, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.752)),
        material=frame_black,
        name="motor_head",
    )
    body.visual(
        Cylinder(radius=0.318, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.638)),
        material=safety_yellow,
        name="lid_clamp_band",
    )
    body.visual(
        Box((0.690, 0.055, 0.105)),
        origin=Origin(xyz=(0.0, 0.345, 0.125)),
        material=frame_black,
        name="side_rail_pos",
    )
    body.visual(
        Box((0.690, 0.055, 0.105)),
        origin=Origin(xyz=(0.0, -0.345, 0.125)),
        material=frame_black,
        name="side_rail_neg",
    )
    for y in (0.350, -0.350):
        for x in (-0.285, 0.285):
            body.visual(
                Box((0.052, 0.040, 0.695)),
                origin=Origin(xyz=(x, y, 0.418)),
                material=frame_black,
                name=f"roll_cage_{x:+.2f}_{y:+.2f}",
            )

    # Industrial rear wheels are tucked inside guard plates and bolted through the skid.
    for y, suffix in ((0.378, "pos"), (-0.378, "neg")):
        body.visual(
            Box((0.170, 0.026, 0.190)),
            origin=Origin(xyz=(-0.255, y, 0.145)),
            material=safety_yellow,
            name=f"wheel_guard_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.087, length=0.050),
            origin=Origin(xyz=(-0.255, y, 0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_rubber,
            name=f"rear_wheel_{suffix}",
        )
        _add_screw_head(body, f"wheel_axle_cap_{suffix}", xyz=(-0.255, y + (0.027 if y > 0 else -0.027), 0.105), axis="y", material=bolt_dark, radius=0.018, thickness=0.008)

    # Front intake and bolted safety flange.
    body.visual(
        Box((0.035, 0.210, 0.175)),
        origin=Origin(xyz=(0.323, 0.0, 0.600)),
        material=collar_steel,
        name="intake_flange",
    )
    body.visual(
        Cylinder(radius=0.075, length=0.120),
        origin=Origin(xyz=(0.370, 0.0, 0.600), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_steel,
        name="intake_socket",
    )
    body.visual(
        Cylinder(radius=0.085, length=0.030),
        origin=Origin(xyz=(0.430, 0.0, 0.600), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_yellow,
        name="socket_guard_ring",
    )
    for y in (-0.080, 0.080):
        for z in (0.540, 0.660):
            _add_screw_head(body, f"flange_bolt_{y:+.2f}_{z:.2f}", xyz=(0.343, y, z), axis="x", material=bolt_dark, radius=0.010, thickness=0.007)

    # Top safety control area: a guarded emergency stop and padlockable latch.
    body.visual(
        Box((0.245, 0.155, 0.020)),
        origin=Origin(xyz=(0.0, -0.135, 0.862)),
        material=collar_steel,
        name="control_plate",
    )
    body.visual(
        Box((0.040, 0.165, 0.035)),
        origin=Origin(xyz=(-0.102, -0.135, 0.887)),
        material=safety_yellow,
        name="lockout_hinge_block",
    )
    body.visual(
        Box((0.070, 0.020, 0.070)),
        origin=Origin(xyz=(0.085, -0.205, 0.892)),
        material=warning_label,
        name="lockout_tag",
    )

    stop_button = model.part("stop_button")
    stop_button.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=safety_red,
        name="red_plunger",
    )
    stop_button.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=frame_black,
        name="button_bezel",
    )
    model.articulation(
        "body_to_stop_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stop_button,
        origin=Origin(xyz=(0.0, -0.135, 0.872)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.08, lower=0.0, upper=0.018),
    )

    lockout_cover = model.part("lockout_cover")
    lockout_cover.visual(
        Cylinder(radius=0.014, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=collar_steel,
        name="hinge_barrel",
    )
    lockout_cover.visual(
        Box((0.115, 0.135, 0.020)),
        # The cover is parked upright and slightly proud at q=0 so it cannot
        # collide with the emergency stop, then can fold downward through its limit.
        origin=Origin(xyz=(0.045, 0.0, 0.070), rpy=(0.0, -0.92, 0.0)),
        material=translucent_guard,
        name="transparent_shield",
    )
    for y, suffix in ((0.055, "pos"), (-0.055, "neg")):
        lockout_cover.visual(
            Box((0.105, 0.012, 0.018)),
            origin=Origin(xyz=(0.047, y, 0.030), rpy=(0.0, -0.92, 0.0)),
            material=safety_yellow,
            name=f"cover_side_arm_{suffix}",
        )
    lockout_cover.visual(
        Box((0.015, 0.150, 0.080)),
        origin=Origin(xyz=(0.090, 0.0, 0.056), rpy=(0.0, -0.92, 0.0)),
        material=safety_yellow,
        name="shield_crossbar",
    )
    model.articulation(
        "body_to_lockout_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lockout_cover,
        origin=Origin(xyz=(-0.102, -0.135, 0.918)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.05, upper=1.25),
    )

    shoulder = model.part("shoulder_yoke")
    shoulder.visual(
        Cylinder(radius=0.074, length=0.065),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_steel,
        name="swivel_collar",
    )
    shoulder.visual(
        Cylinder(radius=0.086, length=0.025),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_yellow,
        name="yaw_stop_ring",
    )
    shoulder.visual(
        Box((0.142, 0.110, 0.030)),
        origin=Origin(xyz=(0.105, 0.0, -0.070)),
        material=safety_yellow,
        name="lower_overtravel_stop",
    )
    shoulder.visual(
        Box((0.142, 0.110, 0.038)),
        origin=Origin(xyz=(0.105, 0.0, 0.080)),
        material=safety_yellow,
        name="upper_overtravel_stop",
    )
    for side, y in (("pos", 0.058), ("neg", -0.058)):
        shoulder.visual(
            Box((0.142, 0.022, 0.124)),
            origin=Origin(xyz=(0.116, y, 0.004)),
            material=safety_yellow,
            name=f"pitch_fork_{side}",
        )
        _add_screw_head(shoulder, f"pitch_fork_cap_{side}", xyz=(0.118, y + (0.009 if y > 0 else -0.009), 0.004), axis="y", material=bolt_dark, radius=0.012, thickness=0.006)
    model.articulation(
        "body_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shoulder,
        origin=Origin(xyz=(0.450, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.55, lower=-0.55, upper=0.55),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.038, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=collar_steel,
        name="proximal_pivot_barrel",
    )
    lower_distal = _add_reinforced_wand(
        lower_wand,
        name="lower",
        length=0.620,
        down_angle=0.16,
        tube_material=tube_blue,
        collar_material=collar_steel,
        brace_material=safety_yellow,
        bolt_material=bolt_dark,
    )
    model.articulation(
        "shoulder_to_lower",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=lower_wand,
        origin=Origin(xyz=(0.118, 0.0, 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.65, lower=-0.28, upper=0.82),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.037, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=collar_steel,
        name="elbow_pivot_barrel",
    )
    upper_wand.visual(
        Box((0.050, 0.070, 0.080)),
        origin=Origin(xyz=(0.035, 0.0, 0.040)),
        material=safety_yellow,
        name="lockout_lug",
    )
    upper_distal = _add_reinforced_wand(
        upper_wand,
        name="upper",
        length=0.700,
        down_angle=0.30,
        tube_material=tube_blue,
        collar_material=collar_steel,
        brace_material=safety_yellow,
        bolt_material=bolt_dark,
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=lower_distal),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.65, lower=-0.55, upper=0.72),
    )

    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Cylinder(radius=0.038, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=collar_steel,
        name="wrist_pivot_barrel",
    )
    nozzle.visual(
        Box((0.072, 0.090, 0.315)),
        origin=Origin(xyz=(0.045, 0.0, -0.118), rpy=(0.0, 0.10, 0.0)),
        material=collar_steel,
        name="drop_neck",
    )
    nozzle.visual(
        Box((0.430, 0.620, 0.085)),
        origin=Origin(xyz=(0.172, 0.0, -0.223)),
        material=frame_black,
        name="nozzle_body",
    )
    nozzle.visual(
        Box((0.455, 0.055, 0.105)),
        origin=Origin(xyz=(0.172, 0.335, -0.204)),
        material=safety_yellow,
        name="edge_guard_pos",
    )
    nozzle.visual(
        Box((0.455, 0.055, 0.105)),
        origin=Origin(xyz=(0.172, -0.335, -0.204)),
        material=safety_yellow,
        name="edge_guard_neg",
    )
    nozzle.visual(
        Box((0.368, 0.502, 0.030)),
        origin=Origin(xyz=(0.188, 0.0, -0.278)),
        material=dark_rubber,
        name="squeegee_strip",
    )
    nozzle.visual(
        Box((0.300, 0.050, 0.030)),
        origin=Origin(xyz=(0.150, 0.0, -0.159)),
        material=warning_label,
        name="pinch_warning_label",
    )
    for y, suffix in ((0.295, "pos"), (-0.295, "neg")):
        nozzle.visual(
            Cylinder(radius=0.038, length=0.038),
            origin=Origin(xyz=(-0.045, y, -0.275), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_rubber,
            name=f"front_roller_{suffix}",
        )
        _add_screw_head(nozzle, f"roller_axle_cap_{suffix}", xyz=(-0.045, y + (0.022 if y > 0 else -0.022), -0.275), axis="y", material=bolt_dark, radius=0.010, thickness=0.006)
    model.articulation(
        "upper_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=nozzle,
        origin=Origin(xyz=upper_distal),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.85, lower=-0.65, upper=0.78),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lower = object_model.get_part("lower_wand")
    upper = object_model.get_part("upper_wand")
    nozzle = object_model.get_part("floor_nozzle")
    shoulder = object_model.get_part("shoulder_yoke")
    stop_button = object_model.get_part("stop_button")
    cover = object_model.get_part("lockout_cover")

    shoulder_pitch = object_model.get_articulation("shoulder_to_lower")
    elbow = object_model.get_articulation("lower_to_upper")
    wrist = object_model.get_articulation("upper_to_nozzle")
    yaw = object_model.get_articulation("body_to_shoulder")
    button_slide = object_model.get_articulation("body_to_stop_button")
    cover_hinge = object_model.get_articulation("body_to_lockout_cover")

    # Prompt-specific safety and mechanism checks: the collar/pivot chain remains
    # close and mechanically retained while the floor head is the lowest working
    # end of the assembly.
    ctx.expect_overlap(shoulder, lower, axes="yz", min_overlap=0.040, name="shoulder fork wraps lower pivot")
    ctx.expect_overlap(lower, upper, axes="yz", min_overlap=0.040, name="elbow fork wraps upper pivot")
    ctx.expect_overlap(upper, nozzle, axes="yz", min_overlap=0.040, name="wrist fork wraps nozzle pivot")
    ctx.expect_gap(lower, nozzle, axis="z", min_gap=0.035, name="floor nozzle sits below wand")
    ctx.expect_gap(
        stop_button,
        body,
        axis="z",
        min_gap=-0.001,
        max_gap=0.004,
        positive_elem="button_bezel",
        negative_elem="control_plate",
        name="emergency stop is seated in its plate",
    )
    ctx.expect_origin_distance(cover, stop_button, axes="xy", max_dist=0.160, name="lockout cover guards the stop")

    rest_upper = ctx.part_world_position(upper)
    rest_nozzle = ctx.part_world_position(nozzle)
    with ctx.pose({shoulder_pitch: 0.45, elbow: 0.28, wrist: -0.20}):
        posed_upper = ctx.part_world_position(upper)
        posed_nozzle = ctx.part_world_position(nozzle)
    ctx.check(
        "positive wand pitch lowers working end",
        rest_upper is not None
        and posed_upper is not None
        and rest_nozzle is not None
        and posed_nozzle is not None
        and posed_upper[2] < rest_upper[2] - 0.050
        and posed_nozzle[2] < rest_nozzle[2] - 0.090,
        details=f"upper rest={rest_upper}, upper posed={posed_upper}, nozzle rest={rest_nozzle}, nozzle posed={posed_nozzle}",
    )

    rest_nozzle_y = ctx.part_world_position(nozzle)
    with ctx.pose({yaw: 0.40}):
        yawed_nozzle_y = ctx.part_world_position(nozzle)
    ctx.check(
        "swivel yaw sweeps wand sideways",
        rest_nozzle_y is not None
        and yawed_nozzle_y is not None
        and abs(yawed_nozzle_y[1] - rest_nozzle_y[1]) > 0.150,
        details=f"rest={rest_nozzle_y}, yawed={yawed_nozzle_y}",
    )

    rest_button = ctx.part_world_position(stop_button)
    with ctx.pose({button_slide: 0.014, cover_hinge: 0.60}):
        pressed_button = ctx.part_world_position(stop_button)
        moved_cover = ctx.part_world_position(cover)
    ctx.check(
        "emergency stop depresses downward",
        rest_button is not None and pressed_button is not None and pressed_button[2] < rest_button[2] - 0.010,
        details=f"rest={rest_button}, pressed={pressed_button}, cover={moved_cover}",
    )

    return ctx.report()


object_model = build_object_model()
