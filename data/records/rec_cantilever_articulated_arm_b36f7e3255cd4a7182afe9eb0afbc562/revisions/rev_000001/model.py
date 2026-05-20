from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(profile, dx: float, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 40):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _side_plate_mesh(
    *,
    length: float,
    height: float,
    thickness: float,
    end_margin: float,
    bore_radius: float,
    pocket_specs: tuple[tuple[float, float, float], ...],
    name: str,
):
    """Return a thin machined side plate with bearing bores and lightening pockets.

    The profile lives in local X/Z, then is rotated so extrusion thickness becomes
    the link's local Y thickness.  The plate runs from roughly -end_margin to
    length + end_margin, leaving visible material around the pivot bores.
    """

    outer = _shift_profile(
        rounded_rect_profile(length + 2.0 * end_margin, height, 0.020, corner_segments=8),
        length / 2.0,
    )
    holes = [
        _circle_profile(0.0, 0.0, bore_radius, 44),
        _circle_profile(length, 0.0, bore_radius, 44),
    ]
    for x, pocket_w, pocket_h in pocket_specs:
        holes.append(
            _shift_profile(
                rounded_rect_profile(pocket_w, pocket_h, 0.012, corner_segments=6),
                x,
            )
        )

    geom = ExtrudeWithHolesGeometry(outer, holes, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _add_bolt_pair(
    part,
    *,
    x_positions: tuple[float, ...],
    y: float,
    z_positions: tuple[float, ...],
    radius: float,
    length: float,
    material: Material,
    prefix: str,
) -> None:
    sign = 1.0 if y >= 0.0 else -1.0
    for ix, x in enumerate(x_positions):
        for iz, z in enumerate(z_positions):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(
                    xyz=(x, y, z),
                    rpy=(sign * math.pi / 2.0, 0.0, 0.0),
                ),
                material=material,
                name=f"{prefix}_{ix}_{iz}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_mechanical_arm_study")

    cast = model.material("matte_cast_iron", rgba=(0.12, 0.13, 0.13, 1.0))
    plate = model.material("dark_machined_plate", rgba=(0.20, 0.22, 0.22, 1.0))
    steel = model.material("brushed_bearing_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_oxide_hardware", rgba=(0.02, 0.022, 0.02, 1.0))
    cover = model.material("removable_cover_plate", rgba=(0.30, 0.32, 0.31, 1.0))
    channel = model.material("dark_cable_channel", rgba=(0.035, 0.04, 0.04, 1.0))

    base = model.part("base_column")
    base.visual(
        Box((0.58, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast,
        name="ground_plate",
    )
    base.visual(
        Box((0.38, 0.28, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=plate,
        name="raised_plinth",
    )
    for i, (x, y) in enumerate(((-0.23, -0.16), (-0.23, 0.16), (0.23, -0.16), (0.23, 0.16))):
        base.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(x, y, 0.050)),
            material=black,
            name=f"anchor_bolt_{i}",
        )

    base.visual(
        Cylinder(radius=0.070, length=1.000),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=cast,
        name="main_column",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
        material=steel,
        name="stationary_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.086, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.088)),
        material=black,
        name="bearing_retainer_ring",
    )
    for i, (x, y, sx, sy) in enumerate(
        (
            (0.095, 0.0, 0.190, 0.018),
            (-0.095, 0.0, 0.190, 0.018),
            (0.0, 0.095, 0.018, 0.190),
            (0.0, -0.095, 0.018, 0.190),
        )
    ):
        base.visual(
            Box((sx, sy, 0.320)),
            origin=Origin(xyz=(x, y, 0.225)),
            material=plate,
            name=f"column_gusset_{i}",
        )

    shoulder = model.part("shoulder_bracket")
    shoulder.visual(
        Cylinder(radius=0.126, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=steel,
        name="yaw_bearing_cartridge",
    )
    shoulder.visual(
        Cylinder(radius=0.092, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black,
        name="upper_lock_ring",
    )
    shoulder.visual(
        Box((0.300, 0.156, 0.072)),
        origin=Origin(xyz=(0.175, 0.0, 0.012)),
        material=plate,
        name="offset_box_beam",
    )
    shoulder.visual(
        Box((0.040, 0.032, 0.086)),
        origin=Origin(xyz=(0.335, 0.094, 0.000)),
        material=plate,
        name="clevis_root_0",
    )
    shoulder.visual(
        Box((0.040, 0.032, 0.086)),
        origin=Origin(xyz=(0.335, -0.094, 0.000)),
        material=plate,
        name="clevis_root_1",
    )
    shoulder.visual(
        Box((0.110, 0.032, 0.205)),
        origin=Origin(xyz=(0.400, 0.094, 0.000)),
        material=plate,
        name="clevis_cheek_0",
    )
    shoulder.visual(
        Box((0.110, 0.032, 0.205)),
        origin=Origin(xyz=(0.400, -0.094, 0.000)),
        material=plate,
        name="clevis_cheek_1",
    )
    shoulder.visual(
        Box((0.120, 0.210, 0.036)),
        origin=Origin(xyz=(0.400, 0.0, -0.104)),
        material=plate,
        name="lower_clevis_bridge",
    )
    shoulder.visual(
        Box((0.090, 0.178, 0.018)),
        origin=Origin(xyz=(0.390, 0.0, 0.110)),
        material=cover,
        name="top_access_cover",
    )
    shoulder.visual(
        Box((0.130, 0.006, 0.060)),
        origin=Origin(xyz=(0.205, -0.081, 0.020)),
        material=channel,
        name="counterweight_pocket_liner",
    )
    for y, suffix in ((0.119, "0"), (-0.119, "1")):
        shoulder.visual(
            Cylinder(radius=0.060, length=0.018),
            origin=Origin(xyz=(0.400, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"shoulder_bearing_cap_{suffix}",
        )
        shoulder.visual(
            Cylinder(radius=0.020, length=0.024),
            origin=Origin(xyz=(0.400, y + (0.011 if y > 0 else -0.011), 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"shoulder_trunnion_end_{suffix}",
        )

    upper = model.part("upper_link")
    upper_len = 0.830
    upper_plate = _side_plate_mesh(
        length=upper_len,
        height=0.150,
        thickness=0.018,
        end_margin=0.065,
        bore_radius=0.043,
        pocket_specs=((0.235, 0.170, 0.058), (0.500, 0.190, 0.058), (0.680, 0.100, 0.048)),
        name="upper_side_plate_mesh",
    )
    for y, suffix in ((0.055, "0"), (-0.055, "1")):
        upper.visual(
            upper_plate,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=plate,
            name=f"ribbed_side_plate_{suffix}",
        )
    upper.visual(
        Cylinder(radius=0.052, length=0.156),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_hub",
    )
    for y, suffix in ((0.070, "0"), (-0.070, "1")):
        upper.visual(
            Cylinder(radius=0.050, length=0.014),
            origin=Origin(xyz=(upper_len, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"elbow_bearing_cap_{suffix}",
        )
    for i, x in enumerate((0.155, 0.360, 0.615)):
        upper.visual(
            Box((0.040, 0.132, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.062)),
            material=plate,
            name=f"upper_top_tie_{i}",
        )
        upper.visual(
            Box((0.035, 0.132, 0.020)),
            origin=Origin(xyz=(x + 0.035, 0.0, -0.062)),
            material=plate,
            name=f"upper_lower_tie_{i}",
        )
    upper.visual(
        Box((0.540, 0.045, 0.014)),
        origin=Origin(xyz=(0.455, 0.0, 0.079)),
        material=channel,
        name="upper_cable_channel",
    )
    for y, suffix in ((0.064, "0"), (-0.064, "1")):
        upper.visual(
            Box((0.185, 0.007, 0.070)),
            origin=Origin(xyz=(0.220, y, -0.012)),
            material=channel,
            name=f"counterweight_pocket_{suffix}",
        )
        upper.visual(
            Box((0.205, 0.007, 0.060)),
            origin=Origin(xyz=(0.515, y, 0.000)),
            material=cover,
            name=f"access_cover_{suffix}",
        )
        _add_bolt_pair(
            upper,
            x_positions=(0.435, 0.595),
            y=y + (0.0065 if y > 0 else -0.0065),
            z_positions=(-0.020, 0.020),
            radius=0.0075,
            length=0.006,
            material=black,
            prefix=f"upper_cover_bolt_{suffix}",
        )

    forearm = model.part("forearm_link")
    forearm_len = 0.720
    forearm_plate = _side_plate_mesh(
        length=forearm_len,
        height=0.124,
        thickness=0.014,
        end_margin=0.055,
        bore_radius=0.036,
        pocket_specs=((0.215, 0.155, 0.046), (0.440, 0.165, 0.046), (0.610, 0.090, 0.038)),
        name="forearm_side_plate_mesh",
    )
    for y, suffix in ((0.032, "0"), (-0.032, "1")):
        forearm.visual(
            forearm_plate,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=plate,
            name=f"forearm_side_plate_{suffix}",
        )
    forearm.visual(
        Cylinder(radius=0.044, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_inner_hub",
    )
    for y, suffix in ((0.0435, "0"), (-0.0435, "1")):
        forearm.visual(
            Cylinder(radius=0.039, length=0.012),
            origin=Origin(xyz=(forearm_len, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"wrist_bearing_cap_{suffix}",
        )
    for i, x in enumerate((0.135, 0.335, 0.555)):
        forearm.visual(
            Box((0.034, 0.084, 0.019)),
            origin=Origin(xyz=(x, 0.0, 0.052)),
            material=plate,
            name=f"forearm_top_tie_{i}",
        )
        forearm.visual(
            Box((0.030, 0.084, 0.018)),
            origin=Origin(xyz=(x + 0.030, 0.0, -0.052)),
            material=plate,
            name=f"forearm_bottom_tie_{i}",
        )
    forearm.visual(
        Box((0.480, 0.032, 0.012)),
        origin=Origin(xyz=(0.385, 0.0, 0.066)),
        material=channel,
        name="forearm_cable_channel",
    )
    for y, suffix in ((0.040, "0"), (-0.040, "1")):
        forearm.visual(
            Box((0.185, 0.006, 0.050)),
            origin=Origin(xyz=(0.375, y, 0.000)),
            material=cover,
            name=f"forearm_access_cover_{suffix}",
        )
        _add_bolt_pair(
            forearm,
            x_positions=(0.300, 0.450),
            y=y + (0.0045 if y > 0 else -0.0045),
            z_positions=(-0.017, 0.017),
            radius=0.006,
            length=0.005,
            material=black,
            prefix=f"forearm_cover_bolt_{suffix}",
        )

    flange = model.part("tool_flange")
    flange.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_trunnion",
    )
    flange.visual(
        Cylinder(radius=0.022, length=0.128),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_neck",
    )
    flange.visual(
        Cylinder(radius=0.064, length=0.026),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plate,
        name="mounting_flange",
    )
    flange.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.154, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="center_pilot_bore",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        flange.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(
                xyz=(0.1545, 0.043 * math.cos(angle), 0.043 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name=f"flange_socket_bolt_{i}",
        )

    yaw = model.articulation(
        "column_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 1.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.8, lower=-2.35, upper=2.35),
    )
    shoulder_pitch = model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper,
        origin=Origin(xyz=(0.400, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.55, lower=-0.75, upper=1.15),
    )
    elbow_pitch = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.75, lower=-1.20, upper=1.35),
    )
    wrist_pitch = model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=flange,
        origin=Origin(xyz=(forearm_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-1.55, upper=1.55),
    )

    # Keep the variables live for readability in authoring traces.
    _ = (yaw, shoulder_pitch, elbow_pitch, wrist_pitch)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    shoulder = object_model.get_part("shoulder_bracket")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm_link")
    flange = object_model.get_part("tool_flange")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.expect_origin_gap(
        shoulder,
        base,
        axis="z",
        min_gap=1.13,
        max_gap=1.15,
        name="yaw cartridge sits on top of the column",
    )
    ctx.expect_origin_distance(
        upper,
        shoulder,
        axes="xy",
        min_dist=0.395,
        max_dist=0.405,
        name="upper link pivots at the offset shoulder clevis",
    )
    ctx.expect_origin_distance(
        forearm,
        upper,
        axes="xy",
        min_dist=0.825,
        max_dist=0.835,
        name="elbow joint is at the end of the upper link",
    )
    ctx.expect_origin_distance(
        flange,
        forearm,
        axes="xy",
        min_dist=0.715,
        max_dist=0.725,
        name="tool flange pivots at the end of the forearm",
    )

    rest_elbow = ctx.part_world_position(forearm)
    rest_flange = ctx.part_world_position(flange)
    with ctx.pose({shoulder_pitch: 0.65, elbow_pitch: 0.45, wrist_pitch: -0.25}):
        raised_elbow = ctx.part_world_position(forearm)
        raised_flange = ctx.part_world_position(flange)

    ctx.check(
        "positive pitch raises the cantilevered arm",
        rest_elbow is not None
        and raised_elbow is not None
        and rest_flange is not None
        and raised_flange is not None
        and raised_elbow[2] > rest_elbow[2] + 0.35
        and raised_flange[2] > rest_flange[2] + 0.70,
        details=f"rest_elbow={rest_elbow}, raised_elbow={raised_elbow}, rest_flange={rest_flange}, raised_flange={raised_flange}",
    )

    return ctx.report()


object_model = build_object_model()
