from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sin, cos

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CARD_LENGTH = 0.324
CARD_HEIGHT = 0.124
CARD_THICKNESS = 0.055
FRONT_FASCIA_Z0 = 0.021
FRONT_FASCIA_DEPTH = CARD_THICKNESS / 2.0 - FRONT_FASCIA_Z0

LEFT_FAN_CENTER = (-0.092, 0.0)
CENTER_FAN_CENTER = (0.020, 0.0)
TAIL_FAN_CENTER = (0.122, 0.0)
FRONT_ROTOR_Z = 0.0128
TAIL_ROTOR_Z = 0.0128
FRONT_ROTOR_THICKNESS = 0.0092
TAIL_ROTOR_THICKNESS = 0.0076


def circle_profile(
    radius: float,
    *,
    segments: int = 56,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos((2.0 * pi * idx) / segments),
            cy + radius * sin((2.0 * pi * idx) / segments),
        )
        for idx in range(segments)
    ]


def annulus_geometry(
    *,
    center_xy: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    depth: float,
    z0: float,
    segments: int = 56,
) -> MeshGeometry:
    ring = ExtrudeWithHolesGeometry(
        circle_profile(outer_radius, segments=segments),
        [circle_profile(inner_radius, segments=segments)],
        depth,
        center=False,
    )
    ring.translate(center_xy[0], center_xy[1], z0)
    return ring


def fan_support_frame_geometry(
    *,
    center_xy: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    front_ring_depth: float,
    front_ring_z0: float,
    spindle_radius: float,
    spindle_length: float,
    spindle_z0: float,
    seat_outer_radius: float,
    seat_inner_radius: float,
    seat_depth: float,
    seat_z0: float,
    strut_width: float,
    strut_thickness: float,
    strut_z0: float,
    strut_angles: tuple[float, ...],
) -> MeshGeometry:
    frame = annulus_geometry(
        center_xy=center_xy,
        outer_radius=outer_radius,
        inner_radius=inner_radius,
        depth=front_ring_depth,
        z0=front_ring_z0,
    )

    seat = annulus_geometry(
        center_xy=center_xy,
        outer_radius=seat_outer_radius,
        inner_radius=seat_inner_radius,
        depth=seat_depth,
        z0=seat_z0,
    )
    frame.merge(seat)

    spindle = CylinderGeometry(spindle_radius, spindle_length, radial_segments=28, closed=True)
    spindle.translate(center_xy[0], center_xy[1], spindle_z0 + spindle_length / 2.0)
    frame.merge(spindle)

    bridge_gap = 0.0012
    strut_length = inner_radius - seat_outer_radius - bridge_gap
    strut_z = strut_z0 + strut_thickness / 2.0
    strut_mid_x = center_xy[0] + seat_outer_radius + bridge_gap / 2.0 + strut_length / 2.0
    for angle in strut_angles:
        strut = BoxGeometry((strut_length, strut_width, strut_thickness))
        strut.translate(strut_mid_x, center_xy[1], strut_z)
        strut.rotate((0.0, 0.0, 1.0), angle, origin=(center_xy[0], center_xy[1], strut_z))
        frame.merge(strut)

    return frame


def fan_rotor_geometry(
    *,
    outer_radius: float,
    hub_radius: float,
    bore_radius: float,
    thickness: float,
    blade_count: int,
    blade_chord: float,
    blade_pitch: float,
    sweep_offset: float,
) -> MeshGeometry:
    rotor = annulus_geometry(
        center_xy=(0.0, 0.0),
        outer_radius=hub_radius,
        inner_radius=bore_radius,
        depth=thickness,
        z0=-thickness / 2.0,
    )

    cap_depth = thickness * 0.24
    cap = annulus_geometry(
        center_xy=(0.0, 0.0),
        outer_radius=hub_radius * 0.76,
        inner_radius=bore_radius,
        depth=cap_depth,
        z0=thickness / 2.0 - cap_depth,
        segments=44,
    )
    rotor.merge(cap)

    blade_span = outer_radius - hub_radius - 0.004
    blade_center_y = hub_radius + blade_span / 2.0 - 0.001
    blade_thickness = thickness * 0.28
    for idx in range(blade_count):
        blade = BoxGeometry((blade_chord, blade_span, blade_thickness))
        blade.rotate_x(blade_pitch)
        blade.translate(0.0, blade_center_y, 0.0)
        blade.rotate_z((2.0 * pi * idx) / blade_count + sweep_offset)
        rotor.merge(blade)

    return rotor


def gpu_shroud_fascia_geometry() -> MeshGeometry:
    half_length = CARD_LENGTH / 2.0
    half_height = CARD_HEIGHT / 2.0
    outer_profile = [
        (-half_length, -0.057),
        (-half_length, 0.052),
        (-0.144, half_height),
        (0.126, half_height),
        (half_length, 0.044),
        (half_length, -0.048),
        (0.142, -half_height),
        (-0.148, -half_height),
    ]
    fascia = ExtrudeWithHolesGeometry(
        outer_profile,
        [
            circle_profile(0.050, center=LEFT_FAN_CENTER),
            circle_profile(0.050, center=CENTER_FAN_CENTER),
            circle_profile(0.034, center=TAIL_FAN_CENTER),
        ],
        FRONT_FASCIA_DEPTH,
        center=False,
    )
    fascia.translate(0.0, 0.0, FRONT_FASCIA_Z0)
    return fascia


def radial_strut_origin(
    *,
    center_xy: tuple[float, float],
    inner_radius: float,
    outer_radius: float,
    angle: float,
    z_center: float,
) -> Origin:
    center_radius = (inner_radius + outer_radius) / 2.0
    return Origin(
        xyz=(
            center_xy[0] + center_radius * cos(angle),
            center_xy[1] + center_radius * sin(angle),
            z_center,
        ),
        rpy=(0.0, 0.0, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    shroud_graphite = model.material("shroud_graphite", rgba=(0.12, 0.12, 0.14, 1.0))
    frame_black = model.material("frame_black", rgba=(0.07, 0.07, 0.08, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    fin_aluminum = model.material("fin_aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.07, 0.19, 0.09, 1.0))
    bracket_silver = model.material("bracket_silver", rgba=(0.75, 0.77, 0.79, 1.0))

    card_body = model.part("card_body")
    card_body.visual(
        mesh_from_geometry(gpu_shroud_fascia_geometry(), "gpu_shroud_fascia"),
        material=shroud_graphite,
        name="shroud_fascia",
    )

    card_body.visual(
        Box((0.294, 0.014, 0.030)),
        origin=Origin(xyz=(-0.004, 0.055, 0.006)),
        material=shroud_graphite,
        name="upper_shroud_rail",
    )
    card_body.visual(
        Box((0.294, 0.014, 0.030)),
        origin=Origin(xyz=(-0.004, -0.055, 0.006)),
        material=shroud_graphite,
        name="lower_shroud_rail",
    )
    card_body.visual(
        Box((0.030, 0.106, 0.030)),
        origin=Origin(xyz=(-0.147, 0.0, 0.006)),
        material=shroud_graphite,
        name="nose_block",
    )
    card_body.visual(
        Box((0.018, 0.040, 0.030)),
        origin=Origin(xyz=(-0.037, 0.0, 0.006)),
        material=shroud_graphite,
        name="middle_bridge",
    )
    card_body.visual(
        Box((0.018, 0.034, 0.030)),
        origin=Origin(xyz=(0.076, 0.0, 0.006)),
        material=shroud_graphite,
        name="rear_bridge",
    )
    card_body.visual(
        Box((0.062, 0.014, 0.030)),
        origin=Origin(xyz=(0.130, 0.038, 0.006)),
        material=shroud_graphite,
        name="tail_upper_bar",
    )
    card_body.visual(
        Box((0.062, 0.014, 0.030)),
        origin=Origin(xyz=(0.130, -0.038, 0.006)),
        material=shroud_graphite,
        name="tail_lower_bar",
    )
    card_body.visual(
        Box((0.020, 0.014, 0.038)),
        origin=Origin(xyz=(0.152, 0.038, -0.002)),
        material=shroud_graphite,
        name="tail_end_upper_frame",
    )
    card_body.visual(
        Box((0.020, 0.014, 0.038)),
        origin=Origin(xyz=(0.152, -0.038, -0.002)),
        material=shroud_graphite,
        name="tail_end_lower_frame",
    )

    front_clip_z0 = FRONT_ROTOR_Z - FRONT_ROTOR_THICKNESS / 2.0 - 0.0011
    tail_clip_z0 = TAIL_ROTOR_Z - TAIL_ROTOR_THICKNESS / 2.0 - 0.0010

    card_body.visual(
        mesh_from_geometry(
            annulus_geometry(
                center_xy=LEFT_FAN_CENTER,
                outer_radius=0.050,
                inner_radius=0.046,
                depth=0.014,
                z0=0.007,
            ),
            "left_fan_frame",
        ),
        material=frame_black,
        name="left_fan_frame",
    )
    card_body.visual(
        mesh_from_geometry(
            annulus_geometry(
                center_xy=LEFT_FAN_CENTER,
                outer_radius=0.0104,
                inner_radius=0.0046,
                depth=0.0011,
                z0=front_clip_z0,
                segments=40,
            ),
            "left_fan_rear_clip",
        ),
        material=frame_black,
        name="left_fan_rear_clip",
    )
    card_body.visual(
        Cylinder(radius=0.0046, length=0.0026),
        origin=Origin(xyz=(LEFT_FAN_CENTER[0], LEFT_FAN_CENTER[1], 0.00615)),
        material=frame_black,
        name="left_fan_spindle",
    )
    for idx, angle in enumerate((0.75, 2.95, 5.05), start=1):
        card_body.visual(
            Box((0.0366, 0.0038, 0.0015)),
            origin=radial_strut_origin(
                center_xy=LEFT_FAN_CENTER,
                inner_radius=0.0099,
                outer_radius=0.0465,
                angle=angle,
                z_center=0.00645,
            ),
            material=frame_black,
            name=f"left_fan_strut_{idx}",
        )

    card_body.visual(
        mesh_from_geometry(
            annulus_geometry(
                center_xy=CENTER_FAN_CENTER,
                outer_radius=0.050,
                inner_radius=0.046,
                depth=0.014,
                z0=0.007,
            ),
            "center_fan_frame",
        ),
        material=frame_black,
        name="center_fan_frame",
    )
    card_body.visual(
        mesh_from_geometry(
            annulus_geometry(
                center_xy=CENTER_FAN_CENTER,
                outer_radius=0.0104,
                inner_radius=0.0046,
                depth=0.0011,
                z0=front_clip_z0,
                segments=40,
            ),
            "center_fan_rear_clip",
        ),
        material=frame_black,
        name="center_fan_rear_clip",
    )
    card_body.visual(
        Cylinder(radius=0.0046, length=0.0024),
        origin=Origin(xyz=(CENTER_FAN_CENTER[0], CENTER_FAN_CENTER[1], 0.0060)),
        material=frame_black,
        name="center_fan_spindle",
    )
    for idx, angle in enumerate((0.30, 2.55, 4.80), start=1):
        card_body.visual(
            Box((0.0366, 0.0038, 0.0015)),
            origin=radial_strut_origin(
                center_xy=CENTER_FAN_CENTER,
                inner_radius=0.0099,
                outer_radius=0.0465,
                angle=angle,
                z_center=0.00645,
            ),
            material=frame_black,
            name=f"center_fan_strut_{idx}",
        )

    card_body.visual(
        mesh_from_geometry(
            annulus_geometry(
                center_xy=TAIL_FAN_CENTER,
                outer_radius=0.034,
                inner_radius=0.031,
                depth=0.014,
                z0=0.007,
            ),
            "tail_fan_frame",
        ),
        material=frame_black,
        name="tail_fan_frame",
    )
    card_body.visual(
        mesh_from_geometry(
            annulus_geometry(
                center_xy=TAIL_FAN_CENTER,
                outer_radius=0.0076,
                inner_radius=0.0036,
                depth=0.0010,
                z0=tail_clip_z0,
                segments=36,
            ),
            "tail_fan_rear_clip",
        ),
        material=frame_black,
        name="tail_fan_rear_clip",
    )
    card_body.visual(
        Cylinder(radius=0.0036, length=0.0022),
        origin=Origin(xyz=(TAIL_FAN_CENTER[0], TAIL_FAN_CENTER[1], 0.0070)),
        material=frame_black,
        name="tail_fan_spindle",
    )
    for idx, angle in enumerate((0.55, 2.65, 4.70), start=1):
        card_body.visual(
            Box((0.0250, 0.0030, 0.0018)),
            origin=radial_strut_origin(
                center_xy=TAIL_FAN_CENTER,
                inner_radius=0.0062,
                outer_radius=0.0312,
                angle=angle,
                z_center=0.0077,
            ),
            material=frame_black,
            name=f"tail_fan_strut_{idx}",
        )

    card_body.visual(
        Box((0.228, 0.112, 0.014)),
        origin=Origin(xyz=(-0.030, 0.0, -0.004)),
        material=fin_aluminum,
        name="heatsink_block",
    )
    card_body.visual(
        Box((0.252, 0.096, 0.003)),
        origin=Origin(xyz=(-0.036, 0.0, -0.0125)),
        material=pcb_green,
        name="pcb",
    )
    card_body.visual(
        Box((0.006, 0.112, 0.084)),
        origin=Origin(xyz=(-0.161, 0.0, -0.004)),
        material=bracket_silver,
        name="io_bracket",
    )

    left_rotor = model.part("left_front_fan_rotor")
    left_rotor.visual(
        mesh_from_geometry(
            fan_rotor_geometry(
                outer_radius=0.044,
                hub_radius=0.011,
                bore_radius=0.0047,
                thickness=FRONT_ROTOR_THICKNESS,
                blade_count=9,
                blade_chord=0.010,
                blade_pitch=0.28,
                sweep_offset=0.22,
            ),
            "left_front_rotor",
        ),
        material=rotor_black,
        name="rotor",
    )

    center_rotor = model.part("center_front_fan_rotor")
    center_rotor.visual(
        mesh_from_geometry(
            fan_rotor_geometry(
                outer_radius=0.044,
                hub_radius=0.011,
                bore_radius=0.0047,
                thickness=FRONT_ROTOR_THICKNESS,
                blade_count=9,
                blade_chord=0.010,
                blade_pitch=0.28,
                sweep_offset=0.54,
            ),
            "center_front_rotor",
        ),
        material=rotor_black,
        name="rotor",
    )

    tail_rotor = model.part("tail_fan_rotor")
    tail_rotor.visual(
        mesh_from_geometry(
            fan_rotor_geometry(
                outer_radius=0.028,
                hub_radius=0.008,
                bore_radius=0.0037,
                thickness=TAIL_ROTOR_THICKNESS,
                blade_count=7,
                blade_chord=0.007,
                blade_pitch=0.24,
                sweep_offset=0.36,
            ),
            "tail_rotor",
        ),
        material=rotor_black,
        name="rotor",
    )

    spin_limits = MotionLimits(effort=0.5, velocity=80.0)
    model.articulation(
        "left_front_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=left_rotor,
        origin=Origin(xyz=(LEFT_FAN_CENTER[0], LEFT_FAN_CENTER[1], FRONT_ROTOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "center_front_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=center_rotor,
        origin=Origin(xyz=(CENTER_FAN_CENTER[0], CENTER_FAN_CENTER[1], FRONT_ROTOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "tail_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=tail_rotor,
        origin=Origin(xyz=(TAIL_FAN_CENTER[0], TAIL_FAN_CENTER[1], TAIL_ROTOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card_body = object_model.get_part("card_body")
    left_rotor = object_model.get_part("left_front_fan_rotor")
    center_rotor = object_model.get_part("center_front_fan_rotor")
    tail_rotor = object_model.get_part("tail_fan_rotor")
    left_spin = object_model.get_articulation("left_front_fan_spin")
    center_spin = object_model.get_articulation("center_front_fan_spin")
    tail_spin = object_model.get_articulation("tail_fan_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for joint_obj in (left_spin, center_spin, tail_spin):
        limits = joint_obj.motion_limits
        ctx.check(
            f"{joint_obj.name}_continuous_axis",
            joint_obj.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint_obj.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"{joint_obj.name} should be a continuous spin joint about +Z.",
        )

    ctx.expect_within(
        left_rotor,
        card_body,
        axes="xy",
        outer_elem="left_fan_frame",
        margin=0.0,
        name="left_rotor_within_frame",
    )
    ctx.expect_within(
        center_rotor,
        card_body,
        axes="xy",
        outer_elem="center_fan_frame",
        margin=0.0,
        name="center_rotor_within_frame",
    )
    ctx.expect_within(
        tail_rotor,
        card_body,
        axes="xy",
        outer_elem="tail_fan_frame",
        margin=0.0,
        name="tail_rotor_within_frame",
    )

    ctx.expect_contact(
        left_rotor,
        card_body,
        elem_b="left_fan_rear_clip",
        name="left_rotor_clipped_to_frame",
    )
    ctx.expect_contact(
        center_rotor,
        card_body,
        elem_b="center_fan_rear_clip",
        name="center_rotor_clipped_to_frame",
    )
    ctx.expect_contact(
        tail_rotor,
        card_body,
        elem_b="tail_fan_rear_clip",
        name="tail_rotor_clipped_to_frame",
    )

    ctx.expect_gap(
        card_body,
        left_rotor,
        axis="z",
        positive_elem="shroud_fascia",
        min_gap=0.002,
        max_gap=0.008,
        name="left_rotor_below_fascia",
    )
    ctx.expect_gap(
        card_body,
        center_rotor,
        axis="z",
        positive_elem="shroud_fascia",
        min_gap=0.002,
        max_gap=0.008,
        name="center_rotor_below_fascia",
    )
    ctx.expect_gap(
        card_body,
        tail_rotor,
        axis="z",
        positive_elem="shroud_fascia",
        min_gap=0.002,
        max_gap=0.010,
        name="tail_rotor_below_fascia",
    )

    ctx.expect_gap(
        left_rotor,
        card_body,
        axis="z",
        negative_elem="heatsink_block",
        min_gap=0.002,
        max_gap=0.008,
        name="left_rotor_above_heatsink",
    )
    ctx.expect_gap(
        center_rotor,
        card_body,
        axis="z",
        negative_elem="heatsink_block",
        min_gap=0.002,
        max_gap=0.008,
        name="center_rotor_above_heatsink",
    )

    with ctx.pose(
        {
            left_spin: 0.8,
            center_spin: 2.1,
            tail_spin: 1.3,
        }
    ):
        ctx.expect_within(
            left_rotor,
            card_body,
            axes="xy",
            outer_elem="left_fan_frame",
            margin=0.0,
            name="left_rotor_within_frame_rotated",
        )
        ctx.expect_within(
            center_rotor,
            card_body,
            axes="xy",
            outer_elem="center_fan_frame",
            margin=0.0,
            name="center_rotor_within_frame_rotated",
        )
        ctx.expect_within(
            tail_rotor,
            card_body,
            axes="xy",
            outer_elem="tail_fan_frame",
            margin=0.0,
            name="tail_rotor_within_frame_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
