from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 64,
):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, 0.0),
                (outer_radius, length),
            ],
            [
                (inner_radius, 0.0),
                (inner_radius, length),
            ],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _helix_points(
    *,
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    samples: int,
    angle_offset: float = 0.0,
):
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        angle = angle_offset + (turns * math.tau * t)
        z_value = z_start + ((z_end - z_start) * t)
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_value,
            )
        )
    return points


def _build_bulb_envelope_mesh():
    return _save_mesh(
        "bulb_envelope.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0145, 0.000),
                (0.0200, 0.006),
                (0.0285, 0.024),
                (0.0345, 0.048),
                (0.0315, 0.072),
                (0.0180, 0.090),
                (0.0040, 0.098),
            ],
            [
                (0.0115, 0.000),
                (0.0165, 0.006),
                (0.0245, 0.024),
                (0.0305, 0.048),
                (0.0275, 0.072),
                (0.0145, 0.090),
                (0.0000, 0.098),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _build_thread_mesh():
    return _save_mesh(
        "bulb_thread.obj",
        tube_from_spline_points(
            _helix_points(
                radius=0.0147,
                z_start=-0.013,
                z_end=0.013,
                turns=3.25,
                samples=96,
            ),
            radius=0.0012,
            samples_per_segment=8,
            radial_segments=12,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )


def _build_support_wire_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return _save_mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=12,
            radial_segments=10,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )


def _hex_profile(across_flats: float):
    radius = across_flats / math.sqrt(3.0)
    return [
        (
            radius * math.cos((math.pi / 6.0) + (index * math.pi / 3.0)),
            radius * math.sin((math.pi / 6.0) + (index * math.pi / 3.0)),
        )
        for index in range(6)
    ]


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _aabb_dims(aabb):
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_screw_bulb_socket", assets=ASSETS)

    utility_paint = model.material("utility_paint", rgba=(0.24, 0.26, 0.28, 1.0))
    molded_black = model.material("molded_black", rgba=(0.08, 0.08, 0.07, 1.0))
    plated_brass = model.material("plated_brass", rgba=(0.70, 0.60, 0.31, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.66, 0.68, 0.72, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.54, 0.57, 0.61, 1.0))
    tough_glass = model.material("tough_glass", rgba=(0.86, 0.92, 0.96, 0.32))
    warm_glass = model.material("warm_glass", rgba=(0.83, 0.88, 0.92, 0.55))
    filament_metal = model.material("filament_metal", rgba=(0.47, 0.43, 0.31, 1.0))

    bulb_envelope_mesh = _build_bulb_envelope_mesh()
    bulb_thread_mesh = _build_thread_mesh()
    socket_collar_mesh = _save_mesh(
        "socket_collar.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0180, 0.000),
                (0.0280, 0.004),
                (0.0310, 0.014),
                (0.0320, 0.030),
                (0.0310, 0.050),
                (0.0260, 0.056),
            ],
            [
                (0.0100, 0.000),
                (0.0130, 0.008),
                (0.0178, 0.016),
                (0.0178, 0.048),
                (0.0162, 0.056),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    thread_receiver_mesh = _ring_shell_mesh(
        "thread_receiver.obj",
        outer_radius=0.0178,
        inner_radius=0.0165,
        length=0.030,
        segments=72,
    )
    seat_rim_mesh = _ring_shell_mesh(
        "seat_rim.obj",
        outer_radius=0.0230,
        inner_radius=0.0156,
        length=0.004,
        segments=72,
    )
    support_left_mesh = _build_support_wire_mesh(
        "support_left.obj",
        [
            (0.0000, 0.0000, 0.038),
            (-0.0030, 0.0000, 0.046),
            (-0.0065, 0.0000, 0.054),
        ],
        radius=0.0008,
    )
    support_right_mesh = _build_support_wire_mesh(
        "support_right.obj",
        [
            (0.0000, 0.0000, 0.038),
            (0.0030, 0.0000, 0.046),
            (0.0065, 0.0000, 0.054),
        ],
        radius=0.0008,
    )
    filament_mesh = _build_support_wire_mesh(
        "filament_loop.obj",
        [
            (-0.0065, 0.0000, 0.054),
            (-0.0035, 0.0000, 0.056),
            (0.0000, 0.0000, 0.055),
            (0.0035, 0.0000, 0.056),
            (0.0065, 0.0000, 0.054),
        ],
        radius=0.0007,
    )
    mount_flange_mesh = _save_mesh(
        "mount_flange.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.090, 0.090, radius=0.009, corner_segments=8),
            0.010,
        ),
    )
    conduit_locknut_mesh = _save_mesh(
        "conduit_locknut.obj",
        ExtrudeGeometry(_hex_profile(0.026), 0.012),
    )

    socket_body = model.part("socket_body")
    socket_body.visual(
        mount_flange_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=utility_paint,
        name="mount_flange",
    )
    socket_body.visual(
        socket_collar_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=utility_paint,
        name="socket_collar",
    )
    socket_body.visual(
        thread_receiver_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
        material=plated_brass,
        name="thread_receiver",
    )
    socket_body.visual(
        seat_rim_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.056)),
        material=plated_brass,
        name="seat_rim",
    )
    socket_body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=molded_black,
        name="insulator_core",
    )
    socket_body.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=plated_brass,
        name="center_contact",
    )
    socket_body.visual(
        Box((0.016, 0.010, 0.020)),
        origin=Origin(xyz=(0.024, 0.000, 0.020)),
        material=utility_paint,
        name="gusset_pos_x",
    )
    socket_body.visual(
        Box((0.016, 0.010, 0.020)),
        origin=Origin(xyz=(-0.024, 0.000, 0.020)),
        material=utility_paint,
        name="gusset_neg_x",
    )
    socket_body.visual(
        Box((0.010, 0.016, 0.020)),
        origin=Origin(xyz=(0.000, 0.024, 0.020)),
        material=utility_paint,
        name="gusset_pos_y",
    )
    socket_body.visual(
        Box((0.010, 0.016, 0.020)),
        origin=Origin(xyz=(0.000, -0.024, 0.020)),
        material=utility_paint,
        name="gusset_neg_y",
    )
    socket_body.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.000, -0.034, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=utility_paint,
        name="conduit_hub",
    )
    socket_body.visual(
        conduit_locknut_mesh,
        origin=Origin(xyz=(0.000, -0.025, 0.020)),
        material=fastener_steel,
        name="conduit_locknut",
    )

    fastener_positions = (
        (0.028, 0.028),
        (-0.028, 0.028),
        (-0.028, -0.028),
        (0.028, -0.028),
    )
    for index, (x_pos, y_pos) in enumerate(fastener_positions):
        socket_body.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, 0.012)),
            material=fastener_steel,
            name=f"flange_fastener_{index}",
        )
        slot_axis = 0.0 if index % 2 == 0 else (math.pi / 2.0)
        socket_body.visual(
            Box((0.010, 0.0015, 0.0012)),
            origin=Origin(xyz=(x_pos, y_pos, 0.0146), rpy=(0.0, 0.0, slot_axis)),
            material=molded_black,
            name=f"fastener_slot_{index}",
        )

    socket_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.070)),
        mass=0.85,
        origin=Origin(xyz=(0.000, 0.000, 0.035)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0135, length=0.030),
        origin=Origin(),
        material=galvanized_steel,
        name="screw_shell",
    )
    bulb.visual(
        bulb_thread_mesh,
        material=galvanized_steel,
        name="thread_ridge",
    )
    bulb.visual(
        Box((0.0022, 0.0045, 0.0080)),
        origin=Origin(xyz=(0.0142, 0.0000, -0.0070)),
        material=galvanized_steel,
        name="thread_lead_start",
    )
    bulb.visual(
        Cylinder(radius=0.0085, length=0.003),
        origin=Origin(xyz=(0.000, 0.000, -0.017)),
        material=molded_black,
        name="base_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, -0.0195)),
        material=plated_brass,
        name="contact_button",
    )
    bulb.visual(
        Cylinder(radius=0.0150, length=0.007),
        origin=Origin(xyz=(0.000, 0.000, 0.0185)),
        material=galvanized_steel,
        name="neck_collar",
    )
    bulb.visual(
        Cylinder(radius=0.0158, length=0.005),
        origin=Origin(xyz=(0.000, 0.000, 0.0215)),
        material=galvanized_steel,
        name="shoulder_collar",
    )
    bulb.visual(
        bulb_envelope_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.022)),
        material=tough_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0035, length=0.056),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=warm_glass,
        name="stem_support",
    )
    bulb.visual(
        support_left_mesh,
        material=filament_metal,
        name="support_left",
    )
    bulb.visual(
        support_right_mesh,
        material=filament_metal,
        name="support_right",
    )
    bulb.visual(
        filament_mesh,
        material=filament_metal,
        name="filament_loop",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.145),
        mass=0.16,
        origin=Origin(xyz=(0.000, 0.000, 0.052)),
    )

    model.articulation(
        "bulb_thread_rotation",
        ArticulationType.CONTINUOUS,
        parent=socket_body,
        child=bulb,
        origin=Origin(xyz=(0.000, 0.000, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    socket_body = object_model.get_part("socket_body")
    bulb = object_model.get_part("bulb")
    bulb_thread_rotation = object_model.get_articulation("bulb_thread_rotation")

    mount_flange = socket_body.get_visual("mount_flange")
    socket_collar = socket_body.get_visual("socket_collar")
    thread_receiver = socket_body.get_visual("thread_receiver")
    seat_rim = socket_body.get_visual("seat_rim")
    center_contact = socket_body.get_visual("center_contact")

    screw_shell = bulb.get_visual("screw_shell")
    shoulder_collar = bulb.get_visual("shoulder_collar")
    contact_button = bulb.get_visual("contact_button")
    glass_envelope = bulb.get_visual("glass_envelope")
    thread_lead_start = bulb.get_visual("thread_lead_start")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=10,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_origin_distance(
        bulb,
        socket_body,
        axes="xy",
        max_dist=0.0005,
        name="bulb_axis_is_coaxial_with_socket_axis",
    )
    ctx.expect_within(
        bulb,
        socket_body,
        axes="xy",
        inner_elem=screw_shell,
        outer_elem=thread_receiver,
        margin=0.0,
        name="threaded_shell_stays_inside_socket_receiver_in_plan",
    )
    ctx.expect_within(
        bulb,
        socket_body,
        axes="z",
        inner_elem=screw_shell,
        outer_elem=thread_receiver,
        margin=0.0005,
        name="threaded_shell_has_practical_engagement_depth",
    )
    ctx.expect_overlap(
        bulb,
        socket_body,
        axes="xy",
        elem_a=screw_shell,
        elem_b=thread_receiver,
        min_overlap=0.025,
        name="threaded_shell_and_receiver_share_a_clear_footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket_body,
        axes="xy",
        elem_a=shoulder_collar,
        elem_b=seat_rim,
        min_overlap=0.028,
        name="bulb_shoulder_covers_socket_seat",
    )
    ctx.expect_gap(
        bulb,
        socket_body,
        axis="z",
        positive_elem=shoulder_collar,
        negative_elem=seat_rim,
        max_gap=0.001,
        max_penetration=0.0,
        name="bulb_shoulder_seats_on_socket_rim",
    )
    ctx.expect_contact(
        bulb,
        socket_body,
        elem_a=contact_button,
        elem_b=center_contact,
        name="electrical_contact_button_meets_socket_contact",
    )

    envelope_aabb = ctx.part_element_world_aabb(bulb, elem=glass_envelope)
    screw_aabb = ctx.part_element_world_aabb(bulb, elem=screw_shell)
    collar_aabb = ctx.part_element_world_aabb(socket_body, elem=socket_collar)
    flange_aabb = ctx.part_element_world_aabb(socket_body, elem=mount_flange)
    lead_rest_aabb = ctx.part_element_world_aabb(bulb, elem=thread_lead_start)

    if (
        envelope_aabb is None
        or screw_aabb is None
        or collar_aabb is None
        or flange_aabb is None
        or lead_rest_aabb is None
    ):
        ctx.fail("visual_measurements_available", "Expected named visuals to produce world-space AABBs.")
        return ctx.report()

    envelope_dims = _aabb_dims(envelope_aabb)
    screw_dims = _aabb_dims(screw_aabb)
    collar_dims = _aabb_dims(collar_aabb)
    flange_dims = _aabb_dims(flange_aabb)
    lead_rest_center = _aabb_center(lead_rest_aabb)

    ctx.check(
        "glass_envelope_is_substantially_wider_than_screw_base",
        envelope_dims[0] > (screw_dims[0] * 2.3),
        details=(
            f"Envelope width {envelope_dims[0]:.4f} m should be much larger than "
            f"screw base width {screw_dims[0]:.4f} m."
        ),
    )
    ctx.check(
        "mount_flange_is_visibly_more_substantial_than_socket_collar",
        flange_dims[0] > (collar_dims[0] * 1.35),
        details=(
            f"Mount flange width {flange_dims[0]:.4f} m should exceed collar width "
            f"{collar_dims[0]:.4f} m by a rugged-service margin."
        ),
    )
    ctx.check(
        "thread_lead_marker_starts_offset_from_axis",
        lead_rest_center[0] > 0.010 and abs(lead_rest_center[1]) < 0.003,
        details=f"Lead marker center was {lead_rest_center}.",
    )

    with ctx.pose({bulb_thread_rotation: math.pi / 2.0}):
        ctx.expect_within(
            bulb,
            socket_body,
            axes="xy",
            inner_elem=screw_shell,
            outer_elem=thread_receiver,
            margin=0.0,
            name="threaded_shell_stays_inside_receiver_after_quarter_turn",
        )
        ctx.expect_gap(
            bulb,
            socket_body,
            axis="z",
            positive_elem=shoulder_collar,
            negative_elem=seat_rim,
            max_gap=0.001,
            max_penetration=0.0,
            name="bulb_shoulder_remains_seated_after_quarter_turn",
        )
        ctx.expect_contact(
            bulb,
            socket_body,
            elem_a=contact_button,
            elem_b=center_contact,
            name="center_contact_remains_touching_after_quarter_turn",
        )
        lead_quarter_aabb = ctx.part_element_world_aabb(bulb, elem=thread_lead_start)
        if lead_quarter_aabb is None:
            ctx.fail(
                "quarter_turn_marker_measurement",
                "Thread lead marker should remain measurable after rotating the bulb.",
            )
            return ctx.report()
        lead_quarter_center = _aabb_center(lead_quarter_aabb)
        ctx.check(
            "quarter_turn_rotates_marker_around_socket_axis",
            abs(lead_quarter_center[0]) < 0.003
            and lead_quarter_center[1] > 0.010
            and abs(lead_quarter_center[2] - lead_rest_center[2]) < 0.001,
            details=(
                f"Rest marker center {lead_rest_center} should move to +Y on quarter turn, "
                f"got {lead_quarter_center}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
