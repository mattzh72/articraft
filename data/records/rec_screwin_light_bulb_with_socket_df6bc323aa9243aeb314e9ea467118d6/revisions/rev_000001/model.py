from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    phase: float = 0.0,
    samples_per_turn: int = 28,
) -> list[tuple[float, float, float]]:
    total_samples = max(8, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for idx in range(total_samples + 1):
        t = turns * 2.0 * math.pi * idx / total_samples
        points.append(
            (
                radius * math.cos(t + phase),
                radius * math.sin(t + phase),
                z_start + pitch * t / (2.0 * math.pi),
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_screw_bulb_socket")

    enamel_black = model.material("enamel_black", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.57, 0.29, 1.0))
    ceramic = model.material("ceramic", rgba=(0.86, 0.85, 0.81, 1.0))
    nickel = model.material("nickel", rgba=(0.74, 0.76, 0.80, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.93, 0.94, 0.96, 0.62))
    service_gray = model.material("service_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    terminal_black = model.material("terminal_black", rgba=(0.08, 0.08, 0.09, 1.0))

    housing_shell = _save_mesh(
        "socket_housing_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.034, -0.050),
                (0.034, -0.010),
                (0.030, -0.006),
                (0.029, 0.012),
            ],
            [
                (0.0265, -0.048),
                (0.0265, -0.002),
                (0.0270, 0.002),
                (0.0270, 0.010),
            ],
            segments=56,
        ),
    )
    insert_shell = _save_mesh(
        "threaded_insert_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0205, -0.0215),
                (0.0205, 0.0000),
                (0.0245, 0.0025),
                (0.0245, 0.0040),
            ],
            [
                (0.0157, -0.0205),
                (0.0157, 0.0035),
            ],
            segments=56,
        ),
    )
    bulb_glass = _save_mesh(
        "bulb_glass_envelope",
        LatheGeometry.from_shell_profiles(
            [
                (0.014, 0.006),
                (0.018, 0.014),
                (0.028, 0.034),
                (0.036, 0.062),
                (0.034, 0.088),
                (0.024, 0.109),
                (0.010, 0.119),
                (0.0, 0.121),
            ],
            [
                (0.0115, 0.009),
                (0.0155, 0.016),
                (0.0250, 0.035),
                (0.0320, 0.062),
                (0.0300, 0.086),
                (0.0210, 0.106),
                (0.0080, 0.116),
                (0.0, 0.118),
            ],
            segments=60,
        ),
    )
    male_thread = _save_mesh(
        "bulb_male_thread",
        tube_from_spline_points(
            _helix_points(
                radius=0.0127,
                z_start=-0.0195,
                pitch=0.0041,
                turns=4.0,
                phase=0.0,
            ),
            radius=0.0010,
            samples_per_segment=4,
            radial_segments=12,
            cap_ends=True,
        ),
    )
    female_thread = _save_mesh(
        "socket_female_thread",
        tube_from_spline_points(
            _helix_points(
                radius=0.0149,
                z_start=-0.0192,
                pitch=0.0041,
                turns=4.0,
                phase=math.pi / 2.5,
            ),
            radius=0.0008,
            samples_per_segment=4,
            radial_segments=12,
            cap_ends=True,
        ),
    )

    socket_body = model.part("socket_body")
    socket_body.visual(housing_shell, material=enamel_black, name="housing_shell")
    socket_body.visual(
        Box((0.090, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.103)),
        material=enamel_black,
        name="box_floor",
    )
    socket_body.visual(
        Box((0.090, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=enamel_black,
        name="box_roof",
    )
    socket_body.visual(
        Box((0.006, 0.082, 0.060)),
        origin=Origin(xyz=(-0.042, 0.0, -0.076)),
        material=enamel_black,
        name="box_left_wall",
    )
    socket_body.visual(
        Box((0.006, 0.082, 0.060)),
        origin=Origin(xyz=(0.042, 0.0, -0.076)),
        material=enamel_black,
        name="box_right_wall",
    )
    socket_body.visual(
        Box((0.090, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, -0.038, -0.076)),
        material=enamel_black,
        name="box_back_wall",
    )
    socket_body.visual(
        Box((0.018, 0.036, 0.028)),
        origin=Origin(xyz=(-0.054, -0.012, -0.078)),
        material=service_gray,
        name="mount_ear_left",
    )
    socket_body.visual(
        Box((0.018, 0.036, 0.028)),
        origin=Origin(xyz=(0.054, -0.012, -0.078)),
        material=service_gray,
        name="mount_ear_right",
    )
    socket_body.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=dark_steel,
        name="cable_gland",
    )
    socket_body.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.106)),
        material=service_gray,
        name="gland_nut",
    )
    for name, xyz, size in (
        ("collar_rib_pos_x", (0.0285, 0.0, 0.001), (0.008, 0.018, 0.016)),
        ("collar_rib_neg_x", (-0.0285, 0.0, 0.001), (0.008, 0.018, 0.016)),
        ("collar_rib_pos_y", (0.0, 0.0285, 0.001), (0.018, 0.008, 0.016)),
        ("collar_rib_neg_y", (0.0, -0.0285, 0.001), (0.018, 0.008, 0.016)),
    ):
        socket_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=service_gray,
            name=name,
        )
    socket_body.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(-0.026, 0.039, -0.041)),
        material=dark_steel,
        name="hinge_bracket_left",
    )
    socket_body.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(0.026, 0.039, -0.041)),
        material=dark_steel,
        name="hinge_bracket_right",
    )
    socket_body.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(-0.026, 0.047, -0.041), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel_left",
    )
    socket_body.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.026, 0.047, -0.041), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel_right",
    )
    socket_body.visual(
        Box((0.016, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.036, -0.096)),
        material=dark_steel,
        name="latch_keeper",
    )
    socket_body.inertial = Inertial.from_geometry(
        Box((0.120, 0.100, 0.140)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    threaded_insert = model.part("threaded_insert")
    threaded_insert.visual(insert_shell, material=brass, name="insert_shell")
    threaded_insert.visual(female_thread, material=brass, name="socket_thread")
    threaded_insert.visual(
        Cylinder(radius=0.0070, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.0180)),
        material=ceramic,
        name="ceramic_carrier",
    )
    threaded_insert.visual(
        Cylinder(radius=0.0036, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=brass,
        name="center_contact",
    )
    threaded_insert.visual(
        Box((0.0107, 0.0040, 0.0020)),
        origin=Origin(xyz=(0.01035, 0.0, -0.0210)),
        material=ceramic,
        name="carrier_rib_pos_x",
    )
    threaded_insert.visual(
        Box((0.0107, 0.0040, 0.0020)),
        origin=Origin(xyz=(-0.01035, 0.0, -0.0210)),
        material=ceramic,
        name="carrier_rib_neg_x",
    )
    threaded_insert.visual(
        Box((0.0040, 0.0107, 0.0020)),
        origin=Origin(xyz=(0.0, 0.01035, -0.0210)),
        material=ceramic,
        name="carrier_rib_pos_y",
    )
    threaded_insert.visual(
        Box((0.0040, 0.0107, 0.0020)),
        origin=Origin(xyz=(0.0, -0.01035, -0.0210)),
        material=ceramic,
        name="carrier_rib_neg_y",
    )
    threaded_insert.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.028),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
    )

    terminal_block = model.part("terminal_block")
    terminal_block.visual(
        Box((0.050, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, -0.008, -0.092)),
        material=terminal_black,
        name="terminal_base",
    )
    terminal_block.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(-0.013, -0.004, -0.076)),
        material=brass,
        name="terminal_lug_left",
    )
    terminal_block.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(0.013, -0.004, -0.076)),
        material=brass,
        name="terminal_lug_right",
    )
    terminal_block.visual(
        Box((0.040, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.002, -0.065)),
        material=service_gray,
        name="clamp_bridge",
    )
    terminal_block.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.032)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.006, -0.081)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.084, 0.004, 0.056)),
        origin=Origin(xyz=(0.0, -0.008, -0.039)),
        material=service_gray,
        name="door_panel",
    )
    service_door.visual(
        Box((0.010, 0.008, 0.040)),
        origin=Origin(xyz=(-0.022, -0.004, -0.041)),
        material=dark_steel,
        name="door_rib_inner",
    )
    service_door.visual(
        Box((0.010, 0.008, 0.040)),
        origin=Origin(xyz=(0.022, -0.004, -0.041)),
        material=dark_steel,
        name="door_rib_outer",
    )
    service_door.visual(
        Box((0.016, 0.008, 0.010)),
        origin=Origin(xyz=(-0.026, -0.004, -0.006)),
        material=dark_steel,
        name="door_hinge_leaf_left",
    )
    service_door.visual(
        Box((0.016, 0.008, 0.010)),
        origin=Origin(xyz=(0.026, -0.004, -0.006)),
        material=dark_steel,
        name="door_hinge_leaf_right",
    )
    service_door.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, -0.051)),
        material=dark_steel,
        name="door_latch_block",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.090, 0.016, 0.060)),
        mass=0.14,
        origin=Origin(xyz=(0.0, -0.004, -0.040)),
    )

    bulb = model.part("bulb")
    bulb.visual(bulb_glass, material=frosted_glass, name="glass_envelope")
    bulb.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=nickel,
        name="neck_ferrule",
    )
    bulb.visual(
        Cylinder(radius=0.0185, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=nickel,
        name="neck_collar",
    )
    bulb.visual(
        Cylinder(radius=0.0126, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=nickel,
        name="screw_shell",
    )
    bulb.visual(male_thread, material=nickel, name="screw_thread")
    bulb.visual(
        Cylinder(radius=0.0034, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=brass,
        name="contact_button",
    )
    bulb.visual(
        Cylinder(radius=0.003, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=ceramic,
        name="led_stem",
    )
    bulb.visual(
        Box((0.018, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=ceramic,
        name="emitter_paddle",
    )
    bulb.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=ceramic,
        name="driver_core",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.074, 0.074, 0.124)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
    )

    model.articulation(
        "body_to_insert",
        ArticulationType.FIXED,
        parent=socket_body,
        child=threaded_insert,
        origin=Origin(),
    )
    model.articulation(
        "body_to_terminal_block",
        ArticulationType.FIXED,
        parent=socket_body,
        child=terminal_block,
        origin=Origin(),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=socket_body,
        child=service_door,
        origin=Origin(xyz=(0.0, 0.047, -0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "insert_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=threaded_insert,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=8.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket_body = object_model.get_part("socket_body")
    threaded_insert = object_model.get_part("threaded_insert")
    terminal_block = object_model.get_part("terminal_block")
    service_door = object_model.get_part("service_door")
    bulb = object_model.get_part("bulb")
    access_hinge = object_model.get_articulation("body_to_service_door")
    bulb_spin = object_model.get_articulation("insert_to_bulb")

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

    ctx.expect_contact(threaded_insert, socket_body, name="insert_is_seated_in_housing")
    ctx.expect_contact(terminal_block, socket_body, name="terminal_block_is_supported")
    ctx.expect_contact(service_door, socket_body, name="closed_service_door_is_mounted")
    ctx.expect_contact(bulb, threaded_insert, name="bulb_is_supported_by_socket")
    ctx.expect_contact(
        bulb,
        threaded_insert,
        elem_a="contact_button",
        elem_b="center_contact",
        name="bulb_center_contact_reaches_insert_contact",
    )
    ctx.expect_overlap(
        bulb,
        threaded_insert,
        axes="xy",
        elem_a="screw_thread",
        elem_b="socket_thread",
        min_overlap=0.026,
        name="bulb_and_socket_threads_are_coaxial",
    )
    ctx.expect_within(
        bulb,
        threaded_insert,
        axes="xy",
        inner_elem="screw_shell",
        outer_elem="insert_shell",
        margin=0.012,
        name="bulb_shell_stays_inside_insert_envelope",
    )
    with ctx.pose({bulb_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_bulb_clearance")
    with ctx.pose({access_hinge: 1.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_door_open_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
