from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.220
BASE_Y = 0.160
BASE_Z = 0.014
PEDESTAL_X = 0.110
PEDESTAL_Y = 0.090
PEDESTAL_Z = 0.028
SHOULDER_Z = 0.086

CHEEK_T = 0.008
INNER_GAP = 0.016
OUTER_WIDTH = INNER_GAP + 2.0 * CHEEK_T
CHEEK_Y = INNER_GAP / 2.0 + CHEEK_T / 2.0

LINK_T = 0.010
TERMINAL_T = 0.008
WASHER_T = 0.002
JOINT_CLEARANCE = 0.0012

SHOULDER_PIN_R = 0.006
ELBOW_PIN_R = 0.006
WRIST_PIN_R = 0.005
JOINT_HEAD_R = 0.010
WRIST_HEAD_R = 0.0085
JOINT_HEAD_T = 0.005
SHOULDER_WASHER_R = 0.009
WRIST_WASHER_R = 0.0075

FIRST_LINK_LEN = 0.115
SECOND_LINK_LEN = 0.185
TERMINAL_LINK_LEN = 0.118


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _washer_y(
    inner_radius: float,
    outer_radius: float,
    thickness: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_y(outer_radius, thickness, center).cut(
        _cyl_y(inner_radius, thickness + 0.003, center)
    )


def _union_all(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _cap_screw_z(
    *,
    x: float,
    y: float,
    seat_z: float,
    shaft_radius: float,
    shaft_length: float,
    head_radius: float,
    head_height: float,
) -> cq.Workplane:
    shaft = _cyl_z(shaft_radius, shaft_length, (x, y, seat_z - shaft_length / 2.0))
    head = _cyl_z(head_radius, head_height, (x, y, seat_z + head_height / 2.0))
    return shaft.union(head)


def _cap_screw_y(
    *,
    x: float,
    z: float,
    seat_y: float,
    direction: float,
    shaft_radius: float,
    shaft_length: float,
    head_radius: float,
    head_height: float,
) -> cq.Workplane:
    head_center_y = seat_y + direction * head_height / 2.0
    shaft_center_y = seat_y - direction * shaft_length / 2.0
    shaft = _cyl_y(shaft_radius, shaft_length, (x, shaft_center_y, z))
    head = _cyl_y(head_radius, head_height, (x, head_center_y, z))
    return shaft.union(head)


def _joint_hardware(
    *,
    x: float,
    z: float,
    pin_radius: float,
    head_radius: float,
    washer_radius: float,
    child_thickness: float,
) -> cq.Workplane:
    cheek_outer_y = OUTER_WIDTH / 2.0
    cheek_inner_y = INNER_GAP / 2.0
    child_face_y = child_thickness / 2.0
    bridge_t = max(cheek_inner_y - child_face_y, 0.0015)

    positive_head = _cyl_y(
        head_radius,
        JOINT_HEAD_T,
        (x, cheek_outer_y + JOINT_HEAD_T / 2.0, z),
    )
    negative_head = _cyl_y(
        head_radius * 0.92,
        JOINT_HEAD_T * 0.92,
        (x, -(cheek_outer_y + JOINT_HEAD_T * 0.46), z),
    )
    positive_washer = _washer_y(
        pin_radius * 0.95,
        washer_radius,
        bridge_t,
        (x, child_face_y + bridge_t / 2.0, z),
    )
    negative_washer = _washer_y(
        pin_radius * 0.95,
        washer_radius,
        bridge_t,
        (x, -(child_face_y + bridge_t / 2.0), z),
    )
    return _union_all(
        positive_head,
        negative_head,
        positive_washer,
        negative_washer,
    )


def _make_base_body() -> cq.Workplane:
    foot = _box((BASE_X, BASE_Y, BASE_Z), (0.0, 0.0, BASE_Z / 2.0))
    pedestal = _box(
        (PEDESTAL_X, PEDESTAL_Y, PEDESTAL_Z),
        (0.0, 0.0, BASE_Z + PEDESTAL_Z / 2.0),
    )
    shoulder_spine = _box((0.040, 0.030, 0.016), (0.0, 0.0, BASE_Z + PEDESTAL_Z + 0.008))

    positive_cheek = _union_all(
        _box((0.050, CHEEK_T, 0.060), (0.0, CHEEK_Y, SHOULDER_Z - 0.006)),
        _cyl_y(0.024, CHEEK_T, (0.0, CHEEK_Y, SHOULDER_Z)),
        _box((0.022, CHEEK_T, 0.020), (-0.012, CHEEK_Y, BASE_Z + PEDESTAL_Z + 0.006)),
    )
    negative_cheek = _union_all(
        _box((0.050, CHEEK_T, 0.060), (0.0, -CHEEK_Y, SHOULDER_Z - 0.006)),
        _cyl_y(0.024, CHEEK_T, (0.0, -CHEEK_Y, SHOULDER_Z)),
        _box((0.022, CHEEK_T, 0.020), (-0.012, -CHEEK_Y, BASE_Z + PEDESTAL_Z + 0.006)),
    )

    return _union_all(foot, pedestal, shoulder_spine, positive_cheek, negative_cheek)


def _make_base_mount_screws() -> cq.Workplane:
    screws: list[cq.Workplane] = []
    for x_pos in (-0.073, 0.073):
        for y_pos in (-0.050, 0.050):
            screws.append(
                _cap_screw_z(
                    x=x_pos,
                    y=y_pos,
                    seat_z=BASE_Z,
                    shaft_radius=0.0025,
                    shaft_length=0.010,
                    head_radius=0.0055,
                    head_height=0.004,
                )
            )
    return _union_all(*screws)


def _make_base_joint_hardware() -> cq.Workplane:
    return _joint_hardware(
        x=0.0,
        z=SHOULDER_Z,
        pin_radius=SHOULDER_PIN_R,
        head_radius=JOINT_HEAD_R,
        washer_radius=SHOULDER_WASHER_R,
        child_thickness=LINK_T,
    )


def _make_first_link_body() -> cq.Workplane:
    root_boss = _cyl_y(0.026, LINK_T, (0.0, 0.0, 0.0))
    main_beam = _box((0.058, LINK_T, 0.026), (0.031, 0.0, 0.0))
    fork_bridge = _box((0.018, OUTER_WIDTH, 0.020), (0.060, 0.0, 0.0))

    positive_cheek = _union_all(
        _box((0.042, CHEEK_T, 0.022), (0.093, CHEEK_Y, 0.0)),
        _cyl_y(0.020, CHEEK_T, (FIRST_LINK_LEN, CHEEK_Y, 0.0)),
    )
    negative_cheek = _union_all(
        _box((0.042, CHEEK_T, 0.022), (0.093, -CHEEK_Y, 0.0)),
        _cyl_y(0.020, CHEEK_T, (FIRST_LINK_LEN, -CHEEK_Y, 0.0)),
    )

    body = _union_all(root_boss, main_beam, fork_bridge, positive_cheek, negative_cheek)
    body = body.cut(
        _cyl_y(SHOULDER_PIN_R + JOINT_CLEARANCE, LINK_T + 0.012, (0.0, 0.0, 0.0))
    )
    body = body.cut(_box((0.028, LINK_T + 0.004, 0.010), (0.032, 0.0, 0.0)))
    return body


def _make_first_link_joint_hardware() -> cq.Workplane:
    return _joint_hardware(
        x=FIRST_LINK_LEN,
        z=0.0,
        pin_radius=ELBOW_PIN_R,
        head_radius=JOINT_HEAD_R,
        washer_radius=SHOULDER_WASHER_R,
        child_thickness=LINK_T,
    )


def _make_second_link_body() -> cq.Workplane:
    root_boss = _cyl_y(0.024, LINK_T, (0.0, 0.0, 0.0))
    main_beam = _box((0.118, LINK_T, 0.022), (0.066, 0.0, 0.0))
    fork_bridge = _box((0.018, OUTER_WIDTH, 0.018), (0.138, 0.0, 0.0))

    positive_cheek = _union_all(
        _box((0.038, CHEEK_T, 0.018), (0.166, CHEEK_Y, 0.0)),
        _cyl_y(0.017, CHEEK_T, (SECOND_LINK_LEN, CHEEK_Y, 0.0)),
    )
    negative_cheek = _union_all(
        _box((0.038, CHEEK_T, 0.018), (0.166, -CHEEK_Y, 0.0)),
        _cyl_y(0.017, CHEEK_T, (SECOND_LINK_LEN, -CHEEK_Y, 0.0)),
    )

    body = _union_all(root_boss, main_beam, fork_bridge, positive_cheek, negative_cheek)
    body = body.cut(
        _cyl_y(ELBOW_PIN_R + JOINT_CLEARANCE, LINK_T + 0.012, (0.0, 0.0, 0.0))
    )
    body = body.cut(_box((0.070, LINK_T + 0.004, 0.008), (0.078, 0.0, 0.0)))
    return body


def _make_second_link_joint_hardware() -> cq.Workplane:
    return _joint_hardware(
        x=SECOND_LINK_LEN,
        z=0.0,
        pin_radius=WRIST_PIN_R,
        head_radius=WRIST_HEAD_R,
        washer_radius=WRIST_WASHER_R,
        child_thickness=TERMINAL_T,
    )


def _make_terminal_link_body() -> cq.Workplane:
    root_boss = _cyl_y(0.018, TERMINAL_T, (0.0, 0.0, 0.0))
    main_beam = _box((0.086, TERMINAL_T, 0.018), (0.049, 0.0, 0.0))
    tip_neck = _box((0.020, TERMINAL_T, 0.014), (0.096, 0.0, 0.0))
    tip_pad = _box((0.016, 0.018, 0.032), (0.110, 0.0, 0.0))

    body = _union_all(root_boss, main_beam, tip_neck, tip_pad)
    body = body.cut(
        _cyl_y(WRIST_PIN_R + JOINT_CLEARANCE, TERMINAL_T + 0.012, (0.0, 0.0, 0.0))
    )
    body = body.cut(_box((0.034, TERMINAL_T + 0.004, 0.006), (0.048, 0.0, 0.0)))
    return body


def _make_tool_plate_body() -> cq.Workplane:
    spacer_block = _box((0.004, 0.012, 0.016), (0.002, 0.0, 0.0))
    neck = _box((0.016, 0.010, 0.012), (0.012, 0.0, 0.0))
    center_rib = _box((0.020, 0.008, 0.012), (0.030, 0.0, 0.0))
    upper_rib = _box((0.018, 0.010, 0.008), (0.034, 0.0, 0.018))
    lower_rib = _box((0.018, 0.010, 0.008), (0.034, 0.0, -0.018))
    plate = _box((0.060, 0.008, 0.056), (0.070, 0.0, 0.0))

    body = _union_all(spacer_block, neck, center_rib, upper_rib, lower_rib, plate)
    for x_pos in (0.056, 0.084):
        for z_pos in (-0.020, 0.020):
            body = body.cut(_cyl_y(0.0028, 0.014, (x_pos, 0.0, z_pos)))
    return body


def _make_tool_plate_hardware() -> cq.Workplane:
    screws: list[cq.Workplane] = []
    for x_pos in (0.056, 0.084):
        for z_pos in (-0.020, 0.020):
            screws.append(
                _cap_screw_y(
                    x=x_pos,
                    z=z_pos,
                    seat_y=0.004,
                    direction=1.0,
                    shaft_radius=0.0028,
                    shaft_length=0.010,
                    head_radius=0.0048,
                    head_height=0.004,
                )
            )
    return _union_all(*screws)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_arm")

    model.material("powder_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("link_aluminum", rgba=(0.71, 0.74, 0.77, 1.0))
    model.material("anodized_dark", rgba=(0.31, 0.34, 0.37, 1.0))
    model.material("tool_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("zinc", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("black_oxide", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("pedestal")
    base.visual(
        mesh_from_cadquery(_make_base_body(), "pedestal_body"),
        material="powder_dark",
        name="pedestal_body",
    )
    base.visual(
        mesh_from_cadquery(_make_base_joint_hardware(), "pedestal_joint_hardware"),
        material="zinc",
        name="pedestal_joint_hardware",
    )
    base.visual(
        mesh_from_cadquery(_make_base_mount_screws(), "pedestal_mount_screws"),
        material="black_oxide",
        name="pedestal_mount_screws",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, 0.120)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_make_first_link_body(), "first_link_body"),
        material="link_aluminum",
        name="first_link_body",
    )
    first_link.visual(
        mesh_from_cadquery(_make_first_link_joint_hardware(), "first_link_joint_hardware"),
        material="zinc",
        name="first_link_joint_hardware",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((FIRST_LINK_LEN, OUTER_WIDTH, 0.050)),
        mass=0.42,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(_make_second_link_body(), "second_link_body"),
        material="link_aluminum",
        name="second_link_body",
    )
    second_link.visual(
        mesh_from_cadquery(_make_second_link_joint_hardware(), "second_link_joint_hardware"),
        material="zinc",
        name="second_link_joint_hardware",
    )
    second_link.inertial = Inertial.from_geometry(
        Box((SECOND_LINK_LEN, OUTER_WIDTH, 0.040)),
        mass=0.56,
        origin=Origin(xyz=(0.092, 0.0, 0.0)),
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        mesh_from_cadquery(_make_terminal_link_body(), "terminal_link_body"),
        material="anodized_dark",
        name="terminal_link_body",
    )
    terminal_link.inertial = Inertial.from_geometry(
        Box((TERMINAL_LINK_LEN, 0.020, 0.036)),
        mass=0.20,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        mesh_from_cadquery(_make_tool_plate_body(), "tool_plate_body"),
        material="tool_gray",
        name="tool_plate_body",
    )
    tool_plate.visual(
        mesh_from_cadquery(_make_tool_plate_hardware(), "tool_plate_hardware"),
        material="black_oxide",
        name="tool_plate_hardware",
    )
    tool_plate.inertial = Inertial.from_geometry(
        Box((0.090, 0.018, 0.060)),
        mass=0.14,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=1.20,
            effort=35.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.85,
            upper=0.35,
            effort=24.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=terminal_link,
        origin=Origin(xyz=(SECOND_LINK_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.20,
            upper=1.10,
            effort=12.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "terminal_mount",
        ArticulationType.FIXED,
        parent=terminal_link,
        child=tool_plate,
        origin=Origin(xyz=(TERMINAL_LINK_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    terminal_link = object_model.get_part("terminal_link")
    tool_plate = object_model.get_part("tool_plate")
    pedestal_joint_hardware = pedestal.get_visual("pedestal_joint_hardware")
    first_link_body = first_link.get_visual("first_link_body")
    first_link_joint_hardware = first_link.get_visual("first_link_joint_hardware")
    second_link_body = second_link.get_visual("second_link_body")
    second_link_joint_hardware = second_link.get_visual("second_link_joint_hardware")
    terminal_link_body = terminal_link.get_visual("terminal_link_body")
    tool_plate_body = tool_plate.get_visual("tool_plate_body")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

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
    ctx.allow_overlap(
        pedestal,
        first_link,
        elem_a=pedestal_joint_hardware,
        elem_b=first_link_body,
        reason="Shoulder bolt and washers intentionally pass through the first-link hub bore.",
    )
    ctx.allow_overlap(
        first_link,
        second_link,
        elem_a=first_link_joint_hardware,
        elem_b=second_link_body,
        reason="Elbow shoulder-bolt hardware intentionally occupies the second-link root bore.",
    )
    ctx.allow_overlap(
        second_link,
        terminal_link,
        elem_a=second_link_joint_hardware,
        elem_b=terminal_link_body,
        reason="Wrist shoulder-bolt hardware intentionally passes through the terminal-link root boss.",
    )
    ctx.allow_overlap(
        terminal_link,
        tool_plate,
        elem_a=terminal_link_body,
        elem_b=tool_plate_body,
        reason="The tool plate uses a socketed fixed mount tongue nested into the terminal tip carrier.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "serial_joint_axes_share_plane",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (0.0, -1.0, 0.0),
        details=(
            f"shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}"
        ),
    )
    ctx.check(
        "link_lengths_progressively_longer_then_slim",
        FIRST_LINK_LEN < SECOND_LINK_LEN and TERMINAL_LINK_LEN < SECOND_LINK_LEN,
        details=(
            f"first={FIRST_LINK_LEN:.3f}, second={SECOND_LINK_LEN:.3f}, "
            f"terminal={TERMINAL_LINK_LEN:.3f}"
        ),
    )

    ctx.expect_contact(pedestal, first_link, name="pedestal_supports_first_link")
    ctx.expect_contact(first_link, second_link, name="first_link_supports_second_link")
    ctx.expect_contact(second_link, terminal_link, name="second_link_supports_terminal_link")
    ctx.expect_contact(terminal_link, tool_plate, name="terminal_link_supports_tool_plate")

    with ctx.pose({shoulder: 0.92, elbow: -1.55, wrist: 0.42}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")
        ctx.expect_contact(pedestal, first_link, name="shoulder_contact_in_folded_pose")
        ctx.expect_contact(first_link, second_link, name="elbow_contact_in_folded_pose")
        ctx.expect_contact(second_link, terminal_link, name="wrist_contact_in_folded_pose")

    with ctx.pose({shoulder: 0.28, elbow: -0.92, wrist: 0.64}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_work_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
