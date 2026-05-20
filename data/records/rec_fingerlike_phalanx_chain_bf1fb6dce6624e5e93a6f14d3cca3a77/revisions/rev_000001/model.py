from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROOT_WIDTH = 0.024
ROOT_THICK = 0.022
ROOT_REAR = 0.020
ROOT_JOINT_X = 0.018

PROXIMAL_LENGTH = 0.052
MIDDLE_LENGTH = 0.034
DISTAL_LENGTH = 0.028

PIN_RADIUS = 0.0022

PROXIMAL_TONGUE_WIDTH = 0.0090
MIDDLE_TONGUE_WIDTH = 0.0078
DISTAL_TONGUE_WIDTH = 0.0068


def _box_from(
    start_x: float,
    length: float,
    width: float,
    thickness: float,
    *,
    z_center: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        length,
        width,
        thickness,
        centered=(False, True, True),
    ).translate((start_x, 0.0, z_center))


def _offset_box(
    start_x: float,
    length: float,
    width: float,
    thickness: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> cq.Workplane:
    return _box_from(
        start_x,
        length,
        width,
        thickness,
        z_center=z_center,
    ).translate((0.0, y_center, 0.0))


def _paired_cheeks(
    *,
    start_x: float,
    length: float,
    outer_width: float,
    inner_gap: float,
    thickness: float,
    z_center: float = 0.0,
) -> cq.Workplane:
    cheek_width = (outer_width - inner_gap) / 2.0
    y_center = inner_gap / 2.0 + cheek_width / 2.0
    return _offset_box(
        start_x,
        length,
        cheek_width,
        thickness,
        y_center=y_center,
        z_center=z_center,
    ).union(
        _offset_box(
            start_x,
            length,
            cheek_width,
            thickness,
            y_center=-y_center,
            z_center=z_center,
        )
    )


def _y_cylinder(
    radius: float,
    length: float,
    *,
    x_center: float = 0.0,
    z_center: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_center, z_center)
        .circle(radius)
        .extrude(length)
        .translate((0.0, length / 2.0, 0.0))
    )


def _joint_fork(
    *,
    start_x: float,
    length: float,
    outer_width: float,
    inner_gap: float,
    thickness: float,
    pin_center_x: float,
    pin_radius: float = PIN_RADIUS,
    z_center: float = 0.0,
) -> cq.Workplane:
    cheeks = _paired_cheeks(
        start_x=start_x,
        length=length,
        outer_width=outer_width,
        inner_gap=inner_gap,
        thickness=thickness,
        z_center=z_center,
    )
    pin = _y_cylinder(
        pin_radius,
        outer_width,
        x_center=pin_center_x,
        z_center=z_center,
    )
    return cheeks.union(pin)


def _make_root_knuckle() -> cq.Workplane:
    rear_body = _box_from(
        -0.026,
        0.040,
        0.0185,
        0.016,
        z_center=-0.0015,
    )
    dorsal_cap = _box_from(
        -0.010,
        0.020,
        0.013,
        0.0055,
        z_center=0.0074,
    )
    lower_heel = _box_from(
        -0.028,
        0.018,
        0.016,
        0.006,
        z_center=-0.0068,
    )
    fork = _joint_fork(
        start_x=ROOT_JOINT_X - 0.0040,
        length=0.0100,
        outer_width=ROOT_WIDTH,
        inner_gap=PROXIMAL_TONGUE_WIDTH + 0.0020,
        thickness=0.0165,
        pin_center_x=ROOT_JOINT_X,
        z_center=0.0008,
    )
    return rear_body.union(dorsal_cap).union(lower_heel).union(fork)


def _make_phalanx_with_fork(
    *,
    link_length: float,
    body_width: float,
    body_thickness: float,
    tongue_width: float,
    tongue_length: float,
    fork_outer_width: float,
    fork_thickness: float,
    next_tongue_width: float,
    dorsal_ridge_height: float,
) -> cq.Workplane:
    tongue = _box_from(
        -0.0040,
        tongue_length + 0.0040,
        tongue_width,
        body_thickness * 0.88,
        z_center=-0.0006,
    )
    tongue = tongue.cut(_y_cylinder(PIN_RADIUS, tongue_width + 0.014, x_center=0.0, z_center=0.0))
    cheek_start = link_length - 0.0040
    cheek_length = 0.0100
    body_start = 0.0060
    body_end = cheek_start + 0.0015
    body = _box_from(
        body_start,
        body_end - body_start,
        body_width,
        body_thickness,
        z_center=0.0010,
    )
    belly = _box_from(
        body_start + 0.0080,
        max(min(body_end - (body_start + 0.0080) - 0.0020, 0.0120), 0.0080),
        body_width * 0.70,
        body_thickness * 0.68,
        z_center=-0.0012,
    )
    dorsal_ridge = _box_from(
        body_start + 0.005,
        max((body_end - body_start) - 0.004, 0.010),
        body_width * 0.60,
        dorsal_ridge_height,
        z_center=body_thickness * 0.43,
    )
    fork = _joint_fork(
        start_x=cheek_start,
        length=cheek_length,
        outer_width=fork_outer_width,
        inner_gap=next_tongue_width + 0.0020,
        thickness=fork_thickness,
        pin_center_x=link_length,
        z_center=0.0008,
    )
    return tongue.union(body).union(belly).union(dorsal_ridge).union(fork)


def _make_distal_shell() -> cq.Workplane:
    tongue = _box_from(
        -0.0040,
        0.0125,
        DISTAL_TONGUE_WIDTH,
        0.0092,
        z_center=-0.0006,
    )
    tongue = tongue.cut(_y_cylinder(PIN_RADIUS, DISTAL_TONGUE_WIDTH + 0.014, x_center=0.0, z_center=0.0))
    body_start = 0.0060
    body = _box_from(
        body_start,
        DISTAL_LENGTH - body_start,
        0.0142,
        0.0135,
        z_center=0.0010,
    )
    belly = _box_from(
        0.0130,
        0.0100,
        0.0102,
        0.0072,
        z_center=-0.0014,
    )
    dorsal_ridge = _box_from(
        0.010,
        0.012,
        0.0094,
        0.0040,
        z_center=0.0056,
    )
    blunt_tip = _box_from(
        0.020,
        0.011,
        0.0150,
        0.0120,
        z_center=-0.0005,
    )
    return tongue.union(body).union(belly).union(dorsal_ridge).union(blunt_tip)


def _make_fingertip_pad() -> cq.Workplane:
    return _box_from(
        0.018,
        0.012,
        0.0140,
        0.0070,
        z_center=-0.0046,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="segmented_phalanx_chain")

    knuckle_finish = model.material("knuckle_finish", rgba=(0.21, 0.22, 0.24, 1.0))
    phalanx_finish = model.material("phalanx_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    pad_finish = model.material("pad_finish", rgba=(0.14, 0.15, 0.16, 1.0))

    root_knuckle = model.part("root_knuckle")
    root_knuckle.visual(
        mesh_from_cadquery(_make_root_knuckle(), "root_knuckle_v2"),
        material=knuckle_finish,
        name="shell",
    )

    proximal = model.part("proximal_phalanx")
    proximal.visual(
        mesh_from_cadquery(
            _make_phalanx_with_fork(
                link_length=PROXIMAL_LENGTH,
                body_width=0.0180,
                body_thickness=0.0155,
                tongue_width=PROXIMAL_TONGUE_WIDTH,
                tongue_length=0.0100,
                fork_outer_width=0.0208,
                fork_thickness=0.0175,
                next_tongue_width=MIDDLE_TONGUE_WIDTH,
                dorsal_ridge_height=0.0045,
            ),
            "proximal_phalanx_v2",
        ),
        material=phalanx_finish,
        name="shell",
    )

    middle = model.part("middle_phalanx")
    middle.visual(
        mesh_from_cadquery(
            _make_phalanx_with_fork(
                link_length=MIDDLE_LENGTH,
                body_width=0.0158,
                body_thickness=0.0138,
                tongue_width=MIDDLE_TONGUE_WIDTH,
                tongue_length=0.0090,
                fork_outer_width=0.0172,
                fork_thickness=0.0150,
                next_tongue_width=DISTAL_TONGUE_WIDTH,
                dorsal_ridge_height=0.0038,
            ),
            "middle_phalanx_v2",
        ),
        material=phalanx_finish,
        name="shell",
    )

    distal = model.part("distal_phalanx")
    distal.visual(
        mesh_from_cadquery(_make_distal_shell(), "distal_phalanx_shell_v2"),
        material=phalanx_finish,
        name="shell",
    )
    distal.visual(
        mesh_from_cadquery(_make_fingertip_pad(), "distal_phalanx_pad_v2"),
        material=pad_finish,
        name="pad",
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root_knuckle,
        child=proximal,
        origin=Origin(xyz=(ROOT_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=0.0, upper=1.40),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.5, velocity=4.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_knuckle = object_model.get_part("root_knuckle")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")

    root_to_proximal = object_model.get_articulation("root_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

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
        root_knuckle,
        proximal,
        reason="Captured root knuckle pin passes through the proximal tongue with nominal zero-clearance hinge fit.",
    )
    ctx.allow_overlap(
        proximal,
        middle,
        reason="Captured middle knuckle pin passes through the middle tongue with nominal zero-clearance hinge fit.",
    )
    ctx.allow_overlap(
        middle,
        distal,
        reason="Captured distal knuckle pin passes through the distal tongue with nominal zero-clearance hinge fit.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(proximal, root_knuckle, name="root_joint_is_supported")
    ctx.expect_contact(middle, proximal, name="middle_joint_is_supported")
    ctx.expect_contact(distal, middle, name="distal_joint_is_supported")

    all_axes_are_y = all(
        abs(joint.axis[0]) < 1e-9 and abs(joint.axis[1] - 1.0) < 1e-9 and abs(joint.axis[2]) < 1e-9
        for joint in (root_to_proximal, proximal_to_middle, middle_to_distal)
    )
    ctx.check(
        "all_joints_share_one_bending_axis",
        all_axes_are_y,
        "Expected all three revolute axes to be parallel to +Y for one bending plane.",
    )

    pad_aabb = ctx.part_element_world_aabb(distal, elem="pad")
    shell_aabb = ctx.part_element_world_aabb(distal, elem="shell")
    pad_is_on_tip = (
        pad_aabb is not None
        and shell_aabb is not None
        and pad_aabb[1][0] >= shell_aabb[1][0] - 0.004
        and pad_aabb[0][2] < shell_aabb[0][2] + 0.003
    )
    ctx.check(
        "fingertip_pad_is_at_distal_tip",
        pad_is_on_tip,
        "Expected the rubber pad visual to sit at the distal tip and slightly below the shell.",
    )

    rest_tip = ctx.part_world_position(distal)
    with ctx.pose(
        {
            root_to_proximal: 0.80,
            proximal_to_middle: 1.00,
            middle_to_distal: 0.70,
        }
    ):
        flexed_tip = ctx.part_world_position(distal)

    flexes_downward = (
        rest_tip is not None
        and flexed_tip is not None
        and flexed_tip[2] < rest_tip[2] - 0.020
    )
    stays_in_plane = (
        rest_tip is not None
        and flexed_tip is not None
        and abs(flexed_tip[1] - rest_tip[1]) < 1e-5
    )
    ctx.check(
        "positive_joint_motion_flexes_chain",
        flexes_downward,
        "Expected positive revolute motion to bend the fingertip downward in Z.",
    )
    ctx.check(
        "flexion_stays_in_single_plane",
        stays_in_plane,
        "Expected flexion to remain in the XZ plane with negligible Y drift.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
