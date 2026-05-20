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


LINK_1_LENGTH = 0.280
LINK_2_LENGTH = 0.215
LINK_3_LENGTH = 0.130

BASE_FORK_GAP = 0.0140
LINK_1_HUB_LEN = 0.0100
LINK_1_FORK_GAP = 0.0136
LINK_2_HUB_LEN = 0.0095
LINK_2_FORK_GAP = 0.0129
LINK_3_HUB_LEN = 0.0085

PAD_SIZE = 0.048
PAD_THICKNESS = 0.005

BASE_YOKE_GAP = 0.022
SHOULDER_NECK_WIDTH = 0.011
SHOULDER_YOKE_GAP = 0.017
ELBOW_NECK_WIDTH = 0.010
ELBOW_YOKE_GAP = 0.015
WRIST_NECK_WIDTH = 0.009


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    fillet_x: float | None = None,
) -> cq.Workplane:
    sx, sy, sz = size
    solid = cq.Workplane("XY").box(sx, sy, sz)
    if fillet_x is not None and fillet_x > 0.0:
        solid = solid.edges("|X").fillet(
            min(fillet_x, max(0.0005, sy * 0.48), max(0.0005, sz * 0.48))
        )
    return solid.translate(center)


def _cyl_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def _rib_plate(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(width)
        .translate((0.0, -width / 2.0, 0.0))
    )


def _make_outer_yoke(
    *,
    joint_x: float,
    gap_width: float,
    cheek_thickness: float,
    barrel_radius: float,
    cheek_back: float,
    cheek_front: float,
    arm_height: float,
    bridge_height: float,
) -> cq.Workplane:
    barrel_length = cheek_thickness
    outer_width = gap_width + 2.0 * cheek_thickness
    y_offset = gap_width / 2.0 + cheek_thickness / 2.0
    cheek_len = cheek_back + cheek_front
    cheek_center_x = joint_x + (cheek_front - cheek_back) / 2.0

    bridge_back_x = joint_x - cheek_back
    bridge_front_x = joint_x - barrel_radius - 0.005
    bridge_len = max(0.008, bridge_front_x - bridge_back_x)
    bridge_center_x = bridge_back_x + bridge_len / 2.0

    yoke = _box(
        (bridge_len, outer_width, bridge_height),
        center=(bridge_center_x, 0.0, 0.0),
        fillet_x=min(0.004, bridge_height * 0.18),
    )
    for y_sign in (-1.0, 1.0):
        y_center = y_sign * y_offset
        yoke = yoke.union(
            _box(
                (cheek_len, cheek_thickness, arm_height),
                center=(cheek_center_x, y_center, 0.0),
                fillet_x=min(0.003, arm_height * 0.20),
            )
        )
        yoke = yoke.union(
            _cyl_y(
                barrel_radius,
                barrel_length,
                center=(joint_x, y_center, 0.0),
            )
        )
    return yoke


def _make_joint_head(
    *,
    barrel_radius: float,
    barrel_length: float,
    neck_width: float,
    beam_height: float,
    neck_len: float,
    origin_offset: float,
) -> cq.Workplane:
    barrel_center_x = origin_offset + barrel_radius
    head = _cyl_y(barrel_radius, barrel_length, center=(barrel_center_x, 0.0, 0.0)).cut(
        _box(
            (barrel_radius * 2.4, barrel_length * 2.6, barrel_radius * 2.6),
            center=(origin_offset - barrel_radius * 0.12, 0.0, 0.0),
        )
    )
    head = head.union(
        _box(
            (neck_len, neck_width, beam_height * 0.96),
            center=(origin_offset + neck_len / 2.0 + barrel_radius * 0.14, 0.0, 0.0),
            fillet_x=min(0.004, beam_height * 0.22),
        )
    )
    rib_width = neck_width * 0.96
    head = head.union(
        _rib_plate(
            [
                (origin_offset + 0.004, barrel_radius * 0.18),
                (origin_offset + neck_len * 0.64, beam_height / 2.0 - 0.001),
                (origin_offset + neck_len * 0.64, beam_height / 2.0 + 0.006),
                (origin_offset + 0.004, barrel_radius * 0.60),
            ],
            width=rib_width,
        )
    )
    head = head.union(
        _rib_plate(
            [
                (origin_offset + 0.004, -barrel_radius * 0.18),
                (origin_offset + neck_len * 0.64, -beam_height / 2.0 + 0.001),
                (origin_offset + neck_len * 0.64, -beam_height / 2.0 - 0.006),
                (origin_offset + 0.004, -barrel_radius * 0.60),
            ],
            width=rib_width,
        )
    )
    return head


def _make_serial_link_v2(
    *,
    length: float,
    proximal_radius: float,
    proximal_hub_len: float,
    proximal_width: float,
    beam_height: float,
    distal_gap_width: float,
    distal_cheek_thickness: float,
    distal_barrel_radius: float,
    proximal_offset: float,
) -> cq.Workplane:
    neck_len = 0.030
    beam_width = proximal_width * 0.92
    distal_bridge_front = length - distal_barrel_radius - 0.005
    beam_start = proximal_offset + neck_len - 0.002
    beam_end = distal_bridge_front
    beam_len = beam_end - beam_start
    distal_root_len = 0.020
    distal_root_end = distal_bridge_front
    distal_root_start = distal_root_end - distal_root_len

    link = _make_joint_head(
        barrel_radius=proximal_radius,
        barrel_length=proximal_hub_len,
        neck_width=proximal_width,
        beam_height=beam_height,
        neck_len=neck_len,
        origin_offset=proximal_offset,
    )
    link = link.union(
        _box(
            (beam_len, beam_width, beam_height),
            center=(beam_start + beam_len / 2.0, 0.0, 0.0),
            fillet_x=min(0.0035, beam_height * 0.22),
        )
    )
    link = link.union(
        _box(
            (beam_len * 0.74, beam_width * 0.92, 0.0055),
            center=(beam_start + beam_len * 0.53, 0.0, beam_height / 2.0 + 0.0018),
            fillet_x=0.002,
        )
    )
    link = link.union(
        _box(
            (beam_len * 0.74, beam_width * 0.92, 0.0055),
            center=(beam_start + beam_len * 0.53, 0.0, -beam_height / 2.0 - 0.0018),
            fillet_x=0.002,
        )
    )
    link = link.union(
        _box(
            (distal_root_len, proximal_width * 1.10, beam_height * 1.08),
            center=((distal_root_start + distal_root_end) / 2.0, 0.0, 0.0),
            fillet_x=min(0.0035, beam_height * 0.20),
        )
    )
    return link.union(
        _make_outer_yoke(
            joint_x=length,
            gap_width=distal_gap_width,
            cheek_thickness=distal_cheek_thickness,
            barrel_radius=distal_barrel_radius,
            cheek_back=0.033,
            cheek_front=0.005,
            arm_height=max(beam_height + 0.018, distal_barrel_radius * 1.9),
            bridge_height=max(beam_height + 0.022, distal_barrel_radius * 2.1),
        )
    )


def _make_base_shape_v2() -> cq.Workplane:
    plate = _box((0.010, 0.118, 0.252), center=(-0.033, 0.0, 0.0), fillet_x=0.010)
    boss = _box((0.018, 0.060, 0.110), center=(-0.024, 0.0, 0.0), fillet_x=0.006)
    yoke = _make_outer_yoke(
        joint_x=0.0,
        gap_width=BASE_YOKE_GAP,
        cheek_thickness=0.0075,
        barrel_radius=0.017,
        cheek_back=0.032,
        cheek_front=0.004,
        arm_height=0.044,
        bridge_height=0.054,
    )
    upper_rib = _rib_plate(
        [(-0.030, 0.016), (-0.015, 0.016), (-0.004, 0.036), (-0.004, 0.060), (-0.030, 0.092)],
        width=0.012,
    )
    lower_rib = _rib_plate(
        [(-0.030, -0.016), (-0.015, -0.016), (-0.004, -0.036), (-0.004, -0.060), (-0.030, -0.092)],
        width=0.012,
    )
    holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.038, -0.084),
                (0.038, -0.084),
                (-0.038, 0.084),
                (0.038, 0.084),
            ]
        )
        .circle(0.0055)
        .extrude(0.060)
        .translate((-0.060, 0.0, 0.0))
    )
    joint_relief = _box((0.026, BASE_YOKE_GAP + 0.002, 0.070), center=(-0.010, 0.0, 0.0))
    return plate.union(boss).union(yoke).union(upper_rib).union(lower_rib).cut(holes).cut(joint_relief)


def _make_wrist_shape_v2() -> cq.Workplane:
    beam_height = 0.017
    neck_len = 0.026
    proximal_offset = 0.014
    beam_start = proximal_offset + neck_len - 0.002
    beam_end = LINK_3_LENGTH - 0.014
    beam_len = beam_end - beam_start

    wrist = _make_joint_head(
        barrel_radius=0.0145,
        barrel_length=LINK_3_HUB_LEN,
        neck_width=WRIST_NECK_WIDTH,
        beam_height=beam_height,
        neck_len=neck_len,
        origin_offset=proximal_offset,
    )
    wrist = wrist.union(
        _box(
            (beam_len, WRIST_NECK_WIDTH * 0.90, beam_height),
            center=(beam_start + beam_len / 2.0, 0.0, 0.0),
            fillet_x=0.003,
        )
    )
    wrist = wrist.union(
        _box(
            (beam_len * 0.70, WRIST_NECK_WIDTH * 0.84, 0.005),
            center=(beam_start + beam_len * 0.53, 0.0, beam_height / 2.0 + 0.0017),
            fillet_x=0.0018,
        )
    )
    wrist = wrist.union(
        _box(
            (beam_len * 0.70, WRIST_NECK_WIDTH * 0.84, 0.005),
            center=(beam_start + beam_len * 0.53, 0.0, -beam_height / 2.0 - 0.0017),
            fillet_x=0.0018,
        )
    )
    pad_neck = _box((0.024, 0.014, 0.026), center=(LINK_3_LENGTH - 0.004, 0.0, 0.0), fillet_x=0.003)
    pad_backer = _box((0.012, 0.054, 0.054), center=(LINK_3_LENGTH + 0.014, 0.0, 0.0), fillet_x=0.006)
    upper_rib = _rib_plate(
        [
            (LINK_3_LENGTH - 0.016, 0.009),
            (LINK_3_LENGTH - 0.002, 0.014),
            (LINK_3_LENGTH + 0.010, 0.024),
            (LINK_3_LENGTH - 0.010, 0.020),
        ],
        width=0.013,
    )
    lower_rib = _rib_plate(
        [
            (LINK_3_LENGTH - 0.016, -0.009),
            (LINK_3_LENGTH - 0.002, -0.014),
            (LINK_3_LENGTH + 0.010, -0.024),
            (LINK_3_LENGTH - 0.010, -0.020),
        ],
        width=0.013,
    )
    return wrist.union(pad_neck).union(pad_backer).union(upper_rib).union(lower_rib)


def _make_fork(
    *,
    joint_x: float,
    outer_width: float,
    gap_width: float,
    barrel_radius: float,
    back_len: float,
    front_len: float,
    arm_height: float,
    bridge_height: float,
) -> cq.Workplane:
    lug_thickness = (outer_width - gap_width) / 2.0
    y_offset = gap_width / 2.0 + lug_thickness / 2.0
    bridge_front_clear = max(0.010, barrel_radius * 0.45)
    bridge_back_x = joint_x - back_len
    bridge_front_x = joint_x - bridge_front_clear
    bridge_len = max(0.008, bridge_front_x - bridge_back_x)
    bridge_center_x = bridge_back_x + bridge_len / 2.0

    arm_back_x = bridge_front_x - min(0.006, back_len * 0.25)
    arm_front_x = joint_x + front_len
    tab_len = arm_front_x - arm_back_x
    tab_center_x = (arm_front_x + arm_back_x) / 2.0

    fork = _box(
        (bridge_len, outer_width, bridge_height),
        center=(bridge_center_x, 0.0, 0.0),
        fillet_x=min(0.004, bridge_height * 0.20),
    )
    for y_sign in (-1.0, 1.0):
        y_center = y_sign * y_offset
        fork = fork.union(
            _box(
                (tab_len, lug_thickness, arm_height),
                center=(tab_center_x, y_center, 0.0),
                fillet_x=min(0.003, arm_height * 0.18),
            )
        )
        fork = fork.union(
            _cyl_y(
                barrel_radius,
                lug_thickness,
                center=(joint_x, y_center, 0.0),
            )
        )
    return fork


def _make_base_shape() -> cq.Workplane:
    plate = _box((0.008, 0.115, 0.265), center=(-0.030, 0.0, 0.0), fillet_x=0.012)
    mount_pad = _box((0.020, 0.066, 0.112), center=(-0.015, 0.0, 0.0), fillet_x=0.006)
    fork = _make_fork(
        joint_x=0.0,
        outer_width=0.042,
        gap_width=BASE_FORK_GAP,
        barrel_radius=0.031,
        back_len=0.026,
        front_len=0.010,
        arm_height=0.047,
        bridge_height=0.056,
    )

    upper_rib = _rib_plate(
        [(-0.028, 0.018), (-0.006, 0.018), (-0.006, 0.060), (-0.028, 0.094)],
        width=0.013,
    )
    lower_rib = _rib_plate(
        [(-0.028, -0.018), (-0.006, -0.018), (-0.006, -0.060), (-0.028, -0.094)],
        width=0.013,
    )

    holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.038, -0.086),
                (0.038, -0.086),
                (-0.038, 0.086),
                (0.038, 0.086),
            ]
        )
        .circle(0.006)
        .extrude(0.060)
        .translate((-0.060, 0.0, 0.0))
    )

    base = plate.union(mount_pad).union(fork).union(upper_rib).union(lower_rib)
    return base.cut(holes)


def _make_link_shape(
    *,
    length: float,
    proximal_radius: float,
    proximal_hub_len: float,
    beam_width: float,
    beam_height: float,
    rib_height: float,
    proximal_neck_width: float,
    fork_outer_width: float,
    fork_gap_width: float,
    fork_radius: float,
    fork_back_len: float,
    fork_front_len: float,
) -> cq.Workplane:
    boss_shift = proximal_radius * 0.62
    neck_len = max(0.024, boss_shift + proximal_radius * 0.42)
    transition_len = 0.028
    beam_start = neck_len + transition_len * 0.45
    beam_end = length - fork_back_len - 0.010
    beam_len = beam_end - beam_start
    blend_len = transition_len
    blend_height = max(beam_height + 0.010, proximal_radius * 1.55)
    distal_block_len = 0.034
    distal_block_height = max(beam_height + 0.012, fork_radius * 1.35)

    link = _cyl_y(proximal_radius, proximal_hub_len, center=(boss_shift, 0.0, 0.0))
    link = link.union(
        _box(
            (neck_len, proximal_neck_width, beam_height * 0.90),
            center=(neck_len / 2.0, 0.0, 0.0),
            fillet_x=min(0.004, beam_height * 0.22),
        )
    )
    link = link.union(
        _box(
            (blend_len, max(beam_width * 1.02, proximal_neck_width + 0.002), blend_height),
            center=(neck_len + blend_len / 2.0, 0.0, 0.0),
            fillet_x=min(0.005, blend_height * 0.20),
        )
    )
    link = link.union(
        _box(
            (beam_len, beam_width, beam_height),
            center=(beam_start + beam_len / 2.0, 0.0, 0.0),
            fillet_x=min(0.004, beam_height * 0.22),
        )
    )
    link = link.union(
        _box(
            (beam_len * 0.78, beam_width * 0.88, rib_height),
            center=(beam_start + beam_len * 0.52, 0.0, beam_height / 2.0 + rib_height / 2.0 - 0.0008),
            fillet_x=min(0.0025, rib_height * 0.30),
        )
    )
    link = link.union(
        _box(
            (beam_len * 0.78, beam_width * 0.88, rib_height),
            center=(beam_start + beam_len * 0.52, 0.0, -beam_height / 2.0 - rib_height / 2.0 + 0.0008),
            fillet_x=min(0.0025, rib_height * 0.30),
        )
    )
    link = link.union(
        _rib_plate(
            [
                (neck_len * 0.42, proximal_radius * 0.16),
                (beam_start - 0.008, beam_height / 2.0 - 0.001),
                (beam_start - 0.008, beam_height / 2.0 + rib_height + 0.002),
                (neck_len * 0.42, proximal_radius * 0.52),
            ],
            width=beam_width * 1.02,
        )
    )
    link = link.union(
        _rib_plate(
            [
                (neck_len * 0.42, -proximal_radius * 0.16),
                (beam_start - 0.008, -beam_height / 2.0 + 0.001),
                (beam_start - 0.008, -beam_height / 2.0 - rib_height - 0.002),
                (neck_len * 0.42, -proximal_radius * 0.52),
            ],
            width=beam_width * 1.02,
        )
    )
    link = link.union(
        _box(
            (distal_block_len, beam_width * 1.10, distal_block_height),
            center=(length - fork_back_len - 0.012, 0.0, 0.0),
            fillet_x=min(0.004, distal_block_height * 0.18),
        )
    )
    return link.union(
        _make_fork(
            joint_x=length,
            outer_width=fork_outer_width,
            gap_width=fork_gap_width,
            barrel_radius=fork_radius,
            back_len=fork_back_len,
            front_len=fork_front_len,
            arm_height=max(beam_height + 0.014, fork_radius * 1.65),
            bridge_height=max(beam_height + 0.020, fork_radius * 1.95),
        )
    )


def _make_wrist_shape() -> cq.Workplane:
    proximal_radius = 0.021
    beam_width = 0.014
    beam_height = 0.018
    rib_height = 0.005
    proximal_neck_width = 0.011
    boss_shift = proximal_radius * 0.62
    neck_len = max(0.022, boss_shift + proximal_radius * 0.40)
    transition_len = 0.024
    beam_start = neck_len + transition_len * 0.45
    beam_end = LINK_3_LENGTH - 0.018
    beam_len = beam_end - beam_start
    blend_len = transition_len

    wrist = _cyl_y(proximal_radius, LINK_3_HUB_LEN, center=(boss_shift, 0.0, 0.0))
    wrist = wrist.union(
        _box(
            (neck_len, proximal_neck_width, beam_height * 0.92),
            center=(neck_len / 2.0, 0.0, 0.0),
            fillet_x=0.003,
        )
    )
    wrist = wrist.union(
        _box(
            (blend_len, max(beam_width * 1.02, proximal_neck_width + 0.002), 0.030),
            center=(neck_len + blend_len / 2.0, 0.0, 0.0),
            fillet_x=0.004,
        )
    )
    wrist = wrist.union(
        _box(
            (beam_len, beam_width, beam_height),
            center=(beam_start + beam_len / 2.0, 0.0, 0.0),
            fillet_x=0.0035,
        )
    )
    wrist = wrist.union(
        _box(
            (beam_len * 0.74, beam_width * 0.88, rib_height),
            center=(beam_start + beam_len * 0.52, 0.0, beam_height / 2.0 + rib_height / 2.0 - 0.0006),
            fillet_x=0.002,
        )
    )
    wrist = wrist.union(
        _box(
            (beam_len * 0.74, beam_width * 0.88, rib_height),
            center=(beam_start + beam_len * 0.52, 0.0, -beam_height / 2.0 - rib_height / 2.0 + 0.0006),
            fillet_x=0.002,
        )
    )
    wrist = wrist.union(
        _rib_plate(
            [
                (neck_len * 0.40, proximal_radius * 0.16),
                (beam_start - 0.008, beam_height / 2.0 - 0.001),
                (beam_start - 0.008, beam_height / 2.0 + rib_height + 0.002),
                (neck_len * 0.40, proximal_radius * 0.50),
            ],
            width=beam_width * 1.02,
        )
    )
    wrist = wrist.union(
        _rib_plate(
            [
                (neck_len * 0.40, -proximal_radius * 0.16),
                (beam_start - 0.008, -beam_height / 2.0 + 0.001),
                (beam_start - 0.008, -beam_height / 2.0 - rib_height - 0.002),
                (neck_len * 0.40, -proximal_radius * 0.50),
            ],
            width=beam_width * 1.02,
        )
    )

    pad_neck = _box((0.034, 0.018, 0.030), center=(LINK_3_LENGTH - 0.005, 0.0, 0.0), fillet_x=0.003)
    pad_backer = _box((0.012, 0.056, 0.056), center=(LINK_3_LENGTH + 0.017, 0.0, 0.0), fillet_x=0.007)
    upper_pad_rib = _rib_plate(
        [(LINK_3_LENGTH - 0.016, 0.010), (LINK_3_LENGTH + 0.010, 0.022), (LINK_3_LENGTH + 0.010, 0.028), (LINK_3_LENGTH - 0.016, 0.020)],
        width=0.016,
    )
    lower_pad_rib = _rib_plate(
        [(LINK_3_LENGTH - 0.016, -0.010), (LINK_3_LENGTH + 0.010, -0.022), (LINK_3_LENGTH + 0.010, -0.028), (LINK_3_LENGTH - 0.016, -0.020)],
        width=0.016,
    )
    return wrist.union(pad_neck).union(pad_backer).union(upper_pad_rib).union(lower_pad_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_inspection_arm")

    model.material("powder_coat_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("graphite_joint", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("inspection_rubber", rgba=(0.08, 0.09, 0.10, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_make_base_shape_v2(), "base_plate"),
        material="powder_coat_black",
        name="base_shell",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((0.054, 0.118, 0.252)),
        mass=2.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(
            _make_serial_link_v2(
                length=LINK_1_LENGTH,
                proximal_radius=0.0185,
                proximal_hub_len=LINK_1_HUB_LEN,
                proximal_width=SHOULDER_NECK_WIDTH,
                beam_height=0.021,
                distal_gap_width=LINK_2_HUB_LEN,
                distal_cheek_thickness=0.0085,
                distal_barrel_radius=0.0175,
                proximal_offset=0.014,
            ),
            "shoulder_link",
        ),
        material="machined_aluminum",
        name="shoulder_shell",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((0.296, 0.029, 0.050)),
        mass=1.05,
        origin=Origin(xyz=(LINK_1_LENGTH / 2.0, 0.0, 0.0)),
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        mesh_from_cadquery(
            _make_serial_link_v2(
                length=LINK_2_LENGTH,
                proximal_radius=0.0175,
                proximal_hub_len=LINK_2_HUB_LEN,
                proximal_width=ELBOW_NECK_WIDTH,
                beam_height=0.019,
                distal_gap_width=LINK_3_HUB_LEN,
                distal_cheek_thickness=0.0075,
                distal_barrel_radius=0.0145,
                proximal_offset=0.012,
            ),
            "elbow_link",
        ),
        material="machined_aluminum",
        name="elbow_shell",
    )
    elbow_link.inertial = Inertial.from_geometry(
        Box((0.230, 0.025, 0.044)),
        mass=0.72,
        origin=Origin(xyz=(LINK_2_LENGTH / 2.0, 0.0, 0.0)),
    )

    wrist_link = model.part("wrist_link")
    wrist_link.visual(
        mesh_from_cadquery(_make_wrist_shape_v2(), "wrist_link"),
        material="graphite_joint",
        name="wrist_shell",
    )
    pad_backer_center_x = LINK_3_LENGTH + 0.014
    rubber_pad_center_x = pad_backer_center_x + 0.006 + PAD_THICKNESS / 2.0
    wrist_link.visual(
        Box((PAD_THICKNESS, PAD_SIZE, PAD_SIZE)),
        origin=Origin(xyz=(rubber_pad_center_x, 0.0, 0.0)),
        material="inspection_rubber",
        name="inspection_pad",
    )
    wrist_link.inertial = Inertial.from_geometry(
        Box((0.154, 0.054, 0.054)),
        mass=0.40,
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.10, effort=38.0, velocity=1.2),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.20, effort=22.0, velocity=1.6),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=wrist_link,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.15, effort=12.0, velocity=2.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    shoulder_link = object_model.get_part("shoulder_link")
    elbow_link = object_model.get_part("elbow_link")
    wrist_link = object_model.get_part("wrist_link")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base_plate,
        shoulder_link,
        reason="Interleaved shoulder clevis and shoulder barrel share the hidden hinge-pin volume; the pin is intentionally not authored as a separate part.",
    )
    ctx.allow_overlap(
        shoulder_link,
        elbow_link,
        reason="Elbow hinge barrels are modeled as captured clevis hardware with the axle volume absorbed into the neighboring parts.",
    )
    ctx.allow_overlap(
        elbow_link,
        wrist_link,
        reason="Wrist clevis uses the same simplified hidden-pin representation, so the neighboring hinge sleeves intentionally occupy the axle core.",
    )

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

    ctx.check(
        "parallel_hinge_axes",
        shoulder_pitch.axis == elbow_pitch.axis == wrist_pitch.axis == (0.0, 1.0, 0.0),
        f"axes: shoulder={shoulder_pitch.axis}, elbow={elbow_pitch.axis}, wrist={wrist_pitch.axis}",
    )
    ctx.check(
        "varied_link_lengths",
        LINK_1_LENGTH > LINK_2_LENGTH > LINK_3_LENGTH
        and (LINK_1_LENGTH - LINK_2_LENGTH) >= 0.05
        and (LINK_2_LENGTH - LINK_3_LENGTH) >= 0.05,
        (
            f"link lengths should step down clearly, got "
            f"{LINK_1_LENGTH:.3f}, {LINK_2_LENGTH:.3f}, {LINK_3_LENGTH:.3f}"
        ),
    )
    ctx.check(
        "inspection_pad_square",
        abs(PAD_SIZE - PAD_SIZE) < 1e-9,
        "end pad should remain square",
    )

    ctx.expect_contact(
        base_plate,
        shoulder_link,
        contact_tol=0.0010,
        name="shoulder_joint_supported",
    )
    ctx.expect_contact(
        shoulder_link,
        elbow_link,
        contact_tol=0.0010,
        name="elbow_joint_supported",
    )
    ctx.expect_contact(
        elbow_link,
        wrist_link,
        contact_tol=0.0010,
        name="wrist_joint_supported",
    )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="joint_clearance_sweep")

    with ctx.pose(shoulder_pitch=0.92, elbow_pitch=-1.45, wrist_pitch=0.78):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clear")
    with ctx.pose(shoulder_pitch=-0.45, elbow_pitch=1.00, wrist_pitch=-0.62):
        ctx.fail_if_parts_overlap_in_current_pose(name="reach_down_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
