from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CENTER_GAP = 0.0088
CHEEK_WIDTH = 0.0046
OUTER_WIDTH = CENTER_GAP + 2.0 * CHEEK_WIDTH
CHEEK_CENTER_Y = CENTER_GAP / 2.0 + CHEEK_WIDTH / 2.0
BARREL_RADIUS = 0.0064
PIN_RADIUS = 0.0020
PIN_HOLE_RADIUS = 0.00235
PIN_HEAD_RADIUS = 0.0036
PIN_HEAD_THICKNESS = 0.0014
PIN_HEAD_CLEARANCE = 0.0005
PIN_HEAD_CENTER_Y = OUTER_WIDTH / 2.0 + PIN_HEAD_THICKNESS / 2.0 + PIN_HEAD_CLEARANCE
FORK_LENGTH = 0.018

BASE_TAIL_LENGTH = 0.046
PROXIMAL_LENGTH = 0.092
MIDDLE_LENGTH = 0.078
DISTAL_SUPPORT_LENGTH = 0.060

PAD_START_X = 0.042
PAD_BLEND_X = 0.052
PAD_END_X = 0.074
PAD_WIDTH = 0.018
PAD_STEM_WIDTH = 0.0072
PAD_BODY_THICKNESS = 0.0030
PAD_NOSE_THICKNESS = 0.0024
PAD_Z_OFFSET = -0.0016


def x_box(
    length: float,
    width: float,
    height: float,
    x_start: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x_start, y_center, z_center))
    )


def y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def fuse_all(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def make_cheek_fork(joint_x: float, plate_start_x: float, plate_height: float) -> cq.Workplane:
    left_plate = x_box(
        joint_x - plate_start_x,
        CHEEK_WIDTH,
        plate_height,
        plate_start_x,
        CHEEK_CENTER_Y,
        0.0,
    )
    right_plate = x_box(
        joint_x - plate_start_x,
        CHEEK_WIDTH,
        plate_height,
        plate_start_x,
        -CHEEK_CENTER_Y,
        0.0,
    )
    left_barrel = y_cylinder(
        BARREL_RADIUS,
        CHEEK_WIDTH,
        x=joint_x,
        y=CHEEK_CENTER_Y,
        z=0.0,
    )
    right_barrel = y_cylinder(
        BARREL_RADIUS,
        CHEEK_WIDTH,
        x=joint_x,
        y=-CHEEK_CENTER_Y,
        z=0.0,
    )
    fork = fuse_all(left_plate, right_plate, left_barrel, right_barrel)
    return fork.cut(y_cylinder(PIN_HOLE_RADIUS, OUTER_WIDTH + 0.003, x=joint_x, z=0.0))


def make_pin_assembly() -> cq.Workplane:
    shaft = y_cylinder(
        PIN_RADIUS,
        OUTER_WIDTH + 2.0 * (PIN_HEAD_THICKNESS + PIN_HEAD_CLEARANCE),
        x=0.0,
        y=0.0,
        z=0.0,
    )
    left_head = y_cylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        x=0.0,
        y=PIN_HEAD_CENTER_Y,
        z=0.0,
    )
    right_head = y_cylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        x=0.0,
        y=-PIN_HEAD_CENTER_Y,
        z=0.0,
    )
    return fuse_all(shaft, left_head, right_head)


def make_root_hub() -> cq.Workplane:
    return y_cylinder(BARREL_RADIUS, CENTER_GAP - 0.0006, x=0.0, y=0.0, z=0.0)


def make_link_shell(
    *,
    length: float,
    beam_width: float,
    beam_height: float,
    bridge_width: float,
    include_distal_fork: bool,
) -> cq.Workplane:
    root_hub = make_root_hub()
    beam_length = length - FORK_LENGTH if include_distal_fork else length
    beam = x_box(beam_length, beam_width, beam_height, 0.0)
    if include_distal_fork:
        fork_bridge = x_box(0.008, bridge_width, beam_height, length - FORK_LENGTH - 0.001)
        fork = make_cheek_fork(length, length - FORK_LENGTH, beam_height + 0.0022)
        return fuse_all(root_hub, beam, fork_bridge, fork)
    return fuse_all(root_hub, beam)


def make_base_shell() -> cq.Workplane:
    tail = x_box(BASE_TAIL_LENGTH, 0.0108, 0.0094, -BASE_TAIL_LENGTH)
    mounting_shoe = x_box(0.014, 0.0142, 0.0102, -BASE_TAIL_LENGTH - 0.014)
    fork = make_cheek_fork(0.0, -FORK_LENGTH, 0.0108)
    return fuse_all(tail, mounting_shoe, fork)


def make_distal_shell() -> cq.Workplane:
    root_hub = make_root_hub()
    beam = x_box(DISTAL_SUPPORT_LENGTH, 0.0068, 0.0072, 0.0)
    pad_support = x_box(0.022, 0.0100, 0.0054, 0.038, 0.0, -0.0008)
    return fuse_all(root_hub, beam, pad_support)


def make_tip_pad() -> cq.Workplane:
    stem = x_box(
        PAD_BLEND_X - PAD_START_X,
        PAD_STEM_WIDTH,
        PAD_BODY_THICKNESS,
        PAD_START_X,
        z_center=PAD_Z_OFFSET,
    )
    shoulder = x_box(
        0.010,
        0.012,
        PAD_BODY_THICKNESS,
        PAD_BLEND_X - 0.001,
        z_center=PAD_Z_OFFSET,
    )
    blade = x_box(
        0.014,
        0.015,
        PAD_BODY_THICKNESS,
        PAD_BLEND_X + 0.008,
        z_center=PAD_Z_OFFSET,
    )
    nose = (
        cq.Workplane("YZ")
        .ellipse(PAD_WIDTH / 2.0, PAD_NOSE_THICKNESS / 2.0)
        .extrude(0.010)
        .translate((PAD_END_X - 0.010, 0.0, PAD_Z_OFFSET))
    )
    return fuse_all(stem, shoulder, blade, nose)


def add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    x_start: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
    material=None,
) -> None:
    sx, sy, sz = size
    part.visual(
        Box(size),
        origin=Origin(xyz=(x_start + sx / 2.0, y_center, z_center)),
        material=material,
        name=name,
    )


def add_y_barrel_visual(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    x_center: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
    material=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(x_center, y_center, z_center),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def add_fork_visuals(
    part,
    *,
    prefix: str,
    joint_x: float,
    x_start: float,
    plate_height: float,
    material=None,
    barrel_material=None,
) -> None:
    plate_length = joint_x - x_start
    add_box_visual(
        part,
        name=f"{prefix}_left_cheek",
        size=(plate_length, CHEEK_WIDTH, plate_height),
        x_start=x_start,
        y_center=CHEEK_CENTER_Y,
        material=material,
    )
    add_box_visual(
        part,
        name=f"{prefix}_right_cheek",
        size=(plate_length, CHEEK_WIDTH, plate_height),
        x_start=x_start,
        y_center=-CHEEK_CENTER_Y,
        material=material,
    )
    add_y_barrel_visual(
        part,
        name=f"{prefix}_left_barrel",
        radius=BARREL_RADIUS,
        length=CHEEK_WIDTH,
        x_center=joint_x,
        y_center=CHEEK_CENTER_Y,
        material=barrel_material,
    )
    add_y_barrel_visual(
        part,
        name=f"{prefix}_right_barrel",
        radius=BARREL_RADIUS,
        length=CHEEK_WIDTH,
        x_center=joint_x,
        y_center=-CHEEK_CENTER_Y,
        material=barrel_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="probing_finger_chain")

    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.76, 1.0))
    pad_black = model.material("pad_black", rgba=(0.11, 0.12, 0.13, 1.0))

    base = model.part("base_mount")
    add_box_visual(
        base,
        name="base_shell",
        size=(0.050, 0.010, 0.0072),
        x_start=-0.060,
        material=graphite,
    )
    add_box_visual(
        base,
        name="base_anchor",
        size=(0.014, 0.013, 0.0086),
        x_start=-0.064,
        material=graphite,
    )
    add_box_visual(
        base,
        name="base_yoke",
        size=(0.010, OUTER_WIDTH, 0.0068),
        x_start=-0.024,
        material=graphite,
    )
    add_fork_visuals(
        base,
        prefix="base",
        joint_x=0.0,
        x_start=-FORK_LENGTH,
        plate_height=0.0086,
        material=graphite,
        barrel_material=steel,
    )

    proximal = model.part("proximal_link")
    add_y_barrel_visual(
        proximal,
        name="proximal_root_barrel",
        radius=BARREL_RADIUS,
        length=CENTER_GAP,
        x_center=0.0,
        material=steel,
    )
    add_box_visual(
        proximal,
        name="proximal_shell",
        size=(PROXIMAL_LENGTH - FORK_LENGTH + 0.003, 0.0076, 0.0058),
        x_start=-0.0015,
        material=graphite,
    )
    add_box_visual(
        proximal,
        name="proximal_knuckle_bridge",
        size=(0.009, OUTER_WIDTH, 0.0060),
        x_start=PROXIMAL_LENGTH - FORK_LENGTH - 0.006,
        material=graphite,
    )
    add_fork_visuals(
        proximal,
        prefix="proximal",
        joint_x=PROXIMAL_LENGTH,
        x_start=PROXIMAL_LENGTH - FORK_LENGTH,
        plate_height=0.0082,
        material=graphite,
        barrel_material=steel,
    )

    middle = model.part("middle_link")
    add_y_barrel_visual(
        middle,
        name="middle_root_barrel",
        radius=BARREL_RADIUS,
        length=CENTER_GAP,
        x_center=0.0,
        material=steel,
    )
    add_box_visual(
        middle,
        name="middle_shell",
        size=(MIDDLE_LENGTH - FORK_LENGTH + 0.003, 0.0068, 0.0052),
        x_start=-0.0015,
        material=graphite,
    )
    add_box_visual(
        middle,
        name="middle_knuckle_bridge",
        size=(0.008, OUTER_WIDTH, 0.0056),
        x_start=MIDDLE_LENGTH - FORK_LENGTH - 0.006,
        material=graphite,
    )
    add_fork_visuals(
        middle,
        prefix="middle",
        joint_x=MIDDLE_LENGTH,
        x_start=MIDDLE_LENGTH - FORK_LENGTH,
        plate_height=0.0078,
        material=graphite,
        barrel_material=steel,
    )

    distal = model.part("distal_link")
    add_y_barrel_visual(
        distal,
        name="distal_root_barrel",
        radius=BARREL_RADIUS,
        length=CENTER_GAP,
        x_center=0.0,
        material=steel,
    )
    add_box_visual(
        distal,
        name="distal_shell",
        size=(0.058, 0.0058, 0.0048),
        x_start=-0.0015,
        material=graphite,
    )
    add_box_visual(
        distal,
        name="distal_support",
        size=(0.018, 0.010, 0.0042),
        x_start=0.035,
        z_center=-0.0010,
        material=graphite,
    )
    distal.visual(
        mesh_from_cadquery(make_tip_pad(), "distal_tip_pad"),
        origin=Origin(),
        material=pad_black,
        name="tip_pad",
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-0.10, upper=1.05),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-0.10, upper=1.10),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.10, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base_mount")
    proximal = object_model.get_part("proximal_link")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")

    base_to_proximal = object_model.get_articulation("base_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    ctx.check(
        "part_and_joint_count",
        len(object_model.parts) == 4 and len(object_model.articulations) == 3,
        details=f"expected 4 parts and 3 articulations, got {len(object_model.parts)} parts and {len(object_model.articulations)} articulations",
    )

    for joint in (base_to_proximal, proximal_to_middle, middle_to_distal):
        limits = joint.motion_limits
        axis_ok = tuple(joint.axis) == (0.0, 1.0, 0.0)
        limits_ok = (
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= 0.0 < limits.upper
        )
        ctx.check(
            f"{joint.name}_configured",
            axis_ok and limits_ok,
            details=f"{joint.name} axis={joint.axis} limits={limits}",
        )

    ctx.expect_contact(
        base,
        proximal,
        elem_b="proximal_root_barrel",
        contact_tol=1e-6,
        name="base_joint_is_physically_pinned",
    )
    ctx.expect_contact(
        proximal,
        middle,
        elem_b="middle_root_barrel",
        contact_tol=1e-6,
        name="middle_joint_is_physically_pinned",
    )
    ctx.expect_contact(
        middle,
        distal,
        elem_b="distal_root_barrel",
        contact_tol=1e-6,
        name="distal_joint_is_physically_pinned",
    )

    with ctx.pose(
        {
            base_to_proximal: 0.55,
            proximal_to_middle: 0.65,
            middle_to_distal: 0.45,
        }
    ):
        ctx.expect_contact(
            base,
            proximal,
            elem_b="proximal_root_barrel",
            contact_tol=1e-6,
            name="base_joint_stays_engaged_when_bent",
        )
        ctx.expect_contact(
            proximal,
            middle,
            elem_b="middle_root_barrel",
            contact_tol=1e-6,
            name="middle_joint_stays_engaged_when_bent",
        )
        ctx.expect_contact(
            middle,
            distal,
            elem_b="distal_root_barrel",
            contact_tol=1e-6,
            name="distal_joint_stays_engaged_when_bent",
        )

    rest_tip_pos = ctx.part_world_position(distal)
    with ctx.pose(
        {
            base_to_proximal: 0.55,
            proximal_to_middle: 0.65,
            middle_to_distal: 0.45,
        }
    ):
        bent_tip_pos = ctx.part_world_position(distal)
    ctx.check(
        "finger_bends_in_single_xz_plane",
        rest_tip_pos is not None
        and bent_tip_pos is not None
        and abs(bent_tip_pos[2] - rest_tip_pos[2]) > 0.035
        and abs(bent_tip_pos[1] - rest_tip_pos[1]) < 0.003,
        details=f"rest={rest_tip_pos} bent={bent_tip_pos}",
    )

    shell_aabb = ctx.part_element_world_aabb(distal, elem="distal_shell")
    pad_aabb = ctx.part_element_world_aabb(distal, elem="tip_pad")
    if shell_aabb is None or pad_aabb is None:
        ctx.fail("tip_pad_bounds_available", "missing distal_shell or tip_pad AABB")
    else:
        shell_size = tuple(shell_aabb[1][i] - shell_aabb[0][i] for i in range(3))
        pad_size = tuple(pad_aabb[1][i] - pad_aabb[0][i] for i in range(3))
        pad_front = pad_aabb[1][0]
        shell_front = shell_aabb[1][0]
        ctx.check(
            "tip_pad_is_spatulate",
            pad_size[1] > shell_size[1] * 1.7
            and pad_size[2] < shell_size[2] * 0.8
            and pad_front > shell_front + 0.010,
            details=f"shell_size={shell_size} pad_size={pad_size} shell_front={shell_front:.4f} pad_front={pad_front:.4f}",
        )

        overall_reach = pad_front - ctx.part_element_world_aabb(base, elem="base_shell")[0][0]
        ctx.check(
            "overall_chain_is_slim_and_tool_like",
            overall_reach > 0.26 and overall_reach / pad_size[1] > 12.0,
            details=f"overall_reach={overall_reach:.4f} tip_width={pad_size[1]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
