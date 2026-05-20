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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PALM_THICKNESS = 0.026
PALM_WIDTH = 0.158
PALM_HEIGHT = 0.168
PALM_FRONT_X = PALM_THICKNESS / 2.0

PIN_RADIUS = 0.0065
TONGUE_WIDTH = 0.0105
CLEVIS_OUTER_WIDTH = 0.018
TOWER_OUTER_WIDTH = 0.024
JOINT_CLEARANCE = 0.0
CLEVIS_SIDE_WIDTH = (CLEVIS_OUTER_WIDTH - TONGUE_WIDTH - 2.0 * JOINT_CLEARANCE) / 2.0
TOWER_SIDE_WIDTH = (TOWER_OUTER_WIDTH - TONGUE_WIDTH - 2.0 * JOINT_CLEARANCE) / 2.0
SHANK_START = PIN_RADIUS + 0.003

TOWER_PIVOT_X = PALM_FRONT_X + 0.022
TOWER_PIVOT_Z = 0.028
FINGER_CENTER_Y = 0.0145

PROXIMAL_LENGTH = 0.072
MIDDLE_LENGTH = 0.052
DISTAL_LENGTH = 0.040
TIP_PAD_LENGTH = 0.016
TIP_PAD_WIDTH = 0.013
TIP_PAD_HEIGHT = 0.011
FORK_LENGTH = 0.016
ROOT_NECK_LENGTH = SHANK_START - PIN_RADIUS
TOWER_CHEEK_LENGTH = 0.014
TOWER_CHEEK_HEIGHT = 0.022

ROOT_LIMITS = MotionLimits(effort=14.0, velocity=2.0, lower=-0.25, upper=1.05)
MID_LIMITS = MotionLimits(effort=10.0, velocity=2.2, lower=-0.10, upper=1.30)
TIP_LIMITS = MotionLimits(effort=8.0, velocity=2.4, lower=-0.05, upper=1.10)


def make_palm_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(PALM_WIDTH, PALM_HEIGHT)
        .extrude(PALM_THICKNESS / 2.0, both=True)
    )

    corner_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.052, -0.056),
                (-0.052, 0.056),
                (0.052, -0.056),
                (0.052, 0.056),
            ]
        )
        .circle(0.006)
        .extrude(PALM_THICKNESS * 0.8, both=True)
    )

    front_relief = (
        cq.Workplane("YZ")
        .rect(0.090, 0.090)
        .extrude(0.009)
        .translate((PALM_FRONT_X - 0.0045, 0.0, 0.0))
    )

    bridge_rib = (
        cq.Workplane("YZ")
        .rect(0.070, 0.018)
        .extrude(0.007)
        .translate((PALM_FRONT_X - 0.0035, 0.0, -0.034))
    )

    return plate.cut(corner_holes).cut(front_relief).union(bridge_rib)


def make_tower() -> cq.Workplane:
    cheek_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.022, -0.052),
                (-0.022, -0.012),
                (-0.017, 0.011),
                (0.008, 0.011),
                (0.008, -0.011),
                (-0.007, -0.011),
                (-0.012, -0.026),
                (-0.012, -0.052),
            ]
        )
        .close()
        .extrude(TOWER_SIDE_WIDTH / 2.0, both=True)
    )

    boss = (
        cq.Workplane("XZ")
        .circle(PIN_RADIUS + 0.0025)
        .extrude(TOWER_SIDE_WIDTH / 2.0, both=True)
    )

    cheek_offset = (TONGUE_WIDTH + JOINT_CLEARANCE + TOWER_SIDE_WIDTH) / 2.0
    left_cheek = cheek_profile.union(boss).translate((0.0, cheek_offset, 0.0))
    right_cheek = cheek_profile.union(boss).translate((0.0, -cheek_offset, 0.0))

    rear_mount = (
        cq.Workplane("XY")
        .box(0.008, TOWER_OUTER_WIDTH, 0.070)
        .translate((-0.018, 0.0, -0.022))
    )

    bridge = (
        cq.Workplane("XY")
        .box(0.010, TOWER_OUTER_WIDTH, 0.020)
        .translate((-0.010, 0.0, -0.034))
    )

    return rear_mount.union(bridge).union(left_cheek).union(right_cheek)


def make_link_segment(
    length: float,
    root_height: float,
    tip_height: float,
    *,
    name_bump: float,
) -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (SHANK_START, -root_height / 2.0),
                (length - 0.018, -tip_height / 2.0),
                (length - 0.018, tip_height / 2.0),
                (SHANK_START, root_height / 2.0),
            ]
        )
        .close()
        .extrude(TONGUE_WIDTH / 2.0, both=True)
    )

    root_barrel = (
        cq.Workplane("XZ")
        .circle(PIN_RADIUS)
        .extrude((TONGUE_WIDTH + JOINT_CLEARANCE) / 2.0, both=True)
    )

    cheek_block = (
        cq.Workplane("XY")
        .box(0.016, CLEVIS_SIDE_WIDTH, max(tip_height + 0.004, 2.0 * PIN_RADIUS + 0.004))
        .translate((length - 0.010, 0.0, 0.0))
    )

    cheek_boss = (
        cq.Workplane("XZ")
        .center(length, 0.0)
        .circle(PIN_RADIUS + name_bump)
        .extrude(CLEVIS_SIDE_WIDTH / 2.0, both=True)
    )

    cheek_offset = (TONGUE_WIDTH + JOINT_CLEARANCE + CLEVIS_SIDE_WIDTH) / 2.0
    upper_cheek = cheek_block.union(cheek_boss).translate((0.0, cheek_offset, 0.0))
    lower_cheek = cheek_block.union(cheek_boss).translate((0.0, -cheek_offset, 0.0))

    return body.union(root_barrel).union(upper_cheek).union(lower_cheek)


def make_terminal_segment(length: float, root_height: float, tip_height: float) -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (SHANK_START, -root_height / 2.0),
                (length - 0.012, -tip_height / 2.0),
                (length, -tip_height / 2.0),
                (length, tip_height / 2.0),
                (length - 0.012, tip_height / 2.0),
                (SHANK_START, root_height / 2.0),
            ]
        )
        .close()
        .extrude(TONGUE_WIDTH / 2.0, both=True)
    )

    root_barrel = (
        cq.Workplane("XZ")
        .circle(PIN_RADIUS)
        .extrude((TONGUE_WIDTH + JOINT_CLEARANCE) / 2.0, both=True)
    )

    tip_stub = (
        cq.Workplane("XY")
        .box(0.010, TONGUE_WIDTH, tip_height * 0.95)
        .translate((length - 0.005, 0.0, 0.0))
    )

    return body.union(root_barrel).union(tip_stub)


def make_tapered_body(length: float, root_height: float, tip_height: float, *, end_x: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (SHANK_START, -root_height / 2.0),
                (end_x, -tip_height / 2.0),
                (end_x, tip_height / 2.0),
                (SHANK_START, root_height / 2.0),
            ]
        )
        .close()
        .extrude(TONGUE_WIDTH / 2.0, both=True)
    )


def add_tower_visuals(part, material) -> None:
    y_offset = (TONGUE_WIDTH / 2.0) + (TOWER_SIDE_WIDTH / 2.0)

    part.visual(
        Box((0.008, TOWER_OUTER_WIDTH, 0.070)),
        origin=Origin(xyz=(-0.018, 0.0, -0.022)),
        material=material,
        name="tower_mount",
    )
    part.visual(
        Box((0.010, TOWER_SIDE_WIDTH, 0.040)),
        origin=Origin(xyz=(-0.010, y_offset, -0.010)),
        material=material,
        name="tower_left_spine",
    )
    part.visual(
        Box((0.010, TOWER_SIDE_WIDTH, 0.040)),
        origin=Origin(xyz=(-0.010, -y_offset, -0.010)),
        material=material,
        name="tower_right_spine",
    )
    part.visual(
        Box((TOWER_CHEEK_LENGTH, TOWER_SIDE_WIDTH, TOWER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.002, y_offset, 0.0)),
        material=material,
        name="tower_left_cheek",
    )
    part.visual(
        Box((TOWER_CHEEK_LENGTH, TOWER_SIDE_WIDTH, TOWER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.002, -y_offset, 0.0)),
        material=material,
        name="tower_right_cheek",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS + 0.0025, length=TOWER_SIDE_WIDTH),
        origin=Origin(xyz=(0.0, y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="tower_left_boss",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS + 0.0025, length=TOWER_SIDE_WIDTH),
        origin=Origin(xyz=(0.0, -y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="tower_right_boss",
    )


def add_forked_link_visuals(
    part,
    *,
    mesh_name: str,
    length: float,
    root_height: float,
    tip_height: float,
    boss_radius: float,
    material,
) -> None:
    y_offset = (TONGUE_WIDTH / 2.0) + (CLEVIS_SIDE_WIDTH / 2.0)
    cheek_height = max(tip_height + 0.004, 2.0 * PIN_RADIUS + 0.002)

    part.visual(
        mesh_from_cadquery(
            make_tapered_body(length, root_height, tip_height, end_x=length - FORK_LENGTH + 0.001),
            mesh_name,
        ),
        material=material,
        name="link_body",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=TONGUE_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="root_knuckle",
    )
    part.visual(
        Box((ROOT_NECK_LENGTH, TONGUE_WIDTH, root_height * 0.95)),
        origin=Origin(xyz=(PIN_RADIUS + (ROOT_NECK_LENGTH / 2.0), 0.0, 0.0)),
        material=material,
        name="root_neck",
    )
    part.visual(
        Box((0.008, CLEVIS_OUTER_WIDTH, tip_height + 0.004)),
        origin=Origin(xyz=(length - 0.012, 0.0, 0.0)),
        material=material,
        name="fork_bridge",
    )
    part.visual(
        Box((0.014, CLEVIS_SIDE_WIDTH, cheek_height)),
        origin=Origin(xyz=(length - 0.007, y_offset, 0.0)),
        material=material,
        name="left_fork_cheek",
    )
    part.visual(
        Box((0.014, CLEVIS_SIDE_WIDTH, cheek_height)),
        origin=Origin(xyz=(length - 0.007, -y_offset, 0.0)),
        material=material,
        name="right_fork_cheek",
    )
    part.visual(
        Cylinder(radius=boss_radius, length=CLEVIS_SIDE_WIDTH),
        origin=Origin(xyz=(length, y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="left_fork_boss",
    )
    part.visual(
        Cylinder(radius=boss_radius, length=CLEVIS_SIDE_WIDTH),
        origin=Origin(xyz=(length, -y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="right_fork_boss",
    )


def add_terminal_link_visuals(
    part,
    *,
    mesh_name: str,
    length: float,
    root_height: float,
    tip_height: float,
    material,
) -> None:
    part.visual(
        mesh_from_cadquery(
            make_tapered_body(length, root_height, tip_height, end_x=length - 0.004),
            mesh_name,
        ),
        material=material,
        name="link_body",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=TONGUE_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="root_knuckle",
    )
    part.visual(
        Box((ROOT_NECK_LENGTH, TONGUE_WIDTH, root_height * 0.95)),
        origin=Origin(xyz=(PIN_RADIUS + (ROOT_NECK_LENGTH / 2.0), 0.0, 0.0)),
        material=material,
        name="root_neck",
    )
    part.visual(
        Box((0.008, TONGUE_WIDTH, tip_height * 0.96)),
        origin=Origin(xyz=(length - 0.004, 0.0, 0.0)),
        material=material,
        name="tip_stub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_gripper_hand_study")

    dark_plate = model.material("dark_plate", rgba=(0.20, 0.21, 0.23, 1.0))
    machined_link = model.material("machined_link", rgba=(0.73, 0.75, 0.78, 1.0))
    pad_material = model.material("pad_material", rgba=(0.16, 0.16, 0.18, 1.0))

    palm = model.part(
        "palm_plate",
        inertial=Inertial.from_geometry(
            Box((PALM_THICKNESS, PALM_WIDTH, PALM_HEIGHT)),
            mass=2.4,
        ),
    )
    palm.visual(
        mesh_from_cadquery(make_palm_plate(), "palm_plate"),
        material=dark_plate,
        name="palm_shell",
    )

    left_tower = model.part(
        "left_tower",
        inertial=Inertial.from_geometry(
            Box((0.031, TOWER_OUTER_WIDTH, 0.070)),
            mass=0.34,
            origin=Origin(xyz=(-0.006, 0.0, -0.020)),
        ),
    )
    add_tower_visuals(left_tower, dark_plate)

    right_tower = model.part(
        "right_tower",
        inertial=Inertial.from_geometry(
            Box((0.031, TOWER_OUTER_WIDTH, 0.070)),
            mass=0.34,
            origin=Origin(xyz=(-0.006, 0.0, -0.020)),
        ),
    )
    add_tower_visuals(right_tower, dark_plate)

    left_proximal = model.part(
        "left_proximal",
        inertial=Inertial.from_geometry(
            Box((PROXIMAL_LENGTH + 0.015, CLEVIS_OUTER_WIDTH, 0.022)),
            mass=0.18,
            origin=Origin(xyz=(0.040, 0.0, 0.0)),
        ),
    )
    add_forked_link_visuals(
        left_proximal,
        mesh_name="left_proximal_body",
        length=PROXIMAL_LENGTH,
        root_height=0.021,
        tip_height=0.016,
        boss_radius=PIN_RADIUS + 0.0022,
        material=machined_link,
    )

    left_middle = model.part(
        "left_middle",
        inertial=Inertial.from_geometry(
            Box((MIDDLE_LENGTH + 0.015, CLEVIS_OUTER_WIDTH, 0.019)),
            mass=0.13,
            origin=Origin(xyz=(0.029, 0.0, 0.0)),
        ),
    )
    add_forked_link_visuals(
        left_middle,
        mesh_name="left_middle_body",
        length=MIDDLE_LENGTH,
        root_height=0.017,
        tip_height=0.013,
        boss_radius=PIN_RADIUS + 0.0019,
        material=machined_link,
    )

    left_distal = model.part(
        "left_distal",
        inertial=Inertial.from_geometry(
            Box((DISTAL_LENGTH + TIP_PAD_LENGTH + 0.010, CLEVIS_OUTER_WIDTH, 0.017)),
            mass=0.10,
            origin=Origin(xyz=(0.028, 0.0, 0.0)),
        ),
    )
    add_terminal_link_visuals(
        left_distal,
        mesh_name="left_distal_body",
        length=DISTAL_LENGTH,
        root_height=0.014,
        tip_height=0.011,
        material=machined_link,
    )
    left_distal.visual(
        Box((TIP_PAD_LENGTH, TIP_PAD_WIDTH, TIP_PAD_HEIGHT)),
        origin=Origin(xyz=(DISTAL_LENGTH + (TIP_PAD_LENGTH / 2.0) - 0.0005, 0.0, 0.0)),
        material=pad_material,
        name="tip_pad",
    )

    right_proximal = model.part(
        "right_proximal",
        inertial=Inertial.from_geometry(
            Box((PROXIMAL_LENGTH + 0.015, CLEVIS_OUTER_WIDTH, 0.022)),
            mass=0.18,
            origin=Origin(xyz=(0.040, 0.0, 0.0)),
        ),
    )
    add_forked_link_visuals(
        right_proximal,
        mesh_name="right_proximal_body",
        length=PROXIMAL_LENGTH,
        root_height=0.021,
        tip_height=0.016,
        boss_radius=PIN_RADIUS + 0.0022,
        material=machined_link,
    )

    right_middle = model.part(
        "right_middle",
        inertial=Inertial.from_geometry(
            Box((MIDDLE_LENGTH + 0.015, CLEVIS_OUTER_WIDTH, 0.019)),
            mass=0.13,
            origin=Origin(xyz=(0.029, 0.0, 0.0)),
        ),
    )
    add_forked_link_visuals(
        right_middle,
        mesh_name="right_middle_body",
        length=MIDDLE_LENGTH,
        root_height=0.017,
        tip_height=0.013,
        boss_radius=PIN_RADIUS + 0.0019,
        material=machined_link,
    )

    right_distal = model.part(
        "right_distal",
        inertial=Inertial.from_geometry(
            Box((DISTAL_LENGTH + TIP_PAD_LENGTH + 0.010, CLEVIS_OUTER_WIDTH, 0.017)),
            mass=0.10,
            origin=Origin(xyz=(0.028, 0.0, 0.0)),
        ),
    )
    add_terminal_link_visuals(
        right_distal,
        mesh_name="right_distal_body",
        length=DISTAL_LENGTH,
        root_height=0.014,
        tip_height=0.011,
        material=machined_link,
    )
    right_distal.visual(
        Box((TIP_PAD_LENGTH, TIP_PAD_WIDTH, TIP_PAD_HEIGHT)),
        origin=Origin(xyz=(DISTAL_LENGTH + (TIP_PAD_LENGTH / 2.0) - 0.0005, 0.0, 0.0)),
        material=pad_material,
        name="tip_pad",
    )

    model.articulation(
        "palm_to_left_tower",
        ArticulationType.FIXED,
        parent=palm,
        child=left_tower,
        origin=Origin(xyz=(TOWER_PIVOT_X, FINGER_CENTER_Y, TOWER_PIVOT_Z)),
    )
    model.articulation(
        "palm_to_right_tower",
        ArticulationType.FIXED,
        parent=palm,
        child=right_tower,
        origin=Origin(xyz=(TOWER_PIVOT_X, -FINGER_CENTER_Y, TOWER_PIVOT_Z)),
    )

    model.articulation(
        "left_root_joint",
        ArticulationType.REVOLUTE,
        parent=left_tower,
        child=left_proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=ROOT_LIMITS,
    )
    model.articulation(
        "left_middle_joint",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MID_LIMITS,
    )
    model.articulation(
        "left_tip_joint",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=TIP_LIMITS,
    )

    model.articulation(
        "right_root_joint",
        ArticulationType.REVOLUTE,
        parent=right_tower,
        child=right_proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=ROOT_LIMITS,
    )
    model.articulation(
        "right_middle_joint",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MID_LIMITS,
    )
    model.articulation(
        "right_tip_joint",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=TIP_LIMITS,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm_plate")
    left_tower = object_model.get_part("left_tower")
    right_tower = object_model.get_part("right_tower")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root_joint = object_model.get_articulation("left_root_joint")
    left_middle_joint = object_model.get_articulation("left_middle_joint")
    left_tip_joint = object_model.get_articulation("left_tip_joint")
    right_root_joint = object_model.get_articulation("right_root_joint")
    right_middle_joint = object_model.get_articulation("right_middle_joint")
    right_tip_joint = object_model.get_articulation("right_tip_joint")

    left_pad = left_distal.get_visual("tip_pad")
    right_pad = right_distal.get_visual("tip_pad")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for pair_a, pair_b, reason in (
        (
            left_tower,
            left_proximal,
            "Pinned root knuckle nests between tower cheeks; explicit steel pin omitted from the study.",
        ),
        (
            left_proximal,
            left_middle,
            "Pinned finger knuckle uses interleaved clevis cheeks around the next link root; explicit pin omitted.",
        ),
        (
            left_middle,
            left_distal,
            "Pinned finger knuckle uses interleaved clevis cheeks around the next link root; explicit pin omitted.",
        ),
        (
            right_tower,
            right_proximal,
            "Pinned root knuckle nests between tower cheeks; explicit steel pin omitted from the study.",
        ),
        (
            right_proximal,
            right_middle,
            "Pinned finger knuckle uses interleaved clevis cheeks around the next link root; explicit pin omitted.",
        ),
        (
            right_middle,
            right_distal,
            "Pinned finger knuckle uses interleaved clevis cheeks around the next link root; explicit pin omitted.",
        ),
    ):
        ctx.allow_overlap(pair_a, pair_b, reason=reason)

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
        "bench gripper part count",
        len(object_model.parts) == 9,
        f"expected 9 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "bench gripper articulation count",
        len(object_model.articulations) == 8,
        f"expected 8 articulations, found {len(object_model.articulations)}",
    )

    ctx.expect_contact(left_tower, palm, name="left tower mounted to palm")
    ctx.expect_contact(right_tower, palm, name="right tower mounted to palm")
    ctx.expect_contact(left_tower, left_proximal, name="left tower knuckle contacts proximal link")
    ctx.expect_contact(left_proximal, left_middle, name="left proximal knuckle contacts middle link")
    ctx.expect_contact(left_middle, left_distal, name="left middle knuckle contacts distal link")
    ctx.expect_contact(right_tower, right_proximal, name="right tower knuckle contacts proximal link")
    ctx.expect_contact(right_proximal, right_middle, name="right proximal knuckle contacts middle link")
    ctx.expect_contact(right_middle, right_distal, name="right middle knuckle contacts distal link")

    def check_joint(name: str, joint, lower: float, upper: float) -> None:
        limits = joint.motion_limits
        ctx.check(
            f"{name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"joint type was {joint.articulation_type}",
        )
        ctx.check(
            f"{name} bends about y axis",
            all(abs(a - b) < 1e-9 for a, b in zip(joint.axis, (0.0, 1.0, 0.0))),
            f"axis was {joint.axis}",
        )
        ctx.check(
            f"{name} limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - lower) < 1e-9
            and abs(limits.upper - upper) < 1e-9,
            f"limits were {limits}",
        )

    check_joint("left_root_joint", left_root_joint, ROOT_LIMITS.lower, ROOT_LIMITS.upper)
    check_joint("left_middle_joint", left_middle_joint, MID_LIMITS.lower, MID_LIMITS.upper)
    check_joint("left_tip_joint", left_tip_joint, TIP_LIMITS.lower, TIP_LIMITS.upper)
    check_joint("right_root_joint", right_root_joint, ROOT_LIMITS.lower, ROOT_LIMITS.upper)
    check_joint("right_middle_joint", right_middle_joint, MID_LIMITS.lower, MID_LIMITS.upper)
    check_joint("right_tip_joint", right_tip_joint, TIP_LIMITS.lower, TIP_LIMITS.upper)

    with ctx.pose(
        left_root_joint=0.0,
        left_middle_joint=0.0,
        left_tip_joint=0.0,
        right_root_joint=0.0,
        right_middle_joint=0.0,
        right_tip_joint=0.0,
    ):
        ctx.expect_overlap(
            left_middle,
            left_proximal,
            axes="yz",
            min_overlap=0.009,
            name="left proximal knuckle aligns with middle link",
        )
        ctx.expect_overlap(
            left_distal,
            left_middle,
            axes="yz",
            min_overlap=0.009,
            name="left middle knuckle aligns with distal link",
        )
        ctx.expect_overlap(
            right_middle,
            right_proximal,
            axes="yz",
            min_overlap=0.009,
            name="right proximal knuckle aligns with middle link",
        )
        ctx.expect_overlap(
            right_distal,
            right_middle,
            axes="yz",
            min_overlap=0.009,
            name="right middle knuckle aligns with distal link",
        )
        ctx.expect_gap(
            left_proximal,
            right_proximal,
            axis="y",
            min_gap=0.010,
            max_gap=0.016,
            name="proximal links sit close together without touching",
        )
        ctx.expect_gap(
            left_distal,
            right_distal,
            axis="y",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem=left_pad,
            negative_elem=right_pad,
            name="tip pads stay close but remain separate",
        )

    with ctx.pose(
        left_root_joint=0.40,
        left_middle_joint=0.55,
        left_tip_joint=0.38,
        right_root_joint=0.18,
        right_middle_joint=0.70,
        right_tip_joint=0.42,
    ):
        ctx.expect_gap(
            left_distal,
            right_distal,
            axis="y",
            min_gap=0.009,
            max_gap=0.017,
            positive_elem=left_pad,
            negative_elem=right_pad,
            name="independent finger curls still keep separate pads",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
