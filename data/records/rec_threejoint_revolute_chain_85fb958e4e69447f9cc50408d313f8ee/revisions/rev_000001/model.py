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


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.140
PLATE_HEIGHT = 0.220
PLATE_BOLT_Y = 0.045
PLATE_BOLT_Z = 0.075
PLATE_BOLT_RADIUS = 0.006

SHOULDER_X = 0.045

HUB_RADIUS = 0.010
HUB_WIDTH = 0.012
EAR_THICKNESS = 0.008
JOINT_SIDE_CLEARANCE = 0.0
EAR_Y = HUB_WIDTH / 2 + JOINT_SIDE_CLEARANCE + EAR_THICKNESS / 2
EAR_HEIGHT = 0.034
EAR_BOX_LENGTH = 0.024
EAR_BOX_CENTER_OFFSET = 0.012
DISTAL_WEB_LENGTH = 0.020
DISTAL_WEB_CENTER_OFFSET = 0.020

LINK1_LENGTH = 0.300
LINK2_LENGTH = 0.220
LINK3_LENGTH = 0.120

LINK1_BEAM_WIDTH = 0.014
LINK1_BEAM_HEIGHT = 0.024
LINK2_BEAM_WIDTH = 0.013
LINK2_BEAM_HEIGHT = 0.021
LINK3_BEAM_WIDTH = 0.012
LINK3_BEAM_HEIGHT = 0.018

PAD_STEM_LENGTH = 0.028
PAD_STEM_WIDTH = 0.014
PAD_STEM_HEIGHT = 0.014
PAD_SIZE = 0.055
PAD_THICKNESS = 0.010


def centered_box(dx: float, dy: float, dz: float, cx: float, cy: float, cz: float) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate((cx, cy, cz))


def y_cylinder(
    radius: float,
    length: float,
    cx: float,
    cy: float,
    cz: float,
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((cx, cy - length / 2, cz))


def fuse_all(*solids: cq.Workplane) -> cq.Workplane:
    body = solids[0]
    for solid in solids[1:]:
        body = body.union(solid)
    return body


def make_base_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)
        .translate((PLATE_THICKNESS / 2, 0.0, 0.0))
        .edges("|X")
        .fillet(0.010)
    )
    holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-PLATE_BOLT_Y, -PLATE_BOLT_Z),
                (-PLATE_BOLT_Y, PLATE_BOLT_Z),
                (PLATE_BOLT_Y, -PLATE_BOLT_Z),
                (PLATE_BOLT_Y, PLATE_BOLT_Z),
            ]
        )
        .circle(PLATE_BOLT_RADIUS)
        .extrude(PLATE_THICKNESS + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )

    support_block = centered_box(0.020, 0.050, 0.054, 0.022, 0.0, 0.0)
    shoulder_bridge = centered_box(0.014, 0.020, 0.030, SHOULDER_X - 0.031, 0.0, 0.0)
    shoulder_web = centered_box(0.020, HUB_WIDTH, 0.026, SHOULDER_X - 0.014, 0.0, 0.0)

    left_ear = centered_box(
        EAR_BOX_LENGTH,
        EAR_THICKNESS,
        EAR_HEIGHT,
        SHOULDER_X - EAR_BOX_CENTER_OFFSET,
        EAR_Y,
        0.0,
    )
    right_ear = centered_box(
        EAR_BOX_LENGTH,
        EAR_THICKNESS,
        EAR_HEIGHT,
        SHOULDER_X - EAR_BOX_CENTER_OFFSET,
        -EAR_Y,
        0.0,
    )
    left_boss = y_cylinder(HUB_RADIUS, EAR_THICKNESS, SHOULDER_X - HUB_RADIUS, EAR_Y, 0.0)
    right_boss = y_cylinder(HUB_RADIUS, EAR_THICKNESS, SHOULDER_X - HUB_RADIUS, -EAR_Y, 0.0)

    return fuse_all(
        plate.cut(holes),
        support_block,
        shoulder_bridge,
        shoulder_web,
        left_ear,
        right_ear,
        left_boss,
        right_boss,
    )


def make_arm_link(length: float, beam_width: float, beam_height: float) -> cq.Workplane:
    tongue_height = max(beam_height + 0.004, 0.026)
    proximal_tab = centered_box(0.018, HUB_WIDTH, tongue_height, 0.009, 0.0, 0.0)
    proximal_nose = y_cylinder(HUB_RADIUS, HUB_WIDTH, HUB_RADIUS, 0.0, 0.0)

    beam_start = 0.018
    beam_end = length - 0.028
    beam = centered_box(
        beam_end - beam_start,
        beam_width,
        beam_height,
        (beam_start + beam_end) / 2,
        0.0,
        0.0,
    )
    distal_web = centered_box(
        DISTAL_WEB_LENGTH,
        beam_width,
        beam_height * 0.90,
        length - DISTAL_WEB_CENTER_OFFSET,
        0.0,
        0.0,
    )
    left_ear = centered_box(
        EAR_BOX_LENGTH,
        EAR_THICKNESS,
        max(EAR_HEIGHT, beam_height + 0.010),
        length - EAR_BOX_CENTER_OFFSET,
        EAR_Y,
        0.0,
    )
    right_ear = centered_box(
        EAR_BOX_LENGTH,
        EAR_THICKNESS,
        max(EAR_HEIGHT, beam_height + 0.010),
        length - EAR_BOX_CENTER_OFFSET,
        -EAR_Y,
        0.0,
    )
    left_boss = y_cylinder(HUB_RADIUS, EAR_THICKNESS, length - HUB_RADIUS, EAR_Y, 0.0)
    right_boss = y_cylinder(HUB_RADIUS, EAR_THICKNESS, length - HUB_RADIUS, -EAR_Y, 0.0)
    return fuse_all(
        proximal_tab,
        proximal_nose,
        beam,
        distal_web,
        left_ear,
        right_ear,
        left_boss,
        right_boss,
    )


def make_wrist_link() -> cq.Workplane:
    tongue_height = max(LINK3_BEAM_HEIGHT + 0.004, 0.024)
    proximal_tab = centered_box(0.018, HUB_WIDTH, tongue_height, 0.009, 0.0, 0.0)
    proximal_nose = y_cylinder(HUB_RADIUS, HUB_WIDTH, HUB_RADIUS, 0.0, 0.0)

    beam_start = 0.018
    beam_end = LINK3_LENGTH - 0.022
    beam = centered_box(
        beam_end - beam_start,
        LINK3_BEAM_WIDTH,
        LINK3_BEAM_HEIGHT,
        (beam_start + beam_end) / 2,
        0.0,
        0.0,
    )
    end_block = centered_box(0.020, 0.024, 0.024, LINK3_LENGTH - 0.010, 0.0, 0.0)
    end_plate = centered_box(0.004, 0.030, 0.030, LINK3_LENGTH - 0.002, 0.0, 0.0)
    return fuse_all(proximal_tab, proximal_nose, beam, end_block, end_plate)


def make_end_pad() -> cq.Workplane:
    stem = centered_box(PAD_STEM_LENGTH, PAD_STEM_WIDTH, PAD_STEM_HEIGHT, PAD_STEM_LENGTH / 2, 0.0, 0.0)
    pad_body = centered_box(
        PAD_THICKNESS,
        PAD_SIZE,
        PAD_SIZE,
        PAD_STEM_LENGTH + PAD_THICKNESS / 2,
        0.0,
        0.0,
    )
    recess = centered_box(
        0.0035,
        PAD_SIZE * 0.72,
        PAD_SIZE * 0.72,
        PAD_STEM_LENGTH + PAD_THICKNESS - 0.00175,
        0.0,
        0.0,
    )
    return stem.union(pad_body).cut(recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_inspection_arm")

    dark_paint = model.material("dark_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    pad_finish = model.material("pad_finish", rgba=(0.76, 0.79, 0.82, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(make_base_plate(), "base_plate"),
        material=dark_paint,
        name="base_plate_shell",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(make_arm_link(LINK1_LENGTH, LINK1_BEAM_WIDTH, LINK1_BEAM_HEIGHT), "shoulder_link"),
        material=aluminum,
        name="shoulder_link_shell",
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        mesh_from_cadquery(make_arm_link(LINK2_LENGTH, LINK2_BEAM_WIDTH, LINK2_BEAM_HEIGHT), "elbow_link"),
        material=aluminum,
        name="elbow_link_shell",
    )

    wrist_link = model.part("wrist_link")
    wrist_link.visual(
        mesh_from_cadquery(make_wrist_link(), "wrist_link"),
        material=aluminum,
        name="wrist_link_shell",
    )

    inspection_pad = model.part("inspection_pad")
    inspection_pad.visual(
        mesh_from_cadquery(make_end_pad(), "inspection_pad"),
        material=pad_finish,
        name="inspection_pad_shell",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=shoulder_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-1.15, upper=1.25),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=wrist_link,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=wrist_link,
        child=inspection_pad,
        origin=Origin(xyz=(LINK3_LENGTH, 0.0, 0.0)),
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

    def require_part(name: str):
        try:
            part = object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - defensive authored lookup
            ctx.fail(f"part_exists_{name}", str(exc))
            return None
        ctx.check(f"part_exists_{name}", True)
        return part

    def require_joint(name: str):
        try:
            joint = object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - defensive authored lookup
            ctx.fail(f"joint_exists_{name}", str(exc))
            return None
        ctx.check(f"joint_exists_{name}", True)
        return joint

    base_plate = require_part("base_plate")
    shoulder_link = require_part("shoulder_link")
    elbow_link = require_part("elbow_link")
    wrist_link = require_part("wrist_link")
    inspection_pad = require_part("inspection_pad")

    shoulder_joint = require_joint("shoulder_joint")
    elbow_joint = require_joint("elbow_joint")
    wrist_joint = require_joint("wrist_joint")

    if any(
        item is None
        for item in (
            base_plate,
            shoulder_link,
            elbow_link,
            wrist_link,
            inspection_pad,
            shoulder_joint,
            elbow_joint,
            wrist_joint,
        )
    ):
        return ctx.report()

    ctx.check(
        "revolute_hinges_are_parallel",
        shoulder_joint.axis == elbow_joint.axis == wrist_joint.axis == (0.0, 1.0, 0.0),
        (
            f"axes were shoulder={shoulder_joint.axis}, elbow={elbow_joint.axis}, "
            f"wrist={wrist_joint.axis}"
        ),
    )

    base_aabb = ctx.part_world_aabb(base_plate)
    ctx.check(
        "base_plate_grounded_to_wall_plane",
        base_aabb is not None and abs(base_aabb[0][0]) <= 1e-4,
        f"base plate min x was {None if base_aabb is None else base_aabb[0][0]}",
    )

    ctx.expect_origin_gap(
        shoulder_link,
        base_plate,
        axis="x",
        min_gap=0.040,
        max_gap=0.050,
        name="shoulder_stands_off_wall_plate",
    )
    ctx.expect_origin_gap(
        elbow_link,
        shoulder_link,
        axis="x",
        min_gap=0.295,
        max_gap=0.305,
        name="first_link_reads_longer",
    )
    ctx.expect_origin_gap(
        wrist_link,
        elbow_link,
        axis="x",
        min_gap=0.215,
        max_gap=0.225,
        name="second_link_reads_mid_length",
    )
    ctx.expect_origin_gap(
        inspection_pad,
        wrist_link,
        axis="x",
        min_gap=0.115,
        max_gap=0.125,
        name="third_link_reads_short",
    )
    ctx.expect_contact(
        shoulder_link,
        base_plate,
        contact_tol=5e-4,
        name="shoulder_joint_mount_contacts_base",
    )
    ctx.expect_contact(
        elbow_link,
        shoulder_link,
        contact_tol=5e-4,
        name="elbow_joint_mount_contacts_shoulder",
    )
    ctx.expect_contact(
        wrist_link,
        elbow_link,
        contact_tol=5e-4,
        name="wrist_joint_mount_contacts_elbow",
    )
    ctx.expect_contact(
        inspection_pad,
        wrist_link,
        contact_tol=5e-4,
        name="pad_is_mounted_to_wrist",
    )
    ctx.expect_overlap(
        inspection_pad,
        wrist_link,
        axes="yz",
        min_overlap=0.012,
        name="pad_mount_is_centered",
    )

    pad_aabb = ctx.part_world_aabb(inspection_pad)
    pad_width = None if pad_aabb is None else pad_aabb[1][1] - pad_aabb[0][1]
    pad_height = None if pad_aabb is None else pad_aabb[1][2] - pad_aabb[0][2]
    ctx.check(
        "end_effector_pad_reads_square",
        pad_width is not None
        and pad_height is not None
        and abs(pad_width - pad_height) <= 0.002
        and 0.050 <= pad_width <= 0.058,
        f"pad footprint was width={pad_width}, height={pad_height}",
    )

    with ctx.pose({shoulder_joint: 0.80, elbow_joint: -0.95, wrist_joint: 0.55}):
        posed_positions = [
            ctx.part_world_position(shoulder_link),
            ctx.part_world_position(elbow_link),
            ctx.part_world_position(wrist_link),
            ctx.part_world_position(inspection_pad),
        ]
        y_offsets = [None if pos is None else pos[1] for pos in posed_positions]
        pad_position = posed_positions[-1]
        ctx.check(
            "hinges_keep_motion_planar",
            all(pos is not None for pos in posed_positions)
            and max(abs(y) for y in y_offsets if y is not None) <= 1e-6
            and pad_position is not None
            and abs(pad_position[2]) >= 0.020,
            f"posed y offsets were {y_offsets}, pad position was {pad_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
