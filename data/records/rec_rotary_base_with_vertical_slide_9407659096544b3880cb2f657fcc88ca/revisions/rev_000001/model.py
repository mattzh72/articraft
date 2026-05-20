from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_SIZE = (0.34, 0.30, 0.08)
BASE_BEARING_RADIUS = 0.068
BASE_BEARING_HEIGHT = 0.014
BASE_TOP_RECESS_DEPTH = 0.010

TURRET_BODY_SIZE = (0.22, 0.22, 0.072)
TURRET_PAD_SIZE = (0.16, 0.12, 0.012)
TURRET_PAD_OFFSET_Y = -0.045

FRAME_BASE_SIZE = (0.14, 0.11, 0.012)
FRAME_BACK_SIZE = (0.054, 0.012, 0.205)
FRAME_CHEEK_SIZE = (0.012, 0.040, 0.205)
FRAME_CHEEK_X = 0.025
FRAME_CHEEK_Y = 0.014

RAM_BODY_SIZE = (0.032, 0.022, 0.22)
RAM_HEAD_SIZE = (0.09, 0.06, 0.012)
RAM_NOSE_RADIUS = 0.018
RAM_NOSE_HEIGHT = 0.010
RAM_Y_OFFSET = 0.014

RAM_TRAVEL = 0.11


def _box(size: tuple[float, float, float], *, offset: tuple[float, float, float] = (0.0, 0.0, 0.0)):
    return cq.Workplane("XY").box(*size, centered=(True, True, False)).translate(offset)


def make_base_body():
    body = cq.Workplane("XY").box(*BASE_SIZE, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.008)

    top_recess = _box((0.24, 0.19, BASE_TOP_RECESS_DEPTH), offset=(0.0, 0.0, BASE_SIZE[2] - BASE_TOP_RECESS_DEPTH))
    side_relief = _box((0.28, 0.05, 0.022), offset=(0.0, 0.125, 0.028))
    side_relief_mirror = _box((0.28, 0.05, 0.022), offset=(0.0, -0.175, 0.028))

    return body.cut(top_recess).cut(side_relief).cut(side_relief_mirror)


def make_base_bearing():
    bearing = cq.Workplane("XY").circle(BASE_BEARING_RADIUS).extrude(BASE_BEARING_HEIGHT)
    bearing = bearing.translate((0.0, 0.0, BASE_SIZE[2] - BASE_TOP_RECESS_DEPTH))
    return bearing.edges(">Z").chamfer(0.0015)


def make_turret_body():
    turret = cq.Workplane("XY").box(*TURRET_BODY_SIZE, centered=(True, True, False))
    turret = turret.edges("|Z").fillet(0.006)

    top_pad = _box(
        TURRET_PAD_SIZE,
        offset=(0.0, TURRET_PAD_OFFSET_Y, TURRET_BODY_SIZE[2]),
    )
    top_pad = top_pad.edges("|Z").fillet(0.003)

    cable_relief = _box((0.08, 0.018, 0.026), offset=(0.0, 0.101, 0.024))

    return turret.union(top_pad).cut(cable_relief)


def make_frame_body():
    base_plate = _box(FRAME_BASE_SIZE)
    back_plate = _box(
        FRAME_BACK_SIZE,
        offset=(0.0, -0.016, FRAME_BASE_SIZE[2]),
    )
    left_cheek = _box(
        FRAME_CHEEK_SIZE,
        offset=(-FRAME_CHEEK_X, FRAME_CHEEK_Y, FRAME_BASE_SIZE[2]),
    )
    right_cheek = _box(
        FRAME_CHEEK_SIZE,
        offset=(FRAME_CHEEK_X, FRAME_CHEEK_Y, FRAME_BASE_SIZE[2]),
    )
    left_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.050, 0.0), (-0.022, 0.0), (-0.022, 0.095)])
        .close()
        .extrude(0.012)
        .translate((0.0, 0.028, 0.0))
    )
    right_gusset = (
        cq.Workplane("XZ")
        .polyline([(0.022, 0.0), (0.050, 0.0), (0.022, 0.095)])
        .close()
        .extrude(0.012)
        .translate((0.0, 0.028, 0.0))
    )

    frame = base_plate.union(back_plate).union(left_cheek).union(right_cheek).union(left_gusset).union(right_gusset)
    return frame.edges("|Z").fillet(0.002)


def make_ram_body():
    ram = cq.Workplane("XY").box(*RAM_BODY_SIZE, centered=(True, True, False))
    return ram.edges("|Z").fillet(0.002)


def make_ram_head():
    head_plate = _box(RAM_HEAD_SIZE, offset=(0.0, 0.0, RAM_BODY_SIZE[2]))
    head_plate = head_plate.edges("|Z").fillet(0.002)
    nose = (
        cq.Workplane("XY")
        .circle(RAM_NOSE_RADIUS)
        .extrude(RAM_NOSE_HEIGHT)
        .translate((0.0, 0.0, RAM_BODY_SIZE[2] + RAM_HEAD_SIZE[2]))
    )
    return head_plate.union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_welding_positioner")

    base_color = model.material("base_cast", rgba=(0.23, 0.25, 0.28, 1.0))
    bearing_color = model.material("bearing_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    turret_color = model.material("turret_orange", rgba=(0.82, 0.36, 0.18, 1.0))
    frame_color = model.material("frame_light", rgba=(0.69, 0.72, 0.76, 1.0))
    ram_color = model.material("ram_steel", rgba=(0.77, 0.80, 0.82, 1.0))
    head_color = model.material("fixture_plate", rgba=(0.49, 0.53, 0.58, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(make_base_body(), "base_body"), material=base_color, name="base_body")
    base.visual(mesh_from_cadquery(make_base_bearing(), "base_bearing"), material=bearing_color, name="base_bearing")

    turret = model.part("turret")
    turret.visual(mesh_from_cadquery(make_turret_body(), "turret_body"), material=turret_color, name="turret_body")

    frame = model.part("upright_frame")
    frame.visual(mesh_from_cadquery(make_frame_body(), "frame_body"), material=frame_color, name="frame_body")

    ram = model.part("z_ram")
    ram.visual(mesh_from_cadquery(make_ram_body(), "ram_body"), material=ram_color, name="ram_body")
    ram.visual(mesh_from_cadquery(make_ram_head(), "ram_head"), material=head_color, name="ram_head")

    base_to_turret = model.articulation(
        "base_to_turret",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] - BASE_TOP_RECESS_DEPTH + BASE_BEARING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "turret_to_frame",
        ArticulationType.FIXED,
        parent=turret,
        child=frame,
        origin=Origin(xyz=(0.0, TURRET_PAD_OFFSET_Y, TURRET_BODY_SIZE[2] + TURRET_PAD_SIZE[2])),
    )
    frame_to_ram = model.articulation(
        "frame_to_z_ram",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ram,
        origin=Origin(xyz=(0.0, RAM_Y_OFFSET, FRAME_BASE_SIZE[2])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.20,
            lower=0.0,
            upper=RAM_TRAVEL,
        ),
    )

    model.meta["primary_joints"] = {
        "turret": base_to_turret.name,
        "ram": frame_to_ram.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    frame = object_model.get_part("upright_frame")
    ram = object_model.get_part("z_ram")
    turret_joint = object_model.get_articulation("base_to_turret")
    ram_joint = object_model.get_articulation("frame_to_z_ram")

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

    ctx.check(
        "parts_present",
        all(part is not None for part in (base, turret, frame, ram)),
        "Expected base, turret, upright_frame, and z_ram parts.",
    )
    ctx.check(
        "joint_types_and_axes",
        turret_joint.articulation_type == ArticulationType.REVOLUTE
        and ram_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in turret_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in ram_joint.axis) == (0.0, 0.0, 1.0),
        "Turret should revolve about +Z and the ram should slide along +Z.",
    )

    ctx.expect_gap(
        turret,
        base,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        name="turret_seats_on_bearing",
    )
    ctx.expect_overlap(
        turret,
        base,
        axes="xy",
        min_overlap=0.13,
        name="turret_centered_over_base",
    )
    ctx.expect_contact(frame, turret, name="frame_mount_contacts_turret")
    ctx.expect_within(ram, frame, axes="xy", margin=0.0, name="ram_stays_within_frame_envelope")

    with ctx.pose(frame_to_z_ram=0.0):
        ctx.expect_within(ram, frame, axes="xy", margin=0.0, name="ram_within_frame_plan_at_rest")
        ctx.expect_gap(
            ram,
            frame,
            axis="z",
            positive_elem="ram_head",
            min_gap=0.002,
            name="ram_head_starts_above_frame",
        )

    with ctx.pose(frame_to_z_ram=RAM_TRAVEL):
        ctx.expect_within(ram, frame, axes="xy", margin=0.0, name="ram_within_frame_plan_extended")
        ctx.expect_gap(
            ram,
            frame,
            axis="z",
            positive_elem="ram_head",
            min_gap=0.10,
            name="ram_head_rises_when_extended",
        )

    with ctx.pose(base_to_turret=0.0):
        frame_pos_0 = ctx.part_world_position(frame)
        ram_pos_0 = ctx.part_world_position(ram)
    with ctx.pose(base_to_turret=1.0):
        frame_pos_1 = ctx.part_world_position(frame)
    with ctx.pose(frame_to_z_ram=RAM_TRAVEL):
        ram_pos_1 = ctx.part_world_position(ram)

    if frame_pos_0 is None or frame_pos_1 is None or ram_pos_0 is None or ram_pos_1 is None:
        ctx.fail("world_pose_queries_available", "Expected frame and ram world positions to be available.")
    else:
        frame_xy_swing = math.hypot(frame_pos_1[0] - frame_pos_0[0], frame_pos_1[1] - frame_pos_0[1])
        frame_z_shift = abs(frame_pos_1[2] - frame_pos_0[2])
        ram_vertical_move = ram_pos_1[2] - ram_pos_0[2]
        ram_xy_drift = math.hypot(ram_pos_1[0] - ram_pos_0[0], ram_pos_1[1] - ram_pos_0[1])

        ctx.check(
            "turret_rotation_swings_upper_structure",
            frame_xy_swing > 0.035 and frame_z_shift < 1e-6,
            f"Expected off-center frame to sweep in XY without lifting; got swing={frame_xy_swing:.4f}, dz={frame_z_shift:.6f}.",
        )
        ctx.check(
            "ram_moves_straight_up",
            ram_vertical_move > 0.10 and ram_xy_drift < 1e-6,
            f"Expected mostly vertical ram extension; got dz={ram_vertical_move:.4f}, xy drift={ram_xy_drift:.6f}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
