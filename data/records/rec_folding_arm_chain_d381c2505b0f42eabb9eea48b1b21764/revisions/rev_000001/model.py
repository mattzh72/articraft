from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


PLATE_T = 0.0030
LUG_T = 0.0070
CLEVIS_GAP = LUG_T
BUSH_T = 0.0010
DOUBLE_CLEVIS_W = 2.0 * PLATE_T + CLEVIS_GAP
CHEEK_Y = CLEVIS_GAP / 2.0 + PLATE_T / 2.0
OUTER_BUSH_Y = CLEVIS_GAP / 2.0 + PLATE_T + BUSH_T / 2.0 - 0.00015

PIVOT_HOLE_R = 0.0042
BUSH_R = 0.0072

FIRST_LEN = 0.086
CENTER_LEN = 0.176
TERMINAL_LEN = 0.082
TONGUE_BACK = 0.024


def _xz_rect(cx: float, cz: float, sx: float, sz: float, thickness: float):
    return cq.Workplane("XZ").center(cx, cz).rect(sx, sz).extrude(thickness / 2.0, both=True)


def _xz_circle(cx: float, cz: float, radius: float, thickness: float):
    return cq.Workplane("XZ").center(cx, cz).circle(radius).extrude(thickness / 2.0, both=True)


def _xy_box(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float):
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _slot_cut(x0: float, x1: float, z: float, radius: float, thickness: float):
    return _xz_circle(x0, z, radius, thickness).union(
        _xz_rect((x0 + x1) / 2.0, z, x1 - x0, 2.0 * radius, thickness)
    ).union(_xz_circle(x1, z, radius, thickness))


def _make_base_lug_shape():
    plate = _xy_box(-0.044, 0.0, -0.029, 0.060, 0.030, 0.006)
    pedestal = _xz_rect(-0.022, -0.014, 0.020, 0.018, LUG_T)
    lug = _xz_circle(0.0, 0.0, 0.012, LUG_T).union(_xz_rect(-0.010, 0.0, 0.020, 0.010, LUG_T))
    rib = _xz_rect(-0.013, -0.0085, 0.018, 0.007, LUG_T)
    left_gusset = _xy_box(-0.028, 0.0065, -0.015, 0.028, 0.003, 0.016)
    right_gusset = _xy_box(-0.028, -0.0065, -0.015, 0.028, 0.003, 0.016)
    stop_tab = _xz_rect(0.005, -0.0105, 0.007, 0.003, LUG_T)

    shape = plate.union(pedestal).union(lug).union(rib).union(left_gusset).union(right_gusset).union(stop_tab)
    shape = shape.cut(_xz_circle(0.0, 0.0, PIVOT_HOLE_R, LUG_T + 0.004))
    shape = shape.cut(_xz_circle(-0.054, -0.029, 0.0033, 0.040))
    shape = shape.cut(_xz_circle(-0.036, -0.029, 0.0033, 0.040))
    return shape


def _make_clevis_to_tongue_link(
    *,
    length: float,
    prox_r: float,
    cheek_len: float,
    cheek_height: float,
    body_start: float,
    body_height: float,
    body_width: float,
    dist_r: float,
    tongue_height: float,
    slot: tuple[float, float, float] | None = None,
):
    cheek_profile = _xz_circle(0.0, 0.0, prox_r, PLATE_T).union(
        _xz_rect(cheek_len / 2.0, 0.0, cheek_len, cheek_height, PLATE_T)
    ).union(_xz_rect(0.012, -0.0065, 0.007, 0.003, PLATE_T))
    right_cheek = cheek_profile.translate((0.0, CHEEK_Y, 0.0))
    left_cheek = cheek_profile.translate((0.0, -CHEEK_Y, 0.0))

    right_bush = _xz_circle(0.0, 0.0, BUSH_R, BUSH_T).translate((0.0, OUTER_BUSH_Y, 0.0))
    left_bush = _xz_circle(0.0, 0.0, BUSH_R, BUSH_T).translate((0.0, -OUTER_BUSH_Y, 0.0))

    rail_end = length - 0.036
    right_rail = _xz_rect((body_start + rail_end) / 2.0, 0.0, rail_end - body_start, body_height, PLATE_T).translate(
        (0.0, CHEEK_Y, 0.0)
    )
    left_rail = _xz_rect((body_start + rail_end) / 2.0, 0.0, rail_end - body_start, body_height, PLATE_T).translate(
        (0.0, -CHEEK_Y, 0.0)
    )
    spacer_a = _xz_rect(body_start + 0.012, 0.0, 0.008, body_height * 0.85, DOUBLE_CLEVIS_W)
    spacer_b = _xz_rect(rail_end - 0.008, 0.0, 0.008, body_height * 0.85, DOUBLE_CLEVIS_W)
    connector = _xz_rect(length - 0.031, 0.0, 0.010, body_height * 0.85, DOUBLE_CLEVIS_W)

    tongue = _xz_circle(length, 0.0, dist_r, LUG_T).union(
        _xz_rect(length - TONGUE_BACK / 2.0, 0.0, TONGUE_BACK, tongue_height, LUG_T)
    )

    shape = right_cheek.union(left_cheek)
    shape = shape.union(right_bush).union(left_bush)
    shape = shape.union(right_rail).union(left_rail).union(spacer_a).union(spacer_b).union(connector).union(tongue)

    shape = shape.cut(_xz_circle(0.0, 0.0, PIVOT_HOLE_R, DOUBLE_CLEVIS_W + 0.004))
    shape = shape.cut(_xz_circle(length, 0.0, PIVOT_HOLE_R, LUG_T + 0.004))

    if slot is not None:
        slot_x0, slot_x1, slot_r = slot
        shape = shape.cut(_slot_cut(slot_x0, slot_x1, 0.0, slot_r, LUG_T + 0.003))

    return shape


def _make_terminal_link_shape():
    prox_r = 0.0115
    cheek_len = 0.025
    cheek_height = 0.011
    rail_start = 0.020
    rail_end = 0.050
    stem_start = 0.058

    cheek_profile = _xz_circle(0.0, 0.0, prox_r, PLATE_T).union(
        _xz_rect(cheek_len / 2.0, 0.0, cheek_len, cheek_height, PLATE_T)
    ).union(_xz_rect(0.011, -0.0062, 0.007, 0.003, PLATE_T))
    shape = cheek_profile.translate((0.0, CHEEK_Y, 0.0)).union(cheek_profile.translate((0.0, -CHEEK_Y, 0.0)))
    shape = shape.union(_xz_circle(0.0, 0.0, BUSH_R, BUSH_T).translate((0.0, OUTER_BUSH_Y, 0.0)))
    shape = shape.union(_xz_circle(0.0, 0.0, BUSH_R, BUSH_T).translate((0.0, -OUTER_BUSH_Y, 0.0)))

    right_rail = _xz_rect((rail_start + rail_end) / 2.0, 0.0, rail_end - rail_start, 0.0085, PLATE_T).translate(
        (0.0, CHEEK_Y, 0.0)
    )
    left_rail = _xz_rect((rail_start + rail_end) / 2.0, 0.0, rail_end - rail_start, 0.0085, PLATE_T).translate(
        (0.0, -CHEEK_Y, 0.0)
    )
    spacer = _xz_rect(0.034, 0.0, 0.008, 0.0075, DOUBLE_CLEVIS_W)
    stem = _xz_rect(0.078, 0.0, 0.040, 0.0085, 0.0085)
    neck = _xz_rect(0.095, 0.0, 0.012, 0.010, 0.010)
    tip_pad = _xz_rect(0.108, 0.0, 0.026, 0.020, 0.016)

    shape = shape.cut(_xz_circle(0.0, 0.0, PIVOT_HOLE_R, DOUBLE_CLEVIS_W + 0.004))
    shape = shape.union(right_rail).union(left_rail).union(spacer).union(stem).union(neck).union(tip_pad)
    shape = shape.cut(_xz_circle(0.108, 0.0065, 0.0028, 0.020))
    shape = shape.cut(_xz_circle(0.108, -0.0065, 0.0028, 0.020))
    return shape


def _first_link_shape():
    return _make_clevis_to_tongue_link(
        length=FIRST_LEN,
        prox_r=0.0125,
        cheek_len=0.026,
        cheek_height=0.011,
        body_start=0.020,
        body_height=0.0075,
        body_width=0.0090,
        dist_r=0.0115,
        tongue_height=0.009,
        slot=(0.040, 0.054, 0.0026),
    )


def _center_link_shape():
    return _make_clevis_to_tongue_link(
        length=CENTER_LEN,
        prox_r=0.0125,
        cheek_len=0.026,
        cheek_height=0.0115,
        body_start=0.020,
        body_height=0.0080,
        body_width=0.0095,
        dist_r=0.0115,
        tongue_height=0.0095,
        slot=(0.065, 0.125, 0.0030),
    )


def _terminal_link_shape():
    return _make_terminal_link_shape()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hatch_support_arm")

    base_finish = model.material("base_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    link_finish = model.material("link_finish", rgba=(0.71, 0.73, 0.76, 1.0))
    bushing_finish = model.material("bushing_finish", rgba=(0.48, 0.50, 0.53, 1.0))

    plate_t = 0.003
    lug_w = 0.008
    cheek_y = 0.0055
    cheek_h = 0.012
    rail_h = 0.008
    pivot_r = 0.006

    def add_plate(part_obj, *, cx, cy, cz, sx, sy, sz, name):
        part_obj.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(cx, cy, cz)),
            material=link_finish,
            name=name,
        )

    def add_bushing(part_obj, *, x, y, z, length, radius, name):
        part_obj.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bushing_finish,
            name=name,
        )

    base = model.part("base_lug")
    base.visual(Box((0.060, 0.030, 0.006)), origin=Origin(xyz=(-0.040, 0.0, -0.027)), material=base_finish, name="base_plate")
    base.visual(Box((0.022, lug_w, 0.018)), origin=Origin(xyz=(-0.018, 0.0, -0.015)), material=base_finish, name="base_riser")
    base.visual(Box((0.014, lug_w, 0.012)), origin=Origin(xyz=(-0.007, 0.0, 0.0)), material=base_finish, name="base_lug_shell")
    base.visual(Box((0.010, 0.003, 0.016)), origin=Origin(xyz=(-0.028, 0.0105, -0.016)), material=base_finish, name="left_mount_ear")
    base.visual(Box((0.010, 0.003, 0.016)), origin=Origin(xyz=(-0.028, -0.0105, -0.016)), material=base_finish, name="right_mount_ear")
    base.visual(Box((0.010, lug_w, 0.003)), origin=Origin(xyz=(-0.001, 0.0, -0.0075)), material=base_finish, name="base_stop")
    add_bushing(base, x=-0.002, y=0.0, z=0.0, length=lug_w, radius=pivot_r, name="base_pivot_boss")
    base.inertial = Inertial.from_geometry(
        Box((0.060, 0.036, 0.030)),
        mass=0.55,
        origin=Origin(xyz=(-0.033, 0.0, -0.016)),
    )

    first = model.part("first_link")
    add_plate(first, cx=0.013, cy=cheek_y, cz=0.0, sx=0.026, sy=plate_t, sz=cheek_h, name="first_right_cheek")
    add_plate(first, cx=0.013, cy=-cheek_y, cz=0.0, sx=0.026, sy=plate_t, sz=cheek_h, name="first_left_cheek")
    add_plate(first, cx=0.024, cy=0.0, cz=0.0, sx=0.004, sy=0.014, sz=0.009, name="first_root_spacer")
    add_plate(first, cx=0.043, cy=cheek_y, cz=0.0, sx=0.034, sy=plate_t, sz=rail_h, name="first_right_rail")
    add_plate(first, cx=0.043, cy=-cheek_y, cz=0.0, sx=0.034, sy=plate_t, sz=rail_h, name="first_left_rail")
    add_plate(first, cx=0.060, cy=0.0, cz=0.0, sx=0.006, sy=0.014, sz=0.008, name="first_transition")
    add_plate(first, cx=0.072, cy=0.0, cz=0.0, sx=0.028, sy=lug_w, sz=0.010, name="first_link_shell")
    add_plate(first, cx=0.024, cy=0.0, cz=-0.007, sx=0.006, sy=0.014, sz=0.0025, name="first_stop_tab")
    add_bushing(first, x=0.006, y=cheek_y, z=0.0, length=plate_t, radius=pivot_r, name="first_right_root_boss")
    add_bushing(first, x=0.006, y=-cheek_y, z=0.0, length=plate_t, radius=pivot_r, name="first_left_root_boss")
    add_bushing(first, x=FIRST_LEN - 0.003, y=0.0, z=0.0, length=lug_w, radius=0.0055, name="first_tip_boss")
    first.inertial = Inertial.from_geometry(
        Box((FIRST_LEN, DOUBLE_CLEVIS_W, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(FIRST_LEN / 2.0, 0.0, 0.0)),
    )

    center = model.part("center_link")
    add_plate(center, cx=0.013, cy=cheek_y, cz=0.0, sx=0.026, sy=plate_t, sz=cheek_h, name="center_right_cheek")
    add_plate(center, cx=0.013, cy=-cheek_y, cz=0.0, sx=0.026, sy=plate_t, sz=cheek_h, name="center_left_cheek")
    add_plate(center, cx=0.024, cy=0.0, cz=0.0, sx=0.004, sy=0.014, sz=0.009, name="center_root_spacer")
    add_plate(center, cx=0.086, cy=cheek_y, cz=0.0, sx=0.120, sy=plate_t, sz=rail_h, name="center_right_rail")
    add_plate(center, cx=0.086, cy=-cheek_y, cz=0.0, sx=0.120, sy=plate_t, sz=rail_h, name="center_left_rail")
    add_plate(center, cx=0.046, cy=0.0, cz=0.0, sx=0.006, sy=0.014, sz=0.008, name="center_spacer_a")
    add_plate(center, cx=0.122, cy=0.0, cz=0.0, sx=0.006, sy=0.014, sz=0.008, name="center_spacer_b")
    add_plate(center, cx=0.150, cy=0.0, cz=0.0, sx=0.010, sy=0.014, sz=0.008, name="center_transition")
    add_plate(center, cx=0.165, cy=0.0, cz=0.0, sx=0.022, sy=lug_w, sz=0.010, name="center_link_shell")
    add_plate(center, cx=0.026, cy=0.0, cz=-0.0055, sx=0.006, sy=0.014, sz=0.0030, name="center_stop_tab")
    add_bushing(center, x=0.006, y=cheek_y, z=0.0, length=plate_t, radius=pivot_r, name="center_right_root_boss")
    add_bushing(center, x=0.006, y=-cheek_y, z=0.0, length=plate_t, radius=pivot_r, name="center_left_root_boss")
    add_bushing(center, x=CENTER_LEN - 0.003, y=0.0, z=0.0, length=lug_w, radius=0.0055, name="center_tip_boss")
    center.inertial = Inertial.from_geometry(
        Box((CENTER_LEN, DOUBLE_CLEVIS_W, 0.019)),
        mass=0.29,
        origin=Origin(xyz=(CENTER_LEN / 2.0, 0.0, 0.0)),
    )

    terminal = model.part("terminal_link")
    add_plate(terminal, cx=0.012, cy=cheek_y, cz=0.0, sx=0.024, sy=plate_t, sz=0.011, name="terminal_right_cheek")
    add_plate(terminal, cx=0.012, cy=-cheek_y, cz=0.0, sx=0.024, sy=plate_t, sz=0.011, name="terminal_left_cheek")
    add_plate(terminal, cx=0.022, cy=0.0, cz=0.0, sx=0.004, sy=0.014, sz=0.0085, name="terminal_root_spacer")
    add_plate(terminal, cx=0.052, cy=0.0, cz=0.0, sx=0.056, sy=0.008, sz=0.008, name="terminal_stem")
    add_plate(terminal, cx=0.086, cy=0.0, cz=0.0, sx=0.012, sy=0.010, sz=0.010, name="terminal_neck")
    add_plate(terminal, cx=0.100, cy=0.0, cz=0.0, sx=0.024, sy=0.016, sz=0.020, name="terminal_link_shell")
    add_bushing(terminal, x=0.006, y=cheek_y, z=0.0, length=plate_t, radius=pivot_r, name="terminal_right_root_boss")
    add_bushing(terminal, x=0.006, y=-cheek_y, z=0.0, length=plate_t, radius=pivot_r, name="terminal_left_root_boss")
    terminal.inertial = Inertial.from_geometry(
        Box((0.112, 0.020, 0.022)),
        mass=0.16,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_first",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.52, upper=1.10, effort=35.0, velocity=1.4),
    )
    model.articulation(
        "first_to_center",
        ArticulationType.REVOLUTE,
        parent=first,
        child=center,
        origin=Origin(xyz=(FIRST_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.05, upper=1.15, effort=25.0, velocity=1.6),
    )
    model.articulation(
        "center_to_terminal",
        ArticulationType.REVOLUTE,
        parent=center,
        child=terminal,
        origin=Origin(xyz=(CENTER_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=1.05, effort=20.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_lug")
    first = object_model.get_part("first_link")
    center = object_model.get_part("center_link")
    terminal = object_model.get_part("terminal_link")
    j1 = object_model.get_articulation("base_to_first")
    j2 = object_model.get_articulation("first_to_center")
    j3 = object_model.get_articulation("center_to_terminal")

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
        "three_parallel_revolutes",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (j1, j2, j3))
        and all(j.axis == (0.0, 1.0, 0.0) for j in (j1, j2, j3)),
        details=f"joint types/axes: {[(j.name, j.articulation_type, j.axis) for j in (j1, j2, j3)]}",
    )

    ctx.expect_contact(base, first, name="base_joint_contact")
    ctx.expect_contact(first, center, name="center_joint_contact")
    ctx.expect_contact(center, terminal, name="terminal_joint_contact")
    ctx.expect_origin_gap(terminal, base, axis="x", min_gap=0.25, max_gap=0.27, name="chain_reach")

    with ctx.pose({j1: 0.82, j2: -0.42, j3: -0.34}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_clearance")

    with ctx.pose({j1: -0.36, j2: 0.92, j3: 0.74}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
