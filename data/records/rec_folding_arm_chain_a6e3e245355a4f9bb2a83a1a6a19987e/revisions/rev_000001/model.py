from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PIN_RADIUS = 0.0055
HOLE_RADIUS = 0.0057
PIN_CONTACT_RADIUS = 0.0060
SIDE_PLATE_THICKNESS = 0.006
CLEVIS_GAP = 0.012
SINGLE_LINK_THICKNESS = 0.006
CLEVIS_TOTAL_WIDTH = CLEVIS_GAP + 2.0 * SIDE_PLATE_THICKNESS

FIRST_LINK_LENGTH = 0.095
CENTER_LINK_LENGTH = 0.275
TERMINAL_LINK_LENGTH = 0.115


def _box(length: float, width: float, height: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _y_cylinder(
    x: float,
    y_center: float,
    z: float,
    radius: float,
    length: float,
) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x, y_center - length / 2.0, z),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )
def _two_hole_plate(
    *,
    length: float,
    width: float,
    thickness: float,
    hole_radius: float,
    boss_radius: float,
    window: tuple[float, float] | None = None,
) -> cq.Workplane:
    plate = _box(length, thickness, width, center=(length / 2.0, 0.0, 0.0))
    for x_pos in (0.0, length):
        plate = plate.union(_y_cylinder(x_pos, 0.0, 0.0, boss_radius, thickness))
        plate = plate.cut(_y_cylinder(x_pos, 0.0, 0.0, hole_radius, thickness * 1.4))
    if window is not None:
        window_length, window_height = window
        plate = plate.cut(
            _box(window_length, thickness * 1.6, window_height, center=(length / 2.0, 0.0, 0.0))
        )
    return plate


def _terminal_plate(
    *,
    length: float,
    width: float,
    thickness: float,
    hole_radius: float,
    root_boss_radius: float,
) -> cq.Workplane:
    stem_length = max(length - 0.014, 0.020)
    plate = _box(stem_length, thickness, width, center=(stem_length / 2.0, 0.0, 0.0))
    plate = plate.union(_y_cylinder(0.0, 0.0, 0.0, root_boss_radius, thickness))
    plate = plate.union(
        _box(0.014, thickness, width * 0.88, center=(length - 0.007, 0.0, 0.0))
    )
    plate = plate.cut(_y_cylinder(0.0, 0.0, 0.0, hole_radius, thickness * 1.4))
    return plate


def make_first_link() -> cq.Workplane:
    return _two_hole_plate(
        length=FIRST_LINK_LENGTH,
        width=0.024,
        thickness=SINGLE_LINK_THICKNESS,
        hole_radius=HOLE_RADIUS,
        boss_radius=0.011,
        window=(0.040, 0.010),
    )


def make_center_link_body() -> cq.Workplane:
    plate_offset = CLEVIS_GAP / 2.0 + SIDE_PLATE_THICKNESS / 2.0
    side_plate = _box(CENTER_LINK_LENGTH, SIDE_PLATE_THICKNESS, 0.026, center=(CENTER_LINK_LENGTH / 2.0, 0.0, 0.0))
    side_plate = side_plate.cut(_box(0.150, SIDE_PLATE_THICKNESS * 1.4, 0.011, center=(CENTER_LINK_LENGTH / 2.0, 0.0, 0.0)))

    link = None
    for y_pos in (plate_offset, -plate_offset):
        plate = side_plate.translate((0.0, y_pos, 0.0))
        for x_pos in (0.0, CENTER_LINK_LENGTH):
            plate = plate.union(_y_cylinder(x_pos, y_pos, 0.0, 0.0135, SIDE_PLATE_THICKNESS))
            plate = plate.cut(_y_cylinder(x_pos, y_pos, 0.0, HOLE_RADIUS, SIDE_PLATE_THICKNESS * 1.4))
        link = plate if link is None else link.union(plate)

    mid_spacer = _box(0.018, CLEVIS_TOTAL_WIDTH, 0.012, center=(CENTER_LINK_LENGTH * 0.56, 0.0, 0.0))
    rear_spacer = _box(0.016, CLEVIS_TOTAL_WIDTH, 0.010, center=(CENTER_LINK_LENGTH * 0.42, 0.0, 0.0))
    return link.union(mid_spacer).union(rear_spacer)


def make_terminal_link() -> cq.Workplane:
    return _terminal_plate(
        length=TERMINAL_LINK_LENGTH,
        width=0.016,
        thickness=SINGLE_LINK_THICKNESS,
        hole_radius=HOLE_RADIUS,
        root_boss_radius=0.010,
    )


def make_tip_pad() -> cq.Workplane:
    stem = _box(0.016, SINGLE_LINK_THICKNESS, 0.012, center=(0.008, 0.0, 0.0))
    pad = _box(0.040, SINGLE_LINK_THICKNESS, 0.030, center=(0.036, 0.0, 0.0))
    pad_shape = stem.union(pad)
    for z_pos in (-0.009, 0.009):
        pad_shape = pad_shape.cut(_y_cylinder(0.036, 0.0, z_pos, 0.003, SINGLE_LINK_THICKNESS * 1.8))
    return pad_shape


def make_base_lug_body() -> cq.Workplane:
    back_length = 0.043
    front_reach = 0.012
    plate_height = 0.028
    mount_x = -0.028
    plate_length = back_length + front_reach
    plate_center_x = (front_reach - back_length) / 2.0

    plate_offset = CLEVIS_GAP / 2.0 + SIDE_PLATE_THICKNESS / 2.0
    side_plate = _box(plate_length, SIDE_PLATE_THICKNESS, plate_height, center=(plate_center_x, 0.0, 0.0))
    side_plate = side_plate.union(_y_cylinder(0.0, 0.0, 0.0, 0.013, SIDE_PLATE_THICKNESS))
    side_plate = side_plate.union(_y_cylinder(mount_x, 0.0, 0.0, 0.0115, SIDE_PLATE_THICKNESS))
    side_plate = side_plate.cut(_y_cylinder(0.0, 0.0, 0.0, HOLE_RADIUS, SIDE_PLATE_THICKNESS * 1.4))

    lug = None
    for y_pos in (plate_offset, -plate_offset):
        plate = side_plate.translate((0.0, y_pos, 0.0))
        lug = plate if lug is None else lug.union(plate)

    bridge = _box(0.022, CLEVIS_TOTAL_WIDTH, 0.022, center=(mount_x - 0.002, 0.0, 0.0))
    gusset = _box(0.014, CLEVIS_TOTAL_WIDTH, 0.016, center=(mount_x + 0.008, 0.0, 0.0))
    mount_hole = _y_cylinder(mount_x, 0.0, 0.0, 0.0045, CLEVIS_TOTAL_WIDTH * 1.2)

    return lug.union(bridge).union(gusset).cut(mount_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hatch_support_arm")

    black_oxide = model.material("black_oxide", color=(0.18, 0.19, 0.21))
    zinc_plate = model.material("zinc_plate", color=(0.70, 0.72, 0.74))
    dark_steel = model.material("dark_steel", color=(0.32, 0.34, 0.37))
    pin_steel = model.material("pin_steel", color=(0.55, 0.57, 0.60))

    base_lug = model.part("base_lug")
    base_lug.visual(mesh_from_cadquery(make_base_lug_body(), "base_lug_body"), material=black_oxide, name="base_lug_body")

    base_pin = model.part("base_pin")
    base_pin.visual(
        Cylinder(radius=PIN_CONTACT_RADIUS, length=CLEVIS_TOTAL_WIDTH + 0.004),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="base_pin_body",
    )

    first_link = model.part("first_link")
    first_link.visual(mesh_from_cadquery(make_first_link(), "first_link"), material=zinc_plate, name="first_link_body")

    center_link = model.part("center_link")
    center_link.visual(
        mesh_from_cadquery(make_center_link_body(), "center_link_body"),
        material=dark_steel,
        name="center_link_body",
    )

    center_root_pin = model.part("center_root_pin")
    center_root_pin.visual(
        Cylinder(radius=PIN_CONTACT_RADIUS, length=CLEVIS_TOTAL_WIDTH + 0.004),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="center_root_pin_body",
    )

    center_tip_pin = model.part("center_tip_pin")
    center_tip_pin.visual(
        Cylinder(radius=PIN_CONTACT_RADIUS, length=CLEVIS_TOTAL_WIDTH + 0.004),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="center_tip_pin_body",
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        mesh_from_cadquery(make_terminal_link(), "terminal_link"),
        material=zinc_plate,
        name="terminal_link_body",
    )

    tip_pad = model.part("tip_pad")
    tip_pad.visual(mesh_from_cadquery(make_tip_pad(), "tip_pad"), material=black_oxide, name="tip_pad_body")

    model.articulation(
        "base_lug_to_pin",
        ArticulationType.FIXED,
        parent=base_lug,
        child=base_pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_first",
        ArticulationType.REVOLUTE,
        parent=base_lug,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-1.35, upper=1.15),
    )
    model.articulation(
        "first_to_center",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=center_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "center_to_root_pin",
        ArticulationType.FIXED,
        parent=center_link,
        child=center_root_pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "center_to_tip_pin",
        ArticulationType.FIXED,
        parent=center_link,
        child=center_tip_pin,
        origin=Origin(xyz=(CENTER_LINK_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "center_to_terminal",
        ArticulationType.REVOLUTE,
        parent=center_link,
        child=terminal_link,
        origin=Origin(xyz=(CENTER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-2.00, upper=2.00),
    )
    model.articulation(
        "terminal_to_pad",
        ArticulationType.FIXED,
        parent=terminal_link,
        child=tip_pad,
        origin=Origin(xyz=(TERMINAL_LINK_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    expected_parts = ("base_lug", "first_link", "center_link", "terminal_link", "tip_pad")
    authored_parts = {part.name for part in object_model.parts}
    for part_name in expected_parts:
        ctx.check(f"has_part_{part_name}", part_name in authored_parts, f"missing part: {part_name}")

    base_lug = object_model.get_part("base_lug")
    base_pin = object_model.get_part("base_pin")
    first_link = object_model.get_part("first_link")
    center_link = object_model.get_part("center_link")
    center_root_pin = object_model.get_part("center_root_pin")
    center_tip_pin = object_model.get_part("center_tip_pin")
    terminal_link = object_model.get_part("terminal_link")
    tip_pad = object_model.get_part("tip_pad")

    ctx.allow_overlap(base_lug, base_pin, reason="Base pivot pin is swaged into the base lug ears.")
    ctx.allow_overlap(base_pin, first_link, reason="Base hinge is modeled as a tight captured pin fit.")
    ctx.allow_overlap(center_link, center_root_pin, reason="Inner pivot pin is retained in the center clevis.")
    ctx.allow_overlap(center_root_pin, first_link, reason="Inner pivot uses a tight captured pin fit.")
    ctx.allow_overlap(center_link, center_tip_pin, reason="Outer pivot pin is retained in the center clevis.")
    ctx.allow_overlap(center_tip_pin, terminal_link, reason="Outer pivot uses a tight captured pin fit.")

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

    base_to_first = object_model.get_articulation("base_to_first")
    first_to_center = object_model.get_articulation("first_to_center")
    center_to_terminal = object_model.get_articulation("center_to_terminal")
    terminal_to_pad = object_model.get_articulation("terminal_to_pad")

    for joint_name, joint_obj in (
        ("base_to_first", base_to_first),
        ("first_to_center", first_to_center),
        ("center_to_terminal", center_to_terminal),
    ):
        ctx.check(
            f"{joint_name}_is_revolute",
            joint_obj.articulation_type == ArticulationType.REVOLUTE,
            f"{joint_name} should be revolute",
        )
        ctx.check(
            f"{joint_name}_axis_parallel",
            tuple(round(v, 6) for v in joint_obj.axis) == (0.0, 1.0, 0.0),
            f"{joint_name} axis was {joint_obj.axis}, expected world Y",
        )

    ctx.check(
        "terminal_to_pad_fixed",
        terminal_to_pad.articulation_type == ArticulationType.FIXED,
        "tip pad should be rigidly mounted to the terminal link",
    )

    ctx.expect_contact(terminal_link, tip_pad, name="terminal_link_contacts_tip_pad")
    ctx.expect_overlap(base_lug, first_link, axes="xz", min_overlap=0.012, name="base_joint_aligned_in_plane")
    ctx.expect_overlap(first_link, center_link, axes="xz", min_overlap=0.012, name="middle_joint_aligned_in_plane")
    ctx.expect_overlap(center_link, terminal_link, axes="xz", min_overlap=0.010, name="terminal_joint_aligned_in_plane")

    def part_center(part_obj):
        aabb = ctx.part_world_aabb(part_obj)
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, (min_z + max_z) / 2.0)

    base_center = part_center(base_lug)
    first_center = part_center(first_link)
    center_center = part_center(center_link)
    terminal_center = part_center(terminal_link)
    pad_center = part_center(tip_pad)
    centers_ok = all(c is not None for c in (base_center, first_center, center_center, terminal_center, pad_center))
    if centers_ok:
        ordered = (
            base_center[0] < first_center[0] < center_center[0] < terminal_center[0] < pad_center[0]
            and abs(base_center[1] - pad_center[1]) < 1e-6
        )
        ctx.check(
            "linkage_progresses_outboard",
            ordered,
            (
                f"expected increasing X centers from base to pad; got "
                f"{base_center}, {first_center}, {center_center}, {terminal_center}, {pad_center}"
            ),
        )
    else:
        ctx.fail("linkage_progresses_outboard", "could not resolve part centers")

    def check_joint_motion(name: str, pose_kwargs: dict[str, float], min_vertical_travel: float) -> None:
        rest = ctx.part_world_position(tip_pad)
        with ctx.pose(pose_kwargs):
            moved = ctx.part_world_position(tip_pad)
        ok = (
            rest is not None
            and moved is not None
            and abs(moved[1] - rest[1]) < 1e-6
            and abs(moved[2] - rest[2]) >= min_vertical_travel
        )
        ctx.check(
            name,
            ok,
            f"tip pad should move in XZ plane only; rest={rest}, moved={moved}",
        )

    check_joint_motion("base_joint_moves_in_plane", {"base_to_first": 0.55}, 0.05)
    check_joint_motion("middle_joint_moves_in_plane", {"first_to_center": -0.75}, 0.03)
    check_joint_motion("terminal_joint_moves_in_plane", {"center_to_terminal": 0.80}, 0.015)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
