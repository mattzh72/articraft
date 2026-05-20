from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_over_range_microwave")

    steel = Material("brushed_stainless", rgba=(0.66, 0.68, 0.66, 1.0))
    dark_steel = Material("dark_stainless_shadow", rgba=(0.18, 0.19, 0.20, 1.0))
    black_glass = Material("smoked_black_glass", rgba=(0.02, 0.025, 0.03, 0.72))
    charcoal = Material("recessed_charcoal", rgba=(0.045, 0.047, 0.052, 1.0))
    button_mat = Material("satin_button_gray", rgba=(0.28, 0.29, 0.30, 1.0))
    warm_cavity = Material("warm_cavity_enamel", rgba=(0.78, 0.77, 0.70, 1.0))
    clear_glass = Material("pale_turntable_glass", rgba=(0.70, 0.88, 0.95, 0.43))

    cabinet = model.part("cabinet")

    # Overall envelope: 30 in wide, deep over-the-range box with an open front
    # cooking cavity behind the left-hinged door.
    cabinet.visual(Box((0.78, 0.41, 0.070)), origin=Origin(xyz=(0.0, 0.205, 0.035)), material=steel, name="bottom_floor")
    cabinet.visual(Box((0.78, 0.41, 0.045)), origin=Origin(xyz=(0.0, 0.205, 0.3975)), material=steel, name="top_shell")
    cabinet.visual(Box((0.045, 0.41, 0.420)), origin=Origin(xyz=(-0.3675, 0.205, 0.210)), material=steel, name="left_side_wall")
    cabinet.visual(Box((0.160, 0.41, 0.420)), origin=Origin(xyz=(0.300, 0.205, 0.210)), material=steel, name="right_control_column")
    cabinet.visual(Box((0.78, 0.035, 0.420)), origin=Origin(xyz=(0.0, 0.3925, 0.210)), material=steel, name="rear_wall")

    # Flat top mounting plate and side cabinet-mounting flanges with screw heads.
    cabinet.visual(Box((0.82, 0.44, 0.018)), origin=Origin(xyz=(0.0, 0.205, 0.429)), material=steel, name="flat_top_plate")
    cabinet.visual(Box((0.014, 0.30, 0.34)), origin=Origin(xyz=(-0.388, 0.205, 0.235)), material=steel, name="side_flange_0")
    cabinet.visual(Box((0.014, 0.30, 0.34)), origin=Origin(xyz=(0.388, 0.205, 0.235)), material=steel, name="side_flange_1")
    for side, x in enumerate((-0.397, 0.397)):
        for z in (0.125, 0.345):
            cabinet.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x, 0.090, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"flange_screw_{side}_{int(z * 1000)}",
            )

    # Enamel interior and the low front ventilation grille strip.
    cabinet.visual(Box((0.555, 0.285, 0.006)), origin=Origin(xyz=(-0.060, 0.215, 0.073)), material=warm_cavity, name="cavity_floor_liner")
    cabinet.visual(Box((0.555, 0.012, 0.270)), origin=Origin(xyz=(-0.060, 0.371, 0.225)), material=warm_cavity, name="cavity_back_liner")
    cabinet.visual(Box((0.725, 0.018, 0.052)), origin=Origin(xyz=(0.0, -0.009, 0.040)), material=dark_steel, name="bottom_grille_backing")
    for i in range(14):
        x = -0.325 + i * 0.050
        cabinet.visual(
            Box((0.031, 0.008, 0.036)),
            origin=Origin(xyz=(x, -0.021, 0.040)),
            material=charcoal,
            name=f"vent_slot_{i}",
        )
    cabinet.visual(Box((0.725, 0.010, 0.006)), origin=Origin(xyz=(0.0, -0.022, 0.069)), material=steel, name="grille_upper_lip")

    # Right-side recessed control-panel strip in the stainless front face.
    cabinet.visual(Box((0.142, 0.018, 0.335)), origin=Origin(xyz=(0.309, -0.009, 0.238)), material=steel, name="control_surround")
    cabinet.visual(Box((0.108, 0.004, 0.260)), origin=Origin(xyz=(0.309, -0.015, 0.242)), material=charcoal, name="control_recess")
    cabinet.visual(Box((0.086, 0.003, 0.052)), origin=Origin(xyz=(0.309, -0.019, 0.342)), material=black_glass, name="display_lens")
    for i in range(7):
        z = 0.126 + i * 0.034
        cabinet.visual(Box((0.087, 0.002, 0.003)), origin=Origin(xyz=(0.309, -0.017, z)), material=dark_steel, name=f"panel_rule_{i}")

    # Body-side hinge leaves are flush with the left front edge.
    for suffix, z in (("lower", 0.135), ("upper", 0.355)):
        cabinet.visual(
            Box((0.020, 0.012, 0.082)),
            origin=Origin(xyz=(-0.400, 0.006, z)),
            material=steel,
            name=f"body_hinge_leaf_{suffix}",
        )

    # Body-mounted central turntable hub on the cavity floor.
    cabinet.visual(Cylinder(radius=0.030, length=0.020), origin=Origin(xyz=(-0.080, 0.215, 0.083)), material=dark_steel, name="turntable_hub")

    door = model.part("door")
    door_width = 0.620
    door_height = 0.340
    door_depth = 0.045
    rail = 0.055
    # Stainless door frame: separate rails with overlapping mitred corners, so the
    # large smoked window is visibly recessed instead of painted onto a slab.
    door.visual(Box((rail, door_depth, door_height)), origin=Origin(xyz=(rail / 2.0, -door_depth / 2.0, 0.0)), material=steel, name="left_stile")
    door.visual(Box((0.064, door_depth, door_height)), origin=Origin(xyz=(door_width - 0.032, -door_depth / 2.0, 0.0)), material=steel, name="right_stile")
    door.visual(Box((door_width, door_depth, rail)), origin=Origin(xyz=(door_width / 2.0, -door_depth / 2.0, door_height / 2.0 - rail / 2.0)), material=steel, name="top_rail")
    door.visual(Box((door_width, door_depth, rail)), origin=Origin(xyz=(door_width / 2.0, -door_depth / 2.0, -door_height / 2.0 + rail / 2.0)), material=steel, name="bottom_rail")
    door.visual(Box((0.480, 0.008, 0.238)), origin=Origin(xyz=(0.292, -0.036, 0.004)), material=black_glass, name="window_glass")
    # Raised bead border around the outside and the viewing window.
    door.visual(Box((door_width - 0.030, 0.010, 0.010)), origin=Origin(xyz=(door_width / 2.0, -0.051, door_height / 2.0 - 0.012)), material=steel, name="outer_bead_top")
    door.visual(Box((door_width - 0.030, 0.010, 0.010)), origin=Origin(xyz=(door_width / 2.0, -0.051, -door_height / 2.0 + 0.012)), material=steel, name="outer_bead_bottom")
    door.visual(Box((0.010, 0.010, door_height - 0.030)), origin=Origin(xyz=(0.016, -0.051, 0.0)), material=steel, name="outer_bead_left")
    door.visual(Box((0.010, 0.010, door_height - 0.030)), origin=Origin(xyz=(door_width - 0.016, -0.051, 0.0)), material=steel, name="outer_bead_right")
    door.visual(Box((0.492, 0.008, 0.009)), origin=Origin(xyz=(0.292, -0.047, 0.133)), material=dark_steel, name="window_bead_top")
    door.visual(Box((0.492, 0.008, 0.009)), origin=Origin(xyz=(0.292, -0.047, -0.125)), material=dark_steel, name="window_bead_bottom")
    door.visual(Box((0.009, 0.008, 0.258)), origin=Origin(xyz=(0.046, -0.047, 0.004)), material=dark_steel, name="window_bead_left")
    door.visual(Box((0.009, 0.008, 0.258)), origin=Origin(xyz=(0.538, -0.047, 0.004)), material=dark_steel, name="window_bead_right")

    # Horizontal pull handle mounted across the right edge of the door.
    door.visual(Cylinder(radius=0.012, length=0.160), origin=Origin(xyz=(0.530, -0.083, -0.010), rpy=(0.0, pi / 2.0, 0.0)), material=steel, name="pull_handle")
    for x in (0.460, 0.600):
        door.visual(
            Cylinder(radius=0.010, length=0.040),
            origin=Origin(xyz=(x, -0.064, -0.010), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"handle_standoff_{int(x * 1000)}",
        )

    # Visible hinge barrels and moving leaves, aligned with the full-height left edge.
    for suffix, z in (("lower", -0.110), ("upper", 0.110)):
        door.visual(Cylinder(radius=0.014, length=0.090), origin=Origin(xyz=(0.000, -0.022, z)), material=steel, name=f"hinge_barrel_{suffix}")
        door.visual(Box((0.026, 0.014, 0.075)), origin=Origin(xyz=(0.014, -0.007, z)), material=steel, name=f"door_hinge_leaf_{suffix}")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.390, 0.000, 0.245)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.145, length=0.009), origin=Origin(xyz=(0.0, 0.0, 0.0045)), material=clear_glass, name="glass_disk")
    turntable.visual(Box((0.115, 0.014, 0.004)), origin=Origin(xyz=(0.052, 0.0, 0.0105)), material=black_glass, name="orientation_mark")
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=turntable,
        origin=Origin(xyz=(-0.080, 0.215, 0.093)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    # A small set of separate, pressable controls on the recessed strip.
    button_specs = [
        ("key_0_0", 0.286, 0.285, 0.030, 0.026),
        ("key_0_1", 0.332, 0.285, 0.030, 0.026),
        ("key_1_0", 0.286, 0.245, 0.030, 0.026),
        ("key_1_1", 0.332, 0.245, 0.030, 0.026),
        ("key_2_0", 0.286, 0.205, 0.030, 0.026),
        ("key_2_1", 0.332, 0.205, 0.030, 0.026),
        ("start_button", 0.309, 0.154, 0.070, 0.030),
    ]
    for name, x, z, sx, sz in button_specs:
        button = model.part(name)
        button.visual(Box((sx, 0.008, sz)), origin=Origin(), material=button_mat, name="button_cap")
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, -0.021, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.20, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("door_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")
    start_press = object_model.get_articulation("start_button_press")

    def coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return tuple((coord(lo, i) + coord(hi, i)) / 2.0 for i in range(3))

    # Closed door seats on the front plane while its window is recessed behind
    # the stainless frame, as specified for the residential front panel.
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="left_side_wall",
        negative_elem="left_stile",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed door is flush with cabinet front",
    )
    frame_aabb = ctx.part_element_world_aabb(door, elem="top_rail")
    window_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
    if frame_aabb is None or window_aabb is None:
        ctx.fail("window recess measurable", "missing door frame or window element AABB")
    else:
        frame_front_y = coord(frame_aabb[0], 1)
        window_front_y = coord(window_aabb[0], 1)
        ctx.check(
            "viewing window is recessed",
            window_front_y > frame_front_y + 0.004,
            details=f"frame_front_y={frame_front_y:.4f}, window_front_y={window_front_y:.4f}",
        )

    closed_handle = elem_center(door, "pull_handle")
    with ctx.pose({door_hinge: 1.25}):
        open_handle = elem_center(door, "pull_handle")
    ctx.check(
        "door swings outward on left hinge",
        closed_handle is not None and open_handle is not None and open_handle[1] < closed_handle[1] - 0.30,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    # The glass turntable sits on the central hub, stays inside the cavity floor
    # outline, and the orientation mark proves the continuous spin joint moves.
    ctx.expect_contact(
        turntable,
        cabinet,
        elem_a="glass_disk",
        elem_b="turntable_hub",
        contact_tol=0.002,
        name="turntable rests on central hub",
    )
    ctx.expect_within(
        turntable,
        cabinet,
        axes="xy",
        inner_elem="glass_disk",
        outer_elem="cavity_floor_liner",
        margin=0.006,
        name="turntable fits within cavity floor",
    )
    mark_closed = elem_center(turntable, "orientation_mark")
    with ctx.pose({turntable_spin: pi / 2.0}):
        mark_rotated = elem_center(turntable, "orientation_mark")
    ctx.check(
        "turntable marker rotates about hub",
        mark_closed is not None
        and mark_rotated is not None
        and abs(mark_rotated[0] - mark_closed[0]) > 0.035
        and abs(mark_rotated[1] - mark_closed[1]) > 0.035,
        details=f"closed_mark={mark_closed}, rotated_mark={mark_rotated}",
    )

    start_rest = elem_center(object_model.get_part("start_button"), "button_cap")
    with ctx.pose({start_press: 0.006}):
        start_pressed = elem_center(object_model.get_part("start_button"), "button_cap")
    ctx.check(
        "start button presses inward",
        start_rest is not None and start_pressed is not None and start_pressed[1] > start_rest[1] + 0.005,
        details=f"rest={start_rest}, pressed={start_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
