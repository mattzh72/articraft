from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_office_microwave")

    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    enamel_gray = model.material("enamel_gray", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_cavity = model.material("dark_cavity", rgba=(0.07, 0.075, 0.075, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    smoky_glass = model.material("smoky_glass", rgba=(0.05, 0.08, 0.10, 0.42))
    pale_glass = model.material("pale_blue_glass", rgba=(0.55, 0.72, 0.82, 0.45))
    chrome = model.material("soft_chrome", rgba=(0.70, 0.70, 0.66, 1.0))

    cabinet = model.part("cabinet")

    # Main metal carcass: real office-countertop proportions, hollow rather than
    # a single solid block.  The front opening is left clear for the door window.
    cabinet.visual(Box((0.460, 0.350, 0.024)), origin=Origin(xyz=(0.0, 0.0, 0.268)), material=warm_white, name="top_skin")
    cabinet.visual(Box((0.460, 0.350, 0.024)), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=warm_white, name="bottom_skin")
    cabinet.visual(Box((0.024, 0.350, 0.280)), origin=Origin(xyz=(-0.218, 0.0, 0.140)), material=warm_white, name="side_skin_0")
    cabinet.visual(Box((0.024, 0.350, 0.280)), origin=Origin(xyz=(0.218, 0.0, 0.140)), material=warm_white, name="side_skin_1")
    cabinet.visual(Box((0.460, 0.020, 0.280)), origin=Origin(xyz=(0.0, 0.165, 0.140)), material=warm_white, name="rear_skin")

    # Door aperture rails and the fixed control-panel fascia.  The lower control
    # strip is split so the key has a real guide slot instead of sliding through a
    # solid block.
    cabinet.visual(Box((0.350, 0.025, 0.024)), origin=Origin(xyz=(-0.056, -0.1825, 0.268)), material=warm_white, name="front_top_rail")
    cabinet.visual(Box((0.350, 0.025, 0.024)), origin=Origin(xyz=(-0.056, -0.1825, 0.012)), material=warm_white, name="front_bottom_rail")
    cabinet.visual(Box((0.024, 0.025, 0.256)), origin=Origin(xyz=(-0.218, -0.1825, 0.140)), material=warm_white, name="front_jamb")
    cabinet.visual(Box((0.024, 0.025, 0.256)), origin=Origin(xyz=(0.105, -0.1825, 0.140)), material=warm_white, name="control_mullion")

    cabinet.visual(Box((0.125, 0.025, 0.202)), origin=Origin(xyz=(0.1675, -0.1825, 0.179)), material=warm_white, name="control_panel")
    cabinet.visual(Box((0.125, 0.025, 0.042)), origin=Origin(xyz=(0.1675, -0.1825, 0.021)), material=warm_white, name="lower_panel")
    cabinet.visual(Box((0.020, 0.025, 0.038)), origin=Origin(xyz=(0.115, -0.1825, 0.061)), material=warm_white, name="key_slot_side_0")
    cabinet.visual(Box((0.020, 0.025, 0.038)), origin=Origin(xyz=(0.220, -0.1825, 0.061)), material=warm_white, name="key_slot_side_1")
    cabinet.visual(Box((0.105, 0.003, 0.006)), origin=Origin(xyz=(0.1675, -0.1965, 0.080)), material=dark_cavity, name="key_slot_lip_top")
    cabinet.visual(Box((0.105, 0.003, 0.006)), origin=Origin(xyz=(0.1675, -0.1965, 0.042)), material=dark_cavity, name="key_slot_lip_bottom")

    # The visible fixed oven chamber, set behind the front aperture.
    cabinet.visual(Box((0.286, 0.305, 0.008)), origin=Origin(xyz=(-0.055, -0.015, 0.058)), material=enamel_gray, name="cavity_floor")
    cabinet.visual(Box((0.286, 0.305, 0.008)), origin=Origin(xyz=(-0.055, -0.015, 0.226)), material=enamel_gray, name="cavity_ceiling")
    cabinet.visual(Box((0.008, 0.305, 0.176)), origin=Origin(xyz=(-0.202, -0.015, 0.142)), material=enamel_gray, name="cavity_wall_0")
    cabinet.visual(Box((0.008, 0.305, 0.176)), origin=Origin(xyz=(0.092, -0.015, 0.142)), material=enamel_gray, name="cavity_wall_1")
    cabinet.visual(Box((0.286, 0.010, 0.176)), origin=Origin(xyz=(-0.055, 0.1425, 0.142)), material=enamel_gray, name="cavity_back")
    cabinet.visual(Cylinder(radius=0.012, length=0.008), origin=Origin(xyz=(-0.055, -0.035, 0.066)), material=chrome, name="turntable_spindle")

    # Simple side vents make the casing read as a microwave, while remaining
    # fused to the side skin.
    for i, z in enumerate((0.155, 0.170, 0.185, 0.200, 0.215)):
        cabinet.visual(Box((0.003, 0.070, 0.006)), origin=Origin(xyz=(0.231, 0.018, z)), material=dark_cavity, name=f"side_vent_{i}")

    door = model.part("door")
    door_w = 0.315
    door_h = 0.226
    door_t = 0.026
    border = 0.035
    door.visual(Box((border, door_t, door_h)), origin=Origin(xyz=(border / 2.0, -0.013, 0.0)), material=black_plastic, name="hinge_stile")
    door.visual(Box((border, door_t, door_h)), origin=Origin(xyz=(door_w - border / 2.0, -0.013, 0.0)), material=black_plastic, name="latch_stile")
    door.visual(Box((door_w, door_t, border)), origin=Origin(xyz=(door_w / 2.0, -0.013, door_h / 2.0 - border / 2.0)), material=black_plastic, name="top_rail")
    door.visual(Box((door_w, door_t, border)), origin=Origin(xyz=(door_w / 2.0, -0.013, -door_h / 2.0 + border / 2.0)), material=black_plastic, name="bottom_rail")
    door.visual(Box((0.258, 0.004, 0.166)), origin=Origin(xyz=(door_w / 2.0, -0.028, 0.0)), material=smoky_glass, name="window_glass")
    door.visual(Box((0.012, 0.006, 0.150)), origin=Origin(xyz=(0.040, -0.029, 0.0)), material=chrome, name="hinge_trim")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.214, -0.195, 0.143)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.75),
    )

    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.108, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.006)), material=pale_glass, name="glass_disk")
    turntable.visual(Cylinder(radius=0.018, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=chrome, name="center_hub")
    model.articulation(
        "turntable_spindle",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=turntable,
        origin=Origin(xyz=(-0.055, -0.035, 0.073)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    knob_geom = KnobGeometry(
        0.052,
        0.020,
        body_style="skirted",
        top_diameter=0.042,
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,
    )
    for name, z in (("upper_dial", 0.194), ("lower_dial", 0.128)):
        dial = model.part(name)
        dial.visual(
            mesh_from_geometry(knob_geom, f"{name}_cap"),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="dial_cap",
        )
        model.articulation(
            f"{name}_axis",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=dial,
            origin=Origin(xyz=(0.1675, -0.195, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=6.0),
        )

    door_key = model.part("door_key")
    door_key.visual(Box((0.085, 0.010, 0.023)), origin=Origin(xyz=(0.0, -0.005, 0.0)), material=black_plastic, name="key_cap")
    door_key.visual(Box((0.064, 0.024, 0.015)), origin=Origin(xyz=(0.0, 0.011, 0.0)), material=black_plastic, name="key_shank")
    model.articulation(
        "key_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=door_key,
        origin=Origin(xyz=(0.1675, -0.195, 0.061)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.10, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    upper_dial = object_model.get_part("upper_dial")
    lower_dial = object_model.get_part("lower_dial")
    door_key = object_model.get_part("door_key")

    door_hinge = object_model.get_articulation("door_hinge")
    key_slide = object_model.get_articulation("key_slide")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="front_jamb",
        negative_elem="hinge_stile",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed door sits flush on the front jamb",
    )

    closed_latch = ctx.part_element_world_aabb(door, elem="latch_stile")
    with ctx.pose({door_hinge: 1.20}):
        opened_latch = ctx.part_element_world_aabb(door, elem="latch_stile")
    ctx.check(
        "door swings outward from the hinge side",
        closed_latch is not None
        and opened_latch is not None
        and opened_latch[0][1] < closed_latch[0][1] - 0.12,
        details=f"closed_latch={closed_latch}, opened_latch={opened_latch}",
    )

    ctx.expect_within(
        turntable,
        cabinet,
        axes="xy",
        inner_elem="glass_disk",
        outer_elem="cavity_floor",
        margin=0.003,
        name="glass turntable is centered over the oven floor",
    )
    ctx.expect_gap(
        turntable,
        cabinet,
        axis="z",
        positive_elem="glass_disk",
        negative_elem="cavity_floor",
        min_gap=0.010,
        max_gap=0.020,
        name="glass turntable rides low above the cavity floor",
    )
    ctx.expect_contact(
        turntable,
        cabinet,
        elem_a="center_hub",
        elem_b="turntable_spindle",
        contact_tol=0.0005,
        name="turntable hub is seated on the spindle",
    )

    ctx.expect_contact(
        upper_dial,
        cabinet,
        elem_a="dial_cap",
        elem_b="control_panel",
        contact_tol=0.002,
        name="upper dial is mounted to the front panel",
    )
    ctx.expect_contact(
        lower_dial,
        cabinet,
        elem_a="dial_cap",
        elem_b="control_panel",
        contact_tol=0.002,
        name="lower dial is mounted to the front panel",
    )

    def _key_clip_check(label: str) -> None:
        cap = ctx.part_element_world_aabb(door_key, elem="key_cap")
        shank = ctx.part_element_world_aabb(door_key, elem="key_shank")
        ok = (
            cap is not None
            and shank is not None
            and cap[0][0] >= 0.124
            and cap[1][0] <= 0.211
            and cap[0][2] >= 0.048
            and cap[1][2] <= 0.074
            and shank[1][1] >= -0.195
            and shank[0][1] <= -0.170
        )
        ctx.check(
            f"door key stays clipped in the guide slot {label}",
            ok,
            details=f"cap={cap}, shank={shank}",
        )

    _key_clip_check("at rest")
    rest_key = ctx.part_element_world_aabb(door_key, elem="key_cap")
    with ctx.pose({key_slide: 0.010}):
        _key_clip_check("when pressed")
        pressed_key = ctx.part_element_world_aabb(door_key, elem="key_cap")
    ctx.check(
        "door key translates inward along its guide",
        rest_key is not None
        and pressed_key is not None
        and pressed_key[0][1] > rest_key[0][1] + 0.008,
        details=f"rest_key={rest_key}, pressed_key={pressed_key}",
    )

    return ctx.report()


object_model = build_object_model()
