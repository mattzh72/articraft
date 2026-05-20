from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rx_offset(offset: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    """Rotate a local offset about X; useful for features on slanted panels."""
    x, y, z = offset
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """CadQuery sleeve/collar with a true center clearance hole."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_casino_machine")

    black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_powdercoat", rgba=(0.08, 0.085, 0.09, 1.0))
    red = model.material("casino_red", rgba=(0.55, 0.02, 0.035, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.66, 1.0))
    glass = model.material("blue_glass", rgba=(0.05, 0.23, 0.45, 0.88))
    reel_white = model.material("lit_reel_white", rgba=(0.96, 0.92, 0.78, 1.0))
    brass = model.material("brass_lock", rgba=(0.92, 0.67, 0.23, 1.0))
    button_red = model.material("glossy_spin_red", rgba=(0.95, 0.03, 0.03, 1.0))

    cabinet = model.part("cabinet")

    # Main slant-top cabinet mass: plinth, belly area, button deck, and monitor head.
    cabinet.visual(
        Box((0.82, 0.66, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=black,
        name="plinth",
    )
    cabinet.visual(
        Box((0.78, 0.58, 0.66)),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=charcoal,
        name="lower_cabinet",
    )
    cabinet.visual(
        Box((0.74, 0.30, 0.06)),
        origin=Origin(xyz=(0.0, 0.18, 0.775)),
        material=black,
        name="button_deck",
    )
    cabinet.visual(
        Box((0.80, 0.16, 0.56)),
        origin=Origin(xyz=(0.0, -0.225, 0.91)),
        material=charcoal,
        name="rear_tower",
    )
    for side_name, x in (("side_cheek_0", -0.385), ("side_cheek_1", 0.385)):
        cabinet.visual(
            Box((0.045, 0.260, 0.620)),
            origin=Origin(xyz=(x, -0.100, 0.910)),
            material=charcoal,
            name=side_name,
        )
    cabinet.visual(
        Box((0.82, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.345, 0.77)),
        material=chrome,
        name="front_deck_lip",
    )

    # Slanted monitor panel, with separate glass and reel-window treatment.
    screen_angle = math.radians(25.0)
    screen_center = (0.0, -0.045, 1.075)
    cabinet.visual(
        Box((0.74, 0.055, 0.54)),
        origin=Origin(xyz=screen_center, rpy=(screen_angle, 0.0, 0.0)),
        material=black,
        name="angled_panel",
    )
    cabinet.visual(
        Box((0.61, 0.012, 0.34)),
        origin=Origin(
            xyz=_add(screen_center, _rx_offset((0.0, 0.033, 0.035), screen_angle)),
            rpy=(screen_angle, 0.0, 0.0),
        ),
        material=glass,
        name="screen_glass",
    )
    for index, x in enumerate((-0.205, 0.0, 0.205)):
        cabinet.visual(
            Box((0.155, 0.009, 0.205)),
            origin=Origin(
                xyz=_add(screen_center, _rx_offset((x, 0.041, 0.035), screen_angle)),
                rpy=(screen_angle, 0.0, 0.0),
            ),
            material=reel_white,
            name=f"reel_window_{index}",
        )
    cabinet.visual(
        Box((0.68, 0.016, 0.035)),
        origin=Origin(
            xyz=_add(screen_center, _rx_offset((0.0, 0.035, 0.250), screen_angle)),
            rpy=(screen_angle, 0.0, 0.0),
        ),
        material=red,
        name="marquee_band",
    )

    # Static sleeve for the prismatically moving spin button.
    spin_sleeve = _annular_cylinder(0.060, 0.047, 0.030)
    cabinet.visual(
        mesh_from_cadquery(spin_sleeve, "spin_sleeve"),
        origin=Origin(xyz=(0.22, 0.205, 0.803)),
        material=chrome,
        name="spin_sleeve",
    )

    # Small fixed legend plates on the deck read as printed labels, not extra controls.
    for index, x in enumerate((-0.18, -0.06)):
        cabinet.visual(
            Box((0.095, 0.040, 0.006)),
            origin=Origin(xyz=(x, 0.205, 0.8075)),
            material=Material("black_label", rgba=(0.01, 0.01, 0.012, 1.0)),
            name=f"deck_label_{index}",
        )

    # Fixed hinge knuckles and leaves for the belly door side hinge.
    hinge_x = -0.310
    hinge_y = 0.318
    hinge_z = 0.440
    cabinet.visual(
        Box((0.032, 0.040, 0.112)),
        origin=Origin(xyz=(hinge_x, 0.302, 0.290)),
        material=chrome,
        name="hinge_leaf_lower",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.108),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.290)),
        material=chrome,
        name="hinge_knuckle_lower",
    )
    cabinet.visual(
        Box((0.032, 0.040, 0.112)),
        origin=Origin(xyz=(hinge_x, 0.302, 0.590)),
        material=chrome,
        name="hinge_leaf_upper",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.108),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.590)),
        material=chrome,
        name="hinge_knuckle_upper",
    )
    cabinet.visual(
        Box((0.032, 0.040, 0.112)),
        origin=Origin(xyz=(hinge_x, 0.284, hinge_z)),
        material=chrome,
        name="hinge_leaf_middle",
    )

    # A chrome coin/bill bezel helps the lower cabinet read as a casino machine.
    cabinet.visual(
        Box((0.18, 0.020, 0.075)),
        origin=Origin(xyz=(0.23, 0.298, 0.625)),
        material=chrome,
        name="bill_bezel",
    )
    cabinet.visual(
        Box((0.13, 0.008, 0.030)),
        origin=Origin(xyz=(0.23, 0.311, 0.625)),
        material=black,
        name="bill_slot",
    )

    # Volume knob collar on the angled panel, with its hole aligned to the shaft axis.
    knob_panel_local = (0.285, 0.026, -0.185)
    collar_origin = _add(screen_center, _rx_offset(knob_panel_local, screen_angle))
    knob_origin = _add(screen_center, _rx_offset((0.285, 0.040, -0.185), screen_angle))
    knob_axis_rpy = (screen_angle - math.pi / 2.0, 0.0, 0.0)
    cabinet.visual(
        mesh_from_cadquery(_annular_cylinder(0.028, 0.014, 0.014), "volume_collar"),
        origin=Origin(xyz=collar_origin, rpy=knob_axis_rpy),
        material=chrome,
        name="volume_collar",
    )

    # Revolving belly access door with lock hardware and a matching middle hinge knuckle.
    belly_door = model.part("belly_door")
    belly_door.visual(
        Box((0.56, 0.026, 0.48)),
        origin=Origin(xyz=(0.285, 0.040, 0.0)),
        material=red,
        name="door_panel",
    )
    belly_door.visual(
        Box((0.53, 0.010, 0.035)),
        origin=Origin(xyz=(0.300, 0.056, 0.205)),
        material=chrome,
        name="door_top_trim",
    )
    belly_door.visual(
        Box((0.028, 0.030, 0.094)),
        origin=Origin(xyz=(0.006, 0.001, 0.0)),
        material=chrome,
        name="door_hinge_leaf",
    )
    belly_door.visual(
        Box((0.028, 0.022, 0.094)),
        origin=Origin(xyz=(0.006, 0.022, 0.0)),
        material=chrome,
        name="door_leaf_bridge",
    )
    belly_door.visual(
        Cylinder(radius=0.014, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="door_hinge_knuckle",
    )
    belly_door.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.415, 0.059, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_lock",
    )
    for name, x in (("handle_post_0", 0.385), ("handle_post_1", 0.465)):
        belly_door.visual(
            Box((0.016, 0.020, 0.034)),
            origin=Origin(xyz=(x, 0.057, -0.090)),
            material=chrome,
            name=name,
        )
    belly_door.visual(
        Box((0.110, 0.014, 0.028)),
        origin=Origin(xyz=(0.425, 0.067, -0.090)),
        material=chrome,
        name="pull_handle",
    )

    model.articulation(
        "belly_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=belly_door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    # Big red spin button captured by the deck sleeve. Positive travel is a press downward.
    spin_button = model.part("spin_button")
    spin_button.visual(
        Cylinder(radius=0.043, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=button_red,
        name="button_cap",
    )
    spin_button.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=black,
        name="button_stem",
    )
    spin_button.visual(
        Cylinder(radius=0.0478, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=black,
        name="button_guide",
    )
    model.articulation(
        "spin_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=spin_button,
        origin=Origin(xyz=(0.22, 0.205, 0.833)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=7.5, velocity=0.20, lower=0.0, upper=0.016),
    )

    # Continuously rotating volume knob about the angled-panel shaft.
    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        Cylinder(radius=0.0145, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=chrome,
        name="knob_shaft",
    )
    volume_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.028,
            body_style="faceted",
            top_diameter=0.044,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0010, width=0.0012),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "volume_knob_cap",
    )
    volume_knob.visual(
        volume_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black,
        name="knob_cap",
    )
    model.articulation(
        "volume_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=volume_knob,
        origin=Origin(xyz=knob_origin, rpy=knob_axis_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    belly_door = object_model.get_part("belly_door")
    spin_button = object_model.get_part("spin_button")
    volume_knob = object_model.get_part("volume_knob")
    belly_hinge = object_model.get_articulation("belly_hinge")
    spin_slide = object_model.get_articulation("spin_slide")
    volume_spin = object_model.get_articulation("volume_spin")

    ctx.allow_overlap(
        cabinet,
        spin_button,
        elem_a="spin_sleeve",
        elem_b="button_guide",
        reason="The hidden button guide is intentionally captured with a tiny radial interference inside the sleeve proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        volume_knob,
        elem_a="volume_collar",
        elem_b="knob_shaft",
        reason="The hidden knob shaft is intentionally seated through the collar bore that supports continuous rotation.",
    )

    ctx.expect_overlap(
        belly_door,
        cabinet,
        axes="xy",
        elem_a="door_hinge_knuckle",
        elem_b="hinge_knuckle_lower",
        min_overlap=0.020,
        name="belly door hinge is coaxial with fixed knuckles",
    )
    ctx.expect_contact(
        belly_door,
        cabinet,
        elem_a="door_hinge_leaf",
        elem_b="hinge_leaf_middle",
        contact_tol=0.0005,
        name="belly door hinge leaf bears on cabinet hinge leaf",
    )
    ctx.expect_within(
        spin_button,
        cabinet,
        axes="xy",
        inner_elem="button_guide",
        outer_elem="spin_sleeve",
        margin=0.0,
        name="spin button guide is centered inside sleeve footprint",
    )
    ctx.expect_overlap(
        spin_button,
        cabinet,
        axes="z",
        elem_a="button_guide",
        elem_b="spin_sleeve",
        min_overlap=0.018,
        name="spin button guide remains captured in sleeve at rest",
    )
    ctx.expect_within(
        volume_knob,
        cabinet,
        axes="xy",
        inner_elem="knob_shaft",
        outer_elem="volume_collar",
        margin=0.0,
        name="volume shaft is centered in collar",
    )
    ctx.expect_overlap(
        volume_knob,
        cabinet,
        axes="z",
        elem_a="knob_shaft",
        elem_b="volume_collar",
        min_overlap=0.003,
        name="volume shaft stays engaged in collar",
    )

    closed_door_aabb = ctx.part_element_world_aabb(belly_door, elem="door_panel")
    with ctx.pose({belly_hinge: 1.2}):
        open_door_aabb = ctx.part_element_world_aabb(belly_door, elem="door_panel")
    ctx.check(
        "belly door swings outward on vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_button_pos = ctx.part_world_position(spin_button)
    with ctx.pose({spin_slide: 0.016}):
        pressed_button_pos = ctx.part_world_position(spin_button)
        ctx.expect_overlap(
            spin_button,
            cabinet,
            axes="z",
            elem_a="button_guide",
            elem_b="spin_sleeve",
            min_overlap=0.010,
            name="pressed spin button travels down into sleeve",
        )
    ctx.check(
        "spin button prismatic motion is downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.010,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_knob_aabb = ctx.part_element_world_aabb(volume_knob, elem="knob_cap")
    with ctx.pose({volume_spin: math.pi}):
        turned_knob_aabb = ctx.part_element_world_aabb(volume_knob, elem="knob_cap")
    ctx.check(
        "volume knob accepts continuous rotation pose",
        rest_knob_aabb is not None and turned_knob_aabb is not None,
        details=f"rest={rest_knob_aabb}, turned={turned_knob_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
