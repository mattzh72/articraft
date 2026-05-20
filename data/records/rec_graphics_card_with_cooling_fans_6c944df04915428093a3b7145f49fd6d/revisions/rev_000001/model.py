from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _support_tab_geometry() -> cq.Workplane:
    """Fold-out prop tab with a real hollow hinge clip around the local Y axis."""
    sleeve_length = 0.020
    outer_radius = 0.0036
    inner_radius = 0.00225
    tab_length = 0.052
    tab_width = 0.014
    tab_thickness = 0.0020

    sleeve = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(sleeve_length)
        .translate((0.0, 0.0, -sleeve_length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )
    plate = cq.Workplane("XY").box(tab_length, tab_width, tab_thickness).translate(
        (outer_radius + tab_length / 2.0 - 0.001, 0.0, -0.0042)
    )
    root_web = cq.Workplane("XY").box(0.008, tab_width * 0.92, 0.0020).translate(
        (0.0046, 0.0, -0.0036)
    )
    stiffening_bead = cq.Workplane("XY").box(tab_length * 0.72, 0.0024, 0.0018).translate(
        (outer_radius + tab_length * 0.36, 0.0, -0.0028)
    )
    small_foot = cq.Workplane("XY").box(0.010, tab_width * 0.92, 0.0030).translate(
        (outer_radius + tab_length - 0.003, 0.0, -0.0050)
    )

    return sleeve.union(root_web).union(plate).union(stiffening_bead).union(small_foot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_graphics_card")

    pcb_green = model.material("solder_mask_green", color=(0.02, 0.26, 0.10, 1.0))
    matte_black = model.material("matte_black", color=(0.005, 0.005, 0.006, 1.0))
    rotor_black = model.material("rotor_dark_graphite", color=(0.015, 0.016, 0.018, 1.0))
    steel = model.material("brushed_steel", color=(0.62, 0.64, 0.66, 1.0))
    dark_port = model.material("black_port_insert", color=(0.015, 0.014, 0.013, 1.0))
    gold = model.material("gold_contacts", color=(0.96, 0.67, 0.18, 1.0))
    component_gray = model.material("component_gray", color=(0.12, 0.13, 0.13, 1.0))

    fan_center = (0.083, 0.0, 0.0170)
    hinge_origin = (-0.014, -0.052, 0.0120)

    card = model.part("card")

    # Short low-profile PCB, with the metal half-height I/O bracket at the left edge.
    card.visual(
        Box((0.165, 0.068, 0.0016)),
        origin=Origin(xyz=(0.075, 0.0, 0.0008)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.0040, 0.078, 0.034)),
        origin=Origin(xyz=(-0.0090, 0.0, 0.0170)),
        material=steel,
        name="half_height_bracket",
    )
    card.visual(
        Box((0.010, 0.034, 0.0022)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0021)),
        material=steel,
        name="bracket_foot",
    )
    card.visual(
        Box((0.0026, 0.024, 0.007)),
        origin=Origin(xyz=(-0.0115, -0.018, 0.020)),
        material=dark_port,
        name="display_port",
    )
    card.visual(
        Box((0.0026, 0.019, 0.006)),
        origin=Origin(xyz=(-0.0115, 0.017, 0.019)),
        material=dark_port,
        name="hdmi_port",
    )

    # PCIe edge connector and a few low components visible outside the shroud.
    card.visual(
        Box((0.060, 0.006, 0.0022)),
        origin=Origin(xyz=(0.054, -0.0370, 0.0018)),
        material=gold,
        name="edge_connector",
    )
    card.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.026, 0.026, 0.0032)),
        material=component_gray,
        name="power_choke_0",
    )
    card.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.044, 0.026, 0.0032)),
        material=component_gray,
        name="power_choke_1",
    )
    card.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(0.130, -0.024, 0.0042)),
        material=component_gray,
        name="capacitor_0",
    )
    card.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(0.144, -0.024, 0.0042)),
        material=component_gray,
        name="capacitor_1",
    )

    # Slim shroud plate with a real circular fan opening cut through it.
    shroud = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.132, 0.056, 0.006, corner_segments=8),
        [_circle_profile(0.0255, segments=80)],
        0.006,
        center=True,
    )
    card.visual(
        mesh_from_geometry(shroud, "slim_shroud"),
        origin=Origin(xyz=(fan_center[0], fan_center[1], 0.0100)),
        material=matte_black,
        name="slim_shroud",
    )
    fan_frame = ExtrudeWithHolesGeometry(
        _circle_profile(0.0305, segments=96),
        [_circle_profile(0.0254, segments=96)],
        0.004,
        center=True,
    )
    card.visual(
        mesh_from_geometry(fan_frame, "fan_frame"),
        origin=Origin(xyz=(fan_center[0], fan_center[1], 0.0148)),
        material=matte_black,
        name="fan_frame",
    )
    # Corner stand-offs visibly hold the shroud off the PCB.
    for idx, (sx, sy) in enumerate(
        ((0.026, -0.021), (0.140, -0.021), (0.026, 0.021), (0.140, 0.021))
    ):
        card.visual(
            Cylinder(radius=0.0024, length=0.0090),
            origin=Origin(xyz=(sx, sy, 0.0059)),
            material=matte_black,
            name=f"shroud_post_{idx}",
        )
    card.visual(
        Cylinder(radius=0.00085, length=0.012),
        origin=Origin(xyz=fan_center),
        material=steel,
        name="fan_shaft",
    )

    # Bracket-end hinge: a pin retained by a small lug mounted to the half-height bracket.
    card.visual(
        Box((0.008, 0.005, 0.0040)),
        origin=Origin(xyz=(-0.014, hinge_origin[1] + 0.014, 0.0060)),
        material=steel,
        name="tab_hinge_lug",
    )
    for idx, y_pos in enumerate((hinge_origin[1] - 0.0132, hinge_origin[1] + 0.0132)):
        card.visual(
            Box((0.0065, 0.0030, 0.0100)),
            origin=Origin(xyz=(-0.014, y_pos, 0.0088)),
            material=steel,
            name=f"hinge_cheek_{idx}",
        )
    card.visual(
        Cylinder(radius=0.00225, length=0.026),
        origin=Origin(xyz=hinge_origin, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    # Continuous axial fan rotor centered inside the circular shroud frame.
    fan_rotor = model.part("fan_rotor")
    rotor_mesh = FanRotorGeometry(
        outer_radius=0.0215,
        hub_radius=0.0066,
        blade_count=7,
        thickness=0.0060,
        blade_pitch_deg=31.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.0010),
        hub=FanRotorHub(style="domed", bore_diameter=0.0032),
    )
    fan_rotor.visual(
        mesh_from_geometry(rotor_mesh, "fan_rotor"),
        origin=Origin(),
        material=rotor_black,
        name="rotor",
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card,
        child=fan_rotor,
        origin=Origin(xyz=fan_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=60.0),
    )

    # Fold-out prop tab.  The mesh contains a hollow sleeve that clips around the
    # parent hinge pin, so the tab remains captured while rotating.
    support_tab = model.part("support_tab")
    support_tab.visual(
        mesh_from_cadquery(_support_tab_geometry(), "support_tab", tolerance=0.00045, angular_tolerance=0.08),
        origin=Origin(),
        material=steel,
        name="tab_panel",
    )
    model.articulation(
        "tab_hinge",
        ArticulationType.REVOLUTE,
        parent=card,
        child=support_tab,
        origin=Origin(xyz=hinge_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    card = object_model.get_part("card")
    fan_rotor = object_model.get_part("fan_rotor")
    support_tab = object_model.get_part("support_tab")
    fan_spin = object_model.get_articulation("fan_spin")
    tab_hinge = object_model.get_articulation("tab_hinge")

    ctx.allow_overlap(
        card,
        support_tab,
        elem_a="hinge_pin",
        elem_b="tab_panel",
        reason="The fold-out tab sleeve is intentionally clipped around the bracket hinge pin so it remains retained while rotating.",
    )

    ctx.expect_within(
        fan_rotor,
        card,
        axes="xy",
        inner_elem="rotor",
        outer_elem="fan_frame",
        margin=0.001,
        name="fan rotor sits inside circular frame",
    )
    with ctx.pose({fan_spin: 1.7}):
        ctx.expect_within(
            fan_rotor,
            card,
            axes="xy",
            inner_elem="rotor",
            outer_elem="fan_frame",
            margin=0.001,
            name="spinning rotor remains framed",
        )

    ctx.expect_overlap(
        card,
        support_tab,
        axes="y",
        elem_a="hinge_pin",
        elem_b="tab_panel",
        min_overlap=0.018,
        name="hinge pin is captured by tab clip",
    )
    ctx.expect_contact(
        card,
        support_tab,
        elem_a="hinge_pin",
        elem_b="tab_panel",
        name="hinge pin bears in clipped sleeve",
    )
    ctx.expect_overlap(
        support_tab,
        card,
        axes="xz",
        elem_a="tab_panel",
        elem_b="hinge_pin",
        min_overlap=0.0015,
        name="tab clip surrounds hinge axis",
    )

    stored_aabb = ctx.part_element_world_aabb(support_tab, elem="tab_panel")
    with ctx.pose({tab_hinge: 1.20}):
        ctx.expect_overlap(
            support_tab,
            card,
            axes="xz",
            elem_a="tab_panel",
            elem_b="hinge_pin",
            min_overlap=0.0015,
            name="deployed tab remains clipped to hinge",
        )
        deployed_aabb = ctx.part_element_world_aabb(support_tab, elem="tab_panel")

    ctx.check(
        "support tab deploys downward",
        stored_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < stored_aabb[0][2] - 0.020,
        details=f"stored={stored_aabb}, deployed={deployed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
