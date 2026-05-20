from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _fan_ring_mesh(*, outer_radius: float, inner_radius: float, thickness: float, name: str):
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=48),
        [_circle_profile(inner_radius, segments=48)],
        thickness,
        center=True,
    )
    ring.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(ring, name)


def _shroud_cover_mesh(*, length: float, height: float, corner_radius: float, hole_radius: float, name: str):
    cover = ExtrudeWithHolesGeometry(
        rounded_rect_profile(length, height, corner_radius, corner_segments=8),
        [_circle_profile(hole_radius, segments=48)],
        0.0016,
        center=True,
    )
    cover.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(cover, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_graphics_card")

    bracket_metal = model.material("bracket_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.11, 0.34, 0.16, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.10, 0.11, 0.12, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.79, 0.67, 0.24, 1.0))
    port_black = model.material("port_black", rgba=(0.08, 0.09, 0.10, 1.0))
    fan_black = model.material("fan_black", rgba=(0.12, 0.13, 0.14, 1.0))

    shroud_cover_mesh = _shroud_cover_mesh(
        length=0.126,
        height=0.059,
        corner_radius=0.006,
        hole_radius=0.0245,
        name="gpu_shroud_cover",
    )
    fan_frame_mesh = _fan_ring_mesh(
        outer_radius=0.0245,
        inner_radius=0.0205,
        thickness=0.0026,
        name="gpu_fan_frame",
    )

    card_body = model.part("card_body")
    card_body.visual(
        Box((0.0016, 0.0185, 0.0790)),
        origin=Origin(xyz=(0.0008, 0.0, 0.0395)),
        material=bracket_metal,
        name="bracket_plate",
    )
    card_body.visual(
        Box((0.0060, 0.0160, 0.0080)),
        origin=Origin(xyz=(0.0030, 0.0, 0.0750)),
        material=bracket_metal,
        name="bracket_screw_ear",
    )
    card_body.visual(
        Box((0.0090, 0.0140, 0.0100)),
        origin=Origin(xyz=(0.0055, 0.0, 0.0130)),
        material=bracket_metal,
        name="board_mount_block",
    )
    card_body.visual(
        Box((0.1650, 0.0016, 0.0670)),
        origin=Origin(xyz=(0.0841, 0.0, 0.0375)),
        material=pcb_green,
        name="pcb_board",
    )
    card_body.visual(
        Box((0.0560, 0.0030, 0.0050)),
        origin=Origin(xyz=(0.0590, 0.0, 0.0025)),
        material=connector_gold,
        name="pcie_edge",
    )
    card_body.visual(
        Box((0.0100, 0.0120, 0.0270)),
        origin=Origin(xyz=(0.0048, 0.0, 0.0530)),
        material=port_black,
        name="display_output_stack",
    )
    card_body.visual(
        Box((0.0110, 0.0120, 0.0100)),
        origin=Origin(xyz=(0.0053, 0.0, 0.0210)),
        material=port_black,
        name="hdmi_output",
    )
    card_body.visual(
        Box((0.1180, 0.0084, 0.0480)),
        origin=Origin(xyz=(0.0860, 0.0050, 0.0380)),
        material=heatsink_gray,
        name="heatsink_block",
    )
    for index, y_pos in enumerate((0.0024, 0.0047, 0.0070, 0.0093)):
        card_body.visual(
            Box((0.0880, 0.0010, 0.0420)),
            origin=Origin(xyz=(0.0900, y_pos, 0.0380)),
            material=heatsink_gray,
            name=f"heatsink_fin_{index}",
        )
    card_body.visual(
        shroud_cover_mesh,
        origin=Origin(xyz=(0.0860, 0.0116, 0.0380)),
        material=shroud_black,
        name="shroud_cover",
    )
    card_body.visual(
        Box((0.1260, 0.0112, 0.0040)),
        origin=Origin(xyz=(0.0860, 0.0060, 0.0615)),
        material=shroud_black,
        name="shroud_top_rail",
    )
    card_body.visual(
        Box((0.1260, 0.0112, 0.0040)),
        origin=Origin(xyz=(0.0860, 0.0060, 0.0145)),
        material=shroud_black,
        name="shroud_bottom_rail",
    )
    card_body.visual(
        Box((0.0100, 0.0112, 0.0550)),
        origin=Origin(xyz=(0.0290, 0.0060, 0.0380)),
        material=shroud_black,
        name="shroud_left_wall",
    )
    card_body.visual(
        Box((0.0120, 0.0112, 0.0550)),
        origin=Origin(xyz=(0.1450, 0.0060, 0.0380)),
        material=shroud_black,
        name="shroud_right_wall",
    )
    card_body.visual(
        fan_frame_mesh,
        origin=Origin(xyz=(0.0820, 0.0118, 0.0380)),
        material=shroud_black,
        name="fan_frame",
    )
    card_body.visual(
        Cylinder(radius=0.0085, length=0.0070),
        origin=Origin(xyz=(0.0820, 0.0060, 0.0380), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=heatsink_gray,
        name="motor_boss",
    )
    card_body.visual(
        Box((0.0076, 0.0045, 0.0080)),
        origin=Origin(xyz=(0.0054, -0.0065, 0.0040)),
        material=bracket_metal,
        name="hinge_lug_left",
    )
    card_body.visual(
        Box((0.0076, 0.0045, 0.0080)),
        origin=Origin(xyz=(0.0054, 0.0065, 0.0040)),
        material=bracket_metal,
        name="hinge_lug_right",
    )
    card_body.inertial = Inertial.from_geometry(
        Box((0.1680, 0.0190, 0.0790)),
        mass=0.42,
        origin=Origin(xyz=(0.0840, 0.0060, 0.0395)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.0080, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_black,
        name="rotor_hub",
    )
    fan_rotor.visual(
        Cylinder(radius=0.0052, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_black,
        name="hub_cap",
    )
    for index in range(7):
        angle = (2.0 * math.pi * index) / 7.0
        fan_rotor.visual(
            Box((0.0240, 0.0014, 0.0062)),
            origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(0.0, angle, 0.22)),
            material=fan_black,
            name=f"blade_{index}",
        )
    fan_rotor.visual(
        Cylinder(radius=0.0185, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_black,
        name="blade_sweep",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0185, length=0.0050),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
    )

    support_tab = model.part("support_tab")
    support_tab.visual(
        Cylinder(radius=0.0030, length=0.0085),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_metal,
        name="hinge_barrel",
    )
    support_tab.visual(
        Box((0.0032, 0.0074, 0.0280)),
        origin=Origin(xyz=(0.0026, 0.0, -0.0155)),
        material=bracket_metal,
        name="tab_blade",
    )
    support_tab.visual(
        Box((0.0080, 0.0090, 0.0040)),
        origin=Origin(xyz=(0.0054, 0.0, -0.0305)),
        material=bracket_metal,
        name="tab_foot",
    )
    support_tab.inertial = Inertial.from_geometry(
        Box((0.0100, 0.0090, 0.0320)),
        mass=0.02,
        origin=Origin(xyz=(0.0040, 0.0, -0.0140)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=fan_rotor,
        origin=Origin(xyz=(0.0820, 0.0095, 0.0380)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=120.0),
    )
    model.articulation(
        "support_tab_hinge",
        ArticulationType.REVOLUTE,
        parent=card_body,
        child=support_tab,
        origin=Origin(xyz=(0.0062, 0.0, 0.0040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card_body = object_model.get_part("card_body")
    fan_rotor = object_model.get_part("fan_rotor")
    support_tab = object_model.get_part("support_tab")
    fan_spin = object_model.get_articulation("fan_spin")
    support_hinge = object_model.get_articulation("support_tab_hinge")

    bracket_plate = card_body.get_visual("bracket_plate")
    pcb_board = card_body.get_visual("pcb_board")
    fan_frame = card_body.get_visual("fan_frame")
    motor_boss = card_body.get_visual("motor_boss")
    hinge_lug_left = card_body.get_visual("hinge_lug_left")
    hinge_lug_right = card_body.get_visual("hinge_lug_right")

    rotor_hub = fan_rotor.get_visual("rotor_hub")
    blade_sweep = fan_rotor.get_visual("blade_sweep")

    hinge_barrel = support_tab.get_visual("hinge_barrel")
    tab_blade = support_tab.get_visual("tab_blade")
    tab_foot = support_tab.get_visual("tab_foot")

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
        "fan_joint_is_continuous",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"fan joint type is {fan_spin.articulation_type}",
    )
    ctx.check(
        "fan_joint_axis_faces_out_of_card",
        tuple(fan_spin.axis) == (0.0, 1.0, 0.0),
        details=f"fan axis is {fan_spin.axis}",
    )
    ctx.check(
        "support_hinge_axis_runs_through_bracket",
        tuple(support_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"support hinge axis is {support_hinge.axis}",
    )

    ctx.expect_contact(
        fan_rotor,
        card_body,
        elem_a=rotor_hub,
        elem_b=motor_boss,
        name="fan_rotor_attached_to_motor_boss",
    )
    ctx.expect_within(
        fan_rotor,
        card_body,
        axes="xz",
        inner_elem=blade_sweep,
        outer_elem=fan_frame,
        name="fan_rotor_stays_within_frame_projection",
    )
    ctx.expect_contact(
        support_tab,
        card_body,
        elem_a=hinge_barrel,
        elem_b=hinge_lug_left,
        name="support_tab_captured_by_left_lug",
    )
    ctx.expect_contact(
        support_tab,
        card_body,
        elem_a=hinge_barrel,
        elem_b=hinge_lug_right,
        name="support_tab_captured_by_right_lug",
    )

    with ctx.pose({support_hinge: 0.0}):
        ctx.expect_gap(
            support_tab,
            card_body,
            axis="x",
            positive_elem=tab_blade,
            negative_elem=bracket_plate,
            min_gap=0.004,
            max_gap=0.012,
            name="support_tab_stores_just_inboard_of_bracket",
        )
        ctx.expect_gap(
            card_body,
            support_tab,
            axis="z",
            positive_elem=pcb_board,
            negative_elem=tab_foot,
            min_gap=0.020,
            name="stored_support_tab_hangs_below_pcb",
        )

    with ctx.pose({support_hinge: 1.05}):
        ctx.expect_contact(
            support_tab,
            card_body,
            elem_a=hinge_barrel,
            elem_b=hinge_lug_left,
            name="support_tab_remains_attached_when_deployed",
        )
        ctx.expect_gap(
            support_tab,
            card_body,
            axis="x",
            positive_elem=tab_foot,
            negative_elem=bracket_plate,
            min_gap=0.015,
            name="deployed_support_tab_swings_forward",
        )
        ctx.expect_gap(
            card_body,
            support_tab,
            axis="z",
            positive_elem=pcb_board,
            negative_elem=tab_foot,
            min_gap=0.006,
            name="deployed_support_tab_stays_below_board",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
