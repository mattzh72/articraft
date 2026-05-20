from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, center: tuple[float, float] = (0.0, 0.0), segments: int = 32):
    cx, cy = center
    return [
        (
            cx + radius * cos((2.0 * pi * index) / segments),
            cy + radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _add_fan_rotor_visuals(
    rotor_part,
    *,
    blade_count: int,
    hub_radius: float,
    hub_height: float,
    blade_length: float,
    blade_width: float,
    blade_height: float,
    blade_center_radius: float,
    blade_phase: float,
    blade_material,
    cap_material,
) -> None:
    rotor_part.visual(
        Cylinder(radius=hub_radius, length=hub_height),
        origin=Origin(xyz=(0.0, 0.0, hub_height * 0.5)),
        material=blade_material,
        name="hub",
    )
    rotor_part.visual(
        Cylinder(radius=hub_radius * 0.48, length=hub_height * 0.32),
        origin=Origin(xyz=(0.0, 0.0, hub_height * 0.84)),
        material=cap_material,
        name="center_cap",
    )

    for blade_index in range(blade_count):
        angle = (2.0 * pi * blade_index) / blade_count
        rotor_part.visual(
            Box((blade_length, blade_width, blade_height)),
            origin=Origin(
                xyz=(
                    cos(angle) * blade_center_radius,
                    sin(angle) * blade_center_radius,
                    hub_height * 0.60,
                ),
                rpy=(0.26, 0.0, angle + blade_phase),
            ),
            material=blade_material,
            name=f"blade_{blade_index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_graphics_card_with_support_brace")

    pcb_green = model.material("pcb_green", rgba=(0.11, 0.16, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    fin_aluminum = model.material("fin_aluminum", rgba=(0.68, 0.71, 0.74, 1.0))
    bracket_silver = model.material("bracket_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    copper = model.material("copper", rgba=(0.73, 0.43, 0.25, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.83, 0.68, 0.20, 1.0))

    pcb_length = 0.174
    pcb_width = 0.106
    pcb_thickness = 0.002
    shroud_length = 0.170
    shroud_width = 0.112
    shroud_height = 0.014
    left_fan_center = (-0.040, 0.008, 0.026)
    right_fan_center = (0.049, -0.008, 0.026)

    card_body = model.part("card_body")
    card_body.inertial = Inertial.from_geometry(
        Box((0.186, 0.118, 0.040)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    card_body.visual(
        Box((pcb_length, pcb_width, pcb_thickness)),
        origin=Origin(xyz=(0.0, 0.0, pcb_thickness * 0.5)),
        material=pcb_green,
        name="pcb_board",
    )
    card_body.visual(
        Box((0.158, 0.100, 0.005)),
        origin=Origin(xyz=(0.004, 0.0, 0.0045)),
        material=charcoal,
        name="heatsink_base",
    )
    for fin_index in range(11):
        fin_y = -0.040 + fin_index * 0.008
        card_body.visual(
            Box((0.152, 0.0022, 0.016)),
            origin=Origin(xyz=(0.006, fin_y, 0.015)),
            material=fin_aluminum,
            name=f"fin_{fin_index:02d}",
        )
    for pipe_index, pipe_y in enumerate((-0.024, 0.0, 0.024)):
        card_body.visual(
            Cylinder(radius=0.004, length=0.148),
            origin=Origin(xyz=(0.010, pipe_y, 0.009), rpy=(0.0, pi / 2.0, 0.0)),
            material=copper,
            name=f"heatpipe_{pipe_index:02d}",
        )

    shroud_outer = rounded_rect_profile(shroud_length, shroud_width, radius=0.012, corner_segments=8)
    shroud_openings = [
        _circle_profile(0.034, center=left_fan_center[:2], segments=36),
        _circle_profile(0.034, center=right_fan_center[:2], segments=36),
    ]
    shroud_mesh = _mesh(
        "gpu_shroud_shell",
        ExtrudeWithHolesGeometry(
            shroud_outer,
            shroud_openings,
            height=shroud_height,
            center=True,
        ),
    )
    card_body.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=matte_black,
        name="shroud_shell",
    )
    card_body.visual(
        Box((0.030, 0.010, 0.002)),
        origin=Origin(xyz=(0.072, 0.030, 0.038)),
        material=dark_gray,
        name="top_badge_strip",
    )
    card_body.visual(
        Box((0.002, 0.118, 0.040)),
        origin=Origin(xyz=(-0.089, 0.0, 0.020)),
        material=bracket_silver,
        name="bracket_plate",
    )
    card_body.visual(
        Box((0.014, 0.024, 0.014)),
        origin=Origin(xyz=(-0.082, -0.020, 0.017)),
        material=charcoal,
        name="display_output_lower",
    )
    card_body.visual(
        Box((0.014, 0.024, 0.014)),
        origin=Origin(xyz=(-0.082, 0.020, 0.017)),
        material=charcoal,
        name="display_output_upper",
    )
    card_body.visual(
        Box((0.008, 0.016, 0.006)),
        origin=Origin(xyz=(-0.079, 0.0, 0.003)),
        material=bracket_silver,
        name="brace_hinge_mount",
    )
    card_body.visual(
        Box((0.050, 0.006, 0.001)),
        origin=Origin(xyz=(-0.010, -0.048, 0.0005)),
        material=connector_gold,
        name="pcie_fingers",
    )
    card_body.visual(
        Cylinder(radius=0.008, length=0.002),
        origin=Origin(xyz=left_fan_center[:2] + (0.025,)),
        material=dark_gray,
        name="left_bearing_cap",
    )
    for spoke_index, spoke_angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        card_body.visual(
            Box((0.033, 0.003, 0.002)),
            origin=Origin(
                xyz=(
                    left_fan_center[0] + cos(spoke_angle) * 0.019,
                    left_fan_center[1] + sin(spoke_angle) * 0.019,
                    0.025,
                ),
                rpy=(0.0, 0.0, spoke_angle),
            ),
            material=dark_gray,
            name=f"left_stator_spoke_{spoke_index:02d}",
        )
    card_body.visual(
        Cylinder(radius=0.008, length=0.002),
        origin=Origin(xyz=right_fan_center[:2] + (0.025,)),
        material=dark_gray,
        name="right_bearing_cap",
    )
    for spoke_index, spoke_angle in enumerate((pi / 5.0, pi / 5.0 + 2.0 * pi / 3.0, pi / 5.0 + 4.0 * pi / 3.0)):
        card_body.visual(
            Box((0.033, 0.003, 0.002)),
            origin=Origin(
                xyz=(
                    right_fan_center[0] + cos(spoke_angle) * 0.019,
                    right_fan_center[1] + sin(spoke_angle) * 0.019,
                    0.025,
                ),
                rpy=(0.0, 0.0, spoke_angle),
            ),
            material=dark_gray,
            name=f"right_stator_spoke_{spoke_index:02d}",
        )
    card_body.visual(
        Box((0.012, 0.016, 0.0016)),
        origin=Origin(xyz=(0.079, 0.0, 0.0008)),
        material=dark_gray,
        name="brace_landing_pad",
    )

    left_fan = model.part("left_fan")
    left_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.008),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    _add_fan_rotor_visuals(
        left_fan,
        blade_count=7,
        hub_radius=0.011,
        hub_height=0.006,
        blade_length=0.022,
        blade_width=0.008,
        blade_height=0.002,
        blade_center_radius=0.018,
        blade_phase=0.44,
        blade_material=charcoal,
        cap_material=bracket_silver,
    )

    right_fan = model.part("right_fan")
    right_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.008),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    _add_fan_rotor_visuals(
        right_fan,
        blade_count=7,
        hub_radius=0.011,
        hub_height=0.006,
        blade_length=0.022,
        blade_width=0.008,
        blade_height=0.002,
        blade_center_radius=0.018,
        blade_phase=0.56,
        blade_material=charcoal,
        cap_material=bracket_silver,
    )

    support_brace = model.part("support_brace")
    support_brace.inertial = Inertial.from_geometry(
        Box((0.166, 0.015, 0.004)),
        mass=0.05,
        origin=Origin(xyz=(0.083, 0.0, -0.002)),
    )
    support_brace.visual(
        Box((0.016, 0.012, 0.003)),
        origin=Origin(xyz=(0.010, 0.0, -0.0015)),
        material=bracket_silver,
        name="hinge_leaf",
    )
    support_brace.visual(
        Box((0.142, 0.008, 0.0024)),
        origin=Origin(xyz=(0.080, 0.0, -0.0032)),
        material=bracket_silver,
        name="main_strut",
    )
    support_brace.visual(
        Box((0.012, 0.015, 0.0026)),
        origin=Origin(xyz=(0.156, 0.0, -0.0013)),
        material=bracket_silver,
        name="tip_pad",
    )

    model.articulation(
        "left_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=left_fan,
        origin=Origin(xyz=left_fan_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=80.0),
    )
    model.articulation(
        "right_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=right_fan,
        origin=Origin(xyz=right_fan_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=80.0),
    )
    model.articulation(
        "support_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=card_body,
        child=support_brace,
        origin=Origin(xyz=(-0.079, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.35),
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

    part_names = {part.name for part in object_model.parts}
    articulation_names = {joint.name for joint in object_model.articulations}
    parts_ok = ctx.check(
        "expected parts exist",
        {"card_body", "left_fan", "right_fan", "support_brace"} <= part_names,
        f"found parts: {sorted(part_names)}",
    )
    joints_ok = ctx.check(
        "expected articulations exist",
        {"left_fan_spin", "right_fan_spin", "support_brace_hinge"} <= articulation_names,
        f"found articulations: {sorted(articulation_names)}",
    )
    if not (parts_ok and joints_ok):
        return ctx.report()

    card_body = object_model.get_part("card_body")
    left_fan = object_model.get_part("left_fan")
    right_fan = object_model.get_part("right_fan")
    support_brace = object_model.get_part("support_brace")
    left_spin = object_model.get_articulation("left_fan_spin")
    right_spin = object_model.get_articulation("right_fan_spin")
    brace_hinge = object_model.get_articulation("support_brace_hinge")

    left_hub = left_fan.get_visual("hub")
    right_hub = right_fan.get_visual("hub")
    left_bearing_cap = card_body.get_visual("left_bearing_cap")
    right_bearing_cap = card_body.get_visual("right_bearing_cap")
    brace_tip = support_brace.get_visual("tip_pad")
    brace_landing_pad = card_body.get_visual("brace_landing_pad")

    ctx.check(
        "fan joints spin normal to the card face",
        left_spin.axis == (0.0, 0.0, 1.0) and right_spin.axis == (0.0, 0.0, 1.0),
        f"left axis={left_spin.axis}, right axis={right_spin.axis}",
    )
    ctx.check(
        "support brace hinges about the card width axis",
        brace_hinge.axis == (0.0, 1.0, 0.0),
        f"brace axis={brace_hinge.axis}",
    )
    ctx.check(
        "brace hinge has a realistic fold range",
        brace_hinge.motion_limits is not None
        and brace_hinge.motion_limits.lower == 0.0
        and brace_hinge.motion_limits.upper is not None
        and 1.10 <= brace_hinge.motion_limits.upper <= 1.45,
        f"brace limits={brace_hinge.motion_limits}",
    )

    ctx.expect_contact(
        left_fan,
        card_body,
        elem_a=left_hub,
        elem_b=left_bearing_cap,
        name="left fan hub rides on bearing cap",
    )
    ctx.expect_contact(
        right_fan,
        card_body,
        elem_a=right_hub,
        elem_b=right_bearing_cap,
        name="right fan hub rides on bearing cap",
    )
    ctx.expect_contact(
        support_brace,
        card_body,
        elem_a=brace_tip,
        elem_b=brace_landing_pad,
        name="brace props the card outer edge",
    )

    left_pos = ctx.part_world_position(left_fan)
    right_pos = ctx.part_world_position(right_fan)
    if left_pos is not None and right_pos is not None:
        ctx.check(
            "fans are diagonally offset across the short shroud",
            left_pos[0] < -0.010
            and right_pos[0] > 0.010
            and right_pos[0] - left_pos[0] > 0.080
            and left_pos[1] > right_pos[1],
            f"left={left_pos}, right={right_pos}",
        )

    with ctx.pose({brace_hinge: 1.30}):
        ctx.expect_gap(
            card_body,
            support_brace,
            axis="z",
            positive_elem=brace_landing_pad,
            negative_elem=brace_tip,
            min_gap=0.12,
            name="folded brace drops clear of the propping pad",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
