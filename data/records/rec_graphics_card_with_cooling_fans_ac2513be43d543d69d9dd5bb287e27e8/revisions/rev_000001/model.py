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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_fan_blades_mesh() -> MeshGeometry:
    blade_profile = [
        (0.010, -0.0044),
        (0.022, -0.0085),
        (0.038, -0.0070),
        (0.041, 0.0018),
        (0.030, 0.0090),
        (0.012, 0.0054),
    ]
    base_blade = ExtrudeGeometry.centered(blade_profile, 0.0018)

    blades = MeshGeometry()
    for blade_index in range(9):
        angle = math.radians(8.0) + (2.0 * math.pi * blade_index / 9.0)
        blades.merge(base_blade.copy().rotate_z(angle))
    return blades


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_graphics_card")

    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.12, 0.12, 0.13, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.08, 0.22, 0.14, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.70, 0.72, 0.76, 1.0))
    connector_black = model.material("connector_black", rgba=(0.06, 0.06, 0.07, 1.0))
    contact_gold = model.material("contact_gold", rgba=(0.83, 0.67, 0.22, 1.0))

    fan_center_x = 0.010

    shroud_top_mesh = _save_mesh(
        "gpu_shroud_top",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.150, 0.105, 0.012, corner_segments=8),
            [_circle_profile(0.046, 56)],
            height=0.002,
            center=True,
        ),
    )
    opening_ring_mesh = _save_mesh(
        "gpu_fan_opening_ring",
        ExtrudeWithHolesGeometry(
            _circle_profile(0.046, 56),
            [_circle_profile(0.043, 56)],
            height=0.003,
            center=True,
        ),
    )
    fan_blades_mesh = _save_mesh("gpu_fan_blades", _build_fan_blades_mesh())

    card_body = model.part("card_body")
    card_body.visual(
        Box((0.170, 0.110, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=pcb_green,
        name="pcb",
    )
    card_body.visual(
        Box((0.056, 0.010, 0.001)),
        origin=Origin(xyz=(0.004, -0.060, 0.0005)),
        material=contact_gold,
        name="pcie_edge",
    )
    card_body.visual(
        Box((0.145, 0.094, 0.004)),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.004)),
        material=heatsink_gray,
        name="heatsink_base",
    )
    fin_pitch = 0.009
    fin_start_y = -0.0405
    for fin_index in range(10):
        card_body.visual(
            Box((0.145, 0.004, 0.014)),
            origin=Origin(
                xyz=(
                    fan_center_x,
                    fin_start_y + fin_index * fin_pitch,
                    0.013,
                )
            ),
            material=heatsink_gray,
            name=f"fin_{fin_index:02d}",
        )
    card_body.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(fan_center_x, 0.0, 0.022)),
        material=matte_black,
        name="motor_boss",
    )
    card_body.visual(
        shroud_top_mesh,
        origin=Origin(xyz=(fan_center_x, 0.0, 0.034)),
        material=shroud_black,
        name="shroud_top",
    )
    card_body.visual(
        opening_ring_mesh,
        origin=Origin(xyz=(fan_center_x, 0.0, 0.0315)),
        material=shroud_black,
        name="fan_opening_ring",
    )
    card_body.visual(
        Box((0.150, 0.003, 0.031)),
        origin=Origin(xyz=(fan_center_x, 0.051, 0.0175)),
        material=shroud_black,
        name="shroud_side_right",
    )
    card_body.visual(
        Box((0.150, 0.003, 0.031)),
        origin=Origin(xyz=(fan_center_x, -0.051, 0.0175)),
        material=shroud_black,
        name="shroud_side_left",
    )
    card_body.visual(
        Box((0.003, 0.099, 0.031)),
        origin=Origin(xyz=(0.084, 0.0, 0.0175)),
        material=shroud_black,
        name="shroud_end_cap",
    )
    card_body.visual(
        Box((0.004, 0.118, 0.038)),
        origin=Origin(xyz=(-0.087, 0.0, 0.019)),
        material=bracket_steel,
        name="io_bracket",
    )
    card_body.visual(
        Box((0.012, 0.034, 0.018)),
        origin=Origin(xyz=(-0.079, 0.019, 0.016)),
        material=connector_black,
        name="display_port_upper",
    )
    card_body.visual(
        Box((0.012, 0.026, 0.018)),
        origin=Origin(xyz=(-0.079, -0.018, 0.016)),
        material=connector_black,
        name="display_port_lower",
    )
    card_body.inertial = Inertial.from_geometry(
        Box((0.175, 0.118, 0.038)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.0165, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="hub_shell",
    )
    fan_rotor.visual(
        Cylinder(radius=0.011, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00725)),
        material=connector_black,
        name="hub_cap",
    )
    fan_rotor.visual(
        fan_blades_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0038)),
        material=matte_black,
        name="fan_blades",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.008),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=fan_rotor,
        origin=Origin(xyz=(fan_center_x, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card_body = object_model.get_part("card_body")
    fan_rotor = object_model.get_part("fan_rotor")
    fan_spin = object_model.get_articulation("fan_spin")

    shroud_top = card_body.get_visual("shroud_top")
    opening_ring = card_body.get_visual("fan_opening_ring")
    motor_boss = card_body.get_visual("motor_boss")
    center_fin = card_body.get_visual("fin_05")
    card_body.get_visual("pcb")
    card_body.get_visual("io_bracket")

    hub_shell = fan_rotor.get_visual("hub_shell")
    fan_blades = fan_rotor.get_visual("fan_blades")

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
        "fan_spin_is_continuous",
        fan_spin.joint_type in (ArticulationType.CONTINUOUS, "continuous"),
        details=f"fan articulation type was {fan_spin.joint_type!r}",
    )
    ctx.check(
        "fan_spin_axis_is_vertical",
        tuple(round(value, 6) for value in fan_spin.axis) == (0.0, 0.0, 1.0),
        details=f"fan spin axis was {fan_spin.axis!r}",
    )
    ctx.check(
        "fan_spin_has_unbounded_rotation",
        fan_spin.motion_limits is not None
        and fan_spin.motion_limits.lower is None
        and fan_spin.motion_limits.upper is None,
        details="continuous fan joint should not clamp lower or upper rotation limits",
    )

    body_aabb = ctx.part_world_aabb(card_body)
    if body_aabb is None:
        ctx.fail("card_body_has_bounds", "card body did not produce a world AABB")
    else:
        dx = body_aabb[1][0] - body_aabb[0][0]
        dy = body_aabb[1][1] - body_aabb[0][1]
        dz = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "mini_itx_gpu_scale",
            0.17 <= dx <= 0.19 and 0.11 <= dy <= 0.125 and 0.035 <= dz <= 0.04,
            details=(
                f"expected compact mini-ITX proportions, got "
                f"{dx:.4f} x {dy:.4f} x {dz:.4f} m"
            ),
        )

    with ctx.pose({fan_spin: 0.0}):
        ctx.expect_contact(
            fan_rotor,
            card_body,
            elem_a=hub_shell,
            elem_b=motor_boss,
            name="rotor_hub_clipped_to_center_boss",
        )
        ctx.expect_gap(
            card_body,
            fan_rotor,
            axis="z",
            positive_elem=shroud_top,
            negative_elem=fan_blades,
            min_gap=0.003,
            name="blade_clearance_below_shroud",
        )
        ctx.expect_gap(
            fan_rotor,
            card_body,
            axis="z",
            positive_elem=fan_blades,
            negative_elem=center_fin,
            min_gap=0.004,
            name="blade_clearance_above_heatsink",
        )
        ctx.expect_within(
            fan_rotor,
            card_body,
            axes="xy",
            inner_elem=fan_blades,
            outer_elem=opening_ring,
            margin=0.0,
            name="fan_blades_stay_inside_opening",
        )

    with ctx.pose({fan_spin: math.pi / 3.0}):
        ctx.expect_contact(
            fan_rotor,
            card_body,
            elem_a=hub_shell,
            elem_b=motor_boss,
            name="rotor_remains_clipped_while_spinning",
        )
        ctx.expect_gap(
            card_body,
            fan_rotor,
            axis="z",
            positive_elem=shroud_top,
            negative_elem=fan_blades,
            min_gap=0.003,
            name="blade_clearance_below_shroud_at_spin_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
