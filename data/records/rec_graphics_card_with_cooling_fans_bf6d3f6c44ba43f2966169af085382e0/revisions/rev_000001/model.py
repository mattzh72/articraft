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
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CARD_LENGTH = 0.328
CARD_HEIGHT = 0.126
CARD_THICKNESS = 0.057
FAN_CENTERS = (-0.104, 0.0, 0.104)
FAN_Z = 0.004
FAN_OPENING_RADIUS = 0.0495


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _ring_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    thickness: float,
    segments: int = 64,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * thickness),
            (outer_radius, 0.5 * thickness),
        ],
        [
            (inner_radius, -0.5 * thickness),
            (inner_radius, 0.5 * thickness),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-math.pi / 2.0)


def _fan_rotor_pack_mesh() -> MeshGeometry:
    outer_rim = _ring_mesh(inner_radius=0.0375, outer_radius=0.0405, thickness=0.0036, segments=72)
    inner_collar = _ring_mesh(inner_radius=0.0140, outer_radius=0.0205, thickness=0.0036, segments=48)
    center_hub = ExtrudeGeometry(_circle_profile(0.0148, segments=40), 0.0036, center=True).rotate_x(-math.pi / 2.0)
    blade_profile = [
        (-0.006, -0.021),
        (0.001, -0.023),
        (0.010, -0.012),
        (0.011, 0.003),
        (0.005, 0.020),
        (-0.002, 0.024),
        (-0.010, 0.010),
        (-0.011, -0.005),
    ]
    base_blade = ExtrudeGeometry(blade_profile, 0.0018, center=True).rotate_x(-math.pi / 2.0)
    base_blade.rotate_z(0.48).translate(0.0245, 0.0, 0.0)

    pack = MeshGeometry()
    pack.merge(outer_rim)
    pack.merge(inner_collar)
    pack.merge(center_hub)
    for index in range(9):
        pack.merge(base_blade.copy().rotate_y((2.0 * math.pi * index) / 9.0))
    return pack


def _add_fan_mount(part, *, fan_index: int, x_pos: float, frame_mesh, accent_material, dark_material) -> None:
    part.visual(
        frame_mesh,
        origin=Origin(xyz=(x_pos, 0.0205, FAN_Z)),
        material=accent_material,
        name=f"fan_{fan_index}_frame",
    )


def _build_gpu_body(model: ArticulatedObject, frame_mesh) -> object:
    shroud_black = model.material("shroud_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    deep_gray = model.material("deep_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    heatsink_silver = model.material("heatsink_silver", rgba=(0.70, 0.73, 0.76, 1.0))
    pcb_black = model.material("pcb_black", rgba=(0.08, 0.10, 0.09, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    contact_gold = model.material("contact_gold", rgba=(0.82, 0.68, 0.24, 1.0))

    body = model.part("gpu_body")

    fascia_holes = [_circle_profile(FAN_OPENING_RADIUS, center=(fan_x, FAN_Z), segments=56) for fan_x in FAN_CENTERS]
    fascia_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(CARD_LENGTH - 0.010, CARD_HEIGHT - 0.004, 0.015, corner_segments=8),
        fascia_holes,
        height=0.007,
        center=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0)
    body.visual(
        _save_mesh("gpu_front_fascia", fascia_geom),
        origin=Origin(xyz=(0.0, 0.0215, 0.0)),
        material=shroud_black,
        name="front_fascia",
    )
    body.visual(
        Box((CARD_LENGTH - 0.014, 0.0025, CARD_HEIGHT - 0.002)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=deep_gray,
        name="backplate",
    )
    body.visual(
        Box((CARD_LENGTH - 0.026, 0.046, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, 0.057)),
        material=satin_graphite,
        name="top_spine",
    )
    body.visual(
        Box((CARD_LENGTH - 0.026, 0.046, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, -0.0585)),
        material=satin_graphite,
        name="bottom_rail",
    )
    body.visual(
        Box((0.010, 0.046, CARD_HEIGHT - 0.012)),
        origin=Origin(xyz=(-0.156, -0.002, 0.0)),
        material=satin_graphite,
        name="io_end_wall",
    )
    body.visual(
        Box((0.010, 0.046, CARD_HEIGHT - 0.018)),
        origin=Origin(xyz=(0.156, -0.002, 0.0)),
        material=satin_graphite,
        name="power_end_wall",
    )
    body.visual(
        Box((0.285, 0.024, 0.108)),
        origin=Origin(xyz=(0.0, -0.012, 0.002)),
        material=heatsink_silver,
        name="heatsink_core",
    )
    for fin_index in range(8):
        body.visual(
            Box((0.274, 0.0014, 0.101)),
            origin=Origin(xyz=(0.0, -0.022 + 0.003 * fin_index, 0.002)),
            material=heatsink_silver,
            name=f"fin_{fin_index}",
        )
    body.visual(
        Box((0.295, 0.0018, 0.106)),
        origin=Origin(xyz=(0.004, -0.0215, 0.002)),
        material=pcb_black,
        name="pcb",
    )
    body.visual(
        Box((0.100, 0.0012, 0.009)),
        origin=Origin(xyz=(0.016, -0.0225, -0.056)),
        material=contact_gold,
        name="pcie_connector",
    )
    body.visual(
        Box((0.008, 0.054, CARD_HEIGHT)),
        origin=Origin(xyz=(-0.165, -0.002, 0.0)),
        material=bracket_steel,
        name="pcie_bracket",
    )
    body.visual(
        Box((0.010, 0.024, 0.020)),
        origin=Origin(xyz=(-0.161, 0.010, 0.025)),
        material=bracket_steel,
        name="display_port_stack_upper",
    )
    body.visual(
        Box((0.010, 0.024, 0.020)),
        origin=Origin(xyz=(-0.161, 0.010, -0.005)),
        material=bracket_steel,
        name="display_port_stack_lower",
    )
    body.visual(
        Box((0.074, 0.018, 0.012)),
        origin=Origin(xyz=(0.090, 0.000, 0.056)),
        material=satin_graphite,
        name="power_connector_housing",
    )
    body.visual(
        Box((0.120, 0.026, 0.010)),
        origin=Origin(xyz=(0.032, 0.004, 0.048)),
        material=deep_gray,
        name="top_accent_ridge",
    )
    body.visual(
        Box((0.022, 0.028, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=deep_gray,
        name="leg_hinge_housing",
    )
    body.visual(
        Cylinder(radius=0.0052, length=0.014),
        origin=Origin(
            xyz=(0.0, -0.012, -0.0685),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=deep_gray,
        name="leg_hinge_left_barrel",
    )
    body.visual(
        Cylinder(radius=0.0052, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.012, -0.0685),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=deep_gray,
        name="leg_hinge_right_barrel",
    )

    for fan_index, fan_x in enumerate(FAN_CENTERS, start=1):
        _add_fan_mount(
            body,
            fan_index=fan_index,
            x_pos=fan_x,
            frame_mesh=frame_mesh,
            accent_material=deep_gray,
            dark_material=satin_graphite,
        )

    body.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_THICKNESS, CARD_HEIGHT)),
        mass=1.85,
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
    )
    return body


def _build_fan_part(model: ArticulatedObject, name: str, rotor_pack_mesh, material_name: str):
    fan_material = model.material(material_name, rgba=(0.13, 0.14, 0.15, 1.0))
    cap_material = model.material(f"{material_name}_cap", rgba=(0.22, 0.23, 0.25, 1.0))
    part = model.part(name)
    part.visual(
        rotor_pack_mesh,
        origin=Origin(),
        material=fan_material,
        name="blade_pack",
    )
    part.visual(
        Cylinder(radius=0.0155, length=0.0032),
        origin=Origin(
            xyz=(0.0, 0.0034, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=cap_material,
        name="hub_cap",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.016),
        mass=0.08,
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )
    return part


def _build_prop_leg(model: ArticulatedObject):
    leg_material = model.material("leg_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    foot_material = model.material("leg_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    leg = model.part("anti_sag_leg")
    leg.visual(
        Cylinder(radius=0.0047, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=leg_material,
        name="hinge_knuckle",
    )
    leg.visual(
        Box((0.098, 0.008, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, -0.004)),
        material=leg_material,
        name="leg_strut",
    )
    leg.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.087, 0.0, -0.010)),
        material=leg_material,
        name="leg_knee_pad",
    )
    leg.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.111, 0.0, -0.012)),
        material=foot_material,
        name="foot_pad",
    )
    leg.inertial = Inertial.from_geometry(
        Box((0.126, 0.016, 0.028)),
        mass=0.11,
        origin=Origin(xyz=(0.058, 0.0, -0.008)),
    )
    return leg


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_gpu_with_support_leg")

    frame_mesh = _save_mesh("gpu_fan_frame_ring", _ring_mesh(inner_radius=0.0415, outer_radius=0.0485, thickness=0.006, segments=72))
    rotor_pack_mesh = _save_mesh("gpu_rotor_blade_pack", _fan_rotor_pack_mesh())

    body = _build_gpu_body(model, frame_mesh)

    fan_parts = []
    for index in range(3):
        fan_parts.append(
            _build_fan_part(
                model,
                f"fan_rotor_{index + 1}",
                rotor_pack_mesh,
                f"fan_blades_{index + 1}",
            )
        )

    leg = _build_prop_leg(model)

    for index, (fan_part, fan_x) in enumerate(zip(fan_parts, FAN_CENTERS), start=1):
        model.articulation(
            f"fan_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=fan_part,
            origin=Origin(xyz=(fan_x, 0.0055, FAN_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=55.0),
        )

    model.articulation(
        "anti_sag_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=leg,
        origin=Origin(xyz=(0.0, 0.0, -0.0685)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("gpu_body")
    fan_1 = object_model.get_part("fan_rotor_1")
    fan_2 = object_model.get_part("fan_rotor_2")
    fan_3 = object_model.get_part("fan_rotor_3")
    leg = object_model.get_part("anti_sag_leg")

    fan_1_spin = object_model.get_articulation("fan_1_spin")
    fan_2_spin = object_model.get_articulation("fan_2_spin")
    fan_3_spin = object_model.get_articulation("fan_3_spin")
    leg_hinge = object_model.get_articulation("anti_sag_leg_hinge")

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

    for part_name in ("gpu_body", "fan_rotor_1", "fan_rotor_2", "fan_rotor_3", "anti_sag_leg"):
        ctx.check(f"{part_name}_present", True)
    for articulation_name in ("fan_1_spin", "fan_2_spin", "fan_3_spin", "anti_sag_leg_hinge"):
        ctx.check(f"{articulation_name}_present", True)

    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    size_x = body_aabb[1][0] - body_aabb[0][0]
    size_y = body_aabb[1][1] - body_aabb[0][1]
    size_z = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "card_size_realistic",
        0.31 <= size_x <= 0.35 and 0.048 <= size_y <= 0.065 and 0.12 <= size_z <= 0.14,
        details=f"gpu body dims were {(size_x, size_y, size_z)}",
    )

    for index, fan in enumerate((fan_1, fan_2, fan_3), start=1):
        ctx.expect_within(
            fan,
            body,
            axes="xz",
            margin=0.010,
            outer_elem=f"fan_{index}_frame",
            name=f"fan_{index}_within_frame",
        )
        ctx.expect_gap(
            body,
            fan,
            axis="y",
            positive_elem="front_fascia",
            negative_elem="blade_pack",
            min_gap=0.002,
            max_gap=0.010,
            name=f"fan_{index}_front_clearance",
        )

    ctx.expect_contact(
        leg,
        body,
        elem_a="hinge_knuckle",
        elem_b="leg_hinge_left_barrel",
        name="prop_leg_left_barrel_contact",
    )
    ctx.expect_contact(
        leg,
        body,
        elem_a="hinge_knuckle",
        elem_b="leg_hinge_right_barrel",
        name="prop_leg_right_barrel_contact",
    )
    ctx.expect_gap(
        body,
        leg,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="foot_pad",
        max_gap=0.02,
        max_penetration=0.0,
        name="prop_leg_tucks_under_body",
    )

    for joint in (fan_1_spin, fan_2_spin, fan_3_spin):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_and_limits",
            tuple(joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"{joint.name} axis={joint.axis} limits={limits}",
        )

    leg_limits = leg_hinge.motion_limits
    ctx.check(
        "prop_leg_hinge_axis_and_limits",
        tuple(leg_hinge.axis) == (0.0, 1.0, 0.0)
        and leg_limits is not None
        and leg_limits.lower == 0.0
        and leg_limits.upper is not None
        and leg_limits.upper >= 1.35,
        details=f"anti_sag_leg_hinge axis={leg_hinge.axis} limits={leg_limits}",
    )

    with ctx.pose({fan_1_spin: 1.1, fan_2_spin: 2.35, fan_3_spin: -0.9}):
        for index, fan in enumerate((fan_1, fan_2, fan_3), start=1):
            ctx.expect_within(
                fan,
                body,
                axes="xz",
                margin=0.010,
                outer_elem=f"fan_{index}_frame",
                name=f"fan_{index}_within_frame_after_spin",
            )

    with ctx.pose({leg_hinge: 1.33}):
        ctx.expect_contact(leg, body, elem_a="hinge_knuckle", elem_b="leg_hinge_left_barrel")
        ctx.expect_contact(leg, body, elem_a="hinge_knuckle", elem_b="leg_hinge_right_barrel")
        ctx.expect_gap(
            body,
            leg,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="foot_pad",
            min_gap=0.08,
            name="prop_leg_deploys_below_card",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
