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


CARD_LENGTH = 0.320
CARD_WIDTH = 0.136
BODY_THICKNESS = 0.052
SHROUD_TOP_THICKNESS = 0.006
FAN_HOLE_RADIUS = 0.041
FAN_RADIUS = 0.034
FAN_Z = 0.037
FAN_CENTERS = (-0.104, 0.0, 0.104)


def _circle_profile(radius: float, *, cx: float = 0.0, cy: float = 0.0, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * cos((2.0 * pi * index) / segments),
            cy + radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _add_fan_rotor_visuals(part, *, blade_count: int, blade_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=0.013, length=0.008),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=hub_material,
        name="motor_plate",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=hub_material,
        name="hub_cap",
    )

    blade_reach = 0.018
    for blade_index in range(blade_count):
        angle = (2.0 * pi * blade_index) / blade_count
        part.visual(
            Box((0.030, 0.010, 0.0022)),
            origin=Origin(
                xyz=(blade_reach * cos(angle), blade_reach * sin(angle), 0.0),
                rpy=(0.12, 0.0, angle + 0.55),
            ),
            material=blade_material,
            name=f"blade_{blade_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.12, 0.13, 0.14, 1.0))
    shroud_gray = model.material("shroud_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    heatsink_dark = model.material("heatsink_dark", rgba=(0.32, 0.34, 0.37, 1.0))
    pcb_black = model.material("pcb_black", rgba=(0.05, 0.08, 0.06, 1.0))
    backplate_black = model.material("backplate_black", rgba=(0.09, 0.10, 0.11, 1.0))
    bracket_silver = model.material("bracket_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    fan_black = model.material("fan_black", rgba=(0.08, 0.08, 0.09, 1.0))
    fan_hub_gray = model.material("fan_hub_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.75, 0.63, 0.22, 1.0))

    card_body = model.part("card_body")
    card_body.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_WIDTH, BODY_THICKNESS)),
        mass=1.65,
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS * 0.5)),
    )

    card_body.visual(
        Box((0.318, 0.128, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=pcb_black,
        name="pcb",
    )
    card_body.visual(
        Box((0.110, 0.007, 0.002)),
        origin=Origin(xyz=(-0.010, -0.0615, 0.0050)),
        material=connector_gold,
        name="pcie_edge",
    )
    card_body.visual(
        Box((0.286, 0.114, 0.031)),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=heatsink_dark,
        name="heatsink_core",
    )
    card_body.visual(
        Box((0.250, 0.098, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0380)),
        material=heatsink_dark,
        name="vapor_plate",
    )

    for fin_index in range(13):
        y = -0.042 + fin_index * 0.007
        card_body.visual(
            Box((0.278, 0.0024, 0.028)),
            origin=Origin(xyz=(0.0, y, 0.0220)),
            material=shroud_gray,
            name=f"fin_{fin_index:02d}",
        )

    for pipe_index, y in enumerate((-0.030, -0.010, 0.010, 0.030)):
        card_body.visual(
            Cylinder(radius=0.0055, length=0.258),
            origin=Origin(xyz=(0.010, y, 0.0260), rpy=(0.0, pi / 2.0, 0.0)),
            material=shroud_gray,
            name=f"heatpipe_{pipe_index}",
        )

    shroud_outer = rounded_rect_profile(0.314, 0.128, 0.014, corner_segments=8)
    shroud_holes = [_circle_profile(FAN_HOLE_RADIUS, cx=fan_x) for fan_x in FAN_CENTERS]
    shroud_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            shroud_outer,
            shroud_holes,
            height=SHROUD_TOP_THICKNESS,
            center=True,
        ),
        "gpu_shroud_top",
    )
    card_body.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=shroud_black,
        name="shroud_top",
    )
    fan_bezel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.048, segments=48),
            [_circle_profile(0.0415, segments=48)],
            height=0.003,
            center=True,
        ),
        "gpu_fan_bezel",
    )
    for fan_index, fan_x in enumerate(FAN_CENTERS):
        card_body.visual(
            fan_bezel_mesh,
            origin=Origin(xyz=(fan_x, 0.0, 0.053)),
            material=shroud_gray,
            name=f"fan_{fan_index}_bezel",
        )

    card_body.visual(
        Box((0.310, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.058, 0.042), rpy=(0.34, 0.0, 0.0)),
        material=shroud_gray,
        name="upper_side_skirt",
    )
    card_body.visual(
        Box((0.310, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, -0.058, 0.042), rpy=(-0.34, 0.0, 0.0)),
        material=shroud_gray,
        name="lower_side_skirt",
    )
    card_body.visual(
        Box((0.020, 0.104, 0.004)),
        origin=Origin(xyz=(0.149, 0.0, 0.042), rpy=(0.0, -0.42, 0.0)),
        material=shroud_gray,
        name="nose_cap",
    )
    card_body.visual(
        Box((0.020, 0.104, 0.004)),
        origin=Origin(xyz=(-0.149, 0.0, 0.042), rpy=(0.0, 0.42, 0.0)),
        material=shroud_gray,
        name="bracket_end_cap",
    )
    card_body.visual(
        Box((0.040, 0.016, 0.014)),
        origin=Origin(xyz=(0.112, 0.053, 0.018)),
        material=shroud_black,
        name="power_connector_block",
    )

    for bearing_index, fan_x in enumerate(FAN_CENTERS):
        card_body.visual(
            Cylinder(radius=0.0105, length=0.004),
            origin=Origin(xyz=(fan_x, 0.0, 0.031)),
            material=fan_hub_gray,
            name=f"fan_{bearing_index}_bearing",
        )

    backplate = model.part("backplate")
    backplate.inertial = Inertial.from_geometry(
        Box((0.314, 0.126, 0.004)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    backplate.visual(
        Box((0.314, 0.126, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=backplate_black,
        name="backplate_panel",
    )
    backplate.visual(
        Box((0.220, 0.018, 0.002)),
        origin=Origin(xyz=(0.020, 0.0, -0.001)),
        material=backplate_black,
        name="backplate_center_rib",
    )
    backplate.visual(
        Box((0.246, 0.006, 0.002)),
        origin=Origin(xyz=(0.006, 0.045, -0.001)),
        material=backplate_black,
        name="backplate_upper_rib",
    )
    backplate.visual(
        Box((0.246, 0.006, 0.002)),
        origin=Origin(xyz=(0.006, -0.045, -0.001)),
        material=backplate_black,
        name="backplate_lower_rib",
    )
    backplate.visual(
        Box((0.050, 0.028, 0.002)),
        origin=Origin(xyz=(0.108, 0.0, -0.001)),
        material=backplate_black,
        name="backplate_tail_pad",
    )

    bracket = model.part("bracket")
    bracket.inertial = Inertial.from_geometry(
        Box((0.012, 0.118, 0.052)),
        mass=0.12,
        origin=Origin(xyz=(-0.004, 0.0, 0.026)),
    )
    bracket_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.050, 0.118, 0.003, corner_segments=6),
            [
                _translate_profile(rounded_rect_profile(0.015, 0.028, 0.0015, corner_segments=4), 0.010, 0.024),
                _translate_profile(rounded_rect_profile(0.015, 0.028, 0.0015, corner_segments=4), 0.010, -0.012),
                _translate_profile(rounded_rect_profile(0.010, 0.070, 0.0012, corner_segments=4), -0.013, 0.0),
            ],
            height=0.004,
            center=True,
        ).rotate_y(pi / 2.0),
        "gpu_bracket_plate",
    )
    bracket.visual(
        bracket_plate_mesh,
        origin=Origin(xyz=(-0.002, 0.0, 0.026)),
        material=bracket_silver,
        name="main_plate",
    )
    bracket.visual(
        Box((0.012, 0.016, 0.008)),
        origin=Origin(xyz=(-0.008, 0.055, 0.046)),
        material=bracket_silver,
        name="upper_tab",
    )
    bracket.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(-0.008, -0.054, 0.010)),
        material=bracket_silver,
        name="lower_lip",
    )
    bracket.visual(
        Box((0.006, 0.020, 0.018)),
        origin=Origin(xyz=(-0.005, -0.030, 0.024)),
        material=bracket_silver,
        name="slot_stiffener",
    )

    fan_parts = []
    for fan_index, fan_x in enumerate(FAN_CENTERS):
        fan_part = model.part(f"fan_{fan_index}_rotor")
        fan_part.inertial = Inertial.from_geometry(
            Cylinder(radius=FAN_RADIUS, length=0.008),
            mass=0.03,
            origin=Origin(),
        )
        _add_fan_rotor_visuals(
            fan_part,
            blade_count=9,
            blade_material=fan_black,
            hub_material=fan_hub_gray,
        )
        fan_parts.append((fan_part, fan_x))

    model.articulation(
        "body_to_backplate",
        ArticulationType.FIXED,
        parent=card_body,
        child=backplate,
        origin=Origin(),
    )
    model.articulation(
        "body_to_bracket",
        ArticulationType.FIXED,
        parent=card_body,
        child=bracket,
        origin=Origin(xyz=(-0.159, 0.0, 0.0)),
    )

    for fan_index, (fan_part, fan_x) in enumerate(fan_parts):
        model.articulation(
            f"fan_{fan_index}_spin",
            ArticulationType.CONTINUOUS,
            parent=card_body,
            child=fan_part,
            origin=Origin(xyz=(fan_x, 0.0, FAN_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=55.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card_body = object_model.get_part("card_body")
    backplate = object_model.get_part("backplate")
    bracket = object_model.get_part("bracket")
    fans = [object_model.get_part(f"fan_{index}_rotor") for index in range(3)]
    fan_joints = [object_model.get_articulation(f"fan_{index}_spin") for index in range(3)]

    def _aabb_size(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (max_x - min_x, max_y - min_y, max_z - min_z)

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
    ctx.fail_if_isolated_parts(max_pose_samples=6, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    body_size = _aabb_size(ctx.part_world_aabb(card_body))
    ctx.check("card_body_aabb_available", body_size is not None, details="card body AABB was unavailable")
    if body_size is not None:
        body_length, body_width, body_thickness = body_size
        ctx.check(
            "graphics_card_length_realistic",
            0.30 <= body_length <= 0.33,
            details=f"expected ~0.32 m length, got {body_length:.4f}",
        )
        ctx.check(
            "graphics_card_width_realistic",
            0.12 <= body_width <= 0.14,
            details=f"expected ~0.136 m width, got {body_width:.4f}",
        )
        ctx.check(
            "graphics_card_thickness_realistic",
            0.048 <= body_thickness <= 0.055,
            details=f"expected ~0.052 m thickness, got {body_thickness:.4f}",
        )

    ctx.expect_gap(card_body, backplate, axis="z", max_gap=0.001, max_penetration=0.0, name="backplate_seated_under_body")
    ctx.expect_overlap(backplate, card_body, axes="xy", min_overlap=0.12, name="backplate_covers_card_body")
    ctx.expect_gap(card_body, bracket, axis="x", max_gap=0.001, max_penetration=0.0, name="bracket_meets_card_end")
    ctx.expect_overlap(bracket, card_body, axes="yz", min_overlap=0.04, name="bracket_overlaps_body_profile")
    ctx.expect_origin_gap(card_body, bracket, axis="x", min_gap=0.15, max_gap=0.17, name="bracket_at_one_end")

    for fan_index, fan_joint in enumerate(fan_joints):
        ctx.check(
            f"fan_{fan_index}_joint_axis",
            fan_joint.axis == (0.0, 0.0, 1.0),
            details=f"expected axial z spin, got {fan_joint.axis}",
        )
        ctx.check(
            f"fan_{fan_index}_joint_type",
            fan_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"expected continuous spin articulation, got {fan_joint.articulation_type}",
        )

    fan_positions = [ctx.part_world_position(fan) for fan in fans]
    ctx.check(
        "fan_positions_available",
        all(position is not None for position in fan_positions),
        details=f"fan positions were {fan_positions}",
    )
    if all(position is not None for position in fan_positions):
        x_positions = [position[0] for position in fan_positions]
        y_positions = [position[1] for position in fan_positions]
        gaps = [x_positions[index + 1] - x_positions[index] for index in range(2)]
        ctx.check(
            "fan_even_spacing",
            abs(gaps[0] - gaps[1]) <= 0.004 and 0.098 <= gaps[0] <= 0.110,
            details=f"fan center gaps were {gaps}",
        )
        ctx.check(
            "fan_row_centered",
            max(abs(y) for y in y_positions) <= 0.002,
            details=f"fan y positions were {y_positions}",
        )

    with ctx.pose({fan_joints[0]: 0.0, fan_joints[1]: 0.7, fan_joints[2]: 1.4}):
        for fan_index, fan in enumerate(fans):
            ctx.expect_contact(
                fan,
                card_body,
                elem_a="hub",
                elem_b=f"fan_{fan_index}_bearing",
                name=f"fan_{fan_index}_hub_contacts_bearing",
            )
            ctx.expect_gap(
                card_body,
                fan,
                axis="z",
                positive_elem="shroud_top",
                min_gap=0.003,
                max_gap=0.012,
                name=f"fan_{fan_index}_stays_below_shroud_top",
            )
            ctx.expect_within(
                fan,
                card_body,
                axes="xy",
                margin=0.012,
                name=f"fan_{fan_index}_within_card_footprint",
            )
        ctx.fail_if_parts_overlap_in_current_pose(name="fan_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="fan_pose_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
