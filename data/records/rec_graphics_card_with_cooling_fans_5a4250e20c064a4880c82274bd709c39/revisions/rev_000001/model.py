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


CARD_LENGTH = 0.285
CARD_HEIGHT = 0.111
CARD_THICKNESS = 0.038
PCB_LENGTH = 0.266
PCB_HEIGHT = 0.099
PCB_THICKNESS = 0.0016
TOP_COVER_Z = 0.031
TOP_COVER_THICKNESS = 0.002


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((index * math.tau) / segments),
            cy + radius * math.sin((index * math.tau) / segments),
        )
        for index in range(segments)
    ]


def _make_ring_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height,
            center=True,
        ),
        name,
    )


def _make_shroud_cover_mesh():
    outer = rounded_rect_profile(CARD_LENGTH - 0.015, CARD_HEIGHT - 0.009, 0.007)
    holes = [
        _circle_profile(0.017, center=(0.092, -0.024)),
        _circle_profile(0.017, center=(0.092, 0.024)),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            holes,
            TOP_COVER_THICKNESS,
            center=True,
        ),
        "graphics_card_shroud_cover",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_graphics_card")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.23, 0.11, 1.0))
    solder_dark = model.material("solder_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.13, 0.14, 0.16, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.06, 0.07, 0.08, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.78, 0.79, 0.80, 1.0))
    gold = model.material("gold", rgba=(0.78, 0.63, 0.19, 1.0))

    housing_mesh = _make_ring_mesh(
        "blower_housing_ring",
        outer_radius=0.022,
        inner_radius=0.017,
        height=0.004,
    )
    impeller_mesh = _make_ring_mesh(
        "blower_impeller_ring",
        outer_radius=0.013,
        inner_radius=0.0055,
        height=0.008,
    )
    shroud_cover_mesh = _make_shroud_cover_mesh()

    pcb = model.part("pcb")
    pcb.visual(
        Box((PCB_LENGTH, PCB_HEIGHT, PCB_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PCB_THICKNESS / 2.0)),
        material=pcb_green,
        name="board_core",
    )
    pcb.visual(
        Box((0.085, 0.006, 0.0002)),
        origin=Origin(xyz=(-0.008, -0.043, PCB_THICKNESS + 0.0001)),
        material=gold,
        name="edge_connector",
    )
    pcb.visual(
        Box((0.034, 0.022, 0.0020)),
        origin=Origin(xyz=(0.018, 0.0, PCB_THICKNESS + 0.0010)),
        material=solder_dark,
        name="gpu_package",
    )
    pcb.inertial = Inertial.from_geometry(
        Box((PCB_LENGTH, PCB_HEIGHT, PCB_THICKNESS)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, PCB_THICKNESS / 2.0)),
    )

    heatsink = model.part("heatsink")
    heatsink.visual(
        Box((0.225, 0.074, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, PCB_THICKNESS + 0.003)),
        material=aluminum,
        name="base_block",
    )

    fin_y_positions = [(-0.024 + 0.0048 * index) for index in range(11)]
    for index, y_pos in enumerate(fin_y_positions):
        heatsink.visual(
            Box((0.186, 0.0014, 0.018)),
            origin=Origin(xyz=(-0.030, y_pos, PCB_THICKNESS + 0.006 + 0.009)),
            material=aluminum,
            name="fin_center" if index == 5 else f"fin_{index}",
        )

    heatsink.visual(
        Box((0.052, 0.064, 0.0136)),
        origin=Origin(xyz=(0.090, 0.0, PCB_THICKNESS + 0.006 + 0.0068)),
        material=aluminum,
        name="blower_plenum",
    )
    heatsink.visual(
        Cylinder(radius=0.003, length=0.0028),
        origin=Origin(xyz=(0.092, -0.024, 0.0226)),
        material=aluminum,
        name="left_bearing",
    )
    heatsink.visual(
        Cylinder(radius=0.003, length=0.0028),
        origin=Origin(xyz=(0.092, 0.024, 0.0226)),
        material=aluminum,
        name="right_bearing",
    )
    heatsink.inertial = Inertial.from_geometry(
        Box((0.225, 0.074, 0.025)),
        mass=0.75,
        origin=Origin(xyz=(-0.005, 0.0, 0.014)),
    )

    shroud = model.part("shroud")
    shroud.visual(
        shroud_cover_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOP_COVER_Z)),
        material=shroud_black,
        name="top_cover",
    )
    skirt_height = TOP_COVER_Z - PCB_THICKNESS
    skirt_center_z = PCB_THICKNESS + skirt_height / 2.0
    shroud.visual(
        Box((CARD_LENGTH - 0.015, 0.0025, skirt_height)),
        origin=Origin(xyz=(0.0, -0.04875, skirt_center_z)),
        material=shroud_black,
        name="left_skirt",
    )
    shroud.visual(
        Box((CARD_LENGTH - 0.015, 0.0025, skirt_height)),
        origin=Origin(xyz=(0.0, 0.04875, skirt_center_z)),
        material=shroud_black,
        name="right_skirt",
    )
    shroud.visual(
        Box((0.0025, CARD_HEIGHT - 0.009, skirt_height)),
        origin=Origin(xyz=(0.13375, 0.0, skirt_center_z)),
        material=shroud_black,
        name="far_end_cap",
    )
    shroud.visual(
        housing_mesh,
        origin=Origin(xyz=(0.092, -0.024, 0.034)),
        material=shroud_black,
        name="left_housing",
    )
    shroud.visual(
        housing_mesh,
        origin=Origin(xyz=(0.092, 0.024, 0.034)),
        material=shroud_black,
        name="right_housing",
    )
    shroud.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH - 0.015, CARD_HEIGHT - 0.009, 0.036)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.002, CARD_HEIGHT - 0.001, 0.004)),
        origin=Origin(xyz=(-0.143, 0.0, 0.036)),
        material=bracket_metal,
        name="top_rail",
    )
    bracket.visual(
        Box((0.002, CARD_HEIGHT - 0.001, 0.004)),
        origin=Origin(xyz=(-0.143, 0.0, 0.002)),
        material=bracket_metal,
        name="bottom_rail",
    )
    bracket.visual(
        Box((0.002, 0.006, 0.034)),
        origin=Origin(xyz=(-0.143, -0.0525, 0.019)),
        material=bracket_metal,
        name="lower_edge",
    )
    bracket.visual(
        Box((0.002, 0.006, 0.034)),
        origin=Origin(xyz=(-0.143, 0.0525, 0.019)),
        material=bracket_metal,
        name="upper_edge",
    )
    for index, y_pos in enumerate((-0.032, -0.010, 0.012, 0.034)):
        bracket.visual(
            Box((0.002, 0.004, 0.030)),
            origin=Origin(xyz=(-0.143, y_pos, 0.019)),
            material=bracket_metal,
            name=f"vent_bar_{index}",
        )
    bracket.visual(
        Box((0.015, 0.070, 0.002)),
        origin=Origin(xyz=(-0.1355, 0.0, -0.001)),
        material=bracket_metal,
        name="mount_flange",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.015, CARD_HEIGHT, 0.038)),
        mass=0.12,
        origin=Origin(xyz=(-0.136, 0.0, 0.018)),
    )

    left_rotor = model.part("left_rotor")
    left_rotor.visual(
        impeller_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rotor_black,
        name="impeller_ring",
    )
    left_rotor.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rotor_black,
        name="hub",
    )
    for index in range(6):
        angle = index * math.tau / 6.0
        left_rotor.visual(
            Box((0.0088, 0.0012, 0.008)),
            origin=Origin(
                xyz=(0.0083 * math.cos(angle), 0.0083 * math.sin(angle), 0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=rotor_black,
            name=f"blade_{index}",
        )
    left_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.010),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    right_rotor = model.part("right_rotor")
    right_rotor.visual(
        impeller_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rotor_black,
        name="impeller_ring",
    )
    right_rotor.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rotor_black,
        name="hub",
    )
    for index in range(6):
        angle = index * math.tau / 6.0 + (math.pi / 6.0)
        right_rotor.visual(
            Box((0.0088, 0.0012, 0.008)),
            origin=Origin(
                xyz=(0.0083 * math.cos(angle), 0.0083 * math.sin(angle), 0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=rotor_black,
            name=f"blade_{index}",
        )
    right_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.010),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation("pcb_to_heatsink", ArticulationType.FIXED, parent=pcb, child=heatsink)
    model.articulation("pcb_to_shroud", ArticulationType.FIXED, parent=pcb, child=shroud)
    model.articulation("pcb_to_bracket", ArticulationType.FIXED, parent=pcb, child=bracket)
    model.articulation(
        "heatsink_to_left_rotor",
        ArticulationType.CONTINUOUS,
        parent=heatsink,
        child=left_rotor,
        origin=Origin(xyz=(0.092, -0.024, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=120.0),
    )
    model.articulation(
        "heatsink_to_right_rotor",
        ArticulationType.CONTINUOUS,
        parent=heatsink,
        child=right_rotor,
        origin=Origin(xyz=(0.092, 0.024, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pcb = object_model.get_part("pcb")
    heatsink = object_model.get_part("heatsink")
    shroud = object_model.get_part("shroud")
    bracket = object_model.get_part("bracket")
    left_rotor = object_model.get_part("left_rotor")
    right_rotor = object_model.get_part("right_rotor")
    left_joint = object_model.get_articulation("heatsink_to_left_rotor")
    right_joint = object_model.get_articulation("heatsink_to_right_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(heatsink, pcb, elem_a="base_block", elem_b="board_core")
    ctx.expect_contact(shroud, pcb, elem_a="left_skirt", elem_b="board_core")
    ctx.expect_contact(shroud, pcb, elem_a="right_skirt", elem_b="board_core")
    ctx.expect_contact(bracket, pcb, elem_a="mount_flange", elem_b="board_core")
    ctx.expect_contact(left_rotor, heatsink, elem_a="hub", elem_b="left_bearing")
    ctx.expect_contact(right_rotor, heatsink, elem_a="hub", elem_b="right_bearing")

    ctx.expect_gap(
        shroud,
        heatsink,
        axis="z",
        positive_elem="top_cover",
        negative_elem="fin_center",
        min_gap=0.003,
        max_gap=0.006,
    )
    ctx.expect_overlap(
        left_rotor,
        shroud,
        axes="xy",
        elem_a="impeller_ring",
        elem_b="left_housing",
        min_overlap=0.024,
    )
    ctx.expect_overlap(
        right_rotor,
        shroud,
        axes="xy",
        elem_a="impeller_ring",
        elem_b="right_housing",
        min_overlap=0.024,
    )
    ctx.expect_origin_distance(
        left_rotor,
        right_rotor,
        axes="y",
        min_dist=0.045,
        max_dist=0.050,
    )

    left_pos = ctx.part_world_position(left_rotor)
    right_pos = ctx.part_world_position(right_rotor)
    bracket_aabb = ctx.part_element_world_aabb(bracket, elem="top_rail")
    assert left_pos is not None
    assert right_pos is not None
    assert bracket_aabb is not None
    bracket_center_x = 0.5 * (bracket_aabb[0][0] + bracket_aabb[1][0])

    ctx.check(
        "blower_positions_near_far_end",
        left_pos[0] > 0.07 and right_pos[0] > 0.07,
        details=f"left={left_pos}, right={right_pos}",
    )
    ctx.check(
        "blower_pair_symmetric_across_card_midline",
        left_pos[1] < 0.0
        and right_pos[1] > 0.0
        and abs(left_pos[0] - right_pos[0]) < 1e-6
        and abs(left_pos[1] + right_pos[1]) < 1e-6,
        details=f"left={left_pos}, right={right_pos}",
    )
    ctx.check(
        "blower_exhaust_direction_toward_bracket_end",
        bracket_center_x < left_pos[0] - 0.20,
        details=f"bracket_center_x={bracket_center_x}, left_rotor={left_pos}",
    )
    ctx.check(
        "blower_joints_are_continuous_and_axial",
        left_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(right_joint.axis) == (0.0, 0.0, 1.0)
        and left_joint.motion_limits is not None
        and right_joint.motion_limits is not None
        and left_joint.motion_limits.lower is None
        and left_joint.motion_limits.upper is None
        and right_joint.motion_limits.lower is None
        and right_joint.motion_limits.upper is None,
        details=(
            f"left_type={left_joint.articulation_type}, left_axis={left_joint.axis}, "
            f"right_type={right_joint.articulation_type}, right_axis={right_joint.axis}"
        ),
    )

    with ctx.pose({left_joint: 1.7, right_joint: -2.3}):
        ctx.expect_contact(left_rotor, heatsink, elem_a="hub", elem_b="left_bearing")
        ctx.expect_contact(right_rotor, heatsink, elem_a="hub", elem_b="right_bearing")
        ctx.expect_overlap(
            left_rotor,
            shroud,
            axes="xy",
            elem_a="impeller_ring",
            elem_b="left_housing",
            min_overlap=0.024,
        )
        ctx.expect_overlap(
            right_rotor,
            shroud,
            axes="xy",
            elem_a="impeller_ring",
            elem_b="right_housing",
            min_overlap=0.024,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
