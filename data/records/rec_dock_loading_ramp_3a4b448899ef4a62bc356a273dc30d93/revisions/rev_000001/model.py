from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _knuckle_centers(count: int, segment_length: float) -> list[float]:
    return [((i + 0.5) - count / 2.0) * segment_length for i in range(count)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_leveling_dock_leveler")

    frame_steel = model.material("frame_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    lip_yellow = model.material("lip_yellow", rgba=(0.72, 0.61, 0.16, 1.0))
    leg_steel = model.material("leg_steel", rgba=(0.53, 0.55, 0.57, 1.0))

    width = 2.15
    platform_length = 2.25
    lip_length = 0.43
    pit_depth = 0.93
    floor_thickness = 0.05
    opening_width = width + 0.08
    wall_thickness = 0.14
    pit_back = -0.16
    pit_front = platform_length + 0.18

    rear_axis_drop = 0.055
    deck_top_z = rear_axis_drop
    deck_thickness = 0.016
    deck_plate_center_z = deck_top_z - deck_thickness / 2.0
    plate_bottom_z = deck_plate_center_z - deck_thickness / 2.0

    side_beam_height = 0.14
    side_beam_center_z = plate_bottom_z - side_beam_height / 2.0
    crossmember_height = 0.08
    crossmember_center_z = plate_bottom_z - crossmember_height / 2.0
    rear_spine_height = plate_bottom_z - (-rear_axis_drop)
    rear_spine_center_z = (plate_bottom_z + (-rear_axis_drop)) / 2.0
    front_header_height = 0.14
    front_header_center_z = plate_bottom_z - front_header_height / 2.0

    rear_hinge_radius = 0.038
    rear_knuckle_length = 0.335
    rear_knuckle_centers = _knuckle_centers(6, rear_knuckle_length)

    lip_hinge_x = platform_length - 0.022
    lip_axis_z = 0.016
    lip_width = width - 0.14

    guide_outer = 0.10
    guide_plate = 0.02
    guide_inner = guide_outer - 2.0 * guide_plate
    guide_length = 0.32
    guide_pad_thickness = 0.02
    guide_pad_center_z = plate_bottom_z - guide_pad_thickness / 2.0
    guide_top_z = guide_pad_center_z - guide_pad_thickness / 2.0
    guide_center_z = guide_top_z - guide_length / 2.0
    leg_x = 1.58
    leg_y = 0.72

    leg_column_size = 0.052
    leg_length = 0.74
    leg_shoulder_thickness = 0.02
    leg_shoulder_size = 0.12
    foot_thickness = 0.02
    foot_size = (0.18, 0.12, foot_thickness)
    leg_travel = 0.104

    pit_frame = model.part("pit_frame")
    pit_frame.visual(
        Box((pit_front - pit_back, width + 0.34, floor_thickness)),
        origin=Origin(
            xyz=((pit_front + pit_back) / 2.0, 0.0, -pit_depth + floor_thickness / 2.0)
        ),
        material=frame_steel,
        name="pit_floor",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        pit_frame.visual(
            Box((pit_front - pit_back, wall_thickness, pit_depth)),
            origin=Origin(
                xyz=(
                    (pit_front + pit_back) / 2.0,
                    side_sign * (opening_width / 2.0 + wall_thickness / 2.0),
                    -pit_depth / 2.0,
                )
            ),
            material=frame_steel,
            name=f"{side_name}_pit_wall",
        )
    pit_frame.visual(
        Box((0.14, width + 0.28, 0.24)),
        origin=Origin(xyz=(-0.13, 0.0, -0.12)),
        material=frame_steel,
        name="rear_pit_beam",
    )
    pit_frame.visual(
        Box((0.08, width - 0.18, 0.10)),
        origin=Origin(xyz=(-0.078, 0.0, -rear_axis_drop)),
        material=frame_steel,
        name="rear_hinge_spine",
    )
    pit_frame.visual(
        Box((0.14, width + 0.28, pit_depth - 0.20)),
        origin=Origin(xyz=(platform_length + 0.11, 0.0, -(pit_depth + 0.20) / 2.0)),
        material=frame_steel,
        name="front_pit_wall",
    )
    for idx, center_y in enumerate(rear_knuckle_centers):
        if idx % 2 == 0:
            pit_frame.visual(
                Cylinder(radius=rear_hinge_radius, length=rear_knuckle_length),
                origin=Origin(xyz=(0.0, center_y, -rear_axis_drop), rpy=(pi / 2.0, 0.0, 0.0)),
                material=frame_steel,
                name=f"frame_knuckle_{idx}",
            )
    pit_frame.inertial = Inertial.from_geometry(
        Box((pit_front - pit_back, width + 0.34, pit_depth)),
        mass=520.0,
        origin=Origin(xyz=((pit_front + pit_back) / 2.0, 0.0, -pit_depth / 2.0)),
    )

    platform = model.part("platform")
    platform.visual(
        Box((platform_length, width, deck_thickness)),
        origin=Origin(xyz=(platform_length / 2.0, 0.0, deck_plate_center_z)),
        material=deck_steel,
        name="deck_plate",
    )
    platform.visual(
        Box((platform_length - 0.16, 0.08, side_beam_height)),
        origin=Origin(
            xyz=(platform_length / 2.0, width / 2.0 - 0.04, side_beam_center_z)
        ),
        material=deck_steel,
        name="left_side_beam",
    )
    platform.visual(
        Box((platform_length - 0.16, 0.08, side_beam_height)),
        origin=Origin(
            xyz=(platform_length / 2.0, -(width / 2.0 - 0.04), side_beam_center_z)
        ),
        material=deck_steel,
        name="right_side_beam",
    )
    platform.visual(
        Box((0.08, width - 0.18, rear_spine_height)),
        origin=Origin(xyz=(0.078, 0.0, rear_spine_center_z)),
        material=deck_steel,
        name="rear_spine",
    )
    platform.visual(
        Box((0.08, width - 0.02, front_header_height)),
        origin=Origin(xyz=(platform_length - 0.062, 0.0, front_header_center_z)),
        material=deck_steel,
        name="front_header",
    )
    platform.visual(
        Box((0.06, lip_width - 0.22, 0.05)),
        origin=Origin(xyz=(lip_hinge_x - 0.03, 0.0, lip_axis_z - 0.035)),
        material=deck_steel,
        name="lip_hinge_shelf",
    )
    platform.visual(
        Box((0.07, 0.05, 0.09)),
        origin=Origin(
            xyz=(lip_hinge_x - 0.028, width / 2.0 - 0.045, lip_axis_z - 0.029)
        ),
        material=deck_steel,
        name="left_lip_bracket",
    )
    platform.visual(
        Box((0.07, 0.05, 0.09)),
        origin=Origin(
            xyz=(lip_hinge_x - 0.028, -(width / 2.0 - 0.045), lip_axis_z - 0.029)
        ),
        material=deck_steel,
        name="right_lip_bracket",
    )
    for idx, cross_x in enumerate((0.62, 1.18, 1.74)):
        platform.visual(
            Box((0.10, width - 0.24, crossmember_height)),
            origin=Origin(xyz=(cross_x, 0.0, crossmember_center_z)),
            material=deck_steel,
            name=f"crossmember_{idx}",
        )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        guide_y = side_sign * leg_y
        platform.visual(
            Box((0.18, 0.18, guide_pad_thickness)),
            origin=Origin(xyz=(leg_x, guide_y, guide_pad_center_z)),
            material=deck_steel,
            name=f"{side_name}_guide_pad",
        )
        platform.visual(
            Box((guide_plate, guide_inner, guide_length)),
            origin=Origin(
                xyz=(leg_x - (guide_inner / 2.0 + guide_plate / 2.0), guide_y, guide_center_z)
            ),
            material=deck_steel,
            name=f"{side_name}_guide_left_plate",
        )
        platform.visual(
            Box((guide_plate, guide_inner, guide_length)),
            origin=Origin(
                xyz=(leg_x + (guide_inner / 2.0 + guide_plate / 2.0), guide_y, guide_center_z)
            ),
            material=deck_steel,
            name=f"{side_name}_guide_right_plate",
        )
        platform.visual(
            Box((guide_inner, guide_plate, guide_length)),
            origin=Origin(
                xyz=(leg_x, guide_y - (guide_inner / 2.0 + guide_plate / 2.0), guide_center_z)
            ),
            material=deck_steel,
            name=f"{side_name}_guide_front_plate",
        )
        platform.visual(
            Box((guide_inner, guide_plate, guide_length)),
            origin=Origin(
                xyz=(leg_x, guide_y + (guide_inner / 2.0 + guide_plate / 2.0), guide_center_z)
            ),
            material=deck_steel,
            name=f"{side_name}_guide_back_plate",
        )
    for idx, center_y in enumerate(rear_knuckle_centers):
        if idx % 2 == 1:
            platform.visual(
                Cylinder(radius=rear_hinge_radius, length=rear_knuckle_length),
                origin=Origin(xyz=(0.0, center_y, -rear_axis_drop), rpy=(pi / 2.0, 0.0, 0.0)),
                material=deck_steel,
                name=f"rear_knuckle_{idx}",
            )
    platform.inertial = Inertial.from_geometry(
        Box((platform_length, width, 0.18)),
        mass=290.0,
        origin=Origin(xyz=(platform_length / 2.0, 0.0, -0.02)),
    )

    lip = model.part("lip")
    lip.visual(
        Box((0.46, lip_width, 0.02)),
        origin=Origin(xyz=(0.25, 0.0, -0.055)),
        material=lip_yellow,
        name="lip_plate",
    )
    lip.visual(
        Box((0.06, lip_width - 0.24, 0.06)),
        origin=Origin(xyz=(0.03, 0.0, -0.046)),
        material=lip_yellow,
        name="lip_rear_stiffener",
    )
    lip.visual(
        Box((0.24, 0.12, 0.055)),
        origin=Origin(xyz=(0.16, 0.46, -0.048)),
        material=lip_yellow,
        name="lip_left_rib",
    )
    lip.visual(
        Box((0.24, 0.12, 0.055)),
        origin=Origin(xyz=(0.16, -0.46, -0.048)),
        material=lip_yellow,
        name="lip_right_rib",
    )
    lip.visual(
        Box((0.08, lip_width - 0.10, 0.03)),
        origin=Origin(xyz=(0.42, 0.0, -0.066)),
        material=lip_yellow,
        name="lip_nose",
    )
    lip.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(0.04, lip_width / 2.0 - 0.10, -0.04)),
        material=lip_yellow,
        name="lip_left_hinge_leaf",
    )
    lip.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(0.04, -(lip_width / 2.0 - 0.10), -0.04)),
        material=lip_yellow,
        name="lip_right_hinge_leaf",
    )
    lip.inertial = Inertial.from_geometry(
        Box((0.46, lip_width, 0.10)),
        mass=48.0,
        origin=Origin(xyz=(0.23, 0.0, -0.05)),
    )

    left_leg = model.part("left_leg")
    left_leg.visual(
        Box((leg_column_size, leg_column_size, leg_length)),
        origin=Origin(xyz=(0.0, 0.0, -leg_length / 2.0)),
        material=leg_steel,
        name="left_column",
    )
    left_leg.visual(
        Box((leg_shoulder_size, leg_shoulder_size, leg_shoulder_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -(guide_length + leg_shoulder_thickness / 2.0))),
        material=leg_steel,
        name="left_shoulder",
    )
    left_leg.visual(
        Box(foot_size),
        origin=Origin(xyz=(0.0, 0.0, -(leg_length + foot_thickness / 2.0))),
        material=leg_steel,
        name="left_foot",
    )
    left_leg.inertial = Inertial.from_geometry(
        Box((leg_shoulder_size, leg_shoulder_size, leg_length + foot_thickness)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -(leg_length + foot_thickness) / 2.0)),
    )

    right_leg = model.part("right_leg")
    right_leg.visual(
        Box((leg_column_size, leg_column_size, leg_length)),
        origin=Origin(xyz=(0.0, 0.0, -leg_length / 2.0)),
        material=leg_steel,
        name="right_column",
    )
    right_leg.visual(
        Box((leg_shoulder_size, leg_shoulder_size, leg_shoulder_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -(guide_length + leg_shoulder_thickness / 2.0))),
        material=leg_steel,
        name="right_shoulder",
    )
    right_leg.visual(
        Box(foot_size),
        origin=Origin(xyz=(0.0, 0.0, -(leg_length + foot_thickness / 2.0))),
        material=leg_steel,
        name="right_foot",
    )
    right_leg.inertial = Inertial.from_geometry(
        Box((leg_shoulder_size, leg_shoulder_size, leg_length + foot_thickness)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -(leg_length + foot_thickness) / 2.0)),
    )

    platform_hinge = model.articulation(
        "pit_to_platform",
        ArticulationType.REVOLUTE,
        parent=pit_frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, -rear_axis_drop)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.75,
            lower=0.0,
            upper=0.48,
        ),
    )
    model.articulation(
        "platform_to_lip",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(xyz=(lip_hinge_x, 0.0, lip_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4200.0,
            velocity=1.2,
            lower=0.0,
            upper=0.65,
        ),
    )
    model.articulation(
        "platform_to_left_leg",
        ArticulationType.PRISMATIC,
        parent=platform,
        child=left_leg,
        origin=Origin(xyz=(leg_x, leg_y, guide_top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.18,
            lower=0.0,
            upper=leg_travel,
        ),
    )
    model.articulation(
        "platform_to_right_leg",
        ArticulationType.PRISMATIC,
        parent=platform,
        child=right_leg,
        origin=Origin(xyz=(leg_x, -leg_y, guide_top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.18,
            lower=0.0,
            upper=leg_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pit_frame = object_model.get_part("pit_frame")
    platform = object_model.get_part("platform")
    lip = object_model.get_part("lip")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    platform_hinge = object_model.get_articulation("pit_to_platform")
    lip_hinge = object_model.get_articulation("platform_to_lip")
    left_leg_slider = object_model.get_articulation("platform_to_left_leg")
    right_leg_slider = object_model.get_articulation("platform_to_right_leg")

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

    ctx.expect_contact(
        platform,
        pit_frame,
        name="platform is physically hinge-mounted to the pit frame",
    )
    ctx.expect_contact(
        lip,
        platform,
        name="lip is physically hinge-mounted to the platform",
    )
    ctx.expect_contact(
        left_leg,
        platform,
        name="left support leg is captured by its guide",
    )
    ctx.expect_contact(
        right_leg,
        platform,
        name="right support leg is captured by its guide",
    )

    rest_front = ctx.part_element_world_aabb(platform, elem="front_header")
    with ctx.pose({platform_hinge: platform_hinge.motion_limits.upper}):
        raised_front = ctx.part_element_world_aabb(platform, elem="front_header")
    ctx.check(
        "platform front edge rises when the deck is raised",
        rest_front is not None
        and raised_front is not None
        and raised_front[0][2] > rest_front[0][2] + 0.45,
        details=f"rest_front={rest_front}, raised_front={raised_front}",
    )

    with ctx.pose(
        {
            platform_hinge: 0.28,
            lip_hinge: lip_hinge.motion_limits.upper,
        }
    ):
        deployed_lip = ctx.part_element_world_aabb(lip, elem="lip_plate")
        deployed_front = ctx.part_element_world_aabb(platform, elem="front_header")
    ctx.check(
        "lip deploys forward and below the deck for truck-bed contact",
        deployed_lip is not None
        and deployed_front is not None
        and deployed_lip[1][0] > deployed_front[1][0] + 0.20
        and deployed_lip[0][2] < deployed_front[0][2] - 0.08,
        details=f"deployed_lip={deployed_lip}, deployed_front={deployed_front}",
    )

    left_leg_rest = ctx.part_world_position(left_leg)
    right_leg_rest = ctx.part_world_position(right_leg)
    with ctx.pose(
        {
            left_leg_slider: left_leg_slider.motion_limits.upper,
            right_leg_slider: right_leg_slider.motion_limits.upper,
        }
    ):
        left_leg_extended = ctx.part_world_position(left_leg)
        right_leg_extended = ctx.part_world_position(right_leg)
        ctx.expect_contact(
            left_leg,
            pit_frame,
            elem_a="left_foot",
            elem_b="pit_floor",
            name="left support foot can land on the pit floor",
        )
        ctx.expect_contact(
            right_leg,
            pit_frame,
            elem_a="right_foot",
            elem_b="pit_floor",
            name="right support foot can land on the pit floor",
        )
    ctx.check(
        "support legs extend downward on their prismatic joints",
        left_leg_rest is not None
        and right_leg_rest is not None
        and left_leg_extended is not None
        and right_leg_extended is not None
        and left_leg_extended[2] < left_leg_rest[2] - 0.08
        and right_leg_extended[2] < right_leg_rest[2] - 0.08,
        details=(
            f"left_rest={left_leg_rest}, left_extended={left_leg_extended}, "
            f"right_rest={right_leg_rest}, right_extended={right_leg_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
