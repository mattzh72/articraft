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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


FRAME_LENGTH = 0.84
FRAME_WIDTH = 0.76
PANEL_LENGTH = 0.714
PANEL_WIDTH = 0.646
HINGE_X = -0.344
HINGE_Z = 0.0015
LEFT_RAIL_Y = -0.310
RIGHT_RAIL_Y = 0.310
RAIL_CLOSED_X = 0.245


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_rear_carriage(part, *, inboard_sign: float, dark_plastic, roller_steel) -> None:
    part.visual(
        Box((0.082, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_plastic,
        name="shoe_body",
    )
    part.visual(
        Box((0.016, 0.010, 0.016)),
        origin=Origin(xyz=(0.018, 0.0, 0.008)),
        material=dark_plastic,
        name="shoe_riser",
    )
    part.visual(
        Box((0.028, 0.034, 0.008)),
        origin=Origin(xyz=(0.032, 0.018 * inboard_sign, 0.014)),
        material=dark_plastic,
        name="shoe_arm",
    )
    part.visual(
        Box((0.042, 0.012, 0.006)),
        origin=Origin(xyz=(0.042, 0.036 * inboard_sign, 0.018)),
        material=dark_plastic,
        name="shoe_head",
    )
    for index, wheel_x in enumerate((-0.022, 0.022)):
        part.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(xyz=(wheel_x, 0.0, -0.010), rpy=(pi / 2.0, 0.0, 0.0)),
            material=roller_steel,
            name=f"roller_{index}",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.100, 0.090, 0.036)),
        mass=0.40,
        origin=Origin(xyz=(0.024, 0.018 * inboard_sign, 0.010)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spoiler_sunroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    rail_black = model.material("rail_black", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    roller_steel = model.material("roller_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.22, 0.33, 0.37, 0.42))
    ceramic_black = model.material("ceramic_black", rgba=(0.07, 0.07, 0.08, 0.92))

    cassette_frame = model.part("cassette_frame")
    cassette_frame.inertial = Inertial.from_geometry(
        Box((FRAME_LENGTH, FRAME_WIDTH, 0.11)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    top_ring = _mesh(
        "sunroof_cassette_ring",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(FRAME_LENGTH, FRAME_WIDTH, 0.045, corner_segments=10),
            [rounded_rect_profile(0.742, 0.672, 0.030, corner_segments=10)],
            height=0.018,
            center=True,
        ),
    )
    cassette_frame.visual(
        top_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=aluminum,
        name="cassette_ring",
    )
    cassette_frame.visual(
        Box((0.060, FRAME_WIDTH, 0.050)),
        origin=Origin(xyz=(-0.390, 0.0, -0.025)),
        material=aluminum,
        name="front_rail",
    )
    cassette_frame.visual(
        Box((0.060, FRAME_WIDTH, 0.056)),
        origin=Origin(xyz=(0.390, 0.0, -0.028)),
        material=aluminum,
        name="rear_gutter",
    )

    for side_name, rail_y, side_sign in (
        ("left", LEFT_RAIL_Y, -1.0),
        ("right", RIGHT_RAIL_Y, 1.0),
    ):
        cassette_frame.visual(
            Box((0.720, 0.016, 0.052)),
            origin=Origin(xyz=(0.030, rail_y + 0.021 * side_sign, -0.026)),
            material=aluminum,
            name=f"{side_name}_outer_wall",
        )
        cassette_frame.visual(
            Box((0.720, 0.014, 0.020)),
            origin=Origin(xyz=(0.030, rail_y - 0.021 * side_sign, -0.044)),
            material=aluminum,
            name=f"{side_name}_inner_wall",
        )
        cassette_frame.visual(
            Box((0.720, 0.036, 0.006)),
            origin=Origin(xyz=(0.030, rail_y, -0.051)),
            material=rail_black,
            name=f"{side_name}_rail_floor",
        )
        cassette_frame.visual(
            Box((0.600, 0.014, 0.005)),
            origin=Origin(xyz=(-0.010, rail_y - 0.008 * side_sign, 0.0015)),
            material=seal_rubber,
            name=f"{side_name}_seal",
        )
        cassette_frame.visual(
            Box((0.500, 0.018, 0.034)),
            origin=Origin(xyz=(-0.060, rail_y - 0.014 * side_sign, -0.017)),
            material=dark_plastic,
            name=f"{side_name}_seal_carrier",
        )

    cassette_frame.visual(
        Box((0.045, 0.610, 0.005)),
        origin=Origin(xyz=(-0.332, 0.0, 0.0015)),
        material=seal_rubber,
        name="front_seal",
    )
    cassette_frame.visual(
        Box((0.045, 0.610, 0.005)),
        origin=Origin(xyz=(0.332, 0.0, 0.0015)),
        material=seal_rubber,
        name="rear_seal",
    )
    cassette_frame.visual(
        Box((0.040, 0.610, 0.034)),
        origin=Origin(xyz=(-0.370, 0.0, -0.017)),
        material=dark_plastic,
        name="front_seal_carrier",
    )
    cassette_frame.visual(
        Box((0.058, 0.610, 0.034)),
        origin=Origin(xyz=(0.361, 0.0, -0.017)),
        material=dark_plastic,
        name="rear_seal_carrier",
    )
    cassette_frame.visual(
        Box((0.180, 0.170, 0.045)),
        origin=Origin(xyz=(-0.300, 0.0, -0.058)),
        material=dark_plastic,
        name="motor_cover",
    )
    cassette_frame.visual(
        Box((0.090, 0.620, 0.036)),
        origin=Origin(xyz=(0.398, 0.0, -0.060)),
        material=rail_black,
        name="drain_trough",
    )

    for side_name, side_y in (("left", -0.270), ("right", 0.270)):
        cassette_frame.visual(
            Cylinder(radius=0.0065, length=0.100),
            origin=Origin(xyz=(HINGE_X, side_y, -0.026), rpy=(pi / 2.0, 0.0, 0.0)),
            material=roller_steel,
            name=f"{side_name}_hinge_sleeve",
        )
        cassette_frame.visual(
            Box((0.024, 0.050, 0.018)),
            origin=Origin(xyz=(HINGE_X + 0.004, side_y, -0.034)),
            material=rail_black,
            name=f"{side_name}_hinge_bracket",
        )

    glass_panel = model.part("glass_panel")
    glass_panel.inertial = Inertial.from_geometry(
        Box((0.730, 0.655, 0.040)),
        mass=8.0,
        origin=Origin(xyz=(0.340, 0.0, 0.002)),
    )

    glass_lite = _mesh(
        "sunroof_glass_lite",
        ExtrudeGeometry(
            rounded_rect_profile(PANEL_LENGTH, PANEL_WIDTH, 0.032, corner_segments=12),
            height=0.0055,
            center=True,
        ),
    )
    glass_panel.visual(
        glass_lite,
        origin=Origin(xyz=(0.337, 0.0, 0.0018)),
        material=glass_tint,
        name="glass_lite",
    )
    ceramic_border_mesh = _mesh(
        "sunroof_ceramic_border",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.712, 0.644, 0.032, corner_segments=12),
            [rounded_rect_profile(0.628, 0.560, 0.025, corner_segments=12)],
            height=0.0012,
            center=True,
        ),
    )
    glass_panel.visual(
        ceramic_border_mesh,
        origin=Origin(xyz=(0.337, 0.0, 0.0039)),
        material=ceramic_black,
        name="ceramic_border",
    )
    glass_panel.visual(
        Cylinder(radius=0.0055, length=0.420),
        origin=Origin(xyz=(0.000, 0.000, -0.026), rpy=(pi / 2.0, 0.0, 0.0)),
        material=roller_steel,
        name="hinge_knuckle",
    )
    glass_panel.visual(
        Box((0.220, 0.536, 0.008)),
        origin=Origin(xyz=(0.120, 0.0, -0.0035)),
        material=rail_black,
        name="front_carrier_bar",
    )
    for side_name, side_y in (("left", -0.155), ("right", 0.155)):
        glass_panel.visual(
            Box((0.024, 0.070, 0.024)),
            origin=Origin(xyz=(0.012, side_y, -0.015)),
            material=rail_black,
            name=f"{side_name}_hinge_strap",
        )
    for side_name, side_y in (("left", -0.276), ("right", 0.276)):
        glass_panel.visual(
            Box((0.370, 0.016, 0.008)),
            origin=Origin(xyz=(0.410, side_y, -0.006)),
            material=rail_black,
            name=f"{side_name}_carrier_rail",
        )
        glass_panel.visual(
            Box((0.050, 0.016, 0.008)),
            origin=Origin(xyz=(0.620, side_y, -0.006)),
            material=rail_black,
            name=f"{side_name}_receiver_pad",
        )
    glass_panel.visual(
        Box((0.028, 0.540, 0.008)),
        origin=Origin(xyz=(0.680, 0.0, 0.000)),
        material=ceramic_black,
        name="rear_spoiler_lip",
    )

    left_rear_carriage = model.part("left_rear_carriage")
    _add_rear_carriage(
        left_rear_carriage,
        inboard_sign=1.0,
        dark_plastic=dark_plastic,
        roller_steel=roller_steel,
    )

    right_rear_carriage = model.part("right_rear_carriage")
    _add_rear_carriage(
        right_rear_carriage,
        inboard_sign=-1.0,
        dark_plastic=dark_plastic,
        roller_steel=roller_steel,
    )

    model.articulation(
        "panel_front_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette_frame,
        child=glass_panel,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=0.0, upper=0.32),
    )
    model.articulation(
        "left_rear_slide",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=left_rear_carriage,
        origin=Origin(xyz=(RAIL_CLOSED_X, LEFT_RAIL_Y, -0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.18, lower=0.0, upper=0.090),
    )
    model.articulation(
        "right_rear_slide",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=right_rear_carriage,
        origin=Origin(xyz=(RAIL_CLOSED_X, RIGHT_RAIL_Y, -0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.18, lower=0.0, upper=0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette_frame = object_model.get_part("cassette_frame")
    glass_panel = object_model.get_part("glass_panel")
    left_rear_carriage = object_model.get_part("left_rear_carriage")
    right_rear_carriage = object_model.get_part("right_rear_carriage")
    panel_front_hinge = object_model.get_articulation("panel_front_hinge")
    left_rear_slide = object_model.get_articulation("left_rear_slide")
    right_rear_slide = object_model.get_articulation("right_rear_slide")

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
        "front_hinge_axis_is_transverse",
        tuple(panel_front_hinge.axis) == (0.0, -1.0, 0.0),
        f"expected hinge axis (0, -1, 0), got {panel_front_hinge.axis}",
    )
    ctx.check(
        "rear_slides_run_longitudinally",
        tuple(left_rear_slide.axis) == (1.0, 0.0, 0.0) and tuple(right_rear_slide.axis) == (1.0, 0.0, 0.0),
        f"slide axes are left={left_rear_slide.axis}, right={right_rear_slide.axis}",
    )

    ctx.expect_gap(
        glass_panel,
        cassette_frame,
        axis="z",
        positive_elem="glass_lite",
        negative_elem="cassette_ring",
        max_gap=0.004,
        max_penetration=0.0,
        name="glass_panel_sits_flush_over_frame",
    )
    ctx.expect_overlap(
        glass_panel,
        cassette_frame,
        axes="xy",
        elem_a="glass_lite",
        elem_b="cassette_ring",
        min_overlap=0.60,
        name="glass_panel_covers_cassette_opening",
    )
    ctx.expect_contact(
        left_rear_carriage,
        cassette_frame,
        contact_tol=0.0015,
        name="left_carriage_is_supported_by_left_rail",
    )
    ctx.expect_contact(
        right_rear_carriage,
        cassette_frame,
        contact_tol=0.0015,
        name="right_carriage_is_supported_by_right_rail",
    )

    closed_left = ctx.part_world_position(left_rear_carriage)
    closed_right = ctx.part_world_position(right_rear_carriage)
    closed_glass_aabb = ctx.part_element_world_aabb(glass_panel, elem="glass_lite")

    with ctx.pose({panel_front_hinge: 0.28, left_rear_slide: 0.075, right_rear_slide: 0.075}):
        open_left = ctx.part_world_position(left_rear_carriage)
        open_right = ctx.part_world_position(right_rear_carriage)
        open_glass_aabb = ctx.part_element_world_aabb(glass_panel, elem="glass_lite")

        ctx.check(
            "rear_carriages_slide_rearward_in_open_pose",
            closed_left is not None
            and open_left is not None
            and closed_right is not None
            and open_right is not None
            and open_left[0] > closed_left[0] + 0.060
            and open_right[0] > closed_right[0] + 0.060,
            (
                f"closed left/right x={closed_left[0] if closed_left else None}/"
                f"{closed_right[0] if closed_right else None}, "
                f"open left/right x={open_left[0] if open_left else None}/"
                f"{open_right[0] if open_right else None}"
            ),
        )
        ctx.check(
            "glass_panel_lifts_rearward_for_spoiler_pose",
            closed_glass_aabb is not None
            and open_glass_aabb is not None
            and open_glass_aabb[1][2] > closed_glass_aabb[1][2] + 0.12,
            (
                f"closed glass max z={closed_glass_aabb[1][2] if closed_glass_aabb else None}, "
                f"open glass max z={open_glass_aabb[1][2] if open_glass_aabb else None}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
