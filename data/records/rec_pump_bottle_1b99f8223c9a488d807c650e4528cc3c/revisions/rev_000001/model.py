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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


BOTTLE_HEIGHT = 0.218
COLLAR_HEIGHT = 0.014
COLLAR_TOP_Z = BOTTLE_HEIGHT + COLLAR_HEIGHT
STEM_HEIGHT = 0.060
SWIVEL_ROTATION = 0.45
PUMP_STROKE = 0.012


def _oval_section(radius_x: float, radius_y: float, z: float, points: int = 36) -> list[tuple[float, float, float]]:
    return [
        (
            radius_x * math.cos((2.0 * math.pi * index) / points),
            radius_y * math.sin((2.0 * math.pi * index) / points),
            z,
        )
        for index in range(points)
    ]


def _build_bottle_mesh():
    return section_loft(
        [
            _oval_section(0.041, 0.038, 0.000),
            _oval_section(0.043, 0.040, 0.010),
            _oval_section(0.043, 0.040, 0.160),
            _oval_section(0.037, 0.034, 0.188),
            _oval_section(0.028, 0.025, 0.205),
            _oval_section(0.018, 0.016, BOTTLE_HEIGHT),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="salon_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.84, 0.76, 0.58, 1.0))
    collar_black = model.material("collar_black", rgba=(0.12, 0.12, 0.13, 1.0))
    head_black = model.material("head_black", rgba=(0.08, 0.08, 0.09, 1.0))
    stem_metal = model.material("stem_metal", rgba=(0.76, 0.79, 0.82, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        mesh_from_geometry(_build_bottle_mesh(), "bottle_shell"),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle_body.visual(
        Cylinder(radius=0.023, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_HEIGHT + (COLLAR_HEIGHT * 0.5))),
        material=collar_black,
        name="pump_collar",
    )
    bottle_body.visual(
        Cylinder(radius=0.005, length=STEM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z + (STEM_HEIGHT * 0.5))),
        material=stem_metal,
        name="plunger_stem",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.292),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
    )

    swivel_clip = model.part("swivel_clip")
    swivel_clip.visual(
        Box((0.010, 0.014, 0.004)),
        origin=Origin(xyz=(-0.012, 0.0, 0.002)),
        material=collar_black,
        name="left_clip_foot",
    )
    swivel_clip.visual(
        Box((0.010, 0.014, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, 0.002)),
        material=collar_black,
        name="right_clip_foot",
    )
    swivel_clip.visual(
        Box((0.002, 0.012, 0.030)),
        origin=Origin(xyz=(-0.009, 0.0, 0.019)),
        material=collar_black,
        name="left_clip_rail",
    )
    swivel_clip.visual(
        Box((0.002, 0.012, 0.030)),
        origin=Origin(xyz=(0.009, 0.0, 0.019)),
        material=collar_black,
        name="right_clip_rail",
    )
    swivel_clip.visual(
        Box((0.022, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.005, 0.028)),
        material=collar_black,
        name="clip_bridge",
    )
    swivel_clip.inertial = Inertial.from_geometry(
        Box((0.034, 0.020, 0.038)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    pump_head = model.part("pump_head")
    pump_head.visual(
        Box((0.004, 0.018, 0.048)),
        origin=Origin(xyz=(-0.012, 0.0, 0.024)),
        material=head_black,
        name="left_head_column",
    )
    pump_head.visual(
        Box((0.004, 0.018, 0.048)),
        origin=Origin(xyz=(0.012, 0.0, 0.024)),
        material=head_black,
        name="right_head_column",
    )
    pump_head.visual(
        Box((0.026, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, 0.040)),
        material=head_black,
        name="rear_head_bridge",
    )
    pump_head.visual(
        Box((0.026, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, 0.039)),
        material=head_black,
        name="front_head_bridge",
    )
    pump_head.visual(
        Box((0.018, 0.024, 0.012)),
        origin=Origin(xyz=(-0.020, 0.0, 0.054)),
        material=head_black,
        name="left_press_pad",
    )
    pump_head.visual(
        Box((0.032, 0.026, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, 0.054)),
        material=head_black,
        name="right_press_pad",
    )
    pump_head.visual(
        Box((0.016, 0.018, 0.016)),
        origin=Origin(xyz=(0.028, 0.0, 0.045)),
        material=head_black,
        name="spout_base",
    )
    pump_head.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.034, 0.0, 0.048),
                    (0.060, 0.0, 0.053),
                    (0.092, 0.0, 0.051),
                    (0.124, 0.0, 0.043),
                ],
                radius=0.0048,
                samples_per_segment=18,
                radial_segments=20,
            ),
            "pump_spout_tube",
        ),
        material=head_black,
        name="spout_tube",
    )
    pump_head.visual(
        Cylinder(radius=0.0032, length=0.016),
        origin=Origin(xyz=(0.132, 0.0, 0.042), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_black,
        name="spout_tip",
    )
    pump_head.inertial = Inertial.from_geometry(
        Box((0.146, 0.034, 0.066)),
        mass=0.10,
        origin=Origin(xyz=(0.039, 0.0, 0.033)),
    )

    model.articulation(
        "body_to_swivel_clip",
        ArticulationType.REVOLUTE,
        parent=bottle_body,
        child=swivel_clip,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.0,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "swivel_clip_to_pump_head",
        ArticulationType.PRISMATIC,
        parent=swivel_clip,
        child=pump_head,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.15,
            lower=0.0,
            upper=PUMP_STROKE,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    swivel_clip = object_model.get_part("swivel_clip")
    pump_head = object_model.get_part("pump_head")
    swivel_joint = object_model.get_articulation("body_to_swivel_clip")
    pump_joint = object_model.get_articulation("swivel_clip_to_pump_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "swivel_axis_is_vertical",
        swivel_joint.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical swivel axis, got {swivel_joint.axis}",
    )
    ctx.check(
        "pump_axis_is_vertical_downstroke",
        pump_joint.axis == (0.0, 0.0, -1.0),
        details=f"Expected downward prismatic axis, got {pump_joint.axis}",
    )

    ctx.expect_contact(
        swivel_clip,
        bottle_body,
        name="swivel_clip_is_seated_on_collar",
    )
    ctx.expect_contact(
        pump_head,
        swivel_clip,
        name="pump_head_stays_clipped_to_swivel",
    )
    ctx.expect_gap(
        pump_head,
        bottle_body,
        axis="z",
        min_gap=0.012,
        negative_elem="pump_collar",
        name="pump_head_clears_collar_at_rest",
    )

    rest_position = ctx.part_world_position(pump_head)
    assert rest_position is not None

    with ctx.pose({pump_joint: 0.010}):
        pressed_position = ctx.part_world_position(pump_head)
        assert pressed_position is not None
        ctx.check(
            "pump_head_moves_down_on_press",
            pressed_position[2] < rest_position[2] - 0.009,
            details=f"Rest z={rest_position[2]:.4f}, pressed z={pressed_position[2]:.4f}",
        )
        ctx.expect_contact(
            pump_head,
            swivel_clip,
            name="pump_head_remains_guided_when_pressed",
        )
        ctx.expect_gap(
            pump_head,
            bottle_body,
            axis="z",
            min_gap=0.001,
            negative_elem="pump_collar",
            name="pump_head_still_clears_collar_when_pressed",
        )

    with ctx.pose({swivel_joint: SWIVEL_ROTATION}):
        left_tip_aabb = ctx.part_element_world_aabb(pump_head, elem="spout_tip")
        assert left_tip_aabb is not None
        left_tip_center_y = 0.5 * (left_tip_aabb[0][1] + left_tip_aabb[1][1])
        ctx.expect_contact(
            swivel_clip,
            bottle_body,
            name="clip_stays_supported_when_swiveled_left",
        )

    with ctx.pose({swivel_joint: -SWIVEL_ROTATION}):
        right_tip_aabb = ctx.part_element_world_aabb(pump_head, elem="spout_tip")
        assert right_tip_aabb is not None
        right_tip_center_y = 0.5 * (right_tip_aabb[0][1] + right_tip_aabb[1][1])
        ctx.expect_contact(
            swivel_clip,
            bottle_body,
            name="clip_stays_supported_when_swiveled_right",
        )

    ctx.check(
        "spout_swivels_left_and_right_about_stem",
        left_tip_center_y > 0.040 and right_tip_center_y < -0.040,
        details=(
            f"Expected spout tip y to change sign across swivel poses, "
            f"got left={left_tip_center_y:.4f}, right={right_tip_center_y:.4f}"
        ),
    )

    with ctx.pose({swivel_joint: SWIVEL_ROTATION * 0.75, pump_joint: 0.010}):
        ctx.expect_contact(
            pump_head,
            swivel_clip,
            name="pump_head_attached_in_combined_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
