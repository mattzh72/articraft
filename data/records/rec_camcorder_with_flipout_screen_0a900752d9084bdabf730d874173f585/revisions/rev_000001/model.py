import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camcorder")

    mat_body = Material(name="body_mat", rgba=(0.2, 0.2, 0.2, 1.0))
    mat_lens = Material(name="lens_mat", rgba=(0.1, 0.1, 0.1, 1.0))
    mat_screen = Material(name="screen_mat", rgba=(0.05, 0.05, 0.1, 1.0))
    mat_button = Material(name="button_mat", rgba=(0.8, 0.1, 0.1, 1.0))
    mat_metal = Material(name="metal_mat", rgba=(0.8, 0.8, 0.8, 1.0))

    # --- BODY ---
    body = model.part("body")
    # Main box
    body.visual(
        Box((0.24, 0.11, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mat_body,
        name="main_body",
    )
    # Handle
    body.visual(
        Box((0.03, 0.03, 0.04)),
        origin=Origin(xyz=(0.08, 0.0, 0.08)),
        material=mat_body,
        name="handle_front_strut",
    )
    body.visual(
        Box((0.03, 0.03, 0.04)),
        origin=Origin(xyz=(-0.08, 0.0, 0.08)),
        material=mat_body,
        name="handle_rear_strut",
    )
    body.visual(
        Box((0.20, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=mat_body,
        name="handle_top_bar",
    )
    # Eyepiece
    body.visual(
        Box((0.04, 0.04, 0.04)),
        origin=Origin(xyz=(-0.14, 0.0, 0.04)),
        material=mat_body,
        name="eyepiece_base",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.03),
        origin=Origin(xyz=(-0.175, 0.0, 0.04), rpy=(0.0, math.pi / 2, 0.0)),
        material=mat_body,
        name="eyepiece_cup",
    )
    # Lens barrel base
    body.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(xyz=(0.17, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=mat_lens,
        name="lens_barrel",
    )
    # LCD Hinge mount
    body.visual(
        Cylinder(radius=0.012, length=0.04),
        origin=Origin(xyz=(0.06, -0.055, 0.0)),
        material=mat_body,
        name="lcd_hinge_mount",
    )
    # Media door hinge mount
    body.visual(
        Box((0.01, 0.01, 0.06)),
        origin=Origin(xyz=(-0.05, 0.055, 0.0)),
        material=mat_body,
        name="media_door_mount",
    )

    # --- FOCUS RING ---
    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        Cylinder(radius=0.042, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=mat_metal,
        name="focus_ring_visual",
    )
    model.articulation(
        name="focus_ring_joint",
        articulation_type=ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # --- LCD ARM ---
    lcd_arm = model.part("lcd_arm")
    lcd_arm.visual(
        Box((0.015, 0.015, 0.04)),
        origin=Origin(xyz=(0.0, -0.0075, 0.0)),
        material=mat_body,
        name="lcd_arm_bracket",
    )
    lcd_arm.visual(
        Cylinder(radius=0.008, length=0.04),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=mat_body,
        name="lcd_arm_hinge2",
    )
    model.articulation(
        name="lcd_swing_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_arm,
        origin=Origin(xyz=(0.06, -0.055, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi / 2),
    )

    # --- LCD SCREEN ---
    lcd_screen = model.part("lcd_screen")
    lcd_screen.visual(
        Box((0.10, 0.01, 0.07)),
        origin=Origin(xyz=(-0.04, -0.005, 0.0)),
        material=mat_body,
        name="lcd_screen_frame",
    )
    lcd_screen.visual(
        Box((0.09, 0.002, 0.06)),
        origin=Origin(xyz=(-0.04, -0.01, 0.0)),
        material=mat_screen,
        name="lcd_screen_panel",
    )
    model.articulation(
        name="lcd_tilt_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=lcd_arm,
        child=lcd_screen,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi),
    )

    # --- MEDIA DOOR ---
    media_door = model.part("media_door")
    media_door.visual(
        Box((0.08, 0.005, 0.06)),
        origin=Origin(xyz=(0.04, 0.0025, 0.0)),
        material=mat_body,
        name="media_door_panel",
    )
    model.articulation(
        name="media_door_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=media_door,
        origin=Origin(xyz=(-0.05, 0.055, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi / 2),
    )

    # --- RECORD BUTTON ---
    rec_button = model.part("rec_button")
    rec_button.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=mat_button,
        name="rec_button_cap",
    )
    model.articulation(
        name="rec_button_joint",
        articulation_type=ArticulationType.PRISMATIC,
        parent=body,
        child=rec_button,
        origin=Origin(xyz=(-0.12, 0.03, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.002, upper=0.0),
    )

    # --- ZOOM ROCKER ---
    zoom_rocker = model.part("zoom_rocker")
    zoom_rocker.visual(
        Box((0.03, 0.01, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=mat_metal,
        name="zoom_rocker_switch",
    )
    model.articulation(
        name="zoom_rocker_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=zoom_rocker,
        origin=Origin(xyz=(0.08, 0.04, 0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.2, upper=0.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Intentional overlaps for mounted/nested components:
    ctx.allow_overlap(
        "body",
        "focus_ring",
        reason="Focus ring wraps around the lens barrel.",
    )
    ctx.allow_overlap(
        "body",
        "lcd_arm",
        elem_a="lcd_hinge_mount",
        elem_b="lcd_arm_bracket",
        reason="LCD arm pivots inside the hinge mount.",
    )
    ctx.allow_overlap(
        "lcd_arm",
        "lcd_screen",
        elem_a="lcd_arm_hinge2",
        elem_b="lcd_screen_frame",
        reason="LCD screen rotates around the arm hinge.",
    )
    ctx.allow_overlap(
        "body",
        "media_door",
        elem_a="media_door_mount",
        elem_b="media_door_panel",
        reason="Media door pivots on its mount.",
    )
    ctx.allow_overlap(
        "body",
        "rec_button",
        reason="Record button is partially embedded in the body.",
    )
    ctx.allow_overlap(
        "body",
        "zoom_rocker",
        reason="Zoom rocker is seated on the body top.",
    )

    # Exact checks
    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    lcd_arm = object_model.get_part("lcd_arm")
    lcd_screen = object_model.get_part("lcd_screen")
    media_door = object_model.get_part("media_door")

    # Focus ring is around the lens barrel
    ctx.expect_within(
        focus_ring,
        body,
        axes="yz",
        margin=0.005,
        name="focus ring stays centered on lens barrel",
    )

    # LCD screen clears body when closed
    ctx.expect_gap(
        body,
        lcd_screen,
        axis="y",
        min_gap=0.0,
        name="lcd screen rests outside the body",
    )

    # Pose checks
    lcd_swing = object_model.get_articulation("lcd_swing_joint")
    with ctx.pose({lcd_swing: math.pi / 2}):
        aabb = ctx.part_world_aabb(lcd_screen)
        ctx.check(
            "lcd screen swings out to the left",
            aabb is not None and aabb[0][1] < -0.1,
            details=f"aabb={aabb}",
        )

    door_joint = object_model.get_articulation("media_door_joint")
    with ctx.pose({door_joint: math.pi / 2}):
        aabb = ctx.part_world_aabb(media_door)
        ctx.check(
            "media door swings out to the right",
            aabb is not None and aabb[1][1] > 0.1,
            details=f"aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()