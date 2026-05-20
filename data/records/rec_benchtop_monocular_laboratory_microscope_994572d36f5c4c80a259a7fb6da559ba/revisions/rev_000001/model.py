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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _circle_profile(radius: float, segments: int = 20) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    body_paint = model.material("body_paint", rgba=(0.92, 0.92, 0.88, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    stage_black = model.material("stage_black", rgba=(0.12, 0.13, 0.14, 1.0))
    optic_black = model.material("optic_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.24, 0.25, 0.28, 1.0))

    body = model.part("body")

    base_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.170, 0.126, 0.020, corner_segments=8),
            0.026,
            cap=True,
            center=True,
        ),
        "microscope_base_shell",
    )
    body.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=body_paint,
        name="base_shell",
    )

    arm_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.0, 0.031, 0.028),
                (0.0, 0.031, 0.056),
                (0.0, 0.028, 0.084),
                (0.0, 0.024, 0.106),
                (0.0, 0.019, 0.120),
                (0.0, 0.016, 0.132),
            ],
            profile=rounded_rect_profile(0.034, 0.022, 0.007, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "microscope_arm",
    )
    body.visual(arm_mesh, material=body_paint, name="curved_arm")
    body.visual(
        Box((0.042, 0.036, 0.054)),
        origin=Origin(xyz=(0.0, 0.029, 0.050)),
        material=body_paint,
        name="arm_root",
    )
    body.visual(
        Box((0.026, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.022, 0.150)),
        material=body_paint,
        name="arm_head_support",
    )
    body.visual(
        Box((0.020, 0.010, 0.080)),
        origin=Origin(xyz=(0.0, 0.028, 0.132)),
        material=trim_gray,
        name="focus_guide",
    )

    stage_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            [
                (-0.038, -0.038),
                (0.038, -0.038),
                (0.038, 0.038),
                (-0.038, 0.038),
            ],
            [_circle_profile(0.010, segments=20)],
            0.006,
            cap=True,
            center=True,
        ),
        "microscope_stage_plate",
    )
    body.visual(
        stage_mesh,
        origin=Origin(xyz=(0.0, -0.032, 0.073)),
        material=stage_black,
        name="stage_plate",
    )
    body.visual(
        Box((0.040, 0.016, 0.048)),
        origin=Origin(xyz=(0.0, 0.013, 0.046)),
        material=body_paint,
        name="stage_support",
    )
    body.visual(
        Box((0.024, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.035)),
        material=trim_gray,
        name="substage_block",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.170, 0.126, 0.190)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.006, 0.014, 0.060)),
        origin=Origin(xyz=(-0.015, 0.026, 0.030)),
        material=trim_gray,
        name="left_slide_cheek",
    )
    head_carriage.visual(
        Box((0.006, 0.014, 0.060)),
        origin=Origin(xyz=(0.015, 0.026, 0.030)),
        material=trim_gray,
        name="right_slide_cheek",
    )
    head_carriage.visual(
        Box((0.006, 0.026, 0.028)),
        origin=Origin(xyz=(-0.015, 0.011, 0.034)),
        material=trim_gray,
        name="left_carriage_web",
    )
    head_carriage.visual(
        Box((0.006, 0.026, 0.028)),
        origin=Origin(xyz=(0.015, 0.011, 0.034)),
        material=trim_gray,
        name="right_carriage_web",
    )
    head_carriage.visual(
        Box((0.040, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, -0.010, 0.032)),
        material=body_paint,
        name="head_block",
    )
    head_carriage.visual(
        Box((0.022, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.002, 0.049)),
        material=body_paint,
        name="prism_housing",
    )
    eyepiece_tilt = math.radians(32.0)
    head_carriage.visual(
        Cylinder(radius=0.011, length=0.062),
        origin=Origin(xyz=(0.0, -0.006, 0.076), rpy=(-eyepiece_tilt, 0.0, 0.0)),
        material=optic_black,
        name="eyepiece_tube",
    )
    head_carriage.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.014, 0.097), rpy=(-eyepiece_tilt, 0.0, 0.0)),
        material=optic_black,
        name="eyecup",
    )
    head_carriage.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=trim_gray,
        name="turret_bearing",
    )
    head_carriage.inertial = Inertial.from_geometry(
        Box((0.070, 0.080, 0.105)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.002, 0.046)),
    )

    stage_carrier = model.part("stage_carrier")
    stage_carrier.visual(
        Box((0.044, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_dark,
        name="slider_plate",
    )
    stage_carrier.visual(
        Box((0.022, 0.036, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=stage_black,
        name="carrier_bridge",
    )
    stage_carrier.visual(
        Box((0.010, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.026, -0.004)),
        material=metal_dark,
        name="stage_handle",
    )
    stage_carrier.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, -0.038, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_black,
        name="stage_knob",
    )
    stage_carrier.inertial = Inertial.from_geometry(
        Box((0.050, 0.046, 0.020)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.014, -0.002)),
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=trim_gray,
        name="turret_disk",
    )
    nosepiece.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=trim_gray,
        name="turret_hub",
    )
    nosepiece.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, -0.010)),
        material=optic_black,
        name="objective_active",
    )
    nosepiece.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(
            xyz=(-0.0104, 0.0060, -0.009),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=optic_black,
        name="objective_side_a",
    )
    nosepiece.visual(
        Cylinder(radius=0.0055, length=0.017),
        origin=Origin(
            xyz=(0.0104, 0.0060, -0.0075),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=optic_black,
        name="objective_side_b",
    )
    nosepiece.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.040),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    head_slide = model.articulation(
        "body_to_head_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=head_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.06,
            lower=0.0,
            upper=0.045,
        ),
    )
    stage_slide = model.articulation(
        "body_to_stage_carrier",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage_carrier,
        origin=Origin(xyz=(0.0, -0.032, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=-0.014,
            upper=0.014,
        ),
    )
    nosepiece_joint = model.articulation(
        "head_to_nosepiece",
        ArticulationType.CONTINUOUS,
        parent=head_carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.0, -0.020, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
        ),
    )

    model.meta["primary_joints"] = {
        "head_slide": head_slide.name,
        "stage_slide": stage_slide.name,
        "nosepiece_joint": nosepiece_joint.name,
    }

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head_carriage = object_model.get_part("head_carriage")
    stage_carrier = object_model.get_part("stage_carrier")
    nosepiece = object_model.get_part("nosepiece")

    head_slide = object_model.get_articulation("body_to_head_carriage")
    stage_slide = object_model.get_articulation("body_to_stage_carrier")
    nosepiece_joint = object_model.get_articulation("head_to_nosepiece")

    with ctx.pose({head_slide: 0.0, stage_slide: 0.0, nosepiece_joint: 0.0}):
        ctx.expect_gap(
            nosepiece,
            body,
            axis="z",
            positive_elem="objective_active",
            negative_elem="stage_plate",
            min_gap=0.004,
            max_gap=0.018,
            name="objective clears the stage at the lowest focus position",
        )
        ctx.expect_gap(
            body,
            stage_carrier,
            axis="z",
            positive_elem="stage_plate",
            negative_elem="slider_plate",
            max_penetration=1e-6,
            max_gap=0.0015,
            name="stage carrier stays in contact just beneath the stage plate",
        )

    with ctx.pose({stage_slide: -0.014}):
        ctx.expect_within(
            stage_carrier,
            body,
            axes="xy",
            inner_elem="slider_plate",
            outer_elem="stage_plate",
            margin=0.0,
            name="stage carrier stays within the stage footprint at left travel",
        )

    with ctx.pose({stage_slide: 0.014}):
        ctx.expect_within(
            stage_carrier,
            body,
            axes="xy",
            inner_elem="slider_plate",
            outer_elem="stage_plate",
            margin=0.0,
            name="stage carrier stays within the stage footprint at right travel",
        )

    with ctx.pose({stage_slide: -0.014}):
        stage_left = ctx.part_world_position(stage_carrier)
    with ctx.pose({stage_slide: 0.014}):
        stage_right = ctx.part_world_position(stage_carrier)
    ctx.check(
        "stage carrier slides left to right",
        stage_left is not None and stage_right is not None and stage_right[0] > stage_left[0] + 0.020,
        details=f"left={stage_left}, right={stage_right}",
    )

    head_low = ctx.part_world_position(head_carriage)
    with ctx.pose({head_slide: 0.045}):
        head_high = ctx.part_world_position(head_carriage)
    ctx.check(
        "head carriage rises on the guide",
        head_low is not None and head_high is not None and head_high[2] > head_low[2] + 0.035,
        details=f"low={head_low}, high={head_high}",
    )

    side_a_rest = ctx.part_element_world_aabb(nosepiece, elem="objective_side_a")
    with ctx.pose({nosepiece_joint: 2.1}):
        side_a_rotated = ctx.part_element_world_aabb(nosepiece, elem="objective_side_a")
    if side_a_rest is None or side_a_rotated is None:
        ctx.fail(
            "nosepiece rotates objective barrels",
            f"rest_aabb={side_a_rest}, rotated_aabb={side_a_rotated}",
        )
    else:
        rest_center = tuple((a + b) * 0.5 for a, b in zip(side_a_rest[0], side_a_rest[1]))
        rotated_center = tuple((a + b) * 0.5 for a, b in zip(side_a_rotated[0], side_a_rotated[1]))
        planar_shift = math.hypot(
            rotated_center[0] - rest_center[0],
            rotated_center[1] - rest_center[1],
        )
        ctx.check(
            "nosepiece rotates objective barrels",
            planar_shift > 0.010,
            details=f"rest_center={rest_center}, rotated_center={rotated_center}, shift={planar_shift}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
