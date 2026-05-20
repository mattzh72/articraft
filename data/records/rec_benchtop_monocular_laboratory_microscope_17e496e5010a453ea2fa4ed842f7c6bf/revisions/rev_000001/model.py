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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _axis_close(
    actual: tuple[float, float, float],
    expected: tuple[float, float, float],
    *,
    tol: float = 1e-6,
) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_microscope")

    enamel = model.material("enamel_ivory", rgba=(0.90, 0.90, 0.85, 1.0))
    stage_black = model.material("stage_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    metal = model.material("brushed_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("eyepiece_glass", rgba=(0.16, 0.20, 0.24, 0.45))

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.205, 0.150, 0.022),
            0.030,
            cap=True,
            closed=True,
        ),
        "microscope_foot_shell",
    )
    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.115, 0.105, 0.006),
            [rounded_rect_profile(0.030, 0.024, 0.003)],
            0.008,
            cap=True,
            center=True,
            closed=True,
        ),
        "microscope_stage_plate",
    )

    stand = model.part("stand")
    stand.visual(
        foot_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=enamel,
        name="foot_shell",
    )
    stand.visual(
        Box((0.070, 0.092, 0.040)),
        origin=Origin(xyz=(-0.045, 0.000, 0.050)),
        material=enamel,
        name="heel_block",
    )
    stand.visual(
        Box((0.045, 0.070, 0.305)),
        origin=Origin(xyz=(-0.040, 0.000, 0.1825)),
        material=enamel,
        name="arm_column",
    )
    stand.visual(
        Box((0.040, 0.056, 0.080)),
        origin=Origin(xyz=(-0.030, 0.000, 0.080)),
        material=enamel,
        name="arm_gusset",
    )
    stand.visual(
        Box((0.060, 0.044, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.094)),
        material=enamel,
        name="stage_support",
    )
    stand.visual(
        Box((0.030, 0.055, 0.125)),
        origin=Origin(xyz=(-0.010, 0.000, 0.240)),
        material=enamel,
        name="guide_web",
    )
    stand.visual(
        Box((0.014, 0.024, 0.185)),
        origin=Origin(xyz=(0.010, 0.000, 0.240)),
        material=metal,
        name="guide_rail",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.028, 0.040, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=stage_black,
        name="stage_mount_block",
    )
    stage.visual(
        stage_plate_mesh,
        origin=Origin(xyz=(0.040, 0.000, 0.022)),
        material=stage_black,
        name="stage_plate",
    )
    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        stage.visual(
            Box((0.012, 0.010, 0.004)),
            origin=Origin(xyz=(0.006, y_sign * 0.028, 0.028)),
            material=metal,
            name=f"{side_name}_clip_anchor",
        )
        stage.visual(
            Box((0.040, 0.004, 0.002)),
            origin=Origin(xyz=(0.020, y_sign * 0.028, 0.029)),
            material=metal,
            name=f"{side_name}_clip_leaf",
        )
        stage.visual(
            Box((0.003, 0.004, 0.006)),
            origin=Origin(xyz=(0.0385, y_sign * 0.028, 0.026)),
            material=metal,
            name=f"{side_name}_clip_tip",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.000, 0.018, 0.055)),
        material=satin_black,
        name="left_slide_shoe",
    )
    carriage.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.000, -0.018, 0.055)),
        material=satin_black,
        name="right_slide_shoe",
    )
    carriage.visual(
        Box((0.020, 0.046, 0.040)),
        origin=Origin(xyz=(0.018, 0.000, 0.060)),
        material=satin_black,
        name="guide_bridge",
    )
    carriage.visual(
        Box((0.028, 0.050, 0.034)),
        origin=Origin(xyz=(0.024, 0.000, 0.058)),
        material=enamel,
        name="tube_clamp",
    )
    carriage.visual(
        Box((0.014, 0.020, 0.040)),
        origin=Origin(xyz=(0.024, 0.000, 0.020)),
        material=enamel,
        name="optical_neck",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.195),
        origin=Origin(xyz=(0.030, 0.000, 0.0975)),
        material=satin_black,
        name="tube_body",
    )
    carriage.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.030, 0.000, 0.205)),
        material=satin_black,
        name="eyepiece_sleeve",
    )
    carriage.visual(
        Cylinder(radius=0.014, length=0.015),
        origin=Origin(xyz=(0.030, 0.000, 0.2275)),
        material=glass,
        name="eyepiece_glass",
    )
    carriage.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.030, 0.000, -0.003)),
        material=satin_black,
        name="objective_collar",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=satin_black,
        name="turret_plate",
    )
    nosepiece.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=satin_black,
        name="turret_hub",
    )
    nosepiece.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, -0.018)),
        material=metal,
        name="central_objective",
    )
    objective_specs = (
        ("objective_0", 0.015, 0.000, 0.011, 0.050),
        ("objective_1", -0.0075, 0.0130, 0.009, 0.036),
        ("objective_2", -0.0075, -0.0130, 0.008, 0.028),
    )
    for name, x_pos, y_pos, collar_radius, barrel_length in objective_specs:
        nosepiece.visual(
            Cylinder(radius=collar_radius, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, -0.011)),
            material=satin_black,
            name=f"{name}_collar",
        )
        nosepiece.visual(
            Cylinder(radius=collar_radius * 0.72, length=barrel_length),
            origin=Origin(xyz=(x_pos, y_pos, -(0.006 + barrel_length / 2.0))),
            material=metal,
            name=name,
        )

    focus_shaft = model.part("focus_shaft")
    focus_shaft.visual(
        Cylinder(radius=0.004, length=0.130),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="focus_shaft",
    )
    focus_shaft.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.000, 0.058, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="left_focus_knob",
    )
    focus_shaft.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.000, -0.058, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="right_focus_knob",
    )

    model.articulation(
        "stand_to_stage",
        ArticulationType.FIXED,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(0.000, 0.000, 0.105)),
    )
    model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.010, 0.000, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.050,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "carriage_to_nosepiece",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.030, 0.000, -0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
        ),
    )
    model.articulation(
        "stand_to_focus_shaft",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=focus_shaft,
        origin=Origin(xyz=(-0.030, 0.000, 0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("carriage")
    nosepiece = object_model.get_part("nosepiece")
    focus_shaft = object_model.get_part("focus_shaft")

    stage_mount = stage.get_visual("stage_mount_block")
    stage_support = stand.get_visual("stage_support")
    guide_rail = stand.get_visual("guide_rail")
    left_shoe = carriage.get_visual("left_slide_shoe")
    right_shoe = carriage.get_visual("right_slide_shoe")
    guide_bridge = carriage.get_visual("guide_bridge")
    objective_collar = carriage.get_visual("objective_collar")
    turret_plate = nosepiece.get_visual("turret_plate")
    selected_objective = nosepiece.get_visual("objective_0")
    arm_column = stand.get_visual("arm_column")
    focus_shaft_visual = focus_shaft.get_visual("focus_shaft")

    stage_joint = object_model.get_articulation("stand_to_stage")
    carriage_slide = object_model.get_articulation("stand_to_carriage")
    turret_joint = object_model.get_articulation("carriage_to_nosepiece")
    focus_joint = object_model.get_articulation("stand_to_focus_shaft")

    ctx.check(
        "all microscope parts present",
        all(part is not None for part in (stand, stage, carriage, nosepiece, focus_shaft)),
        details="Expected stand, stage, carriage, nosepiece, and focus shaft parts.",
    )
    ctx.check(
        "stage is fixed to the stand",
        stage_joint.articulation_type == ArticulationType.FIXED,
        details=f"joint_type={stage_joint.articulation_type}",
    )
    ctx.check(
        "carriage uses a vertical prismatic guide",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and _axis_close(carriage_slide.axis, (0.0, 0.0, 1.0)),
        details=f"type={carriage_slide.articulation_type}, axis={carriage_slide.axis}",
    )
    ctx.check(
        "nosepiece rotates about the optical axis",
        turret_joint.articulation_type == ArticulationType.CONTINUOUS
        and _axis_close(turret_joint.axis, (0.0, 0.0, 1.0)),
        details=f"type={turret_joint.articulation_type}, axis={turret_joint.axis}",
    )
    ctx.check(
        "focus knobs rotate on a horizontal shaft",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and _axis_close(focus_joint.axis, (0.0, 1.0, 0.0)),
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}",
    )
    ctx.allow_overlap(
        stand,
        focus_shaft,
        elem_a=arm_column,
        elem_b=focus_shaft_visual,
        reason="The cast arm is represented as a solid exterior proxy, while the focus shaft intentionally passes through an internal bore in that arm.",
    )

    ctx.expect_contact(
        stage,
        stand,
        elem_a=stage_mount,
        elem_b=stage_support,
        name="stage mount block seats on the stand support",
    )
    ctx.expect_gap(
        carriage,
        stand,
        axis="y",
        min_gap=0.0008,
        max_gap=0.0030,
        positive_elem=left_shoe,
        negative_elem=guide_rail,
        name="left slide shoe clears the guide rail",
    )
    ctx.expect_gap(
        stand,
        carriage,
        axis="y",
        min_gap=0.0008,
        max_gap=0.0030,
        positive_elem=guide_rail,
        negative_elem=right_shoe,
        name="right slide shoe clears the guide rail",
    )
    ctx.expect_gap(
        carriage,
        stand,
        axis="x",
        min_gap=0.0005,
        max_gap=0.0035,
        positive_elem=guide_bridge,
        negative_elem=guide_rail,
        name="guide rail sits inside the slide carriage throat",
    )
    ctx.expect_gap(
        carriage,
        nosepiece,
        axis="z",
        min_gap=0.0015,
        max_gap=0.010,
        positive_elem=objective_collar,
        negative_elem=turret_plate,
        name="nosepiece turret sits just below the objective collar",
    )
    ctx.expect_gap(
        nosepiece,
        stage,
        axis="z",
        min_gap=0.005,
        max_gap=0.020,
        positive_elem=selected_objective,
        negative_elem="stage_plate",
        name="selected objective clears the stage at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    slide_limits = carriage_slide.motion_limits
    upper = 0.080 if slide_limits is None or slide_limits.upper is None else slide_limits.upper
    with ctx.pose({carriage_slide: upper}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            stand,
            axis="y",
            min_gap=0.0008,
            max_gap=0.0030,
            positive_elem=left_shoe,
            negative_elem=guide_rail,
            name="left slide shoe stays aligned at full raise",
        )
        ctx.expect_gap(
            stand,
            carriage,
            axis="y",
            min_gap=0.0008,
            max_gap=0.0030,
            positive_elem=guide_rail,
            negative_elem=right_shoe,
            name="right slide shoe stays aligned at full raise",
        )
        ctx.expect_gap(
            nosepiece,
            stage,
            axis="z",
            min_gap=0.080,
            positive_elem=selected_objective,
            negative_elem="stage_plate",
            name="raised objective lifts well clear of the stage",
        )

    ctx.check(
        "carriage raises upward along the arm",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.06,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
