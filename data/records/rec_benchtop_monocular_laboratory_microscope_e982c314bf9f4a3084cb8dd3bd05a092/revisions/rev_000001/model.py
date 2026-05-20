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
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_monocular_microscope")

    enamel = model.material("enamel", rgba=(0.90, 0.92, 0.94, 1.0))
    stage_black = model.material("stage_black", rgba=(0.14, 0.15, 0.16, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    base = model.part("base")
    base.visual(
        Box((0.165, 0.120, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=enamel,
        name="base_plinth",
    )
    base.visual(
        Box((0.072, 0.040, 0.036)),
        origin=Origin(xyz=(0.0, 0.036, 0.040)),
        material=enamel,
        name="base_casting",
    )
    arm_profile = rounded_rect_profile(0.020, 0.026, radius=0.005, corner_segments=8)
    arm_geom = sweep_profile_along_spline(
        [
            (0.0, 0.036, 0.028),
            (0.0, 0.038, 0.105),
            (0.0, 0.036, 0.185),
            (0.0, 0.031, 0.258),
            (0.0, 0.028, 0.305),
        ],
        profile=arm_profile,
        samples_per_segment=16,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    base.visual(
        _mesh("microscope_arm", arm_geom),
        material=enamel,
        name="swept_arm",
    )
    base.visual(
        Box((0.032, 0.024, 0.090)),
        origin=Origin(xyz=(0.0, 0.028, 0.220)),
        material=enamel,
        name="guide_mount_block",
    )
    base.visual(
        Box((0.056, 0.010, 0.240)),
        origin=Origin(xyz=(0.0, 0.029, 0.220)),
        material=steel,
        name="guide_backplate",
    )
    base.visual(
        Box((0.010, 0.010, 0.240)),
        origin=Origin(xyz=(-0.021, 0.023, 0.220)),
        material=steel,
        name="guide_left_rail",
    )
    base.visual(
        Box((0.010, 0.010, 0.240)),
        origin=Origin(xyz=(0.021, 0.023, 0.220)),
        material=steel,
        name="guide_right_rail",
    )
    base.visual(
        Box((0.050, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.050, 0.191)),
        material=steel,
        name="upper_focus_saddle",
    )
    base.visual(
        Box((0.050, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.050, 0.177)),
        material=steel,
        name="lower_focus_saddle",
    )
    for foot_x, foot_y, foot_name in (
        (-0.055, -0.038, "foot_fl"),
        (0.055, -0.038, "foot_fr"),
        (-0.055, 0.038, "foot_rl"),
        (0.055, 0.038, "foot_rr"),
    ):
        base.visual(
            Box((0.026, 0.018, 0.004)),
            origin=Origin(xyz=(foot_x, foot_y, 0.002)),
            material=rubber,
            name=foot_name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.165, 0.120, 0.320)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.115, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, -0.037, 0.0)),
        material=stage_black,
        name="front_strip",
    )
    stage.visual(
        Box((0.115, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.037, 0.0)),
        material=stage_black,
        name="rear_strip",
    )
    stage.visual(
        Box((0.022, 0.052, 0.006)),
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=stage_black,
        name="left_strip",
    )
    stage.visual(
        Box((0.022, 0.052, 0.006)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=stage_black,
        name="right_strip",
    )
    stage.visual(
        Box((0.096, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.024, 0.0045)),
        material=steel,
        name="front_runner",
    )
    stage.visual(
        Box((0.096, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.024, 0.0045)),
        material=steel,
        name="rear_runner",
    )
    stage.visual(
        Box((0.030, 0.030, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, -0.0775)),
        material=enamel,
        name="support_column",
    )
    stage.visual(
        Box((0.090, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=enamel,
        name="support_crosshead",
    )
    stage.inertial = Inertial.from_geometry(
        Box((0.120, 0.090, 0.146)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )
    model.articulation(
        "base_to_stage",
        ArticulationType.FIXED,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.0, -0.055, 0.162)),
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        Box((0.078, 0.060, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=steel,
        name="carriage_bridge",
    )
    slide_carriage.visual(
        Box((0.074, 0.008, 0.001)),
        origin=Origin(xyz=(0.0, -0.024, -0.0003)),
        material=steel,
        name="front_bearing",
    )
    slide_carriage.visual(
        Box((0.074, 0.008, 0.001)),
        origin=Origin(xyz=(0.0, 0.024, -0.0003)),
        material=steel,
        name="rear_bearing",
    )
    slide_carriage.visual(
        Box((0.068, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.020, 0.0045)),
        material=stage_black,
        name="slide_clamp_bar",
    )
    slide_carriage.visual(
        Box((0.020, 0.034, 0.015)),
        origin=Origin(xyz=(0.049, 0.0, 0.0075)),
        material=stage_black,
        name="carriage_block",
    )
    slide_carriage.visual(
        Box((0.010, 0.020, 0.008)),
        origin=Origin(xyz=(0.064, 0.0, 0.011)),
        material=knob_black,
        name="carriage_finger_tab",
    )
    slide_carriage.inertial = Inertial.from_geometry(
        Box((0.088, 0.060, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(0.012, 0.0, 0.009)),
    )
    model.articulation(
        "stage_to_slide_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=slide_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=-0.028,
            upper=0.028,
        ),
    )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.044, 0.028, 0.074)),
        origin=Origin(xyz=(0.0, 0.004, 0.055)),
        material=enamel,
        name="carriage_block",
    )
    head_carriage.visual(
        Box((0.030, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, -0.012, 0.092)),
        material=enamel,
        name="carriage_neck",
    )
    head_carriage.visual(
        Box((0.058, 0.070, 0.042)),
        origin=Origin(xyz=(0.0, -0.041, 0.100)),
        material=enamel,
        name="head_body",
    )
    head_carriage.visual(
        Box((0.028, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, -0.014, 0.117)),
        material=enamel,
        name="eyetube_saddle",
    )
    head_carriage.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, -0.076, 0.091)),
        material=steel,
        name="nose_collar",
    )
    head_carriage.visual(
        Cylinder(radius=0.011, length=0.058),
        origin=Origin(xyz=(0.0, -0.076, 0.050)),
        material=steel,
        name="objective_barrel",
    )
    head_carriage.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(
            xyz=(0.0, -0.050, 0.150),
            rpy=(-math.radians(35.0), 0.0, 0.0),
        ),
        material=steel,
        name="eyetube",
    )
    head_carriage.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(
            xyz=(0.0, -0.038, 0.181),
            rpy=(-math.radians(35.0), 0.0, 0.0),
        ),
        material=knob_black,
        name="ocular_ring",
    )
    head_carriage.inertial = Inertial.from_geometry(
        Box((0.070, 0.150, 0.210)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -0.010, 0.105)),
    )
    model.articulation(
        "base_to_head_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_carriage,
        origin=Origin(xyz=(0.0, 0.000, 0.153)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.10,
            lower=0.0,
            upper=0.100,
        ),
    )

    focus_knob_pair = model.part("focus_knob_pair")
    focus_knob_pair.visual(
        Cylinder(radius=0.004, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="focus_shaft",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_inner_knob",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_inner_knob",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(-0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="left_focus_knob",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="right_focus_knob",
    )
    focus_knob_pair.inertial = Inertial.from_geometry(
        Box((0.104, 0.036, 0.036)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_focus_knob_pair",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=focus_knob_pair,
        origin=Origin(xyz=(0.0, 0.053, 0.184)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage = object_model.get_part("stage")
    slide_carriage = object_model.get_part("slide_carriage")
    head_carriage = object_model.get_part("head_carriage")

    slide_joint = object_model.get_articulation("stage_to_slide_carriage")
    head_joint = object_model.get_articulation("base_to_head_carriage")
    focus_joint = object_model.get_articulation("base_to_focus_knob_pair")

    ctx.check(
        "slide carriage joint runs left-right",
        tuple(round(v, 3) for v in slide_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide_joint.axis}",
    )
    ctx.check(
        "head carriage joint runs vertically",
        tuple(round(v, 3) for v in head_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={head_joint.axis}",
    )
    ctx.check(
        "focus knob shaft is horizontal",
        tuple(round(v, 3) for v in focus_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={focus_joint.axis}",
    )

    ctx.expect_contact(
        stage,
        base,
        elem_a="support_column",
        elem_b="base_plinth",
        contact_tol=0.001,
        name="stage support seats on the base",
    )
    ctx.expect_gap(
        slide_carriage,
        stage,
        axis="z",
        min_gap=0.0003,
        max_gap=0.003,
        positive_elem="carriage_bridge",
        name="slide carriage rides just above the stage runners",
    )
    ctx.expect_overlap(
        slide_carriage,
        stage,
        axes="xy",
        elem_a="carriage_bridge",
        min_overlap=0.050,
        name="slide carriage remains over the stage footprint",
    )
    ctx.expect_contact(
        slide_carriage,
        stage,
        elem_a="front_bearing",
        elem_b="front_runner",
        contact_tol=0.001,
        name="slide carriage front bearing rides on the stage runner",
    )
    ctx.expect_gap(
        head_carriage,
        stage,
        axis="z",
        min_gap=0.006,
        max_gap=0.030,
        positive_elem="objective_barrel",
        name="objective clears the stage surface",
    )
    ctx.expect_contact(
        head_carriage,
        base,
        elem_a="carriage_block",
        elem_b="guide_left_rail",
        contact_tol=0.001,
        name="head carriage bears on the guide rail",
    )
    ctx.expect_contact(
        object_model.get_part("focus_knob_pair"),
        base,
        elem_a="focus_shaft",
        elem_b="upper_focus_saddle",
        contact_tol=0.001,
        name="focus shaft is carried by the upper saddle",
    )

    with ctx.pose({slide_joint: slide_joint.motion_limits.lower}):
        slide_left = ctx.part_world_position(slide_carriage)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        slide_right = ctx.part_world_position(slide_carriage)
        ctx.expect_overlap(
            slide_carriage,
            stage,
            axes="y",
            elem_a="carriage_bridge",
            min_overlap=0.050,
            name="slide carriage stays aligned with the stage at max travel",
        )
    ctx.check(
        "slide carriage traverses left to right",
        slide_left is not None
        and slide_right is not None
        and slide_right[0] > slide_left[0] + 0.050,
        details=f"left={slide_left}, right={slide_right}",
    )

    head_low = ctx.part_world_position(head_carriage)
    with ctx.pose({head_joint: head_joint.motion_limits.upper}):
        head_high = ctx.part_world_position(head_carriage)
    ctx.check(
        "head carriage raises on the guide",
        head_low is not None
        and head_high is not None
        and head_high[2] > head_low[2] + 0.090,
        details=f"low={head_low}, high={head_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
