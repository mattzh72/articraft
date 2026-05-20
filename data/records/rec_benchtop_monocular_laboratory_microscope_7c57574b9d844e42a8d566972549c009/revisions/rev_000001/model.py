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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_lab_monocular_microscope")

    paint = model.material("paint", rgba=(0.93, 0.94, 0.95, 1.0))
    enamel = model.material("enamel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark = model.material("dark", rgba=(0.08, 0.09, 0.10, 1.0))
    metal = model.material("metal", rgba=(0.67, 0.70, 0.73, 1.0))
    lens = model.material("lens", rgba=(0.10, 0.16, 0.22, 0.55))

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.220, 0.165, 0.026),
            0.022,
            center=True,
        ),
        "microscope_foot",
    )

    stand = model.part("stand")
    stand.visual(
        foot_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=paint,
        name="foot",
    )
    stand.visual(
        Box((0.145, 0.082, 0.020)),
        origin=Origin(xyz=(0.000, 0.030, 0.032)),
        material=paint,
        name="foot_riser",
    )
    stand.visual(
        Box((0.034, 0.040, 0.300)),
        origin=Origin(xyz=(0.000, 0.045, 0.172)),
        material=paint,
        name="arm_column",
    )
    stand.visual(
        Box((0.034, 0.090, 0.040)),
        origin=Origin(xyz=(0.000, 0.005, 0.330)),
        material=paint,
        name="arm_bridge",
    )
    stand.visual(
        Box((0.060, 0.078, 0.032)),
        origin=Origin(xyz=(0.000, -0.050, 0.326)),
        material=paint,
        name="head_block",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.000, -0.022, 0.347)),
        material=paint,
        name="tube_collar",
    )
    stand.visual(
        Cylinder(radius=0.015, length=0.140),
        origin=Origin(xyz=(0.000, 0.015, 0.410), rpy=(-0.60, 0.0, 0.0)),
        material=paint,
        name="monocular_tube",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.000, 0.063, 0.480), rpy=(-0.60, 0.0, 0.0)),
        material=enamel,
        name="eyepiece_sleeve",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.000, 0.072, 0.493), rpy=(-0.60, 0.0, 0.0)),
        material=lens,
        name="eyepiece_lens",
    )
    stand.visual(
        Box((0.032, 0.120, 0.076)),
        origin=Origin(xyz=(0.000, 0.005, 0.148)),
        material=paint,
        name="stage_carrier",
    )
    stand.visual(
        Box((0.118, 0.092, 0.014)),
        origin=Origin(xyz=(0.000, -0.050, 0.193)),
        material=enamel,
        name="stage_support",
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.000, -0.050, 0.306)),
        material=metal,
        name="nosepiece_mount",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.220, 0.165, 0.470)),
        mass=5.2,
        origin=Origin(xyz=(0.000, 0.000, 0.235)),
    )

    stage_block = model.part("stage_block")
    stage_block.visual(
        Box((0.108, 0.050, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=metal,
        name="slide_carriage",
    )
    stage_block.visual(
        Box((0.074, 0.038, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=metal,
        name="cross_slide_block",
    )
    stage_outer = [
        (-0.055, -0.055),
        (0.055, -0.055),
        (0.055, 0.055),
        (-0.055, 0.055),
    ]
    stage_hole = [
        (0.010 * math.cos(theta), 0.010 * math.sin(theta))
        for theta in [2.0 * math.pi * step / 24.0 for step in range(24)]
    ]
    stage_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(stage_outer, [stage_hole], 0.006, center=True),
        "microscope_stage_plate",
    )
    stage_block.visual(
        stage_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.027)),
        material=dark,
        name="stage_plate",
    )
    stage_block.visual(
        Box((0.010, 0.044, 0.006)),
        origin=Origin(xyz=(0.041, 0.000, 0.033)),
        material=metal,
        name="slide_clamp_bar",
    )
    stage_block.visual(
        Box((0.012, 0.012, 0.006)),
        origin=Origin(xyz=(-0.040, 0.040, 0.030)),
        material=metal,
        name="rear_clip",
    )
    stage_block.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.057, 0.000, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="knob_boss",
    )
    stage_block.inertial = Inertial.from_geometry(
        Box((0.118, 0.110, 0.036)),
        mass=0.72,
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
    )

    turret = model.part("objective_turret")
    turret.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, -0.008)),
        material=enamel,
        name="turret_hub",
    )
    turret.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(xyz=(0.019, 0.000, -0.037)),
        material=metal,
        name="objective_long",
    )
    turret.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(
            xyz=(-0.0095, 0.0165, -0.032),
        ),
        material=metal,
        name="objective_mid",
    )
    turret.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(
            xyz=(-0.0095, -0.0165, -0.029),
        ),
        material=metal,
        name="objective_short",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.060)),
        mass=0.26,
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
    )

    stage_knob = model.part("stage_control_knob")
    stage_knob.visual(
        Cylinder(radius=0.0025, length=0.010),
        origin=Origin(xyz=(0.005, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="knob_shaft",
    )
    stage_knob.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.014, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="knob_wheel",
    )
    stage_knob.visual(
        Box((0.004, 0.004, 0.010)),
        origin=Origin(xyz=(0.014, 0.000, 0.009)),
        material=enamel,
        name="knob_grip",
    )
    stage_knob.inertial = Inertial.from_geometry(
        Box((0.024, 0.020, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.012, 0.000, 0.000)),
    )

    model.articulation(
        "stand_to_stage_block",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_block,
        origin=Origin(xyz=(0.000, -0.050, 0.200)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=-0.020,
            upper=0.020,
        ),
    )
    model.articulation(
        "stand_to_objective_turret",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=turret,
        origin=Origin(xyz=(0.000, -0.050, 0.292)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "stage_block_to_stage_control_knob",
        ArticulationType.CONTINUOUS,
        parent=stage_block,
        child=stage_knob,
        origin=Origin(xyz=(0.061, 0.000, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=8.0,
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
    stage_block = object_model.get_part("stage_block")
    turret = object_model.get_part("objective_turret")
    stage_knob = object_model.get_part("stage_control_knob")

    stage_slide = object_model.get_articulation("stand_to_stage_block")
    turret_joint = object_model.get_articulation("stand_to_objective_turret")
    knob_joint = object_model.get_articulation("stage_block_to_stage_control_knob")

    ctx.check("stand exists", stand is not None)
    ctx.check("stage block exists", stage_block is not None)
    ctx.check("turret exists", turret is not None)
    ctx.check("stage knob exists", stage_knob is not None)

    ctx.check(
        "stage slide moves fore and aft",
        stage_slide.axis == (0.0, 1.0, 0.0)
        and stage_slide.motion_limits is not None
        and stage_slide.motion_limits.lower == -0.020
        and stage_slide.motion_limits.upper == 0.020,
        details=f"axis={stage_slide.axis}, limits={stage_slide.motion_limits}",
    )
    ctx.check(
        "turret rotates about optical axis",
        turret_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={turret_joint.axis}",
    )
    ctx.check(
        "stage knob rotates about horizontal shaft",
        knob_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )

    with ctx.pose({stage_slide: 0.0}):
        ctx.expect_contact(
            stage_block,
            stand,
            elem_a="slide_carriage",
            elem_b="stage_support",
            name="slide carriage sits on stage support",
        )
        ctx.expect_within(
            stage_block,
            stand,
            axes="x",
            inner_elem="slide_carriage",
            outer_elem="stage_support",
            margin=0.001,
            name="slide carriage stays centered laterally on support",
        )

    low_pos = None
    high_pos = None
    with ctx.pose({stage_slide: -0.020}):
        ctx.expect_overlap(
            stage_block,
            stand,
            axes="y",
            elem_a="slide_carriage",
            elem_b="stage_support",
            min_overlap=0.045,
            name="stage carriage retains fore support at front travel",
        )
        low_pos = ctx.part_world_position(stage_block)
    with ctx.pose({stage_slide: 0.020}):
        ctx.expect_overlap(
            stage_block,
            stand,
            axes="y",
            elem_a="slide_carriage",
            elem_b="stage_support",
            min_overlap=0.045,
            name="stage carriage retains fore support at rear travel",
        )
        high_pos = ctx.part_world_position(stage_block)
    ctx.check(
        "stage block translates along y",
        low_pos is not None and high_pos is not None and high_pos[1] > low_pos[1] + 0.035,
        details=f"low={low_pos}, high={high_pos}",
    )

    long_obj_rest = None
    long_obj_rotated = None
    with ctx.pose({turret_joint: 0.0}):
        long_obj_rest = ctx.part_element_world_aabb(turret, elem="objective_long")
        ctx.expect_contact(
            turret,
            stand,
            elem_a="turret_hub",
            elem_b="nosepiece_mount",
            name="turret hub seats against nosepiece mount",
        )
    with ctx.pose({turret_joint: math.pi / 2.0}):
        long_obj_rotated = ctx.part_element_world_aabb(turret, elem="objective_long")
    if long_obj_rest is not None and long_obj_rotated is not None:
        rest_center_y = (long_obj_rest[0][1] + long_obj_rest[1][1]) * 0.5
        rot_center_y = (long_obj_rotated[0][1] + long_obj_rotated[1][1]) * 0.5
        ctx.check(
            "turret rotation repositions the long objective around the axis",
            abs(rot_center_y - rest_center_y) > 0.010,
            details=f"rest_y={rest_center_y}, rotated_y={rot_center_y}",
        )
    else:
        ctx.fail(
            "turret objective bounds available",
            details=f"rest={long_obj_rest}, rotated={long_obj_rotated}",
        )

    grip_rest = None
    grip_rotated = None
    with ctx.pose({knob_joint: 0.0}):
        grip_rest = ctx.part_element_world_aabb(stage_knob, elem="knob_grip")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        grip_rotated = ctx.part_element_world_aabb(stage_knob, elem="knob_grip")
    if grip_rest is not None and grip_rotated is not None:
        rest_center_z = (grip_rest[0][2] + grip_rest[1][2]) * 0.5
        rot_center_z = (grip_rotated[0][2] + grip_rotated[1][2]) * 0.5
        ctx.check(
            "knob grip moves around the shaft",
            abs(rot_center_z - rest_center_z) > 0.006,
            details=f"rest_z={rest_center_z}, rotated_z={rot_center_z}",
        )
    else:
        ctx.fail(
            "knob grip bounds available",
            details=f"rest={grip_rest}, rotated={grip_rotated}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
