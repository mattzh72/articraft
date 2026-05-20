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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _tube_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    lip: float = 0.0,
):
    outer = [
        (outer_radius * 0.92, 0.0),
        (outer_radius, min(0.010, length * 0.08)),
        (outer_radius, max(length - max(lip, 0.012), 0.014)),
        (outer_radius * 0.84, length),
    ]
    inner = [
        (0.0, 0.003),
        (inner_radius, min(0.012, length * 0.10)),
        (inner_radius, max(length - max(lip, 0.016), 0.012)),
    ]
    if lip > 0.0:
        inner.append((inner_radius * 0.72, length))
    return _save_mesh(name, LatheGeometry.from_shell_profiles(outer, inner, segments=52))


def _add_objective(
    turret_part,
    *,
    angle: float,
    body_length: float,
    barrel_name: str,
    metal,
    black,
) -> None:
    cx = 0.016 * math.sin(angle)
    cy = 0.016 * math.cos(angle)
    turret_part.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(cx, cy, -0.010)),
        material=metal,
        name=f"{barrel_name}_mount",
    )
    turret_part.visual(
        Cylinder(radius=0.0052, length=body_length),
        origin=Origin(xyz=(cx, cy, -0.016 - body_length * 0.5)),
        material=metal,
        name=barrel_name,
    )
    turret_part.visual(
        Cylinder(radius=0.0039, length=0.010),
        origin=Origin(xyz=(cx, cy, -0.021 - body_length)),
        material=black,
        name=f"{barrel_name}_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_microscope")

    enamel = model.material("enamel", rgba=(0.90, 0.91, 0.88, 1.0))
    enamel_shadow = model.material("enamel_shadow", rgba=(0.80, 0.82, 0.79, 1.0))
    stage_black = model.material("stage_black", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal = model.material("metal", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.82, 0.90, 0.35))
    rubber = model.material("rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    tube_tilt = 0.78

    stand = model.part("stand")

    stand.visual(
        Box((0.074, 0.182, 0.026)),
        origin=Origin(xyz=(-0.068, 0.010, 0.013)),
        material=enamel,
        name="left_base_foot",
    )
    stand.visual(
        Box((0.074, 0.182, 0.026)),
        origin=Origin(xyz=(0.068, 0.010, 0.013)),
        material=enamel,
        name="right_base_foot",
    )
    stand.visual(
        Box((0.206, 0.072, 0.032)),
        origin=Origin(xyz=(0.0, -0.074, 0.016)),
        material=enamel_shadow,
        name="rear_bridge",
    )
    stand.visual(
        Cylinder(radius=0.037, length=0.026),
        origin=Origin(xyz=(-0.068, 0.108, 0.013)),
        material=enamel,
        name="left_front_pad",
    )
    stand.visual(
        Cylinder(radius=0.037, length=0.026),
        origin=Origin(xyz=(0.068, 0.108, 0.013)),
        material=enamel,
        name="right_front_pad",
    )
    stand.visual(
        Box((0.074, 0.058, 0.078)),
        origin=Origin(xyz=(0.0, -0.045, 0.065)),
        material=enamel_shadow,
        name="pillar_block",
    )

    arm_mesh = _save_mesh(
        "microscope_arm",
        sweep_profile_along_spline(
            [
                (0.0, -0.075, 0.050),
                (0.0, -0.112, 0.132),
                (0.0, -0.126, 0.204),
                (0.0, -0.118, 0.266),
                (0.0, -0.100, 0.304),
            ],
            profile=rounded_rect_profile(0.044, 0.074, radius=0.012, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    stand.visual(arm_mesh, material=enamel, name="curved_arm")

    stand.visual(
        Box((0.074, 0.140, 0.040)),
        origin=Origin(xyz=(0.0, -0.048, 0.304)),
        material=enamel_shadow,
        name="head_yoke",
    )
    stand.visual(
        Box((0.054, 0.046, 0.046)),
        origin=Origin(xyz=(0.0, -0.020, 0.281)),
        material=enamel,
        name="prism_housing",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.0, -0.006, 0.313), rpy=(tube_tilt, 0.0, 0.0)),
        material=enamel_shadow,
        name="tube_collar",
    )
    stand.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, -0.004, 0.247)),
        material=dark_metal,
        name="nosepiece_mount",
    )

    body_tube_mesh = _tube_shell_mesh(
        "body_tube_shell",
        outer_radius=0.019,
        inner_radius=0.014,
        length=0.106,
    )
    stand.visual(
        body_tube_mesh,
        origin=Origin(xyz=(0.0, -0.006, 0.314), rpy=(tube_tilt, 0.0, 0.0)),
        material=metal,
        name="body_tube",
    )

    eyepiece_mount_offset = 0.090
    eyepiece_origin = (
        0.0,
        -0.006 - math.sin(tube_tilt) * eyepiece_mount_offset,
        0.314 + math.cos(tube_tilt) * eyepiece_mount_offset,
    )
    eyepiece_mesh = _tube_shell_mesh(
        "eyepiece_shell",
        outer_radius=0.0155,
        inner_radius=0.0115,
        length=0.050,
        lip=0.010,
    )
    stand.visual(
        eyepiece_mesh,
        origin=Origin(xyz=eyepiece_origin, rpy=(tube_tilt, 0.0, 0.0)),
        material=knob_black,
        name="eyepiece_shell",
    )
    stand.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(
            xyz=(
                eyepiece_origin[0],
                eyepiece_origin[1] - math.sin(tube_tilt) * 0.044,
                eyepiece_origin[2] + math.cos(tube_tilt) * 0.044,
            ),
            rpy=(tube_tilt, 0.0, 0.0),
        ),
        material=rubber,
        name="eyecup",
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(
            xyz=(
                eyepiece_origin[0],
                eyepiece_origin[1] - math.sin(tube_tilt) * 0.016,
                eyepiece_origin[2] + math.cos(tube_tilt) * 0.016,
            ),
            rpy=(tube_tilt, 0.0, 0.0),
        ),
        material=glass,
        name="eyepiece_lens",
    )

    stand.visual(
        Box((0.122, 0.122, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_metal,
        name="stage_support_plate",
    )
    stand.visual(
        Box((0.032, 0.064, 0.060)),
        origin=Origin(xyz=(0.062, 0.0, 0.115)),
        material=enamel_shadow,
        name="stage_support_column",
    )
    stand.visual(
        Box((0.082, 0.044, 0.016)),
        origin=Origin(xyz=(0.034, 0.0, 0.132)),
        material=enamel,
        name="stage_support_bridge",
    )
    stand.visual(
        Box((0.050, 0.050, 0.070)),
        origin=Origin(xyz=(0.040, -0.030, 0.115)),
        material=enamel_shadow,
        name="stage_support_arm",
    )
    stand.visual(
        Box((0.032, 0.084, 0.092)),
        origin=Origin(xyz=(0.084, -0.074, 0.171)),
        material=enamel_shadow,
        name="focus_housing",
    )
    stand.visual(
        Cylinder(radius=0.0045, length=0.148),
        origin=Origin(xyz=(0.0, -0.110, 0.188), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="focus_axle",
    )

    stand.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.057, -0.110, 0.188), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="coarse_focus_right",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.066, -0.110, 0.188), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stage_black,
        name="fine_focus_right",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.057, -0.110, 0.188), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="coarse_focus_left",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.066, -0.110, 0.188), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stage_black,
        name="fine_focus_left",
    )

    stand.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=dark_metal,
        name="condenser_body",
    )
    stand.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=glass,
        name="condenser_lens",
    )
    stand.visual(
        Box((0.018, 0.004, 0.008)),
        origin=Origin(xyz=(0.026, 0.0, 0.123)),
        material=dark_metal,
        name="iris_lever",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.240, 0.260, 0.400)),
        mass=5.2,
        origin=Origin(xyz=(0.0, -0.005, 0.145)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.0155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_metal,
        name="turret_hub",
    )
    turret.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=metal,
        name="turret_ring",
    )
    turret.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_metal,
        name="turret_spindle",
    )
    turret.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(0.0, 0.016, -0.010)),
        material=metal,
        name="objective_primary_barrel_mount",
    )
    turret.visual(
        Cylinder(radius=0.0052, length=0.026),
        origin=Origin(xyz=(0.0, 0.016, -0.026)),
        material=metal,
        name="objective_primary_barrel",
    )
    turret.visual(
        Cylinder(radius=0.0039, length=0.010),
        origin=Origin(xyz=(0.0, 0.016, -0.041)),
        material=stage_black,
        name="objective_primary_barrel_tip",
    )
    _add_objective(
        turret,
        angle=2.0 * math.pi / 3.0,
        body_length=0.014,
        barrel_name="objective_secondary_barrel",
        metal=metal,
        black=stage_black,
    )
    _add_objective(
        turret,
        angle=4.0 * math.pi / 3.0,
        body_length=0.018,
        barrel_name="objective_tertiary_barrel",
        metal=metal,
        black=stage_black,
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.075)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    stage_y = model.part("stage_y")
    stage_y.visual(
        Box((0.110, 0.100, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="y_carriage",
    )
    stage_y.visual(
        Box((0.108, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.030, 0.019)),
        material=stage_black,
        name="cross_slide_rail_front",
    )
    stage_y.visual(
        Box((0.108, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.030, 0.019)),
        material=stage_black,
        name="cross_slide_rail_rear",
    )
    stage_y.visual(
        Box((0.026, 0.052, 0.022)),
        origin=Origin(xyz=(0.060, 0.0, 0.016)),
        material=dark_metal,
        name="y_drive_housing",
    )
    stage_y.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.077, 0.0, 0.017), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="y_drive_stem",
    )
    stage_y.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.088, 0.0, 0.017), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="y_drive_knob_outer",
    )
    stage_y.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.094, 0.0, 0.017), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stage_black,
        name="y_drive_knob_inner",
    )
    stage_y.inertial = Inertial.from_geometry(
        Box((0.135, 0.115, 0.045)),
        mass=0.45,
        origin=Origin(xyz=(0.020, 0.0, 0.016)),
    )

    stage_x = model.part("stage_x")
    stage_plate_mesh = _save_mesh(
        "stage_plate_mesh",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.126, 0.126, 0.006, corner_segments=5),
            [_circle_profile(0.018, segments=24)],
            height=0.005,
            center=True,
        ),
    )
    stage_x.visual(
        Box((0.090, 0.086, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=dark_metal,
        name="x_carriage",
    )
    stage_x.visual(
        stage_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stage_black,
        name="stage_plate",
    )
    stage_x.visual(
        Box((0.088, 0.008, 0.008)),
        origin=Origin(xyz=(0.000, 0.043, 0.019)),
        material=metal,
        name="slide_guide_rail",
    )
    stage_x.visual(
        Box((0.016, 0.028, 0.010)),
        origin=Origin(xyz=(0.044, 0.025, 0.020)),
        material=metal,
        name="slide_clamp_block",
    )
    stage_x.visual(
        Box((0.040, 0.006, 0.006)),
        origin=Origin(xyz=(0.020, 0.032, 0.026)),
        material=metal,
        name="slide_clamp_spring",
    )
    stage_x.inertial = Inertial.from_geometry(
        Box((0.130, 0.130, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "stand_to_turret",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.239)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "stand_to_stage_y",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_y,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=-0.025,
            upper=0.025,
        ),
    )
    model.articulation(
        "stage_y_to_stage_x",
        ArticulationType.PRISMATIC,
        parent=stage_y,
        child=stage_x,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=-0.030,
            upper=0.030,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    turret = object_model.get_part("turret")
    stage_y = object_model.get_part("stage_y")
    stage_x = object_model.get_part("stage_x")

    turret_joint = object_model.get_articulation("stand_to_turret")
    stage_y_joint = object_model.get_articulation("stand_to_stage_y")
    stage_x_joint = object_model.get_articulation("stage_y_to_stage_x")

    ctx.check(
        "turret rotates about vertical optical axis",
        tuple(round(v, 6) for v in turret_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={turret_joint.axis}",
    )
    ctx.check(
        "stage assembly slides fore and aft",
        tuple(round(v, 6) for v in stage_y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={stage_y_joint.axis}",
    )
    ctx.check(
        "stage carrier slides left to right",
        tuple(round(v, 6) for v in stage_x_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={stage_x_joint.axis}",
    )

    with ctx.pose({stage_y_joint: 0.0, stage_x_joint: 0.0}):
        ctx.expect_gap(
            stage_y,
            stand,
            axis="z",
            positive_elem="y_carriage",
            negative_elem="stage_support_plate",
            max_gap=0.003,
            max_penetration=0.0,
            name="stage saddle rides just above support plate",
        )
        ctx.expect_overlap(
            stage_y,
            stand,
            axes="xy",
            elem_a="y_carriage",
            elem_b="stage_support_plate",
            min_overlap=0.080,
            name="stage saddle stays over support plate footprint",
        )
        ctx.expect_gap(
            stage_x,
            stage_y,
            axis="z",
            positive_elem="x_carriage",
            negative_elem="cross_slide_rail_front",
            max_gap=0.003,
            max_penetration=0.0,
            name="stage carrier rides just above front cross rail",
        )
        ctx.expect_gap(
            stage_x,
            stage_y,
            axis="z",
            positive_elem="x_carriage",
            negative_elem="cross_slide_rail_rear",
            max_gap=0.003,
            max_penetration=0.0,
            name="stage carrier rides just above rear cross rail",
        )
        ctx.expect_overlap(
            stage_x,
            stage_y,
            axes="x",
            elem_a="x_carriage",
            elem_b="cross_slide_rail_front",
            min_overlap=0.085,
            name="stage carrier remains engaged on cross slide",
        )
        ctx.expect_gap(
            turret,
            stage_x,
            axis="z",
            positive_elem="objective_primary_barrel",
            negative_elem="stage_plate",
            min_gap=0.010,
            max_gap=0.060,
            name="primary objective sits above stage",
        )

    y_rest = ctx.part_world_position(stage_y)
    with ctx.pose({stage_y_joint: stage_y_joint.motion_limits.upper}):
        y_forward = ctx.part_world_position(stage_y)
        ctx.expect_overlap(
            stage_y,
            stand,
            axes="y",
            elem_a="y_carriage",
            elem_b="stage_support_plate",
            min_overlap=0.080,
            name="fore-aft slide retains support overlap at travel limit",
        )
    ctx.check(
        "stage assembly moves forward at positive y travel",
        y_rest is not None and y_forward is not None and y_forward[1] > y_rest[1] + 0.015,
        details=f"rest={y_rest}, forward={y_forward}",
    )

    x_rest = ctx.part_world_position(stage_x)
    with ctx.pose({stage_x_joint: stage_x_joint.motion_limits.upper}):
        x_right = ctx.part_world_position(stage_x)
        ctx.expect_overlap(
            stage_x,
            stage_y,
            axes="x",
            elem_a="x_carriage",
            elem_b="cross_slide_rail_front",
            min_overlap=0.060,
            name="lateral slide retains cross-slide engagement at travel limit",
        )
    ctx.check(
        "stage carrier moves right at positive x travel",
        x_rest is not None and x_right is not None and x_right[0] > x_rest[0] + 0.020,
        details=f"rest={x_rest}, right={x_right}",
    )

    rest_obj = ctx.part_element_world_aabb(turret, elem="objective_primary_barrel")
    with ctx.pose({turret_joint: math.radians(70.0)}):
        rotated_obj = ctx.part_element_world_aabb(turret, elem="objective_primary_barrel")
    if rest_obj is not None and rotated_obj is not None:
        rest_center = tuple((rest_obj[0][i] + rest_obj[1][i]) * 0.5 for i in range(3))
        rot_center = tuple((rotated_obj[0][i] + rotated_obj[1][i]) * 0.5 for i in range(3))
    else:
        rest_center = None
        rot_center = None
    ctx.check(
        "turret rotation swings objective around axis",
        rest_center is not None
        and rot_center is not None
        and abs(rot_center[0] - rest_center[0]) > 0.010
        and abs(rot_center[2] - rest_center[2]) < 0.004,
        details=f"rest_center={rest_center}, rotated_center={rot_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
