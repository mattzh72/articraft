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
    sweep_profile_along_spline,
)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _yoke_arm_mesh(points: list[tuple[float, float, float]], name: str):
    return mesh_from_geometry(
        sweep_profile_along_spline(
            points,
            profile=rounded_rect_profile(0.018, 0.038, radius=0.005, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
        name,
    )


def _rect_profile(width: float, height: float):
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    enamel = model.material("enamel", rgba=(0.89, 0.90, 0.86, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    satin = model.material("satin", rgba=(0.72, 0.74, 0.76, 1.0))
    glass = model.material("glass", rgba=(0.60, 0.74, 0.82, 0.35))
    lamp_blue = model.material("lamp_blue", rgba=(0.29, 0.42, 0.54, 1.0))

    left_arm_mesh = _yoke_arm_mesh(
        [
            (-0.044, -0.020, 0.086),
            (-0.044, -0.018, 0.170),
            (-0.039, -0.004, 0.245),
            (-0.030, 0.032, 0.294),
            (-0.020, 0.055, 0.317),
        ],
        "left_yoke_arm",
    )
    right_arm_mesh = _yoke_arm_mesh(
        [
            (0.044, -0.020, 0.086),
            (0.044, -0.018, 0.170),
            (0.039, -0.004, 0.245),
            (0.030, 0.032, 0.294),
            (0.020, 0.055, 0.317),
        ],
        "right_yoke_arm",
    )
    guide_track_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.078, 0.096),
            [_rect_profile(0.034, 0.058)],
            0.006,
            cap=True,
            center=True,
        ),
        "guide_track_frame",
    )
    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.125, 0.109),
            [_rect_profile(0.030, 0.030)],
            0.008,
            cap=True,
            center=True,
        ),
        "stage_plate",
    )

    frame = model.part("frame")
    frame.visual(
        Box((0.220, 0.160, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=enamel,
        name="base_plate",
    )
    frame.visual(
        Box((0.150, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.043, 0.028)),
        material=enamel,
        name="front_toe",
    )
    frame.visual(
        Box((0.120, 0.100, 0.060)),
        origin=Origin(xyz=(0.0, -0.026, 0.030)),
        material=enamel,
        name="rear_mass",
    )
    frame.visual(
        Box((0.065, 0.075, 0.180)),
        origin=Origin(xyz=(0.0, -0.020, 0.150)),
        material=enamel,
        name="rear_pedestal",
    )
    frame.visual(left_arm_mesh, material=enamel, name="left_yoke_arm")
    frame.visual(right_arm_mesh, material=enamel, name="right_yoke_arm")
    frame.visual(
        Box((0.070, 0.055, 0.036)),
        origin=Origin(xyz=(0.0, 0.060, 0.317)),
        material=enamel,
        name="head_bridge",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.060, 0.293)),
        material=charcoal,
        name="turret_socket",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(
            xyz=(0.0, 0.010, 0.357),
            rpy=(math.radians(55.0), 0.0, 0.0),
        ),
        material=enamel,
        name="tube_body",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.054),
        origin=Origin(
            xyz=(0.0, -0.048, 0.398),
            rpy=(math.radians(55.0), 0.0, 0.0),
        ),
        material=charcoal,
        name="eyepiece_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(
            xyz=(0.0, -0.070, 0.413),
            rpy=(math.radians(55.0), 0.0, 0.0),
        ),
        material=glass,
        name="eyepiece_lens",
    )
    frame.visual(
        Box((0.050, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.002, 0.118)),
        material=enamel,
        name="stage_saddle",
    )
    frame.visual(
        guide_track_mesh,
        origin=Origin(xyz=(0.0, 0.060, 0.133)),
        material=charcoal,
        name="guide_track",
    )
    frame.visual(
        Box((0.014, 0.094, 0.010)),
        origin=Origin(xyz=(-0.024, 0.060, 0.131)),
        material=dark_metal,
        name="guide_rail_left",
    )
    frame.visual(
        Box((0.014, 0.094, 0.010)),
        origin=Origin(xyz=(0.024, 0.060, 0.131)),
        material=dark_metal,
        name="guide_rail_right",
    )
    frame.visual(
        Box((0.060, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.108, 0.128)),
        material=dark_metal,
        name="guide_front_stop",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.220, 0.180, 0.420)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    stage_assembly = model.part("stage_assembly")
    stage_assembly.visual(
        Box((0.070, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_metal,
        name="stage_carriage",
    )
    stage_assembly.visual(
        Box((0.050, 0.040, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=charcoal,
        name="stage_support_block",
    )
    stage_assembly.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=charcoal,
        name="stage_spigot",
    )
    stage_assembly.visual(
        stage_plate_mesh,
        origin=Origin(xyz=(0.0, 0.022, 0.047)),
        material=charcoal,
        name="stage_plate",
    )
    stage_assembly.visual(
        Box((0.032, 0.004, 0.003)),
        origin=Origin(xyz=(-0.022, 0.0705, 0.0525)),
        material=satin,
        name="left_specimen_clip",
    )
    stage_assembly.visual(
        Box((0.032, 0.004, 0.003)),
        origin=Origin(xyz=(0.022, 0.0705, 0.0525)),
        material=satin,
        name="right_specimen_clip",
    )
    stage_assembly.visual(
        Box((0.030, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=enamel,
        name="lamp_housing_flange",
    )
    stage_assembly.visual(
        Box((0.078, 0.060, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=enamel,
        name="lamp_housing_body",
    )
    stage_assembly.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=lamp_blue,
        name="lamp_lens_tube",
    )
    stage_assembly.inertial = Inertial.from_geometry(
        Box((0.130, 0.130, 0.105)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=charcoal,
        name="turret_collar",
    )
    nosepiece.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=dark_metal,
        name="turret_body",
    )
    nosepiece.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=dark_metal,
        name="turret_disc",
    )
    nosepiece.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(0.0, 0.022, -0.059)),
        material=satin,
        name="objective_long",
    )
    nosepiece.visual(
        Cylinder(radius=0.0075, length=0.044),
        origin=Origin(xyz=(-0.019, -0.011, -0.056)),
        material=satin,
        name="objective_mid",
    )
    nosepiece.visual(
        Cylinder(radius=0.0065, length=0.034),
        origin=Origin(xyz=(0.019, -0.011, -0.051)),
        material=satin,
        name="objective_short",
    )
    nosepiece.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.090),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    lamp_door = model.part("lamp_door")
    lamp_door.visual(
        Box((0.0035, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, -0.019, 0.0)),
        material=charcoal,
        name="lamp_access_panel",
    )
    lamp_door.visual(
        Cylinder(radius=0.0025, length=0.028),
        origin=Origin(xyz=(-0.0018, 0.0005, 0.0)),
        material=satin,
        name="lamp_door_knob",
    )
    lamp_door.inertial = Inertial.from_geometry(
        Box((0.007, 0.040, 0.032)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.019, 0.0)),
    )

    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage_assembly,
        origin=Origin(xyz=(0.0, 0.060, 0.136)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.080,
            lower=-0.018,
            upper=0.028,
        ),
    )
    model.articulation(
        "turret_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.060, 0.287)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "lamp_door_hinge",
        ArticulationType.REVOLUTE,
        parent=stage_assembly,
        child=lamp_door,
        origin=Origin(xyz=(0.04075, 0.019, -0.031)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    stage_assembly = object_model.get_part("stage_assembly")
    nosepiece = object_model.get_part("nosepiece")
    lamp_door = object_model.get_part("lamp_door")

    stage_slide = object_model.get_articulation("stage_slide")
    turret_swivel = object_model.get_articulation("turret_swivel")
    lamp_door_hinge = object_model.get_articulation("lamp_door_hinge")

    stage_limits = stage_slide.motion_limits
    turret_limits = turret_swivel.motion_limits
    door_limits = lamp_door_hinge.motion_limits

    with ctx.pose({stage_slide: 0.0, lamp_door_hinge: 0.0}):
        ctx.expect_gap(
            stage_assembly,
            frame,
            axis="z",
            positive_elem="stage_carriage",
            negative_elem="guide_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="stage carriage sits on the lower guide",
        )
        ctx.expect_within(
            stage_assembly,
            frame,
            axes="x",
            inner_elem="stage_carriage",
            outer_elem="guide_track",
            margin=0.0,
            name="stage carriage stays centered laterally on guide",
        )
        ctx.expect_overlap(
            stage_assembly,
            frame,
            axes="y",
            elem_a="stage_carriage",
            elem_b="guide_track",
            min_overlap=0.060,
            name="stage carriage has deep guide engagement at rest",
        )

    rest_stage_pos = ctx.part_world_position(stage_assembly)
    with ctx.pose({stage_slide: stage_limits.upper}):
        extended_stage_pos = ctx.part_world_position(stage_assembly)
        ctx.expect_overlap(
            stage_assembly,
            frame,
            axes="y",
            elem_a="stage_carriage",
            elem_b="guide_track",
            min_overlap=0.040,
            name="stage carriage retains insertion when fully forward",
        )
    ctx.check(
        "stage assembly slides fore and aft",
        rest_stage_pos is not None
        and extended_stage_pos is not None
        and extended_stage_pos[1] > rest_stage_pos[1] + 0.020,
        details=f"rest={rest_stage_pos}, extended={extended_stage_pos}",
    )

    rest_objective_aabb = ctx.part_element_world_aabb(nosepiece, elem="objective_long")
    with ctx.pose({turret_swivel: (2.0 * math.pi) / 3.0}):
        rotated_objective_aabb = ctx.part_element_world_aabb(nosepiece, elem="objective_long")
    rest_objective_center = _aabb_center(rest_objective_aabb)
    rotated_objective_center = _aabb_center(rotated_objective_aabb)
    ctx.check(
        "nosepiece turret rotates objective positions around the tube axis",
        rest_objective_center is not None
        and rotated_objective_center is not None
        and abs(rotated_objective_center[0] - rest_objective_center[0]) > 0.014
        and abs(rotated_objective_center[1] - rest_objective_center[1]) > 0.010,
        details=f"rest={rest_objective_center}, rotated={rotated_objective_center}",
    )
    ctx.check(
        "turret uses a realistic indexing range",
        turret_limits is not None
        and turret_limits.lower is not None
        and turret_limits.upper is not None
        and turret_limits.upper - turret_limits.lower > 4.0,
        details=f"limits={turret_limits}",
    )

    closed_door_aabb = ctx.part_world_aabb(lamp_door)
    with ctx.pose({lamp_door_hinge: door_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(lamp_door)
    closed_door_center = _aabb_center(closed_door_aabb)
    open_door_center = _aabb_center(open_door_aabb)
    ctx.check(
        "lamp access door swings outward on a vertical side hinge",
        closed_door_center is not None
        and open_door_center is not None
        and open_door_center[0] > closed_door_center[0] + 0.010,
        details=f"closed={closed_door_center}, open={open_door_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
