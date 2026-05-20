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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
):
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    mid = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spotting_scope_ball_head_tripod")

    tripod_black = model.material("tripod_black", rgba=(0.10, 0.10, 0.11, 1.0))
    leg_gray = model.material("leg_gray", rgba=(0.29, 0.30, 0.32, 1.0))
    scope_white = model.material("scope_white", rgba=(0.86, 0.87, 0.84, 1.0))
    hardware_black = model.material("hardware_black", rgba=(0.05, 0.05, 0.06, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.21, 0.22, 0.24, 1.0))

    tripod = model.part("tripod")
    hub_z = 0.76
    tripod_top_z = 1.06
    tripod.visual(
        Box((0.13, 0.13, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=tripod_black,
        name="tripod_hub",
    )
    tripod.visual(
        Cylinder(radius=0.026, length=tripod_top_z - hub_z),
        origin=Origin(xyz=(0.0, 0.0, (tripod_top_z + hub_z) * 0.5)),
        material=leg_gray,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, tripod_top_z - 0.006)),
        material=tripod_black,
        name="tripod_plate",
    )

    leg_top_radius = 0.045
    foot_radius = 0.48
    for leg_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        top = (
            leg_top_radius * math.cos(angle),
            leg_top_radius * math.sin(angle),
            hub_z - 0.01,
        )
        foot = (
            foot_radius * math.cos(angle),
            foot_radius * math.sin(angle),
            0.02,
        )
        _add_cylinder_between(
            tripod,
            top,
            foot,
            radius=0.015,
            material=leg_gray,
            name=f"leg_{leg_index}",
        )
        _add_cylinder_between(
            tripod,
            (
                foot[0] - 0.015 * math.cos(angle),
                foot[1] - 0.015 * math.sin(angle),
                0.02,
            ),
            (foot[0], foot[1], 0.0),
            radius=0.018,
            material=tripod_black,
            name=f"foot_{leg_index}",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=tripod_black,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=bracket_gray,
        name="pan_collar",
    )
    pan_head.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=bracket_gray,
        name="tilt_stem",
    )
    pan_head.visual(
        Box((0.050, 0.044, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=tripod_black,
        name="socket_body",
    )
    pan_head.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=hardware_black,
        name="pan_ball",
    )
    pan_head.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.082), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=hardware_black,
        name="tilt_axle",
    )

    scope = model.part("scope_assembly")
    scope.visual(
        Box((0.032, 0.012, 0.062)),
        origin=Origin(xyz=(0.0, 0.034, 0.031)),
        material=bracket_gray,
        name="left_cheek",
    )
    scope.visual(
        Box((0.032, 0.012, 0.062)),
        origin=Origin(xyz=(0.0, -0.034, 0.031)),
        material=bracket_gray,
        name="right_cheek",
    )
    scope.visual(
        Box((0.070, 0.086, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.066)),
        material=bracket_gray,
        name="tilt_bridge",
    )
    scope.visual(
        Box((0.180, 0.030, 0.010)),
        origin=Origin(xyz=(0.085, 0.0, 0.074)),
        material=bracket_gray,
        name="dovetail_plate",
    )
    scope.visual(
        Cylinder(radius=0.041, length=0.340),
        origin=Origin(xyz=(0.110, 0.0, 0.110), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=scope_white,
        name="tube_main",
    )
    scope.visual(
        Cylinder(radius=0.046, length=0.120),
        origin=Origin(xyz=(0.270, 0.0, 0.110), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_black,
        name="dew_shield",
    )
    scope.visual(
        Cylinder(radius=0.048, length=0.020),
        origin=Origin(xyz=(0.330, 0.0, 0.110), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_black,
        name="objective_cell",
    )
    scope.visual(
        Cylinder(radius=0.032, length=0.090),
        origin=Origin(xyz=(-0.090, 0.0, 0.110), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_black,
        name="focuser_body",
    )
    scope.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(-0.170, 0.0, 0.110), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_black,
        name="eyepiece",
    )
    scope.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(-0.105, 0.033, 0.110)),
        material=hardware_black,
        name="focus_knob_left",
    )
    scope.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(-0.105, -0.033, 0.110)),
        material=hardware_black,
        name="focus_knob_right",
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, tripod_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "pan_to_scope",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=scope,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.60,
            upper=1.10,
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

    tripod = object_model.get_part("tripod")
    pan_head = object_model.get_part("pan_head")
    scope = object_model.get_part("scope_assembly")
    pan = object_model.get_articulation("tripod_to_pan")
    tilt = object_model.get_articulation("pan_to_scope")

    ctx.check(
        "tripod, pan head, and scope exist",
        tripod is not None and pan_head is not None and scope is not None,
    )
    ctx.check(
        "pan joint is continuous around vertical axis",
        pan.articulation_type == ArticulationType.CONTINUOUS and pan.axis == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )
    ctx.check(
        "tilt joint pitches upward on negative Y axis",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.axis == (0.0, -1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
    )

    with ctx.pose({tilt: 0.0, pan: 0.0}):
        ctx.expect_contact(
            pan_head,
            tripod,
            elem_a="pan_base",
            elem_b="tripod_plate",
            name="pan base seats on tripod plate",
        )
        ctx.expect_gap(
            scope,
            pan_head,
            axis="z",
            positive_elem="tube_main",
            negative_elem="pan_ball",
            min_gap=0.015,
            name="tube clears ball head in level pose",
        )

    rest_objective = None
    raised_objective = None
    with ctx.pose({tilt: 0.0, pan: 0.0}):
        rest_objective = _aabb_center(ctx.part_element_world_aabb(scope, elem="objective_cell"))
    with ctx.pose({tilt: 0.80, pan: 0.0}):
        raised_objective = _aabb_center(ctx.part_element_world_aabb(scope, elem="objective_cell"))
    ctx.check(
        "positive tilt raises the objective",
        rest_objective is not None
        and raised_objective is not None
        and raised_objective[2] > rest_objective[2] + 0.12,
        details=f"rest={rest_objective}, raised={raised_objective}",
    )

    rest_objective_xy = None
    panned_objective_xy = None
    with ctx.pose({tilt: 0.0, pan: 0.0}):
        rest_objective_xy = _aabb_center(ctx.part_element_world_aabb(scope, elem="objective_cell"))
    with ctx.pose({tilt: 0.0, pan: math.pi * 0.5}):
        panned_objective_xy = _aabb_center(ctx.part_element_world_aabb(scope, elem="objective_cell"))
    ctx.check(
        "pan bearing rotates scope around tripod axis",
        rest_objective_xy is not None
        and panned_objective_xy is not None
        and panned_objective_xy[1] > rest_objective_xy[1] + 0.25
        and abs(panned_objective_xy[0]) < rest_objective_xy[0],
        details=f"rest={rest_objective_xy}, panned={panned_objective_xy}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
