from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spotting_scope_tripod")

    anodized_black = model.material("anodized_black", rgba=(0.14, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.22, 0.28, 0.85))

    def pitched_point(distance: float, pitch: float) -> tuple[float, float, float]:
        return (
            distance * math.cos(pitch),
            0.0,
            -distance * math.sin(pitch),
        )

    def x_cylinder(
        part,
        *,
        radius: float,
        length: float,
        center_x: float,
        center_z: float,
        material,
        name: str,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(center_x, 0.0, center_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=name,
        )

    def y_cylinder(
        part,
        *,
        radius: float,
        length: float,
        center_x: float,
        center_y: float,
        center_z: float,
        material,
        name: str,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(center_x, center_y, center_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=name,
        )

    tripod_body = model.part("tripod_body")

    sleeve_height = 0.18
    sleeve_center_z = 0.11
    sleeve_outer = 0.056
    sleeve_wall = 0.012

    tripod_body.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.022, sleeve_center_z)),
        material=graphite,
        name="sleeve_front",
    )
    tripod_body.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.022, sleeve_center_z)),
        material=graphite,
        name="sleeve_back",
    )
    tripod_body.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(0.022, 0.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_right",
    )
    tripod_body.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(-0.022, 0.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_left",
    )

    tripod_body.visual(
        Box((sleeve_outer, sleeve_wall, 0.028)),
        origin=Origin(xyz=(0.0, 0.022, 0.214)),
        material=anodized_black,
        name="top_collar_front",
    )
    tripod_body.visual(
        Box((sleeve_outer, sleeve_wall, 0.028)),
        origin=Origin(xyz=(0.0, -0.022, 0.214)),
        material=anodized_black,
        name="top_collar_back",
    )
    tripod_body.visual(
        Box((sleeve_wall, sleeve_outer, 0.028)),
        origin=Origin(xyz=(0.022, 0.0, 0.214)),
        material=anodized_black,
        name="top_collar_right",
    )
    tripod_body.visual(
        Box((sleeve_wall, sleeve_outer, 0.028)),
        origin=Origin(xyz=(-0.022, 0.0, 0.214)),
        material=anodized_black,
        name="top_collar_left",
    )

    clamp_z = 0.15
    tripod_body.visual(
        Cylinder(radius=0.0065, length=0.024),
        origin=Origin(
            xyz=(0.038, 0.0, clamp_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=anodized_black,
        name="clamp_stem",
    )
    tripod_body.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.054, 0.0, clamp_z)),
        material=rubber,
        name="clamp_knob",
    )

    hinge_radius = 0.104
    hinge_z = 0.02
    arm_length = 0.062
    arm_width = 0.036
    arm_height = 0.018
    cheek_y = 0.016

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        arm_center_r = 0.022 + arm_length / 2.0
        tripod_body.visual(
            Box((arm_length, arm_width, arm_height)),
            origin=Origin(
                xyz=(arm_center_r * c, arm_center_r * s, hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"crown_arm_{index}",
        )
        for side, y_offset in enumerate((cheek_y, -cheek_y)):
            tripod_body.visual(
                Box((0.020, 0.008, 0.028)),
                origin=Origin(
                    xyz=(
                        hinge_radius * c - 0.010 * c - y_offset * s,
                        hinge_radius * s - 0.010 * s + y_offset * c,
                        hinge_z,
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=anodized_black,
                name=f"hinge_cheek_{index}_{side}",
            )

    center_column = model.part("center_column")
    center_column.visual(
        Box((0.032, 0.032, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=anodized_black,
        name="column_shaft",
    )
    center_column.visual(
        Box((0.038, 0.038, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.326)),
        material=graphite,
        name="column_cap",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=graphite,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=anodized_black,
        name="pan_pillar",
    )
    pan_head.visual(
        Box((0.060, 0.042, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=graphite,
        name="tilt_block",
    )
    pan_head.visual(
        Box((0.018, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, 0.021, 0.079)),
        material=graphite,
        name="yoke_arm_0",
    )
    pan_head.visual(
        Box((0.018, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, -0.021, 0.079)),
        material=graphite,
        name="yoke_arm_1",
    )
    x_cylinder(
        pan_head,
        radius=0.006,
        length=0.20,
        center_x=-0.10,
        center_z=0.045,
        material=anodized_black,
        name="pan_handle",
    )
    pan_head.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.206, 0.0, 0.045)),
        material=rubber,
        name="handle_grip",
    )

    scope = model.part("scope")
    scope.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="tilt_boss",
    )
    scope.visual(
        Box((0.110, 0.028, 0.024)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=graphite,
        name="mount_foot",
    )
    x_cylinder(
        scope,
        radius=0.040,
        length=0.260,
        center_x=0.150,
        center_z=0.052,
        material=anodized_black,
        name="main_body",
    )
    x_cylinder(
        scope,
        radius=0.044,
        length=0.030,
        center_x=0.070,
        center_z=0.052,
        material=graphite,
        name="focus_ring",
    )
    x_cylinder(
        scope,
        radius=0.050,
        length=0.100,
        center_x=0.325,
        center_z=0.052,
        material=anodized_black,
        name="objective_bell",
    )
    x_cylinder(
        scope,
        radius=0.054,
        length=0.052,
        center_x=0.401,
        center_z=0.052,
        material=graphite,
        name="sunshade",
    )
    x_cylinder(
        scope,
        radius=0.024,
        length=0.120,
        center_x=-0.030,
        center_z=0.055,
        material=anodized_black,
        name="eyepiece_housing",
    )
    x_cylinder(
        scope,
        radius=0.028,
        length=0.050,
        center_x=-0.115,
        center_z=0.055,
        material=rubber,
        name="eyecup",
    )
    x_cylinder(
        scope,
        radius=0.046,
        length=0.003,
        center_x=0.4285,
        center_z=0.052,
        material=glass,
        name="objective_glass",
    )

    leg_pitch = math.radians(62.0)
    upper_leg_length = 0.54
    lower_leg_length = 0.40
    foot_distance = 0.84

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.009, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized_black,
            name="hinge_boss",
        )

        top_cap_center = pitched_point(0.030, leg_pitch)
        leg.visual(
            Box((0.060, 0.016, 0.018)),
            origin=Origin(xyz=top_cap_center, rpy=(0.0, leg_pitch, 0.0)),
            material=graphite,
            name="upper_socket",
        )

        upper_center = pitched_point(0.30, leg_pitch)
        leg.visual(
            Box((upper_leg_length, 0.030, 0.026)),
            origin=Origin(xyz=upper_center, rpy=(0.0, leg_pitch, 0.0)),
            material=anodized_black,
            name="upper_leg",
        )

        lower_center = pitched_point(0.64, leg_pitch)
        leg.visual(
            Box((lower_leg_length, 0.022, 0.020)),
            origin=Origin(xyz=lower_center, rpy=(0.0, leg_pitch, 0.0)),
            material=graphite,
            name="lower_leg",
        )

        foot_center = pitched_point(foot_distance, leg_pitch)
        leg.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=foot_center),
            material=rubber,
            name="foot",
        )

        model.articulation(
            f"body_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=tripod_body,
            child=leg,
            origin=Origin(xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=1.6,
                lower=-0.82,
                upper=0.18,
            ),
        )

    lift = model.articulation(
        "body_to_center_column",
        ArticulationType.PRISMATIC,
        parent=tripod_body,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=0.22,
        ),
    )

    pan = model.articulation(
        "column_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=center_column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )

    tilt = model.articulation(
        "head_to_scope",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=scope,
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.85,
        ),
    )

    lift.meta["qc_samples"] = [0.0, 0.10, 0.22]
    pan.meta["qc_samples"] = [0.0, math.pi / 3.0, math.pi]
    tilt.meta["qc_samples"] = [0.0, 0.45, 0.85]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_body = object_model.get_part("tripod_body")
    center_column = object_model.get_part("center_column")
    pan_head = object_model.get_part("pan_head")
    scope = object_model.get_part("scope")
    leg_0 = object_model.get_part("leg_0")

    lift = object_model.get_articulation("body_to_center_column")
    pan = object_model.get_articulation("column_to_pan_head")
    tilt = object_model.get_articulation("head_to_scope")
    leg_fold = object_model.get_articulation("body_to_leg_0")

    for index in range(3):
        for side in range(2):
            ctx.allow_overlap(
                f"leg_{index}",
                tripod_body,
                elem_a="hinge_boss",
                elem_b=f"hinge_cheek_{index}_{side}",
                reason="The leg hinge boss is intentionally represented as captured inside the crown hinge cheek proxy.",
            )

    def aabb_center(bounds):
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    lift_upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    leg_lower = leg_fold.motion_limits.lower if leg_fold.motion_limits is not None else None

    ctx.expect_origin_distance(
        center_column,
        tripod_body,
        axes="xy",
        max_dist=0.001,
        name="center column is centered in the crown",
    )
    ctx.expect_overlap(
        center_column,
        tripod_body,
        axes="z",
        min_overlap=0.03,
        name="collapsed center column remains inserted",
    )

    rest_column_pos = ctx.part_world_position(center_column)
    with ctx.pose({lift: lift_upper}):
        ctx.expect_origin_distance(
            center_column,
            tripod_body,
            axes="xy",
            max_dist=0.001,
            name="extended center column stays centered",
        )
        ctx.expect_overlap(
            center_column,
            tripod_body,
            axes="z",
            min_overlap=0.025,
            name="extended center column retains insertion",
        )
        extended_column_pos = ctx.part_world_position(center_column)

    ctx.check(
        "center column extends upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.18,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    rest_objective = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_glass"))
    with ctx.pose({pan: math.pi / 2.0}):
        panned_objective = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_glass"))

    ctx.check(
        "pan head swings the scope sideways",
        rest_objective is not None
        and panned_objective is not None
        and abs(rest_objective[0]) > 0.35
        and abs(panned_objective[1]) > 0.35
        and abs(panned_objective[0]) < 0.08,
        details=f"rest={rest_objective}, panned={panned_objective}",
    )

    rest_tilted_objective = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_glass"))
    with ctx.pose({tilt: tilt_upper}):
        raised_objective = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_glass"))

    ctx.check(
        "scope tilts upward at the head",
        rest_tilted_objective is not None
        and raised_objective is not None
        and raised_objective[2] > rest_tilted_objective[2] + 0.12,
        details=f"rest={rest_tilted_objective}, raised={raised_objective}",
    )

    rest_foot = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    with ctx.pose({leg_fold: leg_lower}):
        folded_foot = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))

    ctx.check(
        "tripod leg folds toward the crown",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.25,
        details=f"rest={rest_foot}, folded={folded_foot}",
    )

    ctx.expect_contact(
        pan_head,
        scope,
        elem_a="yoke_arm_0",
        elem_b="tilt_boss",
        contact_tol=0.003,
        name="scope boss stays cradled between the yoke arms",
    )

    return ctx.report()


object_model = build_object_model()
