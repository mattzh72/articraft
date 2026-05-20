from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_section(
    *,
    width: float,
    depth: float,
    radius: float,
    z: float,
    y_offset: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_offset, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _cushion_mesh(name: str):
    """A lightly domed, rounded task-chair seat cushion."""
    sections = [
        _rounded_section(width=0.500, depth=0.450, radius=0.055, z=0.040, y_offset=0.020),
        _rounded_section(width=0.545, depth=0.500, radius=0.070, z=0.073, y_offset=0.025),
        _rounded_section(width=0.515, depth=0.468, radius=0.065, z=0.115, y_offset=0.030),
    ]
    return mesh_from_geometry(LoftGeometry(sections, cap=True, closed=True), name)


def _backrest_mesh(name: str):
    """A thick rounded back pad, narrower at the waist and top."""
    sections = [
        _rounded_section(width=0.340, depth=0.070, radius=0.032, z=0.120, y_offset=-0.055),
        _rounded_section(width=0.460, depth=0.098, radius=0.046, z=0.340, y_offset=-0.065),
        _rounded_section(width=0.415, depth=0.080, radius=0.038, z=0.630, y_offset=-0.075),
    ]
    return mesh_from_geometry(LoftGeometry(sections, cap=True, closed=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_star_task_chair")

    black_plastic = model.material("black_plastic", rgba=(0.030, 0.032, 0.035, 1.0))
    satin_black = model.material("satin_black_metal", rgba=(0.075, 0.078, 0.085, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.018, 0.018, 0.020, 1.0))
    fabric = model.material("charcoal_fabric", rgba=(0.095, 0.105, 0.118, 1.0))
    underside = model.material("underseat_mechanism", rgba=(0.045, 0.047, 0.050, 1.0))

    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.030,
            0.032,
            rim=WheelRim(inner_radius=0.020, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.010,
                width=0.026,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.014, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.006),
        ),
        "caster_wheel_rim",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.045,
            0.036,
            inner_radius=0.0305,
            tread=TireTread(style="circumferential", depth=0.0025, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.0014),),
            sidewall=TireSidewall(style="rounded", bulge=0.055),
        ),
        "caster_tire",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.090, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=satin_black,
        name="hub",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=black_plastic,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_black,
        name="lower_collar",
    )

    spoke_radius = 0.455
    caster_pivot_z = 0.108
    for index in range(5):
        theta = math.pi / 2.0 + index * (2.0 * math.pi / 5.0)
        direction_x = math.cos(theta)
        direction_y = math.sin(theta)
        yaw = theta
        base.visual(
            Box((0.415, 0.064, 0.045)),
            origin=Origin(
                xyz=(direction_x * 0.235, direction_y * 0.235, 0.145),
                rpy=(0.0, 0.0, yaw),
            ),
            material=satin_black,
            name=f"spoke_{index}",
        )
        base.visual(
            Cylinder(radius=0.038, length=0.050),
            origin=Origin(xyz=(direction_x * spoke_radius, direction_y * spoke_radius, 0.145)),
            material=satin_black,
            name=f"caster_socket_{index}",
        )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.030, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=chrome,
        name="piston",
    )
    column.visual(
        Cylinder(radius=0.043, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=chrome,
        name="seat_bearing",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.056, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=chrome,
        name="swivel_collar",
    )
    seat.visual(
        Box((0.360, 0.320, 0.042)),
        origin=Origin(xyz=(0.0, 0.000, 0.021)),
        material=underside,
        name="mechanism_plate",
    )
    seat.visual(
        Box((0.360, 0.160, 0.030)),
        origin=Origin(xyz=(0.0, -0.215, 0.040)),
        material=underside,
        name="rear_tongue",
    )
    for x in (-0.155, 0.155):
        seat.visual(
            Box((0.034, 0.050, 0.088)),
            origin=Origin(xyz=(x, -0.285, 0.085)),
            material=underside,
            name=f"recline_cheek_{'negative' if x < 0 else 'positive'}",
        )
    seat.visual(
        _cushion_mesh("seat_cushion"),
        material=fabric,
        name="cushion",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.021, length=0.276),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    backrest.visual(
        Box((0.110, 0.052, 0.094)),
        origin=Origin(xyz=(0.0, -0.028, 0.057)),
        material=underside,
        name="lower_lug",
    )
    backrest.visual(
        Box((0.125, 0.043, 0.250)),
        origin=Origin(xyz=(0.0, -0.045, 0.190)),
        material=black_plastic,
        name="spine",
    )
    backrest.visual(
        _backrest_mesh("backrest_pad"),
        material=fabric,
        name="pad",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=360.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "column_to_seat",
        ArticulationType.REVOLUTE,
        parent=column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.285, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.1, lower=0.0, upper=0.48),
    )

    for index in range(5):
        theta = math.pi / 2.0 + index * (2.0 * math.pi / 5.0)
        pivot_x = math.cos(theta) * spoke_radius
        pivot_y = math.sin(theta) * spoke_radius
        fork_yaw = theta - math.pi / 2.0

        fork = model.part(f"caster_fork_{index}")
        fork.visual(
            Cylinder(radius=0.011, length=0.058),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=chrome,
            name="swivel_pin",
        )
        fork.visual(
            Box((0.036, 0.044, 0.018)),
            origin=Origin(xyz=(0.0, 0.016, -0.005)),
            material=chrome,
            name="neck",
        )
        fork.visual(
            Box((0.078, 0.026, 0.018)),
            origin=Origin(xyz=(0.0, 0.036, 0.000)),
            material=chrome,
            name="fork_bridge",
        )
        for side, x in (("negative", -0.032), ("positive", 0.032)):
            fork.visual(
                Box((0.008, 0.074, 0.106)),
                origin=Origin(xyz=(x, 0.057, -0.055)),
                material=chrome,
                name=f"fork_tine_{side}",
            )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(caster_wheel_mesh, material=chrome, name="rim")
        wheel.visual(caster_tire_mesh, material=rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.006, length=0.056),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="axle_hub",
        )

        model.articulation(
            f"base_to_caster_fork_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=fork,
            origin=Origin(xyz=(pivot_x, pivot_y, caster_pivot_z), rpy=(0.0, 0.0, fork_yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=5.0, lower=-math.pi, upper=math.pi),
        )
        model.articulation(
            f"caster_fork_{index}_to_caster_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.057, -0.055)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    height_joint = object_model.get_articulation("base_to_column")
    swivel_joint = object_model.get_articulation("column_to_seat")
    recline_joint = object_model.get_articulation("seat_to_backrest")

    ctx.allow_overlap(
        base,
        column,
        elem_a="outer_sleeve",
        elem_b="piston",
        reason="The chrome gas piston is intentionally represented as sliding inside the black outer gas-lift sleeve.",
    )
    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem="piston",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="gas piston remains centered in sleeve",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="piston",
        elem_b="outer_sleeve",
        min_overlap=0.140,
        name="low gas lift remains deeply inserted",
    )
    rest_column = ctx.part_world_position(column)
    with ctx.pose({height_joint: 0.100}):
        ctx.expect_within(
            column,
            base,
            axes="xy",
            inner_elem="piston",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="raised gas piston remains centered",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="piston",
            elem_b="outer_sleeve",
            min_overlap=0.055,
            name="raised gas lift retains sleeve insertion",
        )
        raised_column = ctx.part_world_position(column)
    ctx.check(
        "height adjustment slides upward",
        rest_column is not None
        and raised_column is not None
        and raised_column[2] > rest_column[2] + 0.095,
        details=f"rest={rest_column}, raised={raised_column}",
    )

    ctx.expect_contact(
        seat,
        column,
        elem_a="swivel_collar",
        elem_b="piston",
        contact_tol=0.002,
        name="seat swivel collar sits on gas piston",
    )

    ctx.check(
        "seat swivels about vertical column",
        swivel_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel_joint.articulation_type}, axis={swivel_joint.axis}",
    )
    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({swivel_joint: 0.75}):
        yawed_back_aabb = ctx.part_world_aabb(backrest)
    if rest_back_aabb is not None and yawed_back_aabb is not None:
        rest_center_x = (rest_back_aabb[0][0] + rest_back_aabb[1][0]) * 0.5
        yawed_center_x = (yawed_back_aabb[0][0] + yawed_back_aabb[1][0]) * 0.5
    else:
        rest_center_x = yawed_center_x = 0.0
    ctx.check(
        "seat and back rotate together in yaw",
        abs(yawed_center_x - rest_center_x) > 0.10,
        details=f"rest_center_x={rest_center_x}, yawed_center_x={yawed_center_x}",
    )

    ctx.check(
        "backrest reclines on transverse hinge",
        recline_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in recline_joint.axis) == (1.0, 0.0, 0.0)
        and recline_joint.motion_limits is not None
        and recline_joint.motion_limits.lower == 0.0
        and recline_joint.motion_limits.upper is not None
        and recline_joint.motion_limits.upper >= 0.45,
        details=(
            f"type={recline_joint.articulation_type}, axis={recline_joint.axis}, "
            f"limits={recline_joint.motion_limits}"
        ),
    )
    rest_recline_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({recline_joint: 0.45}):
        reclined_aabb = ctx.part_world_aabb(backrest)
    if rest_recline_aabb is not None and reclined_aabb is not None:
        rest_min_y = rest_recline_aabb[0][1]
        reclined_min_y = reclined_aabb[0][1]
    else:
        rest_min_y = reclined_min_y = 0.0
    ctx.check(
        "positive recline moves backrest rearward",
        reclined_min_y < rest_min_y - 0.045,
        details=f"rest_min_y={rest_min_y}, reclined_min_y={reclined_min_y}",
    )

    for index in range(5):
        fork_joint = object_model.get_articulation(f"base_to_caster_fork_{index}")
        wheel_joint = object_model.get_articulation(f"caster_fork_{index}_to_caster_wheel_{index}")
        ctx.check(
            f"caster_{index}_fork_swivel_axis",
            fork_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in fork_joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={fork_joint.articulation_type}, axis={fork_joint.axis}",
        )
        ctx.check(
            f"caster_{index}_wheel_spin_axis",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in wheel_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )

    caster_wheel_0 = object_model.get_part("caster_wheel_0")
    fork_0_joint = object_model.get_articulation("base_to_caster_fork_0")
    rest_wheel_0 = ctx.part_world_position(caster_wheel_0)
    with ctx.pose({fork_0_joint: 0.70}):
        swivelled_wheel_0 = ctx.part_world_position(caster_wheel_0)
    ctx.check(
        "caster fork swivel carries offset wheel",
        rest_wheel_0 is not None
        and swivelled_wheel_0 is not None
        and abs(swivelled_wheel_0[0] - rest_wheel_0[0]) > 0.025,
        details=f"rest={rest_wheel_0}, swivelled={swivelled_wheel_0}",
    )

    return ctx.report()


object_model = build_object_model()
