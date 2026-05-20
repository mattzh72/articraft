from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_shell(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 42,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _add_wheel_visuals(
    part,
    prefix: str,
    *,
    tire_radius: float,
    tire_width: float,
    rubber,
    wheel_gray,
    hub_dark,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.56, -half_width),
        (tire_radius * 0.80, -half_width * 0.98),
        (tire_radius * 0.95, -half_width * 0.72),
        (tire_radius, -half_width * 0.30),
        (tire_radius, half_width * 0.30),
        (tire_radius * 0.95, half_width * 0.72),
        (tire_radius * 0.80, half_width * 0.98),
        (tire_radius * 0.56, half_width),
        (tire_radius * 0.46, half_width * 0.40),
        (tire_radius * 0.42, 0.0),
        (tire_radius * 0.46, -half_width * 0.40),
        (tire_radius * 0.56, -half_width),
    ]
    tire_mesh = _mesh(
        f"{prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=tire_radius * 0.63, length=tire_width * 0.60),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_gray,
        name="rim_barrel",
    )
    for sign in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=tire_radius * 0.55, length=0.008),
            origin=Origin(
                xyz=(sign * tire_width * 0.17, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=wheel_gray,
            name=f"rim_face_{'outer' if sign > 0.0 else 'inner'}",
        )
    part.visual(
        Cylinder(radius=tire_radius * 0.24, length=tire_width * 0.82),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_dark,
        name="hub",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.13, length=tire_width * 0.94),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_gray,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    body_plastic = model.material("body_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    latch_red = model.material("latch_red", rgba=(0.70, 0.14, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.58, 0.36, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=body_plastic,
        name="bottom_panel",
    )
    body.visual(
        Box((0.556, 0.012, 0.28)),
        origin=Origin(xyz=(0.0, 0.174, 0.170)),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((0.556, 0.012, 0.28)),
        origin=Origin(xyz=(0.0, -0.174, 0.170)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.012, 0.336, 0.28)),
        origin=Origin(xyz=(-0.284, 0.0, 0.170)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((0.012, 0.336, 0.28)),
        origin=Origin(xyz=(0.284, 0.0, 0.170)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((0.14, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.168, 0.275)),
        material=dark_steel,
        name="front_latch_receiver",
    )
    body.visual(
        Box((0.044, 0.052, 0.10)),
        origin=Origin(xyz=(-0.268, -0.150, 0.080)),
        material=body_plastic,
        name="left_axle_block",
    )
    body.visual(
        Box((0.044, 0.052, 0.10)),
        origin=Origin(xyz=(0.268, -0.150, 0.080)),
        material=body_plastic,
        name="right_axle_block",
    )
    body.visual(
        Box((0.050, 0.034, 0.030)),
        origin=Origin(xyz=(-0.182, 0.146, 0.015)),
        material=dark_steel,
        name="left_front_foot",
    )
    body.visual(
        Box((0.050, 0.034, 0.030)),
        origin=Origin(xyz=(0.182, 0.146, 0.015)),
        material=dark_steel,
        name="right_front_foot",
    )

    guide_y = -0.205
    guide_z = 0.180
    guide_length = 0.240
    guide_outer_radius = 0.019
    guide_inner_radius = 0.0155
    guide_x = 0.205

    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = x_sign * guide_x
        body.visual(
            _mesh(
                f"body_{side_name}_guide",
                _tube_shell(
                    guide_outer_radius,
                    guide_inner_radius,
                    guide_length,
                ),
            ),
            origin=Origin(xyz=(x_pos, guide_y, guide_z)),
            material=steel,
            name=f"{side_name}_body_guide",
        )
        body.visual(
            Box((0.024, 0.012, 0.082)),
            origin=Origin(xyz=(x_pos, -0.186, 0.112)),
            material=dark_steel,
            name=f"{side_name}_guide_lower_mount",
        )
        body.visual(
            Box((0.024, 0.012, 0.074)),
            origin=Origin(xyz=(x_pos, -0.186, 0.246)),
            material=dark_steel,
            name=f"{side_name}_guide_upper_mount",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.64, 0.44, 0.44)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.599, 0.366, 0.012)),
        origin=Origin(xyz=(0.0, 0.183, 0.006)),
        material=lid_plastic,
        name="lid_top",
    )
    lid.visual(
        Box((0.008, 0.350, 0.055)),
        origin=Origin(xyz=(-0.2945, 0.175, -0.0275)),
        material=lid_plastic,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((0.008, 0.350, 0.055)),
        origin=Origin(xyz=(0.2945, 0.175, -0.0275)),
        material=lid_plastic,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((0.583, 0.008, 0.055)),
        origin=Origin(xyz=(0.0, 0.362, -0.0275)),
        material=lid_plastic,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.148, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.368, -0.016)),
        material=latch_red,
        name="lid_latch_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.61, 0.38, 0.07)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.185, -0.010)),
    )

    handle_stage_1 = model.part("handle_stage_1")
    stage1_outer_radius = 0.013
    stage1_inner_radius = 0.010
    stage1_length = 0.280
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        handle_stage_1.visual(
            _mesh(
                f"{side_name}_stage1_sleeve_mesh",
                _tube_shell(
                    stage1_outer_radius,
                    stage1_inner_radius,
                    stage1_length,
                ),
            ),
            origin=Origin(xyz=(x_sign * guide_x, 0.0, -0.140)),
            material=steel,
            name=f"{side_name}_stage1_sleeve",
        )
    handle_stage_1.visual(
        Box((0.430, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, -0.255)),
        material=dark_steel,
        name="stage1_cross_tie",
    )
    handle_stage_1.visual(
        Box((0.130, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, -0.165)),
        material=dark_steel,
        name="stage1_center_grip_block",
    )
    handle_stage_1.visual(
        Box((0.024, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, 0.020, -0.205)),
        material=dark_steel,
        name="stage1_center_stem",
    )
    handle_stage_1.inertial = Inertial.from_geometry(
        Box((0.46, 0.03, 0.31)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
    )

    handle_stage_2 = model.part("handle_stage_2")
    handle_path = [
        (-guide_x, 0.0, -0.240),
        (-guide_x, 0.0, 0.120),
        (guide_x, 0.0, 0.120),
        (guide_x, 0.0, -0.240),
    ]
    handle_stage_2.visual(
        _mesh(
            "stage2_u_handle",
            wire_from_points(
                handle_path,
                radius=0.0085,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
        ),
        material=steel,
        name="stage2_u_handle",
    )
    handle_stage_2.visual(
        Cylinder(radius=0.0085, length=0.300),
        origin=Origin(xyz=(-guide_x, 0.0, -0.150)),
        material=steel,
        name="left_stage2_rail",
    )
    handle_stage_2.visual(
        Cylinder(radius=0.0085, length=0.300),
        origin=Origin(xyz=(guide_x, 0.0, -0.150)),
        material=steel,
        name="right_stage2_rail",
    )
    handle_stage_2.visual(
        Cylinder(radius=0.013, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        handle_stage_2.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(x_sign * guide_x, 0.0, 0.006)),
            material=dark_steel,
            name=f"{side_name}_stage2_stop",
        )
    handle_stage_2.inertial = Inertial.from_geometry(
        Box((0.46, 0.04, 0.38)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        "left_wheel",
        tire_radius=0.090,
        tire_width=0.046,
        rubber=rubber,
        wheel_gray=steel,
        hub_dark=dark_steel,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.046),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        "right_wheel",
        tire_radius=0.090,
        tire_width=0.046,
        rubber=rubber,
        wheel_gray=steel,
        hub_dark=dark_steel,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.046),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.180, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_handle_stage_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle_stage_1,
        origin=Origin(xyz=(0.0, guide_y, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=handle_stage_1,
        child=handle_stage_2,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.40,
            lower=0.0,
            upper=0.200,
        ),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-0.313, -0.135, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(0.313, -0.135, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle_stage_1 = object_model.get_part("handle_stage_1")
    handle_stage_2 = object_model.get_part("handle_stage_2")
    lid_hinge = object_model.get_articulation("body_to_lid")
    stage1_slide = object_model.get_articulation("body_to_handle_stage_1")
    stage2_slide = object_model.get_articulation("stage_1_to_stage_2")

    stage1_limits = stage1_slide.motion_limits
    stage2_limits = stage2_slide.motion_limits
    lid_limits = lid_hinge.motion_limits

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top",
            negative_elem="front_wall",
            max_penetration=0.0001,
            max_gap=0.010,
            name="lid top seats on the body opening height without penetration",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="lid_top",
            elem_b="bottom_panel",
            min_overlap=0.55,
            name="lid spans the toolbox width",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem="lid_latch_pad",
            negative_elem="front_latch_receiver",
            min_gap=0.0,
            max_gap=0.030,
            name="front latch area stays aligned near receiver",
        )

    ctx.expect_within(
        handle_stage_1,
        body,
        axes="xy",
        inner_elem="left_stage1_sleeve",
        outer_elem="left_body_guide",
        margin=0.003,
        name="left first handle stage stays centered in left rear guide",
    )
    ctx.expect_within(
        handle_stage_1,
        body,
        axes="xy",
        inner_elem="right_stage1_sleeve",
        outer_elem="right_body_guide",
        margin=0.003,
        name="right first handle stage stays centered in right rear guide",
    )
    ctx.expect_overlap(
        handle_stage_1,
        body,
        axes="z",
        elem_a="left_stage1_sleeve",
        elem_b="left_body_guide",
        min_overlap=0.22,
        name="first handle stage is deeply retained at rest",
    )
    ctx.expect_within(
        handle_stage_2,
        handle_stage_1,
        axes="xy",
        inner_elem="left_stage2_rail",
        outer_elem="left_stage1_sleeve",
        margin=0.003,
        name="left second handle rail stays centered in first stage",
    )
    ctx.expect_within(
        handle_stage_2,
        handle_stage_1,
        axes="xy",
        inner_elem="right_stage2_rail",
        outer_elem="right_stage1_sleeve",
        margin=0.003,
        name="right second handle rail stays centered in first stage",
    )
    ctx.expect_overlap(
        handle_stage_2,
        handle_stage_1,
        axes="z",
        elem_a="left_stage2_rail",
        elem_b="left_stage1_sleeve",
        min_overlap=0.24,
        name="second handle stage is deeply retained at rest",
    )

    stage1_rest = ctx.part_world_position(handle_stage_1)
    stage2_rest = ctx.part_world_position(handle_stage_2)
    lid_front_closed = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")

    if stage1_limits is not None and stage1_limits.upper is not None:
        with ctx.pose({stage1_slide: stage1_limits.upper}):
            ctx.expect_overlap(
                handle_stage_1,
                body,
                axes="z",
                elem_a="left_stage1_sleeve",
                elem_b="left_body_guide",
                min_overlap=0.08,
                name="first handle stage remains inserted when fully raised",
            )
            stage1_extended = ctx.part_world_position(handle_stage_1)
        ctx.check(
            "first handle stage extends upward",
            stage1_rest is not None
            and stage1_extended is not None
            and stage1_extended[2] > stage1_rest[2] + 0.10,
            details=f"rest={stage1_rest}, extended={stage1_extended}",
        )

    if stage2_limits is not None and stage2_limits.upper is not None:
        with ctx.pose({stage2_slide: stage2_limits.upper}):
            ctx.expect_overlap(
                handle_stage_2,
                handle_stage_1,
                axes="z",
                elem_a="left_stage2_rail",
                elem_b="left_stage1_sleeve",
                min_overlap=0.08,
                name="second handle stage remains inserted when fully raised",
            )
            stage2_extended = ctx.part_world_position(handle_stage_2)
        ctx.check(
            "second handle stage extends upward",
            stage2_rest is not None
            and stage2_extended is not None
            and stage2_extended[2] > stage2_rest[2] + 0.15,
            details=f"rest={stage2_rest}, extended={stage2_extended}",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            lid_front_open = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
        ctx.check(
            "lid swings upward from rear hinge",
            lid_front_closed is not None
            and lid_front_open is not None
            and lid_front_open[0][2] > lid_front_closed[0][2] + 0.14,
            details=f"closed={lid_front_closed}, open={lid_front_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
