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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="japanese_floor_lamp")

    walnut = model.material("walnut", rgba=(0.22, 0.15, 0.10, 1.0))
    bamboo = model.material("bamboo", rgba=(0.69, 0.60, 0.42, 1.0))
    paper = model.material("paper", rgba=(0.96, 0.93, 0.84, 0.88))
    warm_light = model.material("warm_light", rgba=(0.96, 0.86, 0.58, 0.42))
    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.15, 1.0))
    brass = model.material("brass", rgba=(0.63, 0.52, 0.25, 1.0))

    shade_shell_mesh = mesh_from_geometry(
        CylinderGeometry(radius=0.205, height=0.50, radial_segments=56, closed=False),
        "shade_shell",
    )
    shade_outer_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.196, tube=0.009, radial_segments=18, tubular_segments=56),
        "shade_outer_ring",
    )
    shade_collar_mesh = mesh_from_geometry(
        CylinderGeometry(radius=0.030, height=0.30, radial_segments=36, closed=False),
        "shade_collar",
    )
    shade_collar_lip_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.028, tube=0.004, radial_segments=14, tubular_segments=40),
        "shade_collar_lip",
    )
    pull_chain_loop_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.010, 0.0, 0.008),
                (-0.014, 0.0, -0.018),
                (-0.008, 0.0, -0.095),
                (-0.003, 0.0, -0.170),
                (0.003, 0.0, -0.174),
                (0.008, 0.0, -0.100),
                (0.014, 0.0, -0.020),
                (0.010, 0.0, 0.008),
            ],
            radius=0.0022,
            samples_per_segment=18,
            closed_spline=True,
            radial_segments=14,
            cap_ends=False,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "pull_chain_loop",
    )

    lamp_body = model.part("lamp_body")
    lamp_body.visual(
        Cylinder(radius=0.165, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=walnut,
        name="base_disc",
    )
    lamp_body.visual(
        Cylinder(radius=0.110, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=walnut,
        name="base_riser",
    )
    lamp_body.visual(
        Cylinder(radius=0.0165, length=1.364),
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        material=walnut,
        name="lamp_post",
    )
    lamp_body.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.836)),
        material=bamboo,
        name="shade_seat_collar",
    )
    lamp_body.visual(
        Cylinder(radius=0.032, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 1.290)),
        material=walnut,
        name="socket_housing",
    )
    lamp_body.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 1.135)),
        material=warm_light,
        name="lamp_bulb",
    )
    lamp_body.visual(
        Box((0.080, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.405)),
        material=walnut,
        name="top_housing",
    )
    lamp_body.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 1.449)),
        material=walnut,
        name="top_finial",
    )
    _add_member(
        lamp_body,
        (0.022, 0.0, 1.390),
        (0.084, 0.045, 1.366),
        0.006,
        walnut,
        name="pulley_housing_arm",
    )
    lamp_body.visual(
        Box((0.018, 0.003, 0.034)),
        origin=Origin(xyz=(0.090, 0.053, 1.350)),
        material=charcoal,
        name="pulley_housing_left_cheek",
    )
    lamp_body.visual(
        Box((0.018, 0.003, 0.034)),
        origin=Origin(xyz=(0.090, 0.037, 1.350)),
        material=charcoal,
        name="pulley_housing_right_cheek",
    )
    lamp_body.visual(
        Box((0.018, 0.019, 0.004)),
        origin=Origin(xyz=(0.090, 0.045, 1.366)),
        material=charcoal,
        name="pulley_housing_bridge",
    )
    lamp_body.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 1.43)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
    )

    shade = model.part("shade")
    shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=paper,
        name="shade_shell",
    )
    shade.visual(
        shade_outer_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=bamboo,
        name="shade_bottom_ring",
    )
    shade.visual(
        shade_outer_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.492)),
        material=bamboo,
        name="shade_top_ring",
    )
    shade.visual(
        shade_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=bamboo,
        name="shade_collar",
    )
    shade.visual(
        shade_collar_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=bamboo,
        name="shade_collar_lip",
    )
    shade.visual(
        shade_collar_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.296)),
        material=bamboo,
        name="shade_collar_top_lip",
    )
    for index, angle in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        shade.visual(
            Cylinder(radius=0.0032, length=0.300),
            origin=Origin(
                xyz=(0.028 * math.cos(angle), 0.028 * math.sin(angle), 0.150)
            ),
            material=bamboo,
            name=f"collar_guide_{index}",
        )

    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        radius = 0.188
        shade.visual(
            Cylinder(radius=0.0045, length=0.450),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.260)
            ),
            material=bamboo,
            name=f"shade_spline_{index:02d}",
        )

    for index, angle in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        a_bottom = (0.030 * math.cos(angle), 0.030 * math.sin(angle), 0.028)
        b_bottom = (0.188 * math.cos(angle), 0.188 * math.sin(angle), 0.028)
        a_mid = (0.030 * math.cos(angle), 0.030 * math.sin(angle), 0.150)
        b_mid = (0.188 * math.cos(angle), 0.188 * math.sin(angle), 0.150)
        a_top = (0.030 * math.cos(angle), 0.030 * math.sin(angle), 0.492)
        b_top = (0.188 * math.cos(angle), 0.188 * math.sin(angle), 0.492)
        _add_member(
            shade,
            a_bottom,
            b_bottom,
            0.0036,
            bamboo,
            name=f"bottom_spoke_{index}",
        )
        _add_member(
            shade,
            a_mid,
            b_mid,
            0.0034,
            bamboo,
            name=f"mid_spoke_{index}",
        )
        _add_member(
            shade,
            a_top,
            b_top,
            0.0036,
            bamboo,
            name=f"top_spoke_{index}",
        )

    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.52),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )

    pull_chain = model.part("pull_chain")
    pull_chain.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="chain_pulley_wheel",
    )
    pull_chain.visual(
        pull_chain_loop_mesh,
        material=brass,
        name="pull_chain_loop",
    )
    pull_chain.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.178)),
        material=walnut,
        name="chain_pull_knob",
    )
    pull_chain.inertial = Inertial.from_geometry(
        Box((0.034, 0.020, 0.194)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
    )

    model.articulation(
        "shade_slide",
        ArticulationType.PRISMATIC,
        parent=lamp_body,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "pull_chain_spin",
        ArticulationType.CONTINUOUS,
        parent=lamp_body,
        child=pull_chain,
        origin=Origin(xyz=(0.090, 0.045, 1.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
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

    lamp_body = object_model.get_part("lamp_body")
    shade = object_model.get_part("shade")
    pull_chain = object_model.get_part("pull_chain")
    shade_slide = object_model.get_articulation("shade_slide")
    pull_chain_spin = object_model.get_articulation("pull_chain_spin")

    ctx.check(
        "shade uses a vertical prismatic fit",
        shade_slide.articulation_type == ArticulationType.PRISMATIC
        and shade_slide.motion_limits is not None
        and shade_slide.motion_limits.lower == 0.0
        and shade_slide.motion_limits.upper is not None
        and shade_slide.motion_limits.upper >= 0.15
        and abs(shade_slide.axis[2]) > 0.99,
        details=f"type={shade_slide.articulation_type}, axis={shade_slide.axis}, limits={shade_slide.motion_limits}",
    )
    ctx.check(
        "pull chain uses a continuous pulley rotation",
        pull_chain_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(pull_chain_spin.axis[1]) > 0.99,
        details=f"type={pull_chain_spin.articulation_type}, axis={pull_chain_spin.axis}",
    )

    with ctx.pose({shade_slide: 0.0}):
        ctx.expect_gap(
            shade,
            lamp_body,
            axis="z",
            positive_elem="shade_collar_lip",
            negative_elem="shade_seat_collar",
            max_gap=0.0015,
            max_penetration=0.0,
            name="shade seats on the post collar",
        )
        ctx.expect_overlap(
            shade,
            lamp_body,
            axes="xy",
            elem_a="shade_collar",
            elem_b="lamp_post",
            min_overlap=0.030,
            name="shade collar stays centered over the post",
        )

    rest_position = ctx.part_world_position(shade)
    with ctx.pose({shade_slide: 0.160}):
        lifted_position = ctx.part_world_position(shade)
        ctx.expect_overlap(
            shade,
            lamp_body,
            axes="xy",
            elem_a="shade_collar",
            elem_b="lamp_post",
            min_overlap=0.030,
            name="lifted shade remains centered on the post",
        )
        ctx.expect_overlap(
            shade,
            lamp_body,
            axes="z",
            elem_a="shade_collar",
            elem_b="lamp_post",
            min_overlap=0.130,
            name="shade collar retains insertion when lifted",
        )

    ctx.check(
        "shade lifts upward from the seated position",
        rest_position is not None
        and lifted_position is not None
        and lifted_position[2] > rest_position[2] + 0.12,
        details=f"rest={rest_position}, lifted={lifted_position}",
    )

    chain_wheel_aabb = ctx.part_element_world_aabb(pull_chain, elem="chain_pulley_wheel")
    chain_loop_aabb = ctx.part_element_world_aabb(pull_chain, elem="pull_chain_loop")
    ctx.check(
        "pull chain hangs below the pulley wheel",
        chain_wheel_aabb is not None
        and chain_loop_aabb is not None
        and chain_loop_aabb[0][2] < chain_wheel_aabb[0][2] - 0.10,
        details=f"wheel={chain_wheel_aabb}, loop={chain_loop_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
