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
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _reflector_shell(
    *,
    length: float,
    rear_radius: float,
    front_radius: float,
    wall: float,
    segments: int = 40,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_back: list[int] = []
    outer_front: list[int] = []
    inner_back: list[int] = []
    inner_front: list[int] = []

    inner_rear_radius = max(rear_radius - wall, wall * 1.5)
    inner_front_radius = max(front_radius - wall, wall * 1.5)

    for index in range(segments):
        angle = (2.0 * math.pi * index) / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_back.append(geom.add_vertex(0.0, rear_radius * c, rear_radius * s))
        outer_front.append(geom.add_vertex(length, front_radius * c, front_radius * s))
        inner_back.append(geom.add_vertex(0.0, inner_rear_radius * c, inner_rear_radius * s))
        inner_front.append(geom.add_vertex(length, inner_front_radius * c, inner_front_radius * s))

    for index in range(segments):
        nxt = (index + 1) % segments
        _add_quad(
            geom,
            outer_back[index],
            outer_back[nxt],
            outer_front[nxt],
            outer_front[index],
        )
        _add_quad(
            geom,
            inner_back[index],
            inner_front[index],
            inner_front[nxt],
            inner_back[nxt],
        )
        _add_quad(
            geom,
            outer_back[index],
            inner_back[index],
            inner_back[nxt],
            outer_back[nxt],
        )
        _add_quad(
            geom,
            outer_front[index],
            outer_front[nxt],
            inner_front[nxt],
            inner_front[index],
        )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vice_clamp_task_lamp")

    clamp_black = model.material("clamp_black", rgba=(0.16, 0.17, 0.18, 1.0))
    arm_black = model.material("arm_black", rgba=(0.20, 0.21, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        _save_mesh(
            "clamp_c_frame",
            tube_from_spline_points(
                [
                    (-0.004, 0.0, 0.006),
                    (-0.024, 0.0, -0.014),
                    (-0.034, 0.0, -0.052),
                    (-0.024, 0.0, -0.092),
                    (0.004, 0.0, -0.108),
                    (0.032, 0.0, -0.112),
                    (0.048, 0.0, -0.105),
                ],
                radius=0.013,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=clamp_black,
        name="c_frame",
    )
    clamp_base.visual(
        Box((0.080, 0.050, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.005)),
        material=clamp_black,
        name="upper_jaw",
    )
    clamp_base.visual(
        Box((0.020, 0.040, 0.018)),
        origin=Origin(xyz=(0.050, 0.0, -0.106)),
        material=clamp_black,
        name="lower_jaw_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.032, 0.0, -0.060)),
        material=steel,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.032, 0.0, -0.005)),
        material=steel,
        name="clamp_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.0045, length=0.054),
        origin=Origin(xyz=(0.032, 0.0, -0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    clamp_base.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.032, 0.031, -0.105)),
        material=rubber,
        name="handle_knob_left",
    )
    clamp_base.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.032, -0.031, -0.105)),
        material=rubber,
        name="handle_knob_right",
    )
    clamp_base.visual(
        Cylinder(radius=0.012, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=clamp_black,
        name="post",
    )
    clamp_base.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=clamp_black,
        name="post_socket",
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.110, 0.070, 0.290)),
        mass=2.6,
        origin=Origin(xyz=(0.015, 0.0, 0.020)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.019, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=arm_black,
        name="shoulder_collar",
    )
    lower_arm.visual(
        _save_mesh(
            "lower_link_tube",
            tube_from_spline_points(
                [
                    (0.006, 0.0, 0.018),
                    (0.055, 0.0, 0.106),
                    (0.146, 0.0, 0.222),
                    (0.200, 0.0, 0.258),
                ],
                radius=0.012,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=arm_black,
        name="lower_link_tube",
    )
    lower_arm.visual(
        Box((0.024, 0.050, 0.036)),
        origin=Origin(xyz=(0.203, 0.0, 0.258)),
        material=arm_black,
        name="elbow_back_block",
    )
    lower_arm.visual(
        Box((0.018, 0.010, 0.036)),
        origin=Origin(xyz=(0.214, -0.022, 0.258)),
        material=arm_black,
        name="elbow_fork_left",
    )
    lower_arm.visual(
        Box((0.018, 0.010, 0.036)),
        origin=Origin(xyz=(0.214, 0.022, 0.258)),
        material=arm_black,
        name="elbow_fork_right",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.270, 0.060, 0.310)),
        mass=0.8,
        origin=Origin(xyz=(0.120, 0.0, 0.150)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="upper_knuckle",
    )
    upper_arm.visual(
        _save_mesh(
            "upper_link_tube",
            tube_from_spline_points(
                [
                    (0.014, 0.0, 0.0),
                    (0.102, 0.0, 0.016),
                    (0.208, 0.0, 0.032),
                    (0.270, 0.0, 0.038),
                ],
                radius=0.011,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=arm_black,
        name="upper_link_tube",
    )
    upper_arm.visual(
        Box((0.022, 0.042, 0.030)),
        origin=Origin(xyz=(0.278, 0.0, 0.038)),
        material=arm_black,
        name="tip_block",
    )
    upper_arm.visual(
        Box((0.020, 0.008, 0.026)),
        origin=Origin(xyz=(0.288, -0.018, 0.038)),
        material=arm_black,
        name="tip_fork_left",
    )
    upper_arm.visual(
        Box((0.020, 0.008, 0.026)),
        origin=Origin(xyz=(0.288, 0.018, 0.038)),
        material=arm_black,
        name="tip_fork_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.340, 0.050, 0.080)),
        mass=0.7,
        origin=Origin(xyz=(0.155, 0.0, 0.020)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shade_barrel",
    )
    shade.visual(
        Box((0.018, 0.020, 0.028)),
        origin=Origin(xyz=(0.006, 0.0, -0.018)),
        material=steel,
        name="shade_bracket",
    )
    shade.visual(
        Cylinder(radius=0.032, length=0.028),
        origin=Origin(xyz=(0.024, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shade_rear_cap",
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.012, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="socket_neck",
    )
    shade.visual(
        _save_mesh(
            "reflector_shell",
            _reflector_shell(
                length=0.116,
                rear_radius=0.032,
                front_radius=0.078,
                wall=0.006,
                segments=48,
            ),
        ),
        origin=Origin(xyz=(0.024, 0.0, -0.028)),
        material=satin_aluminum,
        name="shade_shell",
    )
    shade.visual(
        _save_mesh(
            "reflector_rim",
            TorusGeometry(radius=0.075, tube=0.003, radial_segments=14, tubular_segments=48).rotate_y(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.140, 0.0, -0.028)),
        material=steel,
        name="front_rim",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.170, 0.160, 0.120)),
        mass=0.55,
        origin=Origin(xyz=(0.085, 0.0, -0.028)),
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-2.35,
            upper=2.35,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.237, 0.0, 0.258)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.95,
            upper=1.10,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.308, 0.0, 0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.95,
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
    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    base_swivel = object_model.get_articulation("base_swivel")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    shade_tilt = object_model.get_articulation("shade_tilt")

    post_socket = clamp_base.get_visual("post_socket")
    shoulder_collar = lower_arm.get_visual("shoulder_collar")
    elbow_fork_left = lower_arm.get_visual("elbow_fork_left")
    upper_knuckle = upper_arm.get_visual("upper_knuckle")
    tip_fork_left = upper_arm.get_visual("tip_fork_left")
    shade_barrel = shade.get_visual("shade_barrel")
    front_rim = shade.get_visual("front_rim")

    ctx.expect_contact(
        lower_arm,
        clamp_base,
        contact_tol=5e-6,
        elem_a=shoulder_collar,
        elem_b=post_socket,
        name="lower arm sits on the clamp post socket",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        contact_tol=5e-6,
        elem_a=upper_knuckle,
        elem_b=elbow_fork_left,
        name="upper arm knuckle is captured by the elbow fork",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        contact_tol=5e-6,
        elem_a=shade_barrel,
        elem_b=tip_fork_left,
        name="shade barrel is captured by the tip fork",
    )

    rest_swivel_pos = ctx.part_world_position(shade)
    with ctx.pose({base_swivel: 0.75}):
        swung_pos = ctx.part_world_position(shade)
    ctx.check(
        "base swivel rotates the lamp around the post",
        rest_swivel_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_swivel_pos[1] + 0.15,
        details=f"rest={rest_swivel_pos}, swung={swung_pos}",
    )

    with ctx.pose({base_swivel: 0.25, elbow_pitch: 0.0}):
        elbow_rest_pos = ctx.part_world_position(shade)
    with ctx.pose({base_swivel: 0.25, elbow_pitch: 0.80}):
        elbow_raised_pos = ctx.part_world_position(shade)
    ctx.check(
        "elbow pitch raises the shade",
        elbow_rest_pos is not None
        and elbow_raised_pos is not None
        and elbow_raised_pos[2] > elbow_rest_pos[2] + 0.10,
        details=f"rest={elbow_rest_pos}, raised={elbow_raised_pos}",
    )

    with ctx.pose({base_swivel: 0.10, elbow_pitch: 0.35, shade_tilt: 0.0}):
        rim_level = ctx.part_element_world_aabb(shade, elem=front_rim)
    with ctx.pose({base_swivel: 0.10, elbow_pitch: 0.35, shade_tilt: 0.55}):
        rim_tilted = ctx.part_element_world_aabb(shade, elem=front_rim)
    ctx.check(
        "shade tilt aims the reflector downward",
        rim_level is not None
        and rim_tilted is not None
        and rim_tilted[0][2] < rim_level[0][2] - 0.02,
        details=f"level={rim_level}, tilted={rim_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
