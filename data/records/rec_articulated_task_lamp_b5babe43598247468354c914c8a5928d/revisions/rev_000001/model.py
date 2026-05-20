from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _open_frustum_shell(
    *,
    rear_radius: float,
    front_radius: float,
    length: float,
    wall: float,
    segments: int = 40,
) -> MeshGeometry:
    geom = MeshGeometry()

    outer_rear: list[int] = []
    outer_front: list[int] = []
    inner_front: list[int] = []
    inner_rear: list[int] = []

    inner_rear_radius = max(rear_radius - wall, wall * 0.6)
    inner_front_radius = max(front_radius - wall, wall * 0.6)

    for index in range(segments):
        angle = 2.0 * pi * index / segments
        c = cos(angle)
        s = sin(angle)
        outer_rear.append(geom.add_vertex(0.0, rear_radius * c, rear_radius * s))
        outer_front.append(geom.add_vertex(length, front_radius * c, front_radius * s))
        inner_front.append(geom.add_vertex(length, inner_front_radius * c, inner_front_radius * s))
        inner_rear.append(geom.add_vertex(0.0, inner_rear_radius * c, inner_rear_radius * s))

    for index in range(segments):
        nxt = (index + 1) % segments
        _add_quad(geom, outer_rear[index], outer_rear[nxt], outer_front[nxt], outer_front[index])
        _add_quad(geom, inner_front[index], inner_front[nxt], inner_rear[nxt], inner_rear[index])
        _add_quad(geom, outer_rear[index], inner_rear[index], inner_rear[nxt], outer_rear[nxt])

    return geom


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_machine_lamp")

    painted_steel = model.material("painted_steel", rgba=(0.43, 0.46, 0.48, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.20, 0.21, 0.23, 1.0))
    shade_green = model.material("shade_green", rgba=(0.19, 0.31, 0.22, 1.0))
    black_trim = model.material("black_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.76, 0.77, 0.79, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.35)),
        mass=4.2,
        origin=Origin(xyz=(0.02, 0.0, -0.17)),
    )
    clamp_base.visual(
        Cylinder(radius=0.024, length=0.22),
        origin=Origin(xyz=(-0.050, 0.0, -0.110)),
        material=dark_cast,
        name="support_post",
    )
    clamp_base.visual(
        Box((0.072, 0.104, 0.088)),
        origin=Origin(xyz=(-0.012, 0.0, -0.205)),
        material=dark_cast,
        name="clamp_body",
    )
    clamp_base.visual(
        Box((0.120, 0.050, 0.018)),
        origin=Origin(xyz=(0.046, 0.0, -0.162)),
        material=painted_steel,
        name="top_jaw",
    )
    clamp_base.visual(
        Cylinder(radius=0.007, length=0.132),
        origin=Origin(xyz=(0.020, 0.0, -0.274), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="clamp_screw",
    )
    clamp_base.visual(
        Box((0.050, 0.060, 0.044)),
        origin=Origin(xyz=(-0.005, 0.0, -0.269)),
        material=dark_cast,
        name="screw_block",
    )
    clamp_base.visual(
        Cylinder(radius=0.0045, length=0.086),
        origin=Origin(xyz=(-0.046, 0.0, -0.274), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_trim,
        name="clamp_handle",
    )
    clamp_base.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(xyz=(0.086, 0.0, -0.304)),
        material=brushed_aluminum,
        name="pad_stem",
    )
    clamp_base.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.086, 0.0, -0.338)),
        material=black_trim,
        name="pressure_pad",
    )
    clamp_base.visual(
        Box((0.036, 0.014, 0.050)),
        origin=Origin(xyz=(0.000, 0.027, 0.000)),
        material=dark_cast,
        name="left_shoulder_ear",
    )
    clamp_base.visual(
        Box((0.036, 0.014, 0.050)),
        origin=Origin(xyz=(0.000, -0.027, 0.000)),
        material=dark_cast,
        name="right_shoulder_ear",
    )
    clamp_base.visual(
        Box((0.032, 0.068, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, -0.031)),
        material=dark_cast,
        name="shoulder_bridge",
    )
    clamp_base.visual(
        Box((0.050, 0.084, 0.034)),
        origin=Origin(xyz=(-0.028, 0.0, -0.060)),
        material=dark_cast,
        name="pivot_carrier",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.44, 0.07, 0.06)),
        mass=1.5,
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
    )
    lower_arm.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="rear_hub",
    )
    lower_arm.visual(
        Box((0.076, 0.036, 0.032)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=painted_steel,
        name="rear_transition",
    )
    lower_arm.visual(
        Box((0.300, 0.036, 0.026)),
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        material=painted_steel,
        name="main_beam",
    )
    lower_arm.visual(
        Box((0.056, 0.014, 0.048)),
        origin=Origin(xyz=(0.392, 0.027, 0.0)),
        material=painted_steel,
        name="left_elbow_ear",
    )
    lower_arm.visual(
        Box((0.056, 0.014, 0.048)),
        origin=Origin(xyz=(0.392, -0.027, 0.0)),
        material=painted_steel,
        name="right_elbow_ear",
    )
    lower_arm.visual(
        Box((0.048, 0.068, 0.012)),
        origin=Origin(xyz=(0.384, 0.0, -0.028)),
        material=painted_steel,
        name="elbow_bridge",
    )
    lower_arm.visual(
        Box((0.044, 0.068, 0.020)),
        origin=Origin(xyz=(0.348, 0.0, -0.018)),
        material=painted_steel,
        name="elbow_web",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.39, 0.07, 0.06)),
        mass=1.2,
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
    )
    upper_arm.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="rear_hub",
    )
    upper_arm.visual(
        Box((0.072, 0.036, 0.032)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=painted_steel,
        name="rear_transition",
    )
    upper_arm.visual(
        Box((0.260, 0.036, 0.024)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=painted_steel,
        name="main_beam",
    )
    upper_arm.visual(
        Box((0.054, 0.014, 0.046)),
        origin=Origin(xyz=(0.342, 0.027, 0.0)),
        material=painted_steel,
        name="left_tip_ear",
    )
    upper_arm.visual(
        Box((0.054, 0.014, 0.046)),
        origin=Origin(xyz=(0.342, -0.027, 0.0)),
        material=painted_steel,
        name="right_tip_ear",
    )
    upper_arm.visual(
        Box((0.044, 0.068, 0.012)),
        origin=Origin(xyz=(0.336, 0.0, -0.028)),
        material=painted_steel,
        name="tip_bridge",
    )
    upper_arm.visual(
        Box((0.040, 0.068, 0.020)),
        origin=Origin(xyz=(0.306, 0.0, -0.018)),
        material=painted_steel,
        name="tip_web",
    )

    lamp_shade = model.part("lamp_shade")
    lamp_shade.inertial = Inertial.from_geometry(
        Box((0.24, 0.20, 0.20)),
        mass=0.9,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
    )
    lamp_shade.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_trim,
        name="hinge_barrel",
    )
    lamp_shade.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_trim,
        name="neck_tube",
    )
    lamp_shade.visual(
        Cylinder(radius=0.050, length=0.044),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_trim,
        name="socket_housing",
    )
    shade_shell_mesh = mesh_from_geometry(
        _open_frustum_shell(
            rear_radius=0.054,
            front_radius=0.100,
            length=0.160,
            wall=0.006,
            segments=44,
        ),
        "shade_shell",
    )
    lamp_shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        material=shade_green,
        name="shade_shell",
    )
    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(),
        # The lower arm extends along +X from the shoulder, so -Y raises it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.75,
            upper=1.20,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.400, 0.0, 0.0)),
        # The upper arm also extends along +X from the elbow, so -Y lifts it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=-0.15,
            upper=1.70,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lamp_shade,
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        # The shade extends along +X from the hinge, so +Y tips it downward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=-0.90,
            upper=0.75,
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
    lamp_shade = object_model.get_part("lamp_shade")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.check(
        "lamp has four mechanically legible assemblies",
        len(object_model.parts) == 4,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "arm joints pitch about a horizontal axis",
        shoulder_joint.axis == (0.0, -1.0, 0.0) and elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder_joint.axis}, elbow_axis={elbow_joint.axis}",
    )
    ctx.check(
        "shade tilt hinge is horizontal",
        shade_tilt.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"shade_axis={shade_tilt.axis}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, shade_tilt: 0.0}):
        ctx.expect_contact(
            lower_arm,
            clamp_base,
            elem_a="rear_hub",
            elem_b="left_shoulder_ear",
            name="lower arm is carried by the clamp shoulder fork",
        )
        ctx.expect_contact(
            upper_arm,
            lower_arm,
            elem_a="rear_hub",
            elem_b="left_elbow_ear",
            name="upper arm is carried by the lower arm fork",
        )
        ctx.expect_contact(
            lamp_shade,
            upper_arm,
            elem_a="hinge_barrel",
            elem_b="left_tip_ear",
            name="shade is carried by the tip fork",
        )
        ctx.expect_origin_gap(
            lamp_shade,
            clamp_base,
            axis="x",
            min_gap=0.70,
            name="lamp head projects well forward of the bench clamp",
        )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, shade_tilt: 0.0}):
        rest_elbow = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder_joint: 0.80, elbow_joint: 0.0, shade_tilt: 0.0}):
        raised_elbow = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder joint raises the elbow",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.20
        and raised_elbow[0] < rest_elbow[0] - 0.08,
        details=f"rest_elbow={rest_elbow}, raised_elbow={raised_elbow}",
    )

    with ctx.pose({shoulder_joint: 0.55, elbow_joint: 0.0, shade_tilt: 0.0}):
        rest_tip = ctx.part_world_position(lamp_shade)
    with ctx.pose({shoulder_joint: 0.55, elbow_joint: 0.90, shade_tilt: 0.0}):
        raised_tip = ctx.part_world_position(lamp_shade)
    ctx.check(
        "elbow joint raises the lamp head farther upward",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[2] > rest_tip[2] + 0.15,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    with ctx.pose({shoulder_joint: 0.55, elbow_joint: 0.45, shade_tilt: -0.45}):
        shade_up = _aabb_center(ctx.part_element_world_aabb(lamp_shade, elem="shade_shell"))
    with ctx.pose({shoulder_joint: 0.55, elbow_joint: 0.45, shade_tilt: 0.45}):
        shade_down = _aabb_center(ctx.part_element_world_aabb(lamp_shade, elem="shade_shell"))
    ctx.check(
        "shade hinge changes beam aim",
        shade_up is not None
        and shade_down is not None
        and shade_down[2] < shade_up[2] - 0.04,
        details=f"shade_up={shade_up}, shade_down={shade_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
