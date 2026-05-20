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


BASE_RADIUS = 0.165
POST_TOP_Z = 1.31
UPPER_ARM_LENGTH = 0.34
LOWER_ARM_LENGTH = 0.31


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _add_quad(
    geometry: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
    *,
    flip: bool = False,
) -> None:
    if flip:
        geometry.add_face(a, c, b)
        geometry.add_face(a, d, c)
    else:
        geometry.add_face(a, b, c)
        geometry.add_face(a, c, d)


def _open_frustum_shell(
    *,
    rear_radius: float,
    front_radius: float,
    length: float,
    wall: float,
    segments: int = 40,
) -> MeshGeometry:
    geometry = MeshGeometry()
    outer_rear: list[int] = []
    outer_front: list[int] = []
    inner_rear: list[int] = []
    inner_front: list[int] = []

    inner_rear_radius = max(rear_radius - wall, wall * 0.5)
    inner_front_radius = max(front_radius - wall, wall * 0.5)

    for index in range(segments):
        angle = (2.0 * pi * index) / segments
        cy = cos(angle)
        sz = sin(angle)
        outer_rear.append(geometry.add_vertex(0.0, rear_radius * cy, rear_radius * sz))
        outer_front.append(geometry.add_vertex(length, front_radius * cy, front_radius * sz))
        inner_rear.append(
            geometry.add_vertex(0.0, inner_rear_radius * cy, inner_rear_radius * sz)
        )
        inner_front.append(
            geometry.add_vertex(length, inner_front_radius * cy, inner_front_radius * sz)
        )

    for index in range(segments):
        next_index = (index + 1) % segments
        _add_quad(
            geometry,
            outer_rear[index],
            outer_rear[next_index],
            outer_front[next_index],
            outer_front[index],
        )
        _add_quad(
            geometry,
            inner_rear[index],
            inner_front[index],
            inner_front[next_index],
            inner_rear[next_index],
            flip=True,
        )
        _add_quad(
            geometry,
            outer_rear[next_index],
            outer_rear[index],
            inner_rear[index],
            inner_rear[next_index],
        )
        _add_quad(
            geometry,
            outer_front[index],
            outer_front[next_index],
            inner_front[next_index],
            inner_front[index],
            flip=True,
        )

    return geometry


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp")

    brass = model.material("brass", rgba=(0.67, 0.56, 0.34, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.52, 0.43, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.19, 0.19, 0.20, 1.0))
    shade_green = model.material("shade_green", rgba=(0.15, 0.28, 0.18, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_metal,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.128, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=aged_brass,
        name="base_step",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=brass,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=0.013, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=brass,
        name="post_tube",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.270)),
        material=aged_brass,
        name="post_collar",
    )
    stand.visual(
        Box((0.050, 0.022, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, 1.286)),
        material=aged_brass,
        name="shoulder_neck",
    )
    stand.visual(
        Box((0.020, 0.034, 0.012)),
        origin=Origin(xyz=(0.034, 0.0, 1.286)),
        material=aged_brass,
        name="shoulder_bridge",
    )
    stand.visual(
        Box((0.036, 0.008, 0.042)),
        origin=Origin(xyz=(0.042, 0.017, POST_TOP_Z)),
        material=aged_brass,
        name="shoulder_cheek_left",
    )
    stand.visual(
        Box((0.036, 0.008, 0.042)),
        origin=Origin(xyz=(0.042, -0.017, POST_TOP_Z)),
        material=aged_brass,
        name="shoulder_cheek_right",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 1.34)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.055),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aged_brass,
        name="upper_shoulder_sleeve",
    )
    upper_arm.visual(
        Cylinder(radius=0.010, length=0.304),
        origin=Origin(xyz=(0.152, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="upper_tube",
    )
    upper_arm.visual(
        Box((0.024, 0.034, 0.014)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.026, 0.0, -0.014)),
        material=aged_brass,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.034, 0.008, 0.038)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.017, 0.0)),
        material=aged_brass,
        name="elbow_cheek_left",
    )
    upper_arm.visual(
        Box((0.034, 0.008, 0.038)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, -0.017, 0.0)),
        material=aged_brass,
        name="elbow_cheek_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH + 0.03, 0.05, 0.06)),
        mass=1.4,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="elbow_barrel",
    )
    lower_arm.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aged_brass,
        name="lower_elbow_sleeve",
    )
    lower_arm.visual(
        Cylinder(radius=0.009, length=0.274),
        origin=Origin(xyz=(0.137, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="lower_tube",
    )
    lower_arm.visual(
        Box((0.024, 0.034, 0.014)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH - 0.026, 0.0, -0.014)),
        material=aged_brass,
        name="shade_bridge",
    )
    lower_arm.visual(
        Box((0.034, 0.008, 0.036)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.017, 0.0)),
        material=aged_brass,
        name="shade_cheek_left",
    )
    lower_arm.visual(
        Box((0.034, 0.008, 0.036)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH, -0.017, 0.0)),
        material=aged_brass,
        name="shade_cheek_right",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH + 0.03, 0.05, 0.06)),
        mass=1.1,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade_shell_mesh = _save_mesh(
        "pharmacy_shade_shell",
        _open_frustum_shell(
            rear_radius=0.032,
            front_radius=0.074,
            length=0.150,
            wall=0.0025,
            segments=48,
        ),
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="tilt_barrel",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.065),
        origin=Origin(xyz=(0.0325, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aged_brass,
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=shade_green,
        name="shade_collar",
    )
    shade.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="lamp_socket",
    )
    shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=shade_green,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.16)),
        mass=0.9,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pivot",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_arm,
        origin=Origin(xyz=(0.042, 0.0, POST_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lower_arm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-1.20,
            upper=1.10,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=shade,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.90,
            upper=0.55,
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
    upper_arm = object_model.get_part("upper_arm")
    lower_arm = object_model.get_part("lower_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("shoulder_pivot")
    elbow = object_model.get_articulation("elbow_pivot")
    tilt = object_model.get_articulation("shade_tilt")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    ctx.expect_origin_gap(
        upper_arm,
        stand,
        axis="z",
        min_gap=1.30,
        max_gap=1.32,
        name="shoulder pivot sits at the post top",
    )
    ctx.expect_origin_gap(
        lower_arm,
        upper_arm,
        axis="x",
        min_gap=UPPER_ARM_LENGTH - 0.002,
        max_gap=UPPER_ARM_LENGTH + 0.002,
        name="elbow sits at the end of the upper arm",
    )
    ctx.expect_origin_gap(
        shade,
        lower_arm,
        axis="x",
        min_gap=LOWER_ARM_LENGTH - 0.002,
        max_gap=LOWER_ARM_LENGTH + 0.002,
        name="shade tilt joint sits at the arm tip",
    )

    rest_elbow = ctx.part_world_position(lower_arm)
    with ctx.pose({shoulder: 0.75}):
        raised_elbow = ctx.part_world_position(lower_arm)
    ctx.check(
        "shoulder positive rotation raises the elbow",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.18,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )

    with ctx.pose({shoulder: 0.30, elbow: 0.0}):
        straight_tip = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.30, elbow: 0.80}):
        bent_tip = ctx.part_world_position(shade)
    ctx.check(
        "elbow positive rotation lifts the shade support point",
        straight_tip is not None
        and bent_tip is not None
        and bent_tip[2] > straight_tip[2] + 0.10,
        details=f"straight={straight_tip}, bent={bent_tip}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.20, tilt: -0.55}):
        shade_down_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({shoulder: 0.35, elbow: 0.20, tilt: 0.35}):
        shade_up_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "shade tilt changes the head aim",
        shade_down_center is not None
        and shade_up_center is not None
        and shade_up_center[2] > shade_down_center[2] + 0.05,
        details=f"down={shade_down_center}, up={shade_up_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
