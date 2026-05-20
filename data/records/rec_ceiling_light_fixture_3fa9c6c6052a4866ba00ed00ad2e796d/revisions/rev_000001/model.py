from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


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


def _add_cylinder_between(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _point_along(
    start: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (
        start[0] + direction[0] * distance,
        start[1] + direction[1] * distance,
        start[2] + direction[2] * distance,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_head_track_light")

    satin_white = model.material("satin_white", rgba=(0.86, 0.86, 0.82, 1.0))
    warm_white = model.material("warm_white", rgba=(0.96, 0.94, 0.88, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.04, 0.04, 0.045, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.63, 0.61, 1.0))
    black_baffle = model.material("black_baffle", rgba=(0.015, 0.014, 0.012, 1.0))
    warm_lens = model.material("warm_lens", rgba=(1.0, 0.82, 0.45, 0.55))
    copper = model.material("copper_contacts", rgba=(0.73, 0.38, 0.12, 1.0))

    rail = model.part("ceiling_rail")
    rail.visual(
        Box((1.25, 0.075, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_white,
        name="top_spine",
    )
    rail.visual(
        Box((1.25, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, 0.0305, -0.016)),
        material=satin_white,
        name="side_wall_0",
    )
    rail.visual(
        Box((1.25, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, -0.0305, -0.016)),
        material=satin_white,
        name="side_wall_1",
    )
    rail.visual(
        Box((1.25, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.018, -0.033)),
        material=satin_white,
        name="lower_lip_0",
    )
    rail.visual(
        Box((1.25, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.018, -0.033)),
        material=satin_white,
        name="lower_lip_1",
    )
    rail.visual(
        Box((1.20, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=dark_slot,
        name="shadow_slot",
    )
    rail.visual(
        Box((1.18, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0105, -0.0335)),
        material=copper,
        name="conductor_0",
    )
    rail.visual(
        Box((1.18, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.0105, -0.0335)),
        material=copper,
        name="conductor_1",
    )
    for x in (-0.635, 0.635):
        rail.visual(
            Box((0.020, 0.078, 0.045)),
            origin=Origin(xyz=(x, 0.0, -0.014)),
            material=warm_white,
            name=f"end_cap_{0 if x < 0 else 1}",
        )
    for x in (-0.54, 0.54):
        rail.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, 0.0, -0.039)),
            material=brushed_metal,
            name=f"mount_screw_{0 if x < 0 else 1}",
        )
    rail.inertial = Inertial.from_geometry(
        Box((1.27, 0.08, 0.05)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    def add_spotlight(index: int, x: float):
        head = model.part(f"spotlight_{index}")

        head.visual(
            Box((0.110, 0.052, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=warm_white,
            name="track_adapter",
        )
        head.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.019)),
            material=brushed_metal,
            name="swivel_boss",
        )
        head.visual(
            Cylinder(radius=0.010, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, -0.058)),
            material=brushed_metal,
            name="drop_stem",
        )
        head.visual(
            Sphere(radius=0.023),
            origin=Origin(xyz=(0.0, 0.0, -0.096)),
            material=brushed_metal,
            name="ball_knuckle",
        )

        knuckle = (0.0, 0.0, -0.096)
        aim = (0.0, 0.50, -0.8660254038)
        neck_end = _point_along(knuckle, aim, 0.060)
        rear = _point_along(knuckle, aim, 0.048)
        front = _point_along(knuckle, aim, 0.203)

        _add_cylinder_between(
            head,
            knuckle,
            neck_end,
            0.018,
            brushed_metal,
            name="angled_neck",
        )
        _add_cylinder_between(
            head,
            rear,
            front,
            0.047,
            warm_white,
            name="lamp_can",
        )
        _add_cylinder_between(
            head,
            _point_along(knuckle, aim, 0.132),
            _point_along(knuckle, aim, 0.138),
            0.050,
            brushed_metal,
            name="cooling_band_0",
        )
        _add_cylinder_between(
            head,
            _point_along(knuckle, aim, 0.156),
            _point_along(knuckle, aim, 0.162),
            0.050,
            brushed_metal,
            name="cooling_band_1",
        )
        _add_cylinder_between(
            head,
            _point_along(knuckle, aim, 0.192),
            _point_along(knuckle, aim, 0.207),
            0.052,
            warm_white,
            name="front_bezel",
        )
        _add_cylinder_between(
            head,
            _point_along(knuckle, aim, 0.205),
            _point_along(knuckle, aim, 0.211),
            0.037,
            black_baffle,
            name="front_lens",
        )
        _add_cylinder_between(
            head,
            _point_along(knuckle, aim, 0.210),
            _point_along(knuckle, aim, 0.214),
            0.030,
            warm_lens,
            name="glass_lens",
        )
        _add_cylinder_between(
            head,
            (-0.040, 0.004, -0.108),
            (-0.040, 0.030, -0.150),
            0.006,
            brushed_metal,
            name="yoke_arm_0",
        )
        _add_cylinder_between(
            head,
            (0.040, 0.004, -0.108),
            (0.040, 0.030, -0.150),
            0.006,
            brushed_metal,
            name="yoke_arm_1",
        )

        head.inertial = Inertial.from_geometry(
            Box((0.14, 0.20, 0.24)),
            mass=0.65,
            origin=Origin(xyz=(0.0, 0.055, -0.16)),
        )

        joint = model.articulation(
            f"rail_to_spotlight_{index}",
            ArticulationType.REVOLUTE,
            parent=rail,
            child=head,
            origin=Origin(xyz=(x, 0.0, -0.046)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.8,
                velocity=2.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )
        return head, joint

    for i, x in enumerate((-0.40, 0.0, 0.40)):
        add_spotlight(i, x)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("ceiling_rail")
    joints = [object_model.get_articulation(f"rail_to_spotlight_{i}") for i in range(3)]
    heads = [object_model.get_part(f"spotlight_{i}") for i in range(3)]

    ctx.check("three independently articulated spotlight heads", len(joints) == 3)

    for index, (head, joint) in enumerate(zip(heads, joints)):
        ctx.check(
            f"spotlight {index} uses a revolute yaw joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower <= -math.pi + 1e-6
            and joint.motion_limits.upper >= math.pi - 1e-6,
        )
        ctx.expect_gap(
            rail,
            head,
            axis="z",
            positive_elem="lower_lip_0",
            negative_elem="track_adapter",
            max_gap=0.001,
            max_penetration=0.00001,
            name=f"spotlight {index} adapter seats under the rail lip",
        )
        ctx.expect_overlap(
            head,
            rail,
            axes="xy",
            elem_a="track_adapter",
            elem_b="shadow_slot",
            min_overlap=0.010,
            name=f"spotlight {index} adapter is centered in the track slot",
        )

    first_head = heads[0]
    first_joint = joints[0]
    with ctx.pose({first_joint: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(first_head, elem="front_lens")
    with ctx.pose({first_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(first_head, elem="front_lens")

    def center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_center = center_xy(rest_aabb)
    turned_center = center_xy(turned_aabb)
    ctx.check(
        "spotlight yaw visibly rotates the lamp aperture",
        rest_center is not None
        and turned_center is not None
        and turned_center[0] < rest_center[0] - 0.060
        and turned_center[1] < rest_center[1] - 0.060,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
