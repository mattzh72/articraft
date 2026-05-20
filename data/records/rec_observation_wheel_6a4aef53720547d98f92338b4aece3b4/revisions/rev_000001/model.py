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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _radial_point(radius: float, angle: float, *, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(angle), y, radius * math.sin(angle))


def _build_wheel_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    hub_radius: float,
    ring_half_width: float,
    pivot_radius: float,
    gondola_count: int,
) -> object:
    wheel_geom = TorusGeometry(
        radius=outer_radius,
        tube=0.012,
        radial_segments=18,
        tubular_segments=96,
    ).rotate_x(math.pi / 2.0)
    wheel_geom.translate(0.0, ring_half_width, 0.0)
    wheel_geom.merge(
        TorusGeometry(
            radius=outer_radius,
            tube=0.012,
            radial_segments=18,
            tubular_segments=96,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -ring_half_width, 0.0)
    )
    wheel_geom.merge(
        TorusGeometry(
            radius=inner_radius,
            tube=0.0085,
            radial_segments=16,
            tubular_segments=84,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, ring_half_width, 0.0)
    )
    wheel_geom.merge(
        TorusGeometry(
            radius=inner_radius,
            tube=0.0085,
            radial_segments=16,
            tubular_segments=84,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -ring_half_width, 0.0)
    )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        for y_sign in (-ring_half_width, ring_half_width):
            wheel_geom.merge(
                wire_from_points(
                    [
                        _radial_point(hub_radius, angle, y=0.040 if y_sign > 0.0 else -0.040),
                        _radial_point(inner_radius - 0.006, angle, y=y_sign),
                    ],
                    radius=0.006,
                    radial_segments=12,
                    cap_ends=True,
                )
            )
            wheel_geom.merge(
                wire_from_points(
                    [
                        _radial_point(inner_radius + 0.004, angle, y=y_sign),
                        _radial_point(outer_radius - 0.008, angle, y=y_sign),
                    ],
                    radius=0.0048,
                    radial_segments=10,
                    cap_ends=True,
                )
            )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        wheel_geom.merge(
            wire_from_points(
                [
                    _radial_point(inner_radius, angle + math.pi / 8.0, y=ring_half_width),
                    _radial_point(inner_radius, angle + math.pi / 8.0, y=-ring_half_width),
                ],
                radius=0.0045,
                radial_segments=10,
                cap_ends=True,
            )
        )
    for index in range(gondola_count):
        angle = -math.pi / 2.0 + (2.0 * math.pi * index) / gondola_count
        wheel_geom.merge(
            wire_from_points(
                [
                    _radial_point(outer_radius - 0.004, angle, y=ring_half_width * 0.98),
                    _radial_point(pivot_radius, angle, y=0.040),
                ],
                radius=0.0044,
                radial_segments=10,
                cap_ends=False,
            )
        )
        wheel_geom.merge(
            wire_from_points(
                [
                    _radial_point(outer_radius - 0.004, angle, y=-ring_half_width * 0.98),
                    _radial_point(pivot_radius, angle, y=-0.040),
                ],
                radius=0.0044,
                radial_segments=10,
                cap_ends=False,
            )
        )
    wheel_geom.merge(
        CylinderGeometry(radius=0.041, height=0.128).rotate_x(math.pi / 2.0)
    )
    wheel_geom.merge(
        CylinderGeometry(radius=0.060, height=0.016)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.040, 0.0)
    )
    wheel_geom.merge(
        CylinderGeometry(radius=0.060, height=0.016)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.040, 0.0)
    )
    return wheel_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decorative_wheel")

    base_teal = model.material("base_teal", rgba=(0.13, 0.34, 0.39, 1.0))
    cream = model.material("cream", rgba=(0.91, 0.88, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.62, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.24, 1.0))
    gondola_red = model.material("gondola_red", rgba=(0.73, 0.23, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))

    base_height = 0.06
    axle_height = 0.53
    frame_y = 0.125
    ring_half_width = 0.065
    outer_radius = 0.285
    inner_radius = 0.225
    hub_radius = 0.058
    pivot_radius = 0.325
    gondola_count = 8
    pivot_stub_names = (
        "pivot_stub_00",
        "pivot_stub_01",
        "pivot_stub_02",
        "pivot_stub_03",
        "pivot_stub_04",
        "pivot_stub_05",
        "pivot_stub_06",
        "pivot_stub_07",
    )

    support = model.part("support_frame")
    support.visual(
        Box((0.84, 0.34, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=base_teal,
        name="base_plinth",
    )
    support.visual(
        Box((0.78, 0.26, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, base_height + 0.01)),
        material=charcoal,
        name="deck_cap",
    )
    for y_sign in (-frame_y, frame_y):
        bracket_name = "front_bearing_bridge" if y_sign > 0.0 else "rear_bearing_bridge"
        housing_name = "front_bearing_housing" if y_sign > 0.0 else "rear_bearing_housing"
        _add_member(
            support,
            (-0.24, y_sign, base_height),
            (0.0, y_sign, axle_height),
            0.013,
            cream,
        )
        _add_member(
            support,
            (0.24, y_sign, base_height),
            (0.0, y_sign, axle_height),
            0.013,
            cream,
        )
        _add_member(
            support,
            (-0.18, y_sign, 0.18),
            (0.18, y_sign, 0.18),
            0.010,
            cream,
        )
        _add_member(
            support,
            (-0.22, y_sign, base_height),
            (0.0, y_sign, 0.34),
            0.008,
            cream,
        )
        _add_member(
            support,
            (0.22, y_sign, base_height),
            (0.0, y_sign, 0.34),
            0.008,
            cream,
        )
        support.visual(
            Box((0.12, 0.028, 0.09)),
            origin=Origin(xyz=(0.0, y_sign, axle_height - 0.025)),
            material=base_teal,
            name=bracket_name,
        )
        support.visual(
            Cylinder(radius=0.030, length=0.062),
            origin=Origin(
                xyz=(0.0, y_sign * 0.752, axle_height),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name=housing_name,
        )
    for x_sign in (-0.27, 0.27):
        support.visual(
            Box((0.12, 0.05, 0.025)),
            origin=Origin(xyz=(x_sign, 0.0, 0.0125)),
            material=charcoal,
        )
    support.inertial = Inertial.from_geometry(
        Box((0.84, 0.34, 0.62)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    wheel = model.part("wheel")
    wheel_mesh = mesh_from_geometry(
        _build_wheel_geometry(
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            hub_radius=hub_radius,
            ring_half_width=ring_half_width,
            pivot_radius=pivot_radius,
            gondola_count=gondola_count,
        ),
        "decorative_wheel_truss",
    )
    wheel.visual(wheel_mesh, material=cream, name="wheel_truss")
    for index in range(gondola_count):
        angle = -math.pi / 2.0 + (2.0 * math.pi * index) / gondola_count
        wheel.visual(
            Cylinder(radius=0.0042, length=0.086),
            origin=Origin(
                xyz=_radial_point(pivot_radius, angle),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=pivot_stub_names[index],
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.33, length=0.15),
        mass=4.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )

    for index in range(gondola_count):
        angle = -math.pi / 2.0 + (2.0 * math.pi * index) / gondola_count
        gondola = model.part(f"gondola_{index:02d}")
        gondola.visual(
            Cylinder(radius=0.0054, length=0.082),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="pivot_tube",
        )
        gondola.visual(
            Box((0.008, 0.010, 0.070)),
            origin=Origin(xyz=(0.0, -0.031, -0.037)),
            material=steel,
            name="left_hanger",
        )
        gondola.visual(
            Box((0.008, 0.010, 0.070)),
            origin=Origin(xyz=(0.0, 0.031, -0.037)),
            material=steel,
            name="right_hanger",
        )
        gondola.visual(
            Cylinder(radius=0.004, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="roof_bar",
        )
        gondola.visual(
            Box((0.070, 0.086, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.076)),
            material=cream,
            name="canopy",
        )
        gondola.visual(
            Box((0.008, 0.010, 0.036)),
            origin=Origin(xyz=(0.0, -0.032, -0.098)),
            material=steel,
            name="left_drop_link",
        )
        gondola.visual(
            Box((0.008, 0.010, 0.036)),
            origin=Origin(xyz=(0.0, 0.032, -0.098)),
            material=steel,
            name="right_drop_link",
        )
        gondola.visual(
            Box((0.058, 0.084, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.122)),
            material=gondola_red,
            name="floor",
        )
        gondola.visual(
            Box((0.010, 0.076, 0.040)),
            origin=Origin(xyz=(-0.024, 0.0, -0.102)),
            material=gondola_red,
            name="back_panel",
        )
        gondola.visual(
            Box((0.010, 0.076, 0.030)),
            origin=Origin(xyz=(0.024, 0.0, -0.106)),
            material=cream,
            name="front_bar",
        )
        gondola.visual(
            Box((0.042, 0.008, 0.030)),
            origin=Origin(xyz=(0.0, -0.038, -0.106)),
            material=gondola_red,
            name="left_end",
        )
        gondola.visual(
            Box((0.042, 0.008, 0.030)),
            origin=Origin(xyz=(0.0, 0.038, -0.106)),
            material=gondola_red,
            name="right_end",
        )
        gondola.visual(
            Box((0.030, 0.066, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.108)),
            material=cream,
            name="seat",
        )
        gondola.inertial = Inertial.from_geometry(
            Box((0.07, 0.09, 0.15)),
            mass=0.24,
            origin=Origin(xyz=(0.0, 0.0, -0.09)),
        )
        model.articulation(
            f"wheel_to_gondola_{index:02d}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=_radial_point(pivot_radius, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=3.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    bottom_gondola = object_model.get_part("gondola_00")
    bottom_pivot = object_model.get_articulation("wheel_to_gondola_00")
    pivot_stub_names = (
        "pivot_stub_00",
        "pivot_stub_01",
        "pivot_stub_02",
        "pivot_stub_03",
        "pivot_stub_04",
        "pivot_stub_05",
        "pivot_stub_06",
        "pivot_stub_07",
    )

    gondola_names = [f"gondola_{index:02d}" for index in range(8)]
    ctx.check(
        "all gondolas present",
        all(object_model.get_part(name).name == name for name in gondola_names),
        details=str(gondola_names),
    )
    ctx.check(
        "wheel articulation is continuous about the axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    for index in range(8):
        ctx.allow_overlap(
            wheel,
            object_model.get_part(f"gondola_{index:02d}"),
            elem_a=pivot_stub_names[index],
            elem_b="pivot_tube",
            reason="Each gondola is visually suspended on a small axle stub passing through its pivot tube.",
        )

    ctx.expect_overlap(
        bottom_gondola,
        wheel,
        axes="y",
        elem_a="pivot_tube",
        elem_b=pivot_stub_names[0],
        min_overlap=0.078,
        name="bottom gondola pivot tube stays engaged on the wheel pivot stub",
    )
    ctx.expect_gap(
        bottom_gondola,
        support,
        axis="z",
        positive_elem="floor",
        negative_elem="base_plinth",
        min_gap=0.012,
        name="bottom gondola clears the base plinth",
    )

    rest_pos = ctx.part_world_position(bottom_gondola)
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(bottom_gondola)
    ctx.check(
        "wheel rotation carries gondola around the rim",
        rest_pos is not None
        and quarter_turn_pos is not None
        and abs(quarter_turn_pos[0] - rest_pos[0]) > 0.18
        and abs(quarter_turn_pos[2] - rest_pos[2]) > 0.08,
        details=f"rest={rest_pos}, quarter_turn={quarter_turn_pos}",
    )

    floor_rest = ctx.part_element_world_aabb(bottom_gondola, elem="floor")
    with ctx.pose({bottom_pivot: 0.65}):
        floor_swung = ctx.part_element_world_aabb(bottom_gondola, elem="floor")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    floor_rest_center = _aabb_center(floor_rest)
    floor_swung_center = _aabb_center(floor_swung)
    ctx.check(
        "gondola pivot rotates the basket independently",
        floor_rest_center is not None
        and floor_swung_center is not None
        and abs(floor_swung_center[0] - floor_rest_center[0]) > 0.05,
        details=f"rest={floor_rest_center}, swung={floor_swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
