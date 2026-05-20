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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_axis_member(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size: tuple[float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((size[0], size[1], _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis_member(a, b)),
        material=material,
        name=name,
    )


def _add_cyl_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis_member(a, b)),
        material=material,
        name=name,
    )


def _build_wheel(part, *, wood, iron) -> None:
    wheel_radius = 0.96
    rim_offset = 0.33
    paddle_count = 10
    rim_mesh = _save_mesh(
        "undershot_waterwheel_rim",
        TorusGeometry(radius=wheel_radius, tube=0.055, radial_segments=18, tubular_segments=80),
    )

    part.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, rim_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="left_rim",
    )
    part.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -rim_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="right_rim",
    )
    part.visual(
        Cylinder(radius=0.16, length=0.78),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.075, length=1.18),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_shaft",
    )

    for index in range(paddle_count):
        angle = (2.0 * math.pi * index) / paddle_count
        c = math.cos(angle)
        s = math.sin(angle)
        for side in (-rim_offset, rim_offset):
            _add_cyl_member(
                part,
                (c * 0.14, side, s * 0.14),
                (c * (wheel_radius - 0.08), side, s * (wheel_radius - 0.08)),
                radius=0.028,
                material=iron,
            )
        part.visual(
            Box((0.045, 0.80, 0.18)),
            origin=Origin(
                xyz=(c * 0.93, 0.0, s * 0.93),
                rpy=(0.0, (math.pi / 2.0) - angle, 0.0),
            ),
            material=wood,
            name=f"paddle_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    timber = model.material("timber", rgba=(0.50, 0.35, 0.20, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.42, 0.29, 0.17, 1.0))
    iron = model.material("iron", rgba=(0.27, 0.28, 0.29, 1.0))

    support = model.part("support_structure")
    support.inertial = Inertial.from_geometry(
        Box((3.40, 1.90, 2.50)),
        mass=1600.0,
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
    )

    support.visual(
        Box((1.42, 0.18, 0.18)),
        origin=Origin(xyz=(-0.06, 0.72, 0.09)),
        material=timber,
        name="left_ground_sill",
    )
    support.visual(
        Box((1.42, 0.18, 0.18)),
        origin=Origin(xyz=(-0.06, -0.72, 0.09)),
        material=timber,
        name="right_ground_sill",
    )
    support.visual(
        Box((0.18, 1.62, 0.18)),
        origin=Origin(xyz=(-0.62, 0.0, 0.09)),
        material=timber,
        name="upstream_base_tie",
    )
    support.visual(
        Box((0.18, 1.62, 0.18)),
        origin=Origin(xyz=(0.50, 0.0, 0.09)),
        material=timber,
        name="downstream_base_tie",
    )
    for side in (-0.72, 0.72):
        _add_box_member(
            support,
            (-0.62, side, 0.16),
            (-0.28, side, 1.58),
            size=(0.14, 0.16),
            material=timber,
            name=None,
        )
        _add_box_member(
            support,
            (0.48, side, 0.16),
            (0.16, side, 1.56),
            size=(0.14, 0.16),
            material=timber,
            name=None,
        )
        _add_box_member(
            support,
            (-0.28, side, 1.58),
            (0.16, side, 1.56),
            size=(0.14, 0.18),
            material=timber,
            name="left_top_cap" if side > 0.0 else "right_top_cap",
        )
        _add_box_member(
            support,
            (-0.54, side, 0.32),
            (0.06, side, 1.38),
            size=(0.10, 0.14),
            material=timber,
            name=None,
        )
        support.visual(
            Box((0.60, 0.14, 0.14)),
            origin=Origin(xyz=(-0.06, side, 1.24)),
            material=timber,
            name="front_bearing_beam" if side > 0.0 else "rear_bearing_beam",
        )

    support.visual(
        Box((0.22, 0.13, 0.22)),
        origin=Origin(xyz=(0.0, 0.655, 1.24)),
        material=iron,
        name="front_bearing_block",
    )
    support.visual(
        Box((0.22, 0.13, 0.22)),
        origin=Origin(xyz=(0.0, -0.655, 1.24)),
        material=iron,
        name="rear_bearing_block",
    )

    _add_box_member(
        support,
        (-0.34, -0.50, 1.58),
        (-0.34, -0.50, 2.26),
        size=(0.10, 0.10),
        material=timber,
        name="rear_flume_post",
    )
    _add_box_member(
        support,
        (-0.34, 0.50, 1.58),
        (-0.34, 0.50, 2.26),
        size=(0.10, 0.10),
        material=timber,
        name="front_flume_post",
    )
    support.visual(
        Box((0.92, 1.06, 0.08)),
        origin=Origin(xyz=(-0.82, 0.0, 2.30)),
        material=timber,
        name="flume_floor",
    )
    support.visual(
        Box((0.18, 1.06, 0.22)),
        origin=Origin(xyz=(-1.16, 0.0, 2.37)),
        material=timber,
        name="flume_lip",
    )

    guard_ring = _save_mesh(
        "undershot_waterwheel_guard_ring",
        TorusGeometry(radius=1.07, tube=0.025, radial_segments=16, tubular_segments=72),
    )
    support.visual(
        guard_ring,
        origin=Origin(xyz=(0.0, 0.54, 1.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="guard_ring",
    )
    _add_cyl_member(
        support,
        (-0.18, 0.72, 1.56),
        (-0.56, 0.54, 2.14),
        radius=0.026,
        material=iron,
        name="guard_upper_rear_brace",
    )
    _add_cyl_member(
        support,
        (0.12, 0.72, 1.56),
        (0.56, 0.54, 2.14),
        radius=0.026,
        material=iron,
        name="guard_upper_front_brace",
    )
    _add_cyl_member(
        support,
        (-0.52, 0.72, 0.36),
        (-0.56, 0.54, 0.34),
        radius=0.026,
        material=iron,
        name="guard_lower_rear_brace",
    )
    _add_cyl_member(
        support,
        (0.44, 0.72, 0.36),
        (0.56, 0.54, 0.34),
        radius=0.026,
        material=iron,
        name="guard_lower_front_brace",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.98, length=1.18),
        mass=260.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_wheel(wheel, wood=wet_timber, iron=iron)

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=2.5),
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
    support = object_model.get_part("support_structure")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_origin_gap(
        wheel,
        support,
        axis="z",
        min_gap=1.20,
        max_gap=1.28,
        name="wheel axle sits at trestle height",
    )
    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle_shaft",
        elem_b="front_bearing_block",
        name="front journal touches its bearing block",
    )
    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle_shaft",
        elem_b="rear_bearing_block",
        name="rear journal touches its bearing block",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: 1.1}):
        spun_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spin keeps axle centered",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
