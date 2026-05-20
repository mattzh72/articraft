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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _arc_points_2d(
    cx: float,
    cy: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def _build_body_profile() -> list[tuple[float, float]]:
    outer_radius = 0.0375
    outer_mouth_half = 0.0175
    inner_radius = 0.0235
    inner_center_z = 0.0145
    inner_mouth_half = 0.0185

    outer_theta = math.acos(outer_mouth_half / outer_radius)
    inner_theta = math.acos(inner_mouth_half / inner_radius)

    outer_loop = _arc_points_2d(
        0.0,
        0.0,
        outer_radius,
        outer_theta,
        (math.pi - outer_theta) - 2.0 * math.pi,
        segments=48,
    )
    inner_loop = _arc_points_2d(
        0.0,
        inner_center_z,
        inner_radius,
        math.pi - inner_theta,
        2.0 * math.pi + inner_theta,
        segments=36,
    )
    return outer_loop + inner_loop


def _build_shackle_path(
    *,
    span: float,
    leg_height: float,
    leg_segments: int = 5,
    arc_segments: int = 14,
) -> list[tuple[float, float, float]]:
    radius = span * 0.5
    path: list[tuple[float, float, float]] = []
    for index in range(leg_segments):
        t = index / (leg_segments - 1)
        path.append((0.0, 0.0, leg_height * t))
    for index in range(1, arc_segments):
        angle = math.pi - (math.pi * index / arc_segments)
        path.append((radius + radius * math.cos(angle), 0.0, leg_height + radius * math.sin(angle)))
    for index in range(leg_segments):
        t = index / (leg_segments - 1)
        path.append((span, 0.0, leg_height * (1.0 - t)))
    return path


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))


def _aabb_max(aabb, axis: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="discus_padlock")

    body_steel = model.material("body_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.88, 0.89, 0.91, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.61, 0.25, 1.0))
    keyway_dark = model.material("keyway_dark", rgba=(0.13, 0.13, 0.14, 1.0))

    body_thickness = 0.026
    shackle_bar_radius = 0.0045
    shackle_span = 0.026
    shackle_leg_height = 0.020
    shackle_pivot = (-0.013, 0.0, 0.010)

    body = model.part("body")
    body_profile = _build_body_profile()
    body_geom = ExtrudeGeometry(body_profile, body_thickness, center=True).rotate_x(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(body_geom, "discus_padlock_body"),
        material=body_steel,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.0065, length=body_thickness),
        origin=Origin(xyz=(-0.0205, 0.0, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_steel,
        name="pivot_seat",
    )
    body.visual(
        Cylinder(radius=0.0065, length=body_thickness),
        origin=Origin(xyz=(0.0205, 0.0, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_steel,
        name="latch_seat",
    )

    shackle = model.part("shackle")
    shackle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _build_shackle_path(span=shackle_span, leg_height=shackle_leg_height),
                radius=shackle_bar_radius,
                samples_per_segment=4,
                radial_segments=20,
                cap_ends=True,
            ),
            "discus_padlock_shackle",
        ),
        material=shackle_steel,
        name="shackle_bar",
    )
    shackle.visual(
        Sphere(radius=0.0015),
        origin=Origin(xyz=(shackle_span, 0.0, 0.0)),
        material=shackle_steel,
        name="free_tip",
    )

    key_cylinder = model.part("key_cylinder")
    key_cylinder.visual(
        Cylinder(radius=0.0082, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="plug_barrel",
    )
    key_cylinder.visual(
        Cylinder(radius=0.0092, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="plug_bezel",
    )
    key_cylinder.visual(
        Box((0.0024, 0.0012, 0.0075)),
        origin=Origin(xyz=(0.0, 0.0051, -0.0012)),
        material=keyway_dark,
        name="keyway_slot",
    )
    key_cylinder.visual(
        Cylinder(radius=0.0012, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0052, 0.0048), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=keyway_dark,
        name="plug_dot",
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=shackle_pivot),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    model.articulation(
        "body_to_key_cylinder",
        ArticulationType.REVOLUTE,
        parent=body,
        child=key_cylinder,
        origin=Origin(xyz=(0.0115, body_thickness * 0.5, -0.0090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=6.0,
            lower=-0.60,
            upper=0.60,
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

    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    key_cylinder = object_model.get_part("key_cylinder")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    key_joint = object_model.get_articulation("body_to_key_cylinder")

    ctx.check(
        "padlock parts exist",
        all(part is not None for part in (body, shackle, key_cylinder)),
    )
    ctx.check(
        "padlock articulations exist",
        shackle_joint is not None and key_joint is not None,
    )

    ctx.expect_overlap(
        shackle,
        body,
        axes="x",
        min_overlap=0.020,
        name="closed shackle stays captured within the discus body width",
    )
    body_aabb = ctx.part_world_aabb(body)
    shackle_aabb = ctx.part_world_aabb(shackle)
    body_top = _aabb_max(body_aabb, 2)
    shackle_top = _aabb_max(shackle_aabb, 2)
    ctx.check(
        "closed shackle exposes only a short crown above the body",
        body_top is not None
        and shackle_top is not None
        and 0.004 <= shackle_top - body_top <= 0.016,
        details=f"body_top={body_top}, shackle_top={shackle_top}",
    )
    ctx.expect_gap(
        key_cylinder,
        body,
        axis="y",
        positive_elem="plug_barrel",
        min_gap=0.0,
        max_gap=0.0005,
        name="key cylinder seats flush on the front plate",
    )

    body_pos = ctx.part_world_position(body)
    plug_pos = ctx.part_world_position(key_cylinder)
    ctx.check(
        "key cylinder is mounted off center on the front plate",
        body_pos is not None
        and plug_pos is not None
        and plug_pos[0] > body_pos[0] + 0.007
        and plug_pos[2] < body_pos[2] - 0.005,
        details=f"body_pos={body_pos}, plug_pos={plug_pos}",
    )

    closed_free_tip = _aabb_center(ctx.part_element_world_aabb(shackle, elem="free_tip"))
    with ctx.pose({shackle_joint: 1.05}):
        opened_free_tip = _aabb_center(ctx.part_element_world_aabb(shackle, elem="free_tip"))
    ctx.check(
        "shackle pivots upward on its retained side",
        closed_free_tip is not None
        and opened_free_tip is not None
        and opened_free_tip[2] > closed_free_tip[2] + 0.018,
        details=f"closed_free_tip={closed_free_tip}, opened_free_tip={opened_free_tip}",
    )

    closed_dot = _aabb_center(ctx.part_element_world_aabb(key_cylinder, elem="plug_dot"))
    with ctx.pose({key_joint: 0.55}):
        turned_dot = _aabb_center(ctx.part_element_world_aabb(key_cylinder, elem="plug_dot"))
    ctx.check(
        "key cylinder rotates on its own axis",
        closed_dot is not None
        and turned_dot is not None
        and turned_dot[0] > closed_dot[0] + 0.0015
        and turned_dot[2] < closed_dot[2] - 0.0005,
        details=f"closed_dot={closed_dot}, turned_dot={turned_dot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
