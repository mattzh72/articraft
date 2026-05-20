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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _wheel_side_plate_mesh(
    *,
    wheel_radius: float,
    hub_radius: float,
    window_radius: float,
    window_count: int,
    thickness: float,
):
    outer_profile = superellipse_profile(
        wheel_radius * 2.0,
        wheel_radius * 2.0,
        exponent=2.0,
        segments=64,
    )
    hub_keepout = wheel_radius * 0.13
    rim_keepout = wheel_radius * 0.11
    slot_profile = rounded_rect_profile(
        wheel_radius * 0.22,
        wheel_radius * 0.14,
        radius=wheel_radius * 0.035,
        corner_segments=8,
    )
    hole_profiles: list[list[tuple[float, float]]] = []
    for index in range(window_count):
        angle = 2.0 * pi * index / window_count
        radius_here = max(hub_radius + hub_keepout, min(window_radius, wheel_radius - rim_keepout))
        hole_profiles.append(
            _transform_profile(
                slot_profile,
                dx=cos(angle) * radius_here,
                dy=sin(angle) * radius_here,
                angle=angle,
            )
        )
    return ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.55, 0.42, 0.28, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.35, 0.25, 0.16, 1.0))
    iron = model.material("iron", rgba=(0.25, 0.27, 0.30, 1.0))
    damp_stone = model.material("damp_stone", rgba=(0.48, 0.50, 0.52, 1.0))

    wheel_radius = 0.60
    wheel_width = 0.70
    side_plate_thickness = 0.04
    side_plate_offset = 0.31
    paddle_depth = 0.18
    paddle_thickness = 0.05
    paddle_center_radius = wheel_radius - paddle_depth * 0.48
    axle_radius = 0.04
    axle_length = 0.88
    axle_height = 0.74

    support = model.part("support_frame")
    support.inertial = Inertial.from_geometry(
        Box((1.26, 1.12, 1.62)),
        mass=180.0,
        origin=Origin(xyz=(0.0, -0.06, 0.81)),
    )

    # Stone sill and channel bed
    support.visual(
        Box((1.24, 0.36, 0.14)),
        origin=Origin(xyz=(0.0, -0.32, 0.07)),
        material=damp_stone,
        name="rear_stone_sill",
    )
    support.visual(
        Box((1.24, 0.38, 0.12)),
        origin=Origin(xyz=(0.0, 0.34, 0.06)),
        material=damp_stone,
        name="front_stone_sill",
    )
    support.visual(
        Box((1.08, 0.50, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=damp_stone,
        name="channel_bed",
    )

    for side_sign, prefix in [(-1.0, "left"), (1.0, "right")]:
        x = 0.50 * side_sign
        support.visual(
            Box((0.08, 0.10, 1.18)),
            origin=Origin(xyz=(x, 0.25, 0.63)),
            material=weathered_wood,
            name=f"{prefix}_front_post",
        )
        support.visual(
            Box((0.08, 0.10, 1.18)),
            origin=Origin(xyz=(x, -0.25, 0.63)),
            material=weathered_wood,
            name=f"{prefix}_rear_post",
        )
        support.visual(
            Box((0.08, 0.58, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.11)),
            material=dark_wood,
            name=f"{prefix}_bottom_rail",
        )
        support.visual(
            Box((0.08, 0.62, 0.10)),
            origin=Origin(xyz=(x, 0.0, 1.22)),
            material=dark_wood,
            name=f"{prefix}_top_rail",
        )
        support.visual(
            Box((0.06, 0.08, 1.04)),
            origin=Origin(xyz=(x, -0.01, 0.65), rpy=(0.60 * side_sign, 0.0, 0.0)),
            material=dark_wood,
            name=f"{prefix}_diagonal_brace",
        )
        support.visual(
            Box((0.12, 0.16, 0.18)),
            origin=Origin(xyz=(x, 0.0, axle_height)),
            material=iron,
            name=f"{prefix}_bearing_block",
        )

    support.visual(
        Box((1.16, 0.40, 0.06)),
        origin=Origin(xyz=(0.0, -0.52, 1.42)),
        material=weathered_wood,
        name="trough_floor",
    )
    support.visual(
        Box((1.16, 0.08, 0.26)),
        origin=Origin(xyz=(0.0, -0.68, 1.49)),
        material=weathered_wood,
        name="trough_back_wall",
    )
    support.visual(
        Box((1.16, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, -0.34, 1.49)),
        material=weathered_wood,
        name="trough_front_lip",
    )
    for side_sign, prefix in [(-1.0, "left"), (1.0, "right")]:
        support.visual(
            Box((0.08, 0.40, 0.26)),
            origin=Origin(xyz=(0.54 * side_sign, -0.52, 1.49)),
            material=weathered_wood,
            name=f"{prefix}_trough_side",
        )
        support.visual(
            Box((0.08, 0.10, 0.18)),
            origin=Origin(xyz=(0.50 * side_sign, -0.52, 1.31)),
            material=dark_wood,
            name=f"{prefix}_trough_hanger",
        )
        support.visual(
            Box((0.08, 0.38, 0.08)),
            origin=Origin(xyz=(0.50 * side_sign, -0.38, 1.24)),
            material=dark_wood,
            name=f"{prefix}_trough_ledger",
        )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=45.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    wheel_side_mesh = _save_mesh(
        "waterwheel_side_plate",
        _wheel_side_plate_mesh(
            wheel_radius=wheel_radius,
            hub_radius=0.12,
            window_radius=0.34,
            window_count=8,
            thickness=side_plate_thickness,
        ),
    )
    wheel.visual(
        wheel_side_mesh,
        origin=Origin(xyz=(-side_plate_offset, 0.0, 0.0)),
        material=weathered_wood,
        name="left_cheek",
    )
    wheel.visual(
        wheel_side_mesh,
        origin=Origin(xyz=(side_plate_offset, 0.0, 0.0)),
        material=weathered_wood,
        name="right_cheek",
    )
    wheel.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.12, length=0.30),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_wood,
        name="hub_drum",
    )

    for index in range(12):
        angle = 2.0 * pi * index / 12.0
        wheel.visual(
            Box((wheel_width, paddle_thickness, paddle_depth)),
            origin=Origin(
                xyz=(0.0, -paddle_center_radius * sin(angle), paddle_center_radius * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=weathered_wood,
            name=f"paddle_{index:02d}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle",
        elem_b="left_bearing_block",
        contact_tol=1e-6,
        name="left axle journal contacts the left bearing block",
    )
    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle",
        elem_b="right_bearing_block",
        contact_tol=1e-6,
        name="right axle journal contacts the right bearing block",
    )
    ctx.expect_gap(
        support,
        wheel,
        axis="z",
        positive_elem="trough_front_lip",
        max_gap=0.10,
        max_penetration=0.0,
        name="wheel remains below the trough edge",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="z",
        negative_elem="channel_bed",
        min_gap=0.02,
        max_gap=0.08,
        name="wheel clears the stream bed with a shallow undershot gap",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(wheel)

    ctx.check(
        "continuous spin keeps the wheel centered on its axle",
        rest_pos is not None
        and quarter_turn_pos is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(rest_pos, quarter_turn_pos)),
        details=f"rest={rest_pos}, quarter_turn={quarter_turn_pos}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
