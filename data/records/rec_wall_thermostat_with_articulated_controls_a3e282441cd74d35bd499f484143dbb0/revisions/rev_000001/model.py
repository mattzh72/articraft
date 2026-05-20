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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


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
    model = ArticulatedObject(name="wall_thermostat")

    plate_white = model.material("plate_white", rgba=(0.94, 0.94, 0.93, 1.0))
    shell_white = model.material("shell_white", rgba=(0.97, 0.97, 0.96, 1.0))
    dial_metal = model.material("dial_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    dial_shadow = model.material("dial_shadow", rgba=(0.62, 0.64, 0.67, 1.0))
    display_dark = model.material("display_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    accent_black = model.material("accent_black", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("thermostat_body")

    wall_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.145, 0.145, 0.016),
            0.004,
            cap=True,
            center=True,
        ),
        "thermostat_wall_plate",
    )
    body.visual(
        wall_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=plate_white,
        name="wall_plate",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=shell_white,
        name="rear_body",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=display_dark,
        name="axis_support_sleeve",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.145, 0.145, 0.036)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    dial = model.part("dial")
    dial_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.060, -0.0045), (0.060, 0.0045)],
            [
                (0.018, -0.0045),
                (0.020, -0.0035),
                (0.026, -0.0015),
                (0.032, 0.0015),
                (0.038, 0.0045),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_ring",
    )
    dial_grip_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.062, -0.004), (0.062, 0.004)],
            [(0.0565, -0.004), (0.0565, 0.004)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_grip",
    )
    dial.visual(
        dial_ring_mesh,
        material=dial_metal,
        name="dial_ring",
    )
    dial.visual(
        dial_grip_mesh,
        material=dial_shadow,
        name="dial_grip",
    )
    dial.visual(
        Box((0.004, 0.016, 0.0015)),
        origin=Origin(xyz=(0.0, 0.046, 0.00475)),
        material=accent_black,
        name="dial_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.059, length=0.010),
        mass=0.11,
        origin=Origin(),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("thermostat_body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=0.0001,
        name="dial stays centered on the thermostat body",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_ring",
        elem_b="rear_body",
        min_overlap=0.095,
        name="dial ring covers the front body footprint",
    )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        marker_quarter_turn = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))

    ctx.check(
        "dial marker moves around the center on rotation",
        marker_rest is not None
        and marker_quarter_turn is not None
        and marker_rest[1] > 0.035
        and marker_quarter_turn[0] < -0.035
        and abs(marker_quarter_turn[1]) < 0.010,
        details=f"rest={marker_rest}, quarter_turn={marker_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
