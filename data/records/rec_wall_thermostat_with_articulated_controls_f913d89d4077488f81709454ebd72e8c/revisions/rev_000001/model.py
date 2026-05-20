from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


FRONT_Z = 0.028


def _thermostat_shell_mesh():
    """A softly tapered, rounded-rectangle plastic thermostat body."""

    sections = [
        (0.134, 0.098, 0.000),
        (0.132, 0.096, 0.006),
        (0.128, 0.092, 0.018),
        (0.121, 0.085, FRONT_Z),
    ]
    profiles = [
        [(x, y, z) for x, y in superellipse_profile(width, height, exponent=4.2, segments=72)]
        for width, height, z in sections
    ]
    return LoftGeometry(profiles, cap=True, closed=True)


def _add_tick_marks(part, material) -> None:
    """Small static temperature-scale ticks printed into the front plastic."""

    for index in range(17):
        angle = math.radians(-120.0 + 15.0 * index)
        length = 0.0070 if index in (0, 4, 8, 12, 16) else 0.0042
        width = 0.0013 if index in (0, 4, 8, 12, 16) else 0.00085
        radius = 0.044
        part.visual(
            Box((length, width, 0.0007)),
            origin=Origin(
                xyz=(radius * math.sin(angle), radius * math.cos(angle), FRONT_Z + 0.00025),
                rpy=(0.0, 0.0, -angle),
            ),
            material=material,
            name=f"scale_tick_{index}",
        )


def _add_vent_slots(part, material) -> None:
    for side_index, x in enumerate((-0.064, 0.064)):
        for slot_index, y in enumerate((-0.020, -0.010, 0.000, 0.010, 0.020)):
            part.visual(
                Box((0.0008, 0.0036, 0.010)),
                origin=Origin(xyz=(x, y, 0.016)),
                material=material,
                name=f"side_vent_{side_index}_{slot_index}",
            )


def _add_seven_segment_digit(part, digit: int, *, x: float, y: float, scale: float, material) -> None:
    segment_sets = {
        0: "abcfed",
        1: "bc",
        2: "abged",
        3: "abgcd",
        4: "fgbc",
        5: "afgcd",
        6: "afgecd",
        7: "abc",
        8: "abcdefg",
        9: "abfgcd",
    }
    active = segment_sets[digit]
    full_w = 0.0105 * scale
    half_w = full_w * 0.5
    y_span = 0.0135 * scale
    half_y = y_span * 0.5
    thick = 0.00155 * scale
    z = 0.00415
    led_h = 0.00065
    horizontal = Box((full_w, thick, led_h))
    vertical = Box((thick, half_y, led_h))
    specs = {
        "a": (horizontal, (x, y + half_y, z), 0.0),
        "g": (horizontal, (x, y, z), 0.0),
        "d": (horizontal, (x, y - half_y, z), 0.0),
        "b": (vertical, (x + half_w, y + half_y * 0.5, z), 0.0),
        "c": (vertical, (x + half_w, y - half_y * 0.5, z), 0.0),
        "f": (vertical, (x - half_w, y + half_y * 0.5, z), 0.0),
        "e": (vertical, (x - half_w, y - half_y * 0.5, z), 0.0),
    }
    for segment in active:
        geometry, xyz, yaw = specs[segment]
        part.visual(
            geometry,
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            material=material,
            name=f"digit_{digit}_{segment}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.87, 0.82, 1.0))
    soft_shadow = model.material("soft_shadow_gasket", rgba=(0.06, 0.065, 0.07, 1.0))
    printed_grey = model.material("printed_grey", rgba=(0.34, 0.35, 0.34, 1.0))
    satin_metal = model.material("satin_metal_dial", rgba=(0.74, 0.75, 0.73, 1.0))
    black_glass = model.material("black_glass", rgba=(0.02, 0.025, 0.03, 0.78))
    led_cyan = model.material("cool_cyan_display", rgba=(0.20, 0.95, 1.00, 1.0))
    heat_orange = model.material("heat_orange", rgba=(1.0, 0.40, 0.08, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.140, 0.104, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0012)),
        material=warm_white,
        name="wall_backplate",
    )
    housing.visual(
        mesh_from_geometry(_thermostat_shell_mesh(), "thermostat_tapered_shell"),
        material=warm_white,
        name="body_shell",
    )
    housing.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.082, 0.082),
                (0.088, 0.088),
                0.0014,
                opening_shape="circle",
                outer_shape="circle",
                center=False,
            ),
            "thermostat_dial_shadow",
        ),
        origin=Origin(xyz=(0.0, 0.0, FRONT_Z - 0.00035)),
        material=soft_shadow,
        name="dial_shadow",
    )
    _add_tick_marks(housing, printed_grey)
    _add_vent_slots(housing, soft_shadow)
    housing.inertial = Inertial.from_geometry(
        Box((0.134, 0.098, 0.031)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    display = model.part("display")
    display.visual(
        Cylinder(radius=0.0195, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=black_glass,
        name="glass",
    )
    _add_seven_segment_digit(display, 7, x=-0.0063, y=0.0012, scale=0.86, material=led_cyan)
    _add_seven_segment_digit(display, 2, x=0.0062, y=0.0012, scale=0.86, material=led_cyan)
    display.visual(
        Cylinder(radius=0.0011, length=0.0007),
        origin=Origin(xyz=(0.0148, 0.0078, 0.0042)),
        material=led_cyan,
        name="degree_dot",
    )
    display.visual(
        Box((0.010, 0.0014, 0.00065)),
        origin=Origin(xyz=(0.000, -0.0130, 0.00415), rpy=(0.0, 0.0, 0.12)),
        material=heat_orange,
        name="heat_status_wave",
    )
    display.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.004),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.048, 0.048),
                (0.076, 0.076),
                0.012,
                opening_shape="circle",
                outer_shape="circle",
                center=False,
            ),
            "thermostat_rotating_ring",
        ),
        material=satin_metal,
        name="dial_ring",
    )
    for index in range(32):
        angle = 2.0 * math.pi * index / 32.0
        radius = 0.0382
        dial.visual(
            Box((0.0028, 0.0032, 0.0095)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0062),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_metal,
            name=f"grip_rib_{index}",
        )
    dial.visual(
        Box((0.0032, 0.0135, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0300, 0.01225)),
        material=heat_orange,
        name="setpoint_marker",
    )
    dial.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.042, 0.042),
                (0.050, 0.050),
                0.0018,
                opening_shape="circle",
                outer_shape="circle",
                center=False,
            ),
            "thermostat_bearing_sleeve",
        ),
        material=soft_shadow,
        name="bearing_sleeve",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.012),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "housing_to_display",
        ArticulationType.FIXED,
        parent=housing,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, FRONT_Z)),
    )
    model.articulation(
        "dial_rotation",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, FRONT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.18, velocity=3.0, lower=-2.45, upper=2.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    display = object_model.get_part("display")
    dial = object_model.get_part("dial")
    joint = object_model.get_articulation("dial_rotation")

    ctx.expect_contact(
        dial,
        housing,
        elem_a="dial_ring",
        elem_b="body_shell",
        contact_tol=0.0008,
        name="dial ring seats on front shell",
    )
    ctx.expect_gap(
        display,
        housing,
        axis="z",
        positive_elem="glass",
        negative_elem="body_shell",
        max_gap=0.0008,
        max_penetration=0.000001,
        name="display glass is flush mounted",
    )
    ctx.expect_origin_distance(
        display,
        dial,
        axes="xy",
        max_dist=0.001,
        name="display remains centered inside dial",
    )

    rest_aabb = ctx.part_element_world_aabb(dial, elem="setpoint_marker")
    with ctx.pose({joint: 1.20}):
        turned_aabb = ctx.part_element_world_aabb(dial, elem="setpoint_marker")

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return ((float(mins[0]) + float(maxs[0])) * 0.5, (float(mins[1]) + float(maxs[1])) * 0.5)

    rest_center = _aabb_center_xy(rest_aabb)
    turned_center = _aabb_center_xy(turned_aabb)
    ctx.check(
        "dial marker rotates around face",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] - rest_center[0]) > 0.015
        and turned_center[1] < rest_center[1] - 0.008,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
