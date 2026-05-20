from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_large_dial")

    satin_white = model.material("satin_white", rgba=(0.88, 0.88, 0.84, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.70, 0.70, 0.66, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.76, 0.77, 0.74, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.55, 0.56, 0.54, 1.0))
    glass_black = model.material("glass_black", rgba=(0.04, 0.05, 0.055, 1.0))
    lcd_green = model.material("lcd_green", rgba=(0.52, 0.86, 0.70, 1.0))
    ink_dark = model.material("ink_dark", rgba=(0.16, 0.16, 0.15, 1.0))

    plate_profile = rounded_rect_profile(0.180, 0.130, 0.014, corner_segments=10)
    body_profile = rounded_rect_profile(0.145, 0.100, 0.020, corner_segments=12)
    dial_outer = superellipse_profile(0.086, 0.086, exponent=2.0, segments=96)
    dial_inner = superellipse_profile(0.058, 0.058, exponent=2.0, segments=96)

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_geometry(ExtrudeGeometry(plate_profile, 0.006), "wall_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_white,
        name="wall_plate",
    )
    wall_plate.visual(
        mesh_from_geometry(ExtrudeGeometry(body_profile, 0.020), "shallow_body"),
        # A tiny hidden embed ties the shallow body into the plate.
        origin=Origin(xyz=(0.0, 0.0, 0.0157)),
        material=warm_gray,
        name="body_shell",
    )
    wall_plate.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0272)),
        material=glass_black,
        name="center_display",
    )
    wall_plate.visual(
        Box((0.026, 0.006, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.02935)),
        material=lcd_green,
        name="display_window",
    )

    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        radius = 0.0475
        tick_length = 0.0050 if index % 3 == 0 else 0.0032
        tick_width = 0.0014 if index % 3 == 0 else 0.0010
        wall_plate.visual(
            Box((tick_length, tick_width, 0.0008)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.02595),
                rpy=(0.0, 0.0, angle),
            ),
            material=ink_dark,
            name=f"temperature_tick_{index:02d}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(dial_outer, [dial_inner], 0.009),
            "dial_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=dial_silver,
        name="dial_ring",
    )
    dial.visual(
        Box((0.0040, 0.0140, 0.0012)),
        origin=Origin(xyz=(0.0, 0.036, 0.0094), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=ink_dark,
        name="indicator_mark",
    )
    for index in range(18):
        angle = (2.0 * math.pi * index) / 18.0
        dial.visual(
            Box((0.0035, 0.0105, 0.0018)),
            origin=Origin(
                xyz=(0.039 * math.cos(angle), 0.039 * math.sin(angle), 0.0097),
                rpy=(0.0, 0.0, angle),
            ),
            material=grip_gray,
            name=f"grip_rib_{index:02d}",
        )

    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=wall_plate,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    dial = object_model.get_part("dial")
    dial_turn = object_model.get_articulation("dial_turn")

    ctx.expect_within(
        dial,
        wall_plate,
        axes="xy",
        inner_elem="dial_ring",
        outer_elem="body_shell",
        margin=0.0,
        name="large dial fits on the shallow body",
    )
    ctx.expect_gap(
        dial,
        wall_plate,
        axis="z",
        positive_elem="dial_ring",
        negative_elem="body_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial ring seats on the body face",
    )

    rest_aabb = ctx.part_element_world_aabb(dial, elem="indicator_mark")
    with ctx.pose({dial_turn: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(dial, elem="indicator_mark")

    def _aabb_center_xy(aabb):
        lower, upper = aabb
        return ((lower[0] + upper[0]) * 0.5, (lower[1] + upper[1]) * 0.5)

    if rest_aabb is not None and turned_aabb is not None:
        rest_xy = _aabb_center_xy(rest_aabb)
        turned_xy = _aabb_center_xy(turned_aabb)
        ctx.check(
            "indicator mark follows the continuous dial rotation",
            turned_xy[0] < rest_xy[0] - 0.025 and turned_xy[1] < rest_xy[1] - 0.025,
            details=f"rest={rest_xy}, turned={turned_xy}",
        )
    else:
        ctx.fail("indicator mark follows the continuous dial rotation", "indicator AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
