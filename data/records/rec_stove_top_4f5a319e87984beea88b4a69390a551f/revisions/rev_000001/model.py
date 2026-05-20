from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COUNTER_SIZE = (0.90, 0.65, 0.040)
COOKTOP_SIZE = (0.66, 0.52, 0.006)
COUNTER_TOP_Z = COUNTER_SIZE[2]
GLASS_TOP_Z = COUNTER_TOP_Z
STRIP_THICKNESS = 0.0008
STRIP_TOP_Z = GLASS_TOP_Z + STRIP_THICKNESS

BUTTON_Y = -0.215
BUTTON_XS = (-0.225, -0.185, -0.020, 0.020, 0.185, 0.225)
BUTTON_HOLE_RADIUS = 0.0185
BUTTON_CAP_RADIUS = 0.0160
BUTTON_STEM_RADIUS = 0.0100
BUTTON_RETAINER_RADIUS = 0.0205
BUTTON_RETAINER_HEIGHT = 0.0012
BUTTON_CAP_HEIGHT = 0.0055
BUTTON_STEM_LENGTH = 0.0150
BUTTON_TRAVEL = 0.0060


def _counter_frame_shape() -> cq.Workplane:
    slab = cq.Workplane("XY").box(*COUNTER_SIZE).translate((0.0, 0.0, COUNTER_SIZE[2] / 2.0))
    opening = (
        cq.Workplane("XY")
        .box(COOKTOP_SIZE[0], COOKTOP_SIZE[1], COUNTER_SIZE[2] * 2.0)
        .translate((0.0, 0.0, COUNTER_SIZE[2] / 2.0))
    )
    return slab.cut(opening)


def _glass_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(COOKTOP_SIZE[0], COOKTOP_SIZE[1], COOKTOP_SIZE[2])
        .translate((0.0, 0.0, COUNTER_TOP_Z - COOKTOP_SIZE[2] / 2.0))
    )
    for x in BUTTON_XS:
        cutter = (
            cq.Workplane("XY")
            .center(x, BUTTON_Y)
            .circle(BUTTON_HOLE_RADIUS)
            .extrude(0.020)
            .translate((0.0, 0.0, COUNTER_TOP_Z - 0.014))
        )
        plate = plate.cut(cutter)
    return plate


def _control_strip_shape() -> cq.Workplane:
    strip = (
        cq.Workplane("XY")
        .box(0.600, 0.078, STRIP_THICKNESS)
        .translate((0.0, BUTTON_Y, GLASS_TOP_Z + STRIP_THICKNESS / 2.0))
    )
    for x in BUTTON_XS:
        cutter = (
            cq.Workplane("XY")
            .center(x, BUTTON_Y)
            .circle(BUTTON_HOLE_RADIUS)
            .extrude(0.006)
            .translate((0.0, 0.0, GLASS_TOP_Z - 0.002))
        )
        strip = strip.cut(cutter)
    return strip


def _annulus_shape(outer_radius: float, inner_radius: float, height: float = 0.00055) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shallow_electric_cooktop")

    stone = model.material("warm_stone", rgba=(0.72, 0.69, 0.62, 1.0))
    glass = model.material("black_ceramic_glass", rgba=(0.015, 0.018, 0.020, 1.0))
    strip_mat = model.material("charcoal_control_strip", rgba=(0.05, 0.055, 0.060, 1.0))
    zone_mat = model.material("faint_radiant_red", rgba=(0.85, 0.055, 0.015, 1.0))
    zone_dark = model.material("dark_zone_print", rgba=(0.006, 0.006, 0.007, 1.0))
    button_mat = model.material("satin_silver_button", rgba=(0.80, 0.80, 0.76, 1.0))

    counter = model.part("counter")
    counter.visual(
        mesh_from_cadquery(_counter_frame_shape(), "counter_frame", tolerance=0.0008),
        material=stone,
        name="counter_frame",
    )
    counter.visual(
        mesh_from_cadquery(_glass_plate_shape(), "glass_panel", tolerance=0.0007),
        material=glass,
        name="glass_panel",
    )
    counter.visual(
        mesh_from_cadquery(_control_strip_shape(), "control_strip", tolerance=0.0005),
        material=strip_mat,
        name="control_strip",
    )

    ring_meshes = {
        "small": mesh_from_cadquery(_annulus_shape(0.060, 0.054), "small_radiant_ring", tolerance=0.0004),
        "medium": mesh_from_cadquery(_annulus_shape(0.071, 0.064), "medium_radiant_ring", tolerance=0.0004),
        "large": mesh_from_cadquery(_annulus_shape(0.084, 0.076), "large_radiant_ring", tolerance=0.0004),
    }
    zone_layout = (
        ("zone_0", -0.175, 0.125, 0.078, "medium"),
        ("zone_1", 0.175, 0.125, 0.065, "small"),
        ("zone_2", -0.175, -0.070, 0.062, "small"),
        ("zone_3", 0.175, -0.060, 0.088, "large"),
    )
    for zone_name, x, y, radius, ring_key in zone_layout:
        counter.visual(
            Cylinder(radius=radius, length=0.00025),
            origin=Origin(xyz=(x, y, GLASS_TOP_Z + 0.000125)),
            material=zone_dark,
            name=f"{zone_name}_field",
        )
        counter.visual(
            ring_meshes[ring_key],
            origin=Origin(xyz=(x, y, GLASS_TOP_Z + 0.00025)),
            material=zone_mat,
            name=f"{zone_name}_outer_ring",
        )
        counter.visual(
            mesh_from_cadquery(_annulus_shape(radius * 0.52, radius * 0.45), f"{zone_name}_inner_ring", tolerance=0.0004),
            origin=Origin(xyz=(x, y, GLASS_TOP_Z + 0.00025)),
            material=zone_mat,
            name=f"{zone_name}_inner_ring",
        )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_STEM_RADIUS, length=BUTTON_STEM_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, -BUTTON_STEM_LENGTH / 2.0)),
            material=button_mat,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=BUTTON_CAP_RADIUS, length=BUTTON_CAP_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_CAP_HEIGHT / 2.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=BUTTON_RETAINER_RADIUS, length=BUTTON_RETAINER_HEIGHT),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    (GLASS_TOP_Z - COOKTOP_SIZE[2]) - STRIP_TOP_Z - BUTTON_RETAINER_HEIGHT / 2.0,
                )
            ),
            material=button_mat,
            name="retainer_flange",
        )
        model.articulation(
            f"button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=counter,
            child=button,
            origin=Origin(xyz=(x, BUTTON_Y, STRIP_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = list(object_model.articulations)
    ctx.check(
        "only six button plungers articulate",
        len(joints) == 6 and all(j.articulation_type == ArticulationType.PRISMATIC for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    counter = object_model.get_part("counter")
    for index in range(6):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"button_{index}_slide")
        ctx.expect_gap(
            button,
            counter,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_strip",
            min_gap=-0.0002,
            max_gap=0.0010,
            name=f"button_{index} cap starts flush with strip",
        )
        ctx.expect_within(
            button,
            counter,
            axes="xy",
            inner_elem="button_stem",
            outer_elem="glass_panel",
            margin=0.001,
            name=f"button_{index} stem lies inside the cooktop opening footprint",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                button,
                counter,
                axis="z",
                positive_elem="button_cap",
                negative_elem="control_strip",
                min_gap=-BUTTON_TRAVEL - 0.001,
                max_gap=-BUTTON_TRAVEL + 0.001,
                name=f"button_{index} plunges into the strip",
            )
        ctx.check(
            f"button_{index} travels downward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
