from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges()
        .fillet(radius)
        .translate(center)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_wall_thermostat")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    wall_white = model.material("painted_wall_plate", rgba=(0.94, 0.93, 0.88, 1.0))
    grey = model.material("soft_grey_trim", rgba=(0.55, 0.57, 0.55, 1.0))
    dark = model.material("black_glass", rgba=(0.02, 0.025, 0.025, 1.0))
    lcd_green = model.material("lcd_green", rgba=(0.55, 0.85, 0.58, 1.0))
    smoked = model.material("smoked_clear_cover", rgba=(0.45, 0.58, 0.62, 0.38))
    satin = model.material("satin_metal", rgba=(0.70, 0.70, 0.66, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.007, 0.150, 0.150)),
        origin=Origin(xyz=(0.00325, 0.0, 0.0)),
        material=wall_white,
        name="wall_plate",
    )
    chassis.visual(
        mesh_from_cadquery(_rounded_box((0.032, 0.122, 0.122), 0.006, (0.022, 0.0, 0.0)), "rounded_body"),
        material=warm_white,
        name="rounded_body",
    )

    # A shallow raised bezel and separate black display glass make the square
    # thermostat face read at wall-control scale while leaving the service
    # cover visibly in front of it.
    chassis.visual(
        Box((0.0020, 0.078, 0.006)),
        origin=Origin(xyz=(0.0390, 0.0, 0.045)),
        material=grey,
        name="top_bezel",
    )
    chassis.visual(
        Box((0.0020, 0.078, 0.006)),
        origin=Origin(xyz=(0.0390, 0.0, 0.005)),
        material=grey,
        name="bottom_bezel",
    )
    chassis.visual(
        Box((0.0020, 0.006, 0.046)),
        origin=Origin(xyz=(0.0390, -0.039, 0.025)),
        material=grey,
        name="side_bezel_0",
    )
    chassis.visual(
        Box((0.0020, 0.006, 0.046)),
        origin=Origin(xyz=(0.0390, 0.039, 0.025)),
        material=grey,
        name="side_bezel_1",
    )
    chassis.visual(
        Box((0.0022, 0.058, 0.030)),
        origin=Origin(xyz=(0.0390, 0.0, 0.025)),
        material=dark,
        name="display_glass",
    )

    # Simple seven-segment style strokes on the display.
    for i, y in enumerate((-0.020, -0.006, 0.012)):
        chassis.visual(
            Box((0.0010, 0.011, 0.0024)),
            origin=Origin(xyz=(0.0404, y, 0.034)),
            material=lcd_green,
            name=f"digit_top_{i}",
        )
        chassis.visual(
            Box((0.0010, 0.011, 0.0024)),
            origin=Origin(xyz=(0.0404, y, 0.025)),
            material=lcd_green,
            name=f"digit_mid_{i}",
        )
        chassis.visual(
            Box((0.0010, 0.011, 0.0024)),
            origin=Origin(xyz=(0.0404, y, 0.016)),
            material=lcd_green,
            name=f"digit_bottom_{i}",
        )
    chassis.visual(
        Box((0.0010, 0.004, 0.004)),
        origin=Origin(xyz=(0.0404, 0.028, 0.034)),
        material=lcd_green,
        name="degree_dot",
    )

    # Right side dial bearing and a visible short shaft seat.
    chassis.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.025, 0.061, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="side_bushing",
    )
    chassis.visual(
        Box((0.014, 0.004, 0.030)),
        origin=Origin(xyz=(0.025, 0.058, 0.0)),
        material=warm_white,
        name="bushing_fairing",
    )

    # Interleaved top hinge: fixed end knuckles on the chassis, central knuckle
    # on the cover.  The barrels are exposed so the flip-up mechanism is clear.
    hinge_z = 0.070
    hinge_x = 0.042
    for i, y in enumerate((-0.043, 0.043)):
        chassis.visual(
            Box((0.008, 0.020, 0.011)),
            origin=Origin(xyz=(hinge_x - 0.003, y, hinge_z - 0.006)),
            material=warm_white,
            name=f"hinge_stand_{i}",
        )
        chassis.visual(
            Cylinder(radius=0.0043, length=0.020),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin,
            name=f"fixed_knuckle_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.018,
                body_style="domed",
                edge_radius=0.0015,
                grip=KnobGrip(style="ribbed", count=28, depth=0.0008, width=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "dial_cap",
        ),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="dial_cap",
    )

    cover = model.part("service_cover")
    cover.visual(
        Cylinder(radius=0.0038, length=0.060),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="cover_knuckle",
    )
    cover.visual(
        Box((0.004, 0.080, 0.010)),
        origin=Origin(xyz=(0.003, 0.0, -0.006)),
        material=smoked,
        name="cover_lip",
    )
    cover.visual(
        Box((0.004, 0.092, 0.064)),
        origin=Origin(xyz=(0.004, 0.0, -0.037)),
        material=smoked,
        name="cover_panel",
    )
    cover.visual(
        Box((0.006, 0.030, 0.006)),
        origin=Origin(xyz=(0.007, 0.0, -0.071)),
        material=grey,
        name="finger_tab",
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=dial,
        origin=Origin(xyz=(0.025, 0.064, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=cover,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    dial = object_model.get_part("dial")
    cover = object_model.get_part("service_cover")
    dial_spin = object_model.get_articulation("dial_spin")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.check(
        "dial uses continuous rotation",
        getattr(dial_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"dial joint type is {getattr(dial_spin, 'articulation_type', None)}",
    )
    ctx.expect_overlap(
        dial,
        chassis,
        axes="xz",
        elem_a="dial_shaft",
        elem_b="side_bushing",
        min_overlap=0.008,
        name="dial shaft is centered in side bearing",
    )
    ctx.expect_gap(
        dial,
        chassis,
        axis="y",
        positive_elem="dial_shaft",
        negative_elem="side_bushing",
        max_gap=0.001,
        max_penetration=0.0002,
        name="dial shaft seats at the right side",
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            chassis,
            axes="yz",
            elem_a="cover_panel",
            elem_b="display_glass",
            min_overlap=0.025,
            name="closed cover spans the display face",
        )
        ctx.expect_gap(
            cover,
            chassis,
            axis="x",
            positive_elem="cover_panel",
            negative_elem="display_glass",
            min_gap=0.001,
            max_gap=0.010,
            name="closed cover sits in front of display glass",
        )
        closed_panel = ctx.part_element_world_aabb(cover, elem="cover_panel")

    rest_dial = ctx.part_world_position(dial)
    with ctx.pose({dial_spin: 2.2}):
        turned_dial = ctx.part_world_position(dial)
    ctx.check(
        "dial spins about its fixed short shaft",
        rest_dial is not None
        and turned_dial is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_dial, turned_dial)),
        details=f"rest={rest_dial}, turned={turned_dial}",
    )

    with ctx.pose({cover_hinge: 1.45}):
        opened_panel = ctx.part_element_world_aabb(cover, elem="cover_panel")
    ctx.check(
        "service cover flips upward from top hinge",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[0][2] > closed_panel[0][2] + 0.030
        and opened_panel[1][0] > closed_panel[1][0] + 0.020,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    return ctx.report()


object_model = build_object_model()
