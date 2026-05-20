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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Plan-rounded appliance box centered on the local origin."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _waffle_plate(
    width: float,
    depth: float,
    thickness: float,
    *,
    ribs_down: bool = False,
) -> cq.Workplane:
    """A thick square cast plate with an integral raised waffle grid."""
    rib_h = 0.009
    rib_w = 0.008
    rim_w = 0.012
    overlap = 0.001
    sign = -1.0 if ribs_down else 1.0

    plate = cq.Workplane("XY").box(width, depth, thickness).edges("|Z").fillet(0.012)
    rib_z = sign * (thickness * 0.5 + rib_h * 0.5 - overlap)

    def add_rib(x: float, y: float, sx: float, sy: float) -> None:
        nonlocal plate
        rib = cq.Workplane("XY").box(sx, sy, rib_h).translate((x, y, rib_z))
        plate = plate.union(rib)

    # Perimeter lip.
    add_rib(-(width * 0.5 - rim_w * 0.5), 0.0, rim_w, depth, )
    add_rib((width * 0.5 - rim_w * 0.5), 0.0, rim_w, depth, )
    add_rib(0.0, -(depth * 0.5 - rim_w * 0.5), width, rim_w, )
    add_rib(0.0, (depth * 0.5 - rim_w * 0.5), width, rim_w, )

    # Interior waffle grid, leaving four-by-four square pockets.
    clear_w = width - 2.0 * rim_w
    clear_d = depth - 2.0 * rim_w
    for i in range(1, 4):
        x = -clear_w * 0.5 + clear_w * i / 4.0
        y = -clear_d * 0.5 + clear_d * i / 4.0
        add_rib(x, 0.0, rib_w, clear_d)
        add_rib(0.0, y, clear_w, rib_w)

    return plate


def _front_handle() -> cq.Workplane:
    """Short fused U-shaped handle mounted to the upper front edge."""
    grip = cq.Workplane("XY").box(0.034, 0.170, 0.026).translate((0.420, 0.0, 0.026))
    arm_a = cq.Workplane("XY").box(0.090, 0.026, 0.022).translate((0.374, 0.067, 0.020))
    arm_b = cq.Workplane("XY").box(0.090, 0.026, 0.022).translate((0.374, -0.067, 0.020))
    return grip.union(arm_a).union(arm_b)


def _hinge_straps() -> cq.Workplane:
    """Two short leaves that tie the rotating hinge barrels into the upper cover."""
    strap_a = cq.Workplane("XY").box(0.060, 0.060, 0.014).translate((0.030, 0.112, 0.017))
    strap_b = cq.Workplane("XY").box(0.060, 0.060, 0.014).translate((0.030, -0.112, 0.017))
    return strap_a.union(strap_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_clamshell_waffle_maker")

    enamel = model.material("warm_dark_enamel", rgba=(0.10, 0.105, 0.095, 1.0))
    cast_iron = model.material("matte_cast_iron", rgba=(0.025, 0.027, 0.026, 1.0))
    black = model.material("black_bakelite", rgba=(0.006, 0.006, 0.005, 1.0))
    metal = model.material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    pointer_white = model.material("dial_mark_white", rgba=(0.92, 0.90, 0.82, 1.0))

    lower = model.part("lower")
    lower.visual(
        mesh_from_cadquery(_rounded_box((0.390, 0.390, 0.080), 0.035), "lower_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=enamel,
        name="lower_shell",
    )
    lower.visual(
        mesh_from_cadquery(_waffle_plate(0.315, 0.315, 0.026), "lower_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=cast_iron,
        name="lower_plate",
    )
    lower.visual(
        Box((0.052, 0.140, 0.068)),
        origin=Origin(xyz=(-0.207, 0.0, 0.125)),
        material=enamel,
        name="rear_hinge_block",
    )
    lower.visual(
        Cylinder(radius=0.008, length=0.350),
        origin=Origin(xyz=(-0.195, 0.0, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    for i, (x, y) in enumerate(
        ((-0.130, -0.130), (-0.130, 0.130), (0.130, -0.130), (0.130, 0.130))
    ):
        lower.visual(
            Box((0.060, 0.060, 0.014)),
            origin=Origin(xyz=(x, y, 0.006)),
            material=black,
            name=f"foot_{i}",
        )

    top = model.part("top")
    top.visual(
        mesh_from_cadquery(_rounded_box((0.335, 0.360, 0.070), 0.032), "top_cover"),
        origin=Origin(xyz=(0.205, 0.0, 0.034)),
        material=enamel,
        name="top_cover",
    )
    top.visual(
        mesh_from_cadquery(_waffle_plate(0.315, 0.315, 0.026, ribs_down=True), "top_plate"),
        origin=Origin(xyz=(0.192, 0.0, -0.014)),
        material=cast_iron,
        name="top_plate",
    )
    top.visual(
        mesh_from_cadquery(_front_handle(), "front_handle"),
        material=black,
        name="front_handle",
    )
    top.visual(
        mesh_from_cadquery(_hinge_straps(), "hinge_straps"),
        material=metal,
        name="hinge_straps",
    )
    for i, y in enumerate((-0.112, 0.112)):
        top.visual(
            Cylinder(radius=0.015, length=0.062),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.020,
                body_style="faceted",
                top_diameter=0.044,
                edge_radius=0.001,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0012),
                center=False,
            ),
            "temperature_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="temperature_dial",
    )
    dial.visual(
        Box((0.005, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, -0.0215, 0.011)),
        material=pointer_white,
        name="dial_pointer",
    )

    model.articulation(
        "lower_to_top",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=top,
        origin=Origin(xyz=(-0.195, 0.0, 0.165)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lower_to_dial",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=dial,
        origin=Origin(xyz=(0.090, -0.195, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower")
    top = object_model.get_part("top")
    dial = object_model.get_part("dial")
    top_hinge = object_model.get_articulation("lower_to_top")
    dial_joint = object_model.get_articulation("lower_to_dial")

    for barrel in ("hinge_barrel_0", "hinge_barrel_1"):
        ctx.allow_overlap(
            lower,
            top,
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The steel hinge pin is intentionally captured inside the rotating hinge barrel.",
        )
        ctx.expect_overlap(
            lower,
            top,
            axes="y",
            elem_a="hinge_pin",
            elem_b=barrel,
            min_overlap=0.050,
            name=f"{barrel} stays captured on hinge pin",
        )

    with ctx.pose({top_hinge: 0.0, dial_joint: 0.0}):
        ctx.expect_gap(
            top,
            lower,
            axis="z",
            positive_elem="top_plate",
            negative_elem="lower_plate",
            min_gap=0.002,
            max_gap=0.008,
            name="closed cooking plates leave waffle clearance",
        )
        ctx.expect_overlap(
            top,
            lower,
            axes="xy",
            elem_a="top_plate",
            elem_b="lower_plate",
            min_overlap=0.290,
            name="square cooking plates align in the closed pose",
        )
        ctx.expect_gap(
            lower,
            dial,
            axis="y",
            positive_elem="lower_shell",
            negative_elem="temperature_dial",
            max_gap=0.002,
            max_penetration=0.0005,
            name="temperature dial is seated on the side housing",
        )
        ctx.expect_overlap(
            dial,
            lower,
            axes="xz",
            elem_a="temperature_dial",
            elem_b="lower_shell",
            min_overlap=0.040,
            name="temperature dial footprint sits on lower housing side",
        )

    with ctx.pose({top_hinge: 1.25}):
        ctx.expect_gap(
            top,
            lower,
            axis="z",
            positive_elem="top_plate",
            negative_elem="lower_plate",
            min_gap=0.055,
            name="top plate lifts clear on rear hinge",
        )

    with ctx.pose({dial_joint: 1.2}):
        ctx.expect_gap(
            lower,
            dial,
            axis="y",
            positive_elem="lower_shell",
            negative_elem="temperature_dial",
            max_gap=0.002,
            max_penetration=0.0005,
            name="temperature dial rotates about its side shaft without drifting",
        )

    return ctx.report()


object_model = build_object_model()
