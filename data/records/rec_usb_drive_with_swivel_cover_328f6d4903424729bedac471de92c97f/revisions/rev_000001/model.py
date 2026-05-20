from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cad_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    """CadQuery box centered at an explicit local coordinate."""
    return cq.Workplane("XY").box(*size).translate(center)


def _cad_z_cylinder(radius: float, height: float, center: tuple[float, float, float]):
    """CadQuery cylinder along local Z, centered at an explicit coordinate."""
    x, y, z = center
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z - height / 2.0))
    )


def _rugged_body_shell():
    shell = cq.Workplane("XY").box(0.062, 0.024, 0.012)
    try:
        shell = shell.edges("|Z").fillet(0.0026)
    except Exception:
        # The straight box is still a valid molded shell if a backend fillet
        # degenerates on a small edge.
        pass
    return shell.translate((-0.011, 0.0, 0.0))


def _swivel_cover_shell():
    # Local cover frame origin is the pivot pin.  In the default pose the
    # cover points along +Y, leaving the USB plug exposed.  A -90 degree
    # rotation brings the offset channel over the connector.
    x_min = 0.005
    x_max = 0.029
    width = x_max - x_min
    x_center = (x_min + x_max) / 2.0
    length = 0.056
    y_center = 0.032

    top_plate = _cad_box((width, length, 0.0015), (x_center, y_center, 0.0075))
    side_outer = _cad_box((0.0015, length, 0.0115), (x_min + 0.00075, y_center, 0.0017))
    side_inner = _cad_box((0.0015, length, 0.0115), (x_max - 0.00075, y_center, 0.0017))
    nose_bridge = _cad_box((width, 0.0022, 0.0115), (x_center, 0.0608, 0.0017))

    outer_eye = _cad_z_cylinder(0.0039, 0.0110, (0.0, 0.0, 0.0))
    ear_web = _cad_box((0.019, 0.012, 0.0100), (0.0095, 0.0060, 0.0006))
    relief = _cad_z_cylinder(0.0030, 0.0145, (0.0, 0.0, 0.0))

    cover = (
        top_plate.union(side_outer)
        .union(side_inner)
        .union(nose_bridge)
        .union(outer_eye)
        .union(ear_web)
        .cut(relief)
    )
    try:
        cover = cover.edges().fillet(0.00035)
    except Exception:
        pass
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_usb_swivel_drive")

    molded = model.material("molded_black", rgba=(0.015, 0.018, 0.020, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.035, 0.040, 0.043, 1.0))
    armor = model.material("painted_armor", rgba=(0.16, 0.18, 0.17, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.57, 1.0))
    dark_steel = model.material("blackened_screw", rgba=(0.05, 0.052, 0.048, 1.0))
    gold = model.material("contact_gold", rgba=(1.0, 0.70, 0.20, 1.0))
    tongue_mat = model.material("connector_insert", rgba=(0.025, 0.027, 0.030, 1.0))
    cover_metal = model.material("painted_steel", rgba=(0.34, 0.37, 0.36, 1.0))

    drive = model.part("drive")
    drive.visual(
        mesh_from_cadquery(_rugged_body_shell(), "main_shell", tolerance=0.0006),
        material=molded,
        name="main_shell",
    )

    # Overmolded rubber rails and raised armor panels make the compact body read
    # as a heavy-use utility drive rather than a simple plastic stick.
    for idx, y in enumerate((-0.0130, 0.0130)):
        drive.visual(
            Box((0.058, 0.0040, 0.0105)),
            origin=Origin(xyz=(-0.012, y, 0.0)),
            material=rubber,
            name=f"side_rail_{idx}",
        )
    drive.visual(
        Box((0.041, 0.0165, 0.0022)),
        origin=Origin(xyz=(-0.013, 0.0, 0.0065)),
        material=armor,
        name="top_plate",
    )
    for idx, x in enumerate((-0.027, -0.017, -0.007, 0.003)):
        drive.visual(
            Box((0.0032, 0.0180, 0.0024)),
            origin=Origin(xyz=(x, 0.0, 0.0078), rpy=(0.0, 0.0, 0.35)),
            material=rubber,
            name=f"grip_rib_{idx}",
        )

    # Rear lanyard loop: four molded bars leave a real visible opening.
    drive.visual(
        Box((0.0120, 0.0042, 0.0105)),
        origin=Origin(xyz=(-0.0470, -0.0100, 0.0)),
        material=rubber,
        name="lanyard_side_0",
    )
    drive.visual(
        Box((0.0120, 0.0042, 0.0105)),
        origin=Origin(xyz=(-0.0470, 0.0100, 0.0)),
        material=rubber,
        name="lanyard_side_1",
    )
    drive.visual(
        Box((0.0038, 0.0242, 0.0105)),
        origin=Origin(xyz=(-0.0522, 0.0, 0.0)),
        material=rubber,
        name="lanyard_end",
    )

    # USB-A plug, built as a hollow metal sleeve with black tongue and contacts.
    plug_center_x = 0.0355
    drive.visual(
        Box((0.0270, 0.0160, 0.0010)),
        origin=Origin(xyz=(plug_center_x, 0.0, 0.0033)),
        material=steel,
        name="usb_top",
    )
    drive.visual(
        Box((0.0270, 0.0160, 0.0010)),
        origin=Origin(xyz=(plug_center_x, 0.0, -0.0033)),
        material=steel,
        name="usb_bottom",
    )
    for idx, y in enumerate((-0.0080, 0.0080)):
        drive.visual(
            Box((0.0270, 0.0011, 0.0066)),
            origin=Origin(xyz=(plug_center_x, y, 0.0)),
            material=steel,
            name=f"usb_side_{idx}",
        )
    drive.visual(
        Box((0.0030, 0.0160, 0.0066)),
        origin=Origin(xyz=(0.0210, 0.0, 0.0)),
        material=steel,
        name="usb_root_band",
    )
    drive.visual(
        Box((0.0200, 0.0068, 0.0016)),
        origin=Origin(xyz=(0.0380, 0.0, -0.0008)),
        material=tongue_mat,
        name="usb_tongue",
    )
    drive.visual(
        Box((0.0100, 0.0068, 0.0016)),
        origin=Origin(xyz=(0.0245, 0.0, -0.0008)),
        material=tongue_mat,
        name="tongue_anchor",
    )
    for idx, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        drive.visual(
            Box((0.0060, 0.0010, 0.00035)),
            origin=Origin(xyz=(0.0430, y, 0.00005)),
            material=gold,
            name=f"contact_{idx}",
        )

    # Exposed service fasteners.
    for idx, (x, y) in enumerate(((-0.031, -0.0074), (-0.031, 0.0074), (0.006, -0.0074), (0.006, 0.0074))):
        drive.visual(
            Cylinder(radius=0.00155, length=0.0010),
            origin=Origin(xyz=(x, y, 0.0080)),
            material=dark_steel,
            name=f"body_screw_{idx}",
        )

    # Side pivot hardware: a through pin with top/bottom capture ears leaves the
    # moving cover eye visibly retained but clearanced.
    pin_x, pin_y = 0.0170, 0.0170
    drive.visual(
        Cylinder(radius=0.00215, length=0.0180),
        origin=Origin(xyz=(pin_x, pin_y, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    for idx, z in enumerate((-0.0075, 0.0075)):
        drive.visual(
            Cylinder(radius=0.0042, length=0.0016),
            origin=Origin(xyz=(pin_x, pin_y, z)),
            material=dark_steel,
            name=f"pin_head_{idx}",
        )
        drive.visual(
            Box((0.0100, 0.0062, 0.0016)),
            origin=Origin(xyz=(pin_x, 0.0143, z)),
            material=dark_steel,
            name=f"pin_ear_{idx}",
        )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_swivel_cover_shell(), "cover_shell", tolerance=0.00045),
        material=cover_metal,
        name="cover_shell",
    )
    cover_screw_layout = (
        ("cover_screw_0", 0.011, 0.025),
        ("cover_screw_1", 0.023, 0.025),
        ("cover_screw_2", 0.011, 0.048),
        ("cover_screw_3", 0.023, 0.048),
    )
    for screw_name, x, y in cover_screw_layout:
        cover.visual(
            Cylinder(radius=0.00125, length=0.0009),
            origin=Origin(xyz=(x, y, 0.00855)),
            material=dark_steel,
            name=screw_name,
        )
    cover.visual(
        Box((0.018, 0.0020, 0.0024)),
        origin=Origin(xyz=(0.017, 0.0617, -0.0046)),
        material=rubber,
        name="nose_bumper",
    )

    model.articulation(
        "cover_pivot",
        ArticulationType.REVOLUTE,
        parent=drive,
        child=cover,
        origin=Origin(xyz=(pin_x, pin_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=5.0, lower=-math.pi / 2.0, upper=math.pi * 0.95),
        motion_properties=MotionProperties(damping=0.025, friction=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drive = object_model.get_part("drive")
    cover = object_model.get_part("cover")
    pivot = object_model.get_articulation("cover_pivot")

    ctx.check(
        "swivel cover has wide rotation",
        pivot.motion_limits is not None
        and pivot.motion_limits.lower <= -math.pi / 2.0
        and pivot.motion_limits.upper >= math.pi * 0.9,
        details=f"limits={pivot.motion_limits}",
    )

    # Default pose is open: the plug is exposed and the metal cover sits clear
    # of the molded body side.
    ctx.expect_gap(
        cover,
        drive,
        axis="y",
        positive_elem="cover_shell",
        negative_elem="main_shell",
        min_gap=0.0008,
        name="open cover clears side",
    )
    ctx.expect_gap(
        drive,
        drive,
        axis="x",
        positive_elem="usb_top",
        negative_elem="main_shell",
        min_gap=0.0005,
        name="usb shell protrudes from body",
    )
    ctx.expect_within(
        drive,
        drive,
        axes="xy",
        inner_elem="usb_tongue",
        outer_elem="usb_top",
        margin=0.001,
        name="usb tongue stays inside sleeve",
    )

    # At the closed stop the same cover rotates over the connector rather than
    # translating or detaching; the channel remains above the USB metal shell.
    with ctx.pose({pivot: -math.pi / 2.0}):
        ctx.expect_overlap(
            cover,
            drive,
            axes="xy",
            elem_a="cover_shell",
            elem_b="usb_top",
            min_overlap=0.010,
            name="closed cover spans connector",
        )
        ctx.expect_gap(
            cover,
            drive,
            axis="z",
            positive_elem="cover_screw_0",
            negative_elem="usb_top",
            min_gap=0.0006,
            max_gap=0.008,
            name="closed cover sits above plug",
        )

    return ctx.report()


object_model = build_object_model()
