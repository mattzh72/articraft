from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_dial")

    porcelain = Material("warm_white_plastic", rgba=(0.92, 0.89, 0.80, 1.0))
    face_ivory = Material("ivory_body", rgba=(0.82, 0.80, 0.72, 1.0))
    dial_plastic = Material("dial_plastic", rgba=(0.96, 0.95, 0.88, 1.0))
    dial_ring_plastic = Material("dial_ring_plastic", rgba=(0.88, 0.87, 0.80, 1.0))
    dark_print = Material("dark_temperature_print", rgba=(0.08, 0.08, 0.07, 1.0))
    metal = Material("brushed_screw_metal", rgba=(0.55, 0.54, 0.50, 1.0))

    plate_thickness = 0.006
    body_depth = 0.018
    body_front_z = plate_thickness + body_depth

    wall_plate = model.part("wall_plate")
    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.142, 0.170, 0.012, corner_segments=8),
            plate_thickness,
        ),
        "thermostat_wall_plate",
    )
    wall_plate.visual(plate_mesh, material=porcelain, name="plate")

    body_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.116, 0.128, 0.018, corner_segments=10),
            body_depth,
        ),
        "thermostat_shallow_body",
    )
    wall_plate.visual(
        body_mesh,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness)),
        material=face_ivory,
        name="body",
    )

    # Screw heads on the wall plate outside the raised thermostat body.
    for y, name in ((0.073, "upper_screw"), (-0.073, "lower_screw")):
        wall_plate.visual(
            Cylinder(radius=0.006, length=0.0016),
            origin=Origin(xyz=(0.0, y, plate_thickness + 0.0008)),
            material=metal,
            name=name,
        )
        wall_plate.visual(
            Box((0.009, 0.0014, 0.0008)),
            origin=Origin(xyz=(0.0, y, plate_thickness + 0.0020)),
            material=dark_print,
            name=f"{name}_slot",
        )

    # Raised printed temperature ticks sit on the body face around the dial.
    tick_radius = 0.052
    for idx, deg in enumerate(range(-120, 121, 20)):
        angle = math.radians(deg)
        is_major = deg % 40 == 0
        tick_len = 0.0085 if is_major else 0.0058
        tick_width = 0.0015 if is_major else 0.0010
        wall_plate.visual(
            Box((tick_len, tick_width, 0.0008)),
            origin=Origin(
                xyz=(
                    tick_radius * math.sin(angle),
                    tick_radius * math.cos(angle),
                    body_front_z + 0.0004,
                ),
                rpy=(0.0, 0.0, -angle),
            ),
            material=dark_print,
            name=f"tick_{idx}",
        )

    dial = model.part("dial")
    dial_profile = [
        (0.0, 0.0),
        (0.039, 0.0),
        (0.039, 0.0052),
        (0.036, 0.0080),
        (0.032, 0.0106),
        (0.022, 0.0106),
        (0.022, 0.0124),
        (0.0, 0.0124),
    ]
    dial.visual(
        mesh_from_geometry(LatheGeometry(dial_profile, segments=72), "large_rotary_dial"),
        material=dial_plastic,
        name="dial_shell",
    )
    dial.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.027, tube=0.0014, radial_segments=16, tubular_segments=72),
            "raised_dial_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0119)),
        material=dial_ring_plastic,
        name="dial_ring",
    )
    dial.visual(
        Box((0.0040, 0.027, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0135, 0.0129)),
        material=dark_print,
        name="pointer_line",
    )

    model.articulation(
        "dial_axis",
        ArticulationType.CONTINUOUS,
        parent=wall_plate,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, body_front_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    dial = object_model.get_part("dial")
    joint = object_model.get_articulation("dial_axis")

    ctx.check(
        "dial joint is continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={joint.articulation_type}",
    )
    ctx.check(
        "dial axis is normal to the wall body",
        tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_within(
        dial,
        wall_plate,
        axes="xy",
        inner_elem="dial_shell",
        outer_elem="body",
        margin=0.0,
        name="large dial remains centered within body outline",
    )
    ctx.expect_gap(
        dial,
        wall_plate,
        axis="z",
        positive_elem="dial_shell",
        negative_elem="body",
        max_gap=0.0002,
        max_penetration=0.0,
        name="dial rear face sits on body front",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({joint: math.pi * 1.5}):
        spun_pos = ctx.part_world_position(dial)
        ctx.expect_within(
            dial,
            wall_plate,
            axes="xy",
            inner_elem="dial_shell",
            outer_elem="body",
            margin=0.0,
            name="spun dial stays on centerline",
        )
    ctx.check(
        "continuous rotation does not shift dial origin",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
