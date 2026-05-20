from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROLL_Z = 0.175
SUPPORT_X = 0.145
SUPPORT_INNER_X = 0.124
BARREL_HALF_LENGTH = 0.105
BARREL_COLLAR_OUTER_X = 0.118
JOURNAL_RADIUS = 0.011
BEARING_BORE_RADIUS = 0.0145


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float):
    """CadQuery annular sleeve centered on local Z, later rotated to the roll axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _x_cylinder_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Origin for a URDF/CadQuery cylinder whose local Z is used as world X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_sensor_roll_module",
        meta={
            "journal_radius": JOURNAL_RADIUS,
            "bearing_bore_radius": BEARING_BORE_RADIUS,
            "support_inner_x": SUPPORT_INNER_X,
            "barrel_collar_outer_x": BARREL_COLLAR_OUTER_X,
        },
    )

    cast_iron = model.material("graphite_cast_frame", color=(0.16, 0.17, 0.17, 1.0))
    satin_black = model.material("satin_black_trim", color=(0.015, 0.017, 0.018, 1.0))
    parkerized = model.material("parkerized_barrel", color=(0.045, 0.050, 0.052, 1.0))
    bearing_steel = model.material("brushed_bearing_steel", color=(0.62, 0.64, 0.62, 1.0))
    glass = model.material("smoked_sensor_glass", color=(0.03, 0.08, 0.10, 0.82))
    rubber = model.material("black_rubber", color=(0.006, 0.006, 0.005, 1.0))

    frame = model.part("frame")

    # Grounded compact base and short pedestal.  The stacked, slightly proud
    # layers keep the assembly dense rather than making the roll head look tall
    # or spindly.
    frame.visual(
        Box((0.42, 0.17, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        Box((0.34, 0.115, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=cast_iron,
        name="pedestal",
    )
    frame.visual(
        Box((0.27, 0.104, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=cast_iron,
        name="top_saddle",
    )
    for y in (-0.061, 0.061):
        frame.visual(
            Box((0.35, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.083)),
            material=satin_black,
            name=f"side_rail_{0 if y < 0 else 1}",
        )

    # Two coaxial supports carry the bearing housings.  Their top edges embed
    # into the lower arc of the bearing housings, leaving the journal bore open.
    for side, x, support_name, cartridge_name in (
        (0, -SUPPORT_X, "support_0", "bearing_cartridge_0"),
        (1, SUPPORT_X, "support_1", "bearing_cartridge_1"),
    ):
        frame.visual(
            Box((0.041, 0.058, 0.082)),
            origin=Origin(xyz=(x, 0.0, 0.104)),
            material=cast_iron,
            name=support_name,
        )
        frame.visual(
            Box((0.066, 0.080, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.082)),
            material=cast_iron,
            name=f"support_foot_{side}",
        )

        housing = _annular_cylinder(0.043, 0.0185, 0.030)
        frame.visual(
            mesh_from_cadquery(housing, f"bearing_housing_{side}", tolerance=0.0008),
            origin=_x_cylinder_origin(x, 0.0, ROLL_Z),
            material=cast_iron,
            name=f"bearing_housing_{side}",
        )

        cartridge = _annular_cylinder(0.030, BEARING_BORE_RADIUS, 0.036)
        frame.visual(
            mesh_from_cadquery(cartridge, cartridge_name, tolerance=0.0006),
            origin=_x_cylinder_origin(x, 0.0, ROLL_Z),
            material=bearing_steel,
            name=cartridge_name,
        )

        # Outer flange and black cover ring, just proud of the support face.
        flange_x = -0.164 if x < 0 else 0.164
        flange = _annular_cylinder(0.050, 0.0185, 0.008)
        frame.visual(
            mesh_from_cadquery(flange, f"flange_ring_{side}", tolerance=0.0008),
            origin=_x_cylinder_origin(flange_x, 0.0, ROLL_Z),
            material=satin_black,
            name=f"flange_ring_{side}",
        )
        cover = _annular_cylinder(0.036, 0.0165, 0.006)
        cover_x = -0.169 if x < 0 else 0.169
        frame.visual(
            mesh_from_cadquery(cover, f"trim_cover_{side}", tolerance=0.0008),
            origin=_x_cylinder_origin(cover_x, 0.0, ROLL_Z),
            material=satin_black,
            name=f"trim_cover_{side}",
        )

        # Four cap screws sit into each flange face.  They are intentionally
        # slightly embedded so they read as installed fasteners, not floating dots.
        bolt_face_x = -0.170 if x < 0 else 0.170
        for idx, (by, bz) in enumerate(
            ((-0.030, ROLL_Z), (0.030, ROLL_Z), (0.0, ROLL_Z + 0.030), (0.0, ROLL_Z - 0.030))
        ):
            frame.visual(
                Cylinder(radius=0.0042, length=0.004),
                origin=_x_cylinder_origin(bolt_face_x, by, bz),
                material=bearing_steel,
                name=f"flange_bolt_{side}_{idx}",
            )

    for idx, (x, y) in enumerate(
        ((-0.175, -0.062), (0.175, -0.062), (-0.175, 0.062), (0.175, 0.062))
    ):
        frame.visual(
            Box((0.070, 0.026, 0.007)),
            origin=Origin(xyz=(x, y, 0.0035)),
            material=rubber,
            name=f"foot_pad_{idx}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=0.354),
        origin=_x_cylinder_origin(0.0, 0.0, 0.0),
        material=bearing_steel,
        name="journal_shaft",
    )
    barrel.visual(
        Cylinder(radius=0.0235, length=BARREL_HALF_LENGTH * 2.0),
        origin=_x_cylinder_origin(0.0, 0.0, 0.0),
        material=parkerized,
        name="barrel_shell",
    )
    for side, x, collar_name in ((0, -0.112, "end_collar_0"), (1, 0.112, "end_collar_1")):
        barrel.visual(
            Cylinder(radius=0.0250, length=0.012),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=satin_black,
            name=collar_name,
        )
    for side, x in enumerate((-0.072, 0.072)):
        barrel.visual(
            Cylinder(radius=0.0258, length=0.008),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=satin_black,
            name=f"barrel_band_{side}",
        )
    barrel.visual(
        Box((0.116, 0.013, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0246)),
        material=glass,
        name="sensor_window",
    )
    barrel.visual(
        Box((0.072, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.0244, 0.0)),
        material=satin_black,
        name="index_rail",
    )
    for side, x in enumerate((-0.178, 0.178)):
        barrel.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=bearing_steel,
            name=f"shaft_nose_{side}",
        )
    for side, x, washer_name in ((0, -0.1745, "thrust_washer_0"), (1, 0.1745, "thrust_washer_1")):
        barrel.visual(
            Cylinder(radius=0.022, length=0.005),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=bearing_steel,
            name=washer_name,
        )

    model.articulation(
        "frame_to_barrel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, ROLL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=6.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    barrel = object_model.get_part("barrel")
    roll = object_model.get_articulation("frame_to_barrel")

    ctx.check(
        "barrel uses a single revolute roll joint",
        roll.articulation_type == ArticulationType.REVOLUTE and tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"type={roll.articulation_type}, axis={roll.axis}",
    )

    ctx.expect_within(
        barrel,
        frame,
        axes="yz",
        inner_elem="journal_shaft",
        outer_elem="bearing_cartridge_0",
        margin=0.0,
        name="journal is centered inside first bearing cartridge",
    )
    ctx.expect_within(
        barrel,
        frame,
        axes="yz",
        inner_elem="journal_shaft",
        outer_elem="bearing_cartridge_1",
        margin=0.0,
        name="journal is centered inside second bearing cartridge",
    )
    ctx.expect_overlap(
        barrel,
        frame,
        axes="x",
        elem_a="journal_shaft",
        elem_b="bearing_cartridge_0",
        min_overlap=0.020,
        name="first bearing retains the journal axially",
    )
    ctx.expect_overlap(
        barrel,
        frame,
        axes="x",
        elem_a="journal_shaft",
        elem_b="bearing_cartridge_1",
        min_overlap=0.020,
        name="second bearing retains the journal axially",
    )
    ctx.expect_gap(
        frame,
        barrel,
        axis="x",
        positive_elem="support_1",
        negative_elem="end_collar_1",
        min_gap=0.003,
        name="positive support clears the collar",
    )
    ctx.expect_gap(
        barrel,
        frame,
        axis="x",
        positive_elem="end_collar_0",
        negative_elem="support_0",
        min_gap=0.003,
        name="negative support clears the collar",
    )
    ctx.check(
        "bearing bores are larger than the rotating journal",
        object_model.meta["bearing_bore_radius"] - object_model.meta["journal_radius"] >= 0.003,
        details=f"bore={object_model.meta['bearing_bore_radius']}, journal={object_model.meta['journal_radius']}",
    )

    with ctx.pose({roll: math.pi / 2.0}):
        ctx.expect_gap(
            frame,
            barrel,
            axis="x",
            positive_elem="support_1",
            negative_elem="end_collar_1",
            min_gap=0.003,
            name="rolled positive support still clears collar",
        )
        ctx.expect_gap(
            barrel,
            frame,
            axis="x",
            positive_elem="end_collar_0",
            negative_elem="support_0",
            min_gap=0.003,
            name="rolled negative support still clears collar",
        )

    return ctx.report()


object_model = build_object_model()
