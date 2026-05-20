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
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SLAT_COUNT = 6
SLAT_PITCH = 0.085
SLAT_Z0 = -0.5 * (SLAT_COUNT - 1) * SLAT_PITCH
FRAME_OUTER_WIDTH = 0.78
FRAME_OUTER_HEIGHT = 0.64
SIDE_RAIL_WIDTH = 0.055
RAIL_DEPTH = 0.120
RAIL_THICKNESS = 0.050
SHAFT_RADIUS = 0.008
VANE_ROLL = -0.28


def _hollow_bushing_mesh():
    """Short hollow sleeve whose bore clears a slat shaft."""
    sleeve = (
        cq.Workplane("YZ")
        .circle(0.018)
        .circle(0.008)
        .extrude(0.010, both=True)
    )
    return mesh_from_cadquery(sleeve, "bearing_sleeve", tolerance=0.0007)


def _vane_mesh():
    """Airfoil-like louver blade, extruded along the pivot shaft."""
    profile = [
        (-0.030, -0.0015),
        (-0.022, 0.0040),
        (0.010, 0.0055),
        (0.040, 0.0020),
        (0.043, -0.0010),
        (0.020, -0.0050),
        (-0.020, -0.0040),
    ]
    # With CadQuery's both=True the distance is applied to both sides, so
    # 0.310 m gives a 0.620 m total span.
    vane = cq.Workplane("YZ").polyline(profile).close().extrude(0.310, both=True)
    return mesh_from_cadquery(vane, "airfoil_vane", tolerance=0.0008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_louver_vane_array")

    dark_sheet = Material("dark_bronze_powder_coated_sheet_metal", color=(0.08, 0.075, 0.065, 1.0))
    vane_finish = Material("satin_aluminum_vanes", color=(0.60, 0.62, 0.60, 1.0))
    shaft_finish = Material("brushed_steel_shafts", color=(0.72, 0.72, 0.68, 1.0))
    stop_finish = Material("blackened_stop_tabs", color=(0.03, 0.03, 0.028, 1.0))

    bearing_mesh = _hollow_bushing_mesh()
    vane_mesh = _vane_mesh()

    frame = model.part("frame")

    side_x = 0.5 * FRAME_OUTER_WIDTH - 0.5 * SIDE_RAIL_WIDTH
    top_z = 0.5 * FRAME_OUTER_HEIGHT - 0.5 * RAIL_THICKNESS

    # Deep folded sheet-metal outer frame.
    frame.visual(
        Box((SIDE_RAIL_WIDTH, RAIL_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(-side_x, 0.0, 0.0)),
        material=dark_sheet,
        name="side_rail_0",
    )
    frame.visual(
        Box((SIDE_RAIL_WIDTH, RAIL_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(side_x, 0.0, 0.0)),
        material=dark_sheet,
        name="side_rail_1",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, RAIL_DEPTH, RAIL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=dark_sheet,
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, RAIL_DEPTH, RAIL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -top_z)),
        material=dark_sheet,
        name="bottom_rail",
    )

    # Front and rear return lips make the frame read as formed sheet metal
    # rather than a single flat rectangle.
    for y, lip_name in [(-0.067, "front"), (0.067, "rear")]:
        frame.visual(
            Box((FRAME_OUTER_WIDTH, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, y, top_z + 0.012)),
            material=dark_sheet,
            name=f"{lip_name}_top_lip",
        )
        frame.visual(
            Box((FRAME_OUTER_WIDTH, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, y, -top_z - 0.012)),
            material=dark_sheet,
            name=f"{lip_name}_bottom_lip",
        )
        frame.visual(
            Box((0.026, 0.014, FRAME_OUTER_HEIGHT)),
            origin=Origin(xyz=(-side_x - 0.014, y, 0.0)),
            material=dark_sheet,
            name=f"{lip_name}_side_lip_0",
        )
        frame.visual(
            Box((0.026, 0.014, FRAME_OUTER_HEIGHT)),
            origin=Origin(xyz=(side_x + 0.014, y, 0.0)),
            material=dark_sheet,
            name=f"{lip_name}_side_lip_1",
        )

    # Visible fasteners on the face flanges.
    for x in (-0.300, 0.300):
        for z in (-top_z, top_z):
            frame.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, -0.072, z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=shaft_finish,
                name=f"fastener_{x:+.2f}_{z:+.2f}",
            )

    # One real bearing pair, support webs, and fixed limit tabs for every vane.
    for i in range(SLAT_COUNT):
        z = SLAT_Z0 + i * SLAT_PITCH
        for side, x_sign in enumerate((-1.0, 1.0)):
            x_sleeve = x_sign * 0.325
            x_web = x_sign * 0.334
            frame.visual(
                bearing_mesh,
                origin=Origin(xyz=(x_sleeve, 0.0, z)),
                material=shaft_finish,
                name=f"bearing_{i}_{side}",
            )
            for z_off, label in ((0.026, "upper"), (-0.026, "lower")):
                frame.visual(
                    Box((0.030, 0.034, 0.011)),
                    origin=Origin(xyz=(x_web, 0.0, z + z_off)),
                    material=dark_sheet,
                    name=f"bearing_web_{i}_{side}_{label}",
                )
            # Two small bent stop tabs near each collar.  They are outside the
            # blade span so the vanes can sweep without striking the stops.
            for y, label in ((-0.052, "front"), (0.052, "rear")):
                frame.visual(
                    Box((0.030, 0.011, 0.017)),
                    origin=Origin(xyz=(x_web, y, z + (0.030 if y < 0.0 else -0.030))),
                    material=stop_finish,
                    name=f"stop_tab_{i}_{side}_{label}",
                )

    # Independently articulated louver vanes.  Each part carries its own blade,
    # through-shaft, and shaft collars immediately inboard of the fixed bearings.
    for i in range(SLAT_COUNT):
        z = SLAT_Z0 + i * SLAT_PITCH
        slat = model.part(f"slat_{i}")
        slat.visual(
            vane_mesh,
            origin=Origin(rpy=(VANE_ROLL, 0.0, 0.0)),
            material=vane_finish,
            name="vane_shell",
        )
        slat.visual(
            Cylinder(radius=SHAFT_RADIUS, length=0.646),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=shaft_finish,
            name="pivot_shaft",
        )
        for side, x in enumerate((-0.302, 0.302)):
            slat.visual(
                Cylinder(radius=0.0135, length=0.016),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
                material=shaft_finish,
                name=f"shaft_collar_{side}",
            )
            slat.visual(
                Box((0.018, 0.006, 0.018)),
                origin=Origin(xyz=(x, -0.015, 0.0), rpy=(VANE_ROLL, 0.0, 0.0)),
                material=stop_finish,
                name=f"collar_lug_{side}",
            )

        model.articulation(
            f"frame_to_slat_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-0.62, upper=0.62),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{i}") for i in range(SLAT_COUNT)]
    joints = [object_model.get_articulation(f"frame_to_slat_{i}") for i in range(SLAT_COUNT)]

    ctx.check(
        "every vane has its own revolute joint",
        len(slats) == SLAT_COUNT
        and len(joints) == SLAT_COUNT
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
    )

    for lower, upper in zip(slats, slats[1:]):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.010,
            name=f"{upper.name} clears {lower.name} at rest",
        )

    for i, slat in enumerate(slats):
        for side in range(2):
            bearing = f"bearing_{i}_{side}"
            ctx.allow_overlap(
                frame,
                slat,
                elem_a=bearing,
                elem_b="pivot_shaft",
                reason="The rotating shaft is intentionally captured inside the fixed bearing sleeve.",
            )
            ctx.expect_within(
                slat,
                frame,
                axes="yz",
                inner_elem="pivot_shaft",
                outer_elem=bearing,
                margin=0.001,
                name=f"{slat.name} shaft centered in {bearing}",
            )
            ctx.expect_overlap(
                slat,
                frame,
                axes="x",
                elem_a="pivot_shaft",
                elem_b=bearing,
                min_overlap=0.006,
                name=f"{slat.name} shaft retained in {bearing}",
            )

    ctx.expect_gap(
        frame,
        slats[-1],
        axis="z",
        positive_elem="top_rail",
        min_gap=0.025,
        name="top vane clears upper rail",
    )
    ctx.expect_gap(
        slats[0],
        frame,
        axis="z",
        negative_elem="bottom_rail",
        min_gap=0.025,
        name="bottom vane clears lower rail",
    )

    alternating = {
        joint: (joint.motion_limits.upper if i % 2 else joint.motion_limits.lower)
        for i, joint in enumerate(joints)
    }
    with ctx.pose(alternating):
        for lower, upper in zip(slats, slats[1:]):
            ctx.expect_gap(
                upper,
                lower,
                axis="z",
                min_gap=0.006,
                name=f"{upper.name} clears {lower.name} at alternating limits",
            )
        ctx.expect_gap(
            frame,
            slats[-1],
            axis="z",
            positive_elem="top_rail",
            min_gap=0.015,
            name="top vane clears rail at limit",
        )
        ctx.expect_gap(
            slats[0],
            frame,
            axis="z",
            negative_elem="bottom_rail",
            min_gap=0.015,
            name="bottom vane clears rail at limit",
        )

    return ctx.report()


object_model = build_object_model()
