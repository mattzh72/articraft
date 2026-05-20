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


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float):
    """A hollow cylindrical mesh, centered on the local Z axis."""

    outer = cq.Workplane("XY").cylinder(length, outer_radius)
    cutter = cq.Workplane("XY").cylinder(length * 1.6, inner_radius)
    return outer.cut(cutter).edges().chamfer(min(0.006, (outer_radius - inner_radius) * 0.2))


def _bearing_cartridge():
    """Stepped trunnion bearing with a real clear bore."""

    flange = _annular_cylinder(0.235, 0.105, 0.050).translate((0.0, 0.0, -0.030))
    raised_hub = _annular_cylinder(0.165, 0.105, 0.118).translate((0.0, 0.0, 0.006))
    return flange.union(raised_hub).edges().chamfer(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_axis_rotary_fixture")

    cast_dark = model.material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    cast_blue = model.material("blue_gray_casting", rgba=(0.20, 0.25, 0.29, 1.0))
    machined = model.material("machined_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    plate_mat = model.material("ground_tooling_plate", rgba=(0.44, 0.47, 0.48, 1.0))
    black = model.material("black_oxide", rgba=(0.015, 0.015, 0.014, 1.0))
    cover = model.material("service_cover_gray", rgba=(0.12, 0.14, 0.15, 1.0))
    brass = model.material("oiled_bronze", rgba=(0.78, 0.56, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.72, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=cast_dark,
        name="wide_floor_drum",
    )
    base.visual(
        Cylinder(radius=0.53, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=cast_dark,
        name="main_drum",
    )
    base.visual(
        Cylinder(radius=0.62, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.382)),
        material=machined,
        name="lower_shoulder",
    )
    base.visual(
        Cylinder(radius=0.42, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.457)),
        material=machined,
        name="vertical_bearing_face",
    )
    # Leveling feet and a bolted foundation circle keep the grounded mass legible.
    for i in range(8):
        angle = 2.0 * math.pi * i / 8.0
        x = 0.58 * math.cos(angle)
        y = 0.58 * math.sin(angle)
        base.visual(
            Cylinder(radius=0.035, length=0.020),
            origin=Origin(xyz=(x, y, 0.120)),
            material=black,
            name=f"foundation_bolt_{i}",
        )
    for i, angle in enumerate((math.radians(35), math.radians(145), math.radians(215), math.radians(325))):
        x = 0.69 * math.cos(angle)
        y = 0.69 * math.sin(angle)
        base.visual(
            Cylinder(radius=0.055, length=0.030),
            origin=Origin(xyz=(x, y, 0.015)),
            material=machined,
            name=f"leveling_foot_{i}",
        )
    for i, (angle, label) in enumerate(((0.0, "front"), (math.pi, "rear"))):
        # Rectangular service covers are lightly sunk into the drum skin.
        x = 0.522 * math.cos(angle)
        y = 0.522 * math.sin(angle)
        base.visual(
            Box((0.016, 0.240, 0.115)),
            origin=Origin(xyz=(x, y, 0.245), rpy=(0.0, 0.0, angle)),
            material=cover,
            name=f"{label}_service_cover",
        )
        for j, dz in enumerate((-0.040, 0.040)):
            base.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(
                    xyz=(0.532 * math.cos(angle), 0.532 * math.sin(angle), 0.245 + dz),
                    rpy=(0.0, math.pi / 2.0, angle),
                ),
                material=black,
                name=f"{label}_cover_screw_{j}",
            )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.405, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=machined,
        name="rotary_table",
    )
    yoke.visual(
        Cylinder(radius=0.590, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=cast_blue,
        name="table_flange",
    )
    yoke.visual(
        Box((1.92, 0.92, 0.170)),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=cast_blue,
        name="saddle_deck",
    )
    # Low rails and stops on the rotating table; outside the plate sweep radius.
    for y in (-0.48, 0.48):
        yoke.visual(
            Box((1.72, 0.055, 0.075)),
            origin=Origin(xyz=(0.0, y, 0.3625)),
            material=machined,
            name=f"deck_rail_{0 if y < 0 else 1}",
        )
    for x in (-0.41, 0.41):
        yoke.visual(
            Box((0.120, 0.130, 0.105)),
            origin=Origin(xyz=(x, 0.50, 0.3775)),
            material=black,
            name=f"tilt_stop_{0 if x < 0 else 1}",
        )
    # Two deep side towers, cast into the deck with shoulder blocks and ribs.
    for side, x in enumerate((-0.75, 0.75)):
        if side == 0:
            yoke.visual(
                Box((0.320, 0.700, 1.185)),
                origin=Origin(xyz=(x, 0.0, 0.890)),
                material=cast_blue,
                name="tower_0",
            )
        else:
            yoke.visual(
                Box((0.320, 0.700, 1.185)),
                origin=Origin(xyz=(x, 0.0, 0.890)),
                material=cast_blue,
                name="tower_1",
            )
        yoke.visual(
            Box((0.420, 0.780, 0.190)),
            origin=Origin(xyz=(x, 0.0, 0.380)),
            material=cast_blue,
            name=f"tower_foot_{side}",
        )
        yoke.visual(
            Box((0.260, 0.820, 0.070)),
            origin=Origin(xyz=(x, 0.0, 1.505)),
            material=machined,
            name=f"tower_cap_{side}",
        )
        # Front/rear gussets are deliberately thick and overlap the deck/tower.
        for y in (-0.365, 0.365):
            yoke.visual(
                Box((0.095, 0.090, 0.820)),
                origin=Origin(xyz=(x, y, 0.785)),
                material=cast_blue,
                name=f"tower_rib_{side}_{0 if y < 0 else 1}",
            )
        # Service access plate on the outer face of each tower.
        outer_x = x + (0.166 if x > 0 else -0.166)
        yoke.visual(
            Box((0.016, 0.315, 0.420)),
            origin=Origin(xyz=(outer_x, 0.0, 0.880)),
            material=cover,
            name=f"tower_cover_{side}",
        )
        for j, (yy, zz) in enumerate(((-0.120, 0.720), (0.120, 0.720), (-0.120, 1.040), (0.120, 1.040))):
            yoke.visual(
                Cylinder(radius=0.012, length=0.014),
                origin=Origin(xyz=(outer_x, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"tower_cover_screw_{side}_{j}",
            )
        # Hollow trunnion cartridge on the inner face.  The bore clears the shaft.
        inner_x = x - (0.205 if x > 0 else -0.205)
        if side == 0:
            yoke.visual(
                mesh_from_cadquery(_bearing_cartridge(), "bearing_ring_0", tolerance=0.0008),
                origin=Origin(xyz=(inner_x, 0.0, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=machined,
                name="bearing_ring_0",
            )
        else:
            yoke.visual(
                mesh_from_cadquery(_bearing_cartridge(), "bearing_ring_1", tolerance=0.0008),
                origin=Origin(xyz=(inner_x, 0.0, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=machined,
                name="bearing_ring_1",
            )
        yoke.visual(
            mesh_from_cadquery(_annular_cylinder(0.128, 0.105, 0.030), f"bronze_liner_{side}", tolerance=0.0008),
            origin=Origin(xyz=(inner_x - (0.032 if x > 0 else -0.032), 0.0, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"bronze_liner_{side}",
        )
        for j in range(6):
            angle = 2.0 * math.pi * j / 6.0
            yy = 0.177 * math.cos(angle)
            zz = 1.080 + 0.177 * math.sin(angle)
            yoke.visual(
                Cylinder(radius=0.014, length=0.015),
                origin=Origin(xyz=(inner_x - (0.055 if x > 0 else -0.055), yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"bearing_bolt_{side}_{j}",
            )

    tooling_plate = model.part("tooling_plate")
    tooling_plate.visual(
        Cylinder(radius=0.072, length=1.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="trunnion_shaft",
    )
    tooling_plate.visual(
        Cylinder(radius=0.142, length=0.044),
        origin=Origin(xyz=(-0.460, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="bearing_shoulder_0",
    )
    tooling_plate.visual(
        Cylinder(radius=0.142, length=0.044),
        origin=Origin(xyz=(0.470, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="bearing_shoulder_1",
    )
    tooling_plate.visual(
        Box((0.165, 0.910, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=plate_mat,
        name="tooling_panel",
    )
    # Raised perimeter and machined T-slot lands on both broad faces.
    tooling_plate.visual(
        Box((0.190, 0.995, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=machined,
        name="top_edge_rail",
    )
    tooling_plate.visual(
        Box((0.190, 0.995, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
        material=machined,
        name="bottom_edge_rail",
    )
    for y in (-0.485, 0.485):
        tooling_plate.visual(
            Box((0.190, 0.060, 0.690)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=machined,
            name=f"side_edge_rail_{0 if y < 0 else 1}",
        )
    for face_x, suffix in ((-0.086, "front"), (0.086, "rear")):
        for y in (-0.250, 0.0, 0.250):
            tooling_plate.visual(
                Box((0.010, 0.032, 0.515)),
                origin=Origin(xyz=(face_x, y, 0.0)),
                material=black,
                name=f"{suffix}_t_slot_{int((y + 0.25) / 0.25)}",
            )
        for j, (yy, zz) in enumerate(((-0.340, -0.205), (-0.115, -0.205), (0.115, -0.205), (0.340, -0.205),
                                      (-0.340, 0.205), (-0.115, 0.205), (0.115, 0.205), (0.340, 0.205))):
            tooling_plate.visual(
                Cylinder(radius=0.020, length=0.012),
                origin=Origin(xyz=(face_x, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"{suffix}_threaded_hole_{j}",
            )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.497)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_to_plate",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=tooling_plate,
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.45, lower=-1.20, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    plate = object_model.get_part("tooling_plate")
    azimuth = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_plate")

    ctx.allow_overlap(
        plate,
        yoke,
        elem_a="bearing_shoulder_0",
        elem_b="bearing_ring_0",
        reason="The tooling-plate trunnion shoulder is seated a millimeter into the bearing face to show preload/capture.",
    )
    ctx.allow_overlap(
        plate,
        yoke,
        elem_a="bearing_shoulder_1",
        elem_b="bearing_ring_1",
        reason="The opposing trunnion shoulder is seated a millimeter into the bearing face to show preload/capture.",
    )

    ctx.expect_gap(
        yoke,
        base,
        axis="z",
        positive_elem="rotary_table",
        negative_elem="vertical_bearing_face",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotary table sits on the vertical bearing face",
    )
    ctx.expect_overlap(
        plate,
        yoke,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        min_overlap=0.040,
        name="shaft is retained in one trunnion bearing",
    )
    ctx.expect_overlap(
        plate,
        yoke,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        min_overlap=0.040,
        name="shaft is retained in the opposing trunnion bearing",
    )
    ctx.expect_gap(
        plate,
        yoke,
        axis="x",
        positive_elem="bearing_shoulder_0",
        negative_elem="bearing_ring_0",
        max_penetration=0.003,
        name="one trunnion shoulder seats into the bearing face",
    )
    ctx.expect_gap(
        yoke,
        plate,
        axis="x",
        positive_elem="bearing_ring_1",
        negative_elem="bearing_shoulder_1",
        max_penetration=0.003,
        name="opposing trunnion shoulder seats into the bearing face",
    )

    for q in (-1.20, 1.20):
        with ctx.pose({tilt: q}):
            ctx.expect_gap(
                plate,
                yoke,
                axis="z",
                positive_elem="tooling_panel",
                negative_elem="saddle_deck",
                min_gap=0.150,
                name=f"tilted tooling panel clears the saddle deck at {q:+.2f} rad",
            )
            ctx.expect_gap(
                yoke,
                plate,
                axis="x",
                positive_elem="tower_1",
                negative_elem="tooling_panel",
                min_gap=0.300,
                name=f"tooling panel clears one tower at {q:+.2f} rad",
            )
            ctx.expect_gap(
                plate,
                yoke,
                axis="x",
                positive_elem="tooling_panel",
                negative_elem="tower_0",
                min_gap=0.300,
                name=f"tooling panel clears the opposing tower at {q:+.2f} rad",
            )

    with ctx.pose({azimuth: math.pi / 2.0, tilt: 0.70}):
        ctx.expect_gap(
            plate,
            yoke,
            axis="z",
            positive_elem="tooling_panel",
            negative_elem="saddle_deck",
            min_gap=0.200,
            name="combined rotary pose keeps plate above base rails",
        )

    return ctx.report()


object_model = build_object_model()
