from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SHAFT_Y = (-0.25, -0.066, 0.208)
SHAFT_Z = 0.255
SHAFT_RADIUS = 0.018
SHAFT_LENGTH = 0.86


def _housing_shape() -> cq.Workplane:
    """Open cast base with two drilled bearing plates and no lid."""

    base_length = 0.92
    base_width = 0.86
    base_thickness = 0.055
    plate_x = 0.36
    plate_thickness = 0.060
    plate_height = 0.335

    housing = (
        cq.Workplane("XY")
        .box(base_length, base_width, base_thickness)
        .translate((0.0, 0.0, base_thickness / 2.0))
    )

    # Low side rails make the base read as an open gearbox sump while leaving
    # the gear train fully exposed from above.
    rail_height = 0.085
    rail_thickness = 0.040
    for y in (-base_width / 2.0 + rail_thickness / 2.0, base_width / 2.0 - rail_thickness / 2.0):
        rail = (
            cq.Workplane("XY")
            .box(base_length, rail_thickness, rail_height)
            .translate((0.0, y, base_thickness + rail_height / 2.0))
        )
        housing = housing.union(rail)

    # End bearing plates support all parallel shafts.  Their broad faces are
    # drilled later, so the shafts pass through captured bearing bores.
    for x in (-plate_x, plate_x):
        plate = (
            cq.Workplane("XY")
            .box(plate_thickness, base_width * 0.88, plate_height)
            .translate((x, 0.0, base_thickness + plate_height / 2.0))
        )
        housing = housing.union(plate)

        # Rounded exterior bearing bosses around each hole.
        for y in SHAFT_Y:
            boss = (
                cq.Workplane("YZ")
                .center(y, SHAFT_Z)
                .circle(0.055)
                .extrude(0.052, both=True)
                .translate((x, 0.0, 0.0))
            )
            housing = housing.union(boss)

    # Cross ribs and corner pads give the grounded base enough visual mass.
    for x in (-0.24, 0.24):
        rib = (
            cq.Workplane("XY")
            .box(0.035, base_width * 0.74, 0.050)
            .translate((x, 0.0, base_thickness + 0.025))
        )
        housing = housing.union(rib)

    for x in (-0.40, 0.40):
        for y in (-0.34, 0.34):
            pad = (
                cq.Workplane("XY")
                .box(0.105, 0.080, 0.018)
                .translate((x, y, base_thickness + 0.009))
            )
            bolt = (
                cq.Workplane("XY")
                .circle(0.018)
                .extrude(0.014)
                .translate((x, y, base_thickness + 0.018))
            )
            housing = housing.union(pad).union(bolt)

    # One long cutter per shaft axis opens both bearing plates and every boss.
    for y in SHAFT_Y:
        # The bore is a hair smaller than the visual shaft so the modeled
        # bearing reads as a captured, pressed/oiled contact rather than as a
        # floating axle inside an oversized hole.
        cutter = cq.Workplane("YZ").center(y, SHAFT_Z).circle(SHAFT_RADIUS - 0.0005).extrude(1.25, both=True)
        housing = housing.cut(cutter)

    return housing


def _gear_mesh(teeth: int, width: float, name: str):
    module = 0.005
    pitch_radius = module * teeth / 2.0
    outer_radius = pitch_radius + module
    root_radius = max(outer_radius - 0.012, pitch_radius * 0.82)
    tooth_pitch = 2.0 * math.pi / teeth
    profile = []
    for tooth in range(teeth):
        start = tooth * tooth_pitch
        for frac, radius in (
            (0.00, root_radius),
            (0.18, outer_radius),
            (0.50, outer_radius),
            (0.68, root_radius),
        ):
            angle = start + frac * tooth_pitch
            profile.append((radius * math.cos(angle), radius * math.sin(angle)))
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, width), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_reduction_gearbox")

    cast_iron = Material("blue_gray_cast_iron", rgba=(0.20, 0.24, 0.27, 1.0))
    dark_steel = Material("dark_oiled_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    gear_steel = Material("brushed_gear_steel", rgba=(0.70, 0.66, 0.55, 1.0))
    end_mark = Material("polished_shaft_ends", rgba=(0.78, 0.78, 0.72, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "open_gearbox_housing", tolerance=0.0015),
        material=cast_iron,
        name="housing_shell",
    )

    # Progressively larger exposed gears make the reduction stages legible.  The
    # actual train is left uncovered, with each gear rigidly mounted to its own
    # independently revolute shaft.
    shaft_specs = (
        (0, SHAFT_Y[0], 24, 0.054, 0.064, 0.078),
        (1, SHAFT_Y[1], 44, 0.060, 0.078, 0.090),
        (2, SHAFT_Y[2], 60, 0.066, 0.090, 0.102),
    )

    for idx, y, teeth, gear_width, hub_d, hub_length in shaft_specs:
        shaft = model.part(f"shaft_{idx}")
        shaft.visual(
            Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="shaft_barrel",
        )
        shaft.visual(
            _gear_mesh(teeth, gear_width, f"stage_{idx}_spur_gear"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gear_steel,
            name="gear_wheel",
        )
        shaft.visual(
            Cylinder(radius=hub_d / 2.0, length=hub_length),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gear_steel,
            name="gear_hub",
        )
        # Short polished collars protrude beyond the bearing plates so the
        # rotating shaft remains visibly continuous through the housing.
        for x, name in ((-0.438, "end_collar_0"), (0.438, "end_collar_1")):
            shaft.visual(
                Cylinder(radius=0.023, length=0.026),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=end_mark,
                name=name,
            )

        model.articulation(
            f"housing_to_shaft_{idx}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=shaft,
            origin=Origin(xyz=(0.0, y, SHAFT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=12.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(f"housing_to_shaft_{idx}") for idx in range(3)]
    shafts = [object_model.get_part(f"shaft_{idx}") for idx in range(3)]
    housing = object_model.get_part("housing")

    ctx.check(
        "three exposed revolute shaft joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=", ".join(f"{j.name}:{j.articulation_type}" for j in joints),
    )
    ctx.check(
        "shaft axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (1.0, 0.0, 0.0) for j in joints),
        details=", ".join(f"{j.name}:{j.axis}" for j in joints),
    )
    ctx.check(
        "one gear on every shaft",
        all(any(v.name == "gear_wheel" for v in shaft.visuals) for shaft in shafts),
        details="Each shaft part must carry a visible gear wheel.",
    )

    for idx, shaft in enumerate(shafts):
        ctx.allow_overlap(
            housing,
            shaft,
            elem_a="housing_shell",
            elem_b="shaft_barrel",
            reason="The shaft barrel is intentionally captured with a very small bearing-bore press fit in the exposed end plates.",
        )
        ctx.expect_overlap(
            shaft,
            housing,
            axes="x",
            elem_a="shaft_barrel",
            elem_b="housing_shell",
            min_overlap=0.60,
            name=f"shaft_{idx} spans both bearing plates",
        )

    ctx.expect_gap(
        shafts[1],
        shafts[0],
        axis="y",
        positive_elem="gear_wheel",
        negative_elem="gear_wheel",
        min_gap=0.001,
        max_gap=0.018,
        name="first gear pair has running clearance",
    )
    ctx.expect_gap(
        shafts[2],
        shafts[1],
        axis="y",
        positive_elem="gear_wheel",
        negative_elem="gear_wheel",
        min_gap=0.001,
        max_gap=0.018,
        name="second gear pair has running clearance",
    )

    return ctx.report()


object_model = build_object_model()
