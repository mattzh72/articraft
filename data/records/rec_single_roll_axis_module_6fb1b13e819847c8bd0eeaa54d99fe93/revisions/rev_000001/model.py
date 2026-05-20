from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_roll_axis_spindle")

    cast_iron = model.material("dark_cast_iron", rgba=(0.17, 0.20, 0.22, 1.0))
    blue_gray = model.material("blue_gray_machine_paint", rgba=(0.20, 0.27, 0.32, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.69, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    bronze = model.material("oil_bronze_bushing", rgba=(0.72, 0.48, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    warning_red = model.material("red_index_mark", rgba=(0.80, 0.08, 0.05, 1.0))

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.52, 0.34, 0.045, corner_segments=10),
            0.055,
        ),
        "rounded_pedestal_foot",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.36, 0.18, 0.19),
            span_width=0.22,
            trunnion_diameter=0.074,
            trunnion_center_z=0.115,
            base_thickness=0.040,
            corner_radius=0.010,
            center=False,
        ),
        "roll_axis_bearing_yoke",
    )
    bushing_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.038, tube=0.0125, radial_segments=18, tubular_segments=56),
        "bronze_bearing_ring",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(foot_mesh, material=cast_iron, name="rounded_foot")
    for index, (x, y) in enumerate(
        ((-0.205, -0.125), (-0.205, 0.125), (0.205, -0.125), (0.205, 0.125))
    ):
        pedestal.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber,
            name=f"rubber_pad_{index}",
        )
        pedestal.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, y, 0.057)),
            material=dark_steel,
            name=f"foot_bolt_{index}",
        )

    pedestal.visual(
        Cylinder(radius=0.055, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=blue_gray,
        name="short_column",
    )
    pedestal.visual(
        Cylinder(radius=0.090, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=cast_iron,
        name="column_foot_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=cast_iron,
        name="column_head_collar",
    )
    pedestal.visual(
        Box((0.250, 0.205, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=cast_iron,
        name="top_saddle_plate",
    )
    pedestal.visual(
        Box((0.205, 0.020, 0.125)),
        origin=Origin(xyz=(0.0, 0.064, 0.135)),
        material=blue_gray,
        name="front_column_web",
    )
    pedestal.visual(
        Box((0.205, 0.020, 0.125)),
        origin=Origin(xyz=(0.0, -0.064, 0.135)),
        material=blue_gray,
        name="rear_column_web",
    )
    pedestal.visual(
        Box((0.020, 0.205, 0.125)),
        origin=Origin(xyz=(0.064, 0.0, 0.135)),
        material=blue_gray,
        name="side_column_web_0",
    )
    pedestal.visual(
        Box((0.020, 0.205, 0.125)),
        origin=Origin(xyz=(-0.064, 0.0, 0.135)),
        material=blue_gray,
        name="side_column_web_1",
    )

    yoke_base_z = 0.413
    shaft_center_z = yoke_base_z + 0.115
    pedestal.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, yoke_base_z)),
        material=blue_gray,
        name="bearing_yoke",
    )
    for side_index, x in enumerate((-0.183, 0.183)):
        pedestal.visual(
            bushing_ring_mesh,
            origin=Origin(xyz=(x, 0.0, shaft_center_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name=f"bearing_bushing_{side_index}",
        )
        for bolt_index, (dy, dz) in enumerate(
            ((-0.050, -0.044), (-0.050, 0.044), (0.050, -0.044), (0.050, 0.044))
        ):
            pedestal.visual(
                Cylinder(radius=0.0075, length=0.006),
                origin=Origin(
                    xyz=(x, dy, shaft_center_z + dz),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=dark_steel,
                name=f"bearing_cap_bolt_{side_index}_{bolt_index}",
            )

    spindle = model.part("spindle_head")
    spindle.visual(
        Cylinder(radius=0.026, length=0.420),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.066, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="flange_disk",
    )
    spindle.visual(
        Cylinder(radius=0.041, length=0.080),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="central_hub",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_register",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        spindle.visual(
            Cylinder(radius=0.0075, length=0.007),
            origin=Origin(
                xyz=(0.0215, 0.044 * math.cos(angle), 0.044 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"flange_bolt_{index}",
        )
    spindle.visual(
        Box((0.006, 0.016, 0.006)),
        origin=Origin(xyz=(0.0195, 0.0, 0.056)),
        material=warning_red,
        name="index_mark",
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, shaft_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=6.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    spindle = object_model.get_part("spindle_head")
    roll = object_model.get_articulation("roll_axis")

    for bushing_name in ("bearing_bushing_0", "bearing_bushing_1"):
        ctx.allow_overlap(
            pedestal,
            spindle,
            elem_a=bushing_name,
            elem_b="shaft",
            reason=(
                "The rotating shaft is intentionally captured in the oil-bronze "
                "bearing ring with a tiny modeled interference so the spindle reads "
                "as physically carried rather than floating."
            ),
        )
        ctx.expect_within(
            spindle,
            pedestal,
            axes="yz",
            inner_elem="shaft",
            outer_elem=bushing_name,
            margin=0.002,
            name=f"shaft centered in {bushing_name}",
        )
        ctx.expect_overlap(
            spindle,
            pedestal,
            axes="x",
            elem_a="shaft",
            elem_b=bushing_name,
            min_overlap=0.010,
            name=f"shaft retained by {bushing_name}",
        )

    ctx.check(
        "single roll-axis revolute joint",
        len(object_model.articulations) == 1
        and roll.articulation_type == ArticulationType.REVOLUTE
        and tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}, axis={roll.axis}",
    )
    ctx.expect_within(
        spindle,
        pedestal,
        axes="yz",
        inner_elem="flange_disk",
        outer_elem="bearing_yoke",
        margin=0.004,
        name="flange fits inside yoke window",
    )
    ctx.expect_overlap(
        spindle,
        pedestal,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_yoke",
        min_overlap=0.30,
        name="shaft spans both bearing blocks",
    )

    rest_aabb = ctx.part_element_world_aabb(spindle, elem="index_mark")
    with ctx.pose({roll: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(spindle, elem="index_mark")
        ctx.expect_within(
            spindle,
            pedestal,
            axes="yz",
            inner_elem="flange_disk",
            outer_elem="bearing_yoke",
            margin=0.004,
            name="rolled flange remains between bearing cheeks",
        )
    ctx.check(
        "index mark visibly rolls with spindle",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(rest_aabb[0][2] - turned_aabb[0][2]) > 0.020,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
