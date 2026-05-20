from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    Worm,
    mesh_from_cadquery,
)


BASE_Z = 0.035
WORM_Y = -0.225
WORM_Z = 0.270
WHEEL_Z = 0.270
UPPER_Z = 0.420
OUTPUT_X = 0.128


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(height * 1.2).translate((0.0, 0.0, -height * 0.1))
    return outer.cut(bore)


def _add_annular_bearing(
    frame,
    *,
    name: str,
    center: tuple[float, float],
    bottom_z: float,
    outer_radius: float,
    inner_radius: float,
    height: float,
    material: str,
) -> None:
    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(outer_radius, inner_radius, height),
            name,
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(center[0], center[1], bottom_z)),
        material=material,
        name=name,
    )


def _add_split_pillow(frame, *, name: str, x: float) -> None:
    """A built-up split pillow block that leaves a clear window for the shaft."""
    frame.visual(
        Box((0.092, 0.145, 0.018)),
        origin=Origin(xyz=(x, WORM_Y, BASE_Z + 0.008)),
        material="frame_paint",
        name=f"{name}_foot",
    )
    frame.visual(
        Box((0.058, 0.112, 0.150)),
        origin=Origin(xyz=(x, WORM_Y, BASE_Z + 0.018 + 0.075)),
        material="frame_paint",
        name=f"{name}_saddle",
    )
    for side, y_offset in (("near", -0.038), ("far", 0.038)):
        frame.visual(
            Box((0.058, 0.020, 0.138)),
            origin=Origin(xyz=(x, WORM_Y + y_offset, WORM_Z + 0.002)),
            material="frame_paint",
            name=f"{name}_{side}_cheek",
        )
    frame.visual(
        Box((0.068, 0.112, 0.034)),
        origin=Origin(xyz=(x, WORM_Y, WORM_Z + 0.078)),
        material="frame_paint",
        name=f"{name}_cap",
    )
    for side, y_offset in (("near", -0.037), ("far", 0.037)):
        frame.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(xyz=(x, WORM_Y + y_offset, WORM_Z + 0.101)),
            material="dark_bolt",
            name=f"{name}_{side}_cap_screw",
        )


def _add_worm_bushing(frame, *, name: str, x: float) -> None:
    length = 0.046
    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.028, 0.014, length),
            name,
            tolerance=0.00045,
            angular_tolerance=0.07,
        ),
        origin=Origin(xyz=(x - length / 2.0, WORM_Y, WORM_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_bronze",
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="worm_indexing_unit")

    model.material("frame_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("dark_bolt", rgba=(0.045, 0.047, 0.050, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    model.material("polished_steel", rgba=(0.84, 0.84, 0.80, 1.0))
    model.material("bearing_bronze", rgba=(0.78, 0.55, 0.23, 1.0))
    model.material("oiled_bronze", rgba=(0.63, 0.42, 0.16, 1.0))
    model.material("index_blue", rgba=(0.11, 0.17, 0.28, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.700, 0.560, BASE_Z)),
        origin=Origin(xyz=(0.0, -0.035, BASE_Z / 2.0)),
        material="frame_paint",
        name="ground_plate",
    )
    frame.visual(
        Box((0.660, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, WORM_Y, BASE_Z + 0.010)),
        material="dark_bolt",
        name="worm_rail",
    )
    frame.visual(
        Box((0.400, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.155, BASE_Z + 0.010)),
        material="dark_bolt",
        name="rear_rail",
    )
    frame.visual(
        Box((0.050, 0.410, 0.020)),
        origin=Origin(xyz=(-0.315, -0.045, BASE_Z + 0.010)),
        material="dark_bolt",
        name="side_rail_0",
    )
    frame.visual(
        Box((0.050, 0.410, 0.020)),
        origin=Origin(xyz=(0.315, -0.045, BASE_Z + 0.010)),
        material="dark_bolt",
        name="side_rail_1",
    )

    _add_split_pillow(frame, name="worm_block_0", x=-0.245)
    _add_split_pillow(frame, name="worm_block_1", x=0.245)
    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.028, 0.014, 0.046),
            "worm_bushing_0",
            tolerance=0.00045,
            angular_tolerance=0.07,
        ),
        origin=Origin(xyz=(-0.245 - 0.023, WORM_Y, WORM_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_bronze",
        name="worm_bushing_0",
    )
    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.028, 0.014, 0.046),
            "worm_bushing_1",
            tolerance=0.00045,
            angular_tolerance=0.07,
        ),
        origin=Origin(xyz=(0.245 - 0.023, WORM_Y, WORM_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_bronze",
        name="worm_bushing_1",
    )

    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.064, 0.014, 0.072),
            "wheel_lower_bearing",
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z - 0.001)),
        material="bearing_bronze",
        name="wheel_lower_bearing",
    )
    for i, (x, y) in enumerate(((-0.058, -0.058), (0.058, -0.058), (-0.058, 0.058), (0.058, 0.058))):
        frame.visual(
            Box((0.050, 0.050, 0.018)),
            origin=Origin(xyz=(x, y, BASE_Z + 0.006)),
            material="frame_paint",
            name=f"wheel_bearing_pad_{i}",
        )

    # Open rear uprights tie the shaft supports together without hiding the gear train.
    for side, x in (("left", -0.235), ("right", 0.235)):
        frame.visual(
            Box((0.042, 0.052, 0.355)),
            origin=Origin(xyz=(x, 0.168, BASE_Z + 0.175)),
            material="frame_paint",
            name=f"rear_{side}_upright",
        )
    frame.visual(
        Box((0.520, 0.046, 0.038)),
        origin=Origin(xyz=(0.0, 0.168, 0.480)),
        material="frame_paint",
        name="rear_crossbar",
    )
    frame.visual(
        Box((0.018, 0.070, 0.070)),
        origin=Origin(xyz=(-0.038, 0.166, 0.392)),
        material="dark_bolt",
        name="index_pointer_body",
    )
    frame.visual(
        Box((0.055, 0.010, 0.012)),
        origin=Origin(xyz=(-0.022, 0.128, 0.392), rpy=(0.0, 0.0, -0.35)),
        material="polished_steel",
        name="index_pointer_tip",
    )

    # A standoff and two annular bearings carry the independent upper output shaft.
    frame.visual(
        Box((0.058, 0.054, 0.520)),
        origin=Origin(xyz=(OUTPUT_X, 0.166, BASE_Z + 0.260)),
        material="frame_paint",
        name="output_standoff",
    )
    frame.visual(
        Box((0.052, 0.142, 0.030)),
        origin=Origin(xyz=(OUTPUT_X, 0.099, 0.343 + 0.017)),
        material="frame_paint",
        name="output_lower_bearing_arm",
    )
    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.034, 0.011, 0.036),
            "output_lower_bearing",
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(OUTPUT_X, 0.0, 0.343)),
        material="bearing_bronze",
        name="output_lower_bearing",
    )
    frame.visual(
        Box((0.052, 0.142, 0.030)),
        origin=Origin(xyz=(OUTPUT_X, 0.099, 0.520 + 0.017)),
        material="frame_paint",
        name="output_upper_bearing_arm",
    )
    frame.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.034, 0.011, 0.036),
            "output_upper_bearing",
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(OUTPUT_X, 0.0, 0.520)),
        material="bearing_bronze",
        name="output_upper_bearing",
    )

    frame.visual(
        Box((0.014, 0.014, 0.050)),
        origin=Origin(xyz=(-0.038, 0.166, 0.438)),
        material="dark_bolt",
        name="index_pointer_stem",
    )

    # Exposed mounting bolts keep the bench-frame appearance deliberate.
    for i, (x, y) in enumerate(
        [
            (-0.315, -0.285),
            (0.315, -0.285),
            (-0.315, 0.215),
            (0.315, 0.215),
            (-0.075, 0.070),
            (0.075, 0.070),
        ]
    ):
        frame.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x, y, BASE_Z + 0.004)),
            material="dark_bolt",
            name=f"floor_bolt_{i}",
        )

    worm_shaft = model.part("worm_shaft")
    geom, x_origin = _cyl_x(0.014, 0.640)
    worm_shaft.visual(geom, origin=x_origin, material="machined_steel", name="shaft_barrel")
    worm_mesh = Worm(
        module=0.0075,
        lead_angle=13.0,
        n_threads=2,
        length=0.228,
        backlash=0.0008,
        clearance=0.0006,
    ).build()
    worm_shaft.visual(
        mesh_from_cadquery(worm_mesh, "worm_thread", tolerance=0.00045, angular_tolerance=0.06),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="polished_steel",
        name="worm_thread",
    )
    for i, x in enumerate((-0.302, -0.188, 0.188, 0.302)):
        geom, origin = _cyl_x(0.025, 0.024)
        worm_shaft.visual(
            geom,
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=origin.rpy),
            material="machined_steel",
            name=f"worm_retainer_{i}",
        )
    worm_shaft.visual(
        Box((0.118, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material="dark_bolt",
        name="worm_key",
    )

    wheel_spindle = model.part("wheel_spindle")
    wheel_spindle.visual(
        Cylinder(radius=0.014, length=0.510),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material="machined_steel",
        name="vertical_shaft",
    )
    wheel_mesh = SpurGear(
        module=0.0080,
        teeth_number=38,
        width=0.048,
        backlash=0.0010,
        clearance=0.0006,
    ).build(chamfer=0.0012)
    wheel_spindle.visual(
        mesh_from_cadquery(wheel_mesh, "worm_wheel", tolerance=0.00045, angular_tolerance=0.06),
        material="oiled_bronze",
        name="worm_wheel",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.030, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material="machined_steel",
        name="machined_spacer",
    )
    pinion_mesh = SpurGear(
        module=0.0050,
        teeth_number=22,
        width=0.032,
        backlash=0.0008,
        clearance=0.0005,
    ).build()
    wheel_spindle.visual(
        mesh_from_cadquery(pinion_mesh, "upper_pinion", tolerance=0.0004, angular_tolerance=0.06),
        origin=Origin(xyz=(0.0, 0.0, UPPER_Z - WHEEL_Z)),
        material="bearing_bronze",
        name="upper_pinion",
    )
    wheel_spindle.visual(
        Box((0.010, 0.006, 0.070)),
        origin=Origin(xyz=(0.033, 0.0, 0.093)),
        material="dark_bolt",
        name="spindle_key",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.236)),
        material="polished_steel",
        name="top_retainer_cap",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=0.011, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material="machined_steel",
        name="output_barrel",
    )
    output_gear_mesh = SpurGear(
        module=0.0050,
        teeth_number=18,
        width=0.030,
        backlash=0.0008,
        clearance=0.0005,
    ).build()
    output_shaft.visual(
        mesh_from_cadquery(output_gear_mesh, "secondary_gear", tolerance=0.0004, angular_tolerance=0.06),
        material="polished_steel",
        name="secondary_gear",
    )
    output_shaft.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material="machined_steel",
        name="lower_retainer_cap",
    )
    output_shaft.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material="machined_steel",
        name="upper_retainer_cap",
    )
    output_shaft.visual(
        Box((0.008, 0.005, 0.048)),
        origin=Origin(xyz=(0.023, 0.0, 0.000)),
        material="dark_bolt",
        name="output_key",
    )

    model.articulation(
        "worm_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=worm_shaft,
        origin=Origin(xyz=(0.0, WORM_Y, WORM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_spindle,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )
    model.articulation(
        "output_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_X, 0.0, UPPER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    worm = object_model.get_part("worm_shaft")
    wheel = object_model.get_part("wheel_spindle")
    output = object_model.get_part("output_shaft")

    worm_spin = object_model.get_articulation("worm_spin")
    wheel_spin = object_model.get_articulation("wheel_spin")
    output_spin = object_model.get_articulation("output_spin")

    ctx.check(
        "three supported rotary axes",
        len(object_model.articulations) == 3
        and tuple(worm_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(wheel_spin.axis) == (0.0, 0.0, 1.0)
        and tuple(output_spin.axis) == (0.0, 0.0, 1.0),
        details="Expected worm shaft about X and the two vertical shafts about Z.",
    )

    for elem in ("worm_bushing_0", "worm_bushing_1"):
        ctx.allow_overlap(
            frame,
            worm,
            elem_a=elem,
            elem_b="shaft_barrel",
            reason="The worm shaft is intentionally captured inside the bronze pillow-block bushing proxy.",
        )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="wheel_lower_bearing",
        elem_b="vertical_shaft",
        reason="The wheel spindle passes through its lower bearing sleeve; the sleeve is modeled as a retained proxy fit.",
    )
    for elem in ("output_lower_bearing", "output_upper_bearing"):
        ctx.allow_overlap(
            frame,
            output,
            elem_a=elem,
            elem_b="output_barrel",
            reason="The output shaft is intentionally captured through the upper support's bronze bearing proxy.",
        )

    ctx.expect_gap(
        wheel,
        worm,
        axis="y",
        min_gap=0.003,
        max_gap=0.040,
        positive_elem="worm_wheel",
        negative_elem="worm_thread",
        name="worm and wheel have visible running clearance",
    )
    for elem in ("worm_bushing_0", "worm_bushing_1"):
        ctx.expect_within(
            worm,
            frame,
            axes="yz",
            margin=0.006,
            inner_elem="shaft_barrel",
            outer_elem=elem,
            name=f"worm shaft is centered in {elem}",
        )
        ctx.expect_overlap(
            worm,
            frame,
            axes="x",
            min_overlap=0.030,
            elem_a="shaft_barrel",
            elem_b=elem,
            name=f"worm shaft remains inserted through {elem}",
        )
    ctx.expect_gap(
        output,
        wheel,
        axis="x",
        min_gap=0.002,
        max_gap=0.020,
        positive_elem="secondary_gear",
        negative_elem="upper_pinion",
        name="upper spur gears are close but not clipping",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xy",
        margin=0.020,
        inner_elem="vertical_shaft",
        outer_elem="wheel_lower_bearing",
        name="wheel spindle runs inside its lower bearing bore",
    )
    ctx.expect_within(
        output,
        frame,
        axes="xy",
        margin=0.018,
        inner_elem="output_barrel",
        outer_elem="output_upper_bearing",
        name="output shaft runs inside the upper bearing bore",
    )
    ctx.expect_within(
        output,
        frame,
        axes="xy",
        margin=0.011,
        inner_elem="output_barrel",
        outer_elem="output_lower_bearing",
        name="output shaft runs inside the lower bearing bore",
    )

    with ctx.pose({worm_spin: 1.0, wheel_spin: 0.65, output_spin: -0.80}):
        ctx.expect_contact(
            wheel,
            worm,
            contact_tol=0.002,
            elem_a="worm_wheel",
            elem_b="worm_thread",
            name="worm clearance remains in a rotated pose",
        )
        ctx.expect_gap(
            output,
            wheel,
            axis="x",
            min_gap=0.001,
            max_gap=0.024,
            positive_elem="secondary_gear",
            negative_elem="upper_pinion",
            name="upper spur clearance remains in a rotated pose",
        )

    return ctx.report()


object_model = build_object_model()
