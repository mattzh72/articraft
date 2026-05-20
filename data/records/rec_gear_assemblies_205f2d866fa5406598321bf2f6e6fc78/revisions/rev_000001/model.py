from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    Worm,
    mesh_from_cadquery,
)


WORM_Y = -0.095
WORM_Z = 0.180
WHEEL_Z = 0.180
UPPER_GEAR_Z = 0.285
OUTPUT_X = 0.095


def _centered_workplane(shape: object) -> cq.Workplane:
    """Return a CadQuery workplane translated so its local AABB center is zero."""
    wp = shape if isinstance(shape, cq.Workplane) else cq.Workplane("XY").add(shape)
    bb = wp.val().BoundingBox()
    cx = (bb.xmin + bb.xmax) / 2.0
    cy = (bb.ymin + bb.ymax) / 2.0
    cz = (bb.zmin + bb.zmax) / 2.0
    return wp.translate((-cx, -cy, -cz))


def _ring_shape(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A vertical bearing collar with an actual clearance hole."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _spur_gear_shape(
    *,
    module_mm: float,
    teeth: int,
    width_mm: float,
    bore_mm: float,
    hub_mm: float | None = None,
    hub_length_mm: float | None = None,
    spokes: int | None = None,
) -> cq.Workplane:
    build_args = {"bore_d": bore_mm, "chamfer": 0.6}
    if hub_mm is not None:
        build_args["hub_d"] = hub_mm
    if hub_length_mm is not None:
        build_args["hub_length"] = hub_length_mm
    if spokes is not None:
        build_args["n_spokes"] = spokes
        build_args["spoke_width"] = 4.0
        build_args["spokes_id"] = max(bore_mm + 8.0, module_mm * 5.0)
        build_args["spokes_od"] = module_mm * teeth * 0.72
    gear = SpurGear(module_mm, teeth, width_mm).build(**build_args)
    return _centered_workplane(gear)


def _worm_shape() -> cq.Workplane:
    worm = Worm(module=4.0, lead_angle=20.0, n_threads=1, length=118.0).build(
        bore_d=10.0
    )
    return _centered_workplane(worm)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="worm_indexing_unit")

    model.material("cast_iron", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("bronze", rgba=(0.78, 0.50, 0.20, 1.0))
    model.material("brass", rgba=(0.90, 0.68, 0.30, 1.0))
    model.material("blackened", rgba=(0.03, 0.035, 0.04, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.42, 0.30, 0.026)),
        origin=Origin(xyz=(0.04, 0.00, 0.013)),
        material="cast_iron",
        name="base_plate",
    )

    # Open U-yokes at each worm bearing leave the rotating shaft visible and
    # clear, rather than burying it inside a solid proxy.
    for idx, x in enumerate((-0.145, 0.145)):
        frame.visual(
            Box((0.046, 0.014, 0.186)),
            origin=Origin(xyz=(x, WORM_Y - 0.027, 0.118)),
            material="cast_iron",
            name=f"worm_yoke_{idx}_front",
        )
        frame.visual(
            Box((0.046, 0.014, 0.186)),
            origin=Origin(xyz=(x, WORM_Y + 0.027, 0.118)),
            material="cast_iron",
            name=f"worm_yoke_{idx}_rear",
        )
        frame.visual(
            Box((0.050, 0.080, 0.018)),
            origin=Origin(xyz=(x, WORM_Y, 0.216)),
            material="cast_iron",
            name=f"worm_yoke_{idx}_cap",
        )
        frame.visual(
            Box((0.050, 0.080, 0.050)),
            origin=Origin(xyz=(x, WORM_Y, 0.050)),
            material="cast_iron",
            name=f"worm_yoke_{idx}_foot",
        )

    for name, x in (("wheel_lower_bearing", 0.0), ("output_lower_bearing", OUTPUT_X)):
        frame.visual(
            mesh_from_cadquery(_ring_shape(0.030, 0.0072, 0.046), name),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material="cast_iron",
            name=name,
        )

    # Rear gantry and cantilevered upper bearing collars expose the gearing from
    # the front while still showing how both vertical shafts are supported.
    for idx, x in enumerate((-0.065, 0.145)):
        frame.visual(
            Box((0.026, 0.026, 0.305)),
            origin=Origin(xyz=(x, 0.115, 0.176)),
            material="cast_iron",
            name=f"rear_post_{idx}",
        )
    frame.visual(
        Box((0.240, 0.030, 0.024)),
        origin=Origin(xyz=(0.040, 0.115, 0.330)),
        material="cast_iron",
        name="upper_rear_beam",
    )
    for name, x in (("wheel_upper_bearing", 0.0), ("output_upper_bearing", OUTPUT_X)):
        frame.visual(
            Box((0.020, 0.092, 0.018)),
            origin=Origin(xyz=(x, 0.070, 0.333)),
            material="cast_iron",
            name=f"{name}_arm",
        )
        frame.visual(
            mesh_from_cadquery(_ring_shape(0.027, 0.0072, 0.026), name),
            origin=Origin(xyz=(x, 0.0, 0.318)),
            material="cast_iron",
            name=name,
        )
    for idx, x in enumerate((-0.145, 0.145)):
        frame.visual(
            mesh_from_cadquery(_ring_shape(0.0205, 0.0062, 0.016), f"worm_bearing_{idx}"),
            origin=Origin(xyz=(x, WORM_Y, WORM_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_steel",
            name=f"worm_bearing_{idx}",
        )

    worm_shaft = model.part("worm_shaft")
    worm_shaft.visual(
        Cylinder(radius=0.0065, length=0.345),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="shaft_core",
    )
    worm_shaft.visual(
        mesh_from_cadquery(_worm_shape(), "worm_thread", unit_scale=0.001),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="worm_thread",
    )
    for x, name in ((-0.177, "handwheel"), (0.174, "end_collar")):
        worm_shaft.visual(
            Cylinder(radius=0.030 if name == "handwheel" else 0.014, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="blackened" if name == "handwheel" else "dark_steel",
            name=name,
        )

    wheel_spindle = model.part("wheel_spindle")
    wheel_spindle.visual(
        Cylinder(radius=0.0075, length=0.318),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="machined_steel",
        name="vertical_shaft",
    )
    wheel_spindle.visual(
        mesh_from_cadquery(
            _spur_gear_shape(
                module_mm=4.0,
                teeth=34,
                width_mm=22.0,
                bore_mm=12.0,
                hub_mm=34.0,
                hub_length_mm=30.0,
                spokes=6,
            ),
            "worm_wheel",
            unit_scale=0.001,
        ),
        material="bronze",
        name="worm_wheel",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.025, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material="bronze",
        name="wheel_hub",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material="machined_steel",
        name="upper_spacer",
    )
    wheel_spindle.visual(
        mesh_from_cadquery(
            _spur_gear_shape(
                module_mm=3.0,
                teeth=16,
                width_mm=14.0,
                bore_mm=10.0,
                hub_mm=22.0,
                hub_length_mm=18.0,
            ),
            "upper_pinion",
            unit_scale=0.001,
        ),
        origin=Origin(xyz=(0.0, 0.0, UPPER_GEAR_Z - WHEEL_Z)),
        material="brass",
        name="upper_pinion",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=0.0075, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material="machined_steel",
        name="output_spindle",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            _spur_gear_shape(
                module_mm=3.0,
                teeth=36,
                width_mm=16.0,
                bore_mm=10.0,
                hub_mm=26.0,
                hub_length_mm=20.0,
                spokes=5,
            ),
            "output_gear",
            unit_scale=0.001,
        ),
        material="brass",
        name="output_gear",
    )
    output_shaft.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material="dark_steel",
        name="top_coupling",
    )

    model.articulation(
        "frame_to_worm_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=worm_shaft,
        origin=Origin(xyz=(0.0, WORM_Y, WORM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "frame_to_wheel_spindle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=wheel_spindle,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-math.pi, upper=math.pi),
        mimic=Mimic(joint="frame_to_worm_shaft", multiplier=-1.0 / 34.0),
    )
    model.articulation(
        "frame_to_output_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_X, 0.0, UPPER_GEAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-math.pi, upper=math.pi),
        mimic=Mimic(joint="frame_to_worm_shaft", multiplier=16.0 / (34.0 * 36.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    worm_shaft = object_model.get_part("worm_shaft")
    wheel_spindle = object_model.get_part("wheel_spindle")
    output_shaft = object_model.get_part("output_shaft")
    worm_joint = object_model.get_articulation("frame_to_worm_shaft")
    wheel_joint = object_model.get_articulation("frame_to_wheel_spindle")
    output_joint = object_model.get_articulation("frame_to_output_shaft")

    for idx in (0, 1):
        ctx.allow_overlap(
            "frame",
            worm_shaft,
            elem_a=f"worm_bearing_{idx}",
            elem_b="shaft_core",
            reason="The worm shaft journal is intentionally captured inside a bearing bushing.",
        )
        ctx.expect_overlap(
            "frame",
            worm_shaft,
            axes="x",
            elem_a=f"worm_bearing_{idx}",
            elem_b="shaft_core",
            min_overlap=0.012,
            name=f"worm journal {idx} remains inside its bushing",
        )
    for bearing, shaft_part, shaft_elem in (
        ("wheel_lower_bearing", wheel_spindle, "vertical_shaft"),
        ("wheel_upper_bearing", wheel_spindle, "vertical_shaft"),
        ("output_lower_bearing", output_shaft, "output_spindle"),
        ("output_upper_bearing", output_shaft, "output_spindle"),
    ):
        ctx.allow_overlap(
            "frame",
            shaft_part,
            elem_a=bearing,
            elem_b=shaft_elem,
            reason="The vertical spindle journal is intentionally captured in a bearing collar.",
        )
        ctx.expect_within(
            shaft_part,
            "frame",
            axes="xy",
            inner_elem=shaft_elem,
            outer_elem=bearing,
            margin=0.001,
            name=f"{bearing} radially contains its shaft",
        )
        ctx.expect_overlap(
            "frame",
            shaft_part,
            axes="z",
            elem_a=bearing,
            elem_b=shaft_elem,
            min_overlap=0.020,
            name=f"{bearing} surrounds the shaft height",
        )

    ctx.check(
        "three exposed rotary shafts",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (worm_joint, wheel_joint, output_joint)
        ),
        details="Worm shaft, wheel spindle, and secondary output shaft must each be revolute.",
    )
    ctx.check(
        "supported axis directions",
        worm_joint.axis == (1.0, 0.0, 0.0)
        and wheel_joint.axis == (0.0, 0.0, 1.0)
        and output_joint.axis == (0.0, 0.0, 1.0),
        details=f"axes: worm={worm_joint.axis}, wheel={wheel_joint.axis}, output={output_joint.axis}",
    )

    ctx.expect_gap(
        wheel_spindle,
        worm_shaft,
        axis="y",
        positive_elem="worm_wheel",
        negative_elem="worm_thread",
        max_gap=0.030,
        max_penetration=0.0,
        name="worm sits close to the wheel rim",
    )
    ctx.expect_gap(
        output_shaft,
        wheel_spindle,
        axis="x",
        positive_elem="output_gear",
        negative_elem="upper_pinion",
        max_gap=0.014,
        max_penetration=0.0,
        name="upper spur gears are closely meshed",
    )

    rest_worm = ctx.part_world_position(worm_shaft)
    rest_wheel = ctx.part_world_position(wheel_spindle)
    rest_output = ctx.part_world_position(output_shaft)
    with ctx.pose({worm_joint: 0.9}):
        moved_worm = ctx.part_world_position(worm_shaft)
        moved_wheel = ctx.part_world_position(wheel_spindle)
        moved_output = ctx.part_world_position(output_shaft)

    ctx.check(
        "gear joints rotate in place",
        rest_worm == moved_worm and rest_wheel == moved_wheel and rest_output == moved_output,
        details=(
            f"rest={(rest_worm, rest_wheel, rest_output)}, "
            f"posed={(moved_worm, moved_wheel, moved_output)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
