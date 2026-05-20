from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _build_bottle_shell():
    outer_profile = [
        (0.000, 0.000),
        (0.034, 0.000),
        (0.039, 0.004),
        (0.041, 0.100),
        (0.047, 0.138),
        (0.044, 0.156),
        (0.028, 0.178),
        (0.016, 0.190),
        (0.016, 0.206),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.031, 0.004),
        (0.0355, 0.008),
        (0.0375, 0.097),
        (0.0435, 0.135),
        (0.0405, 0.153),
        (0.0245, 0.175),
        (0.0125, 0.187),
        (0.0125, 0.206),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "soap_pump_bottle_shell",
    )


def _build_spout_tube():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.006, 0.000, 0.008),
                (0.014, 0.000, 0.013),
                (0.028, 0.000, 0.014),
                (0.040, 0.000, 0.010),
                (0.050, 0.000, 0.004),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "soap_pump_spout_tube",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.84, 0.86, 0.83, 0.96))
    pump_white = model.material("pump_white", rgba=(0.96, 0.96, 0.95, 1.0))
    pump_grey = model.material("pump_grey", rgba=(0.74, 0.75, 0.77, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _build_bottle_shell(),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.094, 0.094, 0.206)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0065, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=pump_grey,
        name="plunger_stem",
    )
    plunger.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=pump_white,
        name="plunger_sleeve",
    )
    plunger.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=pump_white,
        name="pump_flange",
    )
    plunger.visual(
        Cylinder(radius=0.013, length=0.036),
        origin=Origin(
            xyz=(0.001, 0.0, 0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=pump_white,
        name="actuator_cap",
    )
    plunger.visual(
        Box((0.022, 0.020, 0.010)),
        origin=Origin(xyz=(0.014, 0.0, 0.028)),
        material=pump_white,
        name="head_block",
    )
    plunger.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.010, 0.0, 0.035)),
        material=pump_grey,
        name="spout_seat",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.050, 0.030, 0.140)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=pump_white,
        name="pivot_collar",
    )
    spout.visual(
        Box((0.012, 0.016, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.009)),
        material=pump_white,
        name="spout_root",
    )
    spout.visual(
        _build_spout_tube(),
        material=pump_white,
        name="spout_tube",
    )
    spout.visual(
        Cylinder(radius=0.0052, length=0.008),
        origin=Origin(
            xyz=(0.052, 0.0, 0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pump_white,
        name="spout_tip",
    )
    spout.visual(
        Cylinder(radius=0.0018, length=0.0024),
        origin=Origin(
            xyz=(0.0572, 0.0, 0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=outlet_dark,
        name="nozzle_opening",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.062, 0.020, 0.022)),
        mass=0.03,
        origin=Origin(xyz=(0.027, 0.0, 0.008)),
    )

    model.articulation(
        "bottle_to_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle_body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.206)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=0.018,
        ),
    )

    model.articulation(
        "plunger_to_spout",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=spout,
        origin=Origin(xyz=(0.010, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=-1.75,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    bottle_body = object_model.get_part("bottle_body")
    plunger = object_model.get_part("plunger")
    spout = object_model.get_part("spout")
    pump_slide = object_model.get_articulation("bottle_to_plunger")
    spout_swivel = object_model.get_articulation("plunger_to_spout")

    slide_upper = 0.018
    if pump_slide.motion_limits is not None and pump_slide.motion_limits.upper is not None:
        slide_upper = pump_slide.motion_limits.upper

    swivel_upper = 1.75
    if spout_swivel.motion_limits is not None and spout_swivel.motion_limits.upper is not None:
        swivel_upper = spout_swivel.motion_limits.upper

    ctx.expect_contact(
        plunger,
        bottle_body,
        elem_a="pump_flange",
        elem_b="bottle_shell",
        name="pump flange seats on the bottle neck",
    )

    ctx.expect_contact(
        spout,
        plunger,
        elem_a="pivot_collar",
        elem_b="spout_seat",
        name="spout collar stays seated on the pump head",
    )

    with ctx.pose({pump_slide: 0.0}):
        ctx.expect_within(
            plunger,
            bottle_body,
            axes="xy",
            inner_elem="plunger_stem",
            outer_elem="bottle_shell",
            margin=0.002,
            name="resting plunger stem stays inside bottle footprint",
        )
        resting_plunger_origin = ctx.part_world_position(plunger)

    with ctx.pose({pump_slide: slide_upper}):
        ctx.expect_within(
            plunger,
            bottle_body,
            axes="xy",
            inner_elem="plunger_stem",
            outer_elem="bottle_shell",
            margin=0.002,
            name="extended plunger stem stays inside bottle footprint",
        )
        raised_plunger_origin = ctx.part_world_position(plunger)

    ctx.check(
        "plunger pumps upward along the bottle neck",
        resting_plunger_origin is not None
        and raised_plunger_origin is not None
        and raised_plunger_origin[2] > resting_plunger_origin[2] + 0.012,
        details=f"rest={resting_plunger_origin}, raised={raised_plunger_origin}",
    )

    with ctx.pose({spout_swivel: 0.0}):
        rest_tip_center = _aabb_center(ctx.part_element_world_aabb(spout, elem="spout_tip"))

    with ctx.pose({spout_swivel: swivel_upper}):
        rotated_tip_center = _aabb_center(ctx.part_element_world_aabb(spout, elem="spout_tip"))
        ctx.expect_contact(
            spout,
            plunger,
            elem_a="pivot_collar",
            elem_b="spout_seat",
            name="spout collar remains seated while rotated",
        )

    ctx.check(
        "spout swings laterally around its vertical pivot",
        rest_tip_center is not None
        and rotated_tip_center is not None
        and abs(rotated_tip_center[1] - rest_tip_center[1]) > 0.020,
        details=f"rest_tip={rest_tip_center}, rotated_tip={rotated_tip_center}",
    )
    ctx.check(
        "spout rotation keeps nearly the same height",
        rest_tip_center is not None
        and rotated_tip_center is not None
        and abs(rotated_tip_center[2] - rest_tip_center[2]) < 0.004,
        details=f"rest_tip={rest_tip_center}, rotated_tip={rotated_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
