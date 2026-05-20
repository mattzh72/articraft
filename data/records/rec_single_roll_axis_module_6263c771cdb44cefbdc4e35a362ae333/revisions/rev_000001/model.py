from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.220
BASE_WIDTH = 0.138
BASE_THICKNESS = 0.014
BASE_CENTER_X = -0.072

AXIS_Z = 0.054

PEDESTAL_CENTER_X = -0.018
PEDESTAL_BOTTOM_LENGTH = 0.112
PEDESTAL_TOP_LENGTH = 0.072
PEDESTAL_WIDTH = 0.066
PEDESTAL_TOP_Z = -0.020

HOUSING_RADIUS = 0.022
HOUSING_LENGTH = 0.064
REAR_RING_RADIUS = 0.025
REAR_RING_LENGTH = 0.008
FRONT_RING_RADIUS = 0.026
FRONT_RING_LENGTH = 0.008
BORE_RADIUS = 0.013

SHAFT_RADIUS = 0.0112
SHAFT_LENGTH = 0.088
SHAFT_CENTER_X = 0.010

FLANGE_RADIUS = 0.030
FLANGE_THICKNESS = 0.006
FLANGE_CENTER_X = HOUSING_LENGTH / 2.0 + FRONT_RING_LENGTH - 0.001 + FLANGE_THICKNESS / 2.0

NOSE_RADIUS = 0.0115
NOSE_LENGTH = 0.028
NOSE_CENTER_X = FLANGE_CENTER_X + FLANGE_THICKNESS / 2.0 + NOSE_LENGTH / 2.0
NOSE_FLAT_DEPTH = 0.004


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _base_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_THICKNESS)
        .translate((BASE_CENTER_X, 0.0, -AXIS_Z))
        .edges("|Z")
        .fillet(0.006)
    )

    pedestal_profile = [
        (PEDESTAL_CENTER_X - PEDESTAL_BOTTOM_LENGTH / 2.0, -AXIS_Z + BASE_THICKNESS),
        (PEDESTAL_CENTER_X + PEDESTAL_BOTTOM_LENGTH / 2.0, -AXIS_Z + BASE_THICKNESS),
        (PEDESTAL_CENTER_X + PEDESTAL_TOP_LENGTH / 2.0, PEDESTAL_TOP_Z),
        (PEDESTAL_CENTER_X - PEDESTAL_TOP_LENGTH / 2.0, PEDESTAL_TOP_Z),
    ]
    pedestal = (
        cq.Workplane("XZ")
        .polyline(pedestal_profile)
        .close()
        .extrude(PEDESTAL_WIDTH)
        .translate((0.0, PEDESTAL_WIDTH / 2.0, 0.0))
    )

    housing = _x_cylinder(HOUSING_RADIUS, HOUSING_LENGTH, (0.0, 0.0, 0.0))
    rear_ring = _x_cylinder(
        REAR_RING_RADIUS,
        REAR_RING_LENGTH,
        (-HOUSING_LENGTH / 2.0 - REAR_RING_LENGTH / 2.0 + 0.001, 0.0, 0.0),
    )
    front_ring = _x_cylinder(
        FRONT_RING_RADIUS,
        FRONT_RING_LENGTH,
        (HOUSING_LENGTH / 2.0 + FRONT_RING_LENGTH / 2.0 - 0.001, 0.0, 0.0),
    )

    body = base_plate.union(pedestal).union(housing).union(rear_ring).union(front_ring)

    bore = _x_cylinder(
        BORE_RADIUS,
        HOUSING_LENGTH + FRONT_RING_LENGTH + REAR_RING_LENGTH + 0.020,
        (0.0, 0.0, 0.0),
    )
    front_counterbore = _x_cylinder(
        BORE_RADIUS + 0.0035,
        FRONT_RING_LENGTH + 0.010,
        (HOUSING_LENGTH / 2.0 + FRONT_RING_LENGTH / 2.0 - 0.002, 0.0, 0.0),
    )
    body = body.cut(bore).cut(front_counterbore)

    return body


def _spindle_core_shape() -> cq.Workplane:
    shaft = _x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (SHAFT_CENTER_X, 0.0, 0.0))
    nose = _x_cylinder(NOSE_RADIUS, NOSE_LENGTH, (NOSE_CENTER_X, 0.0, 0.0))
    core = shaft.union(nose)

    flat_cut = (
        cq.Workplane("XY")
        .box(NOSE_LENGTH + 0.004, NOSE_RADIUS * 3.0, 0.020)
        .translate(
            (
                NOSE_CENTER_X,
                0.0,
                NOSE_RADIUS - NOSE_FLAT_DEPTH + 0.010,
            )
        )
    )
    return core.cut(flat_cut)


def _flange_shape() -> cq.Workplane:
    return _x_cylinder(FLANGE_RADIUS, FLANGE_THICKNESS, (FLANGE_CENTER_X, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_roll_cartridge")

    model.material("powder_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("machined_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "cartridge_base"),
        material="powder_charcoal",
        name="cartridge_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, AXIS_Z + HOUSING_RADIUS)),
        mass=4.8,
        origin=Origin(xyz=(BASE_CENTER_X, 0.0, (-AXIS_Z + HOUSING_RADIUS) / 2.0)),
    )

    output_nose = model.part("output_nose")
    output_nose.visual(
        mesh_from_cadquery(_spindle_core_shape(), "spindle_core"),
        material="machined_steel",
        name="spindle_core",
    )
    output_nose.visual(
        mesh_from_cadquery(_flange_shape(), "spindle_flange"),
        material="machined_steel",
        name="spindle_flange",
    )
    output_nose.inertial = Inertial.from_geometry(
        Cylinder(radius=FLANGE_RADIUS, length=SHAFT_LENGTH),
        mass=0.72,
        origin=Origin(xyz=(SHAFT_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_output_nose",
        ArticulationType.REVOLUTE,
        parent=base,
        child=output_nose,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-2.0 * pi,
            upper=2.0 * pi,
            effort=8.0,
            velocity=14.0,
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

    base = object_model.get_part("base")
    output_nose = object_model.get_part("output_nose")
    spin = object_model.get_articulation("base_to_output_nose")
    flange = output_nose.get_visual("spindle_flange")

    ctx.check(
        "roll cartridge parts exist",
        base is not None and output_nose is not None and spin is not None,
        details="Expected base, output nose, and one spindle articulation.",
    )
    ctx.check(
        "spin articulation follows the supported roll axis",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.check(
        "spin articulation has broad rotary travel",
        spin.motion_limits is not None
        and spin.motion_limits.lower is not None
        and spin.motion_limits.upper is not None
        and spin.motion_limits.lower <= -pi
        and spin.motion_limits.upper >= pi,
        details=f"limits={spin.motion_limits}",
    )
    ctx.expect_contact(
        output_nose,
        base,
        elem_a=flange,
        contact_tol=1e-6,
        name="flange seats against the cartridge face",
    )
    with ctx.pose({spin: 1.1}):
        ctx.expect_contact(
            output_nose,
            base,
            elem_a=flange,
            contact_tol=1e-6,
            name="flange stays seated when the nose spins",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
