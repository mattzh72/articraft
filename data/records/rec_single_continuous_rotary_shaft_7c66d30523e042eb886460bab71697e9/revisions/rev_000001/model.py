from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_RADIUS = 0.019
BORE_RADIUS = 0.0215
BODY_FRONT_X = 0.145
SHAFT_REAR_X = -0.065
SHAFT_FRONT_X = 0.315
FACEPLATE_THICKNESS = 0.012
FACEPLATE_RADIUS = 0.040
COLLAR_RADIUS = 0.030
COLLAR_REAR_X = -0.063
COLLAR_FRONT_X = -0.055


def _x_cylinder(x0: float, x1: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(x1 - x0).translate((x0, 0.0, 0.0))


def _build_bearing_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.180, 0.140, 0.020).translate((0.030, 0.0, -0.090))
    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.020, -0.045),
                (-0.020, 0.045),
                (0.080, -0.045),
                (0.080, 0.045),
            ]
        )
        .circle(0.007)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.110))
    )
    base = base.cut(mount_holes)

    pedestal = cq.Workplane("XY").box(0.118, 0.108, 0.064).translate((0.032, 0.0, -0.050))
    barrel = _x_cylinder(-0.040, 0.110, 0.055)
    rear_flange = _x_cylinder(-0.055, -0.025, 0.068)
    nose = _x_cylinder(0.110, BODY_FRONT_X, 0.042)

    body = base.union(pedestal).union(barrel).union(rear_flange).union(nose)

    underside_relief = (
        cq.Workplane("XZ")
        .rect(0.070, 0.030)
        .extrude(0.070)
        .translate((0.048, -0.035, -0.058))
    )
    body = body.cut(underside_relief)

    bore = _x_cylinder(-0.055, BODY_FRONT_X, BORE_RADIUS)
    body = body.cut(bore)

    body = body.faces(">X").edges().chamfer(0.003)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhung_spindle_stage")

    model.material("housing_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("steel", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("faceplate_steel", rgba=(0.63, 0.66, 0.70, 1.0))

    bearing_body = model.part("bearing_body")
    bearing_body.visual(
        mesh_from_cadquery(_build_bearing_body_shape(), "bearing_body"),
        material="housing_gray",
        name="housing",
    )

    spindle = model.part("spindle")
    shaft_length = SHAFT_FRONT_X - SHAFT_REAR_X
    spindle.visual(
        mesh_from_cadquery(_x_cylinder(SHAFT_REAR_X, SHAFT_FRONT_X, SHAFT_RADIUS), "shaft_core"),
        material="steel",
        name="shaft_core",
    )
    spindle.visual(
        mesh_from_cadquery(
            _x_cylinder(COLLAR_REAR_X, COLLAR_FRONT_X, COLLAR_RADIUS),
            "retaining_collar",
        ),
        material="faceplate_steel",
        name="retaining_collar",
    )
    spindle.visual(
        mesh_from_cadquery(
            _x_cylinder(
                SHAFT_FRONT_X,
                SHAFT_FRONT_X + FACEPLATE_THICKNESS,
                FACEPLATE_RADIUS,
            ),
            "faceplate",
        ),
        material="faceplate_steel",
        name="faceplate",
    )

    model.articulation(
        "body_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bearing_body,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
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

    bearing_body = object_model.get_part("bearing_body")
    spindle = object_model.get_part("spindle")
    spindle_joint = object_model.get_articulation("body_to_spindle")

    ctx.check("bearing body exists", bearing_body is not None)
    ctx.check("spindle exists", spindle is not None)
    ctx.check(
        "spindle uses a continuous revolute stage",
        spindle_joint.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spindle_joint.joint_type}",
    )
    ctx.check(
        "spindle axis is aligned with the supported shaft axis",
        tuple(round(v, 6) for v in spindle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spindle_joint.axis}",
    )
    ctx.expect_origin_distance(
        spindle,
        bearing_body,
        axes="yz",
        max_dist=1e-6,
        name="spindle stays centered in the bearing body",
    )
    ctx.expect_overlap(
        spindle,
        bearing_body,
        axes="x",
        elem_a="shaft_core",
        min_overlap=0.145,
        name="shaft remains deeply retained through the support body",
    )
    ctx.expect_contact(
        spindle,
        bearing_body,
        elem_a="retaining_collar",
        name="spindle is thrust-supported by the bearing body",
    )
    ctx.expect_gap(
        spindle,
        bearing_body,
        axis="x",
        positive_elem="faceplate",
        min_gap=0.160,
        name="faceplate projects well past the front support",
    )

    rest_pos = ctx.part_world_position(spindle)
    with ctx.pose({spindle_joint: math.pi * 0.75}):
        turned_pos = ctx.part_world_position(spindle)
        ctx.expect_gap(
            spindle,
            bearing_body,
            axis="x",
            positive_elem="faceplate",
            min_gap=0.160,
            name="overhang is preserved while the spindle rotates",
        )

    ctx.check(
        "spindle rotates in place about the body support axis",
        rest_pos is not None
        and turned_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, turned_pos)) <= 1e-9,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
