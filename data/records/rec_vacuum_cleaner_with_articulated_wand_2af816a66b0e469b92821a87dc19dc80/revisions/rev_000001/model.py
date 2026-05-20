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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_graphite = model.material("body_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    translucent_bin = model.material("translucent_bin", rgba=(0.45, 0.70, 0.92, 0.45))
    nozzle_black = model.material("nozzle_black", rgba=(0.13, 0.14, 0.15, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.18, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.19, 0.06)),
        origin=Origin(xyz=(-0.01, 0.0, 0.03)),
        material=charcoal,
        name="base_tray",
    )
    body.visual(
        Cylinder(radius=0.074, length=0.15),
        origin=Origin(xyz=(-0.10, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_graphite,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.13),
        origin=Origin(xyz=(0.02, 0.0, 0.102), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_bin,
        name="dust_bin",
    )
    body.visual(
        Box((0.10, 0.08, 0.08)),
        origin=Origin(xyz=(0.105, 0.0, 0.092)),
        material=body_graphite,
        name="front_neck",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.032),
        origin=Origin(xyz=(0.154, 0.0, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="wand_socket",
    )
    body.visual(
        Box((0.10, 0.05, 0.03)),
        origin=Origin(xyz=(-0.01, 0.0, 0.145)),
        material=accent_red,
        name="top_latch",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.11, 0.0, 0.120),
                    (-0.08, 0.0, 0.156),
                    (-0.01, 0.0, 0.176),
                    (0.07, 0.0, 0.145),
                ],
                radius=0.012,
                samples_per_segment=16,
                radial_segments=18,
            ),
            "carry_handle",
        ),
        material=body_graphite,
        name="carry_handle",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(-0.11, 0.092, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_wheel",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(-0.11, -0.092, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_wheel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.19, 0.20)),
        mass=6.0,
        origin=Origin(xyz=(-0.01, 0.0, 0.08)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.023, length=0.048),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_collar",
    )
    upper_wand.visual(
        Cylinder(radius=0.016, length=0.29),
        origin=Origin(xyz=(0.170, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.08, 0.034, 0.034)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=charcoal,
        name="hinge_housing",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.34, 0.05, 0.05)),
        mass=0.8,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.021, length=0.045),
        origin=Origin(xyz=(0.0225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_joint_collar",
    )
    lower_wand.visual(
        Cylinder(radius=0.015, length=0.27),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.07, 0.030, 0.030)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=charcoal,
        name="front_socket",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.31, 0.05, 0.05)),
        mass=0.7,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Box((0.29, 0.092, 0.034)),
        origin=Origin(xyz=(0.145, 0.0, -0.078)),
        material=nozzle_black,
        name="nozzle_base",
    )
    nozzle.visual(
        Box((0.060, 0.042, 0.090)),
        origin=Origin(xyz=(0.030, 0.0, -0.045)),
        material=charcoal,
        name="nozzle_neck",
    )
    nozzle.visual(
        Box((0.105, 0.052, 0.050)),
        origin=Origin(xyz=(0.070, 0.0, -0.055)),
        material=body_graphite,
        name="suction_channel",
    )
    nozzle.visual(
        Cylinder(radius=0.012, length=0.084),
        origin=Origin(xyz=(0.272, 0.0, -0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="front_bumper",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.29, 0.10, 0.11)),
        mass=1.1,
        origin=Origin(xyz=(0.145, 0.0, -0.050)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.170, 0.0, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-0.35, upper=1.05),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.55, upper=0.95),
    )
    model.articulation(
        "lower_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.45, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("floor_nozzle")

    shoulder = object_model.get_articulation("body_to_upper_wand")
    elbow = object_model.get_articulation("upper_to_lower_wand")
    nozzle_pitch = object_model.get_articulation("lower_to_floor_nozzle")

    ctx.expect_origin_distance(body, upper_wand, axes="y", max_dist=0.001, name="upper wand stays on body midline")
    ctx.expect_origin_distance(body, lower_wand, axes="y", max_dist=0.001, name="lower wand stays on body midline")
    ctx.expect_origin_distance(body, nozzle, axes="y", max_dist=0.001, name="nozzle stays on body midline")
    ctx.expect_origin_gap(nozzle, body, axis="x", min_gap=0.70, name="wand chain reaches well forward of the body")
    ctx.expect_overlap(nozzle, body, axes="y", min_overlap=0.09, name="body and nozzle remain centered in plan")

    def center_z(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_lower_z = center_z("lower_wand", "lower_tube")
    with ctx.pose({shoulder: 0.70, elbow: 0.55}):
        raised_lower_z = center_z("lower_wand", "lower_tube")
    ctx.check(
        "elbows raise the wand upward",
        rest_lower_z is not None and raised_lower_z is not None and raised_lower_z > rest_lower_z + 0.10,
        details=f"rest_lower_z={rest_lower_z}, raised_lower_z={raised_lower_z}",
    )

    rest_nozzle_z = center_z("floor_nozzle", "nozzle_base")
    with ctx.pose({nozzle_pitch: 0.60}):
        pitched_nozzle_z = center_z("floor_nozzle", "nozzle_base")
    ctx.check(
        "floor nozzle pitches upward at positive joint values",
        rest_nozzle_z is not None and pitched_nozzle_z is not None and pitched_nozzle_z > rest_nozzle_z + 0.03,
        details=f"rest_nozzle_z={rest_nozzle_z}, pitched_nozzle_z={pitched_nozzle_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
