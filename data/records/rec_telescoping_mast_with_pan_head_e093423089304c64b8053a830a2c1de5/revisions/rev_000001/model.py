from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_cylinder(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 64,
):
    """Thin-walled open annular cylinder, used for clearanced telescoping tubes."""

    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _ring_visual(part, name, outer_radius, inner_radius, z_center, height, material):
    part.visual(
        mesh_from_geometry(
            _hollow_cylinder(
                outer_radius,
                inner_radius,
                z_center - height * 0.5,
                z_center + height * 0.5,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_with_pan_head")

    matte_black = model.material("matte_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.13, 0.14, 0.16, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.23, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=matte_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.19, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rubber,
        name="rubber_foot",
    )
    base.visual(
        mesh_from_geometry(_hollow_cylinder(0.055, 0.043, 0.052, 0.580), "base_sleeve"),
        material=dark_metal,
        name="base_sleeve",
    )
    _ring_visual(base, "base_lip", 0.068, 0.043, 0.540, 0.080, dark_metal)
    base.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.084, 0.0, 0.540), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="base_lock_screw",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.116, 0.0, 0.540), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="base_lock_knob",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.64),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        mesh_from_geometry(_hollow_cylinder(0.038, 0.030, -0.460, 0.420), "lower_tube"),
        material=brushed_aluminum,
        name="lower_tube",
    )
    _ring_visual(lower_mast, "lower_seat_flange", 0.058, 0.037, 0.010, 0.020, dark_metal)
    _ring_visual(lower_mast, "lower_lip", 0.047, 0.028, 0.390, 0.060, dark_metal)
    lower_mast.visual(
        Cylinder(radius=0.0085, length=0.038),
        origin=Origin(xyz=(0.058, 0.0, 0.426), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lower_lock_screw",
    )
    lower_mast.visual(
        Cylinder(radius=0.0125, length=0.016),
        origin=Origin(xyz=(0.085, 0.0, 0.426), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lower_lock_knob",
    )
    lower_mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.90),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    middle_mast = model.part("middle_mast")
    middle_mast.visual(
        mesh_from_geometry(_hollow_cylinder(0.026, 0.019, -0.390, 0.350), "middle_tube"),
        material=brushed_aluminum,
        name="middle_tube",
    )
    _ring_visual(middle_mast, "middle_seat_flange", 0.039, 0.025, 0.010, 0.020, dark_metal)
    _ring_visual(middle_mast, "middle_lip", 0.034, 0.018, 0.324, 0.052, dark_metal)
    middle_mast.visual(
        Cylinder(radius=0.0068, length=0.030),
        origin=Origin(xyz=(0.043, 0.0, 0.353), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="middle_lock_screw",
    )
    middle_mast.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.065, 0.0, 0.353), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="middle_lock_knob",
    )
    middle_mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.76),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        mesh_from_geometry(_hollow_cylinder(0.017, 0.012, -0.300, 0.300), "upper_tube"),
        material=brushed_aluminum,
        name="upper_tube",
    )
    _ring_visual(upper_mast, "upper_seat_flange", 0.030, 0.016, 0.009, 0.018, dark_metal)
    upper_mast.visual(
        mesh_from_geometry(_hollow_cylinder(0.026, 0.012, 0.279, 0.325), "pan_mount_collar"),
        material=dark_metal,
        name="pan_mount_collar",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.62),
        mass=0.40,
        origin=Origin(xyz=(0.0, 0.0, 0.00)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.046, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=dark_metal,
        name="pan_bearing",
    )
    pan_head.visual(
        Cylinder(radius=0.058, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=matte_black,
        name="pan_turntable",
    )
    pan_head.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=dark_metal,
        name="head_stem",
    )
    pan_head.visual(
        Box((0.105, 0.074, 0.064)),
        origin=Origin(xyz=(0.070, 0.0, 0.102)),
        material=dark_metal,
        name="head_block",
    )
    pan_head.visual(
        Box((0.034, 0.054, 0.048)),
        origin=Origin(xyz=(0.126, 0.0, 0.102)),
        material=dark_metal,
        name="faceplate_bridge",
    )
    pan_head.visual(
        Box((0.016, 0.124, 0.092)),
        origin=Origin(xyz=(0.150, 0.0, 0.102)),
        material=matte_black,
        name="faceplate",
    )
    for index, (y_pos, z_pos) in enumerate(
        [(-0.040, 0.074), (0.040, 0.074), (-0.040, 0.130), (0.040, 0.130)]
    ):
        pan_head.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.160, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_aluminum,
            name=f"face_screw_{index}",
        )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.13, 0.16)),
        mass=0.65,
        origin=Origin(xyz=(0.075, 0.0, 0.08)),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=130.0, velocity=0.22, lower=0.0, upper=0.36),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=middle_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.20, lower=0.0, upper=0.30),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle_mast,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.18, lower=0.0, upper=0.23),
    )
    model.articulation(
        "upper_to_pan",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower = object_model.get_part("lower_mast")
    middle = object_model.get_part("middle_mast")
    upper = object_model.get_part("upper_mast")
    pan = object_model.get_part("pan_head")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_upper = object_model.get_articulation("middle_to_upper")
    upper_to_pan = object_model.get_articulation("upper_to_pan")

    ctx.expect_within(
        lower,
        base,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="base_sleeve",
        margin=0.0,
        name="lower section is centered inside base sleeve",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="z",
        elem_a="lower_tube",
        elem_b="base_sleeve",
        min_overlap=0.30,
        name="lower section has deep collapsed insertion",
    )
    ctx.expect_within(
        middle,
        lower,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="lower_tube",
        margin=0.0,
        name="middle section is centered inside lower section",
    )
    ctx.expect_within(
        upper,
        middle,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="middle_tube",
        margin=0.0,
        name="upper section is centered inside middle section",
    )
    ctx.expect_contact(
        pan,
        upper,
        elem_a="pan_bearing",
        elem_b="pan_mount_collar",
        contact_tol=0.001,
        name="pan bearing sits on mast collar",
    )

    rest_upper_pos = ctx.part_world_position(upper)
    with ctx.pose(
        {
            base_to_lower: 0.36,
            lower_to_middle: 0.30,
            middle_to_upper: 0.23,
        }
    ):
        ctx.expect_overlap(
            lower,
            base,
            axes="z",
            elem_a="lower_tube",
            elem_b="base_sleeve",
            min_overlap=0.070,
            name="extended lower section remains retained",
        )
        ctx.expect_overlap(
            middle,
            lower,
            axes="z",
            elem_a="middle_tube",
            elem_b="lower_tube",
            min_overlap=0.070,
            name="extended middle section remains retained",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="upper_tube",
            elem_b="middle_tube",
            min_overlap=0.050,
            name="extended upper section remains retained",
        )
        extended_upper_pos = ctx.part_world_position(upper)

    ctx.check(
        "serial prismatic mast rises vertically",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.80,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    with ctx.pose({upper_to_pan: 0.0}):
        face_aabb = ctx.part_element_world_aabb(pan, elem="faceplate")
        face_center_y_rest = (face_aabb[0][1] + face_aabb[1][1]) * 0.5 if face_aabb else None
    with ctx.pose({upper_to_pan: math.pi / 2.0}):
        face_aabb = ctx.part_element_world_aabb(pan, elem="faceplate")
        face_center_y_rotated = (face_aabb[0][1] + face_aabb[1][1]) * 0.5 if face_aabb else None

    ctx.check(
        "pan head rotates faceplate about mast centerline",
        face_center_y_rest is not None
        and face_center_y_rotated is not None
        and face_center_y_rotated > face_center_y_rest + 0.12,
        details=f"rest_y={face_center_y_rest}, rotated_y={face_center_y_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
