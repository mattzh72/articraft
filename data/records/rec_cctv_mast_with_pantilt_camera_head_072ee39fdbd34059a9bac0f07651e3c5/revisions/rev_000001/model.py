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


def _tube_shape(
    length: float,
    outer_radius: float,
    inner_radius: float,
    *,
    collar_radius: float | None = None,
    collar_height: float = 0.0,
) -> cq.Workplane:
    """Open telescoping tube, extruded from local z=0 to z=length."""
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    if collar_radius is not None and collar_height > 0.0:
        collar = (
            cq.Workplane("XY")
            .circle(collar_radius)
            .circle(inner_radius)
            .extrude(collar_height)
            .translate((0.0, 0.0, length - collar_height))
        )
        tube = tube.union(collar)
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_cctv_mast")

    dark_metal = model.material("dark_galvanized_metal", rgba=(0.12, 0.13, 0.13, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    ballast_gray = model.material("rubberized_ballast_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    camera_white = model.material("powder_coated_camera_white", rgba=(0.88, 0.88, 0.84, 1.0))
    glass = model.material("dark_lens_glass", rgba=(0.03, 0.05, 0.08, 0.82))

    # Root: heavy ballast slab, fixed lower sleeve, welded flange, and a clamp knob.
    base = model.part("ballast_base")
    base.visual(
        Box((0.82, 0.56, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=ballast_gray,
        name="ballast_slab",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_metal,
        name="welded_flange",
    )
    base.visual(
        mesh_from_cadquery(
            _tube_shape(0.83, 0.065, 0.048, collar_radius=0.079, collar_height=0.07),
            "outer_sleeve",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="outer_sleeve",
    )
    # Rubber feet are slightly embedded in the slab so they read as bolted-on pads.
    for idx, (x, y) in enumerate(
        ((-0.32, -0.20), (-0.32, 0.20), (0.32, -0.20), (0.32, 0.20))
    ):
        base.visual(
            Cylinder(radius=0.055, length=0.026),
            origin=Origin(xyz=(x, y, 0.004)),
            material=black,
            name=f"rubber_foot_{idx}",
        )
    base.visual(
        Box((0.070, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, -0.070, 0.905)),
        material=dark_metal,
        name="clamp_boss",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.0, -0.090, 0.905), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(0.0, -0.115, 0.905), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="clamp_knob",
    )

    # First moving stage: a hollow middle tube with a locking collar at its top.
    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(
            _tube_shape(1.40, 0.040, 0.030, collar_radius=0.050, collar_height=0.065),
            "middle_tube",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.65)),
        material=brushed,
        name="middle_tube",
    )
    middle_stage.visual(
        mesh_from_cadquery(
            _tube_shape(0.040, 0.072, 0.038),
            "middle_stop_collar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="middle_stop_collar",
    )

    # Second moving stage: slender upper pole with a pan-bearing cap at the top.
    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.024, length=1.05),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=brushed,
        name="upper_pole",
    )
    upper_stage.visual(
        mesh_from_cadquery(
            _tube_shape(0.035, 0.045, 0.022),
            "upper_stop_collar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="upper_stop_collar",
    )
    upper_stage.visual(
        Cylinder(radius=0.044, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=dark_metal,
        name="pan_cap",
    )

    # Pan part: turntable plus U-yoke side plates that carry the tilt trunnions.
    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="turntable",
    )
    pan_yoke.visual(
        Cylinder(radius=0.030, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_metal,
        name="yoke_stem",
    )
    pan_yoke.visual(
        Box((0.085, 0.205, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_metal,
        name="yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.055, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.085, 0.170)),
        material=dark_metal,
        name="yoke_cheek_0",
    )
    pan_yoke.visual(
        Box((0.055, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, 0.085, 0.170)),
        material=dark_metal,
        name="yoke_cheek_1",
    )

    # Box camera head carried by the tilt axis.  The child frame is the trunnion axis.
    camera = model.part("camera_head")
    camera.visual(
        Box((0.240, 0.130, 0.100)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=camera_white,
        name="camera_body",
    )
    camera.visual(
        Box((0.285, 0.165, 0.014)),
        origin=Origin(xyz=(0.080, 0.0, 0.057)),
        material=camera_white,
        name="sun_hood",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.152),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(0.230, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.265, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.55),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.16, lower=0.0, upper=0.45),
    )
    model.articulation(
        "upper_to_pan",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.45, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("ballast_base")
    middle = object_model.get_part("middle_stage")
    upper = object_model.get_part("upper_stage")
    pan = object_model.get_part("pan_yoke")
    camera = object_model.get_part("camera_head")
    base_slide = object_model.get_articulation("base_to_middle")
    upper_slide = object_model.get_articulation("middle_to_upper")
    pan_joint = object_model.get_articulation("upper_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    ctx.expect_within(
        middle,
        base,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="middle stage is centered inside the lower sleeve",
    )
    ctx.expect_overlap(
        middle,
        base,
        axes="z",
        elem_a="middle_tube",
        elem_b="outer_sleeve",
        min_overlap=0.60,
        name="middle stage has deep retained insertion when collapsed",
    )
    ctx.expect_within(
        upper,
        middle,
        axes="xy",
        inner_elem="upper_pole",
        outer_elem="middle_tube",
        margin=0.002,
        name="upper pole is centered inside the middle tube",
    )
    ctx.expect_overlap(
        upper,
        middle,
        axes="z",
        elem_a="upper_pole",
        elem_b="middle_tube",
        min_overlap=0.50,
        name="upper pole has retained insertion when collapsed",
    )
    ctx.expect_contact(
        camera,
        pan,
        elem_a="tilt_trunnion",
        elem_b="yoke_cheek_0",
        contact_tol=0.001,
        name="camera trunnion seats in one yoke cheek",
    )
    ctx.expect_contact(
        camera,
        pan,
        elem_a="tilt_trunnion",
        elem_b="yoke_cheek_1",
        contact_tol=0.001,
        name="camera trunnion seats in the opposite yoke cheek",
    )

    rest_middle_z = ctx.part_world_position(middle)[2]
    rest_upper_z = ctx.part_world_position(upper)[2]
    with ctx.pose({base_slide: 0.55, upper_slide: 0.45}):
        ctx.expect_overlap(
            middle,
            base,
            axes="z",
            elem_a="middle_tube",
            elem_b="outer_sleeve",
            min_overlap=0.09,
            name="extended middle stage remains captured in lower sleeve",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="upper_pole",
            elem_b="middle_tube",
            min_overlap=0.09,
            name="extended upper pole remains captured in middle tube",
        )
        extended_middle_z = ctx.part_world_position(middle)[2]
        extended_upper_z = ctx.part_world_position(upper)[2]

    ctx.check(
        "telescoping stages extend upward",
        extended_middle_z > rest_middle_z + 0.50 and extended_upper_z > rest_upper_z + 0.95,
        details=(
            f"middle z {rest_middle_z:.3f}->{extended_middle_z:.3f}, "
            f"upper z {rest_upper_z:.3f}->{extended_upper_z:.3f}"
        ),
    )

    rest_camera = ctx.part_world_position(camera)
    with ctx.pose({pan_joint: math.pi / 2.0, tilt_joint: 0.60}):
        posed_camera = ctx.part_world_position(camera)
    ctx.check(
        "camera has independent pan and tilt axes",
        posed_camera is not None
        and rest_camera is not None
        and abs(posed_camera[0] - rest_camera[0]) < 0.01
        and abs(posed_camera[1] - rest_camera[1]) < 0.01,
        details=f"rest={rest_camera}, posed={posed_camera}",
    )

    return ctx.report()


object_model = build_object_model()
