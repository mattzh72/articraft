from __future__ import annotations

from math import cos, pi, sin

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
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_cctv_guyed_ptz")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_steel = model.material("dark_powdercoat", rgba=(0.12, 0.13, 0.14, 1.0))
    roof_membrane = model.material("roof_membrane", rgba=(0.18, 0.18, 0.17, 1.0))
    concrete = model.material("concrete_anchor", rgba=(0.48, 0.47, 0.43, 1.0))
    guy_wire_mat = model.material("stainless_guy_wire", rgba=(0.72, 0.74, 0.74, 1.0))
    camera_white = model.material("camera_white", rgba=(0.86, 0.88, 0.86, 1.0))
    glass = model.material("smoked_glass", rgba=(0.05, 0.07, 0.08, 0.65))
    rubber = model.material("black_gaskets", rgba=(0.03, 0.03, 0.03, 1.0))

    roof = model.part("roof")
    roof.visual(
        Box((2.4, 2.0, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=roof_membrane,
        name="roof_slab",
    )
    roof.visual(
        Cylinder(radius=0.18, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=galvanized,
        name="mast_base_plate",
    )
    roof.visual(
        Cylinder(radius=0.036, length=2.075),
        origin=Origin(xyz=(0.0, 0.0, 1.0975)),
        material=galvanized,
        name="mast_tube",
    )
    roof.visual(
        Cylinder(radius=0.075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 2.137)),
        material=galvanized,
        name="top_flange",
    )
    roof.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 1.650)),
        material=galvanized,
        name="guy_collar",
    )

    anchor_radius = 0.90
    for index, angle in enumerate((pi / 2.0, pi / 2.0 + 2.0 * pi / 3.0, pi / 2.0 + 4.0 * pi / 3.0)):
        x = anchor_radius * cos(angle)
        y = anchor_radius * sin(angle)
        roof.visual(
            Box((0.20, 0.14, 0.035)),
            origin=Origin(xyz=(x, y, 0.0125), rpy=(0.0, 0.0, angle)),
            material=concrete,
            name=f"anchor_pad_{index}",
        )
        roof.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(xyz=(x, y, 0.055)),
            material=galvanized,
            name=f"anchor_eye_{index}",
        )
        turnbuckle = tube_from_spline_points(
            [
                (x, y, 0.090),
                (x * 0.960, y * 0.960, 0.160),
                (x * 0.885, y * 0.885, 0.280),
            ],
            radius=0.012,
            samples_per_segment=4,
            radial_segments=12,
        )
        roof.visual(
            mesh_from_geometry(turnbuckle, f"turnbuckle_{index}"),
            material=dark_steel,
            name=f"turnbuckle_{index}",
        )
        cable = tube_from_spline_points(
            [
                (0.0, 0.0, 1.650),
                (x * 0.40, y * 0.40, 1.03),
                (x, y, 0.090),
            ],
            radius=0.006,
            samples_per_segment=8,
            radial_segments=12,
        )
        roof.visual(
            mesh_from_geometry(cable, f"guy_wire_{index}"),
            material=guy_wire_mat,
            name=f"guy_wire_{index}",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.090, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="pan_bearing",
    )
    pan_head.visual(
        Cylinder(radius=0.042, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_steel,
        name="rotating_neck",
    )
    pan_head.visual(
        Box((0.22, 0.32, 0.035)),
        origin=Origin(xyz=(0.09, 0.0, 0.145)),
        material=dark_steel,
        name="yoke_bridge",
    )
    for index, side in enumerate((-1.0, 1.0)):
        pan_head.visual(
            Box((0.075, 0.025, 0.180)),
            origin=Origin(xyz=(0.120, side * 0.1275, 0.235)),
            material=dark_steel,
            name=f"yoke_cheek_{index}",
        )
        pan_head.visual(
            Cylinder(radius=0.032, length=0.008),
            origin=Origin(xyz=(0.120, side * 0.144, 0.235), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"pivot_washer_{index}",
        )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.020, length=0.230),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="tilt_trunnion",
    )
    housing_mesh = superellipse_side_loft(
        [
            (0.025, 0.000, 0.132, 0.095),
            (0.155, 0.000, 0.150, 0.110),
            (0.315, 0.000, 0.135, 0.100),
        ],
        exponents=2.8,
        segments=56,
    ).rotate_z(-pi / 2.0).translate(0.0, 0.0, -0.075)
    camera.visual(
        mesh_from_geometry(housing_mesh, "camera_housing"),
        material=camera_white,
        name="camera_housing",
    )
    camera.visual(
        Cylinder(radius=0.058, length=0.036),
        origin=Origin(xyz=(0.333, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="lens_bezel",
    )
    camera.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.355, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    camera.visual(
        Box((0.365, 0.178, 0.016)),
        origin=Origin(xyz=(0.175, 0.0, 0.071)),
        material=camera_white,
        name="sunshield",
    )
    for index, side in enumerate((-1.0, 1.0)):
        camera.visual(
            Box((0.285, 0.012, 0.014)),
            origin=Origin(xyz=(0.175, side * 0.069, 0.062)),
            material=camera_white,
            name=f"sunshield_rail_{index}",
        )
    camera.visual(
        Box((0.040, 0.100, 0.070)),
        origin=Origin(xyz=(0.015, 0.0, 0.000)),
        material=dark_steel,
        name="rear_seal",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 2.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.120, 0.0, 0.235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.8, lower=-0.95, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    pan_joint = object_model.get_articulation("pan")
    tilt_joint = object_model.get_articulation("tilt")

    ctx.check("vertical pan axis", tuple(round(v, 3) for v in pan_joint.axis) == (0.0, 0.0, 1.0))
    ctx.check("horizontal tilt axis", tuple(round(v, 3) for v in tilt_joint.axis) == (0.0, 1.0, 0.0))
    ctx.expect_contact(
        pan_head,
        roof,
        elem_a="pan_bearing",
        elem_b="top_flange",
        contact_tol=0.002,
        name="pan bearing sits on mast top flange",
    )
    ctx.expect_within(
        camera,
        pan_head,
        axes="y",
        inner_elem="tilt_trunnion",
        outer_elem="yoke_bridge",
        margin=0.001,
        name="tilt trunnion spans within yoke width",
    )

    rest_position = ctx.part_world_position(camera)
    with ctx.pose({pan_joint: 1.0, tilt_joint: -0.45}):
        moved_position = ctx.part_world_position(camera)
        ctx.expect_gap(
            camera,
            pan_head,
            axis="z",
            positive_elem="camera_housing",
            negative_elem="yoke_bridge",
            min_gap=0.002,
            name="tilted housing clears yoke bridge",
        )
    ctx.check(
        "camera pose remains mounted while panning and tilting",
        rest_position is not None and moved_position is not None,
        details=f"rest={rest_position}, moved={moved_position}",
    )

    return ctx.report()


object_model = build_object_model()
