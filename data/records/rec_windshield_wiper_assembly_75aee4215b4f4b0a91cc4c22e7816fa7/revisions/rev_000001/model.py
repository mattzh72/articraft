from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tapered_bar(
    length: float,
    root_width: float,
    tip_width: float,
    thickness: float,
    *,
    y0: float = 0.0,
    z_center: float = 0.0,
) -> MeshGeometry:
    """Simple tapered rectangular beam running in local +Y."""

    y1 = y0 + length
    z0 = z_center - thickness / 2.0
    z1 = z_center + thickness / 2.0
    rw = root_width / 2.0
    tw = tip_width / 2.0
    geom = MeshGeometry()
    verts = [
        (-rw, y0, z0),
        (rw, y0, z0),
        (rw, y0, z1),
        (-rw, y0, z1),
        (-tw, y1, z0),
        (tw, y1, z0),
        (tw, y1, z1),
        (-tw, y1, z1),
    ]
    for v in verts:
        geom.add_vertex(*v)
    for tri in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (1, 5, 6),
        (1, 6, 2),
        (0, 3, 7),
        (0, 7, 4),
    ):
        geom.add_face(*tri)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_wiper_assembly")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.12, 0.125, 0.13, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.46, 0.47, 0.46, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    anodized = model.material("soft_anodized", rgba=(0.20, 0.21, 0.22, 1.0))
    seam_shadow = model.material("seam_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    cowl = model.part("cowl_frame")
    cowl.visual(
        Box((1.18, 0.24, 0.026)),
        origin=Origin(xyz=(0.0, -0.025, 0.013)),
        material=matte_black,
        name="stamped_cowl_tray",
    )
    cowl.visual(
        Box((1.10, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.094, 0.028)),
        material=seam_shadow,
        name="windshield_seam",
    )
    cowl.visual(
        Box((1.16, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.150, 0.035)),
        material=satin_graphite,
        name="front_rolled_lip",
    )
    cowl.visual(
        Box((0.006, 0.20, 0.005)),
        origin=Origin(xyz=(-0.575, -0.030, 0.028)),
        material=seam_shadow,
        name="side_seam_0",
    )
    cowl.visual(
        Box((0.006, 0.20, 0.005)),
        origin=Origin(xyz=(0.575, -0.030, 0.028)),
        material=seam_shadow,
        name="side_seam_1",
    )

    pivot_x = 0.42
    pivot_y = 0.020
    pivot_z = 0.074
    for suffix, x in (("0", -pivot_x), ("1", pivot_x)):
        cowl.visual(
            Cylinder(radius=0.050, length=0.025),
            origin=Origin(xyz=(x, pivot_y, 0.0385)),
            material=satin_graphite,
            name=f"pivot_pedestal_{suffix}",
        )
        cowl.visual(
            Box((0.110, 0.018, 0.008)),
            origin=Origin(xyz=(x, -0.030, 0.030)),
            material=satin_graphite,
            name=f"pivot_mount_rib_{suffix}",
        )
    cowl.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(-pivot_x, pivot_y, 0.061)),
        material=anodized,
        name="upper_collar_0",
    )
    cowl.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.061)),
        material=anodized,
        name="upper_collar_1",
    )
    cowl.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-pivot_x, pivot_y, 0.077)),
        material=satin_metal,
        name="spindle_stub_0",
    )
    cowl.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.077)),
        material=satin_metal,
        name="spindle_stub_1",
    )

    cowl.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, -0.085, 0.045)),
        material=satin_metal,
        name="motor_gear_housing",
    )
    cowl.visual(
        Box((0.135, 0.080, 0.040)),
        origin=Origin(xyz=(0.0, -0.082, 0.036)),
        material=satin_graphite,
        name="motor_casting",
    )
    cowl.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, -0.085, 0.066)),
        material=anodized,
        name="motor_output_boss",
    )
    for i, x in enumerate((-0.52, -0.30, 0.30, 0.52)):
        cowl.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.105, 0.029)),
            material=satin_metal,
            name=f"tray_fastener_{i}",
        )

    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=anodized,
        name="crank_disk",
    )
    motor_crank.visual(
        Box((0.062, 0.014, 0.008)),
        origin=Origin(xyz=(0.031, 0.0, 0.013)),
        material=satin_metal,
        name="eccentric_arm",
    )
    motor_crank.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.062, 0.0, 0.022)),
        material=satin_metal,
        name="crank_pin",
    )
    motor_crank.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.062, 0.0, 0.035)),
        material=satin_metal,
        name="pin_nut",
    )

    left_wiper = model.part("left_wiper")
    right_wiper = model.part("right_wiper")
    for side, wiper, mirror in (("left", left_wiper, -1.0), ("right", right_wiper, 1.0)):
        wiper.visual(
            Cylinder(radius=0.027, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=satin_graphite,
            name="hub_cap",
        )
        wiper.visual(
            Cylinder(radius=0.018, length=0.013),
            origin=Origin(xyz=(0.0, 0.0, 0.0255)),
            material=anodized,
            name="spindle_nut",
        )
        wiper.visual(
            Box((0.090, 0.022, 0.010)),
            origin=Origin(xyz=(0.0, -0.035, 0.012)),
            material=satin_metal,
            name="lower_rocker",
        )
        wiper.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(0.0, -0.050, 0.024)),
            material=satin_metal,
            name="lower_link_pin",
        )
        wiper.visual(
            mesh_from_geometry(
                _tapered_bar(0.480, 0.042, 0.016, 0.012, y0=0.020, z_center=0.025),
                f"{side}_tapered_arm",
            ),
            material=satin_graphite,
            name="tapered_arm",
        )
        wiper.visual(
            Box((0.015, 0.290, 0.006)),
            origin=Origin(xyz=(0.0, 0.245, 0.034)),
            material=anodized,
            name="satin_center_ridge",
        )
        wiper.visual(
            Box((0.100, 0.060, 0.018)),
            origin=Origin(xyz=(0.0, 0.528, 0.020)),
            material=satin_graphite,
            name="blade_adapter",
        )
        wiper.visual(
            Box((0.590, 0.016, 0.014)),
            origin=Origin(xyz=(0.0, 0.555, 0.012)),
            material=anodized,
            name="blade_carrier",
        )
        wiper.visual(
            Box((0.615, 0.010, 0.024)),
            origin=Origin(xyz=(0.0, 0.555, -0.006)),
            material=dark_rubber,
            name="blade_rubber",
        )
        for i, cx in enumerate((-0.235, -0.118, 0.118, 0.235)):
            wiper.visual(
                Box((0.035, 0.020, 0.010)),
                origin=Origin(xyz=(cx, 0.555, 0.0235)),
                material=satin_metal,
                name=f"carrier_claw_{i}",
            )
        wiper.visual(
            Box((0.110, 0.012, 0.010)),
            origin=Origin(xyz=(0.095 * mirror, 0.555, 0.021)),
            material=satin_metal,
            name="end_clip_0",
        )
        wiper.visual(
            Box((0.110, 0.012, 0.010)),
            origin=Origin(xyz=(-0.095 * mirror, 0.555, 0.021)),
            material=satin_metal,
            name="end_clip_1",
        )

    drive_rod = model.part("drive_rod")
    drive_rod.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.014, 0.002, 0.0), (-0.470, 0.050, 0.002)],
                radius=0.0045,
                samples_per_segment=8,
                radial_segments=18,
            ),
            "drive_rod_tube",
        ),
        material=satin_metal,
        name="rod_tube",
    )
    drive_rod.visual(
        mesh_from_geometry(TorusGeometry(radius=0.012, tube=0.0035), "drive_inner_eye"),
        material=satin_metal,
        name="inner_eye",
    )
    drive_rod.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.012, tube=0.0035).translate(-0.475, 0.050, 0.002),
            "drive_outer_eye",
        ),
        material=satin_metal,
        name="outer_eye",
    )

    tie_rod = model.part("tie_rod")
    tie_rod.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.014, 0.0, 0.0), (0.826, 0.0, 0.0)],
                radius=0.0042,
                samples_per_segment=6,
                radial_segments=18,
            ),
            "tie_rod_tube",
        ),
        material=satin_metal,
        name="rod_tube",
    )
    tie_rod.visual(
        mesh_from_geometry(TorusGeometry(radius=0.012, tube=0.0035), "tie_inner_eye"),
        material=satin_metal,
        name="inner_eye",
    )
    tie_rod.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.012, tube=0.0035).translate(0.840, 0.0, 0.0),
            "tie_outer_eye",
        ),
        material=satin_metal,
        name="outer_eye",
    )

    motor_joint = model.articulation(
        "motor_to_crank",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=motor_crank,
        origin=Origin(xyz=(0.0, -0.085, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=0.04, friction=0.01),
    )
    model.articulation(
        "crank_to_drive_rod",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_rod,
        origin=Origin(xyz=(0.062, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "cowl_to_left_wiper",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=left_wiper,
        origin=Origin(xyz=(-pivot_x, pivot_y, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.4, lower=-0.55, upper=0.55),
        mimic=Mimic(joint=motor_joint.name, multiplier=0.82, offset=0.0),
    )
    model.articulation(
        "cowl_to_right_wiper",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=right_wiper,
        origin=Origin(xyz=(pivot_x, pivot_y, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.4, lower=-0.55, upper=0.55),
        mimic=Mimic(joint=motor_joint.name, multiplier=0.82, offset=0.0),
    )
    model.articulation(
        "left_wiper_to_tie_rod",
        ArticulationType.REVOLUTE,
        parent=left_wiper,
        child=tie_rod,
        origin=Origin(xyz=(0.0, -0.050, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cowl = object_model.get_part("cowl_frame")
    left = object_model.get_part("left_wiper")
    right = object_model.get_part("right_wiper")
    drive = object_model.get_part("drive_rod")
    tie = object_model.get_part("tie_rod")
    motor = object_model.get_articulation("motor_to_crank")

    ctx.allow_overlap(
        cowl,
        left,
        elem_a="spindle_stub_0",
        elem_b="hub_cap",
        reason="The satin spindle is intentionally seated through the rotating hub cap.",
    )
    ctx.allow_overlap(
        cowl,
        right,
        elem_a="spindle_stub_1",
        elem_b="hub_cap",
        reason="The satin spindle is intentionally seated through the rotating hub cap.",
    )
    ctx.allow_overlap(
        drive,
        left,
        elem_a="outer_eye",
        elem_b="lower_link_pin",
        reason="The rod eye is intentionally captured around the wiper rocker pin.",
    )

    ctx.expect_within(
        cowl,
        left,
        axes="xy",
        inner_elem="spindle_stub_0",
        outer_elem="hub_cap",
        margin=0.001,
        name="left spindle is centered inside hub",
    )
    ctx.expect_within(
        cowl,
        right,
        axes="xy",
        inner_elem="spindle_stub_1",
        outer_elem="hub_cap",
        margin=0.001,
        name="right spindle is centered inside hub",
    )
    ctx.expect_gap(
        left,
        cowl,
        axis="z",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="hub_cap",
        negative_elem="upper_collar_0",
        name="left hub sits just above collar",
    )
    ctx.expect_gap(
        right,
        cowl,
        axis="z",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="hub_cap",
        negative_elem="upper_collar_1",
        name="right hub sits just above collar",
    )
    ctx.expect_overlap(
        drive,
        left,
        axes="xy",
        min_overlap=0.010,
        elem_a="outer_eye",
        elem_b="lower_link_pin",
        name="drive rod eye is centered on wiper rocker pin",
    )
    ctx.expect_within(
        left,
        drive,
        axes="xy",
        inner_elem="lower_link_pin",
        outer_elem="outer_eye",
        margin=0.004,
        name="drive rod eye retains the rocker pin",
    )
    ctx.expect_overlap(
        tie,
        right,
        axes="xy",
        min_overlap=0.010,
        elem_a="outer_eye",
        elem_b="lower_link_pin",
        name="tie rod reaches opposite rocker pin",
    )

    def _center_x(aabb):
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    rest_left = ctx.part_element_world_aabb(left, elem="blade_rubber")
    rest_right = ctx.part_element_world_aabb(right, elem="blade_rubber")
    with ctx.pose({motor: 0.45}):
        swept_left = ctx.part_element_world_aabb(left, elem="blade_rubber")
        swept_right = ctx.part_element_world_aabb(right, elem="blade_rubber")

    ctx.check(
        "motor drive sweeps both blade carriers",
        rest_left is not None
        and rest_right is not None
        and swept_left is not None
        and swept_right is not None
        and abs(_center_x(swept_left) - _center_x(rest_left)) > 0.08
        and abs(_center_x(swept_right) - _center_x(rest_right)) > 0.08,
        details=f"rest_left={rest_left}, swept_left={swept_left}, rest_right={rest_right}, swept_right={swept_right}",
    )

    return ctx.report()


object_model = build_object_model()
