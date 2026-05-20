from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cylinder_between(part, name, start, end, radius, material, *, segments_note=""):
    """Add a URDF cylinder whose local +Z axis runs from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError(f"zero-length tube {name}")

    ux, uy, uz = vx / length, vy / length, vz / length
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="urban_commuter_fork")

    paint = Material("deep_green_powdercoat", rgba=(0.05, 0.26, 0.18, 1.0))
    dark_paint = Material("black_rack_paint", rgba=(0.015, 0.017, 0.016, 1.0))
    brushed = Material("brushed_steel", rgba=(0.62, 0.64, 0.60, 1.0))
    dark_metal = Material("dark_bolt_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = Material("brazed_boss_bronze", rgba=(0.78, 0.47, 0.19, 1.0))

    # Static bicycle head tube/bearing reference.  The fork/stem/bar assembly
    # rotates inside its hollow bore about the common steerer axis.
    head_tube = model.part("head_tube")
    head_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.045, 0.075),
            (0.039, 0.095),
            (0.037, 0.165),
            (0.037, 0.430),
            (0.040, 0.475),
            (0.046, 0.500),
        ],
        inner_profile=[
            (0.024, 0.075),
            (0.024, 0.500),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    head_tube.visual(
        mesh_from_geometry(head_shell, "head_tube_shell"),
        material=dark_metal,
        name="head_tube_shell",
    )

    steering = model.part("steering")

    # Threaded fork steerer and the quill stem seated in it.
    steering.visual(
        Cylinder(radius=0.0135, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=brushed,
        name="steerer_core",
    )
    for i in range(11):
        steering.visual(
            Cylinder(radius=0.0165, length=0.0025),
            origin=Origin(xyz=(0.0, 0.0, 0.445 + i * 0.012)),
            material=brushed,
            name=f"thread_ridge_{i}",
        )
    for name, z in (("lower_bearing_race", 0.067), ("upper_bearing_race", 0.508)):
        steering.visual(
            Cylinder(radius=0.040, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_metal,
            name=name,
        )
    steering.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=dark_metal,
        name="threaded_locknut",
    )
    steering.visual(
        Cylinder(radius=0.0125, length=0.27),
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material=brushed,
        name="quill_tube",
    )
    steering.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=brushed,
        name="quill_expander_plug",
    )

    _cylinder_between(
        steering,
        "quill_extension",
        (0.0, 0.000, 0.770),
        (0.0, -0.095, 0.875),
        0.014,
        brushed,
    )

    # Riser bar: one continuous bent tube, clamped by the quill stem.
    handlebar_mesh = tube_from_spline_points(
        [
            (-0.380, -0.155, 0.980),
            (-0.290, -0.155, 0.980),
            (-0.205, -0.135, 0.955),
            (-0.110, -0.095, 0.890),
            (0.000, -0.090, 0.875),
            (0.110, -0.095, 0.890),
            (0.205, -0.135, 0.955),
            (0.290, -0.155, 0.980),
            (0.380, -0.155, 0.980),
        ],
        radius=0.011,
        samples_per_segment=12,
        radial_segments=20,
        cap_ends=True,
    )
    steering.visual(
        mesh_from_geometry(handlebar_mesh, "riser_handlebar"),
        material=brushed,
        name="riser_handlebar",
    )
    steering.visual(
        Cylinder(radius=0.025, length=0.090),
        origin=Origin(xyz=(0.0, -0.090, 0.875), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="bar_clamp_sleeve",
    )
    for side, x in (("0", -0.055), ("1", 0.055)):
        steering.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(xyz=(x, -0.090, 0.895), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"side_bolt_{side}",
        )
    for side, x in (("0", -0.340), ("1", 0.340)):
        steering.visual(
            Cylinder(radius=0.017, length=0.095),
            origin=Origin(xyz=(x, -0.155, 0.980), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"grip_{side}",
        )

    # Brazed crown and straight fork blades.
    steering.visual(
        Cylinder(radius=0.035, length=0.165),
        origin=Origin(xyz=(0.0, 0.015, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paint,
        name="brazed_crown",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.010, 0.015)),
        material=paint,
        name="crown_steerer_socket",
    )
    blade_top = {
        "0": (-0.055, 0.018, -0.030),
        "1": (0.055, 0.018, -0.030),
    }
    blade_bottom = {
        "0": (-0.066, 0.075, -0.648),
        "1": (0.066, 0.075, -0.648),
    }
    for side in ("0", "1"):
        _cylinder_between(
            steering,
            f"blade_{side}",
            blade_top[side],
            blade_bottom[side],
            0.012,
            paint,
        )
        steering.visual(
            Cylinder(radius=0.017, length=0.032),
            origin=Origin(
                xyz=(blade_top[side][0], blade_top[side][1], -0.040),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"brazed_socket_{side}",
        )

    # Solid box dropouts clasping the hub axle.
    for side, x in (("0", -0.082), ("1", 0.082)):
        steering.visual(
            Box((0.034, 0.060, 0.080)),
            origin=Origin(xyz=(x, 0.078, -0.674)),
            material=paint,
            name=f"dropout_{side}",
        )
        steering.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x, 0.078, -0.674), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"axle_nut_{side}",
        )
    steering.visual(
        Cylinder(radius=0.007, length=0.225),
        origin=Origin(xyz=(0.0, 0.078, -0.674), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_axle",
    )
    steering.visual(
        Cylinder(radius=0.020, length=0.110),
        origin=Origin(xyz=(0.0, 0.078, -0.674), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="front_hub_shell",
    )

    # Low-rider rack: mid-blade brazed mounts, four standoffs, and a flat deck.
    steering.visual(
        Box((0.345, 0.215, 0.012)),
        origin=Origin(xyz=(0.0, 0.235, -0.305)),
        material=dark_paint,
        name="rack_platform",
    )
    for side, sx in (("0", -0.118), ("1", 0.118)):
        _cylinder_between(
            steering,
            f"rack_side_rail_{side}",
            (sx, 0.130, -0.293),
            (sx, 0.340, -0.293),
            0.006,
            dark_paint,
        )
    _cylinder_between(
        steering,
        "rack_front_rail",
        (-0.145, 0.340, -0.293),
        (0.145, 0.340, -0.293),
        0.006,
        dark_paint,
    )
    _cylinder_between(
        steering,
        "rack_rear_rail",
        (-0.145, 0.130, -0.293),
        (0.145, 0.130, -0.293),
        0.006,
        dark_paint,
    )

    mount_points = {
        "0": (-0.060, 0.050, -0.360),
        "1": (0.060, 0.050, -0.360),
    }
    platform_points = {
        "0a": (-0.140, 0.155, -0.302),
        "0b": (-0.140, 0.305, -0.302),
        "1a": (0.140, 0.155, -0.302),
        "1b": (0.140, 0.305, -0.302),
    }
    for side in ("0", "1"):
        mx, my, mz = mount_points[side]
        steering.visual(
            Cylinder(radius=0.012, length=0.026),
            origin=Origin(xyz=(mx, my, mz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"mid_blade_boss_{side}",
        )
        for suffix in ("a", "b"):
            _cylinder_between(
                steering,
                f"rack_standoff_{side}{suffix}",
                (mx, my, mz),
                platform_points[f"{side}{suffix}"],
                0.006,
                dark_paint,
            )

    model.articulation(
        "steerer_rotation",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steering = object_model.get_part("steering")
    joint = object_model.get_articulation("steerer_rotation")

    ctx.expect_origin_distance(
        head_tube,
        steering,
        axes="xy",
        max_dist=0.001,
        name="steerer axis is concentric with head tube",
    )
    ctx.expect_within(
        steering,
        head_tube,
        axes="xy",
        inner_elem="steerer_core",
        outer_elem="head_tube_shell",
        margin=0.0005,
        name="threaded steerer runs through head tube bore",
    )
    ctx.expect_overlap(
        steering,
        head_tube,
        axes="z",
        elem_a="steerer_core",
        elem_b="head_tube_shell",
        min_overlap=0.25,
        name="steerer has bearing-length insertion",
    )

    rest_aabb = ctx.part_element_world_aabb(steering, elem="hub_axle")
    with ctx.pose({joint: 0.80}):
        turned_aabb = ctx.part_element_world_aabb(steering, elem="hub_axle")
    if rest_aabb is not None and turned_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
        turned_center_x = (turned_aabb[0][0] + turned_aabb[1][0]) * 0.5
        turned_ok = abs(turned_center_x - rest_center_x) > 0.035
    else:
        turned_ok = False
        rest_center_x = None
        turned_center_x = None
    ctx.check(
        "front axle follows steering rotation",
        turned_ok,
        details=f"rest_x={rest_center_x}, turned_x={turned_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
