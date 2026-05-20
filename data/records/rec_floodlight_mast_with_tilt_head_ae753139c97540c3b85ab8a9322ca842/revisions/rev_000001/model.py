from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_X = 2.28
HINGE_Z = 12.00


def _annular_disk(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """A centered hollow disk/ring with its bore along local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -0.5 * thickness))
    )


def _lamp_housing() -> cq.Workplane:
    """One LED floodlight body with a deep rear heat-sink pack."""
    body = cq.Workplane("XY").box(0.50, 0.34, 0.17).translate((0.31, 0.0, 0.105))

    # Rear heat-sink fins, visible behind the floodlight housing.
    for y in (-0.135, -0.081, -0.027, 0.027, 0.081, 0.135):
        fin = cq.Workplane("XY").box(0.085, 0.012, 0.145).translate((0.035, y, 0.115))
        body = body.union(fin)

    # Stiffened trunnion ears on the sides of the housing, tied into the body.
    for y in (-0.168, 0.168):
        ear = cq.Workplane("XY").box(0.090, 0.040, 0.110).translate((0.205, y, 0.105))
        body = body.union(ear)

    return body.clean()


def _sloping_cylinder_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, Origin]:
    """Return length and origin for a cylinder whose local Z follows an X/Z strut."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # This model only uses the helper for X/Z plane braces.
    pitch_about_y = math.atan2(dx, dz)
    center = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return length, Origin(xyz=center, rpy=(0.0, pitch_about_y, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_floodlight_pole")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark = model.material("dark_powdercoat", rgba=(0.035, 0.040, 0.045, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.45, 0.43, 0.39, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.42, 0.62, 0.82, 0.62))
    led = model.material("warm_led_array", rgba=(1.0, 0.88, 0.38, 1.0))

    bushing_mesh = mesh_from_cadquery(_annular_disk(0.105, 0.058, 0.055), "tilt_bushing")
    azimuth_ring_mesh = mesh_from_cadquery(_annular_disk(0.140, 0.055, 0.025), "azimuth_ring")
    lamp_housing_mesh = mesh_from_cadquery(_lamp_housing(), "floodlight_housing")

    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=0.42, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=concrete,
        name="concrete_plinth",
    )
    pole.visual(
        Cylinder(radius=0.125, length=12.0),
        origin=Origin(xyz=(0.0, 0.0, 6.0)),
        material=galvanized,
        name="round_pole",
    )
    pole.visual(
        Cylinder(radius=0.105, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 12.015)),
        material=galvanized,
        name="top_cap",
    )
    pole.visual(
        Cylinder(radius=0.060, length=2.10),
        origin=Origin(xyz=(1.045, 0.0, 12.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="outreach_arm",
    )
    strut_len, strut_origin = _sloping_cylinder_origin((0.09, 0.0, 11.35), (1.60, 0.0, 11.96))
    pole.visual(
        Cylinder(radius=0.035, length=strut_len),
        origin=strut_origin,
        material=galvanized,
        name="diagonal_strut",
    )
    pole.visual(
        Box((0.160, 0.620, 0.105)),
        origin=Origin(xyz=(2.115, 0.0, 12.0)),
        material=galvanized,
        name="outer_bracket",
    )
    for side, y in enumerate((-0.275, 0.275)):
        pole.visual(
            Box((0.140, 0.060, 0.150)),
            origin=Origin(xyz=(2.230, y, 12.155)),
            material=galvanized,
            name=f"upper_lug_{side}",
        )
        pole.visual(
            Box((0.140, 0.060, 0.150)),
            origin=Origin(xyz=(2.230, y, 11.845)),
            material=galvanized,
            name=f"lower_lug_{side}",
        )
        pole.visual(
            bushing_mesh,
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"hinge_bushing_{side}",
        )

    cluster = model.part("cluster_frame")
    cluster.visual(
        Cylinder(radius=0.060, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )
    cluster.visual(
        Box((0.280, 0.130, 0.140)),
        origin=Origin(xyz=(0.145, 0.0, -0.080)),
        material=galvanized,
        name="hinge_neck",
    )
    cluster.visual(
        Box((0.080, 1.280, 0.070)),
        origin=Origin(xyz=(0.250, 0.0, -0.180)),
        material=galvanized,
        name="top_rail",
    )
    cluster.visual(
        Box((0.080, 1.280, 0.070)),
        origin=Origin(xyz=(0.250, 0.0, -1.020)),
        material=galvanized,
        name="bottom_rail",
    )
    for side, y in enumerate((-0.600, 0.600)):
        cluster.visual(
            Box((0.080, 0.080, 0.860)),
            origin=Origin(xyz=(0.250, y, -0.600)),
            material=galvanized,
            name=f"side_rail_{side}",
        )
    cluster.visual(
        Box((0.060, 0.050, 0.860)),
        origin=Origin(xyz=(0.235, 0.0, -0.600)),
        material=galvanized,
        name="center_rail",
    )

    lamp_positions: list[tuple[int, int, float, float]] = [
        (0, 0, -0.320, -0.380),
        (0, 1, 0.320, -0.380),
        (1, 0, -0.320, -0.780),
        (1, 1, 0.320, -0.780),
    ]
    for row_z in (-0.380, -0.780):
        cluster.visual(
            Box((0.080, 1.160, 0.045)),
            origin=Origin(xyz=(0.225, 0.0, row_z)),
            material=galvanized,
            name=f"row_rail_{abs(int(row_z * 100)):02d}",
        )
    for row, col, y, z in lamp_positions:
        cluster.visual(
            Box((0.190, 0.180, 0.025)),
            origin=Origin(xyz=(0.355, y, z - 0.025)),
            material=galvanized,
            name=f"mount_{row}_{col}",
        )

    model.articulation(
        "cluster_tilt",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=cluster,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=0.35, lower=-0.45, upper=0.45),
    )

    for row, col, y, z in lamp_positions:
        lamp = model.part(f"lamp_{row}_{col}")
        lamp.visual(
            azimuth_ring_mesh,
            origin=Origin(),
            material=galvanized,
            name="azimuth_ring",
        )
        lamp.visual(
            Cylinder(radius=0.036, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, 0.0625)),
            material=galvanized,
            name="swivel_post",
        )
        lamp.visual(
            Cylinder(radius=0.075, length=0.015),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=galvanized,
            name="bearing_cap",
        )
        lamp.visual(
            Box((0.140, 0.110, 0.080)),
            origin=Origin(xyz=(0.055, 0.0, 0.085)),
            material=dark,
            name="rear_boss",
        )
        lamp.visual(
            lamp_housing_mesh,
            origin=Origin(),
            material=dark,
            name="housing",
        )
        lamp.visual(
            Box((0.014, 0.280, 0.105)),
            origin=Origin(xyz=(0.565, 0.0, 0.105)),
            material=glass,
            name="front_glass",
        )
        for iy, led_y in enumerate((-0.085, 0.0, 0.085)):
            for iz, led_z in enumerate((0.070, 0.140)):
                lamp.visual(
                    Box((0.006, 0.042, 0.030)),
                    origin=Origin(xyz=(0.573, led_y, led_z)),
                    material=led,
                    name=f"led_{iy}_{iz}",
                )

        model.articulation(
            f"lamp_azimuth_{row}_{col}",
            ArticulationType.REVOLUTE,
            parent=cluster,
            child=lamp,
            origin=Origin(xyz=(0.450, y, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.65, lower=-0.55, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cluster = object_model.get_part("cluster_frame")
    lamp = object_model.get_part("lamp_0_0")
    tilt = object_model.get_articulation("cluster_tilt")
    azimuth = object_model.get_articulation("lamp_azimuth_0_0")

    for side in (0, 1):
        ctx.allow_overlap(
            "pole",
            "cluster_frame",
            elem_a=f"hinge_bushing_{side}",
            elem_b="hinge_pin",
            reason="The tilt hinge pin is intentionally captured inside the arm-end steel bushing.",
        )
        ctx.expect_overlap(
            "pole",
            "cluster_frame",
            axes="y",
            elem_a=f"hinge_bushing_{side}",
            elem_b="hinge_pin",
            min_overlap=0.020,
            name=f"hinge pin remains inserted through bushing {side}",
        )

    ctx.check(
        "cluster uses a bounded tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.0,
    )
    lamp_joints = [j for j in object_model.articulations if j.name.startswith("lamp_azimuth_")]
    ctx.check(
        "four individual lamp azimuth joints",
        len(lamp_joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in lamp_joints),
        details=f"found {[j.name for j in lamp_joints]}",
    )
    ctx.expect_gap(
        lamp,
        cluster,
        axis="z",
        positive_elem="azimuth_ring",
        negative_elem="mount_0_0",
        min_gap=0.0,
        max_gap=0.001,
        name="azimuth ring sits just above its shelf",
    )

    rest_lamp_aabb = ctx.part_element_world_aabb(lamp, elem="housing")
    with ctx.pose({azimuth: 0.40}):
        turned_lamp_aabb = ctx.part_element_world_aabb(lamp, elem="housing")
    rest_lamp_y = None if rest_lamp_aabb is None else 0.5 * (rest_lamp_aabb[0][1] + rest_lamp_aabb[1][1])
    turned_lamp_y = None if turned_lamp_aabb is None else 0.5 * (turned_lamp_aabb[0][1] + turned_lamp_aabb[1][1])
    ctx.check(
        "lamp housing yaws on azimuth ring",
        rest_lamp_y is not None and turned_lamp_y is not None and abs(turned_lamp_y - rest_lamp_y) > 0.06,
        details=f"rest_y={rest_lamp_y}, turned_y={turned_lamp_y}",
    )

    rest_cluster_aabb = ctx.part_world_aabb(cluster)
    with ctx.pose({tilt: 0.35}):
        tilted_cluster_aabb = ctx.part_world_aabb(cluster)
    rest_cluster_min_z = None if rest_cluster_aabb is None else rest_cluster_aabb[0][2]
    tilted_cluster_min_z = None if tilted_cluster_aabb is None else tilted_cluster_aabb[0][2]
    ctx.check(
        "cluster frame pitches on arm-end hinge",
        rest_cluster_min_z is not None
        and tilted_cluster_min_z is not None
        and tilted_cluster_min_z > rest_cluster_min_z + 0.10,
        details=f"rest_min_z={rest_cluster_min_z}, tilted_min_z={tilted_cluster_min_z}",
    )

    return ctx.report()


object_model = build_object_model()
