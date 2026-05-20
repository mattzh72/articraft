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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _bell_body_shape() -> cq.Workplane:
    """Lathe a hollow, flared alpine church bell in local metres."""
    wall_profile = [
        (0.225, -0.620),
        (0.214, -0.585),
        (0.165, -0.470),
        (0.112, -0.300),
        (0.081, -0.195),
        (0.045, -0.160),
        (0.000, -0.148),
        (0.000, -0.185),
        (0.050, -0.190),
        (0.075, -0.275),
        (0.128, -0.465),
        (0.183, -0.582),
        (0.194, -0.620),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(wall_profile)
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0), axisEnd=(0.0, 1.0))
    )

    # Subtle raised bands are part of the cast bronze/brass body, not separate
    # floating decorations.
    shoulder_band = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.096, -0.248),
                (0.116, -0.248),
                (0.118, -0.268),
                (0.099, -0.270),
            ]
        )
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0), axisEnd=(0.0, 1.0))
    )
    sound_bow_band = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.174, -0.505),
                (0.199, -0.505),
                (0.203, -0.535),
                (0.181, -0.538),
            ]
        )
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0), axisEnd=(0.0, 1.0))
    )
    return body.union(shoulder_band).union(sound_bow_band)


def _bearing_plate_shape() -> cq.Workplane:
    """A fixed iron cheek plate with a real clearance hole for the trunnion."""
    return (
        cq.Workplane("XY")
        .box(0.040, 0.120, 0.110)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .circle(0.023)
        .cutThruAll()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="alpine_wooden_bell_cote")

    wood = model.material("weathered_warm_timber", rgba=(0.50, 0.30, 0.15, 1.0))
    end_grain = model.material("darker_end_grain", rgba=(0.30, 0.17, 0.08, 1.0))
    roof_mat = model.material("dark_alpine_shingles", rgba=(0.16, 0.12, 0.09, 1.0))
    brass = model.material("aged_brass", rgba=(0.88, 0.62, 0.22, 1.0))
    iron = model.material("blackened_iron", rgba=(0.03, 0.03, 0.028, 1.0))

    frame = model.part("timber_frame")

    # Narrow rectangular open timber frame.
    for x, name in [(-0.350, "upright_0"), (0.350, "upright_1")]:
        frame.visual(
            Box((0.095, 0.200, 1.180)),
            origin=Origin(xyz=(x, 0.0, 0.690)),
            material=wood,
            name=name,
        )

    frame.visual(
        Box((0.800, 0.210, 0.115)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=end_grain,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.800, 0.205, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 1.265)),
        material=wood,
        name="top_tie",
    )

    # Fixed pivot beam across the uprights; bearing cheeks are bolted to it.
    frame.visual(
        Box((0.700, 0.085, 0.085)),
        origin=Origin(xyz=(0.0, 0.100, 1.000)),
        material=wood,
        name="pivot_beam",
    )
    bearing_mesh = mesh_from_cadquery(
        _bearing_plate_shape(),
        "iron_bearing_plate",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    for x, name in [(-0.245, "bearing_plate_0"), (0.245, "bearing_plate_1")]:
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.005, 1.000)),
            material=iron,
            name=name,
        )

    # Small diagonal knee braces behind the bell reinforce the Alpine timber
    # silhouette while staying clear of the swinging brass body.
    for x, yaw, name in [
        (-0.242, -0.61, "brace_0"),
        (0.242, 0.61, "brace_1"),
    ]:
        frame.visual(
            Box((0.360, 0.060, 0.060)),
            origin=Origin(xyz=(x, 0.085, 1.135), rpy=(0.0, yaw, 0.0)),
            material=wood,
            name=name,
        )

    # Pointed roof with overhanging shingled boards and a round ridge cap.
    roof_angle = math.atan2(0.240, 0.430)
    frame.visual(
        Box((0.540, 0.500, 0.060)),
        origin=Origin(xyz=(-0.215, 0.0, 1.430), rpy=(0.0, -roof_angle, 0.0)),
        material=roof_mat,
        name="roof_plane_0",
    )
    frame.visual(
        Box((0.540, 0.500, 0.060)),
        origin=Origin(xyz=(0.215, 0.0, 1.430), rpy=(0.0, roof_angle, 0.0)),
        material=roof_mat,
        name="roof_plane_1",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 1.555), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=roof_mat,
        name="ridge_cap",
    )

    bell = model.part("bell")
    bell.visual(
        mesh_from_cadquery(
            _bell_body_shape(),
            "hollow_brass_bell",
            tolerance=0.0008,
            angular_tolerance=0.06,
        ),
        origin=Origin(),
        material=brass,
        name="bell_body",
    )
    bell.visual(
        Box((0.405, 0.080, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=wood,
        name="headstock",
    )
    bell.visual(
        Cylinder(radius=0.016, length=0.490),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="trunnion_pin",
    )
    for x, name in [(-0.070, "hanger_0"), (0.070, "hanger_1")]:
        bell.visual(
            Box((0.026, 0.024, 0.125)),
            origin=Origin(xyz=(x, 0.0, -0.120)),
            material=iron,
            name=name,
        )
    bell.visual(
        Cylinder(radius=0.007, length=0.345),
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
        material=iron,
        name="clapper_stem",
    )
    bell.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.555)),
        material=iron,
        name="clapper_ball",
    )

    model.articulation(
        "headstock_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, -0.050, 1.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.62, upper=0.62),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("timber_frame")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("headstock_hinge")

    ctx.check(
        "bell uses a revolute headstock hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={hinge.articulation_type}",
    )
    ctx.check(
        "headstock axis is horizontal across the frame",
        abs(hinge.axis[0]) > 0.99 and abs(hinge.axis[1]) < 1e-6 and abs(hinge.axis[2]) < 1e-6,
        details=f"axis={hinge.axis}",
    )
    for bearing_name in ("bearing_plate_0", "bearing_plate_1"):
        ctx.allow_overlap(
            frame,
            bell,
            elem_a=bearing_name,
            elem_b="trunnion_pin",
            reason="The iron cheek plate intentionally captures the bell trunnion pin at the headstock hinge.",
        )
        ctx.expect_overlap(
            frame,
            bell,
            axes="x",
            elem_a=bearing_name,
            elem_b="trunnion_pin",
            min_overlap=0.008,
            name=f"{bearing_name} captures the trunnion along the hinge axis",
        )
        ctx.expect_overlap(
            frame,
            bell,
            axes="yz",
            elem_a=bearing_name,
            elem_b="trunnion_pin",
            min_overlap=0.015,
            name=f"trunnion passes through {bearing_name}",
        )
    ctx.expect_within(
        bell,
        frame,
        axes="x",
        margin=0.010,
        name="bell sits between the frame uprights",
    )
    ctx.expect_overlap(
        frame,
        bell,
        axes="x",
        elem_a="pivot_beam",
        elem_b="headstock",
        min_overlap=0.35,
        name="fixed pivot beam spans the headstock",
    )

    pivot_aabb = ctx.part_element_world_aabb(frame, elem="pivot_beam")
    ctx.check(
        "pivot beam is a narrow cross timber",
        pivot_aabb is not None and (pivot_aabb[1][0] - pivot_aabb[0][0]) > 0.62,
        details=f"pivot_aabb={pivot_aabb}",
    )

    rest_aabb = ctx.part_world_aabb(bell)
    with ctx.pose({hinge: 0.45}):
        swung_aabb = ctx.part_world_aabb(bell)
    rest_y = None if rest_aabb is None else 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
    swung_y = None if swung_aabb is None else 0.5 * (swung_aabb[0][1] + swung_aabb[1][1])
    ctx.check(
        "bell swings about the headstock axis",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.10,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
