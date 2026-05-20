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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_between(part, name, p1, p2, radius, material):
    """Add a cylinder whose local +Z axis runs between two points."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length tube {name}")
    mid = ((p1[0] + p2[0]) * 0.5, (p1[1] + p2[1]) * 0.5, (p1[2] + p2[2]) * 0.5)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _rounded_pad_mesh(length, width, thickness, name):
    pad = cq.Workplane("XY").box(length, width, thickness)
    pad = pad.edges("|Z").fillet(min(0.035, width * 0.12, length * 0.08))
    return mesh_from_cadquery(pad, name, tolerance=0.0015, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    steel = model.material("satin_black_powder_coat", rgba=(0.015, 0.017, 0.018, 1.0))
    vinyl = model.material("dark_textured_vinyl", rgba=(0.055, 0.058, 0.060, 1.0))
    seam = model.material("slightly_raised_grey_stitching", rgba=(0.22, 0.22, 0.21, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    alloy = model.material("brushed_dark_alloy", rgba=(0.35, 0.36, 0.34, 1.0))

    frame = model.part("frame")

    # Low apartment-scale tubular chassis: parallel floor rails, crossmembers,
    # central spine, and compact hinge towers for the split pads.
    _tube_between(frame, "base_rail_0", (-0.62, -0.18, 0.095), (0.62, -0.18, 0.095), 0.020, steel)
    _tube_between(frame, "base_rail_1", (-0.62, 0.18, 0.095), (0.62, 0.18, 0.095), 0.020, steel)
    _tube_between(frame, "front_crossmember", (-0.62, -0.27, 0.095), (-0.62, 0.27, 0.095), 0.022, steel)
    _tube_between(frame, "rear_crossmember", (0.58, -0.25, 0.095), (0.58, 0.25, 0.095), 0.022, steel)
    _tube_between(frame, "center_spine", (-0.40, 0.0, 0.310), (0.46, 0.0, 0.310), 0.018, steel)
    _tube_between(frame, "front_diag_0", (-0.48, -0.18, 0.095), (-0.34, 0.0, 0.310), 0.014, steel)
    _tube_between(frame, "front_diag_1", (-0.48, 0.18, 0.095), (-0.34, 0.0, 0.310), 0.014, steel)
    _tube_between(frame, "rear_diag_0", (0.44, -0.18, 0.095), (0.34, -0.18, 0.310), 0.014, steel)
    _tube_between(frame, "rear_diag_1", (0.44, 0.18, 0.095), (0.34, 0.18, 0.310), 0.014, steel)
    _tube_between(frame, "rear_upper_cross", (0.34, -0.18, 0.310), (0.34, 0.18, 0.310), 0.012, steel)

    # Hinge towers sit below the pad hinge axes so the moving hinge barrels are
    # visibly captured by side ears without occupying the same solid volume.
    for x, z, prefix in ((-0.34, 0.390, "seat_hinge"), (0.08, 0.400, "back_hinge")):
        _tube_between(frame, f"{prefix}_post", (x, 0.0, 0.310), (x, 0.0, z - 0.055), 0.013, steel)
        _tube_between(frame, f"{prefix}_cross", (x, -0.21, z - 0.055), (x, 0.21, z - 0.055), 0.012, steel)
        frame.visual(
            Box((0.056, 0.036, 0.072)),
            origin=Origin(xyz=(x, -0.205, z - 0.015)),
            material=steel,
            name=f"{prefix}_ear_0",
        )
        frame.visual(
            Box((0.056, 0.036, 0.072)),
            origin=Origin(xyz=(x, 0.205, z - 0.015)),
            material=steel,
            name=f"{prefix}_ear_1",
        )

    # Lower pivot and front axle hardware.
    frame.visual(
        Box((0.090, 0.380, 0.020)),
        origin=Origin(xyz=(0.32, 0.0, 0.092)),
        material=steel,
        name="support_pivot_plate",
    )
    frame.visual(
        Box((0.052, 0.032, 0.060)),
        origin=Origin(xyz=(0.32, -0.108, 0.128)),
        material=steel,
        name="support_pivot_ear_0",
    )
    frame.visual(
        Box((0.052, 0.032, 0.060)),
        origin=Origin(xyz=(0.32, 0.108, 0.128)),
        material=steel,
        name="support_pivot_ear_1",
    )
    _tube_between(frame, "front_axle", (-0.62, -0.255, 0.035), (-0.62, 0.255, 0.035), 0.006, alloy)
    _tube_between(frame, "axle_drop_0", (-0.62, -0.18, 0.075), (-0.62, -0.18, 0.035), 0.007, steel)
    _tube_between(frame, "axle_drop_1", (-0.62, 0.18, 0.075), (-0.62, 0.18, 0.035), 0.007, steel)
    _tube_between(frame, "rear_foot_drop_0", (0.58, -0.22, 0.075), (0.58, -0.22, 0.032), 0.011, steel)
    _tube_between(frame, "rear_foot_drop_1", (0.58, 0.22, 0.075), (0.58, 0.22, 0.032), 0.011, steel)
    frame.visual(Box((0.120, 0.055, 0.025)), origin=Origin(xyz=(0.58, -0.22, 0.0195)), material=rubber, name="rear_foot_0")
    frame.visual(Box((0.120, 0.055, 0.025)), origin=Origin(xyz=(0.58, 0.22, 0.0195)), material=rubber, name="rear_foot_1")

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_pad_mesh(0.78, 0.34, 0.065, "backrest_cushion_mesh"),
        origin=Origin(xyz=(0.390, 0.0, 0.050)),
        material=vinyl,
        name="cushion",
    )
    _tube_between(backrest, "support_rail_0", (0.035, -0.115, 0.014), (0.730, -0.115, 0.014), 0.010, steel)
    _tube_between(backrest, "support_rail_1", (0.035, 0.115, 0.014), (0.730, 0.115, 0.014), 0.010, steel)
    _tube_between(backrest, "hinge_barrel", (0.0, -0.187, 0.0), (0.0, 0.187, 0.0), 0.018, steel)
    backrest.visual(Box((0.660, 0.007, 0.004)), origin=Origin(xyz=(0.405, -0.154, 0.083)), material=seam, name="side_seam_0")
    backrest.visual(Box((0.660, 0.007, 0.004)), origin=Origin(xyz=(0.405, 0.154, 0.083)), material=seam, name="side_seam_1")

    seat = model.part("seat")
    seat.visual(
        _rounded_pad_mesh(0.38, 0.34, 0.065, "seat_cushion_mesh"),
        origin=Origin(xyz=(0.190, 0.0, 0.050)),
        material=vinyl,
        name="cushion",
    )
    _tube_between(seat, "support_rail_0", (0.030, -0.105, 0.014), (0.345, -0.105, 0.014), 0.010, steel)
    _tube_between(seat, "support_rail_1", (0.030, 0.105, 0.014), (0.345, 0.105, 0.014), 0.010, steel)
    _tube_between(seat, "hinge_barrel", (0.0, -0.187, 0.0), (0.0, 0.187, 0.0), 0.018, steel)
    seat.visual(Box((0.285, 0.007, 0.004)), origin=Origin(xyz=(0.205, -0.154, 0.083)), material=seam, name="side_seam_0")
    seat.visual(Box((0.285, 0.007, 0.004)), origin=Origin(xyz=(0.205, 0.154, 0.083)), material=seam, name="side_seam_1")

    support_link = model.part("support_link")
    _tube_between(support_link, "side_tube_0", (0.0, -0.070, 0.012), (0.0, -0.070, 0.292), 0.012, steel)
    _tube_between(support_link, "side_tube_1", (0.0, 0.070, 0.012), (0.0, 0.070, 0.292), 0.012, steel)
    _tube_between(support_link, "lower_bushing", (0.0, -0.092, 0.0), (0.0, 0.092, 0.0), 0.018, steel)
    _tube_between(support_link, "top_roller", (0.0, -0.095, 0.300), (0.0, 0.095, 0.300), 0.010, rubber)
    _tube_between(support_link, "middle_spacer", (0.0, -0.075, 0.150), (0.0, 0.075, 0.150), 0.009, steel)

    wheel_geom = WheelGeometry(
        0.025,
        0.032,
        rim=WheelRim(inner_radius=0.018, flange_height=0.003, flange_thickness=0.002, bead_seat_depth=0.002),
        hub=WheelHub(radius=0.012, width=0.024, cap_style="domed"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        bore=WheelBore(style="round", diameter=0.016),
    )
    tire_geom = TireGeometry(
        0.035,
        0.034,
        inner_radius=0.025,
        tread=TireTread(style="block", depth=0.0025, count=14, land_ratio=0.58),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.003, radius=0.002),
    )
    wheel_mesh = mesh_from_geometry(wheel_geom, "transport_wheel_rim")
    tire_mesh = mesh_from_geometry(tire_geom, "transport_wheel_tire")
    for index, y in enumerate((-0.225, 0.225)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=alloy, name="rim")
        wheel.visual(
            Cylinder(radius=0.0085, length=0.042),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=alloy,
            name="axle_sleeve",
        )
        model.articulation(
            f"frame_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.62, y, 0.035), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=40.0),
        )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.08, 0.0, 0.400)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.34, 0.0, 0.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=0.0, upper=0.55),
    )
    model.articulation(
        "frame_to_support_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_link,
        origin=Origin(xyz=(0.32, 0.0, 0.130), rpy=(0.0, 0.50, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-0.35, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_link = object_model.get_part("support_link")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    back_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    support_hinge = object_model.get_articulation("frame_to_support_link")
    wheel_hinge_0 = object_model.get_articulation("frame_to_wheel_0")
    wheel_hinge_1 = object_model.get_articulation("frame_to_wheel_1")

    ctx.allow_overlap(
        frame,
        wheel_0,
        elem_a="front_axle",
        elem_b="axle_sleeve",
        reason="The solid axle is intentionally captured inside the wheel sleeve proxy so the transport wheel reads mounted while spinning.",
    )
    ctx.allow_overlap(
        frame,
        wheel_1,
        elem_a="front_axle",
        elem_b="axle_sleeve",
        reason="The solid axle is intentionally captured inside the wheel sleeve proxy so the transport wheel reads mounted while spinning.",
    )
    ctx.expect_overlap(
        frame,
        wheel_0,
        axes="y",
        min_overlap=0.035,
        elem_a="front_axle",
        elem_b="axle_sleeve",
        name="wheel_0 sleeve is retained on axle",
    )
    ctx.expect_overlap(
        frame,
        wheel_1,
        axes="y",
        min_overlap=0.035,
        elem_a="front_axle",
        elem_b="axle_sleeve",
        name="wheel_1 sleeve is retained on axle",
    )

    ctx.check(
        "transport wheels use continuous spin joints",
        wheel_hinge_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_hinge_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={wheel_hinge_0.articulation_type}, {wheel_hinge_1.articulation_type}",
    )

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        min_gap=0.015,
        max_gap=0.075,
        positive_elem="cushion",
        negative_elem="cushion",
        name="split pads keep a visible hinge gap",
    )
    ctx.expect_gap(
        frame,
        wheel_0,
        axis="z",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="front_crossmember",
        negative_elem="tire",
        name="wheel_0 tucked under front crossmember",
    )
    ctx.expect_gap(
        frame,
        wheel_1,
        axis="z",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="front_crossmember",
        negative_elem="tire",
        name="wheel_1 tucked under front crossmember",
    )

    back_rest_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.95}):
        back_raised_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest hinge raises the rear pad",
        back_rest_aabb is not None
        and back_raised_aabb is not None
        and back_raised_aabb[1][2] > back_rest_aabb[1][2] + 0.25,
        details=f"rest={back_rest_aabb}, raised={back_raised_aabb}",
    )

    seat_rest_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.45}):
        seat_raised_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat hinge raises the rear seat edge",
        seat_rest_aabb is not None
        and seat_raised_aabb is not None
        and seat_raised_aabb[1][2] > seat_rest_aabb[1][2] + 0.07,
        details=f"rest={seat_rest_aabb}, raised={seat_raised_aabb}",
    )

    link_rest_aabb = ctx.part_world_aabb(support_link)
    with ctx.pose({support_hinge: 0.45}):
        link_rotated_aabb = ctx.part_world_aabb(support_link)
    ctx.check(
        "rear support link swings on lower pivot",
        link_rest_aabb is not None
        and link_rotated_aabb is not None
        and link_rotated_aabb[1][0] > link_rest_aabb[1][0] + 0.035,
        details=f"rest={link_rest_aabb}, rotated={link_rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
