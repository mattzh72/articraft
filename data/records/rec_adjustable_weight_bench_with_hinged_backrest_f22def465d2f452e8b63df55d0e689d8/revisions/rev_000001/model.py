from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")

    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(horizontal, dz)
    yaw = math.atan2(dy, dx) if horizontal > 1.0e-9 else 0.0
    origin = Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_tube(part, name: str, start: tuple[float, float, float], end: tuple[float, float, float], radius: float, material):
    origin, length = _tube_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _rounded_pad_mesh(length: float, width: float, thickness: float, corner_radius: float, edge_radius: float, name: str):
    pad = cq.Workplane("XY").box(length, width, thickness)
    pad = pad.edges("|Z").fillet(corner_radius)
    pad = pad.edges().fillet(edge_radius)
    return mesh_from_cadquery(pad, name, tolerance=0.0015, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_weight_bench")

    steel = model.material("powder_coated_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    satin_steel = model.material("satin_steel_pins", rgba=(0.55, 0.57, 0.58, 1.0))
    vinyl = model.material("black_vinyl_padding", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    hub_gray = model.material("dark_wheel_hub", rgba=(0.18, 0.18, 0.17, 1.0))
    endcap = model.material("soft_rubber_endcaps", rgba=(0.02, 0.02, 0.02, 1.0))

    frame = model.part("base_frame")

    # Welded home-gym steel frame: parallel top rails, crossmembers, ground feet,
    # and diagonal legs all intersect slightly as welded tube construction.
    for y in (-0.18, 0.18):
        _add_tube(frame, f"top_rail_{'0' if y < 0 else '1'}", (-0.92, y, 0.405), (0.68, y, 0.405), 0.024, steel)
        _add_tube(frame, f"front_leg_{'0' if y < 0 else '1'}", (0.55, y, 0.385), (0.84, y, 0.075), 0.024, steel)
        _add_tube(frame, f"rear_leg_{'0' if y < 0 else '1'}", (-0.78, y, 0.385), (-1.02, y, 0.060), 0.024, steel)
        _add_tube(frame, f"center_strut_{'0' if y < 0 else '1'}", (-0.15, y, 0.385), (0.24, y, 0.225), 0.018, steel)

    for x, z, nm in [(-0.86, 0.405, "rear_top_cross"), (-0.20, 0.405, "back_hinge_cross"), (0.30, 0.405, "seat_support_cross"), (0.66, 0.405, "front_top_cross")]:
        _add_tube(frame, nm, (x, -0.245, z), (x, 0.245, z), 0.022, steel)

    frame.visual(Box((0.105, 0.76, 0.055)), origin=Origin(xyz=(-1.03, 0.0, 0.030)), material=steel, name="rear_floor_foot")
    frame.visual(Box((0.105, 0.72, 0.055)), origin=Origin(xyz=(0.85, 0.0, 0.032)), material=steel, name="front_floor_foot")
    frame.visual(Box((0.095, 0.80, 0.018)), origin=Origin(xyz=(-1.03, 0.0, 0.006)), material=endcap, name="rear_foot_sleeve")
    frame.visual(Box((0.095, 0.76, 0.018)), origin=Origin(xyz=(0.85, 0.0, 0.008)), material=endcap, name="front_foot_sleeve")
    _add_tube(frame, "front_axle", (0.86, -0.405, 0.086), (0.86, 0.405, 0.086), 0.014, satin_steel)
    frame.visual(Cylinder(radius=0.026, length=0.021), origin=Origin(xyz=(0.86, -0.392, 0.086), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="wheel_collar_0")
    frame.visual(Cylinder(radius=0.026, length=0.021), origin=Origin(xyz=(0.86, 0.392, 0.086), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="wheel_collar_1")

    # Hinge yokes are part of the welded frame; the moving hinge barrels sit
    # between them without occupying the same volume.
    frame.visual(Box((0.052, 0.032, 0.118)), origin=Origin(xyz=(-0.20, -0.205, 0.462)), material=steel, name="backrest_yoke_0")
    frame.visual(Box((0.052, 0.032, 0.118)), origin=Origin(xyz=(-0.20, 0.205, 0.462)), material=steel, name="backrest_yoke_1")
    frame.visual(Box((0.052, 0.032, 0.102)), origin=Origin(xyz=(0.47, -0.205, 0.462)), material=steel, name="seat_yoke_0")
    frame.visual(Box((0.052, 0.032, 0.102)), origin=Origin(xyz=(0.47, 0.205, 0.462)), material=steel, name="seat_yoke_1")
    frame.visual(Box((0.052, 0.028, 0.042)), origin=Origin(xyz=(-0.20, -0.175, 0.462)), material=satin_steel, name="backrest_inner_spacer_0")
    frame.visual(Box((0.052, 0.028, 0.042)), origin=Origin(xyz=(-0.20, 0.175, 0.462)), material=satin_steel, name="backrest_inner_spacer_1")
    frame.visual(Box((0.052, 0.028, 0.038)), origin=Origin(xyz=(0.47, -0.175, 0.462)), material=satin_steel, name="seat_inner_spacer_0")
    frame.visual(Box((0.052, 0.028, 0.038)), origin=Origin(xyz=(0.47, 0.175, 0.462)), material=satin_steel, name="seat_inner_spacer_1")
    frame.visual(Box((0.052, 0.032, 0.112)), origin=Origin(xyz=(-0.93, -0.175, 0.088)), material=steel, name="ladder_pivot_yoke_0")
    frame.visual(Box((0.052, 0.032, 0.112)), origin=Origin(xyz=(-0.93, 0.175, 0.088)), material=steel, name="ladder_pivot_yoke_1")

    # Rotating backrest: child frame is the transverse hinge axis.  The cushion
    # extends along local -X, so positive rotation about +Y raises the rear edge.
    backrest = model.part("backrest")
    backrest.visual(
        _rounded_pad_mesh(0.96, 0.325, 0.075, 0.045, 0.014, "backrest_pad"),
        origin=Origin(xyz=(-0.480, 0.0, 0.058)),
        material=vinyl,
        name="backrest_pad",
    )
    backrest.visual(Cylinder(radius=0.026, length=0.322), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="backrest_hinge_barrel")
    for y in (-0.105, 0.105):
        _add_tube(backrest, f"backrest_under_rail_{'0' if y < 0 else '1'}", (-0.030, y, 0.010), (-0.835, y, 0.020), 0.014, steel)
        backrest.visual(Box((0.105, 0.030, 0.026)), origin=Origin(xyz=(-0.055, y, 0.012)), material=steel, name=f"backrest_hinge_tab_{'0' if y < 0 else '1'}")
    _add_tube(backrest, "backrest_rear_crossstrap", (-0.760, -0.132, 0.020), (-0.760, 0.132, 0.020), 0.012, steel)

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.20, 0.0, 0.482)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.3, lower=0.0, upper=1.12),
        motion_properties=MotionProperties(damping=0.25, friction=0.10),
    )

    # Smaller independent seat pad hinged at its front edge.
    seat = model.part("seat")
    seat.visual(
        _rounded_pad_mesh(0.44, 0.305, 0.070, 0.040, 0.012, "seat_pad"),
        origin=Origin(xyz=(-0.220, 0.0, 0.054)),
        material=vinyl,
        name="seat_pad",
    )
    seat.visual(Cylinder(radius=0.023, length=0.322), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="seat_hinge_barrel")
    for y in (-0.100, 0.100):
        _add_tube(seat, f"seat_under_rail_{'0' if y < 0 else '1'}", (-0.020, y, 0.006), (-0.365, y, 0.016), 0.012, steel)
    _add_tube(seat, "seat_rear_crossstrap", (-0.342, -0.120, 0.016), (-0.342, 0.120, 0.016), 0.010, steel)

    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.47, 0.0, 0.482)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.42),
        motion_properties=MotionProperties(damping=0.18, friction=0.08),
    )

    # Rear angle-support ladder: a U-shaped notched steel ladder hinged at the
    # low rear frame so it can swing up under the backrest to select an incline.
    ladder = model.part("support_ladder")
    ladder.visual(Cylinder(radius=0.019, length=0.318), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="ladder_pivot_barrel")
    for y in (-0.145, 0.145):
        _add_tube(ladder, f"ladder_side_rail_{'0' if y < 0 else '1'}", (0.000, y, 0.000), (0.610, y, 0.190), 0.014, steel)
        ladder.visual(Box((0.040, 0.026, 0.030)), origin=Origin(xyz=(0.055, y, 0.017)), material=steel, name=f"ladder_pivot_lug_{'0' if y < 0 else '1'}")

    for idx, t in enumerate((0.22, 0.38, 0.54, 0.70, 0.86)):
        x = 0.610 * t
        z = 0.190 * t + 0.012
        _add_tube(ladder, f"ladder_rung_{idx}", (x, -0.165, z), (x, 0.165, z), 0.011, satin_steel)
        ladder.visual(Box((0.050, 0.060, 0.030)), origin=Origin(xyz=(x + 0.005, 0.0, z + 0.025)), material=steel, name=f"ladder_notch_{idx}")

    model.articulation(
        "frame_to_ladder",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ladder,
        origin=Origin(xyz=(-0.93, 0.0, 0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=0.95),
        motion_properties=MotionProperties(damping=0.20, friction=0.12),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.050,
            0.035,
            rim=WheelRim(inner_radius=0.032, flange_height=0.004, flange_thickness=0.0025, bead_seat_depth=0.002),
            hub=WheelHub(radius=0.017, width=0.030, cap_style="flat", bolt_pattern=BoltPattern(count=5, circle_diameter=0.022, hole_diameter=0.003)),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0035, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.032),
        ),
        "transport_wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.043,
            inner_radius=0.050,
            tread=TireTread(style="ribbed", depth=0.003, count=18, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.005, radius=0.0025),
        ),
        "transport_wheel_tire",
    )

    for idx, y in enumerate((-0.420, 0.420)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=hub_gray, name="hub")
        model.articulation(
            f"front_axle_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.86, y, 0.086), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("base_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    ladder = object_model.get_part("support_ladder")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    backrest_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    ladder_hinge = object_model.get_articulation("frame_to_ladder")
    wheel_spin = object_model.get_articulation("front_axle_to_wheel_0")

    ctx.expect_contact(backrest, frame, elem_a="backrest_hinge_barrel", elem_b="backrest_inner_spacer_1", contact_tol=0.0015, name="backrest hinge barrel seats between frame spacers")
    ctx.expect_contact(seat, frame, elem_a="seat_hinge_barrel", elem_b="seat_inner_spacer_1", contact_tol=0.0015, name="seat hinge barrel seats between frame spacers")
    ctx.expect_contact(ladder, frame, elem_a="ladder_pivot_barrel", elem_b="ladder_pivot_yoke_1", contact_tol=0.0015, name="ladder pivot barrel seats in the rear yoke")
    ctx.expect_origin_gap(wheel_1, wheel_0, axis="y", min_gap=0.80, max_gap=0.90, name="transport wheels are on opposite sides of the front axle")

    backrest_closed = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
    with ctx.pose({backrest_hinge: 1.0}):
        backrest_raised = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
    ctx.check(
        "backrest hinge raises the rear pad",
        backrest_closed is not None and backrest_raised is not None and backrest_raised[1][2] > backrest_closed[1][2] + 0.35,
        details=f"closed={backrest_closed}, raised={backrest_raised}",
    )

    seat_closed = ctx.part_element_world_aabb(seat, elem="seat_pad")
    with ctx.pose({seat_hinge: 0.35}):
        seat_raised = ctx.part_element_world_aabb(seat, elem="seat_pad")
    ctx.check(
        "seat front hinge tilts the seat pad upward",
        seat_closed is not None and seat_raised is not None and seat_raised[1][2] > seat_closed[1][2] + 0.04,
        details=f"closed={seat_closed}, raised={seat_raised}",
    )

    ladder_folded = ctx.part_element_world_aabb(ladder, elem="ladder_rung_4")
    with ctx.pose({ladder_hinge: 0.8}):
        ladder_raised = ctx.part_element_world_aabb(ladder, elem="ladder_rung_4")
    ctx.check(
        "rear support ladder swings upward from its lower pivot",
        ladder_folded is not None and ladder_raised is not None and ladder_raised[1][2] > ladder_folded[1][2] + 0.20,
        details=f"folded={ladder_folded}, raised={ladder_raised}",
    )

    wheel_rest = ctx.part_world_position(wheel_0)
    with ctx.pose({wheel_spin: math.pi}):
        wheel_rotated = ctx.part_world_position(wheel_0)
    ctx.check(
        "transport wheel spins about a fixed axle center",
        wheel_rest is not None
        and wheel_rotated is not None
        and max(abs(a - b) for a, b in zip(wheel_rest, wheel_rotated)) < 1.0e-6,
        details=f"rest={wheel_rest}, rotated={wheel_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
