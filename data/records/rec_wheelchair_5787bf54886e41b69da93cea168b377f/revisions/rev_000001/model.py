from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


TUBE_R = 0.012
SLENDER_TUBE_R = 0.008


def _cylinder_rpy_between(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    horiz = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if horiz > 1.0e-9 else 0.0
    pitch = math.atan2(horiz, dz)
    return (0.0, pitch, yaw)


def _cylinder_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    name: str,
    material: Material | str,
):
    length = math.dist(p0, p1)
    mid = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=_cylinder_rpy_between(p0, p1)),
        material=material,
        name=name,
    )


def _tube_spline(
    part,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    name: str,
    material: Material | str,
    samples: int = 12,
):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples,
        radial_segments=18,
        cap_ends=True,
    )
    part.visual(mesh_from_geometry(geom, f"{name}_mesh"), material=material, name=name)


def _add_wheel(
    part,
    *,
    mesh_prefix: str,
    tire_radius: float,
    tire_width: float,
    cap_side: float,
    rubber,
    rim_material,
    hub_material,
    spoke_count: int,
):
    inner_radius = tire_radius * 0.72
    tire = TireGeometry(
        tire_radius,
        tire_width,
        inner_radius=inner_radius,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="block", depth=tire_radius * 0.020, count=22, land_ratio=0.68),
        grooves=(TireGroove(center_offset=0.0, width=tire_width * 0.16, depth=tire_radius * 0.010),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=tire_width * 0.14, radius=tire_radius * 0.020),
    )
    part.visual(
        mesh_from_geometry(tire, f"{mesh_prefix}_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )

    wheel = WheelGeometry(
        inner_radius * 0.96,
        tire_width * 0.82,
        rim=WheelRim(
            inner_radius=inner_radius * 0.58,
            flange_height=tire_radius * 0.030,
            flange_thickness=tire_width * 0.08,
            bead_seat_depth=tire_radius * 0.018,
        ),
        hub=WheelHub(
            radius=tire_radius * 0.20,
            width=tire_width * 0.70,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=tire_radius * 0.26,
                hole_diameter=tire_radius * 0.028,
            ),
        ),
        face=WheelFace(dish_depth=tire_width * 0.12, front_inset=tire_width * 0.06, rear_inset=tire_width * 0.04),
        spokes=WheelSpokes(style="split_y", count=spoke_count, thickness=tire_radius * 0.018, window_radius=tire_radius * 0.07),
        bore=WheelBore(style="round", diameter=tire_radius * 0.075),
    )
    part.visual(
        mesh_from_geometry(wheel, f"{mesh_prefix}_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_material,
        name="rim",
    )

    # A raised outer hub cap gives the wheel an obvious medical-equipment caster / hub detail.
    _cylinder_between(
        part,
        (0.0, cap_side * tire_width * 0.22, 0.0),
        (0.0, cap_side * (tire_width * 0.38 + 0.014), 0.0),
        radius=tire_radius * 0.18,
        name="outer_cap",
        material=hub_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_transport_wheelchair")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_metal = model.material("dark_hinge_metal", rgba=(0.18, 0.19, 0.19, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.012, 0.012, 0.010, 1.0))
    fabric = model.material("wipe_clean_blue_fabric", rgba=(0.02, 0.12, 0.27, 1.0))
    black_plastic = model.material("black_molded_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    grey_plastic = model.material("warm_grey_plastic", rgba=(0.45, 0.46, 0.45, 1.0))
    reflector = model.material("amber_reflector", rgba=(1.0, 0.50, 0.08, 1.0))

    frame = model.part("frame")

    # Foldable-but-rigid-looking lower chassis: narrow transport-chair proportions.
    for y, side in ((0.25, "left"), (-0.25, "right")):
        _cylinder_between(frame, (-0.30, y, 0.19), (0.34, y, 0.22), radius=TUBE_R, name=f"{side}_lower_rail", material=aluminum)
        _cylinder_between(frame, (-0.26, y, 0.50), (0.31, y, 0.50), radius=TUBE_R, name=f"{side}_seat_rail", material=aluminum)
        _cylinder_between(frame, (-0.30, y, 0.19), (-0.26, y, 0.51), radius=TUBE_R, name=f"{side}_rear_upright", material=aluminum)
        _cylinder_between(frame, (0.33, y, 0.22), (0.31, y, 0.50), radius=TUBE_R, name=f"{side}_front_upright", material=aluminum)
        _cylinder_between(frame, (-0.30, y, 0.20), (0.30, y, 0.50), radius=SLENDER_TUBE_R, name=f"{side}_diagonal_brace", material=aluminum)
        _cylinder_between(frame, (-0.18, y, 0.515), (0.10, y, 0.66), radius=SLENDER_TUBE_R, name=f"{side}_arm_support", material=aluminum)
        _cylinder_between(frame, (0.22, y, 0.50), (0.22, y, 0.66), radius=SLENDER_TUBE_R, name=f"{side}_front_arm_post", material=aluminum)
        frame.visual(
            Box((0.39, 0.070, 0.040)),
            origin=Origin(xyz=(0.06, y * 1.09, 0.675)),
            material=black_plastic,
            name=f"{side}_arm_pad",
        )
        _cylinder_between(frame, (0.31, y, 0.50), (0.35, y * 1.13, 0.33), radius=SLENDER_TUBE_R, name=f"{side}_footrest_bracket", material=aluminum)
        _cylinder_between(frame, (0.33, y, 0.22), (0.36, y * 1.14, 0.20), radius=TUBE_R, name=f"{side}_caster_outrigger", material=aluminum)

    _cylinder_between(frame, (-0.28, -0.25, 0.19), (-0.28, 0.25, 0.19), radius=TUBE_R, name="rear_cross_tube", material=aluminum)
    _cylinder_between(frame, (-0.25, -0.25, 0.50), (-0.25, 0.25, 0.50), radius=TUBE_R, name="back_hinge_sleeve", material=dark_metal)
    _cylinder_between(frame, (0.30, -0.25, 0.49), (0.30, 0.25, 0.49), radius=TUBE_R, name="front_cross_tube", material=aluminum)
    _cylinder_between(frame, (0.00, -0.25, 0.435), (0.00, 0.25, 0.435), radius=SLENDER_TUBE_R, name="lower_cross_tube", material=aluminum)
    _cylinder_between(frame, (-0.02, -0.22, 0.36), (0.02, 0.22, 0.50), radius=SLENDER_TUBE_R, name="folding_cross_a", material=aluminum)
    _cylinder_between(frame, (-0.02, 0.22, 0.36), (0.02, -0.22, 0.50), radius=SLENDER_TUBE_R, name="folding_cross_b", material=aluminum)
    frame.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.435), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="scissor_center_pivot",
    )

    # One sling-like seat, not a solid block: thin vinyl/fabric wrapping over the side rails.
    frame.visual(
        Box((0.55, 0.485, 0.030)),
        origin=Origin(xyz=(0.035, 0.0, 0.486)),
        material=fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.54, 0.035, 0.012)),
        origin=Origin(xyz=(0.035, 0.242, 0.508)),
        material=black_plastic,
        name="left_seat_edge_binding",
    )
    frame.visual(
        Box((0.54, 0.035, 0.012)),
        origin=Origin(xyz=(0.035, -0.242, 0.508)),
        material=black_plastic,
        name="right_seat_edge_binding",
    )

    # Rear axle stubs intentionally continue through the wheel hubs.
    _cylinder_between(frame, (-0.28, 0.22, 0.19), (-0.28, 0.338, 0.19), radius=0.0125, name="left_rear_axle", material=dark_metal)
    _cylinder_between(frame, (-0.28, -0.22, 0.19), (-0.28, -0.338, 0.19), radius=0.0125, name="right_rear_axle", material=dark_metal)

    # Vertical sockets for the removable swing-away footrests.
    for y, side in ((0.16, "left"), (-0.16, "right")):
        _cylinder_between(frame, (0.335, y, 0.315), (0.335, y, 0.425), radius=0.017, name=f"{side}_footrest_socket", material=dark_metal)
        _cylinder_between(frame, (0.335, y, 0.400), (0.318, 0.25 if y > 0.0 else -0.25, 0.400), radius=SLENDER_TUBE_R, name=f"{side}_socket_mount", material=aluminum)
        frame.visual(
            Box((0.030, 0.050, 0.018)),
            origin=Origin(xyz=(0.333, y, 0.417)),
            material=grey_plastic,
            name=f"{side}_footrest_latch",
        )

    # Small reflectors and anti-tip feel details on the medical frame.
    frame.visual(Box((0.012, 0.004, 0.045)), origin=Origin(xyz=(-0.29, 0.262, 0.31)), material=reflector, name="left_rear_reflector")
    frame.visual(Box((0.012, 0.004, 0.045)), origin=Origin(xyz=(-0.29, -0.262, 0.31)), material=reflector, name="right_rear_reflector")
    _cylinder_between(frame, (-0.30, 0.25, 0.19), (-0.46, 0.22, 0.10), radius=SLENDER_TUBE_R, name="left_antitip_bar", material=aluminum)
    _cylinder_between(frame, (-0.30, -0.25, 0.19), (-0.46, -0.22, 0.10), radius=SLENDER_TUBE_R, name="right_antitip_bar", material=aluminum)

    # Folding backrest with push handles.
    backrest = model.part("backrest")
    _cylinder_between(backrest, (0.0, -0.27, 0.0), (0.0, 0.27, 0.0), radius=0.014, name="hinge_pin", material=dark_metal)
    for y, side in ((0.235, "left"), (-0.235, "right")):
        _cylinder_between(backrest, (0.0, y, 0.0), (-0.075, y, 0.455), radius=TUBE_R, name=f"{side}_back_post", material=aluminum)
        _tube_spline(
            backrest,
            [(-0.075, y, 0.455), (-0.115, y, 0.530), (-0.205, y, 0.560), (-0.300, y, 0.555)],
            radius=TUBE_R,
            name=f"{side}_push_handle_tube",
            material=aluminum,
            samples=14,
        )
        _cylinder_between(backrest, (-0.220, y, 0.555), (-0.340, y, 0.555), radius=0.018, name=f"{side}_push_grip", material=rubber)
        backrest.visual(
            Box((0.030, 0.030, 0.030)),
            origin=Origin(xyz=(-0.214, y, 0.548)),
            material=dark_metal,
            name=f"{side}_brake_perch",
        )
    backrest.visual(
        Box((0.035, 0.470, 0.385)),
        origin=Origin(xyz=(-0.060, 0.0, 0.250)),
        material=fabric,
        name="back_sling",
    )
    backrest.visual(
        Box((0.030, 0.490, 0.030)),
        origin=Origin(xyz=(-0.075, 0.0, 0.455)),
        material=black_plastic,
        name="top_back_binding",
    )
    backrest.visual(
        Box((0.032, 0.070, 0.040)),
        origin=Origin(xyz=(-0.080, 0.0, 0.220)),
        material=fabric,
        name="lumbar_patch",
    )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.25, 0.0, 0.50)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    # Attendant brake levers at the push handles.
    for y, side, axis_sign in ((0.235, "left", 1.0), (-0.235, "right", -1.0)):
        lever = model.part(f"{side}_brake_lever")
        _cylinder_between(lever, (0.0, -0.018 * axis_sign, 0.0), (0.0, 0.018 * axis_sign, 0.0), radius=0.006, name="pivot_pin", material=dark_metal)
        _cylinder_between(lever, (0.0, 0.0, -0.005), (0.050, 0.0, -0.090), radius=0.0045, name="lever_blade", material=dark_metal)
        lever.visual(
            Box((0.070, 0.018, 0.014)),
            origin=Origin(xyz=(0.070, 0.0, -0.096), rpy=(0.0, 0.45, 0.0)),
            material=black_plastic,
            name="finger_pull",
        )
        model.articulation(
            f"backrest_to_{side}_brake_lever",
            ArticulationType.REVOLUTE,
            parent=backrest,
            child=lever,
            origin=Origin(xyz=(-0.214, y, 0.548)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=0.55),
        )

    # Smaller transport rear wheels.
    for y, side, cap_side in ((0.315, "left", 1.0), (-0.315, "right", -1.0)):
        wheel = model.part(f"{side}_rear_wheel")
        _add_wheel(
            wheel,
            mesh_prefix=f"{side}_rear",
            tire_radius=0.155,
            tire_width=0.050,
            cap_side=cap_side,
            rubber=rubber,
            rim_material=grey_plastic,
            hub_material=dark_metal,
            spoke_count=8,
        )
        model.articulation(
            f"frame_to_{side}_rear_wheel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.28, y, 0.19)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    # Front caster forks swivel, and each caster wheel rolls about its axle.
    for y, side in ((0.285, "left"), (-0.285, "right")):
        fork = model.part(f"{side}_caster_fork")
        _cylinder_between(fork, (0.0, 0.0, 0.028), (0.0, 0.0, -0.035), radius=0.013, name="swivel_stem", material=dark_metal)
        fork.visual(
            Box((0.070, 0.090, 0.018)),
            origin=Origin(xyz=(0.010, 0.0, -0.030)),
            material=aluminum,
            name="fork_crown",
        )
        _cylinder_between(fork, (0.0, -0.032, -0.025), (0.115, -0.032, -0.090), radius=0.0065, name="inner_fork_arm", material=aluminum)
        _cylinder_between(fork, (0.0, 0.032, -0.025), (0.115, 0.032, -0.090), radius=0.0065, name="outer_fork_arm", material=aluminum)
        _cylinder_between(fork, (0.115, -0.040, -0.090), (0.115, 0.040, -0.090), radius=0.006, name="caster_axle_pin", material=dark_metal)
        fork.visual(
            Cylinder(radius=0.028, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=grey_plastic,
            name="swivel_bearing_cap",
        )
        model.articulation(
            f"frame_to_{side}_caster_fork",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.365, y, 0.190)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=8.0, lower=-math.pi, upper=math.pi),
        )

        wheel = model.part(f"{side}_front_wheel")
        _add_wheel(
            wheel,
            mesh_prefix=f"{side}_front",
            tire_radius=0.073,
            tire_width=0.032,
            cap_side=1.0 if y > 0 else -1.0,
            rubber=rubber,
            rim_material=grey_plastic,
            hub_material=dark_metal,
            spoke_count=5,
        )
        model.articulation(
            f"{side}_caster_fork_to_front_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.115, 0.0, -0.090)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=24.0),
        )

    # Swing-away leg riggings with fold-up footplates.
    for y, side, swing_axis in ((0.160, "left", 1.0), (-0.160, "right", -1.0)):
        footrest = model.part(f"{side}_footrest")
        _cylinder_between(footrest, (0.0, 0.0, 0.055), (0.0, 0.0, -0.115), radius=0.012, name="socket_pin", material=dark_metal)
        _cylinder_between(footrest, (0.0, 0.0, -0.075), (0.080, 0.0, -0.205), radius=SLENDER_TUBE_R, name="upper_hanger_tube", material=aluminum)
        _cylinder_between(footrest, (0.080, 0.0, -0.205), (0.240, 0.0, -0.300), radius=SLENDER_TUBE_R, name="lower_hanger_tube", material=aluminum)
        _cylinder_between(footrest, (0.220, -0.085, -0.300), (0.220, 0.085, -0.300), radius=0.006, name="footplate_hinge_pin", material=dark_metal)
        footrest.visual(
            Box((0.080, 0.042, 0.018)),
            origin=Origin(xyz=(0.052, 0.0, -0.105)),
            material=grey_plastic,
            name="release_tab",
        )
        model.articulation(
            f"frame_to_{side}_footrest",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=footrest,
            origin=Origin(xyz=(0.335, y, 0.370)),
            axis=(0.0, 0.0, swing_axis),
            motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.20),
        )

        footplate = model.part(f"{side}_footplate")
        footplate.visual(
            Box((0.205, 0.150, 0.016)),
            origin=Origin(xyz=(0.105, 0.0, -0.010)),
            material=black_plastic,
            name="plate",
        )
        for i, x in enumerate((0.045, 0.090, 0.135, 0.175)):
            footplate.visual(
                Box((0.010, 0.132, 0.006)),
                origin=Origin(xyz=(x, 0.0, 0.000)),
                material=grey_plastic,
                name=f"tread_rib_{i}",
            )
        _cylinder_between(footplate, (0.0, -0.085, 0.0), (0.0, 0.085, 0.0), radius=0.010, name="hinge_sleeve", material=dark_metal)
        _tube_spline(
            footplate,
            [(-0.002, -0.064, 0.000), (-0.035, 0.0, 0.050), (-0.002, 0.064, 0.000)],
            radius=0.004,
            name="heel_loop",
            material=black_plastic,
            samples=10,
        )
        model.articulation(
            f"{side}_footrest_to_footplate",
            ArticulationType.REVOLUTE,
            parent=footrest,
            child=footplate,
            origin=Origin(xyz=(0.220, 0.0, -0.300)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.55),
        )

    return model


def _aabb_center_axis(aabb, axis: str) -> float | None:
    if aabb is None:
        return None
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    return (aabb[0][idx] + aabb[1][idx]) * 0.5


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Intentional, local mechanism penetrations: pins/axles are captured by simplified hub or barrel proxies.
    overlap_specs = [
        ("frame", "backrest", "back_hinge_sleeve", "hinge_pin", "folding backrest hinge pin runs inside the frame hinge sleeve."),
        ("frame", "backrest", "left_seat_rail", "hinge_pin", "left hinge knuckle is represented by the back pin passing through the side rail end."),
        ("frame", "backrest", "right_seat_rail", "hinge_pin", "right hinge knuckle is represented by the back pin passing through the side rail end."),
        ("frame", "backrest", "left_rear_upright", "hinge_pin", "left rear upright wraps the folding back hinge pin."),
        ("frame", "backrest", "right_rear_upright", "hinge_pin", "right rear upright wraps the folding back hinge pin."),
        ("frame", "backrest", "left_rear_upright", "left_back_post", "left back post is closely nested beside the rear upright hinge knuckle."),
        ("frame", "backrest", "right_rear_upright", "right_back_post", "right back post is closely nested beside the rear upright hinge knuckle."),
        ("frame", "backrest", "left_seat_rail", "left_back_post", "left back-post hinge knuckle locally nests against the side seat rail."),
        ("frame", "backrest", "right_seat_rail", "right_back_post", "right back-post hinge knuckle locally nests against the side seat rail."),
        ("frame", "backrest", "back_hinge_sleeve", "left_back_post", "left back post knuckle is welded around the folding hinge sleeve."),
        ("frame", "backrest", "back_hinge_sleeve", "right_back_post", "right back post knuckle is welded around the folding hinge sleeve."),
        ("frame", "left_rear_wheel", "left_rear_axle", "rim", "rear axle stub is intentionally captured through the left wheel hub."),
        ("frame", "left_rear_wheel", "left_rear_axle", "outer_cap", "left axle end sits under the protective hub cap."),
        ("frame", "right_rear_wheel", "right_rear_axle", "rim", "rear axle stub is intentionally captured through the right wheel hub."),
        ("frame", "right_rear_wheel", "right_rear_axle", "outer_cap", "right axle end sits under the protective hub cap."),
        ("frame", "left_caster_fork", "left_caster_outrigger", "swivel_stem", "caster swivel stem is intentionally seated in the front outrigger bearing."),
        ("frame", "left_caster_fork", "left_caster_outrigger", "swivel_bearing_cap", "caster bearing cap is seated into the outrigger socket."),
        ("frame", "right_caster_fork", "right_caster_outrigger", "swivel_stem", "caster swivel stem is intentionally seated in the front outrigger bearing."),
        ("frame", "right_caster_fork", "right_caster_outrigger", "swivel_bearing_cap", "caster bearing cap is seated into the outrigger socket."),
        ("left_caster_fork", "left_front_wheel", "caster_axle_pin", "rim", "front caster axle passes through the wheel hub proxy."),
        ("left_caster_fork", "left_front_wheel", "caster_axle_pin", "outer_cap", "front caster axle end is covered by the small hub cap."),
        ("right_caster_fork", "right_front_wheel", "caster_axle_pin", "rim", "front caster axle passes through the wheel hub proxy."),
        ("right_caster_fork", "right_front_wheel", "caster_axle_pin", "outer_cap", "front caster axle end is covered by the small hub cap."),
        ("frame", "left_footrest", "left_footrest_socket", "socket_pin", "removable footrest pin nests inside the frame socket."),
        ("frame", "left_footrest", "left_footrest_latch", "socket_pin", "spring latch captures the removable left footrest pin."),
        ("frame", "left_footrest", "left_socket_mount", "socket_pin", "left footrest socket mount locally surrounds the removable pin."),
        ("frame", "right_footrest", "right_footrest_socket", "socket_pin", "removable footrest pin nests inside the frame socket."),
        ("frame", "right_footrest", "right_footrest_latch", "socket_pin", "spring latch captures the removable right footrest pin."),
        ("frame", "right_footrest", "right_socket_mount", "socket_pin", "right footrest socket mount locally surrounds the removable pin."),
        ("left_footrest", "left_footplate", "footplate_hinge_pin", "hinge_sleeve", "fold-up footplate hinge pin is captured inside the sleeve."),
        ("left_footrest", "left_footplate", "lower_hanger_tube", "hinge_sleeve", "left footplate sleeve is welded onto the lower hanger tube."),
        ("left_footrest", "left_footplate", "footplate_hinge_pin", "heel_loop", "left heel loop is anchored around the footplate hinge pin."),
        ("right_footrest", "right_footplate", "footplate_hinge_pin", "hinge_sleeve", "fold-up footplate hinge pin is captured inside the sleeve."),
        ("right_footrest", "right_footplate", "lower_hanger_tube", "hinge_sleeve", "right footplate sleeve is welded onto the lower hanger tube."),
        ("right_footrest", "right_footplate", "footplate_hinge_pin", "heel_loop", "right heel loop is anchored around the footplate hinge pin."),
        ("backrest", "left_brake_lever", "left_brake_perch", "pivot_pin", "attendant brake lever pivot pin sits in the handle perch."),
        ("backrest", "left_brake_lever", "left_push_handle_tube", "pivot_pin", "brake clamp pivot wraps around the left push-handle tube."),
        ("backrest", "left_brake_lever", "left_brake_perch", "lever_blade", "brake lever blade emerges from the perch with a small captured clearance proxy."),
        ("backrest", "right_brake_lever", "right_brake_perch", "pivot_pin", "attendant brake lever pivot pin sits in the handle perch."),
        ("backrest", "right_brake_lever", "right_push_handle_tube", "pivot_pin", "brake clamp pivot wraps around the right push-handle tube."),
        ("backrest", "right_brake_lever", "right_brake_perch", "lever_blade", "brake lever blade emerges from the perch with a small captured clearance proxy."),
    ]
    for link_a, link_b, elem_a, elem_b, reason in overlap_specs:
        ctx.allow_overlap(link_a, link_b, elem_a=elem_a, elem_b=elem_b, reason=reason)

    # Proof checks paired with overlap allowances.
    ctx.expect_overlap("backrest", "frame", axes="y", elem_a="hinge_pin", elem_b="back_hinge_sleeve", min_overlap=0.48, name="back hinge spans chair width")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="hinge_pin", elem_b="left_rear_upright", min_overlap=0.010, name="left rear upright holds back hinge")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="hinge_pin", elem_b="right_rear_upright", min_overlap=0.010, name="right rear upright holds back hinge")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="left_back_post", elem_b="left_rear_upright", min_overlap=0.006, name="left back post nests at upright knuckle")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="right_back_post", elem_b="right_rear_upright", min_overlap=0.006, name="right back post nests at upright knuckle")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="left_back_post", elem_b="left_seat_rail", min_overlap=0.006, name="left back post nests beside seat rail")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="right_back_post", elem_b="right_seat_rail", min_overlap=0.006, name="right back post nests beside seat rail")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="left_back_post", elem_b="back_hinge_sleeve", min_overlap=0.010, name="left back post knuckle surrounds hinge sleeve")
    ctx.expect_overlap("backrest", "frame", axes="yz", elem_a="right_back_post", elem_b="back_hinge_sleeve", min_overlap=0.010, name="right back post knuckle surrounds hinge sleeve")
    ctx.expect_overlap("left_rear_wheel", "frame", axes="yz", elem_a="rim", elem_b="left_rear_axle", min_overlap=0.020, name="left rear wheel retained on axle")
    ctx.expect_overlap("right_rear_wheel", "frame", axes="yz", elem_a="rim", elem_b="right_rear_axle", min_overlap=0.020, name="right rear wheel retained on axle")
    ctx.expect_overlap("left_rear_wheel", "frame", axes="yz", elem_a="outer_cap", elem_b="left_rear_axle", min_overlap=0.004, name="left hub cap covers axle end")
    ctx.expect_overlap("right_rear_wheel", "frame", axes="yz", elem_a="outer_cap", elem_b="right_rear_axle", min_overlap=0.004, name="right hub cap covers axle end")
    ctx.expect_overlap("left_caster_fork", "frame", axes="xy", elem_a="swivel_stem", elem_b="left_caster_outrigger", min_overlap=0.016, name="left caster stem seated")
    ctx.expect_overlap("right_caster_fork", "frame", axes="xy", elem_a="swivel_stem", elem_b="right_caster_outrigger", min_overlap=0.016, name="right caster stem seated")
    ctx.expect_overlap("left_caster_fork", "frame", axes="xy", elem_a="swivel_bearing_cap", elem_b="left_caster_outrigger", min_overlap=0.020, name="left caster bearing cap seated")
    ctx.expect_overlap("right_caster_fork", "frame", axes="xy", elem_a="swivel_bearing_cap", elem_b="right_caster_outrigger", min_overlap=0.020, name="right caster bearing cap seated")
    ctx.expect_overlap("left_front_wheel", "left_caster_fork", axes="yz", elem_a="rim", elem_b="caster_axle_pin", min_overlap=0.012, name="left caster wheel retained on fork axle")
    ctx.expect_overlap("right_front_wheel", "right_caster_fork", axes="yz", elem_a="rim", elem_b="caster_axle_pin", min_overlap=0.012, name="right caster wheel retained on fork axle")
    ctx.expect_overlap("left_front_wheel", "left_caster_fork", axes="yz", elem_a="outer_cap", elem_b="caster_axle_pin", min_overlap=0.006, name="left caster cap covers axle")
    ctx.expect_overlap("right_front_wheel", "right_caster_fork", axes="yz", elem_a="outer_cap", elem_b="caster_axle_pin", min_overlap=0.006, name="right caster cap covers axle")
    ctx.expect_overlap("left_footrest", "frame", axes="z", elem_a="socket_pin", elem_b="left_footrest_socket", min_overlap=0.075, name="left swingaway pin retained")
    ctx.expect_overlap("right_footrest", "frame", axes="z", elem_a="socket_pin", elem_b="right_footrest_socket", min_overlap=0.075, name="right swingaway pin retained")
    ctx.expect_overlap("left_footrest", "frame", axes="xz", elem_a="socket_pin", elem_b="left_footrest_latch", min_overlap=0.010, name="left footrest latch captures pin")
    ctx.expect_overlap("right_footrest", "frame", axes="xz", elem_a="socket_pin", elem_b="right_footrest_latch", min_overlap=0.010, name="right footrest latch captures pin")
    ctx.expect_overlap("left_footrest", "frame", axes="xz", elem_a="socket_pin", elem_b="left_socket_mount", min_overlap=0.010, name="left socket mount surrounds pin")
    ctx.expect_overlap("right_footrest", "frame", axes="xz", elem_a="socket_pin", elem_b="right_socket_mount", min_overlap=0.010, name="right socket mount surrounds pin")
    ctx.expect_overlap("left_footplate", "left_footrest", axes="y", elem_a="hinge_sleeve", elem_b="footplate_hinge_pin", min_overlap=0.12, name="left footplate hinge captured")
    ctx.expect_overlap("right_footplate", "right_footrest", axes="y", elem_a="hinge_sleeve", elem_b="footplate_hinge_pin", min_overlap=0.12, name="right footplate hinge captured")
    ctx.expect_overlap("left_footplate", "left_footrest", axes="xz", elem_a="hinge_sleeve", elem_b="lower_hanger_tube", min_overlap=0.010, name="left sleeve welded to hanger")
    ctx.expect_overlap("right_footplate", "right_footrest", axes="xz", elem_a="hinge_sleeve", elem_b="lower_hanger_tube", min_overlap=0.010, name="right sleeve welded to hanger")
    ctx.expect_overlap("left_footplate", "left_footrest", axes="xz", elem_a="heel_loop", elem_b="footplate_hinge_pin", min_overlap=0.006, name="left heel loop anchored at hinge")
    ctx.expect_overlap("right_footplate", "right_footrest", axes="xz", elem_a="heel_loop", elem_b="footplate_hinge_pin", min_overlap=0.006, name="right heel loop anchored at hinge")
    ctx.expect_overlap("left_brake_lever", "backrest", axes="xz", elem_a="lever_blade", elem_b="left_brake_perch", min_overlap=0.006, name="left brake lever emerges from perch")
    ctx.expect_overlap("right_brake_lever", "backrest", axes="xz", elem_a="lever_blade", elem_b="right_brake_perch", min_overlap=0.006, name="right brake lever emerges from perch")
    ctx.expect_overlap("left_brake_lever", "backrest", axes="yz", elem_a="pivot_pin", elem_b="left_push_handle_tube", min_overlap=0.006, name="left brake pivot clamps handle tube")
    ctx.expect_overlap("right_brake_lever", "backrest", axes="yz", elem_a="pivot_pin", elem_b="right_push_handle_tube", min_overlap=0.006, name="right brake pivot clamps handle tube")

    # Scale and chair-specific geometry checks.
    seat_box = ctx.part_element_world_aabb("frame", elem="seat_sling")
    if seat_box is not None:
        seat_height = (seat_box[0][2] + seat_box[1][2]) * 0.5
        seat_width = seat_box[1][1] - seat_box[0][1]
        ctx.check(
            "seat height and width are transport-chair scale",
            0.45 <= seat_height <= 0.52 and 0.44 <= seat_width <= 0.52,
            details=f"seat_height={seat_height:.3f}, seat_width={seat_width:.3f}",
        )
    else:
        ctx.fail("seat sling exists", "seat_sling element was not found")

    rear_wheel_box = ctx.part_world_aabb("left_rear_wheel")
    front_wheel_box = ctx.part_world_aabb("left_front_wheel")
    if rear_wheel_box is not None and front_wheel_box is not None:
        rear_diameter = rear_wheel_box[1][2] - rear_wheel_box[0][2]
        front_diameter = front_wheel_box[1][2] - front_wheel_box[0][2]
        ctx.check(
            "transport wheels are smaller than self-propel wheelchair wheels",
            0.28 <= rear_diameter <= 0.34 and 0.13 <= front_diameter <= 0.17 and rear_diameter < 0.45,
            details=f"rear_diameter={rear_diameter:.3f}, front_diameter={front_diameter:.3f}",
        )

    # Folding backrest moves down and forward like a transport chair latch-back.
    back_joint = object_model.get_articulation("frame_to_backrest")
    rest_back = ctx.part_world_aabb("backrest")
    with ctx.pose({back_joint: 1.35}):
        folded_back = ctx.part_world_aabb("backrest")
    if rest_back is not None and folded_back is not None:
        rest_top = rest_back[1][2]
        folded_top = folded_back[1][2]
        rest_center_x = _aabb_center_axis(rest_back, "x")
        folded_center_x = _aabb_center_axis(folded_back, "x")
        ctx.check(
            "backrest folds forward and lowers push handles",
            folded_top < rest_top - 0.10 and folded_center_x is not None and rest_center_x is not None and folded_center_x > rest_center_x + 0.18,
            details=f"rest_top={rest_top:.3f}, folded_top={folded_top:.3f}, rest_x={rest_center_x}, folded_x={folded_center_x}",
        )

    # Swing-away riggings move outward; footplates flip upward.
    for side, direction in (("left", 1.0), ("right", -1.0)):
        swing = object_model.get_articulation(f"frame_to_{side}_footrest")
        plate = object_model.get_articulation(f"{side}_footrest_to_footplate")
        rest_footrest = ctx.part_world_aabb(f"{side}_footrest")
        with ctx.pose({swing: 1.05}):
            swung_footrest = ctx.part_world_aabb(f"{side}_footrest")
        rest_y = _aabb_center_axis(rest_footrest, "y")
        swung_y = _aabb_center_axis(swung_footrest, "y")
        ctx.check(
            f"{side} footrest swings outward",
            rest_y is not None and swung_y is not None and (swung_y - rest_y) * direction > 0.09,
            details=f"rest_y={rest_y}, swung_y={swung_y}",
        )
        rest_plate = ctx.part_world_aabb(f"{side}_footplate")
        with ctx.pose({plate: 1.35}):
            raised_plate = ctx.part_world_aabb(f"{side}_footplate")
        if rest_plate is not None and raised_plate is not None:
            ctx.check(
                f"{side} footplate folds upward",
                raised_plate[1][2] > rest_plate[1][2] + 0.10,
                details=f"rest_max_z={rest_plate[1][2]:.3f}, raised_max_z={raised_plate[1][2]:.3f}",
            )

    # Caster swivel visibly changes the caster wheel trail direction.
    caster_joint = object_model.get_articulation("frame_to_left_caster_fork")
    rest_caster = ctx.part_world_aabb("left_front_wheel")
    with ctx.pose({caster_joint: math.pi / 2.0}):
        side_caster = ctx.part_world_aabb("left_front_wheel")
    if rest_caster is not None and side_caster is not None:
        rest_center_y = _aabb_center_axis(rest_caster, "y")
        side_center_y = _aabb_center_axis(side_caster, "y")
        ctx.check(
            "front caster swivels about a vertical stem",
            rest_center_y is not None and side_center_y is not None and abs(side_center_y - rest_center_y) > 0.03,
            details=f"rest_y={rest_center_y}, side_y={side_center_y}",
        )

    return ctx.report()


object_model = build_object_model()
