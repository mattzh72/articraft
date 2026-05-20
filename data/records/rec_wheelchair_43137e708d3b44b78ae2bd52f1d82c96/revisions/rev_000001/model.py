from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireShoulder,
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


def _tube_mesh(points, name: str, *, radius: float, samples: int = 8, closed: bool = False):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            closed_spline=closed,
            radial_segments=16,
            cap_ends=not closed,
        ),
        name,
    )


def _circle_points(radius: float, y: float, *, count: int = 48) -> list[tuple[float, float, float]]:
    return [
        (radius * cos((2.0 * pi * i) / count), y, radius * sin((2.0 * pi * i) / count))
        for i in range(count)
    ]


def _add_rear_wheel_visuals(model: ArticulatedObject, part, prefix: str, side_sign: float) -> None:
    wheel_silver = "brushed_aluminum"
    rubber = "soft_black_rubber"
    dark = "dark_hub"
    handrim = "polished_handrim"

    spin_to_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.280,
                0.046,
                inner_radius=0.225,
                tread=TireTread(style="circumferential", depth=0.003, count=4),
                grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
                shoulder=TireShoulder(width=0.006, radius=0.004),
            ),
            f"{prefix}_tire",
        ),
        origin=spin_to_y,
        material=rubber,
        name="tire",
    )
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.225,
                0.034,
                rim=WheelRim(inner_radius=0.170, flange_height=0.010, flange_thickness=0.0035),
                hub=WheelHub(
                    radius=0.038,
                    width=0.040,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.050, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=18, thickness=0.0035, window_radius=0.010),
                bore=WheelBore(style="round", diameter=0.018),
            ),
            f"{prefix}_rim",
        ),
        origin=spin_to_y,
        material=wheel_silver,
        name="spoked_rim",
    )
    part.visual(
        Cylinder(radius=0.046, length=0.058),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hub_cap",
    )

    outer_y = side_sign * 0.044
    inner_y = side_sign * 0.018
    part.visual(
        _tube_mesh(_circle_points(0.238, outer_y, count=72), f"{prefix}_handrim", radius=0.005, samples=4, closed=True),
        material=handrim,
        name="handrim",
    )
    for i in range(8):
        angle = (2.0 * pi * i) / 8.0
        part.visual(
            _tube_mesh(
                [
                    (0.168 * cos(angle), inner_y, 0.168 * sin(angle)),
                    (0.242 * cos(angle), outer_y, 0.242 * sin(angle)),
                ],
                f"{prefix}_handrim_stay_{i}",
                radius=0.003,
                samples=1,
            ),
            material=handrim,
            name=f"handrim_stay_{i}",
        )


def _add_caster_wheel_visuals(model: ArticulatedObject, part, prefix: str) -> None:
    wheel_silver = "brushed_aluminum"
    rubber = "soft_black_rubber"
    dark = "dark_hub"
    spin_to_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.075,
                0.032,
                inner_radius=0.050,
                tread=TireTread(style="block", depth=0.0025, count=16, land_ratio=0.55),
                sidewall=TireSidewall(style="square", bulge=0.015),
            ),
            f"{prefix}_tire",
        ),
        origin=spin_to_y,
        material=rubber,
        name="tire",
    )
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.050,
                0.024,
                rim=WheelRim(inner_radius=0.030, flange_height=0.004, flange_thickness=0.002),
                hub=WheelHub(radius=0.018, width=0.030, cap_style="flat"),
                face=WheelFace(dish_depth=0.002, front_inset=0.001),
                spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
                bore=WheelBore(style="round", diameter=0.008),
            ),
            f"{prefix}_rim",
        ),
        origin=spin_to_y,
        material=wheel_silver,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.058),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_manual_wheelchair")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.75, 1.0))
    dark_frame = model.material("dark_hub", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    vinyl = model.material("black_vinyl", rgba=(0.055, 0.060, 0.065, 1.0))
    handrim = model.material("polished_handrim", rgba=(0.86, 0.86, 0.82, 1.0))
    footrest_mat = model.material("molded_footrest", rgba=(0.08, 0.08, 0.075, 1.0))

    frame = model.part("frame")

    # Seat sling and back pad, both attached to the tubular side rails.
    frame.visual(Box((0.430, 0.500, 0.045)), origin=Origin(xyz=(0.045, 0.0, 0.505)), material=vinyl, name="seat")
    frame.visual(Box((0.045, 0.500, 0.390)), origin=Origin(xyz=(-0.205, 0.0, 0.710)), material=vinyl, name="backrest")

    # Continuous-looking side tubes with rear push handles and front down tubes.
    for side_name, y in (("left", 0.250), ("right", -0.250)):
        frame.visual(
            _tube_mesh(
                [
                    (-0.230, y, 0.890),
                    (-0.220, y, 0.705),
                    (-0.185, y, 0.520),
                    (0.060, y, 0.505),
                    (0.300, y, 0.492),
                    (0.340, y, 0.350),
                    (0.315, y, 0.205),
                ],
                f"{side_name}_side_rail",
                radius=0.015,
                samples=12,
            ),
            material=aluminum,
            name=f"{side_name}_side_rail",
        )
        frame.visual(
            _tube_mesh(
                [
                    (-0.185, y, 0.300),
                    (-0.135, y, 0.405),
                    (-0.060, y, 0.492),
                    (0.140, y, 0.490),
                    (0.310, y, 0.205),
                ],
                f"{side_name}_lower_rail",
                radius=0.013,
                samples=10,
            ),
            material=aluminum,
            name=f"{side_name}_lower_rail",
        )
        frame.visual(
            _tube_mesh(
                [
                    (0.315, y, 0.205),
                    (0.390, y * 0.70, 0.165),
                    (0.505, y * 0.55, 0.150),
                ],
                f"{side_name}_footrest_tube",
                radius=0.011,
                samples=6,
            ),
            material=aluminum,
            name=f"{side_name}_footrest_tube",
        )
        frame.visual(
            Box((0.170, 0.115, 0.024)),
            origin=Origin(xyz=(0.485, y * 0.55, 0.135)),
            material=footrest_mat,
            name=f"{side_name}_footplate",
        )

    # Cross tubes tie the two side frames into a compact welded chair frame.
    for visual_name, x, z, radius, length in (
        ("front_cross_tube", 0.280, 0.492, 0.013, 0.540),
        ("rear_cross_tube", -0.165, 0.505, 0.013, 0.540),
        ("axle_cross_tube", -0.185, 0.300, 0.014, 0.540),
        ("back_top_tube", -0.225, 0.880, 0.012, 0.530),
    ):
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=visual_name,
        )

    # Bearing blocks and sockets for the visible wheel/caster mounting points.
    frame.visual(Box((0.060, 0.045, 0.070)), origin=Origin(xyz=(-0.185, 0.282, 0.300)), material=dark_frame, name="left_axle_block")
    frame.visual(Box((0.060, 0.045, 0.070)), origin=Origin(xyz=(-0.185, -0.282, 0.300)), material=dark_frame, name="right_axle_block")
    for side_name, y in (("left", 0.180), ("right", -0.180)):
        frame.visual(
            Cylinder(radius=0.025, length=0.048),
            origin=Origin(xyz=(0.300, y, 0.222)),
            material=dark_frame,
            name=f"{side_name}_caster_socket",
        )
        frame.visual(
            _tube_mesh([(0.300, y, 0.222), (0.315, y * 1.30, 0.300), (0.315, y * 1.38, 0.430)], f"{side_name}_caster_strut", radius=0.011, samples=6),
            material=aluminum,
            name=f"{side_name}_caster_strut",
        )

    # Rear wheels with pneumatic tires, spoked rims, and separate push handrims.
    left_rear_wheel = model.part("rear_wheel_0")
    _add_rear_wheel_visuals(model, left_rear_wheel, "rear_wheel_0", side_sign=1.0)
    right_rear_wheel = model.part("rear_wheel_1")
    _add_rear_wheel_visuals(model, right_rear_wheel, "rear_wheel_1", side_sign=-1.0)

    # Swiveling caster forks, then caster wheels spinning inside those forks.
    for index, y in ((0, 0.180), (1, -0.180)):
        fork = model.part(f"front_caster_{index}")
        fork.visual(Cylinder(radius=0.011, length=0.060), origin=Origin(xyz=(0.0, 0.0, -0.030)), material=dark_frame, name="stem")
        fork.visual(Box((0.080, 0.082, 0.024)), origin=Origin(xyz=(-0.040, 0.0, -0.030)), material=dark_frame, name="crown")
        fork.visual(Box((0.026, 0.010, 0.118)), origin=Origin(xyz=(-0.075, 0.034, -0.101)), material=dark_frame, name="fork_tine_0")
        fork.visual(Box((0.026, 0.010, 0.118)), origin=Origin(xyz=(-0.075, -0.034, -0.101)), material=dark_frame, name="fork_tine_1")
        fork.visual(Cylinder(radius=0.013, length=0.018), origin=Origin(xyz=(-0.075, 0.045, -0.120), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_frame, name="axle_cap_0")
        fork.visual(Cylinder(radius=0.013, length=0.018), origin=Origin(xyz=(-0.075, -0.045, -0.120), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_frame, name="axle_cap_1")

        wheel = model.part(f"caster_wheel_{index}")
        _add_caster_wheel_visuals(model, wheel, f"caster_wheel_{index}")

        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.300, y, 0.200)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )
        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-0.075, 0.0, -0.120)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    model.articulation(
        "rear_spin_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.185, 0.333, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "rear_spin_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.185, -0.333, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rear_0 = object_model.get_part("rear_wheel_0")
    rear_1 = object_model.get_part("rear_wheel_1")

    ctx.check("has_compact_frame", frame is not None, "Wheelchair should have one root chair frame.")
    ctx.check("has_two_rear_wheels", rear_0 is not None and rear_1 is not None, "Expected two large rear wheels.")
    ctx.check(
        "has_caster_chain",
        all(object_model.get_part(name) is not None for name in ("front_caster_0", "front_caster_1", "caster_wheel_0", "caster_wheel_1")),
        "Expected two swiveling caster forks and two spinning caster wheels.",
    )

    for joint_name in ("rear_spin_0", "rear_spin_1", "caster_swivel_0", "caster_swivel_1", "caster_spin_0", "caster_spin_1"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name}_continuous", joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS, f"{joint_name} should be continuous.")

    if rear_0 is not None and rear_1 is not None and frame is not None:
        ctx.expect_overlap(rear_0, frame, axes="xz", min_overlap=0.04, elem_a="spoked_rim", elem_b="left_axle_block", name="left rear hub aligns to frame bearing")
        ctx.expect_overlap(rear_1, frame, axes="xz", min_overlap=0.04, elem_a="spoked_rim", elem_b="right_axle_block", name="right rear hub aligns to frame bearing")

    with ctx.pose({"caster_swivel_0": pi / 2.0}):
        fork_pos = ctx.part_world_position("front_caster_0")
        wheel_pos = ctx.part_world_position("caster_wheel_0")
    ctx.check(
        "caster_trail_rotates_about_vertical_stem",
        fork_pos is not None and wheel_pos is not None and abs(wheel_pos[1] - fork_pos[1]) > 0.035,
        details=f"fork={fork_pos}, wheel={wheel_pos}",
    )

    aabb = ctx.part_world_aabb(frame)
    ctx.check("frame_aabb_present", aabb is not None, "Expected frame AABB.")
    if aabb is not None:
        mins, maxs = aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("compact_envelope", size[0] < 0.82 and size[1] < 0.74 and size[2] < 0.95, f"frame_size={size!r}")

    return ctx.report()


object_model = build_object_model()
