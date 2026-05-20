from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
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
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _cushion_mesh(
    name: str,
    *,
    length: float,
    width: float,
    height: float,
    radius: float,
):
    """Rounded, slightly crowned rectangular upholstery pad."""
    lower = rounded_rect_profile(length, width, radius, corner_segments=10)
    mid = rounded_rect_profile(
        length * 0.985,
        width * 0.985,
        radius * 0.92,
        corner_segments=10,
    )
    upper = rounded_rect_profile(
        length * 0.94,
        width * 0.92,
        radius * 0.72,
        corner_segments=10,
    )
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, height * 0.72) for x, y in mid],
            [(x, y, height) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _wheel_meshes(prefix: str):
    tire = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.046,
            inner_radius=0.034,
            tread=TireTread(style="ribbed", depth=0.003, count=18, land_ratio=0.60),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        f"{prefix}_tire",
    )
    rim = mesh_from_geometry(
        WheelGeometry(
            0.037,
            0.050,
            rim=WheelRim(
                inner_radius=0.024,
                flange_height=0.003,
                flange_thickness=0.002,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.013,
                width=0.032,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.018, hole_diameter=0.0025),
            ),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        f"{prefix}_rim",
    )
    return tire, rim


def _add_cylinder_y(part, *, radius: float, length: float, origin: tuple[float, float, float], material, name: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=origin, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_incline_utility_weight_bench")

    matte_black = model.material("matte_black_powdercoat", rgba=(0.045, 0.047, 0.050, 1.0))
    dark_steel = model.material("dark_pin_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    brushed = model.material("brushed_hinge_steel", rgba=(0.60, 0.61, 0.62, 1.0))
    vinyl = model.material("black_textured_vinyl", rgba=(0.025, 0.024, 0.023, 1.0))
    seam_grey = model.material("stitched_seam_grey", rgba=(0.28, 0.28, 0.27, 1.0))
    rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.010, 1.0))

    seat_mesh = _cushion_mesh("seat_pad_rounded", length=0.400, width=0.310, height=0.068, radius=0.045)
    back_mesh = _cushion_mesh("backrest_pad_rounded", length=0.760, width=0.315, height=0.070, radius=0.048)
    wheel_tire, wheel_rim = _wheel_meshes("front_transport_wheel")

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.20, 0.46, 0.44)),
        mass=18.0,
        origin=Origin(xyz=(-0.06, 0.0, 0.22)),
    )

    # Rectangular low base and tube-like rails.
    frame.visual(
        Box((1.180, 0.420, 0.026)),
        origin=Origin(xyz=(-0.075, 0.0, 0.048)),
        material=matte_black,
        name="rectangular_base",
    )
    for y, suffix in ((0.180, "0"), (-0.180, "1")):
        frame.visual(
            Box((1.160, 0.050, 0.060)),
            origin=Origin(xyz=(-0.075, y, 0.084)),
            material=matte_black,
            name=f"side_rail_{suffix}",
        )
    for x, name in ((0.500, "front_crossmember"), (-0.645, "rear_crossmember"), (-0.070, "middle_crossmember")):
        frame.visual(
            Box((0.055, 0.430, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.087)),
            material=matte_black,
            name=name,
        )

    # Front upright support for the seat hinge.
    for y, suffix in ((0.145, "0"), (-0.145, "1")):
        frame.visual(
            Box((0.050, 0.050, 0.335)),
            origin=Origin(xyz=(0.370, y, 0.225)),
            material=matte_black,
            name=f"front_upright_{suffix}",
        )
    for y, suffix in ((0.205, "0"), (-0.205, "1")):
        frame.visual(
            Box((0.055, 0.025, 0.330)),
            origin=Origin(xyz=(0.370, y, 0.255)),
            material=matte_black,
            name=f"front_outer_upright_{suffix}",
        )
    frame.visual(
        Box((0.075, 0.370, 0.035)),
        origin=Origin(xyz=(0.370, 0.0, 0.370)),
        material=matte_black,
        name="front_hinge_bridge",
    )
    _add_cylinder_y(
        frame,
        radius=0.014,
        length=0.390,
        origin=(0.370, 0.0, 0.420),
        material=brushed,
        name="seat_hinge_pin",
    )

    # Backrest hinge support at the seat/back junction.
    for y, suffix in ((0.152, "0"), (-0.152, "1")):
        frame.visual(
            Box((0.050, 0.050, 0.350)),
            origin=Origin(xyz=(-0.060, y, 0.230)),
            material=matte_black,
            name=f"junction_upright_{suffix}",
        )
    frame.visual(
        Box((0.080, 0.370, 0.034)),
        origin=Origin(xyz=(-0.060, 0.0, 0.389)),
        material=matte_black,
        name="backrest_hinge_bridge",
    )
    _add_cylinder_y(
        frame,
        radius=0.014,
        length=0.390,
        origin=(-0.060, 0.0, 0.420),
        material=brushed,
        name="backrest_hinge_pin",
    )

    # Lower pivot and ladder rail for the rear prop arm.
    frame.visual(
        Box((0.230, 0.050, 0.040)),
        origin=Origin(xyz=(-0.625, 0.0, 0.087)),
        material=matte_black,
        name="rear_pivot_bridge",
    )
    for y, suffix in ((0.050, "0"), (-0.050, "1")):
        frame.visual(
            Box((0.060, 0.030, 0.090)),
            origin=Origin(xyz=(-0.625, y, 0.142)),
            material=matte_black,
            name=f"rear_pivot_lug_{suffix}",
        )
    _add_cylinder_y(
        frame,
        radius=0.012,
        length=0.145,
        origin=(-0.625, 0.0, 0.142),
        material=brushed,
        name="rear_support_pin",
    )
    frame.visual(
        Box((0.360, 0.034, 0.035)),
        origin=Origin(xyz=(-0.360, 0.058, 0.255), rpy=(0.0, -0.36, 0.0)),
        material=dark_steel,
        name="incline_ladder_rail",
    )
    for i, (x, z, height) in enumerate(((-0.500, 0.170, 0.230), (-0.235, 0.218, 0.325))):
        for side_index, y in enumerate((0.058, -0.058)):
            frame.visual(
                Box((0.040, 0.084, height)),
                origin=Origin(xyz=(x, y, z)),
                material=matte_black,
                name=f"ladder_stanchion_{i}_{side_index}",
            )
    for i, x in enumerate((-0.495, -0.420, -0.345, -0.270)):
        frame.visual(
            Box((0.034, 0.070, 0.060)),
            origin=Origin(xyz=(x, 0.080, 0.307 + (x + 0.495) * 0.32)),
            material=brushed,
            name=f"ladder_tooth_{i}",
        )
    frame.visual(
        Box((0.300, 0.016, 0.030)),
        origin=Origin(xyz=(-0.380, 0.092, 0.330)),
        material=dark_steel,
        name="ladder_tooth_spine",
    )

    # Front carry handle and wheel brackets.
    handle_geom = tube_from_spline_points(
        [
            (0.500, -0.105, 0.112),
            (0.585, -0.070, 0.145),
            (0.630, 0.000, 0.158),
            (0.585, 0.070, 0.145),
            (0.500, 0.105, 0.112),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(handle_geom, "front_carry_handle"), material=matte_black, name="front_handle")
    for y, suffix in ((0.105, "0"), (-0.105, "1")):
        frame.visual(
            Box((0.070, 0.040, 0.085)),
            origin=Origin(xyz=(0.502, y, 0.087)),
            material=matte_black,
            name=f"handle_mount_{suffix}",
        )
    for y, suffix in ((0.242, "0"), (-0.242, "1")):
        frame.visual(
            Box((0.095, 0.040, 0.070)),
            origin=Origin(xyz=(0.555, y, 0.078)),
            material=matte_black,
            name=f"wheel_mount_{suffix}",
        )
        frame.visual(
            Box((0.080, 0.030, 0.055)),
            origin=Origin(xyz=(0.555, 0.213 if y > 0.0 else -0.213, 0.078)),
            material=matte_black,
            name=f"wheel_bracket_web_{suffix}",
        )
        _add_cylinder_y(
            frame,
            radius=0.010,
            length=0.070,
            origin=(0.585, y + (0.008 if y > 0.0 else -0.008), 0.055),
            material=brushed,
            name=f"axle_stub_{suffix}",
        )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.78, 0.33, 0.08)),
        mass=5.0,
        origin=Origin(xyz=(-0.380, 0.0, 0.050)),
    )
    backrest.visual(back_mesh, origin=Origin(xyz=(-0.385, 0.0, 0.015)), material=vinyl, name="back_pad")
    backrest.visual(
        Box((0.700, 0.010, 0.006)),
        origin=Origin(xyz=(-0.390, 0.132, 0.085)),
        material=seam_grey,
        name="back_seam_0",
    )
    backrest.visual(
        Box((0.700, 0.010, 0.006)),
        origin=Origin(xyz=(-0.390, -0.132, 0.085)),
        material=seam_grey,
        name="back_seam_1",
    )
    _add_cylinder_y(
        backrest,
        radius=0.018,
        length=0.340,
        origin=(0.0, 0.0, 0.0),
        material=brushed,
        name="back_hinge_barrel",
    )
    backrest.visual(
        Box((0.580, 0.050, 0.026)),
        origin=Origin(xyz=(-0.390, 0.0, 0.014)),
        material=dark_steel,
        name="back_under_rail",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.42, 0.33, 0.08)),
        mass=3.4,
        origin=Origin(xyz=(-0.200, 0.0, 0.050)),
    )
    seat.visual(seat_mesh, origin=Origin(xyz=(-0.200, 0.0, 0.015)), material=vinyl, name="seat_pad")
    seat.visual(
        Box((0.310, 0.010, 0.006)),
        origin=Origin(xyz=(-0.200, 0.130, 0.085)),
        material=seam_grey,
        name="seat_seam_0",
    )
    seat.visual(
        Box((0.310, 0.010, 0.006)),
        origin=Origin(xyz=(-0.200, -0.130, 0.085)),
        material=seam_grey,
        name="seat_seam_1",
    )
    _add_cylinder_y(
        seat,
        radius=0.018,
        length=0.335,
        origin=(0.0, 0.0, 0.0),
        material=brushed,
        name="seat_hinge_barrel",
    )
    seat.visual(
        Box((0.320, 0.050, 0.026)),
        origin=Origin(xyz=(-0.190, 0.0, 0.007)),
        material=dark_steel,
        name="seat_under_rail",
    )

    support_arm = model.part("rear_support_arm")
    support_arm.inertial = Inertial.from_geometry(
        Box((0.36, 0.20, 0.34)),
        mass=1.3,
        origin=Origin(xyz=(0.160, 0.0, 0.150)),
    )
    support_geom = tube_from_spline_points(
        [
            (0.000, 0.0, 0.000),
            (0.130, 0.0, 0.090),
            (0.320, 0.0, 0.205),
        ],
        radius=0.016,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    support_arm.visual(mesh_from_geometry(support_geom, "rear_prop_arm_tube"), material=matte_black, name="arm_tube")
    _add_cylinder_y(
        support_arm,
        radius=0.024,
        length=0.060,
        origin=(0.0, 0.0, 0.0),
        material=brushed,
        name="lower_bushing",
    )
    _add_cylinder_y(
        support_arm,
        radius=0.018,
        length=0.155,
        origin=(0.320, 0.0, 0.205),
        material=brushed,
        name="upper_roller",
    )

    wheels = []
    for index, y in enumerate((0.292, -0.292)):
        wheel = model.part(f"wheel_{index}")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.055, length=0.046),
            mass=0.36,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        )
        wheel.visual(
            wheel_tire,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_rim,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=brushed,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.055),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hub_sleeve",
        )
        wheels.append((wheel, y))

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.060, 0.0, 0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.370, 0.0, 0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=0.0, upper=0.42),
    )
    model.articulation(
        "rear_support_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.625, 0.0, 0.142)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.35, upper=0.65),
    )
    for index, (wheel, y) in enumerate(wheels):
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.585, y, 0.055)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    rear_support = object_model.get_part("rear_support_arm")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    support_pivot = object_model.get_articulation("rear_support_pivot")

    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_pin",
        elem_b="seat_hinge_barrel",
        reason="The visible hinge pin is intentionally captured inside the seat hinge barrel.",
    )
    ctx.expect_overlap(
        frame,
        seat,
        axes="y",
        elem_a="seat_hinge_pin",
        elem_b="seat_hinge_barrel",
        min_overlap=0.30,
        name="seat hinge pin passes through barrel",
    )
    ctx.expect_within(
        frame,
        seat,
        axes="xz",
        inner_elem="seat_hinge_pin",
        outer_elem="seat_hinge_barrel",
        margin=0.006,
        name="seat hinge pin is concentric with barrel",
    )

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="backrest_hinge_pin",
        elem_b="back_hinge_barrel",
        reason="The visible hinge pin is intentionally captured inside the backrest hinge barrel.",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        axes="y",
        elem_a="backrest_hinge_pin",
        elem_b="back_hinge_barrel",
        min_overlap=0.30,
        name="backrest hinge pin passes through barrel",
    )
    ctx.expect_within(
        frame,
        backrest,
        axes="xz",
        inner_elem="backrest_hinge_pin",
        outer_elem="back_hinge_barrel",
        margin=0.006,
        name="backrest hinge pin is concentric with barrel",
    )

    ctx.allow_overlap(
        frame,
        rear_support,
        elem_a="rear_support_pin",
        elem_b="lower_bushing",
        reason="The lower support-arm pivot pin is intentionally represented inside the bushing.",
    )
    ctx.allow_overlap(
        frame,
        rear_support,
        elem_a="rear_support_pin",
        elem_b="arm_tube",
        reason="The support arm tube is locally welded around the captured lower pivot pin.",
    )
    ctx.expect_overlap(
        frame,
        rear_support,
        axes="y",
        elem_a="rear_support_pin",
        elem_b="lower_bushing",
        min_overlap=0.05,
        name="rear support pin passes through bushing",
    )
    ctx.expect_overlap(
        frame,
        rear_support,
        axes="y",
        elem_a="rear_support_pin",
        elem_b="arm_tube",
        min_overlap=0.02,
        name="rear support tube wraps pivot pin",
    )
    ctx.expect_within(
        frame,
        rear_support,
        axes="xz",
        inner_elem="rear_support_pin",
        outer_elem="lower_bushing",
        margin=0.012,
        name="rear support pin is concentric with bushing",
    )

    for index in (0, 1):
        ctx.allow_overlap(
            frame,
            object_model.get_part(f"wheel_{index}"),
            elem_a=f"axle_stub_{index}",
            elem_b="hub_sleeve",
            reason="The wheel axle stub is intentionally seated inside the rotating hub sleeve.",
        )
        ctx.allow_overlap(
            frame,
            object_model.get_part(f"wheel_{index}"),
            elem_a=f"axle_stub_{index}",
            elem_b="rim",
            reason="The transport-wheel axle passes through the wheel center and is hidden by the rim bore.",
        )
        ctx.expect_overlap(
            frame,
            object_model.get_part(f"wheel_{index}"),
            axes="y",
            elem_a=f"axle_stub_{index}",
            elem_b="hub_sleeve",
            min_overlap=0.015,
            name=f"wheel {index} hub is retained on axle",
        )
        ctx.expect_within(
            frame,
            object_model.get_part(f"wheel_{index}"),
            axes="xz",
            inner_elem=f"axle_stub_{index}",
            outer_elem="hub_sleeve",
            margin=0.004,
            name=f"wheel {index} axle is centered in hub",
        )

    ctx.expect_overlap(
        backrest,
        seat,
        axes="y",
        min_overlap=0.25,
        elem_a="back_pad",
        elem_b="seat_pad",
        name="split pads share bench width",
    )
    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        positive_elem="seat_pad",
        negative_elem="front_hinge_bridge",
        min_gap=0.020,
        name="seat cushion sits above front support",
    )
    ctx.expect_gap(
        backrest,
        frame,
        axis="z",
        positive_elem="back_pad",
        negative_elem="backrest_hinge_bridge",
        min_gap=0.020,
        name="back cushion sits above hinge support",
    )
    ctx.expect_origin_distance(wheel_0, wheel_1, axes="y", min_dist=0.55, max_dist=0.62, name="front wheels flank handle")
    ctx.expect_gap(
        wheel_0,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="side_rail_0",
        min_gap=0.045,
        name="wheel 0 tire clears frame side",
    )
    ctx.expect_gap(
        frame,
        wheel_1,
        axis="y",
        positive_elem="side_rail_1",
        negative_elem="tire",
        min_gap=0.045,
        name="wheel 1 tire clears frame side",
    )

    back_rest_pos = ctx.part_world_aabb(backrest)
    with ctx.pose({backrest_hinge: 0.90}):
        back_open_pos = ctx.part_world_aabb(backrest)
        ctx.expect_gap(backrest, frame, axis="z", min_gap=0.020, elem_a="back_pad", elem_b="rectangular_base", name="raised back clears base")
    ctx.check(
        "backrest hinge raises rear pad",
        back_rest_pos is not None and back_open_pos is not None and back_open_pos[1][2] > back_rest_pos[1][2] + 0.25,
        details=f"rest={back_rest_pos}, opened={back_open_pos}",
    )

    seat_rest_pos = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.36}):
        seat_open_pos = ctx.part_world_aabb(seat)
    ctx.check(
        "seat hinge raises rear edge",
        seat_rest_pos is not None and seat_open_pos is not None and seat_open_pos[1][2] > seat_rest_pos[1][2] + 0.08,
        details=f"rest={seat_rest_pos}, opened={seat_open_pos}",
    )

    support_rest_pos = ctx.part_world_aabb(rear_support)
    with ctx.pose({support_pivot: 0.55}):
        support_open_pos = ctx.part_world_aabb(rear_support)
    ctx.check(
        "rear support arm rotates upward",
        support_rest_pos is not None and support_open_pos is not None and support_open_pos[1][2] > support_rest_pos[1][2] + 0.08,
        details=f"rest={support_rest_pos}, opened={support_open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
