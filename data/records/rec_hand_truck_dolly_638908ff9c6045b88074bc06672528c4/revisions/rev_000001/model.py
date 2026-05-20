from __future__ import annotations

import math

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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _wheel_meshes():
    tire = TireGeometry(
        0.155,
        0.085,
        inner_radius=0.105,
        tread=TireTread(style="block", depth=0.008, count=20, land_ratio=0.54),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.003),),
        sidewall=TireSidewall(style="rounded", bulge=0.045),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    wheel = WheelGeometry(
        0.108,
        0.070,
        rim=WheelRim(
            inner_radius=0.064,
            flange_height=0.008,
            flange_thickness=0.004,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.038,
            width=0.060,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.052, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.008, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.005, window_radius=0.018),
        bore=WheelBore(style="round", diameter=0.045),
    )
    return (
        mesh_from_geometry(tire, "hand_truck_utility_tire"),
        mesh_from_geometry(wheel, "hand_truck_spoked_wheel"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stockroom_hand_truck")

    powder_coat = model.material("red_powder_coat", rgba=(0.78, 0.08, 0.05, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.58, 0.58, 0.55, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    rim_grey = model.material("rim_grey", rgba=(0.68, 0.70, 0.70, 1.0))

    tire_mesh, wheel_mesh = _wheel_meshes()

    frame = model.part("frame")

    rounded_frame = tube_from_spline_points(
        [
            (0.040, -0.245, 0.160),
            (0.035, -0.245, 0.540),
            (0.026, -0.238, 1.030),
            (0.044, -0.205, 1.285),
            (0.095, -0.090, 1.395),
            (0.110, 0.000, 1.420),
            (0.095, 0.090, 1.395),
            (0.044, 0.205, 1.285),
            (0.026, 0.238, 1.030),
            (0.035, 0.245, 0.540),
            (0.040, 0.245, 0.160),
        ],
        radius=0.016,
        samples_per_segment=14,
        radial_segments=18,
        up_hint=(1.0, 0.0, 0.0),
    )
    frame.visual(
        mesh_from_geometry(rounded_frame, "hand_truck_rounded_frame"),
        material=powder_coat,
        name="rounded_frame",
    )

    loop_handle = tube_from_spline_points(
        [
            (0.028, -0.205, 1.285),
            (0.120, -0.255, 1.330),
            (0.230, -0.175, 1.370),
            (0.250, 0.000, 1.385),
            (0.230, 0.175, 1.370),
            (0.120, 0.255, 1.330),
            (0.028, 0.205, 1.285),
            (-0.005, 0.000, 1.285),
        ],
        radius=0.015,
        samples_per_segment=16,
        radial_segments=18,
        closed_spline=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    frame.visual(
        mesh_from_geometry(loop_handle, "hand_truck_loop_handle"),
        material=powder_coat,
        name="loop_handle",
    )

    for z, member_name in ((0.315, "lower_crossbar"), (0.705, "middle_crossbar")):
        _add_member(
            frame,
            (0.040, -0.260, z),
            (0.040, 0.260, z),
            radius=0.012,
            material=powder_coat,
            name=member_name,
        )
    _add_member(
        frame,
        (0.034, -0.252, 0.885),
        (0.034, 0.252, 0.885),
        radius=0.012,
        material=powder_coat,
        name="load_crossbar",
    )

    # Fixed steel toe plate, with a raised rear heel and small front curl.
    frame.visual(
        Box((0.390, 0.600, 0.024)),
        origin=Origin(xyz=(-0.130, 0.000, 0.012)),
        material=worn_steel,
        name="toe_plate",
    )
    frame.visual(
        Box((0.025, 0.600, 0.150)),
        origin=Origin(xyz=(0.055, 0.000, 0.075)),
        material=worn_steel,
        name="rear_toe_lip",
    )
    frame.visual(
        Box((0.026, 0.585, 0.065)),
        origin=Origin(xyz=(-0.322, 0.000, 0.045)),
        material=worn_steel,
        name="front_toe_curl",
    )
    for y in (-0.180, 0.180):
        frame.visual(
            Box((0.320, 0.032, 0.030)),
            origin=Origin(xyz=(-0.115, y, 0.032)),
            material=dark_steel,
            name=f"toe_rib_{0 if y < 0 else 1}",
        )
    for index, y in enumerate((-0.245, 0.245)):
        frame.visual(
            Box((0.056, 0.060, 0.100)),
            origin=Origin(xyz=(0.052, y, 0.116)),
            material=dark_steel,
            name=f"toe_weld_{index}",
        )

    # Rear axle and the welded braces that tie it into the tubular frame.
    frame.visual(
        Cylinder(radius=0.016, length=0.800),
        origin=Origin(xyz=(0.180, 0.000, 0.155), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    for y in (-0.250, 0.250):
        _add_member(
            frame,
            (0.044, y, 0.200),
            (0.180, y, 0.155),
            radius=0.012,
            material=powder_coat,
            name=f"axle_strut_{0 if y < 0 else 1}",
        )
        _add_member(
            frame,
            (0.040, y, 0.315),
            (0.180, y, 0.155),
            radius=0.010,
            material=powder_coat,
            name=f"triangulated_stay_{0 if y < 0 else 1}",
        )

    # Hinge brackets for the center kickstand leg.  The center is open so the
    # leg sleeve can rotate between the two clevis cheeks on a captured pin.
    for index, y in enumerate((-0.074, 0.074)):
        frame.visual(
            Box((0.060, 0.030, 0.038)),
            origin=Origin(xyz=(0.212, y, 0.132)),
            material=dark_steel,
            name=f"kickstand_cheek_{index}",
        )
    for index, y in enumerate((-0.074, 0.074)):
        _add_member(
            frame,
            (0.178, y, 0.155),
            (0.206, y, 0.100),
            radius=0.010,
            material=dark_steel,
            name=f"kickstand_stay_{index}",
        )
    frame.visual(
        Cylinder(radius=0.010, length=0.220),
        origin=Origin(xyz=(0.212, 0.000, 0.132), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="hinge_pin",
    )
    _add_member(
        frame,
        (0.212, -0.087, 0.132),
        (0.212, -0.037, 0.132),
        radius=0.019,
        material=dark_steel,
        name="hinge_lug_0",
    )
    _add_member(
        frame,
        (0.212, 0.037, 0.132),
        (0.212, 0.087, 0.132),
        radius=0.019,
        material=dark_steel,
        name="hinge_lug_1",
    )

    for index, y in enumerate((-0.390, 0.390)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_grey,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=0.110),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="bearing_sleeve",
        )
        model.articulation(
            f"frame_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.180, y, 0.155)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.022, length=0.064),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    kickstand.visual(
        Cylinder(
            radius=0.012,
            length=_distance((0.000, 0.000, -0.004), (0.132, 0.000, -0.118)),
        ),
        origin=Origin(
            xyz=_midpoint((0.000, 0.000, -0.004), (0.132, 0.000, -0.118)),
            rpy=_rpy_for_cylinder((0.000, 0.000, -0.004), (0.132, 0.000, -0.118)),
        ),
        material=dark_steel,
        name="leg_tube",
    )
    _add_member(
        kickstand,
        (0.132, -0.090, -0.118),
        (0.132, 0.090, -0.118),
        radius=0.014,
        material=black_rubber,
        name="rubber_foot",
    )
    model.articulation(
        "frame_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=kickstand,
        origin=Origin(xyz=(0.212, 0.000, 0.132)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-1.05, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    kickstand = object_model.get_part("kickstand")
    kickstand_joint = object_model.get_articulation("frame_to_kickstand")

    ctx.expect_overlap(
        wheel_0,
        frame,
        axes="xz",
        min_overlap=0.045,
        name="first wheel is centered on the rear axle line",
    )
    ctx.expect_overlap(
        wheel_1,
        frame,
        axes="xz",
        min_overlap=0.045,
        name="second wheel is centered on the rear axle line",
    )
    for wheel_name in ("wheel_0", "wheel_1"):
        ctx.allow_overlap(
            "frame",
            wheel_name,
            elem_a="rear_axle",
            elem_b="bearing_sleeve",
            reason="The wheel bearing sleeve is intentionally shown captured around the solid axle proxy.",
        )
        ctx.expect_overlap(
            wheel_name,
            frame,
            axes="xyz",
            min_overlap=0.020,
            elem_a="bearing_sleeve",
            elem_b="rear_axle",
            name=f"{wheel_name} bearing sleeve captures the axle",
        )
    ctx.allow_overlap(
        frame,
        kickstand,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The hinge pin is intentionally represented as a solid shaft through the kickstand sleeve.",
    )
    ctx.allow_overlap(
        frame,
        kickstand,
        elem_a="hinge_pin",
        elem_b="leg_tube",
        reason="The welded kickstand leg root is simplified as a tube entering the hinge sleeve around the pin.",
    )
    ctx.expect_overlap(
        frame,
        kickstand,
        axes="xyz",
        min_overlap=0.018,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        name="kickstand sleeve is captured on the hinge pin",
    )
    ctx.expect_overlap(
        frame,
        kickstand,
        axes="xyz",
        min_overlap=0.010,
        elem_a="hinge_pin",
        elem_b="leg_tube",
        name="kickstand leg root is welded into the hinge sleeve area",
    )

    deployed_aabb = ctx.part_world_aabb(kickstand)
    with ctx.pose({kickstand_joint: -0.85}):
        folded_aabb = ctx.part_world_aabb(kickstand)

    ctx.check(
        "kickstand rotates upward from its deployed foot position",
        deployed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > deployed_aabb[0][2] + 0.035,
        details=f"deployed={deployed_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
