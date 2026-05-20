from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
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
    wire_from_points,
)


def _tube(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _wire(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.025,
            corner_segments=8,
        ),
        name,
    )


def _add_wheel_visuals(part, tire_mesh, rim_mesh, rubber, rim_paint) -> None:
    # Wheel helpers spin about their local X axis; rotate the visuals so the
    # authored wheels spin on the hand truck's rear axle along Y.
    wheel_to_axle = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    part.visual(tire_mesh, origin=wheel_to_axle, material=rubber, name="tire")
    part.visual(rim_mesh, origin=wheel_to_axle, material=rim_paint, name="rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_office_hand_truck")

    powder_blue = model.material("powder_blue", rgba=(0.07, 0.28, 0.55, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.66, 1.0))
    scuffed_plate = model.material("scuffed_plate", rgba=(0.45, 0.46, 0.43, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.115,
            0.056,
            inner_radius=0.081,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.004, count=22, land_ratio=0.62),
            grooves=(
                TireGroove(center_offset=-0.012, width=0.0045, depth=0.002),
                TireGroove(center_offset=0.012, width=0.0045, depth=0.002),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "small_utility_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.082,
            0.052,
            rim=WheelRim(
                inner_radius=0.055,
                flange_height=0.006,
                flange_thickness=0.003,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(
                radius=0.026,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.032, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.009),
            bore=WheelBore(style="round", diameter=0.030),
        ),
        "painted_small_wheel",
    )

    frame = model.part("frame")
    frame.visual(
        _tube(
            "one_piece_tubular_back",
            [
                (-0.035, -0.185, 0.205),
                (-0.035, -0.185, 0.83),
                (-0.045, -0.170, 1.04),
                (0.000, -0.130, 1.155),
                (0.000, 0.130, 1.155),
                (-0.045, 0.170, 1.04),
                (-0.035, 0.185, 0.205),
            ],
            0.014,
        ),
        material=powder_blue,
        name="main_tube",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.410),
        origin=Origin(xyz=(-0.035, 0.0, 0.310), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_blue,
        name="lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.390),
        origin=Origin(xyz=(-0.025, 0.0, 0.610), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_blue,
        name="middle_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(-0.018, 0.0, 0.875), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_blue,
        name="upper_crossbar",
    )
    frame.visual(
        _tube(
            "left_lower_brace",
            [(-0.035, -0.185, 0.240), (0.020, -0.155, 0.145), (0.055, -0.150, 0.070)],
            0.010,
        ),
        material=powder_blue,
        name="brace_0",
    )
    frame.visual(
        _tube(
            "right_lower_brace",
            [(-0.035, 0.185, 0.240), (0.020, 0.155, 0.145), (0.055, 0.150, 0.070)],
            0.010,
        ),
        material=powder_blue,
        name="brace_1",
    )
    frame.visual(
        Box((0.330, 0.430, 0.024)),
        origin=Origin(xyz=(0.178, 0.0, 0.040)),
        material=scuffed_plate,
        name="toe_plate",
    )
    frame.visual(
        Box((0.020, 0.430, 0.045)),
        origin=Origin(xyz=(0.335, 0.0, 0.073)),
        material=scuffed_plate,
        name="upturned_lip",
    )
    frame.visual(
        Box((0.038, 0.430, 0.090)),
        origin=Origin(xyz=(0.015, 0.0, 0.085)),
        material=powder_blue,
        name="toe_plate_mount",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.630),
        origin=Origin(xyz=(-0.185, 0.0, 0.122), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    frame.visual(
        Box((0.085, 0.040, 0.105)),
        origin=Origin(xyz=(-0.145, -0.212, 0.155)),
        material=dark_steel,
        name="axle_plate_0",
    )
    frame.visual(
        Box((0.085, 0.040, 0.105)),
        origin=Origin(xyz=(-0.145, 0.212, 0.155)),
        material=dark_steel,
        name="axle_plate_1",
    )
    frame.visual(
        _tube(
            "left_axle_stay",
            [(-0.035, -0.185, 0.205), (-0.105, -0.205, 0.178), (-0.185, -0.212, 0.155)],
            0.010,
        ),
        material=powder_blue,
        name="axle_stay_0",
    )
    frame.visual(
        _tube(
            "right_axle_stay",
            [(-0.035, 0.185, 0.205), (-0.105, 0.205, 0.178), (-0.185, 0.212, 0.155)],
            0.010,
        ),
        material=powder_blue,
        name="axle_stay_1",
    )
    frame.visual(
        Box((0.045, 0.020, 0.060)),
        origin=Origin(xyz=(-0.060, -0.168, 0.250)),
        material=dark_steel,
        name="stand_hinge_ear_0",
    )
    frame.visual(
        Box((0.045, 0.020, 0.060)),
        origin=Origin(xyz=(-0.060, 0.168, 0.250)),
        material=dark_steel,
        name="stand_hinge_ear_1",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(-0.060, -0.185, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_bolt_0",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(-0.060, 0.185, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_bolt_1",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.45, 0.47, 1.16)),
        mass=7.0,
        origin=Origin(xyz=(0.03, 0.0, 0.58)),
    )

    wheel_0 = model.part("wheel_0")
    _add_wheel_visuals(wheel_0, tire_mesh, rim_mesh, black_rubber, galvanized)
    wheel_0.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.056),
        mass=0.75,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    wheel_1 = model.part("wheel_1")
    _add_wheel_visuals(wheel_1, tire_mesh, rim_mesh, black_rubber, galvanized)
    wheel_1.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.056),
        mass=0.75,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    stand = model.part("parking_stand")
    stand.visual(
        Cylinder(radius=0.014, length=0.316),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    stand.visual(
        _wire(
            "parking_stand_u",
            [
                (0.000, -0.142, 0.000),
                (0.115, -0.142, -0.028),
                (0.240, -0.142, -0.040),
                (0.240, 0.142, -0.040),
                (0.115, 0.142, -0.028),
                (0.000, 0.142, 0.000),
            ],
            0.010,
        ),
        material=dark_steel,
        name="u_leg",
    )
    stand.visual(
        Cylinder(radius=0.014, length=0.310),
        origin=Origin(xyz=(0.246, 0.0, -0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rubber_foot",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.27, 0.32, 0.07)),
        mass=0.6,
        origin=Origin(xyz=(0.13, 0.0, -0.025)),
    )

    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.185, -0.315, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.185, 0.315, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "stand_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=stand,
        origin=Origin(xyz=(-0.060, 0.0, 0.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    stand = object_model.get_part("parking_stand")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")
    stand_hinge = object_model.get_articulation("stand_hinge")

    ctx.allow_overlap(
        frame,
        wheel_0,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The fixed axle is intentionally captured through the wheel hub/rim bore proxy so the wheel reads mounted and spins about the shaft.",
    )
    ctx.allow_overlap(
        frame,
        wheel_1,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The fixed axle is intentionally captured through the wheel hub/rim bore proxy so the wheel reads mounted and spins about the shaft.",
    )

    ctx.check(
        "rear wheels use continuous spin joints",
        wheel_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_1_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"wheel_0={wheel_0_spin.articulation_type}, wheel_1={wheel_1_spin.articulation_type}",
    )
    ctx.check(
        "parking stand is a limited hinge",
        stand_hinge.articulation_type == ArticulationType.REVOLUTE
        and stand_hinge.motion_limits is not None
        and stand_hinge.motion_limits.lower == 0.0
        and stand_hinge.motion_limits.upper is not None
        and stand_hinge.motion_limits.upper > 1.0,
        details=f"type={stand_hinge.articulation_type}, limits={stand_hinge.motion_limits}",
    )
    ctx.expect_overlap(
        wheel_0,
        frame,
        axes="xz",
        min_overlap=0.02,
        elem_a="rim",
        elem_b="rear_axle",
        name="wheel_0 is centered on rear axle",
    )
    ctx.expect_overlap(
        wheel_1,
        frame,
        axes="xz",
        min_overlap=0.02,
        elem_a="rim",
        elem_b="rear_axle",
        name="wheel_1 is centered on rear axle",
    )
    ctx.expect_gap(
        frame,
        stand,
        axis="z",
        min_gap=0.020,
        max_gap=0.050,
        positive_elem="lower_crossbar",
        negative_elem="hinge_sleeve",
        name="stand hinge sits just below lower crossbar",
    )

    stowed_aabb = ctx.part_element_world_aabb(stand, elem="rubber_foot")
    with ctx.pose({stand_hinge: 1.25}):
        deployed_aabb = ctx.part_element_world_aabb(stand, elem="rubber_foot")
        ctx.expect_gap(
            stand,
            frame,
            axis="x",
            min_gap=0.020,
            positive_elem="u_leg",
            negative_elem="rear_axle",
            name="deployed stand clears rear axle",
        )
        ctx.expect_gap(
            frame,
            stand,
            axis="x",
            min_gap=0.015,
            positive_elem="toe_plate",
            negative_elem="u_leg",
            name="deployed stand clears toe plate edge",
        )
        ctx.expect_gap(
            frame,
            stand,
            axis="z",
            min_gap=0.002,
            max_gap=0.090,
            positive_elem="toe_plate",
            negative_elem="rubber_foot",
            name="deployed stand foot sits below toe plate",
        )
    ctx.check(
        "stand foot swings down when deployed",
        stowed_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < stowed_aabb[0][2] - 0.15,
        details=f"stowed={stowed_aabb}, deployed={deployed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
