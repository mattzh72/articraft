from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
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
)


def aft_panel_profile(
    chord_root: float,
    chord_tip: float,
    span: float,
    sweep: float,
    *,
    mirror: bool = False,
) -> list[tuple[float, float]]:
    points = [
        (0.0, 0.0),
        (-sweep, span),
        (-(sweep + chord_tip), span),
        (-chord_root, 0.0),
    ]
    if mirror:
        points = [(x, -y) for x, y in points]
    return points


def forward_panel_profile(
    chord_root: float,
    chord_tip: float,
    span: float,
    sweep: float,
) -> list[tuple[float, float]]:
    return [
        (0.0, 0.0),
        (chord_root, 0.0),
        (sweep + chord_tip, span),
        (sweep, span),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rescue_service_blimp")

    model.material("hull_white", rgba=(0.95, 0.97, 0.98, 1.0))
    model.material("rescue_red", rgba=(0.82, 0.11, 0.12, 1.0))
    model.material("trim_orange", rgba=(0.94, 0.44, 0.14, 1.0))
    model.material("dark_gray", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("metal", rgba=(0.72, 0.74, 0.77, 1.0))

    hull = model.part("hull")

    hull_profile = [
        (0.02, -13.0),
        (0.40, -12.4),
        (0.95, -11.3),
        (1.90, -9.2),
        (2.65, -6.1),
        (3.05, -2.2),
        (3.20, 1.2),
        (3.10, 4.5),
        (2.55, 8.3),
        (1.50, 11.0),
        (0.48, 12.4),
        (0.02, 13.0),
    ]
    hull_shell = LatheGeometry(hull_profile, segments=120)
    hull_shell.rotate_y(math.pi / 2.0)
    hull.visual(
        mesh_from_geometry(hull_shell, "hull_shell"),
        material="hull_white",
        name="envelope",
    )

    upper_fin = ExtrudeGeometry.centered(
        forward_panel_profile(chord_root=2.45, chord_tip=0.78, span=3.10, sweep=0.05),
        0.26,
    )
    upper_fin.rotate_x(math.pi / 2.0)
    hull.visual(
        mesh_from_geometry(upper_fin, "upper_fin"),
        origin=Origin(xyz=(-12.05, 0.0, 0.0)),
        material="rescue_red",
        name="upper_fin",
    )

    lower_fin = ExtrudeGeometry.centered(
        forward_panel_profile(chord_root=1.75, chord_tip=0.42, span=1.95, sweep=-0.10),
        0.18,
    )
    lower_fin.rotate_x(-math.pi / 2.0)
    hull.visual(
        mesh_from_geometry(lower_fin, "lower_fin"),
        origin=Origin(xyz=(-11.80, 0.0, 0.0)),
        material="rescue_red",
        name="lower_fin",
    )

    hull.visual(
        Box((0.92, 0.65, 0.18)),
        origin=Origin(xyz=(-11.24, 0.875, 0.0)),
        material="rescue_red",
        name="left_tail_root",
    )

    hull.visual(
        Box((0.92, 0.65, 0.18)),
        origin=Origin(xyz=(-11.24, -0.875, 0.0)),
        material="rescue_red",
        name="right_tail_root",
    )
    hull.visual(
        Box((0.24, 0.26, 2.20)),
        origin=Origin(xyz=(-11.93, 0.0, 1.72)),
        material="rescue_red",
        name="rudder_post",
    )

    for side_name, side in (("left", 1.0), ("right", -1.0)):
        hull.visual(
            Box((0.70, 1.34, 1.18)),
            origin=Origin(xyz=(-1.75, side * 3.42, -1.10)),
            material="rescue_red",
            name=f"{side_name}_pod_pylon",
        )
        hull.visual(
            Cylinder(radius=0.34, length=1.85),
            origin=Origin(
                xyz=(-1.60, side * 3.82, -2.03),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="rescue_red",
            name=f"{side_name}_pod_body",
        )
        hull.visual(
            Sphere(radius=0.34),
            origin=Origin(xyz=(-0.68, side * 3.82, -2.03)),
            material="trim_orange",
            name=f"{side_name}_pod_nose",
        )
        hull.visual(
            Cylinder(radius=0.058, length=0.42),
            origin=Origin(
                xyz=(-0.40, side * 3.82, -2.03),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="metal",
            name=f"{side_name}_prop_shaft",
        )

    gondola = model.part("gondola")
    gondola.visual(
        Box((1.25, 0.90, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material="rescue_red",
        name="roof_saddle",
    )
    for x in (-0.55, 0.55):
        for y in (-0.34, 0.34):
            gondola.visual(
                Cylinder(radius=0.045, length=0.48),
                origin=Origin(xyz=(x, y, -0.42)),
                material="metal",
                name=f"roof_post_{'f' if x > 0.0 else 'a'}_{'l' if y > 0.0 else 'r'}",
            )
    cabin_shell = LatheGeometry(
        [
            (0.05, -2.15),
            (0.28, -2.00),
            (0.62, -1.65),
            (0.82, -0.95),
            (0.90, -0.10),
            (0.86, 0.85),
            (0.68, 1.65),
            (0.28, 2.10),
            (0.05, 2.22),
        ],
        segments=80,
    )
    cabin_shell.rotate_y(math.pi / 2.0)
    gondola.visual(
        mesh_from_geometry(cabin_shell, "cabin_shell"),
        origin=Origin(xyz=(0.0, 0.0, -1.36)),
        material="dark_gray",
        name="cabin_shell",
    )
    gondola.visual(
        Box((2.75, 0.90, 0.24)),
        origin=Origin(xyz=(0.20, 0.0, -2.10)),
        material="rescue_red",
        name="belly_keel",
    )
    gondola.visual(
        Box((1.30, 1.36, 0.48)),
        origin=Origin(xyz=(-0.55, 0.0, -2.34)),
        material="dark_gray",
        name="gear_fairing",
    )

    for side_name, side in (("left", 1.0), ("right", -1.0)):
        gondola.visual(
            Cylinder(radius=0.05, length=0.72),
            origin=Origin(xyz=(-0.55, side * 0.60, -2.38)),
            material="metal",
            name=f"{side_name}_gear_leg",
        )

    gondola.visual(
        Cylinder(radius=0.024, length=1.12),
        origin=Origin(
            xyz=(-0.55, 0.0, -2.70),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="metal",
        name="rear_beam",
    )
    gondola.visual(
        Cylinder(radius=0.024, length=0.14),
        origin=Origin(
            xyz=(-0.55, 0.67, -2.70),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="metal",
        name="left_axle",
    )
    gondola.visual(
        Cylinder(radius=0.024, length=0.14),
        origin=Origin(
            xyz=(-0.55, -0.67, -2.70),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="metal",
        name="right_axle",
    )

    for side_name, side in (("left", 1.0), ("right", -1.0)):
        gondola.visual(
            Cylinder(radius=0.035, length=0.48),
            origin=Origin(xyz=(1.40, side * 0.16, -2.38)),
            material="metal",
            name=f"{side_name}_fork_leg",
        )
    gondola.visual(
        Cylinder(radius=0.020, length=0.34),
        origin=Origin(
            xyz=(1.40, 0.0, -2.61),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="metal",
        name="front_axle",
    )

    rudder = model.part("rudder")
    rudder_panel = ExtrudeGeometry.centered(
        aft_panel_profile(chord_root=1.70, chord_tip=0.60, span=2.35, sweep=0.10),
        0.22,
    )
    rudder_panel.rotate_x(math.pi / 2.0)
    rudder.visual(
        mesh_from_geometry(rudder_panel, "rudder_panel"),
        material="trim_orange",
        name="panel",
    )

    left_elevator = model.part("left_elevator")
    left_elevator_panel = ExtrudeGeometry.centered(
        aft_panel_profile(chord_root=1.78, chord_tip=0.94, span=2.65, sweep=0.12),
        0.18,
    )
    left_elevator.visual(
        mesh_from_geometry(left_elevator_panel, "left_elevator_panel"),
        material="trim_orange",
        name="panel",
    )

    right_elevator = model.part("right_elevator")
    right_elevator_panel = ExtrudeGeometry.centered(
        aft_panel_profile(
            chord_root=1.78,
            chord_tip=0.94,
            span=2.65,
            sweep=0.12,
            mirror=True,
        ),
        0.18,
    )
    right_elevator.visual(
        mesh_from_geometry(right_elevator_panel, "right_elevator_panel"),
        material="trim_orange",
        name="panel",
    )

    propeller_rotor = FanRotorGeometry(
        0.76,
        0.11,
        3,
        thickness=0.13,
        blade_pitch_deg=34.0,
        blade_sweep_deg=22.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.15),
        hub=FanRotorHub(style="spinner", bore_diameter=0.115),
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        mesh_from_geometry(propeller_rotor, "left_propeller"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_rubber",
        name="rotor",
    )
    left_propeller.visual(
        Cylinder(radius=0.09, length=0.15),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="metal",
        name="hub_collar",
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        mesh_from_geometry(propeller_rotor, "right_propeller"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_rubber",
        name="rotor",
    )
    right_propeller.visual(
        Cylinder(radius=0.09, length=0.15),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="metal",
        name="hub_collar",
    )

    main_rim = WheelGeometry(
        0.155,
        0.12,
        rim=WheelRim(
            inner_radius=0.108,
            flange_height=0.010,
            flange_thickness=0.004,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.055,
            width=0.088,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.050, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.016),
        bore=WheelBore(style="round", diameter=0.045),
    )
    main_tire = TireGeometry(
        0.235,
        0.14,
        inner_radius=0.150,
        carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.06),
        tread=TireTread(style="rib", depth=0.004, count=6, land_ratio=0.70),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.008, radius=0.004),
    )

    nose_rim = WheelGeometry(
        0.120,
        0.09,
        rim=WheelRim(
            inner_radius=0.080,
            flange_height=0.008,
            flange_thickness=0.004,
            bead_seat_depth=0.003,
        ),
        hub=WheelHub(
            radius=0.042,
            width=0.070,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.036, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.007, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="straight", count=4, thickness=0.004, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.038),
    )
    nose_tire = TireGeometry(
        0.185,
        0.10,
        inner_radius=0.117,
        carcass=TireCarcass(belt_width_ratio=0.66, sidewall_bulge=0.05),
        tread=TireTread(style="rib", depth=0.003, count=5, land_ratio=0.72),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )

    for part_name in ("left_wheel", "right_wheel"):
        wheel_part = model.part(part_name)
        wheel_part.visual(
            mesh_from_geometry(main_tire, f"{part_name}_tire"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material="black_rubber",
            name="tire",
        )
        wheel_part.visual(
            mesh_from_geometry(main_rim, f"{part_name}_rim"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material="metal",
            name="rim",
        )

    nose_wheel = model.part("nose_wheel")
    nose_wheel.visual(
        mesh_from_geometry(nose_tire, "nose_wheel_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material="black_rubber",
        name="tire",
    )
    nose_wheel.visual(
        mesh_from_geometry(nose_rim, "nose_wheel_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material="metal",
        name="rim",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.10, 0.0, -3.28)),
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-12.05, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.6, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "hull_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(-11.70, 1.20, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "hull_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-11.70, -1.20, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "hull_to_left_propeller",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=left_propeller,
        origin=Origin(xyz=(-0.12, 3.82, -2.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=35.0),
    )
    model.articulation(
        "hull_to_right_propeller",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=right_propeller,
        origin=Origin(xyz=(-0.12, -3.82, -2.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=35.0),
    )
    model.articulation(
        "gondola_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=model.get_part("left_wheel"),
        origin=Origin(xyz=(-0.55, 0.78, -2.70)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "gondola_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=model.get_part("right_wheel"),
        origin=Origin(xyz=(-0.55, -0.78, -2.70)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "gondola_to_nose_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=nose_wheel,
        origin=Origin(xyz=(1.40, 0.0, -2.61)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    nose_wheel = object_model.get_part("nose_wheel")

    ctx.allow_overlap(
        gondola,
        nose_wheel,
        elem_a="front_axle",
        elem_b="rim",
        reason="The nose wheel rim is intentionally represented around the fork axle instead of a fully hollow hub.",
    )
    ctx.allow_overlap(
        gondola,
        left_wheel,
        elem_a="left_axle",
        elem_b="rim",
        reason="The left main wheel rim is intentionally represented around the axle stub rather than a fully hollow hub shell.",
    )
    ctx.allow_overlap(
        gondola,
        right_wheel,
        elem_a="right_axle",
        elem_b="rim",
        reason="The right main wheel rim is intentionally represented around the axle stub rather than a fully hollow hub shell.",
    )

    rudder_joint = object_model.get_articulation("hull_to_rudder")
    left_elevator_joint = object_model.get_articulation("hull_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("hull_to_right_elevator")

    ctx.expect_gap(
        hull,
        gondola,
        axis="z",
        positive_elem="envelope",
        negative_elem="roof_saddle",
        min_gap=0.02,
        max_gap=0.35,
        name="gondola roof stays just below the envelope",
    )
    ctx.expect_overlap(
        gondola,
        hull,
        axes="x",
        elem_a="cabin_shell",
        elem_b="envelope",
        min_overlap=3.0,
        name="gondola remains centered under the hull length",
    )
    ctx.expect_gap(
        gondola,
        left_wheel,
        axis="z",
        positive_elem="cabin_shell",
        negative_elem="tire",
        min_gap=0.18,
        name="left main wheel hangs below the gondola cabin",
    )
    ctx.expect_gap(
        gondola,
        right_wheel,
        axis="z",
        positive_elem="cabin_shell",
        negative_elem="tire",
        min_gap=0.18,
        name="right main wheel hangs below the gondola cabin",
    )
    ctx.expect_gap(
        gondola,
        nose_wheel,
        axis="z",
        positive_elem="cabin_shell",
        negative_elem="tire",
        min_gap=0.16,
        name="nose wheel hangs below the gondola cabin",
    )

    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rest_box = ctx.part_element_world_aabb(rudder, elem="panel")
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            posed_box = ctx.part_element_world_aabb(rudder, elem="panel")
        rest_center_y = None if rest_box is None else 0.5 * (rest_box[0][1] + rest_box[1][1])
        posed_center_y = None if posed_box is None else 0.5 * (posed_box[0][1] + posed_box[1][1])
        ctx.check(
            "rudder yaws off center at positive deflection",
            rest_center_y is not None
            and posed_center_y is not None
            and abs(posed_center_y - rest_center_y) > 0.18,
            details=f"rest_center_y={rest_center_y}, posed_center_y={posed_center_y}",
        )

    left_limits = left_elevator_joint.motion_limits
    if left_limits is not None and left_limits.upper is not None:
        rest_box = ctx.part_element_world_aabb(left_elevator, elem="panel")
        with ctx.pose({left_elevator_joint: left_limits.upper}):
            posed_box = ctx.part_element_world_aabb(left_elevator, elem="panel")
        ctx.check(
            "left elevator trailing edge rises at positive deflection",
            rest_box is not None and posed_box is not None and posed_box[1][2] > rest_box[1][2] + 0.20,
            details=f"rest={rest_box}, posed={posed_box}",
        )

    right_limits = right_elevator_joint.motion_limits
    if right_limits is not None and right_limits.upper is not None:
        rest_box = ctx.part_element_world_aabb(right_elevator, elem="panel")
        with ctx.pose({right_elevator_joint: right_limits.upper}):
            posed_box = ctx.part_element_world_aabb(right_elevator, elem="panel")
        ctx.check(
            "right elevator trailing edge rises at positive deflection",
            rest_box is not None and posed_box is not None and posed_box[1][2] > rest_box[1][2] + 0.20,
            details=f"rest={rest_box}, posed={posed_box}",
        )

    return ctx.report()


object_model = build_object_model()
