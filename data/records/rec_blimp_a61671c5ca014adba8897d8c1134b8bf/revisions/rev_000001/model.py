from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _cylinder_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    extra: float = 0.0,
) -> tuple[Origin, float]:
    """Return an Origin and length for a cylinder whose local +Z spans a segment."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return Origin(xyz=start), 0.0
    ux, uy, uz = dx / length, dy / length, dz / length
    sx -= ux * extra
    sy -= uy * extra
    sz -= uz * extra
    ex += ux * extra
    ey += uy * extra
    ez += uz * extra
    length += 2.0 * extra
    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)), length


def _add_strut(part, name, start, end, radius, material, *, extra=0.035) -> None:
    origin, length = _cylinder_origin_between(start, end, extra=extra)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    envelope_mat = model.material("warm_grey_fabric", rgba=(0.74, 0.75, 0.70, 1.0))
    seam_mat = model.material("dull_yellow_panel", rgba=(0.78, 0.64, 0.28, 1.0))
    gondola_mat = model.material("dark_gondola", rgba=(0.12, 0.15, 0.17, 1.0))
    glass_mat = model.material("blue_grey_glass", rgba=(0.20, 0.37, 0.50, 0.72))
    metal_mat = model.material("brushed_metal", rgba=(0.48, 0.50, 0.49, 1.0))
    dark_mat = model.material("dark_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    fin_mat = model.material("tailplane_grey", rgba=(0.56, 0.58, 0.55, 1.0))
    safety_mat = model.material("safety_orange", rgba=(0.85, 0.25, 0.08, 1.0))

    hull = model.part("hull")

    # Thick, workhorse-scale envelope: a long center barrel with rounded end caps.
    hull.visual(
        Cylinder(radius=1.80, length=7.70),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=envelope_mat,
        name="envelope_barrel",
    )
    hull.visual(Sphere(radius=1.80), origin=Origin(xyz=(3.85, 0.0, 0.0)), material=envelope_mat, name="nose_cap")
    hull.visual(Sphere(radius=1.80), origin=Origin(xyz=(-3.85, 0.0, 0.0)), material=envelope_mat, name="tail_cap")
    hull.visual(
        Box((6.80, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -1.82)),
        material=seam_mat,
        name="keel_band",
    )

    # Compact gondola with surface windows. It hangs below the hull rather than
    # being fused into the envelope.
    hull.visual(
        Box((2.70, 0.95, 0.72)),
        origin=Origin(xyz=(0.10, 0.0, -2.38)),
        material=gondola_mat,
        name="gondola_cabin",
    )
    hull.visual(
        Box((0.46, 0.018, 0.22)),
        origin=Origin(xyz=(0.85, 0.484, -2.25)),
        material=glass_mat,
        name="side_window_0",
    )
    hull.visual(
        Box((0.46, 0.018, 0.22)),
        origin=Origin(xyz=(0.25, 0.484, -2.25)),
        material=glass_mat,
        name="side_window_1",
    )
    hull.visual(
        Box((0.46, 0.018, 0.22)),
        origin=Origin(xyz=(-0.35, 0.484, -2.25)),
        material=glass_mat,
        name="side_window_2",
    )
    hull.visual(
        Box((0.46, 0.018, 0.22)),
        origin=Origin(xyz=(0.85, -0.484, -2.25)),
        material=glass_mat,
        name="side_window_3",
    )
    hull.visual(
        Box((0.46, 0.018, 0.22)),
        origin=Origin(xyz=(0.25, -0.484, -2.25)),
        material=glass_mat,
        name="side_window_4",
    )
    hull.visual(
        Box((0.46, 0.018, 0.22)),
        origin=Origin(xyz=(-0.35, -0.484, -2.25)),
        material=glass_mat,
        name="side_window_5",
    )
    hull.visual(
        Box((0.52, 0.022, 0.23)),
        origin=Origin(xyz=(1.446, 0.0, -2.24), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=glass_mat,
        name="front_windscreen",
    )

    # Visible suspension struts from hull belly to gondola roof.
    for i, x in enumerate((-0.95, 0.95)):
        _add_strut(hull, f"cabin_strut_{i}_0", (x, -0.36, -2.02), (x * 0.75, -0.18, -1.66), 0.035, metal_mat)
        _add_strut(hull, f"cabin_strut_{i}_1", (x, 0.36, -2.02), (x * 0.75, 0.18, -1.66), 0.035, metal_mat)
    _add_strut(hull, "center_keel_strut", (0.0, 0.0, -2.02), (0.0, 0.0, -1.72), 0.045, metal_mat)

    # Twin engine pods and their outrigger struts.
    for i, y in enumerate((-2.22, 2.22)):
        side = -1.0 if y < 0.0 else 1.0
        hull.visual(
            Cylinder(radius=0.27, length=0.84),
            origin=Origin(xyz=(0.02, y, -1.22), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"engine_pod_{i}",
        )
        hull.visual(
            Cylinder(radius=0.19, length=0.10),
            origin=Origin(xyz=(-0.43, y, -1.22), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gondola_mat,
            name=f"engine_bearing_{i}",
        )
        _add_strut(
            hull,
            f"pod_upper_strut_{i}",
            (0.18, side * 1.32, -1.10),
            (0.08, side * 1.96, -1.08),
            0.035,
            metal_mat,
            extra=0.05,
        )
        _add_strut(
            hull,
            f"pod_lower_strut_{i}",
            (-0.18, side * 0.52, -2.10),
            (-0.05, side * 1.98, -1.39),
            0.032,
            metal_mat,
            extra=0.05,
        )

    # Boxy cruciform tail. Static forward portions are part of the hull; the
    # rear control surfaces are separate articulated parts.
    hull.visual(Box((0.78, 0.13, 1.05)), origin=Origin(xyz=(-5.04, 0.0, 1.92)), material=fin_mat, name="upper_fin")
    hull.visual(Box((0.76, 0.13, 0.82)), origin=Origin(xyz=(-5.02, 0.0, -1.76)), material=fin_mat, name="lower_fin")
    hull.visual(Box((0.76, 1.26, 0.10)), origin=Origin(xyz=(-5.05, -1.56, 0.0)), material=fin_mat, name="tailplane_0")
    hull.visual(Box((0.76, 1.26, 0.10)), origin=Origin(xyz=(-5.05, 1.56, 0.0)), material=fin_mat, name="tailplane_1")

    # Short support fork and axle under the rear of the gondola for the mooring wheel.
    hull.visual(Box((0.30, 0.42, 0.07)), origin=Origin(xyz=(-1.07, 0.0, -2.72)), material=metal_mat, name="wheel_fork_bridge")
    hull.visual(Box((0.16, 0.055, 0.36)), origin=Origin(xyz=(-1.07, -0.18, -2.88)), material=metal_mat, name="wheel_fork_cheek_0")
    hull.visual(Box((0.16, 0.055, 0.36)), origin=Origin(xyz=(-1.07, 0.18, -2.88)), material=metal_mat, name="wheel_fork_cheek_1")
    hull.visual(
        Cylinder(radius=0.024, length=0.47),
        origin=Origin(xyz=(-1.07, 0.0, -3.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="wheel_axle",
    )

    rotor_mesh_0 = mesh_from_geometry(
        FanRotorGeometry(
            0.42,
            0.105,
            4,
            thickness=0.070,
            blade_pitch_deg=34.0,
            blade_sweep_deg=25.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.12),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.028, rear_collar_radius=0.070, bore_diameter=0.030),
        ),
        "propeller_rotor_0",
    )
    rotor_mesh_1 = mesh_from_geometry(
        FanRotorGeometry(
            0.42,
            0.105,
            4,
            thickness=0.070,
            blade_pitch_deg=34.0,
            blade_sweep_deg=25.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.12),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.028, rear_collar_radius=0.070, bore_diameter=0.030),
        ),
        "propeller_rotor_1",
    )
    for i, (y, mesh) in enumerate(((-2.22, rotor_mesh_0), (2.22, rotor_mesh_1))):
        prop = model.part(f"propeller_{i}")
        prop.visual(
            Cylinder(radius=0.045, length=0.18),
            origin=Origin(xyz=(-0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="propeller_shaft",
        )
        prop.visual(
            mesh,
            origin=Origin(xyz=(-0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name="rotor_blades",
        )
        model.articulation(
            f"hull_to_propeller_{i}",
            ArticulationType.CONTINUOUS,
            parent=hull,
            child=prop,
            origin=Origin(xyz=(-0.43, y, -1.22)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=80.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        Box((0.52, 0.11, 0.96)),
        origin=Origin(xyz=(-0.26, 0.0, 0.0)),
        material=safety_mat,
        name="rudder_panel",
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-5.43, 0.0, 1.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.55, upper=0.55),
    )

    for i, y in enumerate((-1.56, 1.56)):
        elevator = model.part(f"elevator_{i}")
        elevator.visual(
            Box((0.54, 1.26, 0.095)),
            origin=Origin(xyz=(-0.27, 0.0, 0.0)),
            material=safety_mat,
            name="elevator_panel",
        )
        model.articulation(
            f"hull_to_elevator_{i}",
            ArticulationType.REVOLUTE,
            parent=hull,
            child=elevator,
            origin=Origin(xyz=(-5.43, y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=14.0, velocity=1.6, lower=-0.42, upper=0.42),
        )

    wheel_part = model.part("mooring_wheel")
    wheel_part.visual(
        Cylinder(radius=0.038, length=0.135),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="axle_sleeve",
    )
    wheel_part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.22,
                0.11,
                inner_radius=0.145,
                tread=TireTread(style="block", depth=0.010, count=18, land_ratio=0.58),
                grooves=(TireGroove(center_offset=0.0, width=0.012, depth=0.004),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "mooring_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_mat,
        name="tire",
    )
    wheel_part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.145,
                0.095,
                rim=WheelRim(inner_radius=0.092, flange_height=0.008, flange_thickness=0.004, bead_seat_depth=0.004),
                hub=WheelHub(
                    radius=0.040,
                    width=0.080,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.050, hole_diameter=0.006),
                ),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.014),
                bore=WheelBore(style="round", diameter=0.070),
            ),
            "mooring_wheel_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal_mat,
        name="rim",
    )
    model.articulation(
        "hull_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=wheel_part,
        origin=Origin(xyz=(-1.07, 0.0, -3.01)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    rudder = object_model.get_part("rudder")
    elevator_0 = object_model.get_part("elevator_0")
    elevator_1 = object_model.get_part("elevator_1")
    wheel = object_model.get_part("mooring_wheel")

    prop_joint_0 = object_model.get_articulation("hull_to_propeller_0")
    prop_joint_1 = object_model.get_articulation("hull_to_propeller_1")
    rudder_joint = object_model.get_articulation("hull_to_rudder")
    elevator_joint_0 = object_model.get_articulation("hull_to_elevator_0")
    elevator_joint_1 = object_model.get_articulation("hull_to_elevator_1")
    wheel_joint = object_model.get_articulation("hull_to_mooring_wheel")

    # The propeller shafts are intentionally captured in the pod bearing faces.
    for i in (0, 1):
        ctx.allow_overlap(
            "hull",
            f"propeller_{i}",
            elem_a=f"engine_bearing_{i}",
            elem_b="propeller_shaft",
            reason="The rotating propeller shaft is seated in the engine pod bearing.",
        )
        ctx.expect_gap(
            "hull",
            f"propeller_{i}",
            axis="x",
            positive_elem=f"engine_bearing_{i}",
            negative_elem="propeller_shaft",
            max_penetration=0.055,
            name=f"propeller_{i} shaft is locally captured by its pod bearing",
        )

    # The mooring wheel spins around a fixed axle running through its sleeve.
    ctx.allow_overlap(
        hull,
        wheel,
        elem_a="wheel_axle",
        elem_b="axle_sleeve",
        reason="The fixed axle intentionally passes through the rotating wheel sleeve.",
    )
    ctx.expect_overlap(
        hull,
        wheel,
        axes="y",
        elem_a="wheel_axle",
        elem_b="axle_sleeve",
        min_overlap=0.12,
        name="mooring wheel sleeve is retained on the axle",
    )
    ctx.expect_within(
        wheel,
        hull,
        axes="y",
        inner_elem="tire",
        outer_elem="wheel_fork_bridge",
        margin=0.0,
        name="mooring wheel sits between the fork cheeks",
    )

    ctx.expect_gap(
        hull,
        rudder,
        axis="x",
        positive_elem="upper_fin",
        negative_elem="rudder_panel",
        max_gap=0.001,
        max_penetration=0.001,
        name="rudder front edge sits on the vertical fin hinge line",
    )
    for i, elevator in enumerate((elevator_0, elevator_1)):
        ctx.expect_gap(
            hull,
            elevator,
            axis="x",
            positive_elem=f"tailplane_{i}",
            negative_elem="elevator_panel",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"elevator_{i} front edge sits on its horizontal hinge line",
        )

    ctx.check(
        "continuous rotors and wheel",
        prop_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and prop_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={(prop_joint_0.articulation_type, prop_joint_1.articulation_type, wheel_joint.articulation_type)}",
    )
    ctx.check(
        "tail control limits are bidirectional",
        rudder_joint.motion_limits.lower < 0.0
        and rudder_joint.motion_limits.upper > 0.0
        and elevator_joint_0.motion_limits.lower < 0.0
        and elevator_joint_0.motion_limits.upper > 0.0
        and elevator_joint_1.motion_limits.lower < 0.0
        and elevator_joint_1.motion_limits.upper > 0.0,
        details="rudder and elevators should deflect both directions from neutral",
    )

    rest_rudder_aabb = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_joint: 0.45}):
        yawed_rudder_aabb = ctx.part_world_aabb(rudder)
    ctx.check(
        "rudder swings sideways about vertical hinge",
        rest_rudder_aabb is not None
        and yawed_rudder_aabb is not None
        and yawed_rudder_aabb[0][1] < rest_rudder_aabb[0][1] - 0.05,
        details=f"rest={rest_rudder_aabb}, yawed={yawed_rudder_aabb}",
    )

    rest_elevator_aabb = ctx.part_world_aabb(elevator_0)
    with ctx.pose({elevator_joint_0: 0.35}):
        raised_elevator_aabb = ctx.part_world_aabb(elevator_0)
    ctx.check(
        "elevator pitches about horizontal hinge",
        rest_elevator_aabb is not None
        and raised_elevator_aabb is not None
        and raised_elevator_aabb[1][2] > rest_elevator_aabb[1][2] + 0.04,
        details=f"rest={rest_elevator_aabb}, raised={raised_elevator_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
