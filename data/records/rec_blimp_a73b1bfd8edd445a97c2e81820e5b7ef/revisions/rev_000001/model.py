from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
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
    LatheGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _filleted_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    solid = cq.Workplane("XY").box(*size).edges().fillet(radius)
    return mesh_from_cadquery(solid, name, tolerance=0.004, angular_tolerance=0.18)


def _add_cross_decal(
    part,
    *,
    center: tuple[float, float, float],
    side_sign: float,
    material: Material,
    prefix: str,
    scale: float = 1.0,
) -> None:
    x, y, z = center
    part.visual(
        Box((0.72 * scale, 0.10, 0.20 * scale)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=f"{prefix}_cross_bar",
    )
    part.visual(
        Box((0.22 * scale, 0.10, 0.72 * scale)),
        origin=Origin(xyz=(x, y + side_sign * 0.002, z)),
        material=material,
        name=f"{prefix}_cross_upright",
    )


def _build_hull_mesh():
    # Revolved about local +Z, then mounted with local Z along world +X.
    # The fuller, slightly shouldered center and tapered ends read as a
    # non-rigid-envelope rescue blimp rather than a plain ellipsoid.
    profile = [
        (0.00, -9.10),
        (0.36, -8.70),
        (0.88, -7.85),
        (1.36, -6.65),
        (1.74, -5.05),
        (1.98, -2.90),
        (2.08, 0.00),
        (2.00, 2.90),
        (1.76, 5.15),
        (1.28, 6.90),
        (0.70, 8.10),
        (0.20, 8.82),
        (0.00, 9.10),
    ]
    return LatheGeometry(profile, segments=96)


def _build_pod_mesh():
    return CapsuleGeometry(radius=0.44, length=0.95, radial_segments=48, height_segments=12)


def _build_propeller_mesh():
    return FanRotorGeometry(
        outer_radius=0.55,
        hub_radius=0.135,
        blade_count=4,
        thickness=0.075,
        blade_pitch_deg=32.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.18),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.030, rear_collar_radius=0.13),
    )


def _build_wheel_meshes():
    wheel = WheelGeometry(
        radius=0.145,
        width=0.090,
        rim=WheelRim(inner_radius=0.080, flange_height=0.008, flange_thickness=0.004),
        hub=WheelHub(radius=0.040, width=0.076, cap_style="domed"),
        face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.020),
        bore=WheelBore(style="round", diameter=0.020),
    )
    tire = TireGeometry(
        outer_radius=0.230,
        width=0.115,
        inner_radius=0.146,
        tread=TireTread(style="ribbed", depth=0.008, count=18, land_ratio=0.62),
        grooves=(TireGroove(center_offset=0.0, width=0.012, depth=0.004),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.008, radius=0.004),
    )
    return wheel, tire


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rescue_service_blimp")

    hull_fabric = model.material("hull_fabric", rgba=(0.94, 0.96, 0.93, 1.0))
    rescue_red = model.material("rescue_red", rgba=(0.82, 0.04, 0.04, 1.0))
    navy = model.material("rescue_navy", rgba=(0.03, 0.08, 0.16, 1.0))
    window_blue = model.material("window_blue", rgba=(0.10, 0.28, 0.42, 0.78))
    light_gray = model.material("light_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    prop_dark = model.material("prop_dark", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    metal = model.material("brushed_metal", rgba=(0.58, 0.62, 0.66, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        _mesh(_build_hull_mesh(), "long_hull"),
        origin=Origin(xyz=(0.0, 0.0, 4.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hull_fabric,
        name="long_hull",
    )

    # Red rescue-service side stripes and crosses, slightly proud of the hull.
    for sign, label in ((1.0, "starboard"), (-1.0, "port")):
        airframe.visual(
            Box((10.6, 0.050, 0.18)),
            origin=Origin(xyz=(-0.35, sign * 2.045, 4.05)),
            material=rescue_red,
            name=f"{label}_hull_stripe",
        )
        _add_cross_decal(
            airframe,
            center=(2.2, sign * 1.98, 4.62),
            side_sign=sign,
            material=rescue_red,
            prefix=f"{label}_hull",
            scale=1.10,
        )

    # Medium gondola/cabin under the envelope, with real struts to the hull.
    airframe.visual(
        Box((4.25, 1.28, 1.12)),
        origin=Origin(xyz=(0.15, 0.0, 1.18)),
        material=light_gray,
        name="gondola_shell",
    )
    airframe.visual(
        Box((3.60, 0.10, 0.28)),
        origin=Origin(xyz=(0.23, 0.66, 1.38)),
        material=window_blue,
        name="window_band_0",
    )
    airframe.visual(
        Box((3.60, 0.10, 0.28)),
        origin=Origin(xyz=(0.23, -0.66, 1.38)),
        material=window_blue,
        name="window_band_1",
    )
    for x in (-1.30, -0.25, 0.85, 1.75):
        airframe.visual(
            Box((0.045, 1.38, 0.34)),
            origin=Origin(xyz=(x, 0.0, 1.40)),
            material=light_gray,
            name=f"window_mullion_{x:+.2f}",
        )
    _add_cross_decal(
        airframe,
        center=(-0.25, 0.683, 1.03),
        side_sign=1.0,
        material=rescue_red,
        prefix="gondola_starboard",
        scale=0.62,
    )
    _add_cross_decal(
        airframe,
        center=(-0.25, -0.683, 1.03),
        side_sign=-1.0,
        material=rescue_red,
        prefix="gondola_port",
        scale=0.62,
    )
    for index, x in enumerate((-1.55, -0.45, 0.70, 1.65)):
        airframe.visual(
            Box((0.16, 0.16, 0.62)),
            origin=Origin(xyz=(x, 0.0, 1.90)),
            material=dark_gray,
            name=f"cabin_pylon_{index}",
        )
        airframe.visual(
            _mesh(
                tube_from_spline_points(
                    [(x, -0.38, 1.76), (x, -0.28, 1.96), (x, -0.18, 2.08)],
                    radius=0.045,
                    samples_per_segment=6,
                    radial_segments=18,
                ),
                f"cabin_strut_{index}_a",
            ),
            material=dark_gray,
            name=f"cabin_strut_{index}_a",
        )
        airframe.visual(
            _mesh(
                tube_from_spline_points(
                    [(x, 0.38, 1.76), (x, 0.28, 1.96), (x, 0.18, 2.08)],
                    radius=0.045,
                    samples_per_segment=6,
                    radial_segments=18,
                ),
                f"cabin_strut_{index}_b",
            ),
            material=dark_gray,
            name=f"cabin_strut_{index}_b",
        )

    # Twin propulsion pod housings on short pylons.
    pod_mesh = _mesh(_build_pod_mesh(), "propulsion_pod")
    for index, sign in enumerate((1.0, -1.0)):
        airframe.visual(
            pod_mesh,
            origin=Origin(xyz=(-0.20, sign * 2.72, 2.32), rpy=(0.0, pi / 2.0, 0.0)),
            material=navy,
            name=f"pod_{index}",
        )
        airframe.visual(
            Cylinder(radius=0.075, length=0.235),
            origin=Origin(xyz=(0.83, sign * 2.72, 2.32), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="pod_shaft_0" if index == 0 else "pod_shaft_1",
        )
        airframe.visual(
            Box((0.28, 0.34, 0.32)),
            origin=Origin(xyz=(-0.22, sign * 2.43, 2.32)),
            material=navy,
            name=f"pod_root_fairing_{index}",
        )
        airframe.visual(
            Box((0.20, 1.90, 0.28)),
            origin=Origin(xyz=(-0.22, sign * 1.55, 2.17)),
            material=dark_gray,
            name=f"pod_support_beam_{index}",
        )
        airframe.visual(
            _mesh(
                tube_from_spline_points(
                    [
                        (-0.45, sign * 0.67, 1.62),
                        (-0.36, sign * 1.42, 1.96),
                        (-0.28, sign * 2.30, 2.23),
                        (-0.20, sign * 2.72, 2.32),
                    ],
                    radius=0.055,
                    samples_per_segment=9,
                    radial_segments=18,
                ),
                f"pod_pylon_{index}",
            ),
            material=dark_gray,
            name=f"pod_pylon_{index}",
        )

    # Reinforced cruciform tail: fixed thick panels plus tubular spars.
    airframe.visual(
        _filleted_box_mesh((1.55, 0.20, 2.15), 0.055, "upper_tail_fin"),
        origin=Origin(xyz=(-8.25, 0.0, 5.48)),
        material=hull_fabric,
        name="upper_tail_fin",
    )
    airframe.visual(
        _filleted_box_mesh((1.30, 0.19, 1.48), 0.050, "lower_tail_fin"),
        origin=Origin(xyz=(-8.15, 0.0, 2.70)),
        material=hull_fabric,
        name="lower_tail_fin",
    )
    airframe.visual(
        _filleted_box_mesh((1.25, 2.75, 0.18), 0.055, "tailplane"),
        origin=Origin(xyz=(-8.18, 1.18, 4.00)),
        material=hull_fabric,
        name="tailplane_0",
    )
    airframe.visual(
        _filleted_box_mesh((1.25, 2.75, 0.18), 0.055, "tailplane"),
        origin=Origin(xyz=(-8.18, -1.18, 4.00)),
        material=hull_fabric,
        name="tailplane_1",
    )
    for index, pts in enumerate(
        (
            [(-8.55, 0.00, 4.30), (-8.10, 0.00, 5.10), (-7.70, 0.00, 6.20)],
            [(-8.55, 0.00, 3.70), (-8.12, 0.00, 3.05), (-7.82, 0.00, 2.10)],
            [(-8.55, 0.12, 4.00), (-8.15, 0.90, 4.02), (-7.82, 2.30, 4.04)],
            [(-8.55, -0.12, 4.00), (-8.15, -0.90, 4.02), (-7.82, -2.30, 4.04)],
        )
    ):
        airframe.visual(
            _mesh(
                tube_from_spline_points(
                    pts,
                    radius=0.045,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
                f"tail_reinforcement_{index}",
            ),
            material=dark_gray,
            name=f"tail_reinforcement_{index}",
        )
    airframe.visual(
        Cylinder(radius=0.040, length=3.10),
        origin=Origin(xyz=(-9.20, 0.0, 4.18)),
        material=dark_gray,
        name="rudder_hinge_post",
    )
    for index, z in enumerate((2.72, 5.62)):
        airframe.visual(
            Box((0.50, 0.10, 0.16)),
            origin=Origin(xyz=(-9.02, 0.0, z)),
            material=dark_gray,
            name=f"rudder_hinge_lug_{index}",
        )
    airframe.visual(
        Cylinder(radius=0.045, length=2.30),
        origin=Origin(xyz=(-8.86, 1.30, 4.00), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="elevator_hinge_0",
    )
    airframe.visual(
        Cylinder(radius=0.045, length=2.30),
        origin=Origin(xyz=(-8.86, -1.30, 4.00), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="elevator_hinge_1",
    )

    # Short support fork and axle stubs for the rear mooring wheel.
    airframe.visual(
        Box((0.64, 0.50, 0.095)),
        origin=Origin(xyz=(-1.92, 0.0, 0.62)),
        material=dark_gray,
        name="wheel_fork_bridge",
    )
    airframe.visual(
        Box((0.30, 0.24, 0.34)),
        origin=Origin(xyz=(-1.92, 0.0, 0.78)),
        material=dark_gray,
        name="wheel_fork_post",
    )
    airframe.visual(
        Box((0.14, 0.065, 0.45)),
        origin=Origin(xyz=(-1.92, 0.18, 0.405)),
        material=dark_gray,
        name="wheel_fork_arm_0",
    )
    airframe.visual(
        Box((0.14, 0.065, 0.45)),
        origin=Origin(xyz=(-1.92, -0.18, 0.405)),
        material=dark_gray,
        name="wheel_fork_arm_1",
    )
    airframe.visual(
        Cylinder(radius=0.022, length=0.44),
        origin=Origin(xyz=(-1.92, 0.0, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_axle",
    )
    airframe.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(-1.92, 0.115, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_axle_stub_0",
    )
    airframe.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(-1.92, -0.115, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_axle_stub_1",
    )

    propeller_mesh = _mesh(_build_propeller_mesh(), "propeller_rotor")
    for index, sign in enumerate((1.0, -1.0)):
        propeller = model.part(f"propeller_{index}")
        propeller.visual(
            propeller_mesh,
            origin=Origin(),
            material=prop_dark,
            name="rotor",
        )
        model.articulation(
            f"propeller_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=propeller,
            origin=Origin(xyz=(0.975, sign * 2.72, 2.32), rpy=(0.0, pi / 2.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=28.0, velocity=90.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        _filleted_box_mesh((0.72, 0.16, 2.28), 0.050, "rudder_panel"),
        origin=Origin(xyz=(-0.405, 0.0, 0.0)),
        material=rescue_red,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.052, length=2.34),
        origin=Origin(),
        material=dark_gray,
        name="rudder_hinge_knuckle",
    )
    model.articulation(
        "rudder_yaw",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-9.20, 0.0, 4.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.6, lower=-0.55, upper=0.55),
    )

    for index, sign in enumerate((1.0, -1.0)):
        elevator = model.part(f"elevator_{index}")
        elevator.visual(
            _filleted_box_mesh((0.66, 1.16, 0.16), 0.045, f"elevator_panel_{index}"),
            origin=Origin(xyz=(-0.36, sign * 0.58, 0.0)),
            material=rescue_red,
            name="elevator_panel",
        )
        elevator.visual(
            Cylinder(radius=0.043, length=1.17),
            origin=Origin(xyz=(0.0, sign * 0.585, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name="elevator_hinge_knuckle",
        )
        model.articulation(
            f"elevator_pitch_{index}",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=elevator,
            origin=Origin(xyz=(-8.91, sign * 0.72, 4.00)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=190.0, velocity=1.7, lower=-0.45, upper=0.45),
        )

    wheel_geom, tire_geom = _build_wheel_meshes()
    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        _mesh(tire_geom, "mooring_tire"),
        material=rubber,
        name="tire",
    )
    mooring_wheel.visual(
        _mesh(wheel_geom, "mooring_wheel"),
        material=metal,
        name="wheel",
    )
    model.articulation(
        "mooring_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=mooring_wheel,
        origin=Origin(xyz=(-1.92, 0.0, 0.25), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    rudder = object_model.get_part("rudder")
    elevator_0 = object_model.get_part("elevator_0")
    elevator_1 = object_model.get_part("elevator_1")
    propeller_0 = object_model.get_part("propeller_0")
    propeller_1 = object_model.get_part("propeller_1")
    mooring_wheel = object_model.get_part("mooring_wheel")

    prop_spin_0 = object_model.get_articulation("propeller_spin_0")
    rudder_yaw = object_model.get_articulation("rudder_yaw")
    elevator_pitch_0 = object_model.get_articulation("elevator_pitch_0")
    wheel_spin = object_model.get_articulation("mooring_wheel_spin")

    ctx.allow_overlap(
        airframe,
        propeller_0,
        elem_a="pod_shaft_0",
        elem_b="rotor",
        reason="The propeller hub is intentionally captured on the visible pod shaft.",
    )
    ctx.allow_overlap(
        airframe,
        propeller_1,
        elem_a="pod_shaft_1",
        elem_b="rotor",
        reason="The propeller hub is intentionally captured on the visible pod shaft.",
    )
    ctx.allow_overlap(
        airframe,
        rudder,
        elem_a="rudder_hinge_post",
        elem_b="rudder_hinge_knuckle",
        reason="The rudder hinge knuckle is coaxially captured on the tail hinge post.",
    )
    ctx.allow_overlap(
        airframe,
        elevator_0,
        elem_a="elevator_hinge_0",
        elem_b="elevator_hinge_knuckle",
        reason="The elevator hinge knuckle is coaxially captured on the horizontal tail hinge.",
    )
    ctx.allow_overlap(
        airframe,
        elevator_1,
        elem_a="elevator_hinge_1",
        elem_b="elevator_hinge_knuckle",
        reason="The elevator hinge knuckle is coaxially captured on the horizontal tail hinge.",
    )
    ctx.allow_overlap(
        airframe,
        mooring_wheel,
        elem_a="wheel_axle",
        elem_b="wheel",
        reason="The mooring wheel hub is intentionally retained by the fork axle through its bore.",
    )

    ctx.expect_gap(
        propeller_0,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=0.02,
        positive_elem="rotor",
        negative_elem="pod_shaft_0",
        name="propeller hub is seated on the pod shaft",
    )
    ctx.expect_gap(
        propeller_1,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=0.02,
        positive_elem="rotor",
        negative_elem="pod_shaft_1",
        name="second propeller hub is seated on its pod shaft",
    )
    ctx.expect_gap(
        rudder,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=0.12,
        positive_elem="rudder_hinge_knuckle",
        negative_elem="rudder_hinge_post",
        name="rudder hinge knuckle stays captured on the post",
    )
    ctx.expect_gap(
        elevator_0,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=0.15,
        positive_elem="elevator_hinge_knuckle",
        negative_elem="elevator_hinge_0",
        name="elevator hinge knuckle stays captured on the root hinge",
    )
    ctx.expect_gap(
        mooring_wheel,
        airframe,
        axis="y",
        max_gap=0.001,
        max_penetration=0.30,
        positive_elem="wheel",
        negative_elem="wheel_axle",
        name="mooring wheel hub is retained by its axle",
    )
    ctx.expect_within(
        mooring_wheel,
        airframe,
        axes="y",
        margin=0.04,
        inner_elem="tire",
        outer_elem="wheel_fork_bridge",
        name="mooring wheel is centered inside the short support fork",
    )

    rudder_rest = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_yaw: 0.45}):
        rudder_deflected = ctx.part_world_aabb(rudder)
    ctx.check(
        "rudder rotates about a vertical tail hinge",
        rudder_rest is not None
        and rudder_deflected is not None
        and (rudder_deflected[1][1] - rudder_deflected[0][1])
        > (rudder_rest[1][1] - rudder_rest[0][1]) + 0.15,
        details=f"rest={rudder_rest}, deflected={rudder_deflected}",
    )

    elevator_rest = ctx.part_world_aabb(elevator_0)
    with ctx.pose({elevator_pitch_0: 0.35}):
        elevator_deflected = ctx.part_world_aabb(elevator_0)
    ctx.check(
        "elevator panel pitches on a horizontal hinge",
        elevator_rest is not None
        and elevator_deflected is not None
        and (elevator_deflected[1][2] - elevator_deflected[0][2])
        > (elevator_rest[1][2] - elevator_rest[0][2]) + 0.05,
        details=f"rest={elevator_rest}, deflected={elevator_deflected}",
    )

    prop_rest = ctx.part_world_aabb(propeller_0)
    wheel_rest = ctx.part_world_aabb(mooring_wheel)
    with ctx.pose({prop_spin_0: pi / 3.0, wheel_spin: pi / 2.0}):
        prop_spun = ctx.part_world_aabb(propeller_0)
        wheel_spun = ctx.part_world_aabb(mooring_wheel)
    ctx.check(
        "continuous rotors stay captured on their shafts",
        prop_rest is not None
        and prop_spun is not None
        and wheel_rest is not None
        and wheel_spun is not None
        and abs(prop_spun[0][0] - prop_rest[0][0]) < 0.02
        and abs(wheel_spun[0][1] - wheel_rest[0][1]) < 0.02,
        details=f"prop_rest={prop_rest}, prop_spun={prop_spun}, wheel_rest={wheel_rest}, wheel_spun={wheel_spun}",
    )

    return ctx.report()


object_model = build_object_model()
