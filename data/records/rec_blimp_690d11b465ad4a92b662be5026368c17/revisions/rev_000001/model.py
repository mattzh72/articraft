from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
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
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _make_envelope_mesh():
    """Long, gently tapered observation-airship gas envelope."""
    stations = [
        (-24.0, 0.22, 0.22, 9.05),
        (-21.5, 1.55, 1.45, 9.05),
        (-17.0, 3.45, 3.25, 9.08),
        (-10.0, 4.55, 4.20, 9.15),
        (-2.0, 5.10, 4.55, 9.18),
        (7.0, 5.25, 4.65, 9.16),
        (15.0, 4.35, 3.85, 9.08),
        (21.0, 2.10, 1.95, 9.02),
        (24.0, 0.24, 0.24, 9.00),
    ]
    loops = []
    for x, radius_y, radius_z, center_z in stations:
        loop = []
        for i in range(72):
            theta = 2.0 * math.pi * i / 72
            loop.append((x, radius_y * math.cos(theta), center_z + radius_z * math.sin(theta)))
        loops.append(loop)
    return section_loft(loops)


def _make_gondola_mesh():
    """Small rounded observation gondola with a blunt glazed bow."""
    stations = [
        (-3.70, 0.12, 0.16, -1.05),
        (-3.15, 0.58, 0.60, -1.08),
        (-2.20, 1.02, 0.98, -1.05),
        (-0.60, 1.15, 1.10, -1.02),
        (1.35, 1.09, 1.05, -1.00),
        (2.85, 0.78, 0.82, -0.98),
        (3.55, 0.20, 0.24, -0.98),
    ]
    loops = []
    for x, radius_y, radius_z, center_z in stations:
        loop = []
        for i in range(48):
            theta = 2.0 * math.pi * i / 48
            # Slightly flatter bottom than top, typical of small gondola shells.
            z_scale = 0.88 if math.sin(theta) < 0.0 else 1.0
            loop.append((x, radius_y * math.cos(theta), center_z + radius_z * z_scale * math.sin(theta)))
        loops.append(loop)
    return section_loft(loops)


def _fin_mesh(name: str, profile: list[tuple[float, float]], thickness: float = 0.14):
    # Profile is authored in the local X/Z plane; rotate the extrusion so its
    # thickness is along local Y.
    geom = ExtrudeGeometry(profile, thickness, center=True).rotate_x(math.pi / 2.0)
    return _mesh(name, geom)


def _add_tube(part, name: str, points, radius: float, material) -> None:
    part.visual(
        _mesh(name, tube_from_spline_points(points, radius=radius, samples_per_segment=2, radial_segments=16)),
        material=material,
        name=name,
    )


def _add_engine(model, gondola, *, side: float, grey, steel, dark, prop_mat) -> None:
    side_name = "left" if side > 0.0 else "right"
    engine = model.part(f"{side_name}_engine")
    # Nacelle local frame: +X forward, +/-Y outboard, Z up.
    engine.visual(
        Cylinder(radius=0.44, length=1.45),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="nacelle_shell",
    )
    engine.visual(
        Cylinder(radius=0.22, length=0.25),
        origin=Origin(xyz=(0.83, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="intake_lip",
    )
    engine.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(-0.74, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="prop_shaft",
    )
    # Two faired struts carry the side-mounted nacelle back to the gondola.
    _add_tube(
        engine,
        f"{side_name}_engine_upper_strut",
        [(-0.25, 0.02, 0.30), (-0.55, -side * 1.58, -0.02)],
        0.045,
        steel,
    )
    _add_tube(
        engine,
        f"{side_name}_engine_lower_strut",
        [(0.30, 0.02, -0.26), (-0.20, -side * 1.52, -0.78)],
        0.040,
        steel,
    )
    engine.visual(
        Box((0.70, 0.10, 0.16)),
        origin=Origin(xyz=(-0.36, -side * 1.35, -0.04)),
        material=steel,
        name="mount_pad",
    )

    model.articulation(
        f"gondola_to_{side_name}_engine",
        ArticulationType.FIXED,
        parent=gondola,
        child=engine,
        origin=Origin(xyz=(-1.55, side * 2.58, -0.75)),
    )

    prop = model.part(f"{side_name}_propeller")
    prop.visual(
        _mesh(
            f"{side_name}_propeller_rotor",
            FanRotorGeometry(
                0.82,
                0.16,
                4,
                thickness=0.095,
                blade_pitch_deg=34.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.18),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.035, rear_collar_radius=0.12),
            ),
        ),
        material=prop_mat,
        name="rotor",
    )
    # The joint frame rotates local +Z into the nacelle's longitudinal +X axis.
    model.articulation(
        f"{side_name}_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=engine,
        child=prop,
        origin=Origin(xyz=(-0.885, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=90.0),
    )


def _add_wheel(model, gondola, *, side: float, rubber, wheel_mat, dark) -> None:
    side_name = "left" if side > 0.0 else "right"
    wheel = model.part(f"{side_name}_wheel")
    tire = TireGeometry(
        0.42,
        0.24,
        inner_radius=0.30,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="ribbed", depth=0.018, count=20, land_ratio=0.64),
        grooves=(TireGroove(center_offset=0.0, width=0.018, depth=0.006),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.020, radius=0.010),
    )
    wheel.visual(
        _mesh(f"{side_name}_wheel_tire", tire),
        # Wheel/Tire helpers spin about local X. Rotate that local X to the
        # child-frame Y axle so the continuous joint turns the tire correctly.
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    rim = WheelGeometry(
        0.30,
        0.22,
        rim=WheelRim(inner_radius=0.18, flange_height=0.025, flange_thickness=0.010),
        hub=WheelHub(radius=0.095, width=0.18, cap_style="domed"),
        face=WheelFace(dish_depth=0.018, front_inset=0.006, rear_inset=0.006),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.012, window_radius=0.036),
        bore=WheelBore(style="round", diameter=0.045),
    )
    wheel.visual(
        _mesh(f"{side_name}_wheel_rim", rim),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=wheel_mat,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.085, length=0.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hub_cap",
    )

    model.articulation(
        f"{side_name}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=wheel,
        origin=Origin(xyz=(-0.65, side * 1.16, -3.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=45.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_blimp")

    envelope_fabric = model.material("envelope_fabric", rgba=(0.86, 0.88, 0.84, 1.0))
    blue = model.material("blue_marking", rgba=(0.08, 0.25, 0.55, 1.0))
    gondola_white = model.material("gondola_white", rgba=(0.88, 0.90, 0.88, 1.0))
    glass = model.material("blue_glass", rgba=(0.25, 0.55, 0.72, 0.46))
    dark = model.material("dark_composite", rgba=(0.07, 0.08, 0.09, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    nacelle_grey = model.material("nacelle_grey", rgba=(0.48, 0.50, 0.50, 1.0))
    prop_black = model.material("prop_black", rgba=(0.035, 0.035, 0.035, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    wheel_aluminum = model.material("wheel_aluminum", rgba=(0.72, 0.73, 0.72, 1.0))
    control_fabric = model.material("tail_fabric", rgba=(0.78, 0.80, 0.78, 1.0))

    envelope = model.part("envelope")
    envelope.visual(_mesh("long_surveillance_envelope", _make_envelope_mesh()), material=envelope_fabric, name="hull")
    # A blue service stripe and belly keel make the scale and orientation legible.
    envelope.visual(
        Box((21.0, 0.035, 0.45)),
        origin=Origin(xyz=(2.0, 5.12, 8.55)),
        material=blue,
        name="side_stripe",
    )
    envelope.visual(
        Box((21.0, 0.035, 0.45)),
        origin=Origin(xyz=(2.0, -5.12, 8.55)),
        material=blue,
        name="side_stripe_mirror",
    )
    envelope.visual(
        Cylinder(radius=0.10, length=18.0),
        origin=Origin(xyz=(4.8, 0.0, 4.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="keel_rail",
    )
    envelope.visual(
        Cylinder(radius=0.080, length=0.92),
        origin=Origin(xyz=(-24.30, 0.0, 9.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tail_stinger",
    )
    envelope.visual(
        Cylinder(radius=0.065, length=1.15),
        origin=Origin(xyz=(-23.62, 0.0, 9.66)),
        material=steel,
        name="vertical_tail_pylon",
    )
    # Four V-struts hang the gondola from the envelope and keep the cabin visibly
    # supported instead of floating.
    _add_tube(envelope, "front_keel_strut_0", [(7.2, -0.82, 4.72), (7.1, -0.46, 3.72)], 0.060, steel)
    _add_tube(envelope, "front_keel_strut_1", [(7.2, 0.82, 4.72), (7.1, 0.46, 3.72)], 0.060, steel)
    _add_tube(envelope, "rear_keel_strut_0", [(2.2, -0.82, 4.70), (2.1, -0.46, 3.72)], 0.060, steel)
    _add_tube(envelope, "rear_keel_strut_1", [(2.2, 0.82, 4.70), (2.1, 0.46, 3.72)], 0.060, steel)

    gondola = model.part("gondola")
    gondola.visual(_mesh("sensor_gondola_shell", _make_gondola_mesh()), material=gondola_white, name="shell")
    gondola.visual(
        Box((6.25, 1.18, 0.060)),
        origin=Origin(xyz=(-0.35, 0.0, -0.030)),
        material=steel,
        name="roof_hardpoint",
    )
    gondola.visual(
        Box((1.45, 1.02, 0.12)),
        origin=Origin(xyz=(-0.65, 0.0, -2.04)),
        material=steel,
        name="gear_mount_plate",
    )
    gondola.visual(
        Box((1.35, 2.24, 0.42)),
        origin=Origin(xyz=(2.55, 0.0, -0.74)),
        material=glass,
        name="bow_observation_glass",
    )
    for x in (-1.25, -0.20, 0.85):
        gondola.visual(
            Box((0.55, 0.035, 0.52)),
            origin=Origin(xyz=(x, 1.12, -0.73)),
            material=glass,
            name=f"window_{x:.1f}_a",
        )
        gondola.visual(
            Box((0.55, 0.035, 0.52)),
            origin=Origin(xyz=(x, -1.12, -0.73)),
            material=glass,
            name=f"window_{x:.1f}_b",
        )
    # Fixed EO/IR sensor ball under the observation cabin.
    gondola.visual(Cylinder(radius=0.13, length=0.45), origin=Origin(xyz=(1.15, 0.0, -2.05)), material=dark, name="sensor_neck")
    gondola.visual(Cylinder(radius=0.34, length=0.40), origin=Origin(xyz=(1.15, 0.0, -2.38)), material=dark, name="sensor_turret")
    gondola.visual(Box((0.22, 0.08, 0.14)), origin=Origin(xyz=(1.48, 0.0, -2.38)), material=glass, name="sensor_window")
    # Real undercarriage: cross beam, splayed struts, and fork plates at each wheel.
    gondola.visual(
        Cylinder(radius=0.055, length=2.40),
        origin=Origin(xyz=(-0.65, 0.0, -2.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="gear_cross_axle",
    )
    for side in (-1.0, 1.0):
        _add_tube(gondola, f"gear_front_strut_{side:+.0f}", [(-0.10, side * 0.40, -2.05), (-0.65, side * 0.98, -3.02)], 0.045, steel)
        _add_tube(gondola, f"gear_rear_strut_{side:+.0f}", [(-1.24, side * 0.40, -2.05), (-0.65, side * 0.98, -3.02)], 0.045, steel)
        gondola.visual(
            Box((0.10, 0.035, 0.58)),
            origin=Origin(xyz=(-0.65, side * 1.00, -3.08)),
            material=steel,
            name=f"wheel_fork_inner_{side:+.0f}",
        )
        gondola.visual(
            Box((0.10, 0.035, 0.58)),
            origin=Origin(xyz=(-0.65, side * 1.36, -3.08)),
            material=steel,
            name=f"wheel_fork_outer_{side:+.0f}",
        )
        gondola.visual(
            Box((0.12, 0.46, 0.090)),
            origin=Origin(xyz=(-0.65, side * 1.155, -2.745)),
            material=steel,
            name=f"wheel_fork_bridge_{side:+.0f}",
        )
        gondola.visual(
            Cylinder(radius=0.052, length=0.44),
            origin=Origin(xyz=(-0.65, side * 1.16, -3.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"axle_stub_{side:+.0f}",
        )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(5.0, 0.0, 3.72)),
    )

    _add_engine(model, gondola, side=1.0, grey=nacelle_grey, steel=steel, dark=dark, prop_mat=prop_black)
    _add_engine(model, gondola, side=-1.0, grey=nacelle_grey, steel=steel, dark=dark, prop_mat=prop_black)
    _add_wheel(model, gondola, side=1.0, rubber=rubber, wheel_mat=wheel_aluminum, dark=dark)
    _add_wheel(model, gondola, side=-1.0, rubber=rubber, wheel_mat=wheel_aluminum, dark=dark)

    vertical_tail = model.part("vertical_tail")
    vertical_tail.visual(
        _fin_mesh("vertical_fixed_fin", [(0.02, 0.00), (1.45, 0.34), (1.60, 2.04), (0.02, 2.58)]),
        material=control_fabric,
        name="fin_panel",
    )
    vertical_tail.visual(
        Cylinder(radius=0.055, length=3.45),
        origin=Origin(xyz=(0.035, 0.0, 1.22)),
        material=steel,
        name="rudder_hinge_post",
    )
    model.articulation(
        "envelope_to_vertical_tail",
        ArticulationType.FIXED,
        parent=envelope,
        child=vertical_tail,
        origin=Origin(xyz=(-23.78, 0.0, 10.20)),
    )

    rudder = model.part("rudder")
    rudder.visual(
        _fin_mesh("rudder_control_surface", [(-1.18, 0.10), (-0.04, 0.08), (-0.04, 2.36), (-1.05, 1.95)]),
        material=control_fabric,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.045, length=3.05),
        origin=Origin(xyz=(-0.035, 0.0, 1.18)),
        material=steel,
        name="rudder_hinge_knuckle",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=vertical_tail,
        child=rudder,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.55, upper=0.55),
    )

    horizontal_tail = model.part("horizontal_tail")
    horizontal_tail.visual(
        Cylinder(radius=0.070, length=8.7),
        origin=Origin(xyz=(0.0, 0.0, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elevator_hinge_spar",
    )
    for side in (-1.0, 1.0):
        horizontal_tail.visual(
            Box((1.20, 0.18, 0.12)),
            origin=Origin(xyz=(0.55, side * 2.14, 0.20)),
            material=steel,
            name=f"stabilizer_root_rib_{side:+.0f}",
        )
        horizontal_tail.visual(
            Box((1.65, 2.15, 0.13)),
            origin=Origin(xyz=(1.18, side * 3.20, 0.10)),
            material=control_fabric,
            name=f"stabilizer_panel_{side:+.0f}",
        )
    model.articulation(
        "envelope_to_horizontal_tail",
        ArticulationType.FIXED,
        parent=envelope,
        child=horizontal_tail,
        origin=Origin(xyz=(-24.55, 0.0, 9.05)),
    )

    elevator = model.part("elevator")
    elevator.visual(
        Cylinder(radius=0.045, length=8.5),
        origin=Origin(xyz=(0.0, 0.0, 0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elevator_torque_tube",
    )
    for side in (-1.0, 1.0):
        elevator.visual(
            Box((1.25, 2.10, 0.11)),
            origin=Origin(xyz=(-0.66, side * 3.20, 0.02)),
            material=control_fabric,
            name=f"elevator_panel_{side:+.0f}",
        )
    model.articulation(
        "elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=horizontal_tail,
        child=elevator,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=-0.42, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    gondola = object_model.get_part("gondola")
    envelope = object_model.get_part("envelope")
    left_engine = object_model.get_part("left_engine")
    left_wheel = object_model.get_part("left_wheel")
    rudder = object_model.get_part("rudder")
    elevator = object_model.get_part("elevator")

    for strut_name in (
        "front_keel_strut_0",
        "front_keel_strut_1",
        "rear_keel_strut_0",
        "rear_keel_strut_1",
    ):
        ctx.allow_overlap(
            envelope,
            gondola,
            elem_a=strut_name,
            elem_b="roof_hardpoint",
            reason="The gondola suspension strut ends are intentionally seated into the roof hardpoint sockets.",
        )
        ctx.expect_gap(
            envelope,
            gondola,
            axis="z",
            positive_elem=strut_name,
            negative_elem="roof_hardpoint",
            max_penetration=0.04,
            name=f"{strut_name} is only shallowly seated in the roof hardpoint",
        )

    for part_name, elem_name in (
        ("horizontal_tail", "elevator_hinge_spar"),
        ("elevator", "elevator_torque_tube"),
    ):
        ctx.allow_overlap(
            envelope,
            part_name,
            elem_a="tail_stinger",
            elem_b=elem_name,
            reason="The aft tail stinger forms the visible hinge/mount boss for the horizontal tail assembly.",
        )
        ctx.expect_gap(
            part_name,
            envelope,
            axis="z",
            positive_elem=elem_name,
            negative_elem="tail_stinger",
            max_penetration=0.12,
            name=f"{part_name} tail hinge is locally captured by the stinger",
        )
    ctx.allow_overlap(
        "horizontal_tail",
        elevator,
        elem_a="elevator_hinge_spar",
        elem_b="elevator_torque_tube",
        reason="The elevator torque tube is intentionally captured inside the fixed hinge spar.",
    )
    ctx.expect_overlap(
        "horizontal_tail",
        elevator,
        axes="y",
        elem_a="elevator_hinge_spar",
        elem_b="elevator_torque_tube",
        min_overlap=8.0,
        name="elevator torque tube shares the horizontal hinge span",
    )
    for panel_name in ("elevator_panel_-1", "elevator_panel_+1"):
        ctx.allow_overlap(
            "horizontal_tail",
            elevator,
            elem_a="elevator_hinge_spar",
            elem_b=panel_name,
            reason="The elevator leading-edge panel is locally wrapped around the hinge spar.",
        )
        ctx.expect_gap(
            "horizontal_tail",
            elevator,
            axis="x",
            positive_elem="elevator_hinge_spar",
            negative_elem=panel_name,
            max_penetration=0.04,
            name=f"{panel_name} has only shallow hinge-line overlap",
        )

    ctx.allow_overlap(
        envelope,
        "vertical_tail",
        elem_a="vertical_tail_pylon",
        elem_b="fin_panel",
        reason="The vertical fin root is intentionally socketed onto the pylon at the top of the tail.",
    )
    ctx.expect_gap(
        "vertical_tail",
        envelope,
        axis="z",
        positive_elem="fin_panel",
        negative_elem="vertical_tail_pylon",
        max_penetration=0.05,
        name="vertical fin root has a shallow pylon socket",
    )

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        engine = object_model.get_part(f"{side_name}_engine")
        prop = object_model.get_part(f"{side_name}_propeller")
        wheel = object_model.get_part(f"{side_name}_wheel")
        strut_name = f"{side_name}_engine_upper_strut"
        ctx.allow_overlap(
            gondola,
            engine,
            elem_a="shell",
            elem_b=strut_name,
            reason="The side engine support strut is seated into a reinforced gondola-side socket.",
        )
        if sign > 0.0:
            ctx.expect_gap(
                engine,
                gondola,
                axis="y",
                positive_elem=strut_name,
                negative_elem="shell",
                max_penetration=0.20,
                name=f"{side_name} engine strut has limited side-socket embed",
            )
        else:
            ctx.expect_gap(
                gondola,
                engine,
                axis="y",
                positive_elem="shell",
                negative_elem=strut_name,
                max_penetration=0.20,
                name=f"{side_name} engine strut has limited side-socket embed",
            )

        ctx.allow_overlap(
            engine,
            prop,
            elem_a="prop_shaft",
            elem_b="rotor",
            reason="The propeller hub is intentionally captured on the short engine shaft.",
        )
        ctx.expect_gap(
            engine,
            prop,
            axis="x",
            positive_elem="prop_shaft",
            negative_elem="rotor",
            max_gap=0.07,
            max_penetration=0.06,
            name=f"{side_name} propeller hub sits on the engine shaft",
        )

        axle_name = f"axle_stub_{'+' if sign > 0.0 else '-'}1"
        ctx.allow_overlap(
            gondola,
            wheel,
            elem_a=axle_name,
            elem_b="rim",
            reason="The landing wheel rim is intentionally captured on a short axle stub.",
        )
        ctx.expect_overlap(
            gondola,
            wheel,
            axes="y",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.05,
            name=f"{side_name} landing wheel remains on its axle stub",
        )
        ctx.allow_overlap(
            gondola,
            wheel,
            elem_a=axle_name,
            elem_b="hub_cap",
            reason="The landing wheel hub cap is intentionally captured by the axle stub.",
        )
        ctx.expect_overlap(
            gondola,
            wheel,
            axes="y",
            elem_a=axle_name,
            elem_b="hub_cap",
            min_overlap=0.06,
            name=f"{side_name} hub cap is retained on the axle stub",
        )

    ctx.allow_overlap(
        "vertical_tail",
        rudder,
        elem_a="rudder_hinge_post",
        elem_b="rudder_hinge_knuckle",
        reason="The rudder hinge knuckle is intentionally wrapped around the fixed hinge post.",
    )
    ctx.expect_overlap(
        "vertical_tail",
        rudder,
        axes="z",
        elem_a="rudder_hinge_post",
        elem_b="rudder_hinge_knuckle",
        min_overlap=2.6,
        name="rudder hinge barrels share a long vertical hinge line",
    )

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="hull",
        negative_elem="shell",
        min_gap=0.30,
        name="gondola hangs below envelope with visible suspension gap",
    )
    wheel_pos = ctx.part_world_position(left_wheel)
    gondola_pos = ctx.part_world_position(gondola)
    engine_pos = ctx.part_world_position(left_engine)
    ctx.check(
        "landing wheel origins are below the gondola",
        wheel_pos is not None and gondola_pos is not None and wheel_pos[2] < gondola_pos[2] - 2.8,
        details=f"wheel={wheel_pos}, gondola={gondola_pos}",
    )
    ctx.check(
        "engine nacelles sit aft of the observation cabin",
        engine_pos is not None and gondola_pos is not None and engine_pos[0] < gondola_pos[0] - 1.0,
        details=f"engine={engine_pos}, gondola={gondola_pos}",
    )

    for joint_name in (
        "left_propeller_spin",
        "right_propeller_spin",
        "left_wheel_spin",
        "right_wheel_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={joint.articulation_type}",
        )

    rudder_hinge = object_model.get_articulation("rudder_hinge")
    elevator_hinge = object_model.get_articulation("elevator_hinge")
    ctx.check(
        "rudder has symmetric steering limits",
        rudder_hinge.motion_limits.lower < -0.4 and rudder_hinge.motion_limits.upper > 0.4,
        details=str(rudder_hinge.motion_limits),
    )
    ctx.check(
        "elevator has realistic limited travel",
        elevator_hinge.motion_limits.lower < -0.3 and elevator_hinge.motion_limits.upper > 0.3,
        details=str(elevator_hinge.motion_limits),
    )

    rest_rudder = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_hinge: 0.45, elevator_hinge: 0.32}):
        moved_rudder = ctx.part_world_aabb(rudder)
        moved_elevator = ctx.part_world_aabb(elevator)
    ctx.check(
        "rudder deflects laterally when posed",
        rest_rudder is not None
        and moved_rudder is not None
        and abs((moved_rudder[0][1] + moved_rudder[1][1]) - (rest_rudder[0][1] + rest_rudder[1][1])) > 0.08,
        details=f"rest={rest_rudder}, moved={moved_rudder}",
    )
    ctx.check(
        "elevator pose remains bounded at tail",
        moved_elevator is not None and moved_elevator[0][0] < -22.1 and moved_elevator[1][0] < -20.0,
        details=f"moved={moved_elevator}",
    )

    return ctx.report()


object_model = build_object_model()
