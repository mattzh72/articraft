from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _rotor_mesh(name: str):
    outer = superellipse_profile(0.36, 0.36, exponent=2.0, segments=72)
    holes = [superellipse_profile(0.092, 0.092, exponent=2.0, segments=36)]
    slot = rounded_rect_profile(0.026, 0.064, 0.006, corner_segments=5)
    for i in range(8):
        angle = i * 2.0 * pi / 8.0
        holes.append(
            _transform_profile(
                slot,
                dx=cos(angle) * 0.118,
                dy=sin(angle) * 0.118,
                angle=angle + pi / 2.0,
            )
        )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, height=0.026, center=True).rotate_y(pi / 2.0),
        name,
    )


def _annular_disc_mesh(name: str, *, outer_radius: float, inner_radius: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(outer_radius * 2.0, outer_radius * 2.0, exponent=2.0, segments=72),
            [superellipse_profile(inner_radius * 2.0, inner_radius * 2.0, exponent=2.0, segments=48)],
            height=thickness,
            center=True,
        ).rotate_y(pi / 2.0),
        name,
    )


def _add_bolt_circle(
    part,
    *,
    name_prefix: str,
    material,
    radius: float,
    count: int,
    y: float,
    head_radius: float,
    head_length: float,
    x_scale: float = 1.0,
    z_scale: float = 1.0,
) -> None:
    for i in range(count):
        a = i * 2.0 * pi / count
        part.visual(
            Cylinder(radius=head_radius, length=head_length),
            origin=Origin(
                xyz=(cos(a) * radius * x_scale, y, sin(a) * radius * z_scale),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{name_prefix}_{i}",
        )


def _add_u_bolt_set(part, *, x: float, material, nut_material) -> None:
    # Two real-looking U-bolts clamp each leaf-spring perch around the axle tube.
    for y in (-0.105, 0.105):
        part.visual(
            Cylinder(radius=0.012, length=0.27),
            origin=Origin(xyz=(x - 0.052, y, -0.035)),
            material=material,
            name=f"u_bolt_rod_{x:+.2f}_{y:+.2f}_0",
        )
        part.visual(
            Cylinder(radius=0.012, length=0.27),
            origin=Origin(xyz=(x + 0.052, y, -0.035)),
            material=material,
            name=f"u_bolt_rod_{x:+.2f}_{y:+.2f}_1",
        )
        part.visual(
            Box((0.128, 0.022, 0.022)),
            origin=Origin(xyz=(x, y, 0.098)),
            material=material,
            name=f"u_bolt_saddle_{x:+.2f}_{y:+.2f}",
        )
        for xoff in (-0.052, 0.052):
            part.visual(
                Cylinder(radius=0.022, length=0.018),
                origin=Origin(xyz=(x + xoff, y, -0.180)),
                material=nut_material,
                name=f"u_bolt_nut_{x:+.2f}_{y:+.2f}_{xoff:+.2f}",
            )


def _add_caliper(part, *, side: float, material, pad_material, bracket_material) -> None:
    # Static caliper and bracket straddle the rotating disc without intersecting it.
    base_x = side * 0.925
    # Inboard and outboard pads are separated by a clear slot for the rotor.
    for xoff in (-0.026, 0.026):
        part.visual(
            Box((0.014, 0.080, 0.125)),
            origin=Origin(xyz=(base_x + xoff, -0.125, 0.055)),
            material=pad_material,
            name=f"{'left' if side > 0 else 'right'}_caliper_pad_{0 if xoff < 0 else 1}",
        )
        part.visual(
            Box((0.014, 0.060, 0.125)),
            origin=Origin(xyz=(base_x + xoff, -0.178, 0.055)),
            material=material,
            name=f"{'left' if side > 0 else 'right'}_caliper_rib_{0 if xoff < 0 else 1}",
        )
    part.visual(
        Box((0.078, 0.033, 0.145)),
        origin=Origin(xyz=(base_x, -0.203, 0.055)),
        material=material,
        name=f"{'left' if side > 0 else 'right'}_caliper_bridge",
    )
    part.visual(
        Box((0.044, 0.115, 0.035)),
        origin=Origin(xyz=(side * 0.865, -0.142, 0.130)),
        material=bracket_material,
        name=f"{'left' if side > 0 else 'right'}_caliper_bracket",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.115),
        origin=Origin(xyz=(side * 0.880, -0.145, 0.085), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bracket_material,
        name=f"{'left' if side > 0 else 'right'}_caliper_pin",
    )


def _add_hub_visuals(part, *, side: float, rotor_mesh, materials) -> None:
    cast_steel, machined, rotor_steel, dark, rubber = materials
    # The child frame is on the wheel-bearing axis.  Positive local X is vehicle-left,
    # so side chooses which direction is outboard.
    part.visual(
        Cylinder(radius=0.045, length=0.130),
        origin=Origin(xyz=(-side * 0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="spindle_shaft",
    )
    part.visual(
        rotor_mesh,
        origin=Origin(xyz=(side * 0.045, 0.0, 0.0)),
        material=rotor_steel,
        name="vented_rotor",
    )
    part.visual(
        Cylinder(radius=0.082, length=0.072),
        origin=Origin(xyz=(side * 0.068, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="wheel_bearing_hub",
    )
    part.visual(
        Cylinder(radius=0.130, length=0.046),
        origin=Origin(xyz=(side * 0.102, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_steel,
        name="wheel_flange",
    )
    part.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(side * 0.138, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="dust_cap",
    )
    for i in range(5):
        a = pi / 2.0 + i * 2.0 * pi / 5.0
        y = cos(a) * 0.092
        z = sin(a) * 0.092
        part.visual(
            Cylinder(radius=0.009, length=0.064),
            origin=Origin(xyz=(side * 0.142, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name=f"wheel_stud_{i}",
        )
        part.visual(
            Cylinder(radius=0.017, length=0.026),
            origin=Origin(xyz=(side * 0.186, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark,
            name=f"lug_nut_{i}",
        )
    # A thin black ABS wheel-speed tone ring reads as real automotive hardware.
    part.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(-side * 0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tone_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_car_axle_assembly")

    cast_iron = model.material("cast_iron", rgba=(0.16, 0.17, 0.16, 1.0))
    black_paint = model.material("satin_black_paint", rgba=(0.035, 0.038, 0.040, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.66, 0.68, 0.67, 1.0))
    dark_steel = model.material("dark_bolt_steel", rgba=(0.23, 0.24, 0.24, 1.0))
    rotor_steel = model.material("burnished_rotor_steel", rgba=(0.42, 0.43, 0.42, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    pad_gray = model.material("brake_pad_graphite", rgba=(0.11, 0.10, 0.095, 1.0))
    weld_paint = model.material("welded_black_steel", rgba=(0.07, 0.075, 0.072, 1.0))

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((1.85, 0.58, 0.58)),
        mass=125.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Differential pumpkin and axle tubes: heavy, cast, and visibly supported.
    housing.visual(
        Cylinder(radius=0.061, length=1.500),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_paint,
        name="axle_tube",
    )
    housing.visual(
        Cylinder(radius=0.235, length=0.300),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="differential_bowl",
    )
    housing.visual(
        Box((0.420, 0.175, 0.430)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=cast_iron,
        name="carrier_saddle",
    )
    housing.visual(
        Cylinder(radius=0.255, length=0.070),
        origin=Origin(xyz=(0.0, 0.195, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="rear_cover",
    )
    housing.visual(
        Cylinder(radius=0.095, length=0.036),
        origin=Origin(xyz=(0.0, 0.247, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_fill_plug",
    )
    housing.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.238, -0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="drain_plug",
    )
    _add_bolt_circle(
        housing,
        name_prefix="cover_bolt",
        material=dark_steel,
        radius=0.200,
        count=10,
        y=0.238,
        head_radius=0.015,
        head_length=0.016,
        x_scale=1.08,
        z_scale=0.88,
    )
    housing.visual(
        Cylinder(radius=0.090, length=0.285),
        origin=Origin(xyz=(0.0, -0.245, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pinion_snout",
    )
    housing.visual(
        Cylinder(radius=0.113, length=0.030),
        origin=Origin(xyz=(0.0, -0.385, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pinion_seal",
    )
    housing.visual(
        Cylinder(radius=0.076, length=0.045),
        origin=Origin(xyz=(0.0, -0.092, 0.205)),
        material=cast_iron,
        name="top_boss",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, -0.095, 0.242)),
        material=machined_steel,
        name="breather",
    )

    # Strengthening ribs tying tubes, pumpkin, and pinion snout together.
    for x in (-0.175, 0.175):
        housing.visual(
            Box((0.042, 0.365, 0.050)),
            origin=Origin(xyz=(x, -0.055, 0.175), rpy=(0.0, 0.0, 0.0)),
            material=cast_iron,
            name=f"upper_rib_{0 if x < 0 else 1}",
        )
        housing.visual(
            Box((0.045, 0.345, 0.045)),
            origin=Origin(xyz=(x, -0.055, -0.178)),
            material=cast_iron,
            name=f"lower_rib_{0 if x < 0 else 1}",
        )

    # Outboard bearing collars, backing plates, ABS sensor bosses, and flanges.
    for side in (-1.0, 1.0):
        label = "left" if side > 0 else "right"
        collar_name = "left_bearing_collar" if side > 0 else "right_bearing_collar"
        housing.visual(
            Cylinder(radius=0.088, length=0.120),
            origin=Origin(xyz=(side * 0.805, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=cast_iron,
            name=collar_name,
        )
        for plate_name, y, z, size in (
            ("upper", 0.0, 0.1105, (0.030, 0.270, 0.046)),
            ("lower", 0.0, -0.1105, (0.030, 0.270, 0.046)),
            ("front", 0.1105, 0.0, (0.030, 0.046, 0.190)),
            ("rear", -0.1105, 0.0, (0.030, 0.046, 0.190)),
        ):
            housing.visual(
                Box(size),
                origin=Origin(xyz=(side * 0.860, y, z)),
                material=weld_paint,
                name=f"{label}_backing_plate_{plate_name}",
            )
        housing.visual(
            Box((0.038, 0.080, 0.055)),
            origin=Origin(xyz=(side * 0.792, 0.075, 0.105)),
            material=dark_steel,
            name=f"{label}_abs_sensor",
        )
        housing.visual(
            Cylinder(radius=0.007, length=0.105),
            origin=Origin(xyz=(side * 0.792, 0.030, 0.137), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"{label}_sensor_lead",
        )
        for a in (pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0):
            housing.visual(
                Cylinder(radius=0.012, length=0.018),
                origin=Origin(
                    xyz=(side * 0.882, cos(a) * 0.120, sin(a) * 0.120),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=dark_steel,
                name=f"{label}_flange_bolt_{round(a, 2)}",
            )

    # Leaf-spring pads, U-bolts, bump-stop pads, shock tabs, and trailing arm brackets.
    for x in (-0.520, 0.520):
        housing.visual(
            Box((0.220, 0.300, 0.045)),
            origin=Origin(xyz=(x, 0.0, -0.118)),
            material=weld_paint,
            name=f"spring_perch_{0 if x < 0 else 1}",
        )
        housing.visual(
            Box((0.165, 0.240, 0.030)),
            origin=Origin(xyz=(x, 0.0, -0.168)),
            material=dark_steel,
            name=f"spring_clamp_plate_{0 if x < 0 else 1}",
        )
        housing.visual(
            Box((0.120, 0.110, 0.060)),
            origin=Origin(xyz=(x, -0.115, 0.000)),
            material=weld_paint,
            name=f"trailing_arm_bracket_{0 if x < 0 else 1}",
        )
        housing.visual(
            Box((0.050, 0.035, 0.155)),
            origin=Origin(xyz=(x, -0.175, 0.100)),
            material=weld_paint,
            name=f"shock_tab_{0 if x < 0 else 1}",
        )
        housing.visual(
            Cylinder(radius=0.018, length=0.082),
            origin=Origin(xyz=(x, -0.177, 0.118), rpy=(pi / 2.0, 0.0, 0.0)),
            material=machined_steel,
            name=f"shock_mount_bolt_{0 if x < 0 else 1}",
        )
        _add_u_bolt_set(housing, x=x, material=machined_steel, nut_material=dark_steel)

    # Real-looking hard brake line follows the axle tube and tees over the pumpkin.
    brake_line = tube_from_spline_points(
        [
            (-0.735, 0.076, 0.080),
            (-0.510, 0.087, 0.092),
            (-0.225, 0.108, 0.155),
            (0.000, 0.130, 0.185),
            (0.225, 0.108, 0.155),
            (0.510, 0.087, 0.092),
            (0.735, 0.076, 0.080),
        ],
        radius=0.006,
        samples_per_segment=12,
        radial_segments=10,
    )
    housing.visual(mesh_from_geometry(brake_line, "formed_brake_line"), material=machined_steel, name="brake_line")
    housing.visual(
        Box((0.052, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.135, 0.188)),
        material=machined_steel,
        name="brake_tee",
    )

    _add_caliper(housing, side=1.0, material=black_paint, pad_material=pad_gray, bracket_material=dark_steel)
    _add_caliper(housing, side=-1.0, material=black_paint, pad_material=pad_gray, bracket_material=dark_steel)

    left_hub = model.part("left_hub")
    left_hub.inertial = Inertial.from_geometry(Cylinder(radius=0.18, length=0.20), mass=17.0)
    _add_hub_visuals(
        left_hub,
        side=1.0,
        rotor_mesh=_rotor_mesh("left_vented_rotor"),
        materials=(cast_iron, machined_steel, rotor_steel, dark_steel, rubber),
    )

    right_hub = model.part("right_hub")
    right_hub.inertial = Inertial.from_geometry(Cylinder(radius=0.18, length=0.20), mass=17.0)
    _add_hub_visuals(
        right_hub,
        side=-1.0,
        rotor_mesh=_rotor_mesh("right_vented_rotor"),
        materials=(cast_iron, machined_steel, rotor_steel, dark_steel, rubber),
    )

    pinion_yoke = model.part("pinion_yoke")
    pinion_yoke.inertial = Inertial.from_geometry(Cylinder(radius=0.12, length=0.18), mass=6.0)
    pinion_yoke.visual(
        Cylinder(radius=0.034, length=0.105),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pinion_stub",
    )
    pinion_yoke.visual(
        Cylinder(radius=0.070, length=0.028),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="companion_flange",
    )
    for a in (pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0):
        pinion_yoke.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(
                xyz=(cos(a) * 0.047, -0.076, sin(a) * 0.047),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=machined_steel,
            name=f"flange_bolt_{round(a, 2)}",
        )
    # U-joint yoke ears and cross pins, connected to the companion flange.
    pinion_yoke.visual(
        Box((0.040, 0.105, 0.045)),
        origin=Origin(xyz=(0.055, -0.112, 0.0)),
        material=dark_steel,
        name="yoke_ear_0",
    )
    pinion_yoke.visual(
        Box((0.040, 0.105, 0.045)),
        origin=Origin(xyz=(-0.055, -0.112, 0.0)),
        material=dark_steel,
        name="yoke_ear_1",
    )
    pinion_yoke.visual(
        Cylinder(radius=0.016, length=0.135),
        origin=Origin(xyz=(0.0, -0.118, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="u_joint_cross",
    )
    pinion_yoke.visual(
        Cylinder(radius=0.020, length=0.080),
        origin=Origin(xyz=(0.0, -0.118, 0.0)),
        material=machined_steel,
        name="u_joint_cap",
    )

    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_hub,
        origin=Origin(xyz=(0.880, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=35.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_hub,
        origin=Origin(xyz=(-0.880, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=35.0),
    )
    model.articulation(
        "pinion_yoke_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=pinion_yoke,
        origin=Origin(xyz=(0.0, -0.385, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    pinion_yoke = object_model.get_part("pinion_yoke")
    left_spin = object_model.get_articulation("left_hub_spin")
    right_spin = object_model.get_articulation("right_hub_spin")
    pinion_spin = object_model.get_articulation("pinion_yoke_spin")

    ctx.allow_overlap(
        housing,
        left_hub,
        elem_a="left_bearing_collar",
        elem_b="spindle_shaft",
        reason="The rotating spindle is intentionally captured inside the left wheel-bearing collar.",
    )
    ctx.allow_overlap(
        housing,
        right_hub,
        elem_a="right_bearing_collar",
        elem_b="spindle_shaft",
        reason="The rotating spindle is intentionally captured inside the right wheel-bearing collar.",
    )
    ctx.allow_overlap(
        housing,
        pinion_yoke,
        elem_a="pinion_seal",
        elem_b="pinion_stub",
        reason="The pinion stub intentionally passes through the static seal at the nose of the differential.",
    )
    ctx.allow_overlap(
        housing,
        pinion_yoke,
        elem_a="pinion_snout",
        elem_b="pinion_stub",
        reason="The pinion shaft is intentionally represented as captured inside the simplified cast pinion snout.",
    )

    ctx.check(
        "hub joints are continuous axle rotations",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_spin.articulation_type},{left_spin.axis}; right={right_spin.articulation_type},{right_spin.axis}",
    )
    ctx.check(
        "pinion yoke rotates on the driveline axis",
        pinion_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(pinion_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={pinion_spin.articulation_type}, axis={pinion_spin.axis}",
    )

    ctx.expect_within(
        left_hub,
        housing,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="left_bearing_collar",
        margin=0.001,
        name="left spindle is centered in bearing collar",
    )
    ctx.expect_overlap(
        left_hub,
        housing,
        axes="x",
        elem_a="spindle_shaft",
        elem_b="left_bearing_collar",
        min_overlap=0.050,
        name="left spindle remains inserted",
    )
    ctx.expect_within(
        right_hub,
        housing,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="right_bearing_collar",
        margin=0.001,
        name="right spindle is centered in bearing collar",
    )
    ctx.expect_overlap(
        right_hub,
        housing,
        axes="x",
        elem_a="spindle_shaft",
        elem_b="right_bearing_collar",
        min_overlap=0.050,
        name="right spindle remains inserted",
    )
    ctx.expect_within(
        pinion_yoke,
        housing,
        axes="xz",
        inner_elem="pinion_stub",
        outer_elem="pinion_seal",
        margin=0.001,
        name="pinion stub is centered in seal",
    )
    ctx.expect_overlap(
        pinion_yoke,
        housing,
        axes="y",
        elem_a="pinion_stub",
        elem_b="pinion_seal",
        min_overlap=0.018,
        name="pinion stub passes through seal",
    )
    ctx.expect_within(
        pinion_yoke,
        housing,
        axes="xz",
        inner_elem="pinion_stub",
        outer_elem="pinion_snout",
        margin=0.001,
        name="pinion shaft stays inside snout",
    )
    ctx.expect_overlap(
        pinion_yoke,
        housing,
        axes="y",
        elem_a="pinion_stub",
        elem_b="pinion_snout",
        min_overlap=0.060,
        name="pinion shaft remains supported in snout",
    )

    left_rest = ctx.part_world_position(left_hub)
    right_rest = ctx.part_world_position(right_hub)
    with ctx.pose({left_spin: pi / 2.0, right_spin: -pi / 2.0, pinion_spin: pi / 2.0}):
        left_rotated = ctx.part_world_position(left_hub)
        right_rotated = ctx.part_world_position(right_hub)
        ctx.expect_within(
            left_hub,
            housing,
            axes="yz",
            inner_elem="spindle_shaft",
            outer_elem="left_bearing_collar",
            margin=0.001,
            name="left hub stays supported while rotating",
        )
        ctx.expect_within(
            right_hub,
            housing,
            axes="yz",
            inner_elem="spindle_shaft",
            outer_elem="right_bearing_collar",
            margin=0.001,
            name="right hub stays supported while rotating",
        )
    ctx.check(
        "hub centers remain fixed during spin",
        left_rest is not None
        and right_rest is not None
        and left_rotated is not None
        and right_rotated is not None
        and max(abs(left_rest[i] - left_rotated[i]) for i in range(3)) < 1e-6
        and max(abs(right_rest[i] - right_rotated[i]) for i in range(3)) < 1e-6,
        details=f"left {left_rest}->{left_rotated}, right {right_rest}->{right_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
