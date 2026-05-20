from __future__ import annotations

from math import atan2, pi, sqrt

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


def _bar_between(part, name, a, b, thickness, material):
    """Add a rectangular tube-like brace between two points."""
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length_xy = sqrt(dx * dx + dy * dy)
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = -atan2(dz, length_xy) if length_xy > 1e-9 else 0.0
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _cylinder_x(part, name, center, length, radius, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, name, center, length, radius, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_z(part, name, center, length, radius, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wheelchair")

    steel = model.material("dark_powder_coated_steel", rgba=(0.05, 0.06, 0.06, 1.0))
    plate = model.material("safety_black_plate", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    rim_mat = model.material("brushed_reinforced_aluminum", rgba=(0.62, 0.66, 0.66, 1.0))
    guard_mat = model.material("matte_guard_panel", rgba=(0.10, 0.11, 0.10, 1.0))
    warning = model.material("safety_yellow_lockout", rgba=(1.0, 0.72, 0.02, 1.0))
    red = model.material("red_release_handle", rgba=(0.85, 0.03, 0.025, 1.0))

    chassis = model.part("chassis")

    # Heavy rectangular chair base: seat pan, side rails, axle tube, and reinforced back frame.
    chassis.visual(
        Box((0.64, 0.58, 0.045)),
        origin=Origin(xyz=(0.03, 0.0, 0.535)),
        material=plate,
        name="seat_pan",
    )
    chassis.visual(
        Box((0.60, 0.50, 0.035)),
        origin=Origin(xyz=(0.03, 0.0, 0.575)),
        material=model.material("dark_wear_resistant_seat_pad", rgba=(0.035, 0.038, 0.036, 1.0)),
        name="seat_pad",
    )
    chassis.visual(
        Box((0.055, 0.52, 0.62)),
        origin=Origin(xyz=(-0.335, 0.0, 0.835)),
        material=guard_mat,
        name="back_guard_panel",
    )
    _cylinder_x(chassis, "side_rail_0", (0.03, -0.305, 0.49), 0.72, 0.022, steel)
    _cylinder_x(chassis, "side_rail_1", (0.03, 0.305, 0.49), 0.72, 0.022, steel)
    _cylinder_y(chassis, "front_cross_tube", (0.39, 0.0, 0.49), 0.64, 0.022, steel)
    _cylinder_y(chassis, "rear_cross_tube", (-0.31, 0.0, 0.49), 0.64, 0.022, steel)
    chassis.visual(
        Cylinder(radius=0.030, length=0.778),
        origin=Origin(xyz=(-0.27, 0.0, 0.35), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle_tube",
    )
    _cylinder_y(chassis, "back_top_tube", (-0.36, 0.0, 1.15), 0.58, 0.024, steel)

    for idx, y in enumerate((-0.27, 0.27)):
        _cylinder_z(chassis, f"front_post_{idx}", (0.34, y, 0.405), 0.23, 0.022, steel)
        _cylinder_z(chassis, f"rear_post_{idx}", (-0.30, y, 0.435), 0.29, 0.024, steel)
        _cylinder_z(chassis, f"back_post_{idx}", (-0.36, y, 0.835), 0.64, 0.024, steel)
        _bar_between(chassis, f"load_brace_{idx}", (0.32, y, 0.505), (-0.32, y, 0.72), 0.030, steel)
        _bar_between(chassis, f"rear_gusset_{idx}", (-0.31, y, 0.49), (-0.36, y, 0.78), 0.032, steel)
        chassis.visual(
            Box((0.12, 0.015, 0.16)),
            origin=Origin(xyz=(-0.26, y * 1.06, 0.46)),
            material=plate,
            name=f"axle_reinforcement_{idx}",
        )
        chassis.visual(
            Box((0.18, 0.012, 0.30)),
            origin=Origin(xyz=(-0.08, y * 1.415, 0.662)),
            material=guard_mat,
            name=f"wheel_guard_{idx}",
        )
        chassis.visual(
            Box((0.040, 0.065, 0.030)),
            origin=Origin(xyz=(-0.08, y * 1.305, 0.525)),
            material=steel,
            name=f"guard_standoff_{idx}",
        )
        chassis.visual(
            Box((0.050, 0.070, 0.018)),
            origin=Origin(xyz=(0.42, y * 1.07, 0.255)),
            material=plate,
            name=f"caster_mount_plate_{idx}",
        )
        _bar_between(chassis, f"caster_strut_{idx}", (0.42, y * 1.07, 0.265), (0.31, y, 0.49), 0.030, steel)
        chassis.visual(
            Box((0.042, 0.075, 0.048)),
            origin=Origin(xyz=(0.43, y * 1.07, 0.305)),
            material=steel,
            name=f"caster_stop_block_{idx}",
        )
        chassis.visual(
            Box((0.055, 0.022, 0.10)),
            origin=Origin(xyz=(0.417, y * 0.55, 0.425)),
            material=plate,
            name=f"footrest_hinge_tab_{idx}",
        )
        chassis.visual(
            Box((0.055, 0.022, 0.040)),
            origin=Origin(xyz=(0.402, y * 0.55, 0.475)),
            material=plate,
            name=f"footrest_tab_web_{idx}",
        )
        chassis.visual(
            Box((0.065, 0.025, 0.110)),
            origin=Origin(xyz=(-0.045, y * 1.23, 0.555)),
            material=plate,
            name=f"brake_pivot_bracket_{idx}",
        )
        chassis.visual(
            Box((0.040, 0.020, 0.080)),
            origin=Origin(xyz=(0.055, y * 1.20, 0.545)),
            material=warning,
            name=f"brake_overtravel_stop_{idx}",
        )
        for bolt_i, z in enumerate((0.405, 0.505)):
            _cylinder_y(
                chassis,
                f"axle_bolt_{idx}_{bolt_i}",
                (-0.215, y * 1.092, z),
                0.014,
                0.010,
                rim_mat,
            )
            _cylinder_y(
                chassis,
                f"guard_bolt_{idx}_{bolt_i}",
                (-0.11, y * 1.430, 0.59 + bolt_i * 0.13),
                0.012,
                0.008,
                rim_mat,
            )

    # Bolted crossed sub-frame: visible load path and central service pivot plates.
    _bar_between(chassis, "cross_brace_0", (-0.28, -0.245, 0.455), (0.34, 0.245, 0.455), 0.030, steel)
    _bar_between(chassis, "cross_brace_1", (-0.28, 0.245, 0.455), (0.34, -0.245, 0.455), 0.030, steel)
    chassis.visual(
        Box((0.09, 0.09, 0.020)),
        origin=Origin(xyz=(0.03, 0.0, 0.455)),
        material=warning,
        name="center_pivot_plate",
    )
    _cylinder_z(chassis, "center_pivot_bolt", (0.03, 0.0, 0.468), 0.020, 0.018, rim_mat)
    chassis.visual(
        Box((0.060, 0.76, 0.055)),
        origin=Origin(xyz=(0.42, 0.0, 0.275)),
        material=steel,
        name="front_bumper_bar",
    )

    # Drive wheel geometry is mesh-backed so tires, hubs, spokes, bores and bolt circles read correctly.
    drive_tire_geom = TireGeometry(
        0.345,
        0.070,
        inner_radius=0.275,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.010, count=28, land_ratio=0.56),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    drive_wheel_geom = WheelGeometry(
        0.292,
        0.062,
        rim=WheelRim(inner_radius=0.205, flange_height=0.012, flange_thickness=0.006, bead_seat_depth=0.005),
        hub=WheelHub(
            radius=0.055,
            width=0.070,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.070, hole_diameter=0.007),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=8, thickness=0.006, window_radius=0.028),
        bore=WheelBore(style="round", diameter=0.022),
    )

    for idx, y in enumerate((-0.43, 0.43)):
        wheel = model.part(f"drive_wheel_{idx}")
        wheel.visual(mesh_from_geometry(drive_tire_geom, f"drive_tire_{idx}"), material=rubber, name="tire")
        wheel.visual(mesh_from_geometry(drive_wheel_geom, f"drive_rim_{idx}"), material=rim_mat, name="rim")
        _cylinder_x(wheel, "guarded_hub_cap", (0.0, 0.0, 0.0), 0.082, 0.048, steel)
        model.articulation(
            f"drive_axle_{idx}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(-0.27, y, 0.35), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=8.0),
        )

    # Front caster swivels: separate kingpin/fork assemblies and rolling wheels.
    caster_tire_geom = TireGeometry(
        0.102,
        0.046,
        inner_radius=0.064,
        tread=TireTread(style="block", depth=0.006, count=16, land_ratio=0.55),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.006, radius=0.002),
    )
    caster_wheel_geom = WheelGeometry(
        0.070,
        0.040,
        rim=WheelRim(inner_radius=0.048, flange_height=0.005, flange_thickness=0.003, bead_seat_depth=0.002),
        hub=WheelHub(radius=0.022, width=0.040, cap_style="flat"),
        face=WheelFace(dish_depth=0.003, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.008),
        bore=WheelBore(style="round", diameter=0.010),
    )

    for idx, y in enumerate((-0.29, 0.29)):
        caster = model.part(f"caster_swivel_{idx}")
        _cylinder_z(caster, "kingpin", (0.0, 0.0, -0.020), 0.040, 0.017, rim_mat)
        caster.visual(
            Box((0.060, 0.118, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=steel,
            name="fork_bridge",
        )
        for side_idx, yy in enumerate((-0.042, 0.042)):
            caster.visual(
                Box((0.038, 0.012, 0.216)),
                origin=Origin(xyz=(0.0, yy, -0.148)),
                material=steel,
                name=f"fork_plate_{side_idx}",
            )
            _cylinder_y(caster, f"fork_axle_boss_{side_idx}", (0.0, yy, -0.148), 0.018, 0.017, rim_mat)
        caster.visual(
            Box((0.052, 0.026, 0.025)),
            origin=Origin(xyz=(0.055, 0.0, -0.005)),
            material=warning,
            name="swivel_stop_lug",
        )
        model.articulation(
            f"caster_swivel_joint_{idx}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=caster,
            origin=Origin(xyz=(0.42, y, 0.250)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=4.0),
        )

        caster_wheel = model.part(f"caster_wheel_{idx}")
        caster_wheel.visual(mesh_from_geometry(caster_tire_geom, f"caster_tire_{idx}"), material=rubber, name="tire")
        caster_wheel.visual(mesh_from_geometry(caster_wheel_geom, f"caster_rim_{idx}"), material=rim_mat, name="rim")
        caster_wheel.visual(
            Cylinder(radius=0.012, length=0.104),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=rim_mat,
            name="axle_sleeve",
        )
        model.articulation(
            f"caster_axle_{idx}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.148), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=10.0),
        )

    # Swing-away footrests with stop tabs and anti-slip plates.
    for idx, y in enumerate((-0.14, 0.14)):
        footrest = model.part(f"footrest_{idx}")
        footrest.visual(
            Cylinder(radius=0.022, length=0.075),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=rim_mat,
            name="hinge_barrel",
        )
        footrest.visual(
            Box((0.034, 0.034, 0.280)),
            origin=Origin(xyz=(0.031, 0.0, -0.145)),
            material=steel,
            name="hanger_tube",
        )
        _bar_between(footrest, "diagonal_support", (0.035, 0.0, -0.225), (0.185, 0.0, -0.270), 0.024, steel)
        footrest.visual(
            Box((0.270, 0.105, 0.026)),
            origin=Origin(xyz=(0.205, 0.0, -0.285)),
            material=plate,
            name="foot_plate",
        )
        footrest.visual(
            Box((0.235, 0.085, 0.008)),
            origin=Origin(xyz=(0.205, 0.0, -0.266)),
            material=rubber,
            name="anti_slip_pad",
        )
        footrest.visual(
            Box((0.032, 0.050, 0.040)),
            origin=Origin(xyz=(0.005, 0.0, -0.075)),
            material=warning,
            name="swing_stop_tab",
        )
        model.articulation(
            f"footrest_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=footrest,
            origin=Origin(xyz=(0.44, y, 0.425)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.15, upper=1.55),
        )

    # Wheel lockout levers: distinct moving controls with pads, stops, and red handles.
    for idx, y in enumerate((-0.335, 0.335)):
        sign = -1.0 if y < 0.0 else 1.0
        brake = model.part(f"brake_lever_{idx}")
        brake.visual(
            Cylinder(radius=0.014, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rim_mat,
            name="pivot_pin",
        )
        brake.visual(
            Box((0.095, 0.020, 0.032)),
            origin=Origin(xyz=(0.047, 0.0, 0.020)),
            material=warning,
            name="arm_root_link",
        )
        brake.visual(
            Box((0.040, 0.025, 0.220)),
            origin=Origin(xyz=(0.055, 0.0, 0.145), rpy=(0.0, 0.22, 0.0)),
            material=warning,
            name="lockout_arm",
        )
        brake.visual(
            Box((0.055, 0.045, 0.045)),
            origin=Origin(xyz=(0.055, sign * 0.028, 0.260)),
            material=red,
            name="release_handle",
        )
        brake.visual(
            Box((0.155, 0.022, 0.026)),
            origin=Origin(xyz=(0.075, sign * 0.030, -0.020), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name="shoe_link",
        )
        brake.visual(
            Box((0.048, 0.030, 0.060)),
            origin=Origin(xyz=(0.155, sign * 0.026, -0.030)),
            material=rubber,
            name="brake_shoe",
        )
        brake.visual(
            Box((0.025, 0.050, 0.050)),
            origin=Origin(xyz=(0.115, 0.0, 0.000)),
            material=warning,
            name="lockout_tooth",
        )
        model.articulation(
            f"brake_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=brake,
            origin=Origin(xyz=(-0.045, y, 0.585)),
            axis=(0.0, sign, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drive_joints = [object_model.get_articulation(f"drive_axle_{i}") for i in range(2)]
    caster_swivels = [object_model.get_articulation(f"caster_swivel_joint_{i}") for i in range(2)]
    caster_axles = [object_model.get_articulation(f"caster_axle_{i}") for i in range(2)]
    footrest_hinges = [object_model.get_articulation(f"footrest_hinge_{i}") for i in range(2)]
    brake_pivots = [object_model.get_articulation(f"brake_pivot_{i}") for i in range(2)]

    ctx.check(
        "drive wheels roll on continuous axles",
        all(j.articulation_type == ArticulationType.CONTINUOUS and tuple(j.axis) == (1.0, 0.0, 0.0) for j in drive_joints),
    )
    ctx.check(
        "casters have swivel and rolling joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in caster_swivels + caster_axles),
    )
    ctx.check(
        "footrests and lockouts have bounded motion",
        all(j.motion_limits is not None and j.motion_limits.upper > 0.5 for j in footrest_hinges + brake_pivots),
    )

    chassis = object_model.get_part("chassis")
    for i in range(2):
        wheel = object_model.get_part(f"drive_wheel_{i}")
        ctx.allow_overlap(
            chassis,
            wheel,
            elem_a="rear_axle_tube",
            elem_b="rim",
            reason="The reinforced rear axle tube intentionally seats into the drive wheel hub/rim bore.",
        )
        ctx.expect_overlap(
            wheel,
            chassis,
            axes="z",
            min_overlap=0.050,
            elem_a="rim",
            elem_b="rear_axle_tube",
            name=f"drive wheel {i} aligns with axle tube height",
        )
        caster = object_model.get_part(f"caster_swivel_{i}")
        caster_wheel = object_model.get_part(f"caster_wheel_{i}")
        for boss_i in range(2):
            ctx.allow_overlap(
                caster,
                caster_wheel,
                elem_a=f"fork_axle_boss_{boss_i}",
                elem_b="axle_sleeve",
                reason="The caster wheel axle sleeve is intentionally captured by the fork axle boss.",
            )
            ctx.allow_overlap(
                caster,
                caster_wheel,
                elem_a=f"fork_plate_{boss_i}",
                elem_b="axle_sleeve",
                reason="The caster axle sleeve intentionally passes through the fork side plate bore.",
            )
            ctx.expect_overlap(
                caster,
                caster_wheel,
                axes="y",
                min_overlap=0.010,
                elem_a=f"fork_axle_boss_{boss_i}",
                elem_b="axle_sleeve",
                name=f"caster {i} boss {boss_i} captures axle sleeve",
            )
            ctx.expect_overlap(
                caster,
                caster_wheel,
                axes="y",
                min_overlap=0.006,
                elem_a=f"fork_plate_{boss_i}",
                elem_b="axle_sleeve",
                name=f"caster {i} fork plate {boss_i} retains axle sleeve",
            )
        ctx.expect_within(
            caster_wheel,
            caster,
            axes="y",
            margin=0.020,
            inner_elem="rim",
            outer_elem="fork_bridge",
            name=f"caster wheel {i} is retained between fork plates",
        )
        footrest = object_model.get_part(f"footrest_{i}")
        ctx.allow_overlap(
            chassis,
            footrest,
            elem_a=f"footrest_hinge_tab_{i}",
            elem_b="hinge_barrel",
            reason="The swing-away footrest hinge barrel is intentionally captured in the welded hinge tab.",
        )
        ctx.expect_overlap(
            footrest,
            chassis,
            axes="z",
            min_overlap=0.050,
            elem_a="hinge_barrel",
            elem_b=f"footrest_hinge_tab_{i}",
            name=f"footrest {i} hinge barrel is retained by tab",
        )
        ctx.expect_gap(
            footrest,
            chassis,
            axis="x",
            min_gap=-0.040,
            max_gap=0.42,
            positive_elem="foot_plate",
            negative_elem="front_bumper_bar",
            name=f"footrest {i} projects from supported front frame",
        )
        brake = object_model.get_part(f"brake_lever_{i}")
        ctx.allow_overlap(
            brake,
            chassis,
            elem_a="pivot_pin",
            elem_b=f"brake_pivot_bracket_{i}",
            reason="The brake lockout pivot pin is intentionally captured through the welded pivot bracket.",
        )
        ctx.allow_overlap(
            brake,
            chassis,
            elem_a="arm_root_link",
            elem_b=f"brake_pivot_bracket_{i}",
            reason="The boxed root of the brake lever is intentionally nested in the pivot bracket slot.",
        )
        ctx.allow_overlap(
            brake,
            chassis,
            elem_a="lockout_tooth",
            elem_b=f"brake_overtravel_stop_{i}",
            reason="The lockout tooth is intentionally seated against the over-travel stop at the parked position.",
        )
        ctx.expect_overlap(
            brake,
            chassis,
            axes="y",
            min_overlap=0.020,
            elem_a="pivot_pin",
            elem_b=f"brake_pivot_bracket_{i}",
            name=f"brake {i} pivot pin is captured by bracket",
        )
        ctx.expect_overlap(
            brake,
            chassis,
            axes="z",
            min_overlap=0.015,
            elem_a="arm_root_link",
            elem_b=f"brake_pivot_bracket_{i}",
            name=f"brake {i} lever root remains in bracket slot",
        )
        ctx.expect_overlap(
            brake,
            chassis,
            axes="z",
            min_overlap=0.015,
            elem_a="lockout_tooth",
            elem_b=f"brake_overtravel_stop_{i}",
            name=f"brake {i} lockout tooth bears on overtravel stop",
        )

    # A decisive pose check: the lockout handle rotates without translating its pivot.
    brake = object_model.get_part("brake_lever_1")
    brake_joint = object_model.get_articulation("brake_pivot_1")
    rest = ctx.part_world_position(brake)
    with ctx.pose({brake_joint: 0.50}):
        moved = ctx.part_world_position(brake)
    ctx.check(
        "brake lockout pivots about fixed bracket",
        rest is not None and moved is not None and max(abs(a - b) for a, b in zip(rest, moved)) < 1e-6,
        details=f"rest={rest}, moved={moved}",
    )

    return ctx.report()


object_model = build_object_model()
