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
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


AXIS_X = Origin(rpy=(0.0, pi / 2.0, 0.0))
AXIS_Y = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _cyl_x(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _bolt_grid(part, *, x, y, z, side, material, prefix):
    """Small proud bolt heads on vertical adapter or service plates."""
    for iy, yy in enumerate((-0.045, 0.045)):
        for iz, zz in enumerate((-0.050, 0.050)):
            part.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x + side * 0.004, y + yy, z + zz), rpy=(0.0, pi / 2.0, 0.0)),
                material=material,
                name=f"{prefix}_bolt_{iy}_{iz}",
            )


def _add_drive_wheel(part, *, prefix: str, side: float, materials):
    steel, dark, rubber = materials
    tire = TireGeometry(
        0.310,
        0.058,
        inner_radius=0.236,
        carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.09),
        tread=TireTread(style="ribbed", depth=0.004, count=28, land_ratio=0.64),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
        sidewall=TireSidewall(style="rounded", bulge=0.06),
        shoulder=TireShoulder(width=0.006, radius=0.004),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=rubber, name="tire")

    wheel = WheelGeometry(
        0.246,
        0.052,
        rim=WheelRim(inner_radius=0.190, flange_height=0.009, flange_thickness=0.004, bead_seat_depth=0.004),
        hub=WheelHub(
            radius=0.052,
            width=0.078,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.072, hole_diameter=0.007),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="straight", count=12, thickness=0.0035, window_radius=0.018),
        bore=WheelBore(style="round", diameter=0.026),
    )
    part.visual(mesh_from_geometry(wheel, f"{prefix}_wheel_core"), material=steel, name="hub")

    handrim = TorusGeometry(0.276, 0.0065, radial_segments=18, tubular_segments=96).rotate_y(pi / 2.0)
    part.visual(
        mesh_from_geometry(handrim, f"{prefix}_pushrim"),
        origin=Origin(xyz=(side * 0.043, 0.0, 0.0)),
        material=steel,
        name="pushrim",
    )
    for i in range(8):
        angle = 2.0 * pi * i / 8.0
        y = cos(angle) * 0.276
        z = sin(angle) * 0.276
        part.visual(
            Cylinder(radius=0.004, length=0.040),
            origin=Origin(xyz=(side * 0.026, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark,
            name=f"pushrim_standoff_{i}",
        )


def _add_caster_wheel(part, *, prefix: str, materials):
    steel, dark, rubber = materials
    tire = TireGeometry(
        0.066,
        0.032,
        inner_radius=0.041,
        tread=None,
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.003, radius=0.002),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=rubber, name="tire")
    wheel = WheelGeometry(
        0.043,
        0.030,
        rim=WheelRim(inner_radius=0.030, flange_height=0.003, flange_thickness=0.002, bead_seat_depth=0.001),
        hub=WheelHub(radius=0.018, width=0.040, cap_style="flat"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.002, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.010),
    )
    part.visual(mesh_from_geometry(wheel, f"{prefix}_wheel_core"), material=steel, name="hub")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wheelchair")

    frame_paint = model.material("aged_olive_frame", rgba=(0.18, 0.24, 0.18, 1.0))
    black = model.material("black_rubber", rgba=(0.025, 0.024, 0.022, 1.0))
    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    dark_steel = model.material("dark_parkerized_steel", rgba=(0.20, 0.21, 0.20, 1.0))
    vinyl = model.material("oxblood_vinyl", rgba=(0.31, 0.055, 0.050, 1.0))
    hatch_paint = model.material("cream_service_panels", rgba=(0.78, 0.73, 0.61, 1.0))

    frame = model.part("frame")

    # Old-school welded side frames: parallel tubular rectangles tied by cross tubes.
    for side, label in ((1.0, "left"), (-1.0, "right")):
        x = side * 0.255
        _cyl_y(frame, 0.017, 0.68, (x, 0.035, 0.205), frame_paint, f"{label}_lower_rail")
        _cyl_y(frame, 0.017, 0.68, (x, 0.065, 0.470), frame_paint, f"{label}_seat_rail")
        _cyl_z(frame, 0.017, 0.550, (x, -0.300, 0.380), frame_paint, f"{label}_rear_post")
        _cyl_z(frame, 0.017, 0.320, (x, 0.385, 0.340), frame_paint, f"{label}_front_post")
        _cyl_y(frame, 0.014, 0.32, (x, -0.235, 0.575), frame_paint, f"{label}_back_socket")
        _cyl_z(frame, 0.030, 0.090, (x, 0.430, 0.205), dark_steel, f"{label}_caster_sleeve")
        _cyl_x(frame, 0.022, 0.112, (side * 0.335, -0.055, 0.310), dark_steel, f"{label}_axle_stub")
        frame.visual(
            Box((0.055, 0.165, 0.185)),
            origin=Origin(xyz=(side * 0.280, -0.055, 0.310)),
            material=dark_steel,
            name=f"{label}_adapter_plate",
        )
        frame.visual(
            Box((0.048, 0.055, 0.130)),
            origin=Origin(xyz=(side * 0.165, 0.430, 0.335)),
            material=dark_steel,
            name=f"{label}_footrest_lug",
        )
        frame.visual(
            Box((0.050, 0.090, 0.080)),
            origin=Origin(xyz=(side * 0.165, 0.430, 0.430)),
            material=dark_steel,
            name=f"{label}_footrest_strut",
        )
        _bolt_grid(frame, x=side * 0.304, y=-0.055, z=0.310, side=side, material=steel, prefix=f"{label}_adapter")

    frame.visual(
        Box((0.040, 0.090, 0.090)),
        origin=Origin(xyz=(0.265, -0.325, 0.475)),
        material=dark_steel,
        name="left_back_hinge_lug",
    )
    frame.visual(
        Box((0.040, 0.090, 0.090)),
        origin=Origin(xyz=(-0.265, -0.325, 0.475)),
        material=dark_steel,
        name="right_back_hinge_lug",
    )

    for y, z, label in (
        (-0.325, 0.205, "rear_lower"),
        (0.385, 0.205, "front_lower"),
        (-0.300, 0.470, "rear_seat"),
        (0.385, 0.470, "front_seat"),
        (0.050, 0.470, "middle_seat"),
    ):
        _cyl_x(frame, 0.016, 0.540, (0.0, y, z), frame_paint, f"{label}_cross_tube")

    # Retrofit service box and hatches under the cushion.
    frame.visual(Box((0.370, 0.265, 0.170)), origin=Origin(xyz=(0.0, 0.030, 0.385)), material=dark_steel, name="service_box")
    frame.visual(Box((0.010, 0.160, 0.100)), origin=Origin(xyz=(0.183, 0.030, 0.385)), material=hatch_paint, name="left_service_hatch")
    frame.visual(Box((0.010, 0.160, 0.100)), origin=Origin(xyz=(-0.183, 0.030, 0.385)), material=hatch_paint, name="right_service_hatch")
    for side, label in ((1.0, "left"), (-1.0, "right")):
        for yy in (-0.045, 0.045):
            for zz in (-0.028, 0.028):
                frame.visual(
                    Cylinder(radius=0.005, length=0.004),
                    origin=Origin(xyz=(side * 0.190, 0.030 + yy, 0.385 + zz), rpy=(0.0, pi / 2.0, 0.0)),
                    material=steel,
                    name=f"{label}_hatch_screw_{yy:+.2f}_{zz:+.2f}",
                )

    frame.visual(Box((0.460, 0.405, 0.032)), origin=Origin(xyz=(0.0, 0.035, 0.492)), material=dark_steel, name="seat_pan")
    frame.visual(Box((0.435, 0.380, 0.062)), origin=Origin(xyz=(0.0, 0.035, 0.539)), material=vinyl, name="seat_cushion")
    frame.visual(Box((0.410, 0.030, 0.012)), origin=Origin(xyz=(0.0, -0.166, 0.571)), material=steel, name="rear_cushion_clamp")
    frame.visual(Box((0.410, 0.030, 0.012)), origin=Origin(xyz=(0.0, 0.236, 0.571)), material=steel, name="front_cushion_clamp")

    # Scab plates and gussets at hard-working frame joints.
    for side, label in ((1.0, "left"), (-1.0, "right")):
        frame.visual(Box((0.044, 0.120, 0.014)), origin=Origin(xyz=(side * 0.255, 0.358, 0.468)), material=steel, name=f"{label}_front_gusset")
        frame.visual(Box((0.044, 0.120, 0.014)), origin=Origin(xyz=(side * 0.255, -0.270, 0.468)), material=steel, name=f"{label}_rear_gusset")
        frame.visual(Box((0.090, 0.075, 0.020)), origin=Origin(xyz=(side * 0.255, 0.430, 0.265)), material=steel, name=f"{label}_caster_clamp")

    left_drive_wheel = model.part("left_drive_wheel")
    right_drive_wheel = model.part("right_drive_wheel")
    _add_drive_wheel(left_drive_wheel, prefix="left_drive", side=1.0, materials=(steel, dark_steel, black))
    _add_drive_wheel(right_drive_wheel, prefix="right_drive", side=-1.0, materials=(steel, dark_steel, black))

    left_caster_yoke = model.part("left_caster_yoke")
    right_caster_yoke = model.part("right_caster_yoke")
    for part, side, label in ((left_caster_yoke, 1.0, "left"), (right_caster_yoke, -1.0, "right")):
        part.visual(
            Cylinder(radius=0.018, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=dark_steel,
            name="swivel_stem",
        )
        part.visual(Box((0.085, 0.050, 0.018)), origin=Origin(xyz=(0.0, 0.0, -0.067)), material=dark_steel, name="fork_crown")
        part.visual(Box((0.012, 0.052, 0.130)), origin=Origin(xyz=(0.035, 0.0, -0.128)), material=dark_steel, name="outer_fork_cheek")
        part.visual(Box((0.012, 0.052, 0.130)), origin=Origin(xyz=(-0.035, 0.0, -0.128)), material=dark_steel, name="inner_fork_cheek")
        part.visual(
            Cylinder(radius=0.008, length=0.086),
            origin=Origin(xyz=(0.0, 0.0, -0.172), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="axle_pin",
        )
        part.visual(Box((0.055, 0.010, 0.025)), origin=Origin(xyz=(0.0, -0.026, -0.085)), material=steel, name="fork_bridge_plate")

    left_caster_wheel = model.part("left_caster_wheel")
    right_caster_wheel = model.part("right_caster_wheel")
    _add_caster_wheel(left_caster_wheel, prefix="left_caster", materials=(steel, dark_steel, black))
    _add_caster_wheel(right_caster_wheel, prefix="right_caster", materials=(steel, dark_steel, black))

    backrest = model.part("backrest")
    for side, label in ((1.0, "left"), (-1.0, "right")):
        _cyl_z(backrest, 0.017, 0.590, (side * 0.225, -0.020, 0.295), frame_paint, f"{label}_cane")
        _cyl_y(backrest, 0.015, 0.105, (side * 0.225, -0.075, 0.602), frame_paint, f"{label}_push_handle")
    backrest.visual(
        Cylinder(radius=0.022, length=0.080),
        origin=Origin(xyz=(0.265, -0.020, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    backrest.visual(
        Cylinder(radius=0.022, length=0.080),
        origin=Origin(xyz=(-0.265, -0.020, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_barrel",
    )
    _cyl_x(backrest, 0.014, 0.450, (0.0, -0.020, 0.165), frame_paint, "lower_back_cross")
    _cyl_x(backrest, 0.014, 0.450, (0.0, -0.020, 0.510), frame_paint, "upper_back_cross")
    backrest.visual(Box((0.405, 0.026, 0.315)), origin=Origin(xyz=(0.0, -0.044, 0.330)), material=vinyl, name="back_sling")
    backrest.visual(Box((0.380, 0.010, 0.030)), origin=Origin(xyz=(0.0, -0.060, 0.235)), material=steel, name="sling_service_strip")

    left_footrest = model.part("left_footrest")
    right_footrest = model.part("right_footrest")
    for part, side, label in ((left_footrest, 1.0, "left"), (right_footrest, -1.0, "right")):
        part.visual(
            Cylinder(radius=0.016, length=0.185),
            origin=Origin(xyz=(0.0, 0.0, -0.092)),
            material=dark_steel,
            name="hinge_tube",
        )
        _cyl_y(part, 0.014, 0.255, (side * 0.010, 0.128, -0.175), frame_paint, "lower_swing_tube")
        part.visual(Box((0.170, 0.240, 0.026)), origin=Origin(xyz=(side * 0.075, 0.270, -0.200)), material=dark_steel, name="footplate")
        part.visual(Box((0.155, 0.205, 0.008)), origin=Origin(xyz=(side * 0.075, 0.270, -0.178)), material=black, name="foot_grip_pad")
        part.visual(Box((0.050, 0.105, 0.060)), origin=Origin(xyz=(side * 0.022, 0.175, -0.175)), material=steel, name="folding_stop")

    model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(0.405, -0.055, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=22.0),
    )
    model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.405, -0.055, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=22.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_caster_yoke,
        origin=Origin(xyz=(0.255, 0.430, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_caster_yoke,
        origin=Origin(xyz=(-0.255, 0.430, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=left_caster_yoke,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=right_caster_yoke,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )
    model.articulation(
        "backrest_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.325, 0.500)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.18, upper=0.30),
    )
    model.articulation(
        "left_footrest_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_footrest,
        origin=Origin(xyz=(0.165, 0.430, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=1.20),
    )
    model.articulation(
        "right_footrest_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_footrest,
        origin=Origin(xyz=(-0.165, 0.430, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-1.20, upper=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_drive = object_model.get_part("left_drive_wheel")
    right_drive = object_model.get_part("right_drive_wheel")
    left_yoke = object_model.get_part("left_caster_yoke")
    right_yoke = object_model.get_part("right_caster_yoke")
    left_caster = object_model.get_part("left_caster_wheel")
    right_caster = object_model.get_part("right_caster_wheel")
    backrest = object_model.get_part("backrest")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")

    # Captured shafts and hinge barrels are intentionally modeled as seated
    # inside their bushings so the legacy mechanism reads as real hardware.
    for label, wheel in (("left", left_drive), ("right", right_drive)):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=f"{label}_axle_stub",
            elem_b="hub",
            reason="The drive wheel hub is captured on a protruding axle stub.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="x",
            elem_a=f"{label}_axle_stub",
            elem_b="hub",
            min_overlap=0.006,
            name=f"{label} drive axle inserts into hub",
        )

    for label, yoke in (("left", left_yoke), ("right", right_yoke)):
        ctx.allow_overlap(
            frame,
            yoke,
            elem_a=f"{label}_caster_sleeve",
            elem_b="swivel_stem",
            reason="The caster swivel stem is nested in the welded sleeve.",
        )
        ctx.expect_overlap(
            frame,
            yoke,
            axes="z",
            elem_a=f"{label}_caster_sleeve",
            elem_b="swivel_stem",
            min_overlap=0.030,
            name=f"{label} caster stem retained in sleeve",
        )

    for label, yoke, wheel in (("left", left_yoke, left_caster), ("right", right_yoke, right_caster)):
        ctx.allow_overlap(
            yoke,
            wheel,
            elem_a="axle_pin",
            elem_b="hub",
            reason="The small caster wheel rotates on a pin through its hub.",
        )
        ctx.expect_overlap(
            yoke,
            wheel,
            axes="x",
            elem_a="axle_pin",
            elem_b="hub",
            min_overlap=0.020,
            name=f"{label} caster axle passes through hub",
        )

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="left_back_hinge_lug",
        elem_b="left_hinge_barrel",
        reason="Backrest hinge barrel is captured between bolted frame lugs.",
    )
    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="right_back_hinge_lug",
        elem_b="right_hinge_barrel",
        reason="Backrest hinge barrel is captured between bolted frame lugs.",
    )
    ctx.expect_overlap(frame, backrest, axes="x", elem_a="left_back_hinge_lug", elem_b="left_hinge_barrel", min_overlap=0.015, name="left back hinge seated")
    ctx.expect_overlap(frame, backrest, axes="x", elem_a="right_back_hinge_lug", elem_b="right_hinge_barrel", min_overlap=0.015, name="right back hinge seated")

    for label, footrest in (("left", left_footrest), ("right", right_footrest)):
        ctx.allow_overlap(
            frame,
            footrest,
            elem_a=f"{label}_footrest_lug",
            elem_b="hinge_tube",
            reason="Swing-away footrest tube is captured in a legacy lug.",
        )
        ctx.expect_overlap(
            frame,
            footrest,
            axes="z",
            elem_a=f"{label}_footrest_lug",
            elem_b="hinge_tube",
            min_overlap=0.070,
            name=f"{label} footrest hinge tube retained",
        )

    ctx.expect_origin_distance(left_drive, right_drive, axes="x", min_dist=0.78, max_dist=0.84, name="drive wheels straddle chair")

    seat_aabb = ctx.part_element_world_aabb(frame, elem="seat_cushion")
    ctx.check(
        "seat height is wheelchair scale",
        seat_aabb is not None and 0.50 <= float(seat_aabb[1][2]) <= 0.59,
        details=f"seat_aabb={seat_aabb}",
    )

    for joint_name in (
        "left_drive_spin",
        "right_drive_spin",
        "left_caster_swivel",
        "right_caster_swivel",
        "backrest_pivot",
        "left_footrest_swing",
        "right_footrest_swing",
    ):
        ctx.check(f"{joint_name} present", object_model.get_articulation(joint_name) is not None)

    with ctx.pose({"left_caster_swivel": 0.80, "right_caster_swivel": -0.80}):
        ctx.expect_origin_distance(left_caster, right_caster, axes="x", min_dist=0.48, max_dist=0.56, name="caster swivels remain fork-supported")

    with ctx.pose({"backrest_pivot": 0.25, "left_footrest_swing": 0.75, "right_footrest_swing": -0.75}):
        ctx.expect_origin_distance(backrest, frame, axes="z", min_dist=0.49, max_dist=0.52, name="backrest stays hinged to frame")

    return ctx.report()


object_model = build_object_model()
