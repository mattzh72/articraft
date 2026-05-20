from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
)


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, radius, length, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _wheel_meshes(radius: float, width: float, name: str, *, spokes: int):
    rim = WheelGeometry(
        radius * 0.83,
        width * 0.78,
        rim=WheelRim(
            inner_radius=radius * 0.57,
            flange_height=radius * 0.025,
            flange_thickness=0.004,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=radius * 0.115,
            width=width * 0.82,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=6,
                circle_diameter=radius * 0.17,
                hole_diameter=0.004,
            ),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
        spokes=WheelSpokes(
            style="split_y",
            count=spokes,
            thickness=max(0.003, radius * 0.012),
            window_radius=radius * 0.055,
        ),
        bore=WheelBore(style="round", diameter=radius * 0.055),
    )
    tire = TireGeometry(
        radius,
        width,
        inner_radius=radius * 0.86,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.045),
        tread=TireTread(style="block", depth=radius * 0.012, count=28, land_ratio=0.62),
        grooves=(TireGroove(center_offset=0.0, width=width * 0.16, depth=radius * 0.006),),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=width * 0.12, radius=0.004),
    )
    return mesh_from_geometry(rim, f"{name}_rim"), mesh_from_geometry(tire, f"{name}_tire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_wheelchair")

    aluminum = model.material("brushed_anodized_aluminum", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_aluminum = model.material("dark_hard_anodized", rgba=(0.10, 0.11, 0.12, 1.0))
    cushion = model.material("matte_black_vinyl", rgba=(0.015, 0.016, 0.017, 1.0))
    tire_mat = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    rim_mat = model.material("machined_silver_rim", rgba=(0.82, 0.83, 0.80, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.05, 0.25, 0.95, 1.0))
    index_white = model.material("engraved_white_index", rgba=(0.92, 0.92, 0.86, 1.0))
    adjustment_orange = model.material("adjustment_orange", rgba=(1.0, 0.42, 0.05, 1.0))
    green_gap = model.material("controlled_gap_green", rgba=(0.0, 0.65, 0.32, 1.0))

    frame = model.part("frame")
    _box(frame, (0.58, 0.52, 0.035), (0.03, 0.0, 0.50), cushion, "flat_seat_datum")
    _box(frame, (0.75, 0.050, 0.060), (0.03, 0.265, 0.38), aluminum, "side_beam_0")
    _box(frame, (0.75, 0.050, 0.060), (0.03, -0.265, 0.38), aluminum, "side_beam_1")
    for i, y in enumerate((0.245, -0.245)):
        rear_y = 0.285 if y > 0.0 else -0.285
        _box(frame, (0.045, 0.045, 0.245), (-0.260, rear_y, 0.465), aluminum, f"rear_post_{i}")
        _box(frame, (0.045, 0.045, 0.285), (0.340, y, 0.360), aluminum, f"front_post_{i}")
        _box(frame, (0.150, 0.105, 0.030), (0.410, y * 0.86, 0.225), aluminum, f"caster_datum_pad_{i}")
        _box(frame, (0.235, 0.052, 0.160), (-0.250, y / abs(y) * 0.305, 0.320), dark_aluminum, f"axle_index_rail_{i}")
        _box(frame, (0.060, 0.060, 0.035), (0.2875, y * 0.86, 0.4275), dark_aluminum, f"footrest_socket_{i}")
    _box(frame, (0.115, 0.465, 0.040), (0.365, 0.0, 0.225), aluminum, "front_cross_datum")
    _box(frame, (0.060, 0.550, 0.035), (-0.250, 0.0, 0.5175), aluminum, "back_hinge_beam")
    _box(frame, (0.047, 0.635, 0.030), (-0.306, 0.0, 0.550), datum_blue, "rear_alignment_bar")
    _box(frame, (0.380, 0.055, 0.030), (0.060, 0.0, 0.485), datum_blue, "seat_center_datum")

    # Thin proud index ticks and green controlled-gap labels are slightly
    # embedded in their rail faces so they are supported, not floating decals.
    for rail_i, sign in enumerate((1.0, -1.0)):
        face_y = sign * 0.331
        for tick_i, x in enumerate((-0.335, -0.300, -0.265, -0.230, -0.195)):
            _box(
                frame,
                (0.006, 0.004, 0.070 if tick_i == 2 else 0.045),
                (x, face_y, 0.372),
                index_white,
                f"axle_tick_{rail_i}_{tick_i}",
            )
        _box(frame, (0.090, 0.004, 0.018), (-0.265, face_y, 0.258), green_gap, f"gap_label_{rail_i}")

    backrest = model.part("backrest")
    _cyl(backrest, 0.018, 0.510, (0.0, 0.0, 0.0), dark_aluminum, "back_hinge_barrel", rpy=(math.pi / 2, 0.0, 0.0))
    _box(backrest, (0.055, 0.480, 0.420), (0.0, 0.0, 0.228), cushion, "flat_back_datum")
    _box(backrest, (0.030, 0.035, 0.190), (-0.005, 0.215, 0.520), dark_aluminum, "push_handle_0")
    _box(backrest, (0.030, 0.035, 0.190), (-0.005, -0.215, 0.520), dark_aluminum, "push_handle_1")
    _box(backrest, (0.100, 0.034, 0.030), (0.035, 0.215, 0.610), cushion, "handle_grip_0")
    _box(backrest, (0.100, 0.034, 0.030), (0.035, -0.215, 0.610), cushion, "handle_grip_1")
    _box(backrest, (0.010, 0.450, 0.022), (0.029, 0.0, 0.120), datum_blue, "back_angle_datum")
    for i, y in enumerate((-0.160, -0.080, 0.0, 0.080, 0.160)):
        _box(backrest, (0.012, 0.004, 0.055), (0.034, y, 0.125), index_white, f"back_angle_tick_{i}")

    model.articulation(
        "back_angle_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.250, 0.0, 0.553)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.7, lower=0.0, upper=0.35),
    )

    drive_rim_mesh, drive_tire_mesh = _wheel_meshes(0.320, 0.055, "drive_wheel", spokes=12)
    for i, sign in enumerate((1.0, -1.0)):
        block = model.part(f"axle_block_{i}")
        _box(block, (0.105, 0.020, 0.130), (0.0, 0.0, 0.0), dark_aluminum, "slider_plate")
        _box(block, (0.088, 0.004, 0.018), (0.0, -sign * 0.012, -0.052), green_gap, "gap_witness")
        _cyl(block, 0.015, 0.020, (0.030, sign * 0.020, 0.036), adjustment_orange, "camber_clamp_knob", rpy=(math.pi / 2, 0.0, 0.0))
        _cyl(block, 0.010, 0.0275, (0.0, sign * 0.02375, 0.0), aluminum, "axle_stub", rpy=(math.pi / 2, 0.0, 0.0))

        model.articulation(
            f"axle_adjust_{i}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=block,
            origin=Origin(xyz=(-0.250, sign * 0.340, 0.320)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.04, lower=-0.040, upper=0.040),
        )

        wheel = model.part(f"drive_wheel_{i}")
        wheel.visual(
            drive_rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=rim_mat,
            name="rim",
        )
        wheel.visual(
            drive_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=tire_mat,
            name="tire",
        )
        _cyl(wheel, 0.020, 0.008, (0.0, -sign * 0.0235, 0.0), adjustment_orange, "axle_reference_cap", rpy=(math.pi / 2, 0.0, 0.0))
        model.articulation(
            f"drive_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=block,
            child=wheel,
            origin=Origin(xyz=(0.0, sign * 0.065, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=14.0),
        )

    caster_rim_mesh, caster_tire_mesh = _wheel_meshes(0.075, 0.034, "caster_wheel", spokes=6)
    for i, sign in enumerate((1.0, -1.0)):
        y = sign * 0.210
        yoke = model.part(f"caster_yoke_{i}")
        _cyl(yoke, 0.018, 0.070, (0.0, 0.0, -0.035), aluminum, "swivel_stem")
        _box(yoke, (0.075, 0.085, 0.028), (0.0, 0.0, -0.050), dark_aluminum, "fork_bridge")
        _box(yoke, (0.030, 0.010, 0.130), (0.0, 0.031, -0.125), dark_aluminum, "fork_arm_0")
        _box(yoke, (0.030, 0.010, 0.130), (0.0, -0.031, -0.125), dark_aluminum, "fork_arm_1")
        _cyl(yoke, 0.008, 0.014, (0.0, 0.0245, -0.165), aluminum, "axle_boss_0", rpy=(math.pi / 2, 0.0, 0.0))
        _cyl(yoke, 0.008, 0.014, (0.0, -0.0245, -0.165), aluminum, "axle_boss_1", rpy=(math.pi / 2, 0.0, 0.0))
        _box(yoke, (0.065, 0.004, 0.012), (0.0, sign * 0.044, -0.062), datum_blue, "caster_zero_mark")

        model.articulation(
            f"caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(0.410, y, 0.210)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=5.0),
        )

        caster = model.part(f"caster_wheel_{i}")
        caster.visual(
            caster_rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=rim_mat,
            name="rim",
        )
        caster.visual(
            caster_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=tire_mat,
            name="tire",
        )
        model.articulation(
            f"caster_roll_{i}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=caster,
            origin=Origin(xyz=(0.0, 0.0, -0.165)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    for i, sign in enumerate((1.0, -1.0)):
        footrest = model.part(f"footrest_{i}")
        _cyl(footrest, 0.014, 0.260, (0.0, 0.0, -0.130), dark_aluminum, "hanger_stem")
        foot_y = sign * 0.110
        _box(footrest, (0.030, 0.140, 0.026), (0.0, sign * 0.055, -0.265), aluminum, "offset_link")
        _box(footrest, (0.200, 0.034, 0.026), (0.095, foot_y, -0.265), aluminum, "forward_link")
        _box(footrest, (0.300, 0.060, 0.026), (0.320, foot_y, -0.265), aluminum, "footplate_arm")
        _box(footrest, (0.235, 0.155, 0.016), (0.390, foot_y, -0.286), dark_aluminum, "flat_foot_datum")
        _box(footrest, (0.185, 0.005, 0.004), (0.390, foot_y + sign * 0.058, -0.276), datum_blue, "toe_datum_edge")
        _box(footrest, (0.185, 0.005, 0.004), (0.390, foot_y - sign * 0.058, -0.276), datum_blue, "heel_datum_edge")
        for tick_i, x in enumerate((0.145, 0.185, 0.225, 0.265, 0.305)):
            _box(footrest, (0.006, 0.110, 0.004), (x + 0.170, foot_y, -0.276), index_white, f"foot_tick_{tick_i}")
        _cyl(footrest, 0.012, 0.020, (0.0, 0.0, -0.015), adjustment_orange, "height_index_collar")

        lower, upper = (0.0, 1.15) if sign > 0 else (-1.15, 0.0)
        model.articulation(
            f"footrest_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=footrest,
            origin=Origin(xyz=(0.2875, sign * 0.210, 0.410)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    back_hinge = object_model.get_articulation("back_angle_hinge")
    ctx.expect_contact(
        frame,
        backrest,
        elem_a="back_hinge_beam",
        elem_b="back_hinge_barrel",
        contact_tol=0.004,
        name="back hinge barrel is seated on frame beam",
    )

    for i, side_name in enumerate(("positive side", "negative side")):
        axle = object_model.get_part(f"axle_block_{i}")
        wheel = object_model.get_part(f"drive_wheel_{i}")
        adjust = object_model.get_articulation(f"axle_adjust_{i}")
        spin = object_model.get_articulation(f"drive_spin_{i}")
        ctx.check(
            f"{side_name} drive wheel has continuous spin",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=str(spin.articulation_type),
        )
        ctx.check(
            f"{side_name} axle block is a calibration slider",
            adjust.articulation_type == ArticulationType.PRISMATIC
            and adjust.motion_limits is not None
            and adjust.motion_limits.lower == -0.040
            and adjust.motion_limits.upper == 0.040,
            details=str(adjust.motion_limits),
        )
        ctx.expect_overlap(
            axle,
            frame,
            axes="xz",
            min_overlap=0.070,
            elem_a="slider_plate",
            elem_b=f"axle_index_rail_{i}",
            name=f"{side_name} slider is retained in indexed rail projection",
        )
        if i == 1:
            ctx.allow_overlap(
                axle,
                wheel,
                elem_a="axle_stub",
                elem_b="rim",
                reason="The axle stub is intentionally captured inside the wheel hub/rim bore on this side.",
            )
            ctx.expect_overlap(
                axle,
                wheel,
                axes="xz",
                min_overlap=0.015,
                elem_a="axle_stub",
                elem_b="rim",
                name="negative side axle stub remains captured in rim bore",
            )
        if i == 0:
            ctx.expect_gap(
                wheel,
                frame,
                axis="y",
                min_gap=0.020,
                max_gap=0.080,
                name="positive side drive wheel has controlled frame clearance",
            )
        else:
            ctx.expect_gap(
                frame,
                wheel,
                axis="y",
                min_gap=0.020,
                max_gap=0.080,
                name="negative side drive wheel has controlled frame clearance",
            )

    for i in (0, 1):
        yoke = object_model.get_part(f"caster_yoke_{i}")
        caster = object_model.get_part(f"caster_wheel_{i}")
        swivel = object_model.get_articulation(f"caster_swivel_{i}")
        roll = object_model.get_articulation(f"caster_roll_{i}")
        ctx.check(
            f"caster {i} has vertical continuous swivel",
            swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"type={swivel.articulation_type}, axis={swivel.axis}",
        )
        ctx.check(
            f"caster {i} has rolling axle",
            roll.articulation_type == ArticulationType.CONTINUOUS,
            details=str(roll.articulation_type),
        )
        ctx.expect_contact(
            yoke,
            caster,
            elem_a="axle_boss_0",
            elem_b="rim",
            contact_tol=0.006,
            name=f"caster {i} fork bosses reach wheel axle region",
        )

    for i in (0, 1):
        footrest = object_model.get_part(f"footrest_{i}")
        ctx.expect_contact(
            frame,
            footrest,
            elem_a=f"footrest_socket_{i}",
            elem_b="hanger_stem",
            contact_tol=0.003,
            name=f"footrest {i} hanger is seated in socket",
        )

    rest_back_aabb = ctx.part_element_world_aabb(backrest, elem="flat_back_datum")
    rest_back_x = None if rest_back_aabb is None else (rest_back_aabb[0][0] + rest_back_aabb[1][0]) / 2.0
    with ctx.pose({back_hinge: 0.30}):
        reclined_back_aabb = ctx.part_element_world_aabb(backrest, elem="flat_back_datum")
        reclined_back_x = None if reclined_back_aabb is None else (reclined_back_aabb[0][0] + reclined_back_aabb[1][0]) / 2.0
    ctx.check(
        "back frame joint reclines rearward",
        rest_back_x is not None and reclined_back_x is not None and reclined_back_x < rest_back_x - 0.010,
        details=f"rest_x={rest_back_x}, reclined_x={reclined_back_x}",
    )

    block_0 = object_model.get_part("axle_block_0")
    axle_adjust_0 = object_model.get_articulation("axle_adjust_0")
    rest_axle = ctx.part_world_position(block_0)
    with ctx.pose({axle_adjust_0: 0.035}):
        shifted_axle = ctx.part_world_position(block_0)
        ctx.expect_overlap(
            block_0,
            frame,
            axes="xz",
            min_overlap=0.050,
            elem_a="slider_plate",
            elem_b="axle_index_rail_0",
            name="shifted axle block remains captured by index rail",
        )
    ctx.check(
        "axle calibration slider moves forward on datum rail",
        rest_axle is not None and shifted_axle is not None and shifted_axle[0] > rest_axle[0] + 0.030,
        details=f"rest={rest_axle}, shifted={shifted_axle}",
    )

    return ctx.report()


object_model = build_object_model()
