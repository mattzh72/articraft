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


def _tube_between(part, name, start, end, radius, material, extra=0.010):
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length + extra),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_wheelchair")

    frame_mat = model.material("anodized_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    hinge_mat = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    fabric_mat = model.material("navy_fabric", rgba=(0.02, 0.05, 0.12, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    rim_mat = model.material("satin_spokes", rgba=(0.82, 0.83, 0.80, 1.0))
    foot_mat = model.material("textured_footplate", rgba=(0.06, 0.065, 0.07, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.43, 0.420, 0.025)),
        origin=Origin(xyz=(0.035, 0.0, 0.465)),
        material=fabric_mat,
        name="seat_sling",
    )
    # Compact welded tubular base: narrow enough for an apartment desk while
    # keeping large rear drive wheels clear of the fabric and side rails.
    _tube_between(frame, "side_rail_0", (-0.19, 0.205, 0.455), (0.285, 0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "side_rail_1", (-0.19, -0.205, 0.455), (0.285, -0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "front_rail", (0.285, -0.205, 0.455), (0.285, 0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "rear_rail", (-0.19, -0.205, 0.455), (-0.19, 0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "rear_axle", (-0.19, -0.390, 0.285), (-0.19, 0.390, 0.285), 0.014, hinge_mat)
    _tube_between(frame, "upright_0", (-0.19, 0.205, 0.285), (-0.19, 0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "upright_1", (-0.19, -0.205, 0.285), (-0.19, -0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "lower_rail_0", (-0.19, 0.205, 0.285), (0.285, 0.205, 0.165), 0.011, frame_mat)
    _tube_between(frame, "lower_rail_1", (-0.19, -0.205, 0.285), (0.285, -0.205, 0.165), 0.011, frame_mat)
    _tube_between(frame, "front_post_0", (0.285, 0.205, 0.165), (0.285, 0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "front_post_1", (0.285, -0.205, 0.165), (0.285, -0.205, 0.455), 0.012, frame_mat)
    _tube_between(frame, "caster_socket_0", (0.335, 0.205, 0.118), (0.335, 0.205, 0.225), 0.018, hinge_mat)
    _tube_between(frame, "caster_socket_1", (0.335, -0.205, 0.118), (0.335, -0.205, 0.225), 0.018, hinge_mat)
    _tube_between(frame, "caster_strut_0", (0.285, 0.205, 0.235), (0.335, 0.205, 0.235), 0.011, frame_mat)
    _tube_between(frame, "caster_strut_1", (0.285, -0.205, 0.235), (0.335, -0.205, 0.235), 0.011, frame_mat)
    _tube_between(frame, "back_socket_0", (-0.195, 0.205, 0.455), (-0.195, 0.205, 0.520), 0.012, frame_mat)
    _tube_between(frame, "back_socket_1", (-0.195, -0.205, 0.455), (-0.195, -0.205, 0.520), 0.012, frame_mat)
    _tube_between(frame, "back_hinge_barrel", (-0.195, -0.225, 0.520), (-0.195, 0.225, 0.520), 0.016, hinge_mat)
    _tube_between(frame, "fold_crossbrace_0", (-0.185, -0.195, 0.300), (0.255, 0.195, 0.445), 0.008, hinge_mat, extra=0.030)
    _tube_between(frame, "fold_crossbrace_1", (-0.185, 0.195, 0.300), (0.255, -0.195, 0.445), 0.008, hinge_mat, extra=0.030)
    for index, y in enumerate((0.155, -0.155)):
        _tube_between(frame, f"footrest_socket_{index}", (0.315, y, 0.245), (0.315, y, 0.375), 0.015, hinge_mat)
        _tube_between(frame, f"footrest_bracket_{index}", (0.285, 0.205 if y > 0.0 else -0.205, 0.365), (0.315, y, 0.365), 0.009, frame_mat)
    for index, y in enumerate((0.335, -0.335)):
        collar_y = y - 0.026 if y > 0.0 else y + 0.026
        _tube_between(
            frame,
            f"axle_bearing_{index}",
            (-0.19, collar_y - (0.012 if y > 0.0 else -0.012), 0.285),
            (-0.19, collar_y + (0.012 if y > 0.0 else -0.012), 0.285),
            0.036,
            hinge_mat,
            extra=0.0,
        )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.035, 0.390, 0.300)),
        origin=Origin(xyz=(-0.012, 0.0, 0.190)),
        material=fabric_mat,
        name="back_sling",
    )
    _tube_between(backrest, "back_post_0", (-0.015, 0.205, 0.000), (-0.015, 0.205, 0.380), 0.012, frame_mat)
    _tube_between(backrest, "back_post_1", (-0.015, -0.205, 0.000), (-0.015, -0.205, 0.380), 0.012, frame_mat)
    _tube_between(backrest, "push_handle_0", (-0.015, 0.205, 0.360), (0.065, 0.205, 0.410), 0.011, frame_mat)
    _tube_between(backrest, "push_handle_1", (-0.015, -0.205, 0.360), (0.065, -0.205, 0.410), 0.011, frame_mat)
    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.195, 0.0, 0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    drive_wheel_mesh = WheelGeometry(
        0.218,
        0.046,
        rim=WheelRim(inner_radius=0.155, flange_height=0.010, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.045,
            width=0.040,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.058, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.007, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="straight", count=12, thickness=0.003, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.044),
    )
    drive_tire_mesh = TireGeometry(
        0.255,
        0.052,
        inner_radius=0.218,
        tread=TireTread(style="circumferential", depth=0.004, count=4),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
    )
    handrim_mesh = TireGeometry(
        0.214,
        0.010,
        inner_radius=0.207,
        sidewall=TireSidewall(style="rounded", bulge=0.03),
    )
    for index, y in enumerate((0.335, -0.335)):
        wheel = model.part(f"drive_wheel_{index}")
        wheel.visual(
            mesh_from_geometry(drive_wheel_mesh, f"drive_rim_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_mat,
            name="rim",
        )
        wheel.visual(
            mesh_from_geometry(drive_tire_mesh, f"drive_tire_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber_mat,
            name="tire",
        )
        handrim_y = 0.034 if y > 0.0 else -0.034
        wheel.visual(
            mesh_from_geometry(handrim_mesh, f"handrim_{index}"),
            origin=Origin(xyz=(0.0, handrim_y, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=hinge_mat,
            name="handrim",
        )
        for spoke_i, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0)):
            r = 0.185
            x = math.cos(angle) * r
            z = math.sin(angle) * r
            _tube_between(
                wheel,
                f"handrim_standoff_{spoke_i}",
                (x, handrim_y * 0.62, z),
                (x, handrim_y, z),
                0.0035,
                hinge_mat,
                extra=0.003,
            )
        model.articulation(
            f"frame_to_drive_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.190, y, 0.285)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=22.0, velocity=8.0),
        )

    caster_wheel_mesh = WheelGeometry(
        0.042,
        0.026,
        rim=WheelRim(inner_radius=0.028, flange_height=0.004, flange_thickness=0.002),
        hub=WheelHub(radius=0.014, width=0.020, cap_style="domed"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.002, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.010),
    )
    caster_tire_mesh = TireGeometry(
        0.055,
        0.030,
        inner_radius=0.043,
        tread=TireTread(style="circumferential", depth=0.002, count=2),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
    )
    for index, y in enumerate((0.205, -0.205)):
        fork = model.part(f"caster_swivel_{index}")
        _tube_between(fork, "swivel_stem", (0.0, 0.0, -0.068), (0.0, 0.0, 0.035), 0.010, hinge_mat)
        fork.visual(
            Box((0.074, 0.078, 0.010)),
            origin=Origin(xyz=(-0.032, 0.0, -0.068)),
            material=hinge_mat,
            name="fork_crown",
        )
        fork.visual(
            Box((0.016, 0.010, 0.135)),
            origin=Origin(xyz=(-0.032, 0.026, -0.112)),
            material=hinge_mat,
            name="fork_arm_0",
        )
        fork.visual(
            Box((0.016, 0.010, 0.135)),
            origin=Origin(xyz=(-0.032, -0.026, -0.112)),
            material=hinge_mat,
            name="fork_arm_1",
        )
        _tube_between(fork, "caster_axle", (-0.032, -0.034, -0.135), (-0.032, 0.034, -0.135), 0.006, hinge_mat)
        model.articulation(
            f"frame_to_caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.335, y, 0.175)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=7.0),
        )
        caster = model.part(f"caster_wheel_{index}")
        caster.visual(
            mesh_from_geometry(caster_wheel_mesh, f"caster_rim_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_mat,
            name="rim",
        )
        caster.visual(
            mesh_from_geometry(caster_tire_mesh, f"caster_tire_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber_mat,
            name="tire",
        )
        model.articulation(
            f"caster_swivel_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=caster,
            origin=Origin(xyz=(-0.032, 0.0, -0.135)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=9.0),
        )

    for index, y in enumerate((0.155, -0.155)):
        footrest = model.part(f"footrest_{index}")
        _tube_between(footrest, "hanger_post", (0.0, 0.0, 0.000), (0.0, 0.0, -0.220), 0.010, frame_mat)
        _tube_between(footrest, "lower_hanger", (0.0, 0.0, -0.210), (0.125, 0.0, -0.235), 0.009, frame_mat)
        footrest.visual(
            Box((0.040, 0.034, 0.022)),
            origin=Origin(xyz=(0.125, 0.0, -0.235)),
            material=hinge_mat,
            name="plate_hinge",
        )
        model.articulation(
            f"frame_to_footrest_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=footrest,
            origin=Origin(xyz=(0.315, y, 0.365)),
            axis=(0.0, 0.0, 1.0 if y > 0.0 else -1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
        )
        plate = model.part(f"footplate_{index}")
        plate.visual(
            Box((0.175, 0.115, 0.014)),
            origin=Origin(xyz=(0.1075, 0.0, -0.007)),
            material=foot_mat,
            name="tread_plate",
        )
        plate.visual(
            Box((0.155, 0.095, 0.004)),
            origin=Origin(xyz=(0.1145, 0.0, 0.002)),
            material=rubber_mat,
            name="grip_pad",
        )
        model.articulation(
            f"footrest_to_plate_{index}",
            ArticulationType.REVOLUTE,
            parent=footrest,
            child=plate,
            origin=Origin(xyz=(0.125, 0.0, -0.235)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    drive_0 = object_model.get_part("drive_wheel_0")
    drive_1 = object_model.get_part("drive_wheel_1")
    plate_0 = object_model.get_part("footplate_0")
    plate_1 = object_model.get_part("footplate_1")

    for index in (0, 1):
        ctx.allow_overlap(
            "caster_swivel_%d" % index,
            frame,
            elem_a="swivel_stem",
            elem_b=f"caster_socket_{index}",
            reason="The caster stem is intentionally captured inside the vertical swivel socket.",
        )
        ctx.expect_overlap(
            f"caster_swivel_{index}",
            frame,
            axes="z",
            min_overlap=0.050,
            name=f"caster_{index}_stem_retained_in_socket",
        )
        ctx.allow_overlap(
            f"caster_swivel_{index}",
            f"caster_wheel_{index}",
            elem_a="caster_axle",
            elem_b="rim",
            reason="The caster wheel hub rotates around a captured axle shaft through the fork.",
        )
        ctx.expect_overlap(
            f"caster_swivel_{index}",
            f"caster_wheel_{index}",
            axes="y",
            min_overlap=0.020,
            name=f"caster_{index}_axle_through_hub",
        )
        ctx.allow_overlap(
            f"footrest_{index}",
            frame,
            elem_a="hanger_post",
            elem_b=f"footrest_socket_{index}",
            reason="The swing-away footrest hanger is represented as a retained pin inside its frame socket.",
        )
        ctx.expect_overlap(
            f"footrest_{index}",
            frame,
            axes="z",
            min_overlap=0.080,
            name=f"footrest_{index}_hanger_retained",
        )
        ctx.allow_overlap(
            f"footrest_{index}",
            frame,
            elem_a="hanger_post",
            elem_b=f"footrest_bracket_{index}",
            reason="The footrest hanger passes through the welded upper bracket before entering the socket.",
        )
        ctx.expect_contact(
            f"footrest_{index}",
            frame,
            contact_tol=0.002,
            name=f"footrest_{index}_hanger_supported_by_bracket",
        )
        ctx.allow_overlap(
            f"drive_wheel_{index}",
            frame,
            elem_a="rim",
            elem_b=f"axle_bearing_{index}",
            reason="The compact drive wheel hub is captured by a narrow bearing collar on the rear axle.",
        )
        ctx.expect_overlap(
            f"drive_wheel_{index}",
            frame,
            axes="y",
            min_overlap=0.004,
            elem_a="rim",
            elem_b=f"axle_bearing_{index}",
            name=f"drive_wheel_{index}_bearing_retained",
        )

    for index in (0, 1):
        ctx.allow_overlap(
            backrest,
            frame,
            elem_a=f"back_post_{index}",
            elem_b="back_hinge_barrel",
            reason="The folding back cane lower knuckle is captured around the frame hinge barrel.",
        )
        ctx.expect_overlap(
            backrest,
            frame,
            axes="yz",
            min_overlap=0.014,
            name=f"back_post_{index}_hinge_retained",
        )
        ctx.allow_overlap(
            backrest,
            frame,
            elem_a=f"back_post_{index}",
            elem_b=f"back_socket_{index}",
            reason="The folding back cane lower end nests into the rear receiver socket at the upright pose.",
        )
        ctx.expect_contact(
            backrest,
            frame,
            contact_tol=0.002,
            name=f"back_post_{index}_seated_in_socket",
        )

    ctx.check(
        "articulated_wheelchair_mechanisms",
        all(object_model.get_articulation(name) is not None for name in (
            "frame_to_drive_wheel_0",
            "frame_to_drive_wheel_1",
            "frame_to_caster_swivel_0",
            "frame_to_caster_swivel_1",
            "caster_swivel_to_wheel_0",
            "caster_swivel_to_wheel_1",
            "frame_to_backrest",
            "frame_to_footrest_0",
            "frame_to_footrest_1",
            "footrest_to_plate_0",
            "footrest_to_plate_1",
        )),
        "Expected explicit drive wheel, caster swivel, folding back, swing-away footrest, and fold-up footplate joints.",
    )

    ctx.expect_gap(
        drive_0,
        frame,
        axis="y",
        min_gap=0.050,
        positive_elem="tire",
        negative_elem="seat_sling",
        name="positive_drive_wheel_clears_seat",
    )
    ctx.expect_gap(
        frame,
        drive_1,
        axis="y",
        min_gap=0.050,
        positive_elem="seat_sling",
        negative_elem="tire",
        name="negative_drive_wheel_clears_seat",
    )

    back_hinge = object_model.get_articulation("frame_to_backrest")
    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 1.35}):
        folded_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest_folds_forward_lower",
        rest_back_aabb is not None
        and folded_back_aabb is not None
        and folded_back_aabb[1][2] < rest_back_aabb[1][2] - 0.12
        and folded_back_aabb[1][0] > rest_back_aabb[1][0] + 0.20,
        details=f"rest={rest_back_aabb}, folded={folded_back_aabb}",
    )

    for index, plate in enumerate((plate_0, plate_1)):
        hinge = object_model.get_articulation(f"footrest_to_plate_{index}")
        rest_aabb = ctx.part_world_aabb(plate)
        with ctx.pose({hinge: 1.45}):
            folded_aabb = ctx.part_world_aabb(plate)
        ctx.check(
            f"footplate_{index}_folds_up",
            rest_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][2] > rest_aabb[1][2] + 0.10,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
