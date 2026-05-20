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


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _tube_x(part, name: str, *, x: float, y: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _tube_y(part, name: str, *, x: float, y: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _tube_z(part, name: str, *, x: float, y: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def _box(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _drive_wheel_meshes():
    wheel = WheelGeometry(
        0.265,
        0.044,
        rim=WheelRim(
            inner_radius=0.196,
            flange_height=0.010,
            flange_thickness=0.004,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.050,
            width=0.038,
            cap_style="flat",
            bolt_pattern=BoltPattern(
                count=6,
                circle_diameter=0.074,
                hole_diameter=0.006,
            ),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="radial", count=12, thickness=0.006, window_radius=0.020),
        bore=WheelBore(style="round", diameter=0.026),
    )
    tire = TireGeometry(
        0.305,
        0.055,
        inner_radius=0.258,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.06),
        tread=TireTread(style="ribbed", depth=0.004, count=36, land_ratio=0.68),
        grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.003),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    pushrim = TireGeometry(
        0.255,
        0.012,
        inner_radius=0.243,
        carcass=TireCarcass(belt_width_ratio=0.78, sidewall_bulge=0.02),
        sidewall=TireSidewall(style="rounded", bulge=0.02),
        shoulder=TireShoulder(width=0.002, radius=0.0015),
    )
    return (
        mesh_from_geometry(wheel, "drive_wheel_rim"),
        mesh_from_geometry(tire, "drive_wheel_tire"),
        mesh_from_geometry(pushrim, "drive_pushrim"),
    )


def _caster_wheel_meshes():
    wheel = WheelGeometry(
        0.052,
        0.030,
        rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.002),
        hub=WheelHub(radius=0.019, width=0.028, cap_style="flat"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="solid", count=3, thickness=0.003, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.010),
    )
    tire = TireGeometry(
        0.075,
        0.036,
        inner_radius=0.052,
        tread=TireTread(style="smooth", depth=0.0015, count=18, land_ratio=0.8),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    return (
        mesh_from_geometry(wheel, "caster_wheel_rim"),
        mesh_from_geometry(tire, "caster_wheel_tire"),
    )


def _add_frame(frame, metal, black, fabric, fastener):
    # Straight tubes and boxy clamp bosses keep the frame cheap to cut, bend, and bolt.
    _box(frame, "seat_panel", (0.50, 0.44, 0.030), (0.02, 0.0, 0.485), fabric)
    _box(frame, "front_seat_lip", (0.045, 0.44, 0.045), (0.265, 0.0, 0.470), fabric)
    _box(frame, "back_panel", (0.045, 0.44, 0.410), (-0.285, 0.0, 0.730), fabric)

    for side, y in (("left", 0.24), ("right", -0.24)):
        _tube_x(frame, f"{side}_side_rail", x=0.04, y=y, z=0.360, length=0.76, radius=0.016, material=metal)
        _tube_x(frame, f"{side}_seat_ledger", x=0.02, y=0.92 * y, z=0.456, length=0.50, radius=0.012, material=metal)
        _tube_z(frame, f"{side}_rear_post", x=-0.255, y=y, z=0.665, length=0.640, radius=0.016, material=metal)
        _tube_z(frame, f"{side}_front_post", x=0.385, y=y, z=0.355, length=0.285, radius=0.016, material=metal)
        _tube_z(frame, f"{side}_seat_strut_0", x=-0.170, y=0.92 * y, z=0.410, length=0.115, radius=0.010, material=metal)
        _tube_z(frame, f"{side}_seat_strut_1", x=0.210, y=0.92 * y, z=0.410, length=0.115, radius=0.010, material=metal)
        _tube_x(frame, f"{side}_footrest_arm", x=0.445, y=0.58 * y, z=0.235, length=0.150, radius=0.011, material=metal)
        _tube_z(frame, f"{side}_caster_socket", x=0.420, y=y, z=0.235, length=0.130, radius=0.022, material=black)
        _box(frame, f"{side}_caster_plate", (0.115, 0.066, 0.026), (0.385, y, 0.245), metal)
        _box(frame, f"{side}_axle_clamp", (0.070, 0.050, 0.065), (-0.165, y, 0.330), metal)
        _box(frame, f"{side}_seat_bolt", (0.020, 0.012, 0.010), (0.205, 0.92 * y, 0.476), fastener)
        _box(frame, f"{side}_back_bolt", (0.016, 0.012, 0.016), (-0.258, 0.92 * y, 0.785), fastener)

        # Two simple hinge cheeks accept the molded swing-up foot plate barrel.
        foot_y = 0.58 * y
        _box(frame, f"{side}_foot_hinge_cheek_0", (0.030, 0.014, 0.043), (0.515, foot_y + 0.036, 0.235), metal)
        _box(frame, f"{side}_foot_hinge_cheek_1", (0.030, 0.014, 0.043), (0.515, foot_y - 0.036, 0.235), metal)
        _box(frame, f"{side}_foot_hinge_bridge", (0.026, 0.086, 0.026), (0.487, foot_y, 0.224), metal)

    _tube_y(frame, "front_cross_tube", x=0.360, y=0.0, z=0.245, length=0.540, radius=0.015, material=metal)
    _tube_y(frame, "seat_cross_tube", x=0.050, y=0.0, z=0.455, length=0.465, radius=0.012, material=metal)
    _tube_y(frame, "rear_cross_tube", x=-0.240, y=0.0, z=0.360, length=0.520, radius=0.015, material=metal)
    _tube_y(frame, "axle_tube", x=-0.165, y=0.0, z=0.315, length=0.585, radius=0.018, material=metal)
    _tube_y(frame, "left_axle_pin", x=-0.165, y=0.316, z=0.315, length=0.060, radius=0.011, material=fastener)
    _tube_y(frame, "right_axle_pin", x=-0.165, y=-0.316, z=0.315, length=0.060, radius=0.011, material=fastener)


def _add_drive_wheel(part, *, side_sign: float, rim_mesh, tire_mesh, pushrim_mesh, metal, rubber, fastener):
    wheel_rotation = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    part.visual(tire_mesh, origin=wheel_rotation, material=rubber, name="tire")
    part.visual(rim_mesh, origin=wheel_rotation, material=metal, name="spoked_rim")
    _tube_y(part, "hub_barrel", x=0.0, y=0.0, z=0.0, length=0.070, radius=0.054, material=metal)

    part.visual(
        pushrim_mesh,
        origin=Origin(xyz=(0.0, side_sign * 0.064, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal,
        name="pushrim",
    )
    for i in range(6):
        angle = i * math.tau / 6.0
        x = 0.218 * math.cos(angle)
        z = 0.218 * math.sin(angle)
        _tube_y(
            part,
            f"pushrim_standoff_{i}",
            x=x,
            y=side_sign * 0.044,
            z=z,
            length=0.052,
            radius=0.0045,
            material=fastener,
        )


def _add_caster_fork(part, *, metal, fastener):
    _tube_z(part, "swivel_stem", x=0.0, y=0.0, z=0.020, length=0.125, radius=0.012, material=fastener)
    _box(part, "fork_crown", (0.080, 0.092, 0.027), (-0.020, 0.0, -0.052), metal)
    _box(part, "fork_arm_0", (0.035, 0.012, 0.145), (-0.045, 0.032, -0.137), metal)
    _box(part, "fork_arm_1", (0.035, 0.012, 0.145), (-0.045, -0.032, -0.137), metal)
    _tube_y(part, "wheel_axle", x=-0.045, y=0.0, z=-0.145, length=0.075, radius=0.006, material=fastener)
    _tube_y(part, "fork_axle_cap_0", x=-0.045, y=0.043, z=-0.145, length=0.022, radius=0.013, material=fastener)
    _tube_y(part, "fork_axle_cap_1", x=-0.045, y=-0.043, z=-0.145, length=0.022, radius=0.013, material=fastener)


def _add_caster_wheel(part, *, rim_mesh, tire_mesh, metal, rubber):
    wheel_rotation = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    part.visual(tire_mesh, origin=wheel_rotation, material=rubber, name="tire")
    part.visual(rim_mesh, origin=wheel_rotation, material=metal, name="molded_hub")


def _add_footrest(part, *, side_sign: float, plastic, fastener):
    _tube_y(part, "hinge_barrel", x=0.0, y=0.0, z=0.0, length=0.056, radius=0.014, material=fastener)
    _box(part, "hinge_lug", (0.038, 0.050, 0.030), (0.032, 0.0, -0.010), plastic)
    _tube_x(part, "support_tube", x=0.070, y=0.0, z=-0.035, length=0.150, radius=0.012, material=fastener)
    _box(part, "footplate", (0.190, 0.135, 0.018), (0.155, side_sign * 0.008, -0.056), plastic)
    _box(part, "raised_toe_lip", (0.020, 0.135, 0.028), (0.255, side_sign * 0.008, -0.043), plastic)
    _box(part, "snap_tab", (0.026, 0.042, 0.014), (0.035, 0.0, -0.026), plastic)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_wheelchair")

    metal = _mat("black_powder_coated_tube", (0.035, 0.038, 0.040, 1.0))
    fastener = _mat("zinc_plated_fasteners", (0.58, 0.58, 0.54, 1.0))
    rubber = _mat("matte_black_rubber", (0.010, 0.010, 0.010, 1.0))
    fabric = _mat("dark_nylon_sling", (0.030, 0.050, 0.080, 1.0))
    plastic = _mat("molded_gray_plastic", (0.34, 0.34, 0.33, 1.0))

    frame = model.part("frame")
    _add_frame(frame, metal, rubber, fabric, fastener)

    drive_rim, drive_tire, drive_pushrim = _drive_wheel_meshes()
    caster_rim, caster_tire = _caster_wheel_meshes()

    left_drive_wheel = model.part("left_drive_wheel")
    _add_drive_wheel(
        left_drive_wheel,
        side_sign=1.0,
        rim_mesh=drive_rim,
        tire_mesh=drive_tire,
        pushrim_mesh=drive_pushrim,
        metal=fastener,
        rubber=rubber,
        fastener=fastener,
    )
    right_drive_wheel = model.part("right_drive_wheel")
    _add_drive_wheel(
        right_drive_wheel,
        side_sign=-1.0,
        rim_mesh=drive_rim,
        tire_mesh=drive_tire,
        pushrim_mesh=drive_pushrim,
        metal=fastener,
        rubber=rubber,
        fastener=fastener,
    )

    for name, child, y, axis in (
        ("frame_to_left_drive_wheel", left_drive_wheel, 0.351, (0.0, 1.0, 0.0)),
        ("frame_to_right_drive_wheel", right_drive_wheel, -0.351, (0.0, 1.0, 0.0)),
    ):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=child,
            origin=Origin(xyz=(-0.165, y, 0.315)),
            axis=axis,
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        )

    for side, side_sign, y in (("left", 1.0, 0.24), ("right", -1.0, -0.24)):
        fork = model.part(f"{side}_caster_fork")
        _add_caster_fork(fork, metal=metal, fastener=fastener)
        wheel = model.part(f"{side}_caster_wheel")
        _add_caster_wheel(wheel, rim_mesh=caster_rim, tire_mesh=caster_tire, metal=fastener, rubber=rubber)

        model.articulation(
            f"frame_to_{side}_caster_fork",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.420, y, 0.190)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=5.0),
        )
        model.articulation(
            f"{side}_caster_fork_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-0.045, 0.0, -0.145)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=10.0),
        )

        footrest = model.part(f"{side}_footrest")
        _add_footrest(footrest, side_sign=side_sign, plastic=plastic, fastener=fastener)
        model.articulation(
            f"frame_to_{side}_footrest",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=footrest,
            origin=Origin(xyz=(0.515, 0.58 * y, 0.235)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_drive = object_model.get_part("left_drive_wheel")
    right_drive = object_model.get_part("right_drive_wheel")

    for side, fork in (("left", left_fork), ("right", right_fork)):
        ctx.allow_overlap(
            frame,
            fork,
            elem_a=f"{side}_caster_socket",
            elem_b="swivel_stem",
            reason="The caster kingpin is intentionally captured inside the simplified solid socket sleeve.",
        )
        ctx.allow_overlap(
            frame,
            fork,
            elem_a=f"{side}_caster_plate",
            elem_b="swivel_stem",
            reason="The caster kingpin passes through the bolted top plate before entering the socket.",
        )
        ctx.expect_within(
            fork,
            frame,
            axes="xy",
            inner_elem="swivel_stem",
            outer_elem=f"{side}_caster_socket",
            margin=0.004,
            name=f"{side} caster stem centered in socket",
        )
        ctx.expect_overlap(
            fork,
            frame,
            axes="z",
            elem_a="swivel_stem",
            elem_b=f"{side}_caster_socket",
            min_overlap=0.050,
            name=f"{side} caster stem retained vertically",
        )
        ctx.expect_overlap(
            fork,
            frame,
            axes="z",
            elem_a="swivel_stem",
            elem_b=f"{side}_caster_plate",
            min_overlap=0.010,
            name=f"{side} caster stem passes through plate",
        )

    for side, drive, pin in (
        ("left", left_drive, "left_axle_pin"),
        ("right", right_drive, "right_axle_pin"),
    ):
        ctx.allow_overlap(
            frame,
            drive,
            elem_a=pin,
            elem_b="hub_barrel",
            reason="The fixed axle pin is intentionally captured in the wheel hub bore.",
        )
        ctx.expect_overlap(
            frame,
            drive,
            axes="y",
            elem_a=pin,
            elem_b="hub_barrel",
            min_overlap=0.025,
            name=f"{side} drive axle retained in hub",
        )

    ctx.expect_contact(
        frame,
        left_drive,
        elem_a="left_axle_pin",
        elem_b="hub_barrel",
        contact_tol=0.003,
        name="left drive axle seats on hub",
    )
    ctx.expect_contact(
        frame,
        right_drive,
        elem_a="right_axle_pin",
        elem_b="hub_barrel",
        contact_tol=0.003,
        name="right drive axle seats on hub",
    )

    for side in ("left", "right"):
        footrest = object_model.get_part(f"{side}_footrest")
        ctx.allow_overlap(
            frame,
            footrest,
            elem_a=f"{side}_footrest_arm",
            elem_b="hinge_barrel",
            reason="The molded footrest hinge barrel rotates around a retained pin at the end of the support arm.",
        )
        ctx.expect_overlap(
            frame,
            footrest,
            axes="y",
            elem_a=f"{side}_footrest_arm",
            elem_b="hinge_barrel",
            min_overlap=0.020,
            name=f"{side} footrest barrel retained on hinge arm",
        )

    for side in ("left", "right"):
        fork = object_model.get_part(f"{side}_caster_fork")
        wheel = object_model.get_part(f"{side}_caster_wheel")
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="wheel_axle",
            elem_b="molded_hub",
            reason="The caster axle passes through the wheel hub bore in the simplified wheel proxy.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="molded_hub",
            min_overlap=0.025,
            name=f"{side} caster wheel captured on axle",
        )

    left_foot = object_model.get_part("left_footrest")
    left_joint = object_model.get_articulation("frame_to_left_footrest")
    rest_aabb = ctx.part_world_aabb(left_foot)
    with ctx.pose({left_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(left_foot)
    ctx.check(
        "left footrest folds upward",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] > rest_aabb[1][2] + 0.07,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
