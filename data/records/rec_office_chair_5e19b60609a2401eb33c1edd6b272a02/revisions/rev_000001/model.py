from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _rounded_panel_mesh(width: float, height: float, thickness: float, radius: float, name: str):
    """Rounded rectangle in XZ with its thickness along local Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        thickness,
        cap=True,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_task_chair_with_coat_hook")

    black = _mat(model, "satin_black_plastic", (0.015, 0.016, 0.018, 1.0))
    tire_black = _mat(model, "soft_black_rubber", (0.005, 0.005, 0.006, 1.0))
    graphite = _mat(model, "graphite_fabric", (0.10, 0.115, 0.13, 1.0))
    dark_frame = _mat(model, "dark_powder_coated_steel", (0.025, 0.027, 0.030, 1.0))
    satin_metal = _mat(model, "brushed_gas_lift_metal", (0.55, 0.57, 0.56, 1.0))
    rim_grey = _mat(model, "caster_grey_polymer", (0.24, 0.25, 0.25, 1.0))

    seat_cushion_mesh = mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.225, 0.100, 0.145, 0.405),
                (-0.125, 0.095, 0.160, 0.465),
                (0.075, 0.095, 0.165, 0.485),
                (0.220, 0.105, 0.145, 0.420),
            ],
            exponents=3.0,
            segments=64,
            cap=True,
        ),
        "contoured_seat_cushion",
    )
    back_panel_mesh = _rounded_panel_mesh(0.430, 0.540, 0.044, 0.075, "slim_back_pad")
    back_frame_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.225, -0.067, 0.032),
                (-0.230, -0.067, 0.360),
                (-0.195, -0.067, 0.535),
                (-0.080, -0.067, 0.590),
                (0.080, -0.067, 0.590),
                (0.195, -0.067, 0.535),
                (0.230, -0.067, 0.360),
                (0.225, -0.067, 0.032),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
        "back_perimeter_tube",
    )
    hook_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.005),
                (0.0, 0.0, -0.075),
                (0.0, -0.010, -0.128),
                (0.0, -0.036, -0.152),
                (0.0, -0.060, -0.132),
                (0.0, -0.054, -0.090),
            ],
            radius=0.007,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
        "folding_coat_hook",
    )
    caster_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.026,
            0.024,
            rim=WheelRim(inner_radius=0.017, flange_height=0.0025, flange_thickness=0.002),
            hub=WheelHub(radius=0.010, width=0.018, cap_style="domed"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.002, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.006),
        ),
        "caster_wheel_rim",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.038,
            0.030,
            inner_radius=0.026,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.045),
            tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.60),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_rubber_tire",
    )

    base = model.part("base")
    base.visual(Cylinder(radius=0.064, length=0.052), origin=Origin(xyz=(0, 0, 0.145)), material=satin_metal, name="center_hub")
    base.visual(Cylinder(radius=0.046, length=0.250), origin=Origin(xyz=(0, 0, 0.275)), material=satin_metal, name="column_sleeve")
    base.visual(Cylinder(radius=0.034, length=0.028), origin=Origin(xyz=(0, 0, 0.386)), material=black, name="column_collar")

    arm_length = 0.485
    arm_z = 0.145
    caster_radius = 0.475
    caster_angles = [math.radians(90.0 + 72.0 * i) for i in range(5)]
    for i, angle in enumerate(caster_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        yaw = angle
        base.visual(
            Box((arm_length, 0.054, 0.030)),
            origin=Origin(xyz=(0.5 * arm_length * c, 0.5 * arm_length * s, arm_z), rpy=(0, 0, yaw)),
            material=dark_frame,
            name=f"star_spoke_{i}",
        )
        base.visual(
            Box((0.090, 0.070, 0.026)),
            origin=Origin(xyz=(caster_radius * c, caster_radius * s, arm_z - 0.002), rpy=(0, 0, yaw)),
            material=black,
            name=f"caster_socket_{i}",
        )

    for i, angle in enumerate(caster_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        yoke = model.part(f"caster_{i}")
        yoke.visual(Cylinder(radius=0.011, length=0.034), origin=Origin(xyz=(0, 0, -0.017)), material=satin_metal, name="swivel_stem")
        yoke.visual(Cylinder(radius=0.023, length=0.014), origin=Origin(xyz=(0, 0, -0.041)), material=black, name="swivel_bearing")
        yoke.visual(Box((0.082, 0.026, 0.010)), origin=Origin(xyz=(0, 0, -0.043)), material=black, name="fork_bridge")
        yoke.visual(Box((0.006, 0.022, 0.073)), origin=Origin(xyz=(-0.027, 0, -0.082)), material=black, name="fork_leg_0")
        yoke.visual(Box((0.006, 0.022, 0.073)), origin=Origin(xyz=(0.027, 0, -0.082)), material=black, name="fork_leg_1")
        yoke.visual(Cylinder(radius=0.008, length=0.006), origin=Origin(xyz=(-0.033, 0, -0.090), rpy=(0, math.pi / 2.0, 0)), material=satin_metal, name="axle_cap_0")
        yoke.visual(Cylinder(radius=0.008, length=0.006), origin=Origin(xyz=(0.033, 0, -0.090), rpy=(0, math.pi / 2.0, 0)), material=satin_metal, name="axle_cap_1")
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=yoke,
            origin=Origin(xyz=(caster_radius * c, caster_radius * s, 0.130), rpy=(0, 0, angle - math.pi / 2.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=8.0),
        )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(caster_rim_mesh, origin=Origin(), material=rim_grey, name="rim")
        wheel.visual(caster_tire_mesh, origin=Origin(), material=tire_black, name="tire")
        wheel.visual(Cylinder(radius=0.006, length=0.048), origin=Origin(rpy=(0, math.pi / 2.0, 0)), material=satin_metal, name="axle_sleeve")
        model.articulation(
            f"caster_{i}_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.090)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    seat_frame = model.part("seat_frame")
    seat_frame.visual(Cylinder(radius=0.032, length=0.090), origin=Origin(xyz=(0, 0, 0.045)), material=satin_metal, name="inner_column")
    seat_frame.visual(Cylinder(radius=0.052, length=0.020), origin=Origin(xyz=(0, 0, 0.098)), material=black, name="swivel_cap")
    seat_frame.visual(Box((0.315, 0.270, 0.028)), origin=Origin(xyz=(0, 0, 0.086)), material=dark_frame, name="underseat_plate")
    seat_frame.visual(seat_cushion_mesh, origin=Origin(), material=graphite, name="seat_cushion")
    seat_frame.visual(Box((0.075, 0.190, 0.026)), origin=Origin(xyz=(-0.175, -0.155, 0.128), rpy=(0, 0, -0.24)), material=dark_frame, name="rear_hinge_arm_0")
    seat_frame.visual(Box((0.075, 0.190, 0.026)), origin=Origin(xyz=(0.175, -0.155, 0.128), rpy=(0, 0, 0.24)), material=dark_frame, name="rear_hinge_arm_1")
    seat_frame.visual(Box((0.040, 0.048, 0.072)), origin=Origin(xyz=(-0.225, -0.238, 0.154)), material=dark_frame, name="recline_lug_0")
    seat_frame.visual(Box((0.040, 0.048, 0.072)), origin=Origin(xyz=(0.225, -0.238, 0.154)), material=dark_frame, name="recline_lug_1")
    seat_frame.visual(Cylinder(radius=0.012, length=0.490), origin=Origin(xyz=(0, -0.255, 0.178), rpy=(0, math.pi / 2.0, 0)), material=satin_metal, name="recline_cross_pin")
    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=3.5),
    )

    backrest = model.part("backrest")
    backrest.visual(back_panel_mesh, origin=Origin(xyz=(0.0, -0.036, 0.300)), material=graphite, name="back_pad")
    backrest.visual(back_frame_mesh, origin=Origin(), material=dark_frame, name="rear_frame")
    backrest.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(xyz=(-0.160, 0.0, 0.000), rpy=(0, math.pi / 2.0, 0)),
        material=satin_metal,
        name="hinge_knuckle_0",
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(xyz=(0.160, 0.0, 0.000), rpy=(0, math.pi / 2.0, 0)),
        material=satin_metal,
        name="hinge_knuckle_1",
    )
    backrest.visual(
        mesh_from_geometry(
            tube_from_spline_points(
            [(-0.160, 0.0, 0.0225), (-0.200, -0.040, 0.040), (-0.225, -0.067, 0.065)],
            radius=0.010,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
            ),
            "back_lower_strut_0",
        ),
        material=dark_frame,
        name="lower_strut_0",
    )
    backrest.visual(
        mesh_from_geometry(
            tube_from_spline_points(
            [(0.160, 0.0, 0.0225), (0.200, -0.040, 0.040), (0.225, -0.067, 0.065)],
            radius=0.010,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
            ),
            "back_lower_strut_1",
        ),
        material=dark_frame,
        name="lower_strut_1",
    )
    backrest.visual(Box((0.120, 0.014, 0.052)), origin=Origin(xyz=(0.0, -0.064, 0.452)), material=black, name="hook_mount_plate")
    backrest.visual(Box((0.012, 0.028, 0.032)), origin=Origin(xyz=(-0.043, -0.084, 0.452)), material=black, name="hook_hinge_ear_0")
    backrest.visual(Box((0.012, 0.028, 0.032)), origin=Origin(xyz=(0.043, -0.084, 0.452)), material=black, name="hook_hinge_ear_1")
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.255, 0.178)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.2, lower=0.0, upper=0.38),
    )

    coat_hook = model.part("coat_hook")
    coat_hook.visual(Cylinder(radius=0.009, length=0.074), origin=Origin(rpy=(0, math.pi / 2.0, 0)), material=black, name="hinge_barrel")
    coat_hook.visual(hook_mesh, origin=Origin(), material=black, name="hook_curve")
    coat_hook.visual(Cylinder(radius=0.012, length=0.010), origin=Origin(xyz=(0.0, -0.057, -0.088), rpy=(math.pi / 2.0, 0, 0)), material=black, name="rounded_tip")
    model.articulation(
        "back_to_hook",
        ArticulationType.REVOLUTE,
        parent=backrest,
        child=coat_hook,
        origin=Origin(xyz=(0.0, -0.088, 0.452)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat_frame = object_model.get_part("seat_frame")
    backrest = object_model.get_part("backrest")
    coat_hook = object_model.get_part("coat_hook")
    swivel = object_model.get_articulation("base_to_seat")
    recline = object_model.get_articulation("seat_to_back")
    hook_hinge = object_model.get_articulation("back_to_hook")

    ctx.allow_overlap(
        backrest,
        seat_frame,
        elem_a="hinge_knuckle_0",
        elem_b="recline_cross_pin",
        reason="The transverse recline pin is intentionally captured inside the backrest hinge knuckle.",
    )
    ctx.allow_overlap(
        backrest,
        seat_frame,
        elem_a="hinge_knuckle_1",
        elem_b="recline_cross_pin",
        reason="The transverse recline pin is intentionally captured inside the backrest hinge knuckle.",
    )
    for i in range(5):
        ctx.allow_overlap(
            f"caster_{i}",
            f"wheel_{i}",
            elem_a="fork_leg_0",
            elem_b="axle_sleeve",
            reason="The wheel axle sleeve is intentionally captured in the caster fork for the spin joint.",
        )
        ctx.allow_overlap(
            f"caster_{i}",
            f"wheel_{i}",
            elem_a="fork_leg_1",
            elem_b="axle_sleeve",
            reason="The wheel axle sleeve is intentionally captured in the caster fork for the spin joint.",
        )

    ctx.expect_contact(
        seat_frame,
        base,
        elem_a="inner_column",
        elem_b="column_sleeve",
        contact_tol=0.001,
        name="seat column bears on central gas-lift sleeve",
    )
    ctx.expect_contact(
        backrest,
        seat_frame,
        elem_a="hinge_knuckle_0",
        elem_b="recline_cross_pin",
        contact_tol=0.002,
        name="backrest hinge knuckle sits on transverse recline pin",
    )
    ctx.expect_overlap(
        backrest,
        seat_frame,
        axes="x",
        elem_a="hinge_knuckle_1",
        elem_b="recline_cross_pin",
        min_overlap=0.060,
        name="recline hinge spans across the back frame",
    )
    ctx.expect_contact(
        coat_hook,
        backrest,
        elem_a="hinge_barrel",
        elem_b="hook_hinge_ear_0",
        contact_tol=0.018,
        name="folding hook is carried by rear back-frame hinge ears",
    )
    ctx.expect_within(
        "wheel_0",
        "caster_0",
        axes="x",
        inner_elem="tire",
        outer_elem="fork_bridge",
        margin=0.026,
        name="caster wheel fits between fork legs",
    )
    for i in range(5):
        ctx.expect_overlap(
            f"wheel_{i}",
            f"caster_{i}",
            axes="z",
            elem_a="axle_sleeve",
            elem_b="fork_leg_0",
            min_overlap=0.006,
            name=f"caster axle sleeve is vertically captured {i}",
        )

    expected = [
        "base_to_seat",
        "seat_to_back",
        "back_to_hook",
        *[f"base_to_caster_{i}" for i in range(5)],
        *[f"caster_{i}_to_wheel_{i}" for i in range(5)],
    ]
    ctx.check(
        "primary chair mechanisms are articulated",
        all(object_model.get_articulation(name) is not None for name in expected),
        details=f"expected joints={expected}",
    )

    rest_seat_pos = ctx.part_world_position(seat_frame)
    with ctx.pose({swivel: 1.0}):
        spun_seat_pos = ctx.part_world_position(seat_frame)
    ctx.check(
        "seat swivels about a fixed vertical column axis",
        rest_seat_pos is not None
        and spun_seat_pos is not None
        and abs(rest_seat_pos[0] - spun_seat_pos[0]) < 1e-6
        and abs(rest_seat_pos[1] - spun_seat_pos[1]) < 1e-6
        and abs(rest_seat_pos[2] - spun_seat_pos[2]) < 1e-6,
        details=f"rest={rest_seat_pos}, spun={spun_seat_pos}",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({recline: 0.38}):
        reclined_back_aabb = ctx.part_world_aabb(backrest)
    rest_back_y = None if rest_back_aabb is None else 0.5 * (rest_back_aabb[0][1] + rest_back_aabb[1][1])
    reclined_back_y = None if reclined_back_aabb is None else 0.5 * (reclined_back_aabb[0][1] + reclined_back_aabb[1][1])
    ctx.check(
        "positive recline moves the backrest rearward",
        rest_back_y is not None and reclined_back_y is not None and reclined_back_y < rest_back_y - 0.035,
        details=f"rest_y={rest_back_y}, reclined_y={reclined_back_y}",
    )

    rest_hook_aabb = ctx.part_world_aabb(coat_hook)
    with ctx.pose({hook_hinge: 1.10}):
        deployed_hook_aabb = ctx.part_world_aabb(coat_hook)
    rest_hook_y = None if rest_hook_aabb is None else rest_hook_aabb[0][1]
    deployed_hook_y = None if deployed_hook_aabb is None else deployed_hook_aabb[0][1]
    ctx.check(
        "hook hinge deploys the coat hook behind the back",
        rest_hook_y is not None and deployed_hook_y is not None and deployed_hook_y < rest_hook_y - 0.045,
        details=f"folded_min_y={rest_hook_y}, deployed_min_y={deployed_hook_y}",
    )

    return ctx.report()


object_model = build_object_model()
