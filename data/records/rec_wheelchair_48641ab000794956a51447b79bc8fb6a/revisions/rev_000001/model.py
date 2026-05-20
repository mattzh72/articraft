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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_wheelchair")

    aluminum = model.material("powder_coated_aluminum", rgba=(0.34, 0.38, 0.38, 1.0))
    dark_aluminum = model.material("dark_anodized_tube", rgba=(0.08, 0.095, 0.10, 1.0))
    rubber = model.material("black_weather_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.76, 0.77, 0.72, 1.0))
    seal = model.material("matte_seal_gasket", rgba=(0.015, 0.018, 0.018, 1.0))
    cushion = model.material("sealed_blue_cushion", rgba=(0.03, 0.12, 0.22, 1.0))
    reflector = model.material("amber_reflector", rgba=(1.0, 0.50, 0.06, 1.0))

    def add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    frame = model.part("frame")

    # A sealed chair tub and rigid welded frame form one continuous root assembly.
    add_box(frame, "seat_shell", (0.56, 0.56, 0.052), (0.00, 0.00, 0.512), aluminum)
    add_box(frame, "seat_cushion", (0.49, 0.48, 0.045), (0.025, 0.00, 0.552), cushion)
    add_box(frame, "front_drip_lip", (0.060, 0.60, 0.055), (0.300, 0.00, 0.500), aluminum)
    add_box(frame, "side_drip_lip_0", (0.55, 0.045, 0.055), (0.01, 0.302, 0.500), aluminum)
    add_box(frame, "side_drip_lip_1", (0.55, 0.045, 0.055), (0.01, -0.302, 0.500), aluminum)

    add_box(frame, "back_panel", (0.052, 0.57, 0.52), (-0.290, 0.00, 0.780), aluminum)
    add_box(frame, "back_drip_cap", (0.085, 0.63, 0.038), (-0.300, 0.00, 1.055), aluminum)
    add_box(frame, "back_gasket", (0.012, 0.51, 0.42), (-0.258, 0.00, 0.790), seal)
    add_box(frame, "armrest_0", (0.55, 0.052, 0.058), (0.020, 0.315, 0.735), aluminum)
    add_box(frame, "armrest_1", (0.55, 0.052, 0.058), (0.020, -0.315, 0.735), aluminum)
    add_box(frame, "side_splash_0", (0.55, 0.035, 0.180), (-0.045, 0.320, 0.595), aluminum)
    add_box(frame, "side_splash_1", (0.55, 0.035, 0.180), (-0.045, -0.320, 0.595), aluminum)
    add_box(frame, "mudguard_0", (0.47, 0.034, 0.040), (-0.135, 0.335, 0.665), aluminum)
    add_box(frame, "mudguard_1", (0.47, 0.034, 0.040), (-0.135, -0.335, 0.665), aluminum)
    add_box(frame, "arm_stanchion_0_0", (0.040, 0.032, 0.090), (0.220, 0.315, 0.690), aluminum)
    add_box(frame, "arm_stanchion_0_1", (0.040, 0.032, 0.090), (-0.185, 0.315, 0.690), aluminum)
    add_box(frame, "arm_stanchion_1_0", (0.040, 0.032, 0.090), (0.220, -0.315, 0.690), aluminum)
    add_box(frame, "arm_stanchion_1_1", (0.040, 0.032, 0.090), (-0.185, -0.315, 0.690), aluminum)

    # Structural lower rails, axle tube, caster columns, and cross braces.
    add_cyl(frame, "rear_axle", 0.024, 0.800, (-0.150, 0.00, 0.315), stainless, rpy=(-math.pi / 2, 0.0, 0.0))
    add_box(frame, "lower_rail_0", (0.66, 0.038, 0.038), (0.040, 0.170, 0.315), dark_aluminum)
    add_box(frame, "lower_rail_1", (0.66, 0.038, 0.038), (0.040, -0.170, 0.315), dark_aluminum)
    add_box(frame, "rear_upright_0", (0.052, 0.052, 0.230), (-0.170, 0.235, 0.405), dark_aluminum)
    add_box(frame, "rear_upright_1", (0.052, 0.052, 0.230), (-0.170, -0.235, 0.405), dark_aluminum)
    add_box(frame, "front_upright_0", (0.050, 0.050, 0.330), (0.260, 0.220, 0.375), dark_aluminum)
    add_box(frame, "front_upright_1", (0.050, 0.050, 0.330), (0.260, -0.220, 0.375), dark_aluminum)
    add_box(frame, "front_cross_tube", (0.052, 0.480, 0.040), (0.285, 0.00, 0.315), dark_aluminum)
    brace_len = math.hypot(0.54, 0.46)
    brace_ang = math.atan2(0.46, 0.54)
    add_box(frame, "cross_brace_0", (brace_len, 0.026, 0.026), (0.055, 0.00, 0.415), dark_aluminum, rpy=(0.0, 0.0, brace_ang))
    add_box(frame, "cross_brace_1", (brace_len, 0.026, 0.026), (0.055, 0.00, 0.415), dark_aluminum, rpy=(0.0, 0.0, -brace_ang))
    add_cyl(frame, "cross_brace_pivot", 0.025, 0.030, (0.055, 0.00, 0.415), stainless)

    # Explicit weather-sealed mechanical nodes.
    for idx, side in enumerate((1.0, -1.0)):
        add_cyl(frame, f"axle_collar_{idx}", 0.046, 0.043, (-0.150, side * 0.300, 0.315), stainless, rpy=(-math.pi / 2, 0.0, 0.0))
        add_box(frame, f"back_hinge_plate_{idx}", (0.030, 0.020, 0.105), (-0.263, side * 0.287, 0.555), stainless)
        add_cyl(frame, f"back_hinge_bolt_{idx}", 0.014, 0.018, (-0.250, side * 0.300, 0.560), stainless, rpy=(-math.pi / 2, 0.0, 0.0))
        add_cyl(frame, f"caster_socket_{idx}", 0.036, 0.075, (0.350, side * 0.220, 0.247), seal)
        add_box(frame, f"caster_gusset_{idx}", (0.062, 0.070, 0.030), (0.287, side * 0.220, 0.275), stainless)
        add_box(frame, f"foot_hanger_{idx}", (0.050, 0.036, 0.340), (0.385, side * 0.130, 0.330), dark_aluminum)
        add_box(frame, f"hanger_bridge_{idx}", (0.120, 0.036, 0.030), (0.335, side * 0.130, 0.315), dark_aluminum)
        add_box(frame, f"foot_hinge_base_{idx}", (0.075, 0.190, 0.026), (0.430, side * 0.130, 0.160), stainless)
        add_box(frame, f"foot_hinge_lug_{idx}_0", (0.052, 0.014, 0.064), (0.430, side * 0.130 + 0.082, 0.205), stainless)
        add_box(frame, f"foot_hinge_lug_{idx}_1", (0.052, 0.014, 0.064), (0.430, side * 0.130 - 0.082, 0.205), stainless)
        add_cyl(frame, f"foot_hinge_pin_{idx}", 0.012, 0.190, (0.430, side * 0.130, 0.205), stainless, rpy=(-math.pi / 2, 0.0, 0.0))
        add_box(frame, f"amber_reflector_{idx}", (0.012, 0.070, 0.030), (-0.360, side * 0.318, 0.680), reflector)

    drive_wheel_geom = WheelGeometry(
        0.252,
        0.066,
        rim=WheelRim(inner_radius=0.178, flange_height=0.014, flange_thickness=0.006, bead_seat_depth=0.006),
        hub=WheelHub(
            radius=0.064,
            width=0.060,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.074, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=12, thickness=0.0045, window_radius=0.017),
        bore=WheelBore(style="round", diameter=0.026),
    )
    drive_tire_geom = TireGeometry(
        0.310,
        0.076,
        inner_radius=0.245,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.08),
        tread=TireTread(style="block", depth=0.008, count=28, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.004),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.010, radius=0.005),
    )

    for idx, side in enumerate((1.0, -1.0)):
        wheel = model.part(f"drive_wheel_{idx}")
        wheel.visual(mesh_from_geometry(drive_wheel_geom, f"drive_wheel_core_{idx}"), material=stainless, name="wheel_core")
        wheel.visual(mesh_from_geometry(drive_tire_geom, f"drive_tire_{idx}"), material=rubber, name="tire")
        push_rim = TorusGeometry(radius=0.254, tube=0.006, radial_segments=16, tubular_segments=72)
        push_rim.rotate_y(math.pi / 2)
        wheel.visual(mesh_from_geometry(push_rim, f"push_rim_{idx}"), origin=Origin(xyz=(0.056, 0.0, 0.0)), material=stainless, name="push_rim")
        add_cyl(wheel, "bearing_seal", 0.058, 0.020, (-0.028, 0.0, 0.0), seal, rpy=(0.0, math.pi / 2, 0.0))
        for spoke_i in range(8):
            a = spoke_i * math.tau / 8.0
            y = 0.254 * math.cos(a)
            z = 0.254 * math.sin(a)
            add_cyl(wheel, f"push_rim_standoff_{spoke_i}", 0.004, 0.108, (0.030, y, z), stainless, rpy=(0.0, math.pi / 2, 0.0))

        model.articulation(
            f"drive_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.150, side * 0.385, 0.315), rpy=(0.0, 0.0, side * math.pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=12.0),
        )

    caster_wheel_geom = WheelGeometry(
        0.064,
        0.034,
        rim=WheelRim(inner_radius=0.038, flange_height=0.005, flange_thickness=0.0025, bead_seat_depth=0.002),
        hub=WheelHub(radius=0.020, width=0.032, cap_style="flat"),
        face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.014),
    )
    caster_tire_geom = TireGeometry(
        0.088,
        0.044,
        inner_radius=0.064,
        tread=TireTread(style="ribbed", depth=0.004, count=18, land_ratio=0.62),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.005, radius=0.0025),
    )

    for idx, side in enumerate((1.0, -1.0)):
        yoke = model.part(f"caster_yoke_{idx}")
        add_cyl(yoke, "swivel_stem", 0.018, 0.130, (0.0, 0.0, 0.020), stainless)
        add_cyl(yoke, "rain_cap", 0.043, 0.024, (0.0, 0.0, -0.020), seal)
        add_box(yoke, "fork_crown", (0.082, 0.096, 0.026), (-0.048, 0.0, -0.045), stainless)
        add_box(yoke, "fork_leg_0", (0.030, 0.010, 0.116), (-0.048, 0.034, -0.103), stainless)
        add_box(yoke, "fork_leg_1", (0.030, 0.010, 0.116), (-0.048, -0.034, -0.103), stainless)
        add_cyl(yoke, "fork_axle", 0.007, 0.092, (-0.048, 0.0, -0.146), stainless, rpy=(-math.pi / 2, 0.0, 0.0))
        model.articulation(
            f"caster_swivel_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(0.350, side * 0.220, 0.230)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-math.pi, upper=math.pi),
        )

        caster = model.part(f"caster_wheel_{idx}")
        caster.visual(mesh_from_geometry(caster_wheel_geom, f"caster_wheel_core_{idx}"), material=stainless, name="wheel_core")
        caster.visual(mesh_from_geometry(caster_tire_geom, f"caster_tire_{idx}"), material=rubber, name="tire")
        add_cyl(caster, "bearing_seal", 0.022, 0.014, (-0.014, 0.0, 0.0), seal, rpy=(0.0, math.pi / 2, 0.0))
        model.articulation(
            f"caster_roll_{idx}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=caster,
            origin=Origin(xyz=(-0.048, 0.0, -0.146), rpy=(0.0, 0.0, math.pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    for idx, side in enumerate((1.0, -1.0)):
        foot = model.part(f"footrest_{idx}")
        add_cyl(foot, "hinge_barrel", 0.017, 0.132, (0.0, 0.0, 0.0), stainless, rpy=(-math.pi / 2, 0.0, 0.0))
        add_box(foot, "side_tab_0", (0.120, 0.015, 0.034), (0.060, 0.044, -0.017), stainless)
        add_box(foot, "side_tab_1", (0.120, 0.015, 0.034), (0.060, -0.044, -0.017), stainless)
        add_box(foot, "foot_plate", (0.225, 0.118, 0.019), (0.170, 0.0, -0.036), dark_aluminum)
        add_box(foot, "foot_plate_tread", (0.190, 0.085, 0.006), (0.182, 0.0, -0.023), rubber)
        model.articulation(
            f"footrest_fold_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=foot,
            origin=Origin(xyz=(0.430, side * 0.130, 0.205)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    for idx in range(2):
        wheel = object_model.get_part(f"drive_wheel_{idx}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="rear_axle",
            elem_b="wheel_core",
            reason="The stainless rear axle is intentionally captured inside the sealed drive-wheel hub bearing.",
        )
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="rear_axle",
            elem_b="bearing_seal",
            reason="The bearing seal is compressed around the weatherproof rear axle interface.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            min_overlap=0.025,
            elem_a="rear_axle",
            elem_b="wheel_core",
            name=f"drive wheel {idx} remains captured on rear axle",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="xz",
            min_overlap=0.025,
            elem_a="wheel_core",
            elem_b="rear_axle",
            name=f"drive wheel {idx} hub surrounds axle line",
        )

        yoke = object_model.get_part(f"caster_yoke_{idx}")
        caster = object_model.get_part(f"caster_wheel_{idx}")
        ctx.allow_overlap(
            frame,
            yoke,
            elem_a=f"caster_socket_{idx}",
            elem_b="swivel_stem",
            reason="The caster swivel stem is intentionally seated inside a sealed socket bearing.",
        )
        ctx.allow_overlap(
            frame,
            yoke,
            elem_a=f"caster_socket_{idx}",
            elem_b="rain_cap",
            reason="The rain cap locally compresses against the caster socket as a drip seal.",
        )
        ctx.expect_within(
            yoke,
            frame,
            axes="xy",
            inner_elem="swivel_stem",
            outer_elem=f"caster_socket_{idx}",
            margin=0.002,
            name=f"caster {idx} stem stays centered in sealed socket",
        )
        ctx.expect_overlap(
            yoke,
            frame,
            axes="z",
            min_overlap=0.070,
            elem_a="swivel_stem",
            elem_b=f"caster_socket_{idx}",
            name=f"caster {idx} stem has retained socket insertion",
        )
        ctx.allow_overlap(
            yoke,
            caster,
            elem_a="fork_axle",
            elem_b="wheel_core",
            reason="The caster wheel hub rotates around the through-axle captured by the fork cheeks.",
        )
        ctx.allow_overlap(
            yoke,
            caster,
            elem_a="fork_axle",
            elem_b="bearing_seal",
            reason="The small caster bearing seal is compressed around the through-axle.",
        )
        ctx.expect_overlap(
            yoke,
            caster,
            axes="y",
            min_overlap=0.030,
            elem_a="fork_axle",
            elem_b="wheel_core",
            name=f"caster {idx} wheel remains pinned in fork",
        )

        foot = object_model.get_part(f"footrest_{idx}")
        ctx.allow_overlap(
            frame,
            foot,
            elem_a=f"foot_hinge_pin_{idx}",
            elem_b="hinge_barrel",
            reason="The folding footrest barrel is intentionally captured by the stainless hinge pin.",
        )
        for pin_tab in ("side_tab_0", "side_tab_1"):
            ctx.allow_overlap(
                frame,
                foot,
                elem_a=f"foot_hinge_pin_{idx}",
                elem_b=pin_tab,
                reason="The hinge pin intentionally passes through each footrest side tab as a captured pivot leaf.",
            )
        ctx.expect_overlap(
            frame,
            foot,
            axes="y",
            min_overlap=0.10,
            elem_a=f"foot_hinge_pin_{idx}",
            elem_b="hinge_barrel",
            name=f"footrest {idx} hinge barrel spans the pin",
        )
        for pin_tab in ("side_tab_0", "side_tab_1"):
            ctx.expect_overlap(
                frame,
                foot,
                axes="xz",
                min_overlap=0.010,
                elem_a=f"foot_hinge_pin_{idx}",
                elem_b=pin_tab,
                name=f"footrest {idx} {pin_tab} is pinned on hinge axis",
            )

    caster_joint = object_model.get_articulation("caster_swivel_0")
    caster_wheel = object_model.get_part("caster_wheel_0")
    rest_pos = ctx.part_world_position(caster_wheel)
    with ctx.pose({caster_joint: 0.85}):
        swiveled_pos = ctx.part_world_position(caster_wheel)
    ctx.check(
        "front caster swivel has visible trail",
        rest_pos is not None
        and swiveled_pos is not None
        and abs(rest_pos[0] - swiveled_pos[0]) + abs(rest_pos[1] - swiveled_pos[1]) > 0.025,
        details=f"rest={rest_pos}, swiveled={swiveled_pos}",
    )

    foot_joint = object_model.get_articulation("footrest_fold_0")
    foot = object_model.get_part("footrest_0")
    rest_aabb = ctx.part_world_aabb(foot)
    with ctx.pose({foot_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(foot)
    ctx.check(
        "footrest folds upward for transfer clearance",
        rest_aabb is not None and folded_aabb is not None and folded_aabb[1][2] > rest_aabb[1][2] + 0.080,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
