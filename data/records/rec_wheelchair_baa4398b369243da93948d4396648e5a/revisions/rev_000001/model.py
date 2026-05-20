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
    PivotForkGeometry,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wheelchair")

    steel = model.material("powder_coated_steel", rgba=(0.08, 0.10, 0.11, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    tire_edge = model.material("worn_tread_edges", rgba=(0.04, 0.04, 0.035, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    zinc = model.material("zinc_plated_hardware", rgba=(0.58, 0.60, 0.58, 1.0))
    blue_vinyl = model.material("service_vinyl", rgba=(0.03, 0.12, 0.28, 1.0))
    black_grip = model.material("black_grip", rgba=(0.015, 0.015, 0.014, 1.0))

    def cylinder_between(part, name, p0, p1, radius, material):
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 0.0:
            return
        ux, uy, uz = dx / length, dy / length, dz / length
        yaw = math.atan2(uy, ux)
        pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
                rpy=(0.0, pitch, yaw),
            ),
            material=material,
            name=name,
        )

    def cyl_x(part, name, xyz, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_y(part, name, xyz, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    frame = model.part("frame")

    # Welded tubular chair base: wide enough for gloved service access and sized
    # like a real adult utility wheelchair.
    for side, x in (("left", -0.245), ("right", 0.245)):
        foot_x = x * 0.50
        caster_x = x * 1.16
        cylinder_between(frame, f"{side}_seat_rail", (x, -0.30, 0.535), (x, 0.355, 0.535), 0.021, steel)
        cylinder_between(frame, f"{side}_lower_rail", (x, -0.30, 0.225), (x, 0.430, 0.225), 0.019, steel)
        cylinder_between(frame, f"{side}_front_upright", (x, 0.360, 0.205), (x, 0.350, 0.545), 0.020, steel)
        cylinder_between(frame, f"{side}_rear_socket", (x, -0.300, 0.330), (x, -0.300, 0.575), 0.022, steel)
        cylinder_between(frame, f"{side}_diagonal_brace", (x, -0.265, 0.240), (x, 0.335, 0.535), 0.017, steel)
        cylinder_between(frame, f"{side}_caster_strut", (x, 0.355, 0.225), (caster_x, 0.608, 0.310), 0.018, steel)
        cylinder_between(frame, f"{side}_footrest_hanger", (x, 0.365, 0.245), (foot_x, 0.510, 0.135), 0.016, steel)
        cylinder_between(frame, f"{side}_footrest_stay", (x, 0.455, 0.315), (foot_x, 0.510, 0.135), 0.013, steel)

        # replaceable axle block and caster swivel sleeve
        frame.visual(
            Box((0.080, 0.070, 0.080)),
            origin=Origin(xyz=(x, -0.095, 0.325)),
            material=zinc,
            name=f"{side}_axle_block",
        )
        frame.visual(
            Cylinder(radius=0.030, length=0.070),
            origin=Origin(xyz=(caster_x, 0.640, 0.275)),
            material=zinc,
            name=f"{side}_caster_sleeve",
        )
        frame.visual(
            Box((0.090, 0.070, 0.010)),
            origin=Origin(xyz=(caster_x, 0.640, 0.240)),
            material=zinc,
            name=f"{side}_caster_washer",
        )

        # Footplate hinge ears stay on the fixed frame; the moving plate carries
        # a central barrel between them.
        frame.visual(
            Box((0.030, 0.030, 0.052)),
            origin=Origin(xyz=(foot_x - 0.073, 0.545, 0.155)),
            material=zinc,
            name=f"{side}_footrest_ear_0",
        )
        frame.visual(
            Box((0.030, 0.030, 0.052)),
            origin=Origin(xyz=(foot_x + 0.073, 0.545, 0.155)),
            material=zinc,
            name=f"{side}_footrest_ear_1",
        )
        frame.visual(
            Box((0.178, 0.046, 0.016)),
            origin=Origin(xyz=(foot_x, 0.526, 0.128)),
            material=zinc,
            name=f"{side}_footrest_bridge",
        )

    cyl_x(frame, "front_seat_crossmember", (0.0, 0.355, 0.535), 0.555, 0.020, steel)
    cyl_x(frame, "rear_seat_crossmember", (0.0, -0.300, 0.535), 0.555, 0.020, steel)
    cyl_x(frame, "front_lower_crossmember", (0.0, 0.425, 0.225), 0.555, 0.018, steel)
    cyl_x(frame, "rear_lower_crossmember", (0.0, -0.290, 0.225), 0.555, 0.018, steel)
    cyl_x(frame, "drive_axle_tube", (0.0, -0.095, 0.325), 0.780, 0.018, steel)
    cyl_x(frame, "back_hinge_pin", (0.0, -0.300, 0.575), 0.560, 0.018, zinc)
    cylinder_between(frame, "cross_brace_a", (-0.225, -0.265, 0.240), (0.225, 0.335, 0.515), 0.014, zinc)
    cylinder_between(frame, "cross_brace_b", (0.225, -0.265, 0.240), (-0.225, 0.335, 0.515), 0.014, zinc)
    frame.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.045, 0.382), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="cross_brace_pivot",
    )

    # Wear-surface panels overlap the tubes slightly, like bolted-on service
    # cushions and skid plates rather than floating upholstery.
    frame.visual(
        Box((0.500, 0.610, 0.050)),
        origin=Origin(xyz=(0.0, 0.025, 0.565)),
        material=blue_vinyl,
        name="replaceable_seat_pad",
    )
    frame.visual(
        Box((0.420, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.250, 0.599)),
        material=black_grip,
        name="tool_tray_insert",
    )
    frame.visual(
        Box((0.260, 0.220, 0.018)),
        origin=Origin(xyz=(0.0, -0.045, 0.532)),
        material=zinc,
        name="underseat_service_plate",
    )
    for i, (x, y) in enumerate(((-0.105, -0.125), (0.105, -0.125), (-0.105, 0.035), (0.105, 0.035))):
        frame.visual(
            Cylinder(radius=0.013, length=0.007),
            origin=Origin(xyz=(x, y, 0.545)),
            material=aluminum,
            name=f"service_plate_bolt_{i}",
        )

    # Folding service backrest with push handles and a visible hinge barrel.
    backrest = model.part("backrest")
    cyl_x(backrest, "lower_hinge_barrel", (0.0, 0.0, 0.0), 0.440, 0.020, zinc)
    for side, x in (("left", -0.215), ("right", 0.215)):
        cylinder_between(backrest, f"{side}_back_post", (x, -0.035, 0.035), (x, -0.055, 0.610), 0.020, steel)
        cylinder_between(backrest, f"{side}_push_handle_neck", (x, -0.055, 0.600), (x, -0.145, 0.690), 0.017, steel)
        cyl_y(backrest, f"{side}_push_grip", (x, -0.178, 0.710), 0.115, 0.025, black_grip)
        backrest.visual(
            Box((0.050, 0.012, 0.070)),
            origin=Origin(xyz=(x, -0.025, 0.015)),
            material=zinc,
            name=f"{side}_folding_latch",
        )
    backrest.visual(
        Box((0.455, 0.036, 0.360)),
        origin=Origin(xyz=(0.0, -0.040, 0.315)),
        material=blue_vinyl,
        name="replaceable_back_sling",
    )
    cyl_x(backrest, "upper_back_crossbar", (0.0, -0.065, 0.590), 0.465, 0.018, steel)
    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.300, 0.575)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.05),
    )

    drive_wheel_mesh = WheelGeometry(
        0.278,
        0.064,
        rim=WheelRim(inner_radius=0.205, flange_height=0.012, flange_thickness=0.005, bead_seat_depth=0.006),
        hub=WheelHub(
            radius=0.052,
            width=0.074,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.070, hole_diameter=0.007),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=8, thickness=0.008, window_radius=0.030),
        bore=WheelBore(style="round", diameter=0.030),
    )
    drive_tire_mesh = TireGeometry(
        0.335,
        0.072,
        inner_radius=0.268,
        carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.045),
        tread=TireTread(style="block", depth=0.010, count=28, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.010, radius=0.005),
    )
    handrim_mesh = TireGeometry(
        0.300,
        0.012,
        inner_radius=0.284,
        sidewall=TireSidewall(style="rounded", bulge=0.02),
        shoulder=TireShoulder(width=0.003, radius=0.002),
    )
    for side, x, outward in (("left", -0.390, -1.0), ("right", 0.390, 1.0)):
        wheel = model.part(f"{side}_drive_wheel")
        wheel.visual(mesh_from_geometry(drive_tire_mesh, f"{side}_drive_tire"), material=black_rubber, name="utility_tire")
        wheel.visual(mesh_from_geometry(drive_wheel_mesh, f"{side}_drive_rim"), material=aluminum, name="bolted_rim")
        wheel.visual(
            mesh_from_geometry(handrim_mesh, f"{side}_push_rim"),
            origin=Origin(xyz=(outward * 0.050, 0.0, 0.0)),
            material=zinc,
            name="push_rim",
        )
        for idx in range(8):
            ang = idx * math.tau / 8.0
            r0, r1 = 0.246, 0.287
            p0 = (0.0, r0 * math.cos(ang), r0 * math.sin(ang))
            p1 = (outward * 0.050, r1 * math.cos(ang), r1 * math.sin(ang))
            cylinder_between(wheel, f"push_rim_standoff_{idx}", p0, p1, 0.006, zinc)
        for idx in range(8):
            ang = (idx + 0.5) * math.tau / 8.0
            p0 = (0.0, 0.246 * math.cos(ang), 0.246 * math.sin(ang))
            p1 = (0.0, 0.286 * math.cos(ang), 0.286 * math.sin(ang))
            cylinder_between(wheel, f"rim_tire_bead_clamp_{idx}", p0, p1, 0.006, zinc)
        wheel.visual(
            Cylinder(radius=0.038, length=0.024),
            origin=Origin(xyz=(outward * 0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name="service_hub_cap",
        )
        model.articulation(
            f"frame_to_{side}_drive_wheel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, -0.095, 0.325)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=8.0),
        )

    caster_tire = TireGeometry(
        0.086,
        0.045,
        inner_radius=0.050,
        tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.55),
        sidewall=TireSidewall(style="square", bulge=0.018),
        shoulder=TireShoulder(width=0.006, radius=0.002),
    )
    caster_wheel = WheelGeometry(
        0.055,
        0.040,
        rim=WheelRim(inner_radius=0.034, flange_height=0.005, flange_thickness=0.003, bead_seat_depth=0.002),
        hub=WheelHub(radius=0.020, width=0.046, cap_style="flat"),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.004, window_radius=0.008),
        bore=WheelBore(style="round", diameter=0.012),
    )
    caster_fork_mesh = PivotForkGeometry(
        (0.092, 0.070, 0.160),
        gap_width=0.052,
        bore_diameter=0.014,
        bore_center_z=0.058,
        bridge_thickness=0.018,
        corner_radius=0.004,
        center=False,
    )
    for side, x in (("left", -0.245), ("right", 0.245)):
        fork = model.part(f"{side}_caster_fork")
        fork.visual(
            Box((0.014, 0.040, 0.170)),
            origin=Origin(xyz=(-0.039, 0.0, -0.110)),
            material=zinc,
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.014, 0.040, 0.170)),
            origin=Origin(xyz=(0.039, 0.0, -0.110)),
            material=zinc,
            name="fork_cheek_1",
        )
        fork.visual(
            Box((0.092, 0.046, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=zinc,
            name="fork_crown",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(-0.051, 0.0, -0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name="axle_boss_0",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(0.051, 0.0, -0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name="axle_boss_1",
        )
        fork.visual(
            Cylinder(radius=0.015, length=0.088),
            origin=Origin(xyz=(0.0, 0.0, 0.035)),
            material=zinc,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(radius=0.044, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=aluminum,
            name="bearing_collars",
        )
        fork.visual(
            Box((0.060, 0.040, 0.024)),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=zinc,
            name="stem_shoulder_block",
        )
        model.articulation(
            f"frame_to_{side}_caster_fork",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(x * 1.16, 0.640, 0.240)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=5.0),
        )

        small_wheel = model.part(f"{side}_caster_wheel")
        small_wheel.visual(mesh_from_geometry(caster_tire, f"{side}_caster_tire"), material=black_rubber, name="caster_tire")
        small_wheel.visual(mesh_from_geometry(caster_wheel, f"{side}_caster_hub"), material=aluminum, name="caster_hub")
        model.articulation(
            f"{side}_caster_fork_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=small_wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.120)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=10.0),
        )

    for side, x in (("left", -0.123), ("right", 0.123)):
        foot = model.part(f"{side}_footplate")
        cyl_x(foot, "hinge_barrel", (0.0, 0.0, 0.0), 0.118, 0.016, zinc)
        foot.visual(
            Box((0.090, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, 0.020, 0.000)),
            material=zinc,
            name="hinge_web",
        )
        foot.visual(
            Box((0.170, 0.225, 0.025)),
            origin=Origin(xyz=(0.0, 0.130, -0.002)),
            material=aluminum,
            name="replaceable_tread_plate",
        )
        for i, y in enumerate((0.063, 0.118, 0.173, 0.223)):
            foot.visual(
                Box((0.150, 0.010, 0.010)),
                origin=Origin(xyz=(0.0, y, 0.014)),
                material=tire_edge,
                name=f"anti_slip_rib_{i}",
            )
        foot.visual(
            Box((0.190, 0.030, 0.018)),
            origin=Origin(xyz=(0.0, 0.240, 0.000)),
            material=black_rubber,
            name="front_wear_bumper",
        )
        model.articulation(
            f"frame_to_{side}_footplate",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=foot,
            origin=Origin(xyz=(x, 0.545, 0.155)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    back_joint = object_model.get_articulation("frame_to_backrest")

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_hinge_pin",
        elem_b="lower_hinge_barrel",
        reason="The folding backrest barrel is intentionally represented as rotating around the fixed hinge pin.",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        elem_a="back_hinge_pin",
        elem_b="lower_hinge_barrel",
        axes="x",
        min_overlap=0.40,
        name="backrest hinge pin spans the folding barrel",
    )

    for side in ("left", "right"):
        drive = object_model.get_part(f"{side}_drive_wheel")
        caster_fork = object_model.get_part(f"{side}_caster_fork")
        caster_wheel = object_model.get_part(f"{side}_caster_wheel")
        footplate = object_model.get_part(f"{side}_footplate")

        ctx.allow_overlap(
            frame,
            drive,
            elem_a="drive_axle_tube",
            elem_b="bolted_rim",
            reason="The replaceable drive wheel is captured on the through-axle; the rim mesh is a solid proxy around the bore.",
        )
        ctx.expect_overlap(
            frame,
            drive,
            elem_a="drive_axle_tube",
            elem_b="bolted_rim",
            axes="x",
            min_overlap=0.02,
            name=f"{side} through-axle retains the drive rim",
        )
        ctx.allow_overlap(
            frame,
            caster_fork,
            elem_a=f"{side}_caster_sleeve",
            elem_b="swivel_stem",
            reason="The caster swivel stem is intentionally nested inside the replaceable sleeve bearing.",
        )
        ctx.allow_overlap(
            frame,
            caster_fork,
            elem_a=f"{side}_caster_washer",
            elem_b="swivel_stem",
            reason="The swivel stem passes through the serviceable bearing washer below the sleeve.",
        )
        ctx.expect_within(
            caster_fork,
            frame,
            axes="xy",
            inner_elem="swivel_stem",
            outer_elem=f"{side}_caster_sleeve",
            margin=0.002,
            name=f"{side} caster stem centered in sleeve",
        )
        ctx.expect_overlap(
            caster_fork,
            frame,
            elem_a="swivel_stem",
            elem_b=f"{side}_caster_sleeve",
            axes="z",
            min_overlap=0.060,
            name=f"{side} caster stem retained in sleeve",
        )
        ctx.expect_overlap(
            caster_fork,
            frame,
            elem_a="swivel_stem",
            elem_b=f"{side}_caster_washer",
            axes="z",
            min_overlap=0.006,
            name=f"{side} caster stem passes through thrust washer",
        )
        ctx.expect_overlap(
            drive,
            frame,
            axes="z",
            min_overlap=0.05,
            name=f"{side} drive wheel axle height crosses frame block",
        )
        ctx.expect_origin_distance(
            caster_fork,
            frame,
            axes="z",
            max_dist=0.30,
            name=f"{side} caster swivel mounted under front frame",
        )
        ctx.expect_overlap(
            caster_wheel,
            caster_fork,
            axes="z",
            min_overlap=0.02,
            name=f"{side} caster wheel retained in fork",
        )
        with ctx.pose({object_model.get_articulation(f"frame_to_{side}_footplate"): 1.20}):
            moved = ctx.part_world_aabb(footplate)
        ctx.check(
            f"{side} footplate folds above hinge",
            moved is not None and moved[1][2] > 0.30,
            details=f"folded_aabb={moved}",
        )

    with ctx.pose({back_joint: 0.85}):
        folded = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest service hinge folds rearward",
        folded is not None and folded[0][1] < -0.92,
        details=f"folded_aabb={folded}",
    )

    return ctx.report()


object_model = build_object_model()
