from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
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
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """Small CadQuery helper for padded/formed chair surfaces."""
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz)
    if radius > 0.0:
        shape = shape.edges().fillet(min(radius, sx * 0.45, sy * 0.45, sz * 0.45))
    return mesh_from_cadquery(shape, name, tolerance=0.0015, angular_tolerance=0.15)


def _arm_tube_mesh(name: str, side: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.00, 0.00, 0.115),
                (0.02, 0.00, 0.165),
                (0.10, -0.010 * side, 0.225),
                (0.21, -0.010 * side, 0.245),
            ],
            radius=0.014,
            samples_per_segment=16,
            radial_segments=18,
        ),
        name,
    )


def _back_frame_mesh(name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.020, -0.250, 0.080),
                (-0.025, -0.255, 0.390),
                (-0.015, -0.210, 0.620),
                (-0.020, 0.000, 0.670),
                (-0.015, 0.210, 0.620),
                (-0.025, 0.255, 0.390),
                (-0.020, 0.250, 0.080),
            ],
            radius=0.018,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name,
    )


def _seat_cushion_mesh():
    # Rounded, slightly broad seat pan with a real waterfall front edge.
    base = cq.Workplane("XY").box(0.58, 0.52, 0.078).edges().fillet(0.032)
    front_roll = (
        cq.Workplane("YZ")
        .circle(0.039)
        .extrude(0.48)
        .translate((0.285, 0.0, -0.018))
        .rotate((0.285, 0.0, -0.018), (0.285, 1.0, -0.018), 90)
    )
    cushion = base.union(front_roll)
    return mesh_from_cadquery(cushion, "seat_cushion", tolerance=0.0015, angular_tolerance=0.15)


def _wheel_meshes(prefix: str):
    tire = TireGeometry(
        0.030,
        0.028,
        inner_radius=0.020,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.62),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
        sidewall=TireSidewall(style="rounded", bulge=0.03),
        shoulder=TireShoulder(width=0.003, radius=0.0015),
    )
    rim = WheelGeometry(
        0.020,
        0.024,
        rim=WheelRim(inner_radius=0.013, flange_height=0.0022, flange_thickness=0.0014),
        hub=WheelHub(radius=0.0085, width=0.018, cap_style="domed"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0017, window_radius=0.0035),
        bore=WheelBore(style="round", diameter=0.006),
    )
    return mesh_from_geometry(tire, f"{prefix}_tire"), mesh_from_geometry(rim, f"{prefix}_rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ergonomic_office_chair")

    black_fabric = model.material("black_fabric", rgba=(0.035, 0.038, 0.042, 1.0))
    charcoal_mesh = model.material("charcoal_mesh", rgba=(0.08, 0.09, 0.10, 0.92))
    dark_plastic = model.material("dark_plastic", rgba=(0.02, 0.023, 0.027, 1.0))
    satin_plastic = model.material("satin_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    aluminum = model.material("cast_aluminum", rgba=(0.47, 0.49, 0.51, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(Cylinder(radius=0.105, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=aluminum, name="hub")
    base.visual(Cylinder(radius=0.045, length=0.220), origin=Origin(xyz=(0.0, 0.0, 0.190)), material=dark_plastic, name="outer_sleeve")
    base.visual(Cylinder(radius=0.060, length=0.022), origin=Origin(xyz=(0.0, 0.0, 0.304)), material=chrome, name="top_collar")

    for index in range(5):
        angle = (2.0 * pi * index / 5.0) + (pi / 2.0)
        c = cos(angle)
        s = sin(angle)

        spoke = model.part(f"spoke_{index}")
        spoke.visual(
            Box((0.355, 0.064, 0.035)),
            origin=Origin(xyz=(0.2825 * c, 0.2825 * s, 0.105), rpy=(0.0, 0.0, angle)),
            material=aluminum,
            name="spoke_beam",
        )
        spoke.visual(
            Box((0.100, 0.088, 0.040)),
            origin=Origin(xyz=(0.455 * c, 0.455 * s, 0.096), rpy=(0.0, 0.0, angle)),
            material=satin_plastic,
            name="caster_socket",
        )
        spoke.visual(
            Cylinder(radius=0.018, length=0.035),
            origin=Origin(xyz=(0.455 * c, 0.455 * s, 0.0915)),
            material=chrome,
            name="swivel_boss",
        )
        model.articulation(
            f"base_to_spoke_{index}",
            ArticulationType.FIXED,
            parent=base,
            child=spoke,
        )

        caster = model.part(f"caster_{index}")
        caster.visual(Cylinder(radius=0.010, length=0.018), origin=Origin(xyz=(0.0, 0.0, -0.009)), material=chrome, name="stem")
        caster.visual(Cylinder(radius=0.021, length=0.010), origin=Origin(xyz=(0.0, 0.0, -0.005)), material=dark_plastic, name="swivel_cap")
        caster.visual(Box((0.070, 0.034, 0.020)), origin=Origin(xyz=(0.0, 0.0, -0.007)), material=dark_plastic, name="fork_bridge")
        caster.visual(Box((0.006, 0.027, 0.062)), origin=Origin(xyz=(0.027, 0.0, -0.048)), material=dark_plastic, name="fork_cheek_0")
        caster.visual(Box((0.006, 0.027, 0.062)), origin=Origin(xyz=(-0.027, 0.0, -0.048)), material=dark_plastic, name="fork_cheek_1")
        caster.visual(
            Cylinder(radius=0.0045, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, -0.048), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name="axle",
        )
        model.articulation(
            f"spoke_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=spoke,
            child=caster,
            origin=Origin(xyz=(0.455 * c, 0.455 * s, 0.074), rpy=(0.0, 0.0, angle + pi / 2.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )

        wheel = model.part(f"wheel_{index}")
        tire_mesh, rim_mesh = _wheel_meshes(f"caster_{index}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(rim_mesh, material=aluminum, name="rim")
        model.articulation(
            f"caster_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.048)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=25.0),
        )

    lift_mast = model.part("lift_mast")
    lift_mast.visual(Cylinder(radius=0.024, length=0.250), origin=Origin(xyz=(0.0, 0.0, -0.005)), material=chrome, name="inner_piston")
    lift_mast.visual(Cylinder(radius=0.036, length=0.020), origin=Origin(xyz=(0.0, 0.0, 0.110)), material=dark_plastic, name="upper_bushing")
    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.22, lower=0.0, upper=0.100),
    )

    swivel_core = model.part("swivel_core")
    swivel_core.visual(Cylinder(radius=0.052, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=dark_plastic, name="bearing_housing")
    swivel_core.visual(Box((0.350, 0.285, 0.045)), origin=Origin(xyz=(0.030, 0.0, 0.060)), material=dark_plastic, name="mechanism_plate")
    swivel_core.visual(Box((0.230, 0.190, 0.028)), origin=Origin(xyz=(-0.035, 0.0, 0.092)), material=aluminum, name="tilt_carriage")
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=lift_mast,
        child=swivel_core,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0),
    )

    seat = model.part("seat")
    seat.visual(_seat_cushion_mesh(), origin=Origin(xyz=(0.075, 0.0, 0.055)), material=black_fabric, name="cushion")
    seat.visual(Box((0.640, 0.540, 0.028)), origin=Origin(xyz=(0.040, 0.0, 0.012)), material=satin_plastic, name="under_pan")
    seat.visual(Box((0.200, 0.180, 0.028)), origin=Origin(xyz=(-0.080, 0.0, 0.000)), material=aluminum, name="tilt_pivot_plate")
    seat.visual(Cylinder(radius=0.018, length=0.500), origin=Origin(xyz=(-0.275, 0.0, 0.060), rpy=(pi / 2.0, 0.0, 0.0)), material=chrome, name="rear_hinge_barrel")
    seat.visual(Box((0.050, 0.070, 0.040)), origin=Origin(xyz=(-0.275, 0.180, 0.036)), material=satin_plastic, name="rear_hinge_bracket_0")
    seat.visual(Box((0.050, 0.070, 0.040)), origin=Origin(xyz=(-0.275, -0.180, 0.036)), material=satin_plastic, name="rear_hinge_bracket_1")
    seat.visual(Box((0.050, 0.044, 0.034)), origin=Origin(xyz=(0.060, -0.245, -0.018)), material=satin_plastic, name="lever_pivot_socket")
    for side, label in ((1.0, "left"), (-1.0, "right")):
        seat.visual(
            Box((0.090, 0.044, 0.088)),
            origin=Origin(xyz=(0.030, side * 0.292, 0.040)),
            material=satin_plastic,
            name=f"{label}_arm_socket",
        )
    model.articulation(
        "seat_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_core,
        child=seat,
        origin=Origin(xyz=(-0.080, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.070, upper=0.160),
    )

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_box_mesh("back_mesh_panel", (0.018, 0.440, 0.560), 0.018),
        origin=Origin(xyz=(-0.015, 0.0, 0.365)),
        material=charcoal_mesh,
        name="mesh_panel",
    )
    backrest.visual(_back_frame_mesh("back_frame"), material=satin_plastic, name="outer_frame")
    backrest.visual(Box((0.020, 0.560, 0.590)), origin=Origin(xyz=(-0.025, 0.0, 0.375)), material=charcoal_mesh, name="support_web")
    backrest.visual(Box((0.038, 0.330, 0.105)), origin=Origin(xyz=(0.010, 0.0, 0.270)), material=black_fabric, name="lumbar_pad")
    backrest.visual(Cylinder(radius=0.017, length=0.220), origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(pi / 2.0, 0.0, 0.0)), material=chrome, name="hinge_knuckle")
    backrest.visual(Box((0.052, 0.130, 0.058)), origin=Origin(xyz=(-0.040, 0.0, 0.076)), material=satin_plastic, name="hinge_yoke")
    backrest.visual(Box((0.020, 0.090, 0.035)), origin=Origin(xyz=(-0.020, 0.0, 0.036)), material=satin_plastic, name="hinge_link")
    backrest.visual(Box((0.038, 0.140, 0.260)), origin=Origin(xyz=(-0.018, 0.0, 0.185)), material=satin_plastic, name="spine_plate")
    backrest.visual(Box((0.040, 0.530, 0.045)), origin=Origin(xyz=(-0.018, 0.0, 0.088)), material=satin_plastic, name="lower_bridge")
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.285, 0.0, 0.065), rpy=(0.0, -0.170, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.060, upper=0.380),
    )

    for side, label in ((1.0, "left"), (-1.0, "right")):
        armrest = model.part(f"{label}_armrest")
        armrest.visual(Box((0.034, 0.034, 0.255)), origin=Origin(xyz=(0.0, 0.0, 0.075)), material=chrome, name="post")
        armrest.visual(_arm_tube_mesh(f"{label}_arm_support", side), material=satin_plastic, name="support")
        armrest.visual(_rounded_box_mesh(f"{label}_arm_pad", (0.330, 0.086, 0.038), 0.020), origin=Origin(xyz=(0.165, -0.008 * side, 0.252)), material=black_fabric, name="pad")
        model.articulation(
            f"{label}_arm_height",
            ArticulationType.PRISMATIC,
            parent=seat,
            child=armrest,
            origin=Origin(xyz=(0.030, side * 0.292, 0.030)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.12, lower=0.0, upper=0.080),
        )

    height_lever = model.part("height_lever")
    height_lever.visual(Cylinder(radius=0.012, length=0.038), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=chrome, name="pivot_barrel")
    height_lever.visual(Cylinder(radius=0.006, length=0.200), origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="lever_rod")
    height_lever.visual(Box((0.070, 0.030, 0.014)), origin=Origin(xyz=(0.205, 0.0, -0.005)), material=dark_plastic, name="paddle")
    model.articulation(
        "height_lever_pull",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=height_lever,
        origin=Origin(xyz=(0.060, -0.245, -0.018), rpy=(0.0, 0.0, -0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-0.350, upper=0.250),
    )

    tension_knob = model.part("tension_knob")
    tension_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.040,
                body_style="lobed",
                top_diameter=0.066,
                crown_radius=0.002,
                grip=KnobGrip(style="ribbed", count=14, depth=0.002),
            ),
            "tilt_tension_knob",
        ),
        origin=Origin(rpy=(pi, 0.0, 0.0)),
        material=dark_plastic,
        name="knob",
    )
    tension_knob.visual(Cylinder(radius=0.008, length=0.120), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=chrome, name="threaded_stem")
    model.articulation(
        "tension_adjust",
        ArticulationType.CONTINUOUS,
        parent=seat,
        child=tension_knob,
        origin=Origin(xyz=(0.360, 0.0, -0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_mast = object_model.get_part("lift_mast")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    height_slide = object_model.get_articulation("height_slide")
    back_recline = object_model.get_articulation("back_recline")
    seat_tilt = object_model.get_articulation("seat_tilt")

    ctx.allow_overlap(
        base,
        lift_mast,
        elem_a="outer_sleeve",
        elem_b="inner_piston",
        reason="The chromed gas-lift piston is intentionally retained inside the lower sleeve.",
    )
    ctx.expect_within(
        lift_mast,
        base,
        axes="xy",
        inner_elem="inner_piston",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="piston centered in gas-lift sleeve",
    )
    ctx.expect_overlap(
        lift_mast,
        base,
        axes="z",
        elem_a="inner_piston",
        elem_b="outer_sleeve",
        min_overlap=0.120,
        name="gas-lift piston retains insertion",
    )
    ctx.allow_overlap(
        base,
        lift_mast,
        elem_a="top_collar",
        elem_b="inner_piston",
        reason="The top collar is a simplified bushing wrapped around the chromed piston.",
    )
    ctx.expect_within(
        lift_mast,
        base,
        axes="xy",
        inner_elem="inner_piston",
        outer_elem="top_collar",
        margin=0.004,
        name="piston centered through top collar",
    )

    ctx.allow_overlap(
        seat,
        backrest,
        elem_a="rear_hinge_barrel",
        elem_b="hinge_knuckle",
        reason="Backrest hinge knuckle wraps around the rear seat hinge barrel.",
    )
    ctx.expect_overlap(
        seat,
        backrest,
        axes="y",
        elem_a="rear_hinge_barrel",
        elem_b="hinge_knuckle",
        min_overlap=0.150,
        name="back hinge spans the seat hinge barrel",
    )

    for index in range(5):
        spoke = object_model.get_part(f"spoke_{index}")
        caster = object_model.get_part(f"caster_{index}")
        wheel = object_model.get_part(f"wheel_{index}")
        ctx.expect_contact(
            spoke,
            caster,
            elem_a="swivel_boss",
            elem_b="stem",
            contact_tol=0.003,
            name=f"caster {index} stem seats against swivel boss",
        )
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="Caster axle is intentionally captured through the wheel hub.",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.020,
            name=f"caster {index} axle passes through hub",
        )

    for label in ("left", "right"):
        armrest = object_model.get_part(f"{label}_armrest")
        socket = f"{label}_arm_socket"
        ctx.allow_overlap(
            seat,
            armrest,
            elem_a=socket,
            elem_b="post",
            reason="Armrest post telescopes inside its height-adjustment socket.",
        )
        ctx.expect_overlap(
            seat,
            armrest,
            axes="z",
            elem_a=socket,
            elem_b="post",
            min_overlap=0.045,
            name=f"{label} armrest post remains inserted",
        )
        rest_pos = ctx.part_world_position(armrest)
        with ctx.pose({object_model.get_articulation(f"{label}_arm_height"): 0.070}):
            raised_pos = ctx.part_world_position(armrest)
        ctx.check(
            f"{label} armrest height adjustment raises pad",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.060,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    height_lever = object_model.get_part("height_lever")
    ctx.allow_overlap(
        seat,
        height_lever,
        elem_a="lever_pivot_socket",
        elem_b="pivot_barrel",
        reason="The height lever pivot barrel is captured inside the under-seat socket.",
    )
    ctx.expect_overlap(
        seat,
        height_lever,
        axes="y",
        elem_a="lever_pivot_socket",
        elem_b="pivot_barrel",
        min_overlap=0.025,
        name="height lever pivot is captured in socket",
    )
    ctx.allow_overlap(
        seat,
        height_lever,
        elem_a="lever_pivot_socket",
        elem_b="lever_rod",
        reason="The lever rod emerges through the side of the pivot socket.",
    )
    ctx.expect_overlap(
        seat,
        height_lever,
        axes="x",
        elem_a="lever_pivot_socket",
        elem_b="lever_rod",
        min_overlap=0.020,
        name="height lever rod passes through pivot socket",
    )

    tension_knob = object_model.get_part("tension_knob")
    ctx.allow_overlap(
        seat,
        tension_knob,
        elem_a="under_pan",
        elem_b="threaded_stem",
        reason="The tilt-tension knob's threaded stem passes through the under-seat pan.",
    )
    ctx.expect_overlap(
        seat,
        tension_knob,
        axes="z",
        elem_a="under_pan",
        elem_b="threaded_stem",
        min_overlap=0.015,
        name="tension knob threaded stem engages under pan",
    )

    low_pos = ctx.part_world_position(seat)
    with ctx.pose({height_slide: 0.090}):
        high_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            lift_mast,
            base,
            axes="z",
            elem_a="inner_piston",
            elem_b="outer_sleeve",
            min_overlap=0.030,
            name="raised gas lift still retained in sleeve",
        )
    ctx.check(
        "gas lift raises the seated assembly",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.080,
        details=f"low={low_pos}, high={high_pos}",
    )

    rest_back = ctx.part_world_aabb(backrest)
    with ctx.pose({back_recline: 0.320}):
        reclined_back = ctx.part_world_aabb(backrest)
    ctx.check(
        "back recline moves top rearward",
        rest_back is not None
        and reclined_back is not None
        and reclined_back[0][0] < rest_back[0][0] - 0.060,
        details=f"rest={rest_back}, reclined={reclined_back}",
    )

    rest_seat_aabb = ctx.part_element_world_aabb(seat, elem="cushion")
    with ctx.pose({seat_tilt: 0.140}):
        tilted_seat_aabb = ctx.part_element_world_aabb(seat, elem="cushion")
    ctx.check(
        "seat tilt lifts the front edge",
        rest_seat_aabb is not None
        and tilted_seat_aabb is not None
        and tilted_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.015,
        details=f"rest={rest_seat_aabb}, tilted={tilted_seat_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
