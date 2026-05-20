from __future__ import annotations

from math import pi

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


def add_square_tube(part, *, name: str, outer: float, wall: float, length: float, center_z: float, material) -> None:
    """Build a visibly hollow square mast tube from four connected wall plates."""
    side_center = (outer - wall) / 2.0
    inner_span = outer - 2.0 * wall
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, side_center, center_z)),
        material=material,
        name=f"{name}_wall_0",
    )
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, -side_center, center_z)),
        material=material,
        name=f"{name}_wall_1",
    )
    part.visual(
        Box((wall, inner_span, length)),
        origin=Origin(xyz=(side_center, 0.0, center_z)),
        material=material,
        name=f"{name}_wall_2",
    )
    part.visual(
        Box((wall, inner_span, length)),
        origin=Origin(xyz=(-side_center, 0.0, center_z)),
        material=material,
        name=f"{name}_wall_3",
    )


def add_guide_pads(part, *, child_outer: float, parent_inner: float, z: float, name: str, material) -> None:
    """Add four glide pads that lightly contact the inside of the next-larger sleeve."""
    pad = (parent_inner - child_outer) / 2.0
    if pad <= 0.0:
        return
    face = child_outer / 2.0 + pad / 2.0
    part.visual(Box((0.060, pad, 0.120)), origin=Origin(xyz=(0.0, face, z)), material=material, name=f"{name}_pad_0")
    part.visual(Box((0.060, pad, 0.120)), origin=Origin(xyz=(0.0, -face, z)), material=material, name=f"{name}_pad_1")
    part.visual(Box((pad, 0.060, 0.120)), origin=Origin(xyz=(face, 0.0, z)), material=material, name=f"{name}_pad_2")
    part.visual(Box((pad, 0.060, 0.120)), origin=Origin(xyz=(-face, 0.0, z)), material=material, name=f"{name}_pad_3")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_security_camera_trailer_mast")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.035, 0.04, 0.045, 1.0))
    galvanized = model.material("galvanized", rgba=(0.58, 0.62, 0.60, 1.0))
    mast_aluminum = model.material("mast_aluminum", rgba=(0.78, 0.80, 0.76, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    camera_white = model.material("camera_white", rgba=(0.92, 0.93, 0.90, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.03, 0.045, 0.055, 1.0))
    amber = model.material("amber_lens", rgba=(1.0, 0.55, 0.06, 1.0))

    base = model.part("base")
    # Rectangular road trailer chassis and equipment enclosure.
    base.visual(Box((1.78, 0.74, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.26)), material=dark_steel, name="lower_frame")
    base.visual(Box((1.55, 0.62, 0.08)), origin=Origin(xyz=(0.05, 0.0, 0.35)), material=painted_steel, name="deck_plate")
    base.visual(Box((0.82, 0.50, 0.36)), origin=Origin(xyz=(-0.36, 0.0, 0.57)), material=painted_steel, name="equipment_box")
    base.visual(Box((0.34, 0.018, 0.035)), origin=Origin(xyz=(-0.62, 0.259, 0.71)), material=dark_steel, name="cabinet_hinge")
    base.visual(Box((0.09, 0.018, 0.055)), origin=Origin(xyz=(-0.10, 0.259, 0.57)), material=galvanized, name="cabinet_latch")

    # Drawbar, coupler, fenders, and stabilizer feet make the base read as a towable trailer.
    base.visual(Box((0.92, 0.08, 0.06)), origin=Origin(xyz=(-1.04, 0.0, 0.31)), material=dark_steel, name="tow_tongue")
    base.visual(Cylinder(radius=0.045, length=0.16), origin=Origin(xyz=(-1.52, 0.0, 0.31), rpy=(0.0, pi / 2.0, 0.0)), material=galvanized, name="coupler_socket")
    for y in (-0.50, 0.50):
        base.visual(Box((0.54, 0.10, 0.07)), origin=Origin(xyz=(-0.28, y, 0.58)), material=painted_steel, name=f"fender_{0 if y < 0 else 1}")
    for x, y, idx in ((0.67, 0.29, 0), (0.67, -0.29, 1), (-0.74, 0.29, 2), (-0.74, -0.29, 3)):
        base.visual(Box((0.06, 0.06, 0.30)), origin=Origin(xyz=(x, y, 0.16)), material=galvanized, name=f"jack_leg_{idx}")
        base.visual(Box((0.18, 0.14, 0.025)), origin=Origin(xyz=(x, y, 0.0125)), material=dark_steel, name=f"jack_pad_{idx}")

    # Detailed tire and rim meshes on a through axle.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.255,
            0.105,
            inner_radius=0.178,
            tread=TireTread(style="block", depth=0.010, count=20, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.009, radius=0.004),
        ),
        "road_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.178,
            0.086,
            rim=WheelRim(inner_radius=0.125, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.003),
            hub=WheelHub(
                radius=0.046,
                width=0.050,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.058, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.009, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.016),
            bore=WheelBore(style="round", diameter=0.026),
        ),
        "road_wheel",
    )
    base.visual(Cylinder(radius=0.032, length=1.07), origin=Origin(xyz=(-0.30, 0.0, 0.31), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="wheel_axle")
    for y, suffix in ((-0.48, "0"), (0.48, "1")):
        wheel_origin = Origin(xyz=(-0.30, y, 0.31), rpy=(0.0, 0.0, pi / 2.0))
        base.visual(tire_mesh, origin=wheel_origin, material=tire_rubber, name=f"tire_{suffix}")
        base.visual(wheel_mesh, origin=wheel_origin, material=rim_silver, name=f"rim_{suffix}")

    # Fixed receiver sleeve for the telescoping mast.
    socket_x = 0.48
    socket_bottom = 0.39
    socket_top = 1.35
    add_square_tube(base, name="receiver", outer=0.280, wall=0.040, length=socket_top - socket_bottom, center_z=(socket_top + socket_bottom) / 2.0, material=painted_steel)
    for v in base.visuals[-4:]:
        v.origin = Origin(xyz=(socket_x + v.origin.xyz[0], v.origin.xyz[1], v.origin.xyz[2]), rpy=v.origin.rpy)
    add_square_tube(base, name="receiver_collar", outer=0.340, wall=0.055, length=0.085, center_z=socket_top - 0.030, material=dark_steel)
    for v in base.visuals[-4:]:
        v.origin = Origin(xyz=(socket_x + v.origin.xyz[0], v.origin.xyz[1], v.origin.xyz[2]), rpy=v.origin.rpy)
    base.visual(Box((0.16, 0.055, 0.035)), origin=Origin(xyz=(socket_x, 0.195, 1.32)), material=dark_steel, name="receiver_clamp_block")
    base.visual(Cylinder(radius=0.025, length=0.12), origin=Origin(xyz=(socket_x, 0.282, 1.32), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="receiver_clamp_knob")
    base.visual(Box((0.26, 0.030, 0.020)), origin=Origin(xyz=(socket_x, 0.355, 1.32)), material=dark_steel, name="receiver_clamp_handle")

    # Four square telescoping mast sections.  Each part frame is at the seating plane
    # where it slides out of the section below; the tube extends downward for retained insertion.
    mast_0 = model.part("mast_0")
    add_square_tube(mast_0, name="tube", outer=0.180, wall=0.017, length=1.25, center_z=-0.325, material=mast_aluminum)
    add_square_tube(mast_0, name="collar", outer=0.220, wall=0.026, length=0.070, center_z=0.265, material=galvanized)
    add_guide_pads(mast_0, child_outer=0.180, parent_inner=0.200, z=-0.850, name="lower", material=dark_steel)
    mast_0.visual(Box((0.12, 0.045, 0.030)), origin=Origin(xyz=(0.0, 0.132, 0.25)), material=dark_steel, name="clamp_block")
    mast_0.visual(Cylinder(radius=0.018, length=0.085), origin=Origin(xyz=(0.0, 0.195, 0.25), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="clamp_knob")

    mast_1 = model.part("mast_1")
    add_square_tube(mast_1, name="tube", outer=0.130, wall=0.013, length=1.05, center_z=-0.275, material=mast_aluminum)
    add_square_tube(mast_1, name="collar", outer=0.165, wall=0.021, length=0.060, center_z=0.220, material=galvanized)
    add_guide_pads(mast_1, child_outer=0.130, parent_inner=0.146, z=-0.700, name="lower", material=dark_steel)
    mast_1.visual(Box((0.10, 0.037, 0.026)), origin=Origin(xyz=(0.0, 0.101, 0.205)), material=dark_steel, name="clamp_block")
    mast_1.visual(Cylinder(radius=0.015, length=0.070), origin=Origin(xyz=(0.0, 0.150, 0.205), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="clamp_knob")

    mast_2 = model.part("mast_2")
    add_square_tube(mast_2, name="tube", outer=0.098, wall=0.010, length=0.87, center_z=-0.215, material=mast_aluminum)
    add_square_tube(mast_2, name="collar", outer=0.126, wall=0.016, length=0.052, center_z=0.194, material=galvanized)
    add_guide_pads(mast_2, child_outer=0.098, parent_inner=0.104, z=-0.560, name="lower", material=dark_steel)
    mast_2.visual(Box((0.080, 0.030, 0.022)), origin=Origin(xyz=(0.0, 0.078, 0.182)), material=dark_steel, name="clamp_block")
    mast_2.visual(Cylinder(radius=0.012, length=0.058), origin=Origin(xyz=(0.0, 0.118, 0.182), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="clamp_knob")

    mast_3 = model.part("mast_3")
    add_square_tube(mast_3, name="tube", outer=0.064, wall=0.008, length=0.75, center_z=-0.175, material=mast_aluminum)
    add_square_tube(mast_3, name="top_cap", outer=0.090, wall=0.015, length=0.050, center_z=0.185, material=galvanized)
    add_guide_pads(mast_3, child_outer=0.064, parent_inner=0.078, z=-0.460, name="lower", material=dark_steel)
    mast_3.visual(Box((0.100, 0.100, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.209)), material=galvanized, name="bearing_plate")

    pan_head = model.part("pan_head")
    pan_head.visual(Cylinder(radius=0.082, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=dark_steel, name="pan_bearing")
    pan_head.visual(Box((0.18, 0.25, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.075)), material=camera_white, name="yoke_base")
    pan_head.visual(Box((0.075, 0.026, 0.320)), origin=Origin(xyz=(0.0, 0.105, 0.255)), material=camera_white, name="yoke_cheek_0")
    pan_head.visual(Box((0.075, 0.026, 0.320)), origin=Origin(xyz=(0.0, -0.105, 0.255)), material=camera_white, name="yoke_cheek_1")
    pan_head.visual(Cylinder(radius=0.035, length=0.008), origin=Origin(xyz=(0.0, 0.097, 0.280), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="tilt_bearing_0")
    pan_head.visual(Cylinder(radius=0.035, length=0.008), origin=Origin(xyz=(0.0, -0.097, 0.280), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="tilt_bearing_1")

    camera = model.part("camera")
    camera.visual(Box((0.26, 0.13, 0.13)), origin=Origin(xyz=(0.100, 0.0, 0.0)), material=camera_white, name="camera_body")
    camera.visual(Box((0.18, 0.09, 0.030)), origin=Origin(xyz=(0.020, 0.0, 0.080)), material=camera_white, name="sun_shield")
    camera.visual(Cylinder(radius=0.052, length=0.075), origin=Origin(xyz=(0.258, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=lens_glass, name="lens_barrel")
    camera.visual(Cylinder(radius=0.038, length=0.012), origin=Origin(xyz=(0.301, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=lens_glass, name="front_glass")
    camera.visual(Cylinder(radius=0.021, length=0.184), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=galvanized, name="tilt_trunnion")
    camera.visual(Box((0.030, 0.010, 0.030)), origin=Origin(xyz=(0.235, -0.068, 0.042)), material=amber, name="status_led")

    model.articulation(
        "base_to_mast_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_0,
        origin=Origin(xyz=(socket_x, 0.0, socket_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.25, lower=0.0, upper=0.75),
    )
    model.articulation(
        "mast_0_to_mast_1",
        ArticulationType.PRISMATIC,
        parent=mast_0,
        child=mast_1,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=0.65),
    )
    model.articulation(
        "mast_1_to_mast_2",
        ArticulationType.PRISMATIC,
        parent=mast_1,
        child=mast_2,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.22, lower=0.0, upper=0.55),
    )
    model.articulation(
        "mast_2_to_mast_3",
        ArticulationType.PRISMATIC,
        parent=mast_2,
        child=mast_3,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.20, lower=0.0, upper=0.45),
    )
    model.articulation(
        "mast_3_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=mast_3,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8),
    )
    model.articulation(
        "pan_head_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.80, upper=0.55),
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

    mast_pairs = (
        ("base_to_mast_0", "base", "mast_0"),
        ("mast_0_to_mast_1", "mast_0", "mast_1"),
        ("mast_1_to_mast_2", "mast_1", "mast_2"),
        ("mast_2_to_mast_3", "mast_2", "mast_3"),
    )
    for joint_name, parent_name, child_name in mast_pairs:
        joint = object_model.get_articulation(joint_name)
        parent = object_model.get_part(parent_name)
        child = object_model.get_part(child_name)
        ctx.check(f"{joint_name} is prismatic", joint.articulation_type == ArticulationType.PRISMATIC)
        ctx.expect_overlap(child, parent, axes="xy", min_overlap=0.050, name=f"{child_name} is coaxial with {parent_name}")
        upper = joint.motion_limits.upper or 0.0
        with ctx.pose({joint: upper}):
            ctx.expect_overlap(child, parent, axes="z", min_overlap=0.090, name=f"{child_name} remains inserted at full extension")

    pan = object_model.get_articulation("mast_3_to_pan_head")
    tilt = object_model.get_articulation("pan_head_to_camera")
    ctx.check("pan head has continuous bearing", pan.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("camera tilt is revolute", tilt.articulation_type == ArticulationType.REVOLUTE)

    rest_head_position = ctx.part_world_position(object_model.get_part("pan_head"))
    with ctx.pose(
        {
            object_model.get_articulation("base_to_mast_0"): 0.75,
            object_model.get_articulation("mast_0_to_mast_1"): 0.65,
            object_model.get_articulation("mast_1_to_mast_2"): 0.55,
            object_model.get_articulation("mast_2_to_mast_3"): 0.45,
        }
    ):
        extended_head_position = ctx.part_world_position(object_model.get_part("pan_head"))
    ctx.check(
        "four mast sliders raise the camera head",
        rest_head_position is not None
        and extended_head_position is not None
        and extended_head_position[2] > rest_head_position[2] + 2.30,
        details=f"rest={rest_head_position}, extended={extended_head_position}",
    )

    camera = object_model.get_part("camera")
    pan_head = object_model.get_part("pan_head")
    with ctx.pose({tilt: 0.45}):
        ctx.expect_within(camera, pan_head, axes="y", inner_elem="tilt_trunnion", outer_elem="yoke_base", margin=0.0, name="tilt trunnion stays between yoke cheeks")
    with ctx.pose({pan: pi / 2.0}):
        ctx.expect_gap(pan_head, object_model.get_part("mast_3"), axis="z", max_gap=0.001, max_penetration=0.0, positive_elem="pan_bearing", negative_elem="bearing_plate", name="pan bearing stays seated on mast cap")

    return ctx.report()


object_model = build_object_model()
