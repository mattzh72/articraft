from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
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
    model = ArticulatedObject(name="industrial_platform_cart")

    deck_mat = model.material("powder_coated_steel", rgba=(0.18, 0.20, 0.21, 1.0))
    frame_mat = model.material("dark_steel_frame", rgba=(0.04, 0.045, 0.05, 1.0))
    tread_mat = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    rim_mat = model.material("zinc_plated_hardware", rgba=(0.70, 0.68, 0.62, 1.0))
    grip_mat = model.material("molded_grip_rubber", rgba=(0.02, 0.025, 0.02, 1.0))
    handle_mat = model.material("safety_blue_handle", rgba=(0.02, 0.24, 0.60, 1.0))
    bumper_mat = model.material("scuffed_corner_rubber", rgba=(0.025, 0.025, 0.023, 1.0))

    # Cargo platform and welded under-frame.  Dimensions are for a medium-duty
    # warehouse platform cart: roughly 1.1 m x 0.64 m deck and low 200 mm deck
    # height over 150 mm casters.
    deck_frame = model.part("deck_frame")
    deck_frame.visual(
        Box((1.10, 0.64, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=deck_mat,
        name="cargo_deck",
    )
    deck_frame.visual(
        Box((1.04, 0.58, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        material=Material("slightly_worn_deck_top", rgba=(0.11, 0.12, 0.12, 1.0)),
        name="deck_top_sheet",
    )
    for idx, y in enumerate((-0.24, -0.16, -0.08, 0.0, 0.08, 0.16, 0.24)):
        deck_frame.visual(
            Box((0.94, 0.014, 0.007)),
            origin=Origin(xyz=(0.0, y, 0.2885)),
            material=frame_mat,
            name=f"deck_grip_rib_{idx}",
        )

    # Box-section perimeter rails and cross-members that make the platform read
    # as load-bearing rather than a plain slab.
    for idx, y in enumerate((-0.345, 0.345)):
        deck_frame.visual(
            Box((1.18, 0.050, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.1875)),
            material=frame_mat,
            name=f"side_rail_{idx}",
        )
    for idx, x in enumerate((-0.555, 0.555)):
        deck_frame.visual(
            Box((0.050, 0.74, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.1875)),
            material=frame_mat,
            name=f"end_rail_{idx}",
        )
    for idx, x in enumerate((-0.28, 0.0, 0.28)):
        deck_frame.visual(
            Box((0.060, 0.62, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.195)),
            material=frame_mat,
            name=f"cross_member_{idx}",
        )

    caster_positions = (
        (-0.43, -0.235),
        (-0.43, 0.235),
        (0.43, -0.235),
        (0.43, 0.235),
    )
    for idx, (x, y) in enumerate(caster_positions):
        deck_frame.visual(
            Box((0.18, 0.16, 0.030)),
            origin=Origin(xyz=(x, y, 0.200)),
            material=frame_mat,
            name=f"caster_plate_{idx}",
        )
        deck_frame.visual(
            Box((0.105, 0.085, 0.010)),
            origin=Origin(xyz=(x, y, 0.220)),
            material=rim_mat,
            name=f"caster_bolt_plate_{idx}",
        )
        for bolt_idx, (sx, sy) in enumerate(((-0.055, -0.040), (-0.055, 0.040), (0.055, -0.040), (0.055, 0.040))):
            deck_frame.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x + sx, y + sy, 0.228)),
                material=rim_mat,
                name=f"caster_bolt_{idx}_{bolt_idx}",
            )

    # Soft rubber bumpers at the four corners, tied into the perimeter frame.
    for idx, (x, y) in enumerate(((-0.585, -0.355), (-0.585, 0.355), (0.585, -0.355), (0.585, 0.355))):
        deck_frame.visual(
            Sphere(0.040),
            origin=Origin(xyz=(x, y, 0.212)),
            material=bumper_mat,
            name=f"corner_bumper_{idx}",
        )

    # Handle hinge support cheeks and gusseted feet welded to the rear rail.
    for idx, y in enumerate((-0.285, 0.285)):
        deck_frame.visual(
            Box((0.055, 0.014, 0.100)),
            origin=Origin(xyz=(-0.540, y, 0.335)),
            material=frame_mat,
            name=f"handle_cheek_{idx}",
        )
        deck_frame.visual(
            Box((0.095, 0.050, 0.012)),
            origin=Origin(xyz=(-0.540, y, 0.286)),
            material=frame_mat,
            name=f"handle_foot_{idx}",
        )
        deck_frame.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(xyz=(-0.540, y, 0.330), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rim_mat,
            name=f"hinge_boss_{idx}",
        )

    handle = model.part("push_handle")
    handle.visual(
        Cylinder(radius=0.022, length=0.506),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="hinge_tube",
    )
    for idx, y in enumerate((-0.240, 0.240)):
        handle.visual(
            Cylinder(radius=0.018, length=0.780),
            origin=Origin(xyz=(0.0, y, 0.390)),
            material=handle_mat,
            name=f"upright_tube_{idx}",
        )
        handle.visual(
            Sphere(0.020),
            origin=Origin(xyz=(0.0, y, 0.780)),
            material=handle_mat,
            name=f"top_bend_{idx}",
        )
        handle.visual(
            Sphere(0.022),
            origin=Origin(xyz=(0.0, -0.253 if y < 0.0 else 0.253, 0.0)),
            material=handle_mat,
            name=f"hinge_end_{idx}",
        )
    handle.visual(
        Cylinder(radius=0.019, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.780), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="top_crossbar",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 0.780), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_mat,
        name="rubber_grip",
    )
    for idx, y in enumerate((-0.185, 0.185)):
        handle.visual(
            Cylinder(radius=0.010, length=0.700),
            origin=Origin(xyz=(0.0, y, 0.350)),
            material=frame_mat,
            name=f"inner_reinforcement_{idx}",
        )

    model.articulation(
        "deck_to_handle",
        ArticulationType.REVOLUTE,
        parent=deck_frame,
        child=handle,
        origin=Origin(xyz=(-0.540, 0.0, 0.330)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.2, lower=0.0, upper=1.55),
        motion_properties=MotionProperties(damping=0.15, friction=0.08),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.040,
            rim=WheelRim(inner_radius=0.034, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "caster_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.048,
            inner_radius=0.054,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.006, radius=0.002),
        ),
        "caster_tire",
    )

    for idx, (x, y) in enumerate(caster_positions):
        fork = model.part(f"caster_fork_{idx}")
        fork.visual(
            Cylinder(radius=0.043, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=rim_mat,
            name="swivel_bearing",
        )
        fork.visual(
            Box((0.055, 0.092, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.011)),
            material=rim_mat,
            name="fork_bridge",
        )
        for side, y_arm in enumerate((-0.035, 0.035)):
            fork.visual(
                Box((0.045, 0.006, 0.105)),
                origin=Origin(xyz=(0.0, y_arm, -0.060)),
                material=rim_mat,
                name=f"fork_leg_{side}",
            )
            fork.visual(
                Cylinder(radius=0.010, length=0.008),
                origin=Origin(xyz=(0.0, y_arm, -0.105), rpy=(pi / 2.0, 0.0, 0.0)),
                material=frame_mat,
                name=f"axle_boss_{side}",
            )

        wheel = model.part(f"caster_wheel_{idx}")
        wheel.visual(
            tire_mesh,
            origin=Origin(),
            material=tread_mat,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(),
            material=rim_mat,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.008, length=0.062),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_mat,
            name="axle_shaft",
        )

        model.articulation(
            f"deck_to_fork_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck_frame,
            child=fork,
            origin=Origin(xyz=(x, y, 0.185)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=6.0),
            motion_properties=MotionProperties(damping=0.04, friction=0.03),
        )
        model.articulation(
            f"fork_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.105), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck_frame = object_model.get_part("deck_frame")
    handle = object_model.get_part("push_handle")
    handle_joint = object_model.get_articulation("deck_to_handle")

    wheel_parts = [object_model.get_part(f"caster_wheel_{idx}") for idx in range(4)]
    fork_parts = [object_model.get_part(f"caster_fork_{idx}") for idx in range(4)]
    swivel_joints = [object_model.get_articulation(f"deck_to_fork_{idx}") for idx in range(4)]
    wheel_joints = [object_model.get_articulation(f"fork_to_wheel_{idx}") for idx in range(4)]

    ctx.check("four swivel casters", len(wheel_parts) == 4 and len(fork_parts) == 4)
    ctx.check(
        "casters swivel and wheels roll continuously",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in swivel_joints + wheel_joints),
    )
    ctx.check(
        "folding handle has realistic hinge limits",
        handle_joint.articulation_type == ArticulationType.REVOLUTE
        and handle_joint.motion_limits is not None
        and handle_joint.motion_limits.lower == 0.0
        and handle_joint.motion_limits.upper is not None
        and 1.2 <= handle_joint.motion_limits.upper <= 1.7,
    )

    for idx, (fork, wheel) in enumerate(zip(fork_parts, wheel_parts)):
        ctx.expect_overlap(
            fork,
            wheel,
            axes="xy",
            min_overlap=0.020,
            name=f"caster {idx} wheel retained in fork plan",
        )
        ctx.expect_gap(
            deck_frame,
            wheel,
            axis="z",
            min_gap=0.0,
            max_gap=0.025,
            name=f"caster {idx} supports deck just above wheel",
        )

    with ctx.pose({"deck_to_handle": 0.0}):
        upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({"deck_to_handle": 1.55}):
        folded_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle folds forward toward the cargo deck",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][0] > upright_aabb[1][0] + 0.60
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.60,
        details=f"upright_aabb={upright_aabb}, folded_aabb={folded_aabb}",
    )

    with ctx.pose({"deck_to_fork_0": 0.0}):
        wheel_rest = ctx.part_world_aabb(wheel_parts[0])
    with ctx.pose({"deck_to_fork_0": pi / 2.0}):
        wheel_swiveled = ctx.part_world_aabb(wheel_parts[0])
    if wheel_rest is not None and wheel_swiveled is not None:
        rest_dx = wheel_rest[1][0] - wheel_rest[0][0]
        rest_dy = wheel_rest[1][1] - wheel_rest[0][1]
        swivel_dx = wheel_swiveled[1][0] - wheel_swiveled[0][0]
        swivel_dy = wheel_swiveled[1][1] - wheel_swiveled[0][1]
        caster_swaps_footprint = rest_dx > rest_dy * 2.0 and swivel_dy > swivel_dx * 2.0
    else:
        caster_swaps_footprint = False
    ctx.check(
        "caster fork swivels the wheel footprint",
        caster_swaps_footprint,
        details=f"rest={wheel_rest}, swiveled={wheel_swiveled}",
    )

    return ctx.report()


object_model = build_object_model()
