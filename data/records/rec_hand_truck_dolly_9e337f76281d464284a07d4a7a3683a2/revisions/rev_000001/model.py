from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="package_delivery_hand_truck")

    blue_steel = Material("powder_coated_blue", rgba=(0.05, 0.20, 0.72, 1.0))
    dark_steel = Material("dark_axle_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    scuffed_plate = Material("scuffed_nose_plate", rgba=(0.58, 0.60, 0.56, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    wheel_gray = Material("cast_wheel_gray", rgba=(0.54, 0.56, 0.58, 1.0))

    frame = model.part("frame")

    main_frame_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.140, 0.020, 0.160),
                (-0.140, 0.012, 0.520),
                (-0.140, 0.000, 1.115),
                (-0.115, 0.000, 1.205),
                (0.000, 0.000, 1.245),
                (0.115, 0.000, 1.205),
                (0.140, 0.000, 1.115),
                (0.140, 0.012, 0.520),
                (0.140, 0.020, 0.160),
            ],
            radius=0.014,
            samples_per_segment=10,
            radial_segments=20,
            cap_ends=True,
        ),
        "main_frame_tube",
    )
    frame.visual(main_frame_mesh, material=blue_steel, name="main_frame_tube")

    # A thin steel nose plate and rear upturned lip make the load shelf read as
    # a real hand-truck toe plate instead of a generic block.
    frame.visual(
        Box((0.390, 0.315, 0.030)),
        origin=Origin(xyz=(0.0, -0.132, 0.030)),
        material=scuffed_plate,
        name="nose_plate",
    )
    frame.visual(
        Box((0.390, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, 0.020, 0.090)),
        material=blue_steel,
        name="rear_plate_lip",
    )

    # Fixed crossmembers and side feet tie the narrow uprights into the plate.
    frame.visual(
        Cylinder(radius=0.012, length=0.345),
        origin=Origin(xyz=(0.0, 0.014, 0.345), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_steel,
        name="lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.330),
        origin=Origin(xyz=(0.0, 0.004, 0.730), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_steel,
        name="middle_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.315),
        origin=Origin(xyz=(0.0, -0.002, 1.065), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_steel,
        name="upper_crossbar",
    )
    for i, x in enumerate((-0.140, 0.140)):
        frame.visual(
            Box((0.052, 0.048, 0.150)),
            origin=Origin(xyz=(x, 0.004, 0.120)),
            material=blue_steel,
            name=f"side_foot_{i}",
        )
        frame.visual(
            Box((0.048, 0.075, 0.062)),
            origin=Origin(xyz=(x, 0.052, 0.170)),
            material=blue_steel,
            name=f"axle_bracket_{i}",
        )

    # Lower axle and small top hinge pivots are fixed hardware on the frame.
    frame.visual(
        Cylinder(radius=0.0172, length=0.600),
        origin=Origin(xyz=(0.0, 0.080, 0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="lower_axle",
    )
    for i, x in enumerate((-0.140, 0.140)):
        frame.visual(
            Box((0.052, 0.046, 0.044)),
            origin=Origin(xyz=(x, -0.016, 1.160)),
            material=blue_steel,
            name=f"hinge_bracket_{i}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.070),
            origin=Origin(xyz=(x, -0.045, 1.160), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"hinge_pin_{i}",
        )
        for j, cap_x in enumerate((x - 0.031, x + 0.031)):
            frame.visual(
                Cylinder(radius=0.017, length=0.006),
                origin=Origin(xyz=(cap_x, -0.045, 1.160), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"hinge_pin_cap_{i}_{j}",
            )

    # One articulated part represents the folding upper handle loop.  At q=0 it
    # lies down against the frame uprights; at q=pi it swings above the frame for use.
    handle_loop = model.part("handle_loop")
    handle_loop_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.140, -0.018, -0.026),
                (-0.120, -0.018, -0.060),
                (-0.120, -0.018, -0.305),
                (-0.090, -0.018, -0.385),
                (0.000, -0.018, -0.415),
                (0.090, -0.018, -0.385),
                (0.120, -0.018, -0.305),
                (0.120, -0.018, -0.060),
                (0.140, -0.018, -0.026),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=20,
            cap_ends=True,
        ),
        "folding_handle_tube",
    )
    handle_loop.visual(handle_loop_mesh, material=blue_steel, name="loop_tube")

    collar_geom = LatheGeometry.from_shell_profiles(
        [(0.027, -0.020), (0.027, 0.020)],
        [(0.013, -0.020), (0.013, 0.020)],
        segments=32,
    )
    collar_geom.rotate_y(pi / 2.0)
    pivot_collar_mesh = mesh_from_geometry(collar_geom, "handle_pivot_collar")
    for i, x in enumerate((-0.140, 0.140)):
        handle_loop.visual(
            pivot_collar_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=blue_steel,
            name=f"pivot_collar_{i}",
        )
    handle_loop.visual(
        Cylinder(radius=0.018, length=0.178),
        origin=Origin(xyz=(0.0, -0.018, -0.410), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="rubber_grip",
    )

    model.articulation(
        "frame_to_handle_loop",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle_loop,
        origin=Origin(xyz=(0.0, -0.045, 1.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=pi),
    )

    wheel_geom = WheelGeometry(
        0.112,
        0.062,
        rim=WheelRim(inner_radius=0.074, flange_height=0.007, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.036,
            width=0.052,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.048, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.007, window_radius=0.014),
        bore=WheelBore(style="round", diameter=0.034),
    )
    wheel_mesh = mesh_from_geometry(wheel_geom, "gray_spoked_wheel")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.165,
            0.072,
            inner_radius=0.112,
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.56),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "black_utility_tire",
    )

    for i, x in enumerate((-0.245, 0.245)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        wheel.visual(wheel_mesh, material=wheel_gray, name="wheel_rim")
        model.articulation(
            f"frame_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, 0.080, 0.170)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    handle_loop = object_model.get_part("handle_loop")
    handle_joint = object_model.get_articulation("frame_to_handle_loop")

    for i in range(2):
        ctx.allow_overlap(
            frame,
            handle_loop,
            elem_a=f"hinge_bracket_{i}",
            elem_b=f"pivot_collar_{i}",
            reason=(
                "The simplified clevis block is locally embedded in the folding handle "
                "collar to depict a clipped, captured top hinge pivot."
            ),
        )
        ctx.expect_within(
            frame,
            handle_loop,
            axes="yz",
            inner_elem=f"hinge_pin_{i}",
            outer_elem=f"pivot_collar_{i}",
            margin=0.004,
            name=f"handle collar_{i} surrounds top hinge pin",
        )
        ctx.expect_overlap(
            frame,
            handle_loop,
            axes="x",
            elem_a=f"hinge_pin_{i}",
            elem_b=f"pivot_collar_{i}",
            min_overlap=0.035,
            name=f"handle collar_{i} is retained on hinge pin",
        )

    ctx.check(
        "folding handle has top hinge limits",
        handle_joint.motion_limits is not None
        and handle_joint.motion_limits.lower == 0.0
        and handle_joint.motion_limits.upper is not None
        and handle_joint.motion_limits.upper > 3.0,
        details=f"limits={handle_joint.motion_limits}",
    )
    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            frame,
            handle_loop,
            axis="y",
            positive_elem="main_frame_tube",
            negative_elem="loop_tube",
            min_gap=0.0,
            max_gap=0.040,
            name="folded handle lies close to uprights",
        )
        ctx.expect_overlap(
            handle_loop,
            frame,
            axes="xz",
            elem_a="loop_tube",
            elem_b="main_frame_tube",
            min_overlap=0.20,
            name="folded handle loop covers the upright span",
        )

    with ctx.pose({handle_joint: pi}):
        deployed_aabb = ctx.part_world_aabb(handle_loop)
        ctx.check(
            "handle loop deploys above the frame",
            deployed_aabb is not None and deployed_aabb[1][2] > 1.52,
            details=f"deployed_aabb={deployed_aabb}",
        )

    for i in range(2):
        wheel = object_model.get_part(f"wheel_{i}")
        spin = object_model.get_articulation(f"frame_to_wheel_{i}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="lower_axle",
            elem_b="wheel_rim",
            reason=(
                "The wheel hub is intentionally modeled as captured around the lower "
                "axle shaft so the transport wheel remains mounted while it spins."
            ),
        )
        ctx.check(
            f"wheel_{i} rotates continuously on lower axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="yz",
            inner_elem="lower_axle",
            outer_elem="wheel_rim",
            margin=0.010,
            name=f"wheel_{i} hub is centered on axle",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a="wheel_rim",
            elem_b="lower_axle",
            min_overlap=0.050,
            name=f"wheel_{i} hub overlaps axle span",
        )

    return ctx.report()


object_model = build_object_model()
