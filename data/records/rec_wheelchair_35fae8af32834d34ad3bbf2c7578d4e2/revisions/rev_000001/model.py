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
    MotionProperties,
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


def _tube_between(part, name: str, p0, p1, radius: float, material: Material) -> None:
    """Add a cylindrical tube between two points.

    The authored chair frame is intentionally tube-built; this helper covers the
    axis-aligned and side-frame diagonal members used below.
    """

    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return

    if abs(dx) < 1e-8 and abs(dz) < 1e-8:
        rpy = (-math.pi / 2.0 if dy >= 0.0 else math.pi / 2.0, 0.0, 0.0)
    elif abs(dy) < 1e-8:
        rpy = (0.0, math.atan2(dx, dz), 0.0)
    elif abs(dx) < 1e-8:
        rpy = (-math.atan2(dy, dz), 0.0, 0.0)
    else:
        # General XY diagonal brace: model as a short rectangular bar instead.
        yaw = math.atan2(dy, dx)
        part.visual(
            Box((length, radius * 2.2, radius * 1.6)),
            origin=Origin(
                xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
                rpy=(0.0, 0.0, yaw),
            ),
            material=material,
            name=name,
        )
        return

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            rpy=rpy,
        ),
        material=material,
        name=name,
    )


def _box_between_xy(part, name: str, p0, p1, width: float, thickness: float, material: Material) -> None:
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    length = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _bolt_head(part, name: str, xyz, material: Material, *, axis: str = "y", radius: float = 0.012) -> None:
    if axis == "y":
        rpy = (-math.pi / 2.0, 0.0, 0.0)
    elif axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=0.007),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wheelchair")

    frame_paint = model.material("textured_olive_powder_coat", rgba=(0.18, 0.24, 0.18, 1.0))
    black_molded = model.material("black_molded_rubber", rgba=(0.02, 0.022, 0.020, 1.0))
    dark_fabric = model.material("charcoal_reinforced_vinyl", rgba=(0.045, 0.050, 0.055, 1.0))
    bare_metal = model.material("brushed_service_metal", rgba=(0.58, 0.58, 0.54, 1.0))
    fastener_black = model.material("black_oxide_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))
    safety_yellow = model.material("yellow_safety_release_tabs", rgba=(0.95, 0.72, 0.08, 1.0))

    # The fixed root is a practical welded tube frame with plates, collars and
    # serviceable fasteners.  X is forward, Y is side-to-side, Z is up.
    frame = model.part("frame")
    tube_r = 0.018
    side_y = 0.245

    for idx, side in enumerate((-1.0, 1.0)):
        y = side * side_y
        _tube_between(frame, f"seat_rail_{idx}", (-0.34, y, 0.515), (0.36, y, 0.515), tube_r, frame_paint)
        _tube_between(frame, f"lower_rail_{idx}", (-0.39, y, 0.335), (0.50, y, 0.235), tube_r, frame_paint)
        _tube_between(frame, f"front_upright_{idx}", (0.36, y, 0.515), (0.50, y, 0.235), tube_r, frame_paint)
        _tube_between(frame, f"rear_backstay_{idx}", (-0.34, y, 0.515), (-0.46, y, 1.105), tube_r, frame_paint)
        _tube_between(frame, f"rear_axle_drop_{idx}", (-0.34, y, 0.515), (-0.39, y, 0.335), tube_r, frame_paint)
        _tube_between(frame, f"arm_post_front_{idx}", (0.24, y, 0.535), (0.24, y, 0.735), 0.014, frame_paint)
        _tube_between(frame, f"arm_post_rear_{idx}", (-0.22, y, 0.535), (-0.22, y, 0.735), 0.014, frame_paint)
        _tube_between(frame, f"arm_pad_spine_{idx}", (-0.26, y, 0.735), (0.30, y, 0.735), 0.012, frame_paint)
        frame.visual(
            Box((0.52, 0.070, 0.035)),
            origin=Origin(xyz=(0.02, y, 0.755)),
            material=black_molded,
            name=f"rugged_arm_pad_{idx}",
        )
        _tube_between(frame, f"push_handle_tube_{idx}", (-0.45, y, 1.100), (-0.63, y, 1.130), 0.014, frame_paint)
        _tube_between(frame, f"push_grip_{idx}", (-0.60, y, 1.125), (-0.72, y, 1.145), 0.020, black_molded)
        _tube_between(frame, f"anti_tip_strut_{idx}", (-0.39, y, 0.335), (-0.66, y, 0.175), 0.014, frame_paint)
        frame.visual(
            Cylinder(radius=0.040, length=0.032),
            origin=Origin(xyz=(-0.672, y, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_molded,
            name=f"anti_tip_roller_{idx}",
        )
        frame.visual(
            Box((0.095, 0.040, 0.090)),
            origin=Origin(xyz=(-0.37, side * 0.265, 0.335)),
            material=bare_metal,
            name=f"rear_axle_plate_{idx}",
        )
        _bolt_head(frame, f"axle_bolt_upper_{idx}", (-0.39, side * 0.286, 0.365), fastener_black, axis="y", radius=0.010)
        _bolt_head(frame, f"axle_bolt_lower_{idx}", (-0.35, side * 0.286, 0.305), fastener_black, axis="y", radius=0.010)
        _tube_between(frame, f"rear_axle_boss_{idx}", (-0.37, side * 0.285, 0.335), (-0.37, side * 0.350, 0.335), 0.024, bare_metal)
        _tube_between(frame, f"rear_axle_stub_{idx}", (-0.37, side * 0.345, 0.335), (-0.37, side * 0.425, 0.335), 0.012, bare_metal)

        # Front caster bearing blocks and the footrest hinge receiver barrels.
        frame.visual(
            Box((0.075, 0.080, 0.060)),
            origin=Origin(xyz=(0.550, side * 0.245, 0.265)),
            material=bare_metal,
            name=f"caster_bearing_block_{idx}",
        )
        _tube_between(frame, f"caster_head_tube_{idx}", (0.550, y, 0.250), (0.550, y, 0.375), 0.024, frame_paint)
        _bolt_head(frame, f"caster_clamp_bolt_{idx}", (0.577, side * 0.285, 0.285), fastener_black, axis="y", radius=0.008)
        _tube_between(frame, f"footrest_upper_bridge_{idx}", (0.390, y, 0.505), (0.390, side * 0.145, 0.505), 0.012, frame_paint)
        _tube_between(frame, f"footrest_lower_bridge_{idx}", (0.390, y, 0.410), (0.390, side * 0.145, 0.410), 0.012, frame_paint)
        _tube_between(frame, f"footrest_socket_{idx}", (0.390, side * 0.145, 0.410), (0.390, side * 0.145, 0.505), 0.020, bare_metal)
        frame.visual(
            Box((0.030, 0.040, 0.018)),
            origin=Origin(xyz=(0.420, side * 0.145, 0.495)),
            material=safety_yellow,
            name=f"footrest_release_tab_{idx}",
        )

    _tube_between(frame, "rear_axle_cross_tube", (-0.39, -side_y, 0.335), (-0.39, side_y, 0.335), tube_r, frame_paint)
    _tube_between(frame, "front_cross_tube", (0.47, -side_y, 0.245), (0.47, side_y, 0.245), tube_r, frame_paint)
    _tube_between(frame, "front_seat_cross_tube", (0.29, -side_y, 0.515), (0.29, side_y, 0.515), tube_r, frame_paint)
    _tube_between(frame, "rear_seat_cross_tube", (-0.28, -side_y, 0.515), (-0.28, side_y, 0.515), tube_r, frame_paint)
    _tube_between(frame, "back_top_cross_tube", (-0.46, -side_y, 1.105), (-0.46, side_y, 1.105), 0.016, frame_paint)
    _tube_between(frame, "back_mid_cross_tube", (-0.41, -side_y, 0.795), (-0.41, side_y, 0.795), 0.014, frame_paint)

    # Sling seat and back panels are thin supported surfaces, not solid blocks.
    frame.visual(
        Box((0.62, 0.500, 0.035)),
        origin=Origin(xyz=(0.005, 0.0, 0.535)),
        material=dark_fabric,
        name="reinforced_sling_seat",
    )
    frame.visual(
        Box((0.055, 0.500, 0.470)),
        origin=Origin(xyz=(-0.435, 0.0, 0.835), rpy=(0.0, -0.15, 0.0)),
        material=dark_fabric,
        name="reinforced_back_panel",
    )
    for i, z in enumerate((0.705, 0.815, 0.925)):
        frame.visual(
            Box((0.030, 0.535, 0.026)),
            origin=Origin(xyz=(-0.445, 0.0, z), rpy=(0.0, -0.15, 0.0)),
            material=black_molded,
            name=f"back_webbing_band_{i}",
        )
    for i, x in enumerate((-0.23, 0.04, 0.27)):
        frame.visual(
            Box((0.030, 0.535, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.550)),
            material=black_molded,
            name=f"seat_rib_{i}",
        )

    # Foldable cross-bracing under the sling gives the utility frame a rugged,
    # serviceable read while remaining one welded/fixed assembly here.
    _box_between_xy(frame, "underseat_xbrace_a", (-0.31, -0.245, 0.490), (0.31, 0.245, 0.490), 0.036, 0.016, bare_metal)
    _box_between_xy(frame, "underseat_xbrace_b", (-0.31, 0.245, 0.490), (0.31, -0.245, 0.490), 0.036, 0.016, bare_metal)
    frame.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        material=fastener_black,
        name="xbrace_center_pivot_bolt",
    )

    # Large rear drive wheels with block tires, thick rims, hub details and
    # separate push rims.
    rear_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.255,
            0.055,
            rim=WheelRim(inner_radius=0.185, flange_height=0.012, flange_thickness=0.006, bead_seat_depth=0.006),
            hub=WheelHub(
                radius=0.052,
                width=0.050,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.070, hole_diameter=0.008),
            ),
            face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.007, window_radius=0.018),
            bore=WheelBore(style="round", diameter=0.025),
        ),
        "rear_spoked_rim",
    )
    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.330,
            0.068,
            inner_radius=0.255,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.08),
            tread=TireTread(style="block", depth=0.012, count=28, land_ratio=0.54),
            grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "rear_block_tire",
    )
    push_rim_mesh = mesh_from_geometry(
        TireGeometry(
            0.285,
            0.014,
            inner_radius=0.272,
            sidewall=TireSidewall(style="rounded", bulge=0.02),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "knurled_push_rim",
    )

    rear_wheels = []
    for idx, side in enumerate((-1.0, 1.0)):
        wheel = model.part(f"rear_wheel_{idx}")
        rear_wheels.append(wheel)
        wheel.visual(rear_tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=black_molded, name="tire")
        wheel.visual(rear_wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=bare_metal, name="spoked_rim")
        wheel.visual(
            push_rim_mesh,
            origin=Origin(xyz=(0.0, side * 0.052, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=bare_metal,
            name="hand_push_rim",
        )
        wheel.visual(
            Cylinder(radius=0.038, length=0.016),
            origin=Origin(xyz=(0.0, side * 0.032, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=fastener_black,
            name="outer_hub_cap",
        )
        for j, ang in enumerate((math.radians(35.0), math.radians(145.0), math.radians(215.0), math.radians(325.0))):
            x = 0.285 * math.cos(ang)
            z = 0.285 * math.sin(ang)
            _tube_between(
                wheel,
                f"push_rim_standoff_{j}",
                (x, side * 0.022, z),
                (x, side * 0.052, z),
                0.004,
                bare_metal,
            )
        model.articulation(
            f"frame_to_rear_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.370, side * 0.395, 0.335)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=12.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.02),
        )

    # Front caster assemblies: swiveling forks carrying independently rolling
    # small utility wheels.
    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.058,
            0.034,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.003, bead_seat_depth=0.002),
            hub=WheelHub(radius=0.018, width=0.028, cap_style="flat"),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "caster_rim",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.087,
            0.044,
            inner_radius=0.060,
            tread=TireTread(style="block", depth=0.005, count=16, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.012),
            shoulder=TireShoulder(width=0.005, radius=0.002),
        ),
        "caster_tire",
    )

    for idx, side in enumerate((-1.0, 1.0)):
        fork = model.part(f"caster_fork_{idx}")
        _tube_between(fork, "swivel_stem", (0.0, 0.0, -0.005), (0.0, 0.0, -0.085), 0.018, bare_metal)
        fork.visual(Box((0.110, 0.125, 0.022)), origin=Origin(xyz=(0.045, 0.0, -0.062)), material=frame_paint, name="fork_bridge")
        for j, y in enumerate((-0.058, 0.058)):
            fork.visual(
                Box((0.040, 0.014, 0.145)),
                origin=Origin(xyz=(0.060, y, -0.145)),
                material=frame_paint,
                name=f"fork_cheek_{j}",
            )
            _bolt_head(fork, f"caster_axle_bolt_{j}", (0.060, y, -0.172), fastener_black, axis="y", radius=0.009)
        model.articulation(
            f"frame_to_caster_fork_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.550, side * 0.245, 0.250)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.10, friction=0.08),
        )

        caster = model.part(f"caster_wheel_{idx}")
        caster.visual(caster_tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=black_molded, name="tire")
        caster.visual(caster_wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=bare_metal, name="rim")
        caster.visual(
            Cylinder(radius=0.010, length=0.132),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bare_metal,
            name="axle_pin",
        )
        model.articulation(
            f"caster_fork_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=caster,
            origin=Origin(xyz=(0.060, 0.0, -0.172)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=14.0),
            motion_properties=MotionProperties(damping=0.04, friction=0.02),
        )

    # Swing-away footrest hangers and folding treaded foot plates.  The paired
    # footrests are intentionally not mimicked: each has its own service hinge.
    for idx, side in enumerate((-1.0, 1.0)):
        hanger = model.part(f"footrest_{idx}")
        hanger.visual(
            Cylinder(radius=0.016, length=0.170),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=bare_metal,
            name="hinge_barrel",
        )
        _tube_between(hanger, "hanger_post", (0.0, 0.0, -0.060), (0.0, 0.0, -0.300), 0.014, frame_paint)
        _tube_between(hanger, "lower_support", (0.0, 0.0, -0.280), (0.325, 0.0, -0.390), 0.014, frame_paint)
        _tube_between(hanger, "heel_loop", (0.090, 0.0, -0.345), (0.325, 0.0, -0.390), 0.010, black_molded)
        hanger.visual(
            Box((0.035, 0.060, 0.030)),
            origin=Origin(xyz=(0.325, 0.0, -0.390)),
            material=bare_metal,
            name="footplate_hinge_lug",
        )
        model.articulation(
            f"frame_to_footrest_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=hanger,
            origin=Origin(xyz=(0.390, side * 0.145, 0.410)),
            axis=(0.0, 0.0, side),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.05),
            motion_properties=MotionProperties(damping=0.08, friction=0.05),
        )

        plate = model.part(f"footplate_{idx}")
        plate.visual(
            Box((0.225, 0.155, 0.024)),
            origin=Origin(xyz=(0.130, 0.0, -0.020)),
            material=black_molded,
            name="textured_plate",
        )
        plate.visual(
            Box((0.014, 0.155, 0.020)),
            origin=Origin(xyz=(0.236, 0.0, -0.002)),
            material=black_molded,
            name="front_toe_lip",
        )
        for j, x in enumerate((0.045, 0.085, 0.125, 0.165)):
            plate.visual(
                Box((0.018, 0.135, 0.012)),
                origin=Origin(xyz=(x + 0.018, 0.0, -0.006)),
                material=fastener_black,
                name=f"traction_rib_{j}",
            )
        model.articulation(
            f"footrest_to_footplate_{idx}",
            ArticulationType.REVOLUTE,
            parent=hanger,
            child=plate,
            origin=Origin(xyz=(0.325, 0.0, -0.390)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.45),
            motion_properties=MotionProperties(damping=0.04, friction=0.03),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    footrest_0 = object_model.get_part("footrest_0")
    footplate_0 = object_model.get_part("footplate_0")
    caster_fork_0 = object_model.get_part("caster_fork_0")
    caster_wheel_0 = object_model.get_part("caster_wheel_0")

    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            object_model.get_part(f"rear_wheel_{idx}"),
            elem_a=f"rear_axle_stub_{idx}",
            elem_b="spoked_rim",
            reason="The fixed axle stub is intentionally captured inside the rotating rear wheel hub bore.",
        )
        ctx.allow_overlap(
            frame,
            object_model.get_part(f"rear_wheel_{idx}"),
            elem_a=f"rear_axle_stub_{idx}",
            elem_b="outer_hub_cap",
            reason="The serviceable axle nut and hub cap are modeled seated over the fixed axle stub.",
        )
        ctx.allow_overlap(
            frame,
            object_model.get_part(f"caster_fork_{idx}"),
            reason="The caster swivel stem is intentionally journaled through the bearing block.",
        )
        ctx.allow_overlap(
            object_model.get_part(f"caster_fork_{idx}"),
            object_model.get_part(f"caster_wheel_{idx}"),
            reason="The caster axle pin is intentionally captured through the fork cheeks and wheel hub.",
        )
        ctx.allow_overlap(
            frame,
            object_model.get_part(f"footrest_{idx}"),
            reason="The swing-away footrest hinge barrel is intentionally nested in the receiver socket.",
        )

    ctx.expect_overlap(
        frame,
        rear_wheel_0,
        axes="y",
        min_overlap=0.025,
        name="rear axle stub remains inserted in wheel hub",
    )
    ctx.expect_overlap(
        rear_wheel_0,
        rear_wheel_1,
        axes="z",
        min_overlap=0.55,
        elem_a="tire",
        elem_b="tire",
        name="large rear tires share a common rolling stance",
    )
    ctx.expect_gap(
        frame,
        footplate_0,
        axis="z",
        min_gap=0.05,
        name="deployed footplate stays below the fixed frame",
    )

    rest_footrest = ctx.part_world_aabb(footrest_0)
    with ctx.pose({"frame_to_footrest_0": 0.85}):
        swung_footrest = ctx.part_world_aabb(footrest_0)
    ctx.check(
        "footrest swings outward from service hinge",
        rest_footrest is not None
        and swung_footrest is not None
        and swung_footrest[0][1] < rest_footrest[0][1] - 0.05,
        details=f"rest={rest_footrest}, swung={swung_footrest}",
    )

    rest_plate_aabb = ctx.part_world_aabb(footplate_0)
    with ctx.pose({"footrest_to_footplate_0": 1.20}):
        folded_plate_aabb = ctx.part_world_aabb(footplate_0)
    ctx.check(
        "footplate folds upward",
        rest_plate_aabb is not None
        and folded_plate_aabb is not None
        and folded_plate_aabb[1][2] > rest_plate_aabb[1][2] + 0.10,
        details=f"rest={rest_plate_aabb}, folded={folded_plate_aabb}",
    )

    rest_caster = ctx.part_world_position(caster_wheel_0)
    with ctx.pose({"frame_to_caster_fork_0": 0.75, "caster_fork_to_wheel_0": 1.2}):
        swivel_caster = ctx.part_world_position(caster_wheel_0)
        ctx.expect_overlap(
            caster_fork_0,
            caster_wheel_0,
            axes="z",
            min_overlap=0.08,
            name="caster fork cheeks still straddle the small wheel",
        )
    ctx.check(
        "caster swivel changes wheel trail",
        rest_caster is not None
        and swivel_caster is not None
        and abs(swivel_caster[1] - rest_caster[1]) > 0.025,
        details=f"rest={rest_caster}, swivel={swivel_caster}",
    )
    ctx.expect_overlap(
        frame,
        caster_fork_0,
        axes="z",
        min_overlap=0.07,
        name="caster swivel stem remains seated through bearing block",
    )

    return ctx.report()


object_model = build_object_model()
