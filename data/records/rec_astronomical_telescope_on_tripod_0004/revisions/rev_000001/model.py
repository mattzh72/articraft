from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_ota_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.074, -0.132),
            (0.076, -0.112),
            (0.072, -0.082),
            (0.071, 0.082),
            (0.078, 0.114),
            (0.082, 0.146),
        ],
        [
            (0.052, -0.126),
            (0.054, -0.108),
            (0.062, -0.078),
            (0.062, 0.096),
            (0.069, 0.126),
            (0.071, 0.138),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _x_cylinder(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    center = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    pitch = math.atan2(-dz, dx)
    return length, Origin(xyz=center, rpy=(0.0, pitch, 0.0))


def _add_leg_geometry(part, *, aluminum, graphite, rubber) -> None:
    open_splay = math.radians(23.0)
    direction = (math.sin(open_splay), 0.0, -math.cos(open_splay))

    upper_start = (0.030, 0.0, -0.036)
    upper_end = (
        upper_start[0] + direction[0] * 0.330,
        0.0,
        upper_start[2] + direction[2] * 0.330,
    )
    lower_start = (
        upper_end[0] + direction[0] * 0.012,
        0.0,
        upper_end[2] + direction[2] * 0.012,
    )
    lower_end = (
        lower_start[0] + direction[0] * 0.360,
        0.0,
        lower_start[2] + direction[2] * 0.360,
    )

    part.visual(
        Box((0.044, 0.024, 0.024)),
        origin=Origin(xyz=(0.022, 0.0, -0.046)),
        material=graphite,
        name="top_mount",
    )

    upper_len, upper_origin = _segment_origin(upper_start, upper_end)
    part.visual(
        Box((upper_len, 0.022, 0.036)),
        origin=upper_origin,
        material=aluminum,
        name="upper_leg",
    )

    lower_len, lower_origin = _segment_origin(lower_start, lower_end)
    part.visual(
        Box((lower_len, 0.018, 0.028)),
        origin=lower_origin,
        material=aluminum,
        name="lower_leg",
    )

    clamp_len, clamp_origin = _segment_origin(upper_end, lower_start)
    part.visual(
        Box((clamp_len + 0.030, 0.024, 0.026)),
        origin=clamp_origin,
        material=graphite,
        name="leg_clamp",
    )
    part.visual(
        Box((0.030, 0.032, 0.016)),
        origin=Origin(xyz=(lower_end[0] + 0.010, 0.0, lower_end[2] + 0.008)),
        material=rubber,
        name="foot_pad",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.34, 0.05, 0.72)),
        mass=0.85,
        origin=Origin(xyz=(0.170, 0.0, -0.340)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maksutov_alt_az_telescope", assets=ASSETS)

    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.33, 0.35, 0.39, 1.0))
    orange_accent = model.material("orange_accent", rgba=(0.84, 0.47, 0.14, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.70, 0.82, 0.90, 0.38))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    ota_shell_mesh = _save_mesh("ota_shell.obj", _build_ota_shell_mesh())

    tripod = model.part("tripod_base")
    tripod.visual(
        Cylinder(radius=0.062, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.706)),
        material=mount_gray,
        name="crown",
    )
    tripod.visual(
        Cylinder(radius=0.051, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.733)),
        material=graphite,
        name="azimuth_seat",
    )
    tripod.visual(
        Cylinder(radius=0.024, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.556)),
        material=mount_gray,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.448)),
        material=graphite,
        name="spreader_collar",
    )

    leg_pivot_radius = 0.056
    leg_pivot_z = 0.694
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        tripod.visual(
            Box((0.044, 0.028, 0.024)),
            origin=Origin(
                xyz=(
                    (leg_pivot_radius + 0.022) * math.cos(angle),
                    (leg_pivot_radius + 0.022) * math.sin(angle),
                    leg_pivot_z - 0.022,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"hinge_block_{index}",
        )
    tripod.inertial = Inertial.from_geometry(
        Box((0.74, 0.74, 0.75)),
        mass=4.9,
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
    )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.049, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=graphite,
        name="turntable",
    )
    mount_head.visual(
        Cylinder(radius=0.056, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=mount_gray,
        name="azimuth_housing",
    )
    mount_head.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(-0.024, -0.006, 0.080)),
        material=mount_gray,
        name="main_casting",
    )
    mount_head.visual(
        Box((0.050, 0.014, 0.220)),
        origin=Origin(xyz=(0.000, 0.083, 0.164)),
        material=graphite,
        name="fork_arm",
    )
    mount_head.visual(
        Box((0.032, 0.014, 0.054)),
        origin=Origin(xyz=(0.000, 0.079, 0.192)),
        material=mount_gray,
        name="trunnion_block",
    )
    mount_head.visual(
        Box((0.046, 0.046, 0.060)),
        origin=Origin(xyz=(-0.042, -0.004, 0.108)),
        material=mount_gray,
        name="motor_box",
    )
    mount_head.visual(
        Box((0.034, 0.014, 0.038)),
        origin=Origin(xyz=(-0.010, 0.082, 0.228)),
        material=mount_gray,
        name="saddle_cap",
    )
    altitude_bearing_geom, altitude_bearing_origin = _y_cylinder(0.024, 0.026, (0.000, 0.058, 0.192))
    mount_head.visual(
        altitude_bearing_geom,
        origin=altitude_bearing_origin,
        material=graphite,
        name="altitude_bearing",
    )
    altitude_lock_geom, altitude_lock_origin = _x_cylinder(0.012, 0.024, (-0.055, 0.014, 0.148))
    mount_head.visual(
        altitude_lock_geom,
        origin=altitude_lock_origin,
        material=orange_accent,
        name="altitude_lock_knob",
    )
    mount_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.26)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.018, 0.130)),
    )

    ota = model.part("ota")
    ota.visual(
        ota_shell_mesh,
        origin=Origin(xyz=(0.086, -0.168, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="ota_shell",
    )
    front_cell_geom, front_cell_origin = _x_cylinder(0.079, 0.012, (0.222, -0.168, 0.0))
    ota.visual(
        front_cell_geom,
        origin=front_cell_origin,
        material=graphite,
        name="front_cell",
    )
    corrector_geom, corrector_origin = _x_cylinder(0.067, 0.004, (0.222, -0.168, 0.0))
    ota.visual(
        corrector_geom,
        origin=corrector_origin,
        material=warm_glass,
        name="corrector_plate",
    )
    secondary_geom, secondary_origin = _x_cylinder(0.016, 0.0025, (0.224, -0.168, 0.0))
    ota.visual(
        secondary_geom,
        origin=secondary_origin,
        material=graphite,
        name="secondary_spot",
    )
    rear_cell_geom, rear_cell_origin = _x_cylinder(0.055, 0.010, (-0.036, -0.168, 0.0))
    ota.visual(
        rear_cell_geom,
        origin=rear_cell_origin,
        material=graphite,
        name="rear_cell",
    )
    baffle_tube_geom, baffle_tube_origin = _x_cylinder(0.013, 0.060, (-0.012, -0.168, 0.0))
    ota.visual(
        baffle_tube_geom,
        origin=baffle_tube_origin,
        material=graphite,
        name="baffle_tube",
    )
    visual_back_geom, visual_back_origin = _x_cylinder(0.020, 0.066, (-0.074, -0.168, 0.0))
    ota.visual(
        visual_back_geom,
        origin=visual_back_origin,
        material=graphite,
        name="visual_back",
    )
    ota.visual(
        Box((0.040, 0.032, 0.056)),
        origin=Origin(xyz=(-0.124, -0.168, 0.028)),
        material=graphite,
        name="diagonal_body",
    )
    ota.visual(
        Cylinder(radius=0.0115, length=0.072),
        origin=Origin(xyz=(-0.126, -0.168, 0.092)),
        material=graphite,
        name="eyepiece_barrel",
    )
    finder_scope_geom, finder_scope_origin = _x_cylinder(0.014, 0.130, (0.088, -0.168, 0.086))
    ota.visual(
        finder_scope_geom,
        origin=finder_scope_origin,
        material=graphite,
        name="finder_scope",
    )
    ota.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.044, -0.168, 0.072)),
        material=mount_gray,
        name="finder_rear_saddle",
    )
    ota.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.132, -0.168, 0.072)),
        material=mount_gray,
        name="finder_front_saddle",
    )
    ota.visual(
        Box((0.122, 0.074, 0.094)),
        origin=Origin(xyz=(0.018, -0.082, 0.0)),
        material=mount_gray,
        name="side_bracket",
    )
    ota.visual(
        Box((0.054, 0.024, 0.064)),
        origin=Origin(xyz=(0.002, -0.046, 0.0)),
        material=graphite,
        name="dovetail_block",
    )
    altitude_stub_geom, altitude_stub_origin = _y_cylinder(0.021, 0.018, (0.000, 0.000, 0.0))
    ota.visual(
        altitude_stub_geom,
        origin=altitude_stub_origin,
        material=graphite,
        name="altitude_stub",
    )
    ota.inertial = Inertial.from_geometry(
        Box((0.40, 0.26, 0.24)),
        mass=4.2,
        origin=Origin(xyz=(0.060, -0.104, 0.015)),
    )

    focus_knob = model.part("focus_knob")
    focus_shaft_geom, focus_shaft_origin = _x_cylinder(0.009, 0.008, (-0.004, 0.0, 0.0))
    focus_knob.visual(
        focus_shaft_geom,
        origin=focus_shaft_origin,
        material=graphite,
        name="focus_shaft",
    )
    focus_knob.visual(
        Box((0.004, 0.010, 0.006)),
        origin=Origin(xyz=(-0.010, 0.013, 0.0)),
        material=orange_accent,
        name="focus_grip",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Box((0.020, 0.030, 0.030)),
        mass=0.05,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )

    leg_parts = []
    for index in range(3):
        leg = model.part(f"leg_{index}")
        _add_leg_geometry(leg, aluminum=aluminum, graphite=graphite, rubber=rubber)
        leg_parts.append(leg)

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=mount_head,
        origin=Origin(xyz=(0.0, 0.0, 0.739)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=mount_head,
        child=ota,
        origin=Origin(xyz=(0.000, 0.058, 0.192)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "ota_to_focus_knob",
        ArticulationType.CONTINUOUS,
        parent=ota,
        child=focus_knob,
        origin=Origin(xyz=(-0.041, -0.168, -0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=6.0,
        ),
    )
    for index, leg in enumerate(leg_parts):
        angle = index * (2.0 * math.pi / 3.0)
        model.articulation(
            f"tripod_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=tripod,
            child=leg,
            origin=Origin(
                xyz=(leg_pivot_radius * math.cos(angle), leg_pivot_radius * math.sin(angle), leg_pivot_z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.8,
                lower=0.0,
                upper=0.40,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    tripod = object_model.get_part("tripod_base")
    mount_head = object_model.get_part("mount_head")
    ota = object_model.get_part("ota")
    focus_knob = object_model.get_part("focus_knob")
    legs = [object_model.get_part(f"leg_{index}") for index in range(3)]

    azimuth_axis = object_model.get_articulation("azimuth_axis")
    altitude_axis = object_model.get_articulation("altitude_axis")
    focus_axis = object_model.get_articulation("ota_to_focus_knob")
    leg_joints = [object_model.get_articulation(f"tripod_to_leg_{index}") for index in range(3)]

    azimuth_seat = tripod.get_visual("azimuth_seat")
    hinge_blocks = [tripod.get_visual(f"hinge_block_{index}") for index in range(3)]
    turntable = mount_head.get_visual("turntable")
    altitude_bearing = mount_head.get_visual("altitude_bearing")

    ota_shell = ota.get_visual("ota_shell")
    front_cell = ota.get_visual("front_cell")
    corrector_plate = ota.get_visual("corrector_plate")
    secondary_spot = ota.get_visual("secondary_spot")
    rear_cell = ota.get_visual("rear_cell")
    altitude_stub = ota.get_visual("altitude_stub")
    finder_scope = ota.get_visual("finder_scope")
    finder_rear_saddle = ota.get_visual("finder_rear_saddle")
    finder_front_saddle = ota.get_visual("finder_front_saddle")
    visual_back = ota.get_visual("visual_back")
    diagonal_body = ota.get_visual("diagonal_body")

    focus_shaft = focus_knob.get_visual("focus_shaft")
    focus_grip = focus_knob.get_visual("focus_grip")
    leg_mounts = [leg.get_visual("top_mount") for leg in legs]
    upper_legs = [leg.get_visual("upper_leg") for leg in legs]
    leg_feet = [leg.get_visual("foot_pad") for leg in legs]

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    def _require_aabb(part, elem, check_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        ctx.check(check_name, aabb is not None, f"Missing AABB for {part.name}:{elem.name}")
        return aabb

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        ota,
        mount_head,
        reason="The telescope trunnion stub rides inside the altitude bearing sleeve.",
        elem_a=altitude_stub,
        elem_b=altitude_bearing,
    )
    ctx.allow_overlap(
        focus_knob,
        ota,
        reason="The focus axle passes through the rear focusing housing of the telescope.",
        elem_a=focus_shaft,
        elem_b=ota_shell,
    )
    ctx.allow_overlap(
        focus_knob,
        ota,
        reason="The focus axle is intentionally seated inside the rear cell casting.",
        elem_a=focus_shaft,
        elem_b=rear_cell,
    )
    for index, leg in enumerate(legs):
        ctx.allow_overlap(
            leg,
            tripod,
            reason="Each tripod upper leg is captured slightly inside its crown hinge socket.",
            elem_a=upper_legs[index],
            elem_b=hinge_blocks[index],
        )
    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        mount_head,
        tripod,
        axes="xy",
        min_overlap=0.080,
        elem_a=turntable,
        elem_b=azimuth_seat,
    )
    ctx.expect_gap(
        mount_head,
        tripod,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=turntable,
        negative_elem=azimuth_seat,
    )
    ctx.expect_contact(
        ota,
        mount_head,
        elem_a=altitude_stub,
        elem_b=altitude_bearing,
    )
    ctx.expect_contact(
        focus_knob,
        ota,
        elem_a=focus_shaft,
        elem_b=rear_cell,
    )
    ctx.expect_within(
        ota,
        ota,
        axes="yz",
        inner_elem=corrector_plate,
        outer_elem=ota_shell,
    )
    ctx.expect_within(
        ota,
        ota,
        axes="yz",
        inner_elem=secondary_spot,
        outer_elem=corrector_plate,
    )
    ctx.expect_contact(ota, ota, elem_a=front_cell, elem_b=corrector_plate)
    ctx.expect_contact(ota, ota, elem_a=finder_scope, elem_b=finder_rear_saddle)
    ctx.expect_contact(ota, ota, elem_a=finder_scope, elem_b=finder_front_saddle)
    ctx.expect_contact(ota, ota, elem_a=visual_back, elem_b=diagonal_body)

    for index, leg in enumerate(legs):
        ctx.expect_contact(
            leg,
            tripod,
            elem_a=leg_mounts[index],
            elem_b=hinge_blocks[index],
            name=f"leg_{index}_hinge_mount_contact",
        )

    front_cell_rest = _require_aabb(ota, front_cell, "front_cell_rest_aabb")
    rear_cell_rest = _require_aabb(ota, rear_cell, "rear_cell_rest_aabb")
    focus_grip_rest = _require_aabb(focus_knob, focus_grip, "focus_grip_rest_aabb")
    foot_aabbs = [
        _require_aabb(legs[index], leg_feet[index], f"leg_{index}_foot_aabb")
        for index in range(3)
    ]

    foot_radii = [math.hypot(_center(aabb)[0], _center(aabb)[1]) for aabb in foot_aabbs]
    for index, aabb in enumerate(foot_aabbs):
        ctx.check(
            f"leg_{index}_foot_ground_height",
            0.0 <= aabb[0][2] <= 0.03,
            f"leg_{index} foot min z {aabb[0][2]:.4f} m should sit near the ground plane",
        )
    ctx.check(
        "tripod_open_footprint",
        all(0.30 <= radius <= 0.40 for radius in foot_radii),
        f"Tripod foot radii {foot_radii!r} do not read as a compact open tripod",
    )

    front_center = _center(front_cell_rest)
    rear_center = _center(rear_cell_rest)
    ctx.check(
        "compact_maksutov_tube_length",
        0.24 <= front_center[0] - rear_center[0] <= 0.28,
        f"Optical tube body length {front_center[0] - rear_center[0]:.4f} m is not compact-Mak sized",
    )

    with ctx.pose({altitude_axis: 1.10}):
        ctx.fail_if_isolated_parts(name="altitude_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="altitude_pose_no_unintended_overlap")
        ctx.expect_contact(
            ota,
            mount_head,
            elem_a=altitude_stub,
            elem_b=altitude_bearing,
            name="altitude_pose_stub_bearing_contact",
        )
        front_cell_high = _require_aabb(ota, front_cell, "front_cell_high_aabb")
        ctx.check(
            "altitude_axis_raises_front_cell",
            _center(front_cell_high)[2] > front_center[2] + 0.15,
            "Front of the telescope did not rise enough at a high-altitude pose",
        )

    with ctx.pose({azimuth_axis: 1.7, altitude_axis: 0.85}):
        ctx.fail_if_isolated_parts(name="azimuth_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="azimuth_pose_no_unintended_overlap")
        front_cell_turned = _require_aabb(ota, front_cell, "front_cell_turned_aabb")
        turned_center = _center(front_cell_turned)
        ctx.check(
            "azimuth_axis_swings_telescope",
            math.hypot(turned_center[0] - front_center[0], turned_center[1] - front_center[1]) > 0.18,
            "Azimuth motion did not sweep the telescope noticeably in plan view",
        )
        ctx.check(
            "azimuth_pose_keeps_scope_clear_of_tripod",
            turned_center[2] > 0.95,
            "Combined azimuth and altitude pose drops the OTA too low",
        )

    with ctx.pose({focus_axis: math.pi / 2.0}):
        ctx.expect_contact(
            focus_knob,
            ota,
            elem_a=focus_shaft,
            elem_b=rear_cell,
            name="focus_knob_stays_mounted",
        )
        focus_grip_rotated = _require_aabb(focus_knob, focus_grip, "focus_grip_rotated_aabb")
        rotated_center = _center(focus_grip_rotated)
        rest_center = _center(focus_grip_rest)
        ctx.check(
            "focus_knob_rotates_grip",
            abs(rotated_center[1] - rest_center[1]) > 0.008 and abs(rotated_center[2] - rest_center[2]) > 0.008,
            "Focus grip did not move enough to read as a rotating focus knob",
        )

    folded_pose = {
        leg_joint: leg_joint.motion_limits.upper
        for leg_joint in leg_joints
        if leg_joint.motion_limits is not None and leg_joint.motion_limits.upper is not None
    }
    with ctx.pose(folded_pose):
        ctx.fail_if_isolated_parts(name="tripod_folded_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="tripod_folded_no_unintended_overlap")
        folded_foot = _require_aabb(legs[0], leg_feet[0], "leg_0_folded_foot_aabb")
        ctx.expect_contact(
            legs[0],
            tripod,
            elem_a=leg_mounts[0],
            elem_b=hinge_blocks[0],
            contact_tol=0.003,
            name="leg_0_folded_mount_contact",
        )
        ctx.check(
            "tripod_leg_folds_inward",
            math.hypot(_center(folded_foot)[0], _center(folded_foot)[1]) < foot_radii[0] - 0.20,
            "Folded tripod leg did not retract inward enough",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
