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
    repair_loft,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_exercise_bike", assets=ASSETS)

    graphite_matte = model.material("graphite_matte", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal_satin = model.material("charcoal_satin", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.67, 0.69, 0.72, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.54, 0.55, 0.57, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.08, 0.08, 0.09, 1.0))
    console_glass = model.material("console_glass", rgba=(0.26, 0.29, 0.32, 0.55))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def rpy_for_cylinder(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        length_xy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(length_xy, dz)
        return (0.0, pitch, yaw)

    def add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
        part.visual(
            Cylinder(radius=radius, length=distance(a, b)),
            origin=Origin(xyz=midpoint(a, b), rpy=rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    def yz_section(
        x_pos: float,
        width: float,
        height: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y, z_center + z)
            for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
        ]

    def saddle_mesh():
        return repair_loft(
            section_loft(
                [
                    yz_section(-0.08, 0.12, 0.024, 0.008, 0.445),
                    yz_section(0.00, 0.18, 0.050, 0.016, 0.458),
                    yz_section(0.10, 0.15, 0.042, 0.014, 0.450),
                    yz_section(0.18, 0.06, 0.020, 0.007, 0.434),
                ]
            )
        )

    def saddle_insert_mesh():
        return repair_loft(
            section_loft(
                [
                    yz_section(-0.03, 0.10, 0.012, 0.005, 0.470),
                    yz_section(0.04, 0.13, 0.016, 0.006, 0.478),
                    yz_section(0.12, 0.09, 0.012, 0.005, 0.468),
                ]
            )
        )

    def drive_cover_mesh():
        return repair_loft(
            section_loft(
                [
                    yz_section(0.03, 0.108, 0.072, 0.020, 0.135),
                    yz_section(0.09, 0.136, 0.118, 0.028, 0.162),
                    yz_section(0.15, 0.154, 0.150, 0.032, 0.205),
                    yz_section(0.20, 0.142, 0.156, 0.030, 0.240),
                ]
            )
        )

    def belt_tower_mesh():
        return repair_loft(
            section_loft(
                [
                    yz_section(-0.02, 0.150, 0.140, 0.026, 0.240),
                    yz_section(0.07, 0.144, 0.188, 0.028, 0.295),
                    yz_section(0.15, 0.120, 0.164, 0.024, 0.340),
                ]
            )
        )

    def bearing_ring_mesh(outer_radius: float, inner_radius: float, length: float):
        half = length * 0.5
        return LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(math.pi / 2.0)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.26, 0.64, 0.98)),
        mass=42.0,
        origin=Origin(xyz=(0.06, 0.0, 0.49)),
    )

    frame.visual(
        Box((0.60, 0.10, 0.035)),
        origin=Origin(xyz=(-0.42, 0.0, 0.0175)),
        material=charcoal_satin,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.50, 0.09, 0.035)),
        origin=Origin(xyz=(0.52, 0.0, 0.0175)),
        material=charcoal_satin,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.08, 0.05, 0.018)),
        origin=Origin(xyz=(-0.63, 0.0, 0.043)),
        material=black_rubber,
        name="rear_level_pad",
    )
    frame.visual(
        Box((0.07, 0.05, 0.014)),
        origin=Origin(xyz=(0.73, 0.0, 0.040)),
        material=black_rubber,
        name="front_foot_pad",
    )
    frame.visual(
        Box((1.25, 0.07, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.055)),
        material=charcoal_satin,
        name="base_rail",
    )

    frame.visual(
        save_mesh(
            "bike_main_spine.obj",
            sweep_profile_along_spline(
                [(-0.34, 0.0, 0.07), (-0.22, 0.0, 0.12), (-0.09, 0.0, 0.17)],
                profile=rounded_rect_profile(0.078, 0.056, radius=0.014, corner_segments=8),
                samples_per_segment=18,
                cap_profile=True,
            ),
        ),
        material=graphite_matte,
        name="main_spine",
    )
    frame.visual(
        save_mesh(
            "bike_front_mast.obj",
            sweep_profile_along_spline(
                [(0.08, 0.0, 0.28), (0.19, 0.0, 0.55), (0.31, 0.0, 0.84)],
                profile=rounded_rect_profile(0.066, 0.050, radius=0.012, corner_segments=8),
                samples_per_segment=18,
                cap_profile=True,
            ),
        ),
        material=graphite_matte,
        name="front_mast",
    )
    frame.visual(
        save_mesh(
            "bike_seat_support.obj",
            sweep_profile_along_spline(
                [(-0.14, 0.0, 0.20), (-0.20, 0.0, 0.46), (-0.28, 0.0, 0.68)],
                profile=rounded_rect_profile(0.060, 0.048, radius=0.011, corner_segments=8),
                samples_per_segment=18,
                cap_profile=True,
            ),
        ),
        material=graphite_matte,
        name="seat_support",
    )
    frame.visual(
        save_mesh(
            "bike_seat_tube_gusset.obj",
            sweep_profile_along_spline(
                [(-0.10, 0.0, 0.165), (-0.125, 0.0, 0.19), (-0.15, 0.0, 0.225)],
                profile=rounded_rect_profile(0.050, 0.040, radius=0.010, corner_segments=8),
                samples_per_segment=12,
                cap_profile=True,
            ),
        ),
        material=charcoal_satin,
        name="seat_tube_gusset",
    )

    add_member(
        frame,
        (-0.63, 0.03, 0.035),
        (-0.22, 0.03, 0.33),
        0.017,
        charcoal_satin,
        name="left_rear_brace",
    )
    add_member(
        frame,
        (-0.63, -0.03, 0.035),
        (-0.22, -0.03, 0.33),
        0.017,
        charcoal_satin,
        name="right_rear_brace",
    )
    add_member(
        frame,
        (0.70, 0.03, 0.035),
        (0.12, 0.03, 0.29),
        0.016,
        charcoal_satin,
        name="left_front_brace",
    )
    add_member(
        frame,
        (0.70, -0.03, 0.035),
        (0.12, -0.03, 0.29),
        0.016,
        charcoal_satin,
        name="right_front_brace",
    )

    frame.visual(
        save_mesh("bike_lower_drive_cover.obj", drive_cover_mesh()),
        material=graphite_matte,
        name="lower_drive_cover",
    )
    frame.visual(
        save_mesh("bike_belt_tower.obj", belt_tower_mesh()),
        material=graphite_matte,
        name="belt_tower",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.14, 0.0, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_matte,
        name="flywheel_shell",
    )
    frame.visual(
        Cylinder(radius=0.173, length=0.010),
        origin=Origin(xyz=(0.14, 0.055, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_satin,
        name="left_side_cover",
    )
    frame.visual(
        Cylinder(radius=0.173, length=0.010),
        origin=Origin(xyz=(0.14, -0.055, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_satin,
        name="right_side_cover",
    )
    frame.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.12, 0.056, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="left_housing_trim",
    )
    frame.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.12, -0.056, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="right_housing_trim",
    )
    frame.visual(
        Box((0.09, 0.006, 0.045)),
        origin=Origin(xyz=(0.16, 0.0, 0.52)),
        material=warm_gray,
        name="seam_break",
    )

    bearing_ring = save_mesh("bike_bearing_ring.obj", bearing_ring_mesh(0.050, 0.0145, 0.030))
    frame.visual(
        bearing_ring,
        origin=Origin(xyz=(-0.02, 0.076, 0.24)),
        material=satin_alloy,
        name="left_bearing_boss",
    )
    frame.visual(
        bearing_ring,
        origin=Origin(xyz=(-0.02, -0.076, 0.24)),
        material=satin_alloy,
        name="right_bearing_boss",
    )
    frame.visual(
        Cylinder(radius=0.048, length=0.124),
        origin=Origin(xyz=(-0.02, 0.0, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_satin,
        name="bottom_bracket_shell",
    )

    frame.visual(
        Box((0.07, 0.07, 0.024)),
        origin=Origin(xyz=(0.31, 0.0, 0.852)),
        material=charcoal_satin,
        name="front_mast_top",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(-0.28, 0.0, 0.650)),
        material=charcoal_satin,
        name="seat_socket",
    )

    cockpit = model.part("cockpit")
    cockpit.inertial = Inertial.from_geometry(
        Box((0.48, 0.42, 0.42)),
        mass=4.5,
        origin=Origin(xyz=(-0.08, 0.0, 0.20)),
    )
    cockpit.visual(
        Box((0.07, 0.07, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal_satin,
        name="stem_base",
    )
    cockpit.visual(
        Cylinder(radius=0.028, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=graphite_matte,
        name="stem_column",
    )
    cockpit.visual(
        Box((0.055, 0.13, 0.040)),
        origin=Origin(xyz=(-0.01, 0.0, 0.235)),
        material=charcoal_satin,
        name="bar_clamp_bridge",
    )
    cockpit.visual(
        save_mesh(
            "bike_handlebar_loop.obj",
            sweep_profile_along_spline(
                [
                    (-0.21, -0.13, 0.33),
                    (-0.10, -0.18, 0.30),
                    (-0.01, -0.05, 0.23),
                    (-0.01, 0.05, 0.23),
                    (-0.10, 0.18, 0.30),
                    (-0.21, 0.13, 0.33),
                ],
                profile=rounded_rect_profile(0.026, 0.018, radius=0.005, corner_segments=8),
                samples_per_segment=18,
                cap_profile=True,
            ),
        ),
        material=graphite_matte,
        name="handlebar_loop",
    )
    cockpit.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.21, -0.15, 0.33), rpy=(math.pi / 2.0, 0.0, 0.20)),
        material=black_rubber,
        name="left_grip",
    )
    cockpit.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.21, 0.15, 0.33), rpy=(math.pi / 2.0, 0.0, -0.20)),
        material=black_rubber,
        name="right_grip",
    )
    cockpit.visual(
        Box((0.10, 0.034, 0.085)),
        origin=Origin(xyz=(-0.11, 0.0, 0.295), rpy=(0.20, 0.0, 0.0)),
        material=console_glass,
        name="console",
    )
    cockpit.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(-0.12, 0.0, 0.255)),
        material=satin_alloy,
        name="console_stalk",
    )
    add_member(
        cockpit,
        (-0.02, 0.0, 0.235),
        (-0.11, 0.0, 0.255),
        0.012,
        charcoal_satin,
        name="console_arm",
    )

    saddle = model.part("saddle_assembly")
    saddle.inertial = Inertial.from_geometry(
        Box((0.42, 0.22, 0.62)),
        mass=5.0,
        origin=Origin(xyz=(0.05, 0.0, 0.31)),
    )
    saddle.visual(
        Cylinder(radius=0.022, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=satin_alloy,
        name="lower_post",
    )
    saddle.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=satin_alloy,
        name="upper_post",
    )
    saddle.visual(
        Box((0.08, 0.05, 0.04)),
        origin=Origin(xyz=(0.03, 0.0, 0.39)),
        material=charcoal_satin,
        name="seat_clamp",
    )
    saddle.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.03, 0.040, 0.39), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="seat_adjust_knob",
    )
    add_member(
        saddle,
        (0.00, -0.030, 0.39),
        (0.11, -0.040, 0.425),
        0.007,
        charcoal_satin,
        name="left_saddle_rail",
    )
    add_member(
        saddle,
        (0.00, 0.030, 0.39),
        (0.11, 0.040, 0.425),
        0.007,
        charcoal_satin,
        name="right_saddle_rail",
    )
    saddle.visual(
        save_mesh("bike_saddle_shell.obj", saddle_mesh()),
        material=saddle_vinyl,
        name="saddle_shell",
    )
    saddle.visual(
        save_mesh("bike_saddle_insert.obj", saddle_insert_mesh()),
        material=charcoal_satin,
        name="saddle_insert",
    )

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Box((0.42, 0.32, 0.12)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crankset.visual(
        Cylinder(radius=0.012, length=0.184),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="axle_spindle",
    )
    crankset.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.103, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_satin,
        name="left_crank_cap",
    )
    crankset.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, -0.103, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_satin,
        name="right_crank_cap",
    )
    crankset.visual(
        Box((0.030, 0.028, 0.038)),
        origin=Origin(xyz=(0.012, 0.103, 0.0)),
        material=charcoal_satin,
        name="left_arm_root",
    )
    crankset.visual(
        Box((0.030, 0.028, 0.038)),
        origin=Origin(xyz=(-0.012, -0.103, 0.0)),
        material=charcoal_satin,
        name="right_arm_root",
    )
    crankset.visual(
        Box((0.165, 0.018, 0.028)),
        origin=Origin(xyz=(0.095, 0.103, 0.0)),
        material=graphite_matte,
        name="left_crank_arm",
    )
    crankset.visual(
        Box((0.165, 0.018, 0.028)),
        origin=Origin(xyz=(-0.095, -0.103, 0.0)),
        material=graphite_matte,
        name="right_crank_arm",
    )
    crankset.visual(
        Cylinder(radius=0.006, length=0.065),
        origin=Origin(xyz=(0.176, 0.120, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="left_pedal_spindle",
    )
    crankset.visual(
        Cylinder(radius=0.006, length=0.065),
        origin=Origin(xyz=(-0.176, -0.120, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="right_pedal_spindle",
    )
    crankset.visual(
        Box((0.105, 0.045, 0.020)),
        origin=Origin(xyz=(0.176, 0.150, 0.0)),
        material=black_rubber,
        name="left_pedal_body",
    )
    crankset.visual(
        Box((0.105, 0.045, 0.020)),
        origin=Origin(xyz=(-0.176, -0.150, 0.0)),
        material=black_rubber,
        name="right_pedal_body",
    )
    crankset.visual(
        Box((0.060, 0.010, 0.028)),
        origin=Origin(xyz=(0.176, 0.150, 0.018)),
        material=warm_gray,
        name="left_toe_cage",
    )
    crankset.visual(
        Box((0.060, 0.010, 0.028)),
        origin=Origin(xyz=(-0.176, -0.150, 0.018)),
        material=warm_gray,
        name="right_toe_cage",
    )

    model.articulation(
        "frame_to_cockpit",
        ArticulationType.FIXED,
        parent=frame,
        child=cockpit,
        origin=Origin(xyz=(0.31, 0.0, 0.864)),
    )
    model.articulation(
        "frame_to_saddle",
        ArticulationType.FIXED,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(-0.28, 0.0, 0.662)),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(-0.02, 0.0, 0.24)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=14.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    cockpit = object_model.get_part("cockpit")
    saddle = object_model.get_part("saddle_assembly")
    crankset = object_model.get_part("crankset")

    crank_spin = object_model.get_articulation("frame_to_crank")

    rear_stabilizer = frame.get_visual("rear_stabilizer")
    front_mast_top = frame.get_visual("front_mast_top")
    seat_socket = frame.get_visual("seat_socket")
    seat_support = frame.get_visual("seat_support")
    left_bearing = frame.get_visual("left_bearing_boss")
    right_bearing = frame.get_visual("right_bearing_boss")
    bottom_bracket_shell = frame.get_visual("bottom_bracket_shell")
    belt_tower = frame.get_visual("belt_tower")

    stem_base = cockpit.get_visual("stem_base")
    lower_post = saddle.get_visual("lower_post")
    axle_spindle = crankset.get_visual("axle_spindle")
    left_cap = crankset.get_visual("left_crank_cap")
    right_cap = crankset.get_visual("right_crank_cap")
    left_pedal = crankset.get_visual("left_pedal_body")
    right_pedal = crankset.get_visual("right_pedal_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        frame,
        crankset,
        elem_a=bottom_bracket_shell,
        elem_b=axle_spindle,
        reason="The crank spindle runs inside the bottom-bracket shell.",
    )
    ctx.allow_overlap(
        frame,
        crankset,
        elem_a=belt_tower,
        elem_b=axle_spindle,
        reason="The drive housing encloses the spindle passage through the flywheel case.",
    )
    ctx.allow_overlap(
        frame,
        saddle,
        elem_a=seat_socket,
        elem_b=lower_post,
        reason="The saddle post is inserted into the seat socket for height adjustment.",
    )
    ctx.allow_overlap(
        frame,
        saddle,
        elem_a=seat_support,
        elem_b=lower_post,
        reason="The outer seat-tube support wraps around the inserted saddle post.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_fixed=True)

    ctx.expect_contact(cockpit, frame, elem_a=stem_base, elem_b=front_mast_top)
    ctx.expect_overlap(cockpit, frame, axes="xy", elem_a=stem_base, elem_b=front_mast_top, min_overlap=0.055)
    ctx.expect_gap(
        cockpit,
        frame,
        axis="z",
        positive_elem=stem_base,
        negative_elem=front_mast_top,
        max_gap=0.001,
        max_penetration=0.001,
    )

    ctx.expect_within(saddle, frame, axes="xy", inner_elem=lower_post, outer_elem=seat_socket)
    ctx.expect_overlap(saddle, frame, axes="z", elem_a=lower_post, elem_b=seat_socket, min_overlap=0.015)

    ctx.expect_contact(crankset, frame, elem_a=left_cap, elem_b=left_bearing)
    ctx.expect_contact(crankset, frame, elem_a=right_cap, elem_b=right_bearing)
    ctx.expect_overlap(crankset, frame, axes="z", elem_a=left_cap, elem_b=left_bearing, min_overlap=0.070)
    ctx.expect_overlap(crankset, frame, axes="z", elem_a=right_cap, elem_b=right_bearing, min_overlap=0.070)
    ctx.expect_within(
        crankset,
        frame,
        axes="xz",
        inner_elem=axle_spindle,
        outer_elem=bottom_bracket_shell,
        margin=0.0,
    )
    ctx.expect_origin_gap(cockpit, saddle, axis="x", min_gap=0.52, max_gap=0.68)
    ctx.expect_origin_gap(cockpit, saddle, axis="z", min_gap=0.10, max_gap=0.22)
    ctx.expect_origin_gap(saddle, crankset, axis="z", min_gap=0.40, max_gap=0.52)

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    left_rest_aabb = ctx.part_element_world_aabb(crankset, elem=left_pedal)
    right_rest_aabb = ctx.part_element_world_aabb(crankset, elem=right_pedal)
    assert left_rest_aabb is not None
    assert right_rest_aabb is not None
    left_rest = aabb_center(left_rest_aabb)
    right_rest = aabb_center(right_rest_aabb)
    ctx.check(
        "pedal_rest_phase",
        left_rest[0] > right_rest[0] + 0.30,
        f"Expected opposed crank arms at rest, got left={left_rest} right={right_rest}",
    )

    with ctx.pose({crank_spin: math.pi / 2.0}):
        ctx.expect_contact(crankset, frame, elem_a=left_cap, elem_b=left_bearing)
        ctx.expect_contact(crankset, frame, elem_a=right_cap, elem_b=right_bearing)

        left_quarter_aabb = ctx.part_element_world_aabb(crankset, elem=left_pedal)
        right_quarter_aabb = ctx.part_element_world_aabb(crankset, elem=right_pedal)
        assert left_quarter_aabb is not None
        assert right_quarter_aabb is not None
        left_quarter = aabb_center(left_quarter_aabb)
        right_quarter = aabb_center(right_quarter_aabb)
        ctx.check(
            "pedal_quarter_turn_axis",
            right_quarter[2] > left_quarter[2] + 0.30,
            f"Expected positive y-axis crank rotation to move the rearward crank upward, got left={left_quarter} right={right_quarter}",
        )

    with ctx.pose({crank_spin: -math.pi / 2.0}):
        left_low_aabb = ctx.part_element_world_aabb(crankset, elem=left_pedal)
        rear_aabb = ctx.part_element_world_aabb(frame, elem=rear_stabilizer)
        assert left_low_aabb is not None
        assert rear_aabb is not None
        left_low_min_z = left_low_aabb[0][2]
        rear_top_z = rear_aabb[1][2]
        ctx.check(
            "pedal_floor_clearance",
            left_low_min_z > rear_top_z + 0.020,
            f"Pedal drops too close to base: pedal_min_z={left_low_min_z:.4f}, rear_top_z={rear_top_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
