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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_chair", assets=ASSETS)

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.20, 0.21, 0.23, 1.0))
    mesh_fabric = model.material("mesh_fabric", rgba=(0.22, 0.25, 0.29, 0.78))
    seat_fabric = model.material("seat_fabric", rgba=(0.18, 0.20, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.62, 0.65, 0.69, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _translate_profile(profile, dx: float = 0.0, dy: float = 0.0):
        return [(x + dx, y + dy) for x, y in profile]

    def _section(profile, z: float):
        return [(x, y, z) for x, y in profile]

    def _spoke_points(angle: float) -> list[tuple[float, float, float]]:
        radii = (0.060, 0.150, 0.240, 0.286)
        heights = (0.048, 0.046, 0.056, 0.072)
        return [
            (radius * math.cos(angle), radius * math.sin(angle), z)
            for radius, z in zip(radii, heights)
        ]

    def _wheel_tire_mesh(name: str, radius: float, width: float):
        half_width = width * 0.5
        profile = [
            (radius * 0.62, -half_width),
            (radius * 0.87, -half_width * 0.96),
            (radius * 0.98, -half_width * 0.56),
            (radius, -half_width * 0.15),
            (radius, half_width * 0.15),
            (radius * 0.98, half_width * 0.56),
            (radius * 0.87, half_width * 0.96),
            (radius * 0.62, half_width),
            (radius * 0.46, half_width * 0.42),
            (radius * 0.42, 0.0),
            (radius * 0.46, -half_width * 0.42),
            (radius * 0.62, -half_width),
        ]
        return _save_mesh(
            name,
            LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0),
        )

    def _seat_cushion_mesh():
        bottom = rounded_rect_profile(0.48, 0.44, 0.070, corner_segments=8)
        middle = rounded_rect_profile(0.50, 0.46, 0.082, corner_segments=8)
        top = rounded_rect_profile(0.47, 0.42, 0.072, corner_segments=8)
        geom = section_loft(
            [
                _section(bottom, 0.000),
                _section(_translate_profile(middle, 0.0, 0.010), 0.038),
                _section(_translate_profile(top, 0.0, 0.020), 0.070),
            ]
        )
        return _save_mesh("chair_seat_cushion.obj", geom)

    def _back_frame_mesh():
        loop_points = [
            (-0.205, 0.0, -0.015),
            (-0.235, 0.0, 0.145),
            (-0.228, 0.0, 0.355),
            (-0.145, 0.0, 0.545),
            (0.0, 0.0, 0.610),
            (0.145, 0.0, 0.545),
            (0.228, 0.0, 0.355),
            (0.235, 0.0, 0.145),
            (0.205, 0.0, -0.015),
            (0.105, 0.0, -0.030),
            (-0.105, 0.0, -0.030),
        ]
        geom = tube_from_spline_points(
            loop_points,
            radius=0.012,
            samples_per_segment=16,
            closed_spline=True,
            radial_segments=16,
        )
        return _save_mesh("chair_back_frame.obj", geom)

    def _lumbar_bar_mesh():
        geom = tube_from_spline_points(
            [
                (-0.190, 0.0, 0.195),
                (-0.090, 0.0, 0.225),
                (0.0, 0.0, 0.238),
                (0.090, 0.0, 0.225),
                (0.190, 0.0, 0.195),
            ],
            radius=0.009,
            samples_per_segment=16,
            radial_segments=14,
        )
        return _save_mesh("chair_lumbar_bar.obj", geom)

    def _back_mesh_panel():
        outer = rounded_rect_profile(0.430, 0.560, 0.058, corner_segments=8)
        holes = []
        for row_y in (-0.205, -0.135, -0.065, 0.005, 0.075, 0.145, 0.215):
            for col_x in (-0.140, -0.070, 0.0, 0.070, 0.140):
                holes.append(
                    _translate_profile(
                        rounded_rect_profile(0.032, 0.046, 0.011, corner_segments=4),
                        col_x,
                        row_y,
                    )
                )
        geom = ExtrudeWithHolesGeometry(
            outer_profile=outer,
            hole_profiles=holes,
            height=0.004,
            center=True,
        ).rotate_x(math.pi / 2.0)
        return _save_mesh("chair_back_mesh_panel.obj", geom)

    def _back_top_bar_mesh():
        geom = tube_from_spline_points(
            [
                (-0.222, 0.0, -0.010),
                (-0.132, 0.0, 0.002),
                (0.0, 0.0, 0.010),
                (0.132, 0.0, 0.002),
                (0.222, 0.0, -0.010),
            ],
            radius=0.013,
            samples_per_segment=16,
            radial_segments=14,
        )
        return _save_mesh("chair_back_top_bar.obj", geom)

    def _seat_front_trim_mesh():
        lower = rounded_rect_profile(0.360, 0.150, 0.034, corner_segments=6)
        middle = rounded_rect_profile(0.374, 0.128, 0.036, corner_segments=6)
        upper = rounded_rect_profile(0.338, 0.090, 0.030, corner_segments=6)
        geom = section_loft(
            [
                _section(_translate_profile(lower, 0.0, -0.010), -0.020),
                _section(_translate_profile(middle, 0.0, 0.008), 0.000),
                _section(_translate_profile(upper, 0.0, 0.026), 0.020),
            ]
        )
        return _save_mesh("chair_seat_front_trim.obj", geom)

    seat_mesh = _seat_cushion_mesh()
    back_frame_mesh = _back_frame_mesh()
    back_panel_mesh = _back_mesh_panel()
    back_top_bar_mesh = _back_top_bar_mesh()
    seat_front_trim_mesh = _seat_front_trim_mesh()

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=graphite,
        name="hub_shell",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=aluminum,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=charcoal,
        name="column_shroud",
    )
    base.visual(
        Box((0.160, 0.140, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, 0.434)),
        material=dark_frame,
        name="top_plate",
    )
    base.visual(
        Box((0.140, 0.080, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.428)),
        material=dark_frame,
        name="rear_crossmember",
    )
    base.visual(
        Box((0.026, 0.080, 0.034)),
        origin=Origin(xyz=(-0.050, -0.030, 0.421)),
        material=dark_frame,
        name="tilt_cheek_left",
    )
    base.visual(
        Box((0.026, 0.080, 0.034)),
        origin=Origin(xyz=(0.050, -0.030, 0.421)),
        material=dark_frame,
        name="tilt_cheek_right",
    )

    caster_radius = 0.028
    caster_width = 0.018
    wheel_center_radius = 0.308
    wheel_center_z = 0.038

    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        caster_x = wheel_center_radius * math.cos(angle)
        caster_y = wheel_center_radius * math.sin(angle)
        base.visual(
            _save_mesh(
                f"chair_spoke_{index}.obj",
                tube_from_spline_points(
                    _spoke_points(angle),
                    radius=0.017,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=graphite,
            name=f"spoke_{index}",
        )
        base.visual(
            Box((0.044, 0.016, 0.006)),
            origin=Origin(xyz=(caster_x, caster_y, 0.072), rpy=(0.0, 0.0, angle)),
            material=charcoal,
            name=f"fork_{index}_bridge",
        )
        base.visual(
            Box((0.004, 0.012, 0.036)),
            origin=Origin(
                xyz=(
                    caster_x - 0.014 * math.cos(angle),
                    caster_y - 0.014 * math.sin(angle),
                    0.054,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"fork_{index}_left",
        )
        base.visual(
            Box((0.004, 0.012, 0.036)),
            origin=Origin(
                xyz=(
                    caster_x + 0.014 * math.cos(angle),
                    caster_y + 0.014 * math.sin(angle),
                    0.054,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"fork_{index}_right",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.700, 0.700, 0.470)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
    )

    seat = model.part("seat_assembly")
    seat.visual(
        Box((0.120, 0.080, 0.008)),
        origin=Origin(xyz=(0.0, 0.015, 0.004)),
        material=dark_frame,
        name="mount_pad",
    )
    seat.visual(
        Box((0.240, 0.180, 0.030)),
        origin=Origin(xyz=(0.0, 0.030, 0.020)),
        material=dark_frame,
        name="underseat_shell",
    )
    seat.visual(
        seat_mesh,
        origin=Origin(xyz=(0.0, 0.110, 0.030)),
        material=seat_fabric,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.120, 0.050, 0.110)),
        origin=Origin(xyz=(0.0, -0.070, 0.055)),
        material=dark_frame,
        name="rear_bracket",
    )
    back_tilt = math.radians(8.0)
    seat.visual(
        back_frame_mesh,
        origin=Origin(xyz=(0.0, -0.086, 0.102), rpy=(back_tilt, 0.0, 0.0)),
        material=graphite,
        name="back_frame_loop",
    )
    seat.visual(
        Box((0.460, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.126, 0.322), rpy=(back_tilt, 0.0, 0.0)),
        material=graphite,
        name="lumbar_bar",
    )
    seat.visual(
        back_panel_mesh,
        origin=Origin(xyz=(0.0, -0.086, 0.365), rpy=(back_tilt, 0.0, 0.0)),
        material=mesh_fabric,
        name="back_mesh_panel",
    )
    seat.visual(
        back_top_bar_mesh,
        origin=Origin(xyz=(0.0, -0.126, 0.660), rpy=(back_tilt, 0.0, 0.0)),
        material=graphite,
        name="back_top_bar",
    )
    seat.visual(
        Box((0.038, 0.022, 0.085)),
        origin=Origin(xyz=(-0.206, -0.126, 0.618), rpy=(back_tilt, 0.0, 0.0)),
        material=graphite,
        name="back_top_left_connector",
    )
    seat.visual(
        Box((0.038, 0.022, 0.085)),
        origin=Origin(xyz=(0.206, -0.126, 0.618), rpy=(back_tilt, 0.0, 0.0)),
        material=graphite,
        name="back_top_right_connector",
    )
    seat.visual(
        seat_front_trim_mesh,
        origin=Origin(xyz=(0.0, 0.195, 0.054)),
        material=graphite,
        name="seat_front_trim",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = 0.240 * side_sign
        seat.visual(
            Cylinder(radius=0.010, length=0.180),
            origin=Origin(xyz=(x_pos, 0.085, 0.120)),
            material=dark_frame,
            name=f"{side_name}_arm_front_post",
        )
        seat.visual(
            Cylinder(radius=0.010, length=0.180),
            origin=Origin(xyz=(x_pos, -0.075, 0.120)),
            material=dark_frame,
            name=f"{side_name}_arm_rear_post",
        )
        seat.visual(
            Box((0.060, 0.240, 0.035)),
            origin=Origin(xyz=(x_pos, 0.005, 0.2275)),
            material=seat_fabric,
            name=f"{side_name}_arm_pad",
        )
    seat.inertial = Inertial.from_geometry(
        Box((0.620, 0.820, 0.800)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -0.020, 0.340)),
    )

    model.articulation(
        "seat_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, -0.030, 0.440)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(15.0),
        ),
    )

    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        caster_x = wheel_center_radius * math.cos(angle)
        caster_y = wheel_center_radius * math.sin(angle)
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=caster_radius, length=caster_width),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.017, length=0.003),
            origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="rim_inner",
        )
        wheel.visual(
            Cylinder(radius=0.017, length=0.003),
            origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="rim_outer",
        )
        wheel.visual(
            Box((0.004, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.020, 0.010)),
            material=aluminum,
            name="marker",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=caster_radius, length=caster_width),
            mass=0.28,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(
                xyz=(caster_x, caster_y, wheel_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=24.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat_assembly")
    seat_tilt = object_model.get_articulation("seat_tilt")
    top_plate = base.get_visual("top_plate")
    mount_pad = seat.get_visual("mount_pad")
    seat_cushion = seat.get_visual("seat_cushion")
    left_arm_pad = seat.get_visual("left_arm_pad")
    right_arm_pad = seat.get_visual("right_arm_pad")
    seat_front_trim = seat.get_visual("seat_front_trim")
    back_top_bar = seat.get_visual("back_top_bar")
    wheel_parts = [object_model.get_part(f"wheel_{index}") for index in range(5)]
    wheel_joints = [object_model.get_articulation(f"wheel_spin_{index}") for index in range(5)]
    wheel_hubs = [wheel.get_visual("hub") for wheel in wheel_parts]
    wheel_markers = [wheel.get_visual("marker") for wheel in wheel_parts]
    base_fork_lefts = [base.get_visual(f"fork_{index}_left") for index in range(5)]
    base_fork_rights = [base.get_visual(f"fork_{index}_right") for index in range(5)]

    def _center_of(aabb):
        return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))

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
    for index in range(5):
        ctx.allow_overlap(
            base,
            wheel_parts[index],
            elem_a=base_fork_lefts[index],
            elem_b=wheel_hubs[index],
            reason="caster fork cheek captures the wheel hub at the axle seat",
        )
        ctx.allow_overlap(
            base,
            wheel_parts[index],
            elem_a=base_fork_rights[index],
            elem_b=wheel_hubs[index],
            reason="caster fork cheek captures the wheel hub at the axle seat",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("base_exists", base is not None, "missing base part")
    ctx.check("seat_exists", seat is not None, "missing seat assembly part")
    ctx.expect_origin_gap(seat, base, axis="z", min_gap=0.42, max_gap=0.46, name="seat_above_pedestal")
    ctx.expect_contact(
        seat,
        base,
        elem_a=mount_pad,
        elem_b=top_plate,
        contact_tol=0.001,
        name="seat_mount_pad_contacts_top_plate",
    )
    ctx.expect_overlap(
        seat,
        base,
        elem_a=mount_pad,
        elem_b=top_plate,
        axes="xy",
        min_overlap=0.05,
        name="seat_mount_pad_overlaps_top_plate",
    )

    ctx.check(
        "seat_tilt_is_revolute",
        seat_tilt.articulation_type == ArticulationType.REVOLUTE,
        f"expected revolute seat tilt, got {seat_tilt.articulation_type!r}",
    )
    ctx.check(
        "seat_tilt_axis_is_transverse",
        tuple(seat_tilt.axis) == (1.0, 0.0, 0.0),
        f"expected transverse x-axis tilt, got {seat_tilt.axis!r}",
    )
    ctx.check(
        "seat_tilt_range_is_fifteen_degrees",
        seat_tilt.motion_limits is not None
        and abs(seat_tilt.motion_limits.upper - math.radians(15.0)) < 0.02
        and abs(seat_tilt.motion_limits.lower) < 1e-6,
        f"unexpected tilt limits: {seat_tilt.motion_limits!r}",
    )

    seat_cushion_aabb = ctx.part_element_world_aabb(seat, elem=seat_cushion)
    left_arm_aabb = ctx.part_element_world_aabb(seat, elem=left_arm_pad)
    right_arm_aabb = ctx.part_element_world_aabb(seat, elem=right_arm_pad)
    if seat_cushion_aabb and left_arm_aabb and right_arm_aabb:
        seat_center = _center_of(seat_cushion_aabb)
        left_center = _center_of(left_arm_aabb)
        right_center = _center_of(right_arm_aabb)
        ctx.check(
            "armrests_flank_seat",
            left_center[0] < seat_center[0] - 0.18 and right_center[0] > seat_center[0] + 0.18,
            f"arm centers {left_center!r}, {right_center!r} do not flank seat center {seat_center!r}",
        )
        ctx.check(
            "armrests_level",
            abs(left_center[2] - right_center[2]) < 0.01,
            f"arm pad heights differ: {left_center[2]:.4f} vs {right_center[2]:.4f}",
        )
    else:
        ctx.fail("armrest_aabbs_available", "could not measure seat cushion and arm pad visuals")

    rest_front_aabb = ctx.part_element_world_aabb(seat, elem=seat_front_trim)
    rest_back_aabb = ctx.part_element_world_aabb(seat, elem=back_top_bar)
    if rest_front_aabb and rest_back_aabb:
        rest_front_center = _center_of(rest_front_aabb)
        rest_back_center = _center_of(rest_back_aabb)
        with ctx.pose({seat_tilt: math.radians(15.0)}):
            tilt_front_aabb = ctx.part_element_world_aabb(seat, elem=seat_front_trim)
            tilt_back_aabb = ctx.part_element_world_aabb(seat, elem=back_top_bar)
        if tilt_front_aabb and tilt_back_aabb:
            tilt_front_center = _center_of(tilt_front_aabb)
            tilt_back_center = _center_of(tilt_back_aabb)
            ctx.check(
                "tilt_raises_front_edge",
                tilt_front_center[2] > rest_front_center[2] + 0.04,
                f"front edge did not rise enough: rest={rest_front_center!r}, tilt={tilt_front_center!r}",
            )
            ctx.check(
                "tilt_moves_back_rearward",
                tilt_back_center[1] < rest_back_center[1] - 0.10,
                f"back top did not swing rearward enough: rest={rest_back_center!r}, tilt={tilt_back_center!r}",
            )
        else:
            ctx.fail("tilt_feature_measurements_available", "could not measure tilted seat features")
    else:
        ctx.fail("rest_feature_measurements_available", "could not measure seat tilt features at rest")

    for index in range(5):
        wheel = wheel_parts[index]
        joint = wheel_joints[index]
        hub = wheel_hubs[index]
        marker = wheel_markers[index]
        fork_left = base_fork_lefts[index]
        fork_right = base_fork_rights[index]

        ctx.check(
            f"wheel_{index}_exists",
            wheel is not None,
            f"missing wheel_{index}",
        )
        ctx.check(
            f"wheel_{index}_joint_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"expected continuous joint, got {joint.articulation_type!r}",
        )
        ctx.check(
            f"wheel_{index}_axis_is_axle",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"unexpected wheel axis {joint.axis!r}",
        )
        ctx.expect_contact(
            wheel,
            base,
            elem_a=hub,
            elem_b=fork_left,
            contact_tol=0.001,
            name=f"wheel_{index}_hub_contacts_left_fork",
        )
        ctx.expect_contact(
            wheel,
            base,
            elem_a=hub,
            elem_b=fork_right,
            contact_tol=0.001,
            name=f"wheel_{index}_hub_contacts_right_fork",
        )
        ctx.expect_origin_distance(
            wheel,
            base,
            axes="xy",
            min_dist=0.29,
            max_dist=0.32,
            name=f"wheel_{index}_sits_out_on_star_base",
        )
        ctx.expect_origin_gap(
            wheel,
            base,
            axis="z",
            min_gap=0.035,
            max_gap=0.041,
            name=f"wheel_{index}_sits_near_floor",
        )

        rest_marker_aabb = ctx.part_element_world_aabb(wheel, elem=marker)
        if rest_marker_aabb:
            rest_marker_center = _center_of(rest_marker_aabb)
            with ctx.pose({joint: math.pi / 2.0}):
                spun_marker_aabb = ctx.part_element_world_aabb(wheel, elem=marker)
            if spun_marker_aabb:
                spun_marker_center = _center_of(spun_marker_aabb)
                ctx.check(
                    f"wheel_{index}_marker_moves_with_spin",
                    math.dist(rest_marker_center, spun_marker_center) > 0.010,
                    f"wheel marker barely moved: rest={rest_marker_center!r}, spun={spun_marker_center!r}",
                )
            else:
                ctx.fail(f"wheel_{index}_marker_spun_aabb", "could not measure marker in spun pose")
        else:
            ctx.fail(f"wheel_{index}_marker_rest_aabb", "could not measure wheel marker at rest")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
