from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_sliding_window")

    frame_paint = model.material("frame_paint", rgba=(0.37, 0.35, 0.31, 1.0))
    adapter_steel = model.material("adapter_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    hardware = model.material("hardware", rgba=(0.52, 0.54, 0.57, 1.0))
    gasket = model.material("gasket", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.67, 0.80, 0.88, 0.28))

    frame_width = 1.48
    frame_height = 1.18
    frame_depth = 0.16
    jamb_width = 0.10
    head_height = 0.095
    sill_height = 0.11
    inner_width = frame_width - 2.0 * jamb_width
    frame_bottom = -frame_height / 2.0
    frame_top = frame_height / 2.0
    opening_bottom = frame_bottom + sill_height
    opening_top = frame_top - head_height

    front_track_y = 0.030
    rear_track_y = -0.033

    def add_bolt(
        part,
        *,
        center: tuple[float, float, float],
        axis: str,
        sign: float,
        plate_thickness: float,
        material,
        prefix: str,
        shank_radius: float = 0.0035,
        shank_length: float = 0.010,
        head_radius: float = 0.0075,
        head_length: float = 0.003,
    ) -> None:
        x, y, z = center
        if axis == "x":
            rpy = (0.0, math.pi / 2.0, 0.0)
            shank_xyz = (x + sign * 0.001, y, z)
            head_xyz = (x + sign * (plate_thickness / 2.0 + head_length / 2.0), y, z)
        elif axis == "y":
            rpy = (math.pi / 2.0, 0.0, 0.0)
            shank_xyz = (x, y + sign * 0.001, z)
            head_xyz = (x, y + sign * (plate_thickness / 2.0 + head_length / 2.0), z)
        else:
            rpy = (0.0, 0.0, 0.0)
            shank_xyz = (x, y, z + sign * 0.001)
            head_xyz = (x, y, z + sign * (plate_thickness / 2.0 + head_length / 2.0))

        part.visual(
            Cylinder(radius=shank_radius, length=shank_length),
            origin=Origin(xyz=shank_xyz, rpy=rpy),
            material=hardware,
            name=f"{prefix}_stud",
        )
        part.visual(
            Cylinder(radius=head_radius, length=head_length),
            origin=Origin(xyz=head_xyz, rpy=rpy),
            material=material,
            name=f"{prefix}_head",
        )

    frame = model.part("frame_assembly")
    frame.visual(
        Box((jamb_width, frame_depth, frame_height)),
        origin=Origin(xyz=(-frame_width / 2.0 + jamb_width / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((jamb_width, frame_depth, frame_height)),
        origin=Origin(xyz=(frame_width / 2.0 - jamb_width / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((inner_width, frame_depth, head_height)),
        origin=Origin(xyz=(0.0, 0.0, frame_top - head_height / 2.0)),
        material=frame_paint,
        name="head",
    )
    frame.visual(
        Box((inner_width, frame_depth, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, frame_bottom + sill_height / 2.0)),
        material=frame_paint,
        name="sill",
    )

    frame.visual(
        Box((inner_width, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, front_track_y, -0.487)),
        material=frame_paint,
        name="front_track_floor",
    )
    frame.visual(
        Box((inner_width, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.005, -0.476)),
        material=frame_paint,
        name="front_track_rear_lip",
    )
    frame.visual(
        Box((inner_width, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.055, -0.476)),
        material=frame_paint,
        name="front_track_front_lip",
    )
    frame.visual(
        Box((inner_width, 0.052, 0.020)),
        origin=Origin(xyz=(0.0, front_track_y, 0.489)),
        material=frame_paint,
        name="front_head_track",
    )
    frame.visual(
        Box((inner_width, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.005, 0.479)),
        material=frame_paint,
        name="front_head_rear_lip",
    )
    frame.visual(
        Box((inner_width, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.055, 0.479)),
        material=frame_paint,
        name="front_head_front_lip",
    )

    frame.visual(
        Box((inner_width, 0.044, 0.020)),
        origin=Origin(xyz=(0.0, rear_track_y, -0.482)),
        material=frame_paint,
        name="rear_sill_seat",
    )
    frame.visual(
        Box((inner_width, 0.044, 0.020)),
        origin=Origin(xyz=(0.0, rear_track_y, 0.482)),
        material=frame_paint,
        name="rear_head_clamp",
    )
    frame.visual(
        Box((0.020, 0.044, opening_top - opening_bottom)),
        origin=Origin(xyz=(-0.635, rear_track_y, 0.0075)),
        material=frame_paint,
        name="rear_left_setting_leg",
    )
    frame.visual(
        Box((0.020, 0.044, opening_top - opening_bottom)),
        origin=Origin(xyz=(0.635, rear_track_y, 0.0075)),
        material=frame_paint,
        name="rear_right_setting_leg",
    )

    frame.visual(
        Box((0.040, 0.018, 0.090)),
        origin=Origin(xyz=(0.640, front_track_y, 0.435)),
        material=hardware,
        name="close_stop_upper",
    )
    frame.visual(
        Box((0.040, 0.018, 0.090)),
        origin=Origin(xyz=(0.640, front_track_y, -0.435)),
        material=hardware,
        name="close_stop_lower",
    )
    frame.visual(
        Box((0.060, 0.018, 0.120)),
        origin=Origin(xyz=(-0.610, front_track_y, 0.330)),
        material=hardware,
        name="open_stop_upper",
    )
    frame.visual(
        Box((0.060, 0.018, 0.120)),
        origin=Origin(xyz=(-0.610, front_track_y, -0.330)),
        material=hardware,
        name="open_stop_lower",
    )

    frame.visual(
        Box((0.012, 0.100, 0.900)),
        origin=Origin(xyz=(-frame_width / 2.0 - 0.006, 0.0, -0.010)),
        material=adapter_steel,
        name="left_adapter_plate",
    )
    for index, z in enumerate((-0.380, -0.120, 0.140, 0.400)):
        add_bolt(
            frame,
            center=(-frame_width / 2.0 - 0.006, -0.030, z),
            axis="x",
            sign=-1.0,
            plate_thickness=0.012,
            material=adapter_steel,
            prefix=f"left_adapter_inner_{index}",
        )
        add_bolt(
            frame,
            center=(-frame_width / 2.0 - 0.006, 0.030, z),
            axis="x",
            sign=-1.0,
            plate_thickness=0.012,
            material=adapter_steel,
            prefix=f"left_adapter_outer_{index}",
        )

    frame.visual(
        Box((0.012, 0.100, 0.900)),
        origin=Origin(xyz=(frame_width / 2.0 + 0.006, 0.0, -0.010)),
        material=adapter_steel,
        name="right_adapter_plate",
    )
    for index, z in enumerate((-0.380, -0.120, 0.140, 0.400)):
        add_bolt(
            frame,
            center=(frame_width / 2.0 + 0.006, -0.030, z),
            axis="x",
            sign=1.0,
            plate_thickness=0.012,
            material=adapter_steel,
            prefix=f"right_adapter_inner_{index}",
        )
        add_bolt(
            frame,
            center=(frame_width / 2.0 + 0.006, 0.030, z),
            axis="x",
            sign=1.0,
            plate_thickness=0.012,
            material=adapter_steel,
            prefix=f"right_adapter_outer_{index}",
        )

    frame.visual(
        Box((1.200, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, frame_top + 0.006)),
        material=adapter_steel,
        name="head_adapter_plate",
    )
    for index, x in enumerate((-0.500, -0.180, 0.180, 0.500)):
        add_bolt(
            frame,
            center=(x, -0.028, frame_top + 0.006),
            axis="z",
            sign=1.0,
            plate_thickness=0.012,
            material=adapter_steel,
            prefix=f"head_adapter_left_{index}",
        )
        add_bolt(
            frame,
            center=(x, 0.028, frame_top + 0.006),
            axis="z",
            sign=1.0,
            plate_thickness=0.012,
            material=adapter_steel,
            prefix=f"head_adapter_right_{index}",
        )

    frame.visual(
        Box((0.120, 0.006, 0.120)),
        origin=Origin(xyz=(-0.580, 0.083, 0.435)),
        material=adapter_steel,
        name="upper_left_fishplate",
    )
    frame.visual(
        Box((0.120, 0.006, 0.120)),
        origin=Origin(xyz=(0.580, 0.083, 0.435)),
        material=adapter_steel,
        name="upper_right_fishplate",
    )
    frame.visual(
        Box((0.120, 0.006, 0.110)),
        origin=Origin(xyz=(-0.580, 0.083, -0.435)),
        material=adapter_steel,
        name="lower_left_fishplate",
    )
    frame.visual(
        Box((0.120, 0.006, 0.110)),
        origin=Origin(xyz=(0.580, 0.083, -0.435)),
        material=adapter_steel,
        name="lower_right_fishplate",
    )
    for prefix, x, z in (
        ("ulf", -0.580, 0.435),
        ("urf", 0.580, 0.435),
        ("llf", -0.580, -0.435),
        ("lrf", 0.580, -0.435),
    ):
        for index, (dx, dz) in enumerate(((-0.030, -0.030), (0.030, -0.030), (-0.030, 0.030), (0.030, 0.030))):
            add_bolt(
                frame,
                center=(x + dx, 0.083, z + dz),
                axis="y",
                sign=1.0,
                plate_thickness=0.006,
                material=adapter_steel,
                prefix=f"{prefix}_bolt_{index}",
            )

    frame.visual(
        Box((0.170, 0.006, 0.135)),
        origin=Origin(xyz=(0.645, 0.083, -0.325)),
        material=adapter_steel,
        name="right_service_hatch",
    )
    for index, (dx, dz) in enumerate(((-0.060, -0.045), (0.060, -0.045), (-0.060, 0.045), (0.060, 0.045))):
        add_bolt(
            frame,
            center=(0.645 + dx, 0.083, -0.325 + dz),
            axis="y",
            sign=1.0,
            plate_thickness=0.006,
            material=adapter_steel,
            prefix=f"right_service_bolt_{index}",
        )

    frame.visual(
        Box((0.250, 0.006, 0.095)),
        origin=Origin(xyz=(0.220, 0.083, -0.535)),
        material=adapter_steel,
        name="sill_service_hatch",
    )
    for index, (dx, dz) in enumerate(((-0.090, -0.028), (0.090, -0.028), (-0.090, 0.028), (0.090, 0.028))):
        add_bolt(
            frame,
            center=(0.220 + dx, 0.083, -0.535 + dz),
            axis="y",
            sign=1.0,
            plate_thickness=0.006,
            material=adapter_steel,
            prefix=f"sill_service_bolt_{index}",
        )

    frame.visual(
        Box((0.220, 0.012, 0.020)),
        origin=Origin(xyz=(-0.320, 0.074, -0.530)),
        material=gasket,
        name="weep_cover_left",
    )
    frame.visual(
        Box((0.220, 0.012, 0.020)),
        origin=Origin(xyz=(0.000, 0.074, -0.530)),
        material=gasket,
        name="weep_cover_center",
    )
    frame.visual(
        Box((0.220, 0.012, 0.020)),
        origin=Origin(xyz=(0.320, 0.074, -0.530)),
        material=gasket,
        name="weep_cover_right",
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=48.0,
    )

    fixed_lite = model.part("fixed_lite")
    fixed_lite.visual(
        Box((0.046, 0.030, 0.944)),
        origin=Origin(xyz=(-0.327, 0.0, 0.0)),
        material=frame_paint,
        name="left_stile",
    )
    fixed_lite.visual(
        Box((0.046, 0.030, 0.944)),
        origin=Origin(xyz=(0.327, 0.0, 0.0)),
        material=frame_paint,
        name="right_stile",
    )
    fixed_lite.visual(
        Box((0.654, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.447)),
        material=frame_paint,
        name="top_rail",
    )
    fixed_lite.visual(
        Box((0.654, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.447)),
        material=frame_paint,
        name="bottom_rail",
    )
    fixed_lite.visual(
        Box((0.618, 0.010, 0.856)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="fixed_glass",
    )
    fixed_lite.visual(
        Box((0.018, 0.012, 0.860)),
        origin=Origin(xyz=(0.328, 0.012, 0.0)),
        material=gasket,
        name="meeting_interlock",
    )
    fixed_lite.visual(
        Box((0.610, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, 0.428)),
        material=gasket,
        name="top_glazing_bead",
    )
    fixed_lite.visual(
        Box((0.610, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, -0.428)),
        material=gasket,
        name="bottom_glazing_bead",
    )
    fixed_lite.inertial = Inertial.from_geometry(
        Box((0.700, 0.030, 0.944)),
        mass=12.0,
    )

    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        Box((0.050, 0.034, 0.932)),
        origin=Origin(xyz=(-0.325, 0.0, 0.0)),
        material=frame_paint,
        name="left_stile",
    )
    sliding_sash.visual(
        Box((0.050, 0.034, 0.932)),
        origin=Origin(xyz=(0.325, 0.0, 0.0)),
        material=frame_paint,
        name="right_stile",
    )
    sliding_sash.visual(
        Box((0.650, 0.034, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=frame_paint,
        name="top_rail",
    )
    sliding_sash.visual(
        Box((0.650, 0.034, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.440)),
        material=frame_paint,
        name="bottom_rail",
    )
    sliding_sash.visual(
        Box((0.612, 0.010, 0.840)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="sliding_glass",
    )
    sliding_sash.visual(
        Box((0.140, 0.024, 0.020)),
        origin=Origin(xyz=(-0.180, 0.0, -0.468)),
        material=hardware,
        name="bottom_left_shoe",
    )
    sliding_sash.visual(
        Box((0.140, 0.024, 0.020)),
        origin=Origin(xyz=(0.180, 0.0, -0.468)),
        material=hardware,
        name="bottom_right_shoe",
    )
    sliding_sash.visual(
        Box((0.140, 0.020, 0.020)),
        origin=Origin(xyz=(-0.180, 0.0, 0.468)),
        material=hardware,
        name="top_left_guide",
    )
    sliding_sash.visual(
        Box((0.140, 0.020, 0.020)),
        origin=Origin(xyz=(0.180, 0.0, 0.468)),
        material=hardware,
        name="top_right_guide",
    )
    sliding_sash.visual(
        Box((0.018, 0.020, 0.260)),
        origin=Origin(xyz=(0.296, 0.017, 0.000)),
        material=hardware,
        name="pull_handle",
    )
    sliding_sash.visual(
        Box((0.012, 0.014, 0.820)),
        origin=Origin(xyz=(-0.331, -0.010, 0.0)),
        material=gasket,
        name="brush_seal",
    )
    sliding_sash.visual(
        Box((0.620, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.013, -0.420)),
        material=gasket,
        name="bottom_weatherstrip",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((0.700, 0.034, 0.972)),
        mass=14.0,
    )

    model.articulation(
        "frame_to_fixed_lite",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_lite,
        origin=Origin(xyz=(-0.270, rear_track_y, 0.0)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.270, front_track_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.55,
            lower=0.0,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_assembly")
    fixed_lite = object_model.get_part("fixed_lite")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_slide = object_model.get_articulation("frame_to_sliding_sash")

    frame_track_floor = frame.get_visual("front_track_floor")
    frame_head_track = frame.get_visual("front_head_track")
    rear_sill_seat = frame.get_visual("rear_sill_seat")
    rear_head_clamp = frame.get_visual("rear_head_clamp")
    close_stop = frame.get_visual("close_stop_upper")
    open_stop = frame.get_visual("open_stop_upper")

    fixed_bottom_rail = fixed_lite.get_visual("bottom_rail")
    fixed_top_rail = fixed_lite.get_visual("top_rail")
    sash_left_stile = sliding_sash.get_visual("left_stile")
    sash_right_stile = sliding_sash.get_visual("right_stile")
    sash_bottom_left_shoe = sliding_sash.get_visual("bottom_left_shoe")
    sash_top_left_guide = sliding_sash.get_visual("top_left_guide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.fail_if_parts_overlap_in_current_pose()

    frame_visual_names = {visual.name for visual in frame.visuals}
    ctx.check(
        "retrofit_details_present",
        {
            "left_adapter_plate",
            "right_adapter_plate",
            "head_adapter_plate",
            "right_service_hatch",
            "sill_service_hatch",
            "upper_left_fishplate",
            "upper_right_fishplate",
        }.issubset(frame_visual_names),
        "Missing required retrofit adapters, hatches, or reinforcement plates.",
    )
    limits = sash_slide.motion_limits
    ctx.check(
        "sash_joint_axis_and_limits",
        tuple(sash_slide.axis) == (-1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.46 <= limits.upper <= 0.52,
        "Sliding sash should move left in a bounded horizontal channel with realistic travel.",
    )

    ctx.expect_contact(
        fixed_lite,
        frame,
        elem_a=fixed_bottom_rail,
        elem_b=rear_sill_seat,
        name="fixed_lite_seated_on_rear_sill",
    )
    ctx.expect_contact(
        fixed_lite,
        frame,
        elem_a=fixed_top_rail,
        elem_b=rear_head_clamp,
        name="fixed_lite_captured_under_rear_head_clamp",
    )

    with ctx.pose({sash_slide: 0.0}):
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a=sash_bottom_left_shoe,
            elem_b=frame_track_floor,
            name="closed_sash_bears_on_track_floor",
        )
        ctx.expect_within(
            sliding_sash,
            frame,
            axes="y",
            inner_elem=sash_bottom_left_shoe,
            outer_elem=frame_track_floor,
            margin=0.0,
            name="closed_sash_shoe_stays_in_track_width",
        )
        ctx.expect_gap(
            frame,
            sliding_sash,
            axis="z",
            positive_elem=frame_head_track,
            negative_elem=sash_top_left_guide,
            min_gap=0.0005,
            max_gap=0.0025,
            name="closed_sash_top_guide_clearance",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_lite,
            axis="y",
            min_gap=0.020,
            max_gap=0.040,
            name="closed_sash_runs_ahead_of_fixed_lite",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="xz",
            min_overlap=0.140,
            name="closed_sash_keeps_meeting_overlap",
        )
        ctx.expect_contact(
            frame,
            sliding_sash,
            elem_a=close_stop,
            elem_b=sash_right_stile,
            name="closed_sash_hits_close_stop",
        )

    with ctx.pose({sash_slide: limits.upper if limits is not None and limits.upper is not None else 0.50}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_has_no_unintended_overlaps")
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a=sash_bottom_left_shoe,
            elem_b=frame_track_floor,
            name="open_sash_still_bears_on_track_floor",
        )
        ctx.expect_gap(
            frame,
            sliding_sash,
            axis="z",
            positive_elem=frame_head_track,
            negative_elem=sash_top_left_guide,
            min_gap=0.0005,
            max_gap=0.0025,
            name="open_sash_top_guide_clearance",
        )
        ctx.expect_contact(
            frame,
            sliding_sash,
            elem_a=open_stop,
            elem_b=sash_left_stile,
            name="open_sash_hits_serviceable_open_stop",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="xz",
            min_overlap=0.100,
            name="open_sash_stays_in_capture_overlap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
