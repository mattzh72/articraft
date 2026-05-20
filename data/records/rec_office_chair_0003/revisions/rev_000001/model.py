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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_chair", assets=ASSETS)

    polished_aluminum = model.material("polished_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.20, 0.21, 0.23, 1.0))
    charcoal_fabric = model.material("charcoal_fabric", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.09, 0.10, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.12, 0.13, 0.14, 1.0))

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.39),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )
    pedestal.visual(
        Cylinder(radius=0.062, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=polished_aluminum,
        name="base_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.037, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=polished_aluminum,
        name="gas_shroud",
    )
    pedestal.visual(
        Cylinder(radius=0.023, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=polished_aluminum,
        name="column_top",
    )

    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        tip_x = math.cos(angle) * 0.308
        tip_y = math.sin(angle) * 0.308
        pedestal.visual(
            Box((0.220, 0.042, 0.020)),
            origin=Origin(xyz=(math.cos(angle) * 0.164, math.sin(angle) * 0.164, 0.078), rpy=(0.0, 0.0, angle)),
            material=polished_aluminum,
            name=f"star_arm_{index + 1}",
        )
        pedestal.visual(
            Box((0.028, 0.036, 0.008)),
            origin=Origin(xyz=(math.cos(angle) * 0.284, math.sin(angle) * 0.284, 0.065), rpy=(0.0, 0.0, angle)),
            material=polished_aluminum,
            name=f"star_tip_{index + 1}",
        )
        pedestal.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(tip_x, tip_y, 0.075)),
            material=dark_frame,
            name=f"caster_socket_{index + 1}",
        )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.54, 0.50, 0.13)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    seat_cushion_geom = section_loft(
        [
            _xy_section(0.500, 0.455, 0.055, 0.024, y_shift=-0.004),
            _xy_section(0.520, 0.472, 0.078, 0.050, y_shift=0.008),
            _xy_section(0.500, 0.450, 0.100, 0.082, y_shift=0.012),
        ]
    )
    seat.visual(
        _save_mesh("chair_seat_cushion.obj", seat_cushion_geom),
        material=charcoal_fabric,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.265, 0.225, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_frame,
        name="seat_mount_plate",
    )
    seat.visual(
        Box((0.360, 0.315, 0.020)),
        origin=Origin(xyz=(0.0, -0.010, 0.022)),
        material=satin_black,
        name="seat_pan",
    )
    seat.visual(
        Box((0.018, 0.250, 0.050)),
        origin=Origin(xyz=(0.251, -0.018, 0.040)),
        material=dark_frame,
        name="left_arm_mount",
    )
    seat.visual(
        Box((0.018, 0.250, 0.050)),
        origin=Origin(xyz=(-0.251, -0.018, 0.040)),
        material=dark_frame,
        name="right_arm_mount",
    )
    seat.visual(
        Box((0.048, 0.030, 0.090)),
        origin=Origin(xyz=(0.150, -0.215, 0.075)),
        material=dark_frame,
        name="left_hinge_block",
    )
    seat.visual(
        Box((0.048, 0.030, 0.090)),
        origin=Origin(xyz=(-0.150, -0.215, 0.075)),
        material=dark_frame,
        name="right_hinge_block",
    )
    seat.visual(
        Box((0.316, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.215, 0.038)),
        material=dark_frame,
        name="rear_crossmember",
    )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.48, 0.14, 0.63)),
        mass=5.0,
        origin=Origin(xyz=(0.0, -0.040, 0.315)),
    )

    back_cushion_geom = section_loft(
        [
            _xy_section(0.340, 0.055, 0.024, 0.170, y_shift=-0.028),
            _xy_section(0.410, 0.064, 0.030, 0.330, y_shift=-0.040),
            _xy_section(0.455, 0.058, 0.026, 0.560, y_shift=-0.055),
        ]
    )
    backrest.visual(
        _save_mesh("chair_backrest_cushion.obj", back_cushion_geom),
        material=charcoal_fabric,
        name="back_cushion",
    )
    backrest.visual(
        Box((0.032, 0.020, 0.230)),
        origin=Origin(xyz=(0.150, -0.035, 0.115)),
        material=dark_frame,
        name="left_back_bracket",
    )
    backrest.visual(
        Box((0.032, 0.020, 0.230)),
        origin=Origin(xyz=(-0.150, -0.035, 0.115)),
        material=dark_frame,
        name="right_back_bracket",
    )
    backrest.visual(
        Box((0.250, 0.026, 0.045)),
        origin=Origin(xyz=(0.0, -0.038, 0.170)),
        material=dark_frame,
        name="lower_lumbar_bar",
    )
    backrest.visual(
        Box((0.455, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.055, 0.545)),
        material=satin_black,
        name="top_frame",
    )

    left_armrest = model.part("left_armrest")
    left_armrest.inertial = Inertial.from_geometry(
        Box((0.105, 0.315, 0.255)),
        mass=1.2,
        origin=Origin(xyz=(0.052, -0.010, 0.128)),
    )
    left_armrest.visual(
        Box((0.018, 0.080, 0.028)),
        origin=Origin(xyz=(0.009, -0.080, 0.014)),
        material=dark_frame,
        name="mount_block",
    )
    left_armrest.visual(
        Box((0.018, 0.080, 0.028)),
        origin=Origin(xyz=(0.009, 0.060, 0.014)),
        material=dark_frame,
        name="rear_mount_block",
    )
    left_armrest.visual(
        Cylinder(radius=0.014, length=0.185),
        origin=Origin(xyz=(0.018, -0.080, 0.108)),
        material=dark_frame,
        name="front_post",
    )
    left_armrest.visual(
        Cylinder(radius=0.014, length=0.200),
        origin=Origin(xyz=(0.018, 0.055, 0.112)),
        material=dark_frame,
        name="rear_post",
    )
    left_armrest.visual(
        Box((0.090, 0.280, 0.026)),
        origin=Origin(xyz=(0.052, -0.010, 0.210)),
        material=soft_pad,
        name="arm_pad",
    )

    right_armrest = model.part("right_armrest")
    right_armrest.inertial = Inertial.from_geometry(
        Box((0.105, 0.315, 0.255)),
        mass=1.2,
        origin=Origin(xyz=(-0.052, -0.010, 0.128)),
    )
    right_armrest.visual(
        Box((0.018, 0.080, 0.028)),
        origin=Origin(xyz=(-0.009, -0.080, 0.014)),
        material=dark_frame,
        name="mount_block",
    )
    right_armrest.visual(
        Box((0.018, 0.080, 0.028)),
        origin=Origin(xyz=(-0.009, 0.060, 0.014)),
        material=dark_frame,
        name="rear_mount_block",
    )
    right_armrest.visual(
        Cylinder(radius=0.014, length=0.185),
        origin=Origin(xyz=(-0.018, -0.080, 0.108)),
        material=dark_frame,
        name="front_post",
    )
    right_armrest.visual(
        Cylinder(radius=0.014, length=0.200),
        origin=Origin(xyz=(-0.018, 0.055, 0.112)),
        material=dark_frame,
        name="rear_post",
    )
    right_armrest.visual(
        Box((0.090, 0.280, 0.026)),
        origin=Origin(xyz=(-0.052, -0.010, 0.210)),
        material=soft_pad,
        name="arm_pad",
    )

    caster_radius = 0.026
    caster_width = 0.012

    for index in range(5):
        fork = model.part(f"caster_{index + 1}_fork")
        fork.inertial = Inertial.from_geometry(
            Box((0.030, 0.024, 0.060)),
            mass=0.24,
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
        )
        fork.visual(
            Cylinder(radius=0.0055, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=dark_frame,
            name="stem",
        )
        fork.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=satin_black,
            name="swivel_housing",
        )
        fork.visual(
            Box((0.028, 0.018, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=dark_frame,
            name="yoke_bridge",
        )
        fork.visual(
            Box((0.004, 0.018, 0.028)),
            origin=Origin(xyz=(0.012, 0.0, -0.042)),
            material=dark_frame,
            name="left_plate",
        )
        fork.visual(
            Box((0.004, 0.018, 0.028)),
            origin=Origin(xyz=(-0.012, 0.0, -0.042)),
            material=dark_frame,
            name="right_plate",
        )

        wheel = model.part(f"caster_{index + 1}_wheel")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=caster_radius, length=caster_width),
            mass=0.18,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=caster_radius, length=caster_width),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_black,
            name="wheel_tire",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_frame,
            name="hub_barrel",
        )
        wheel.visual(
            Box((0.004, 0.008, 0.008)),
            origin=Origin(xyz=(0.008, 0.0, 0.0)),
            material=dark_frame,
            name="left_axle_cap",
        )
        wheel.visual(
            Box((0.004, 0.008, 0.008)),
            origin=Origin(xyz=(-0.008, 0.0, 0.0)),
            material=dark_frame,
            name="right_axle_cap",
        )
        wheel.visual(
            Box((0.004, 0.004, 0.006)),
            origin=Origin(xyz=(0.004, 0.0, caster_radius - 0.003)),
            material=polished_aluminum,
            name="valve_cap",
        )

    model.articulation(
        "pedestal_to_seat",
        ArticulationType.FIXED,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.205, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(20.0),
        ),
    )
    model.articulation(
        "seat_to_left_armrest",
        ArticulationType.FIXED,
        parent=seat,
        child=left_armrest,
        origin=Origin(xyz=(0.260, -0.018, 0.026)),
    )
    model.articulation(
        "seat_to_right_armrest",
        ArticulationType.FIXED,
        parent=seat,
        child=right_armrest,
        origin=Origin(xyz=(-0.260, -0.018, 0.026)),
    )

    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        tip_x = math.cos(angle) * 0.308
        tip_y = math.sin(angle) * 0.308
        fork = model.get_part(f"caster_{index + 1}_fork")
        wheel = model.get_part(f"caster_{index + 1}_wheel")
        model.articulation(
            f"pedestal_to_caster_{index + 1}_fork",
            ArticulationType.FIXED,
            parent=pedestal,
            child=fork,
            origin=Origin(xyz=(tip_x, tip_y, 0.082), rpy=(0.0, 0.0, angle)),
        )
        model.articulation(
            f"caster_{index + 1}_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.056)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=35.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")
    backrest_recline = object_model.get_articulation("seat_to_backrest")

    column_top = pedestal.get_visual("column_top")
    seat_mount_plate = seat.get_visual("seat_mount_plate")
    left_arm_mount = seat.get_visual("left_arm_mount")
    right_arm_mount = seat.get_visual("right_arm_mount")
    left_hinge_block = seat.get_visual("left_hinge_block")
    right_hinge_block = seat.get_visual("right_hinge_block")
    left_arm_block = left_armrest.get_visual("mount_block")
    right_arm_block = right_armrest.get_visual("mount_block")
    back_cushion = backrest.get_visual("back_cushion")
    left_back_bracket = backrest.get_visual("left_back_bracket")
    right_back_bracket = backrest.get_visual("right_back_bracket")

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
        fork = object_model.get_part(f"caster_{index + 1}_fork")
        socket = pedestal.get_visual(f"caster_socket_{index + 1}")
        stem = fork.get_visual("stem")
        ctx.allow_overlap(
            fork,
            pedestal,
            elem_a=stem,
            elem_b=socket,
            reason="Caster swivel stem is intentionally seated inside the base socket.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        seat,
        pedestal,
        elem_a=seat_mount_plate,
        elem_b=column_top,
        name="seat_mounts_on_column_top",
    )
    ctx.expect_contact(
        left_armrest,
        seat,
        elem_a=left_arm_block,
        elem_b=left_arm_mount,
        name="left_armrest_contacts_seat_mount",
    )
    ctx.expect_contact(
        right_armrest,
        seat,
        elem_a=right_arm_block,
        elem_b=right_arm_mount,
        name="right_armrest_contacts_seat_mount",
    )
    ctx.expect_gap(
        seat,
        backrest,
        axis="y",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem=left_hinge_block,
        negative_elem=left_back_bracket,
        name="left_backrest_bracket_sits_just_behind_left_hinge_block",
    )
    ctx.expect_gap(
        seat,
        backrest,
        axis="y",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem=right_hinge_block,
        negative_elem=right_back_bracket,
        name="right_backrest_bracket_sits_just_behind_right_hinge_block",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="xz",
        min_overlap=0.020,
        elem_a=left_back_bracket,
        elem_b=left_hinge_block,
        name="left_backrest_bracket_aligned_with_left_hinge_block",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="xz",
        min_overlap=0.020,
        elem_a=right_back_bracket,
        elem_b=right_hinge_block,
        name="right_backrest_bracket_aligned_with_right_hinge_block",
    )
    ctx.expect_origin_distance(
        left_armrest,
        right_armrest,
        axes="x",
        min_dist=0.50,
        max_dist=0.66,
        name="armrest_span_matches_office_chair_width",
    )

    seat_aabb = ctx.part_world_aabb(seat)
    back_aabb = ctx.part_world_aabb(backrest)
    left_arm_aabb = ctx.part_world_aabb(left_armrest)
    right_arm_aabb = ctx.part_world_aabb(right_armrest)
    ctx.check(
        "chair_has_expected_seat_height",
        seat_aabb is not None and 0.35 <= seat_aabb[0][2] <= 0.39 and 0.47 <= seat_aabb[1][2] <= 0.50,
        details=f"seat_aabb={seat_aabb}",
    )
    ctx.check(
        "backrest_rises_above_seat",
        seat_aabb is not None and back_aabb is not None and back_aabb[1][2] > seat_aabb[1][2] + 0.50,
        details=f"seat_aabb={seat_aabb}, back_aabb={back_aabb}",
    )
    ctx.check(
        "armrests_flank_seat",
        seat_aabb is not None
        and left_arm_aabb is not None
        and right_arm_aabb is not None
        and left_arm_aabb[0][0] > seat_aabb[1][0] - 0.01
        and right_arm_aabb[1][0] < seat_aabb[0][0] + 0.01,
        details=f"seat_aabb={seat_aabb}, left_arm_aabb={left_arm_aabb}, right_arm_aabb={right_arm_aabb}",
    )

    rest_back_aabb = ctx.part_element_world_aabb(backrest, elem=back_cushion)
    with ctx.pose({backrest_recline: math.radians(20.0)}):
        reclined_back_aabb = ctx.part_element_world_aabb(backrest, elem=back_cushion)
        ctx.check(
            "backrest_reclines_rearward",
            rest_back_aabb is not None
            and reclined_back_aabb is not None
            and reclined_back_aabb[1][1] < rest_back_aabb[1][1] - 0.05,
            details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
        )
        ctx.check(
            "backrest_top_drops_slightly_when_reclined",
            rest_back_aabb is not None
            and reclined_back_aabb is not None
            and reclined_back_aabb[1][2] < rest_back_aabb[1][2] - 0.01,
            details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
        )

    for index in range(5):
        fork = object_model.get_part(f"caster_{index + 1}_fork")
        wheel = object_model.get_part(f"caster_{index + 1}_wheel")
        spin = object_model.get_articulation(f"caster_{index + 1}_spin")
        socket = pedestal.get_visual(f"caster_socket_{index + 1}")
        stem = fork.get_visual("stem")
        left_plate = fork.get_visual("left_plate")
        right_plate = fork.get_visual("right_plate")
        left_axle_cap = wheel.get_visual("left_axle_cap")
        right_axle_cap = wheel.get_visual("right_axle_cap")
        valve_cap = wheel.get_visual("valve_cap")

        ctx.expect_contact(
            fork,
            pedestal,
            elem_a=stem,
            elem_b=socket,
            name=f"caster_{index + 1}_fork_stem_seats_in_base_socket",
        )
        ctx.expect_contact(
            wheel,
            fork,
            elem_a=left_axle_cap,
            elem_b=left_plate,
            name=f"caster_{index + 1}_left_axle_cap_contacts_fork_plate",
        )
        ctx.expect_contact(
            wheel,
            fork,
            elem_a=right_axle_cap,
            elem_b=right_plate,
            name=f"caster_{index + 1}_right_axle_cap_contacts_fork_plate",
        )

        wheel_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster_{index + 1}_wheel_reaches_floor_plane",
            wheel_aabb is not None and -0.003 <= wheel_aabb[0][2] <= 0.003,
            details=f"wheel_aabb={wheel_aabb}",
        )

        rest_valve_aabb = ctx.part_element_world_aabb(wheel, elem=valve_cap)
        with ctx.pose({spin: math.pi / 2.0}):
            spun_valve_aabb = ctx.part_element_world_aabb(wheel, elem=valve_cap)
            ctx.check(
                f"caster_{index + 1}_spin_moves_valve_cap_off_top_dead_center",
                rest_valve_aabb is not None
                and spun_valve_aabb is not None
                and spun_valve_aabb[1][2] < rest_valve_aabb[1][2] - 0.015,
                details=f"rest={rest_valve_aabb}, spun={spun_valve_aabb}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
