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
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_swivel_bar_stool", assets=ASSETS)

    powder_coat = model.material("powder_coat", rgba=(0.24, 0.25, 0.27, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    molded_poly = model.material("molded_poly", rgba=(0.18, 0.18, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    zinc = model.material("zinc", rgba=(0.76, 0.78, 0.80, 1.0))
    warning_gray = model.material("warning_gray", rgba=(0.42, 0.44, 0.46, 1.0))

    foot_ring_mesh = _save_mesh(
        "foot_ring.obj",
        TorusGeometry(radius=0.165, tube=0.015, radial_segments=18, tubular_segments=56),
    )
    support_ring_mesh = _save_mesh(
        "support_ring.obj",
        TorusGeometry(radius=0.145, tube=0.006, radial_segments=16, tubular_segments=52),
    )
    seat_shell_mesh = _save_mesh(
        "seat_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.100, 0.000),
                (0.165, 0.006),
                (0.208, 0.018),
                (0.220, 0.032),
                (0.216, 0.042),
                (0.185, 0.055),
                (0.105, 0.063),
                (0.0, 0.066),
            ],
            segments=72,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.240, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_coat,
        name="base_plate",
    )
    pedestal_base.visual(
        Cylinder(radius=0.205, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber,
        name="base_pad",
    )
    for xyz, size, name, material in (
        ((0.086, 0.0, 0.070), (0.062, 0.050, 0.060), "lower_collar_pos_x", dark_steel),
        ((-0.086, 0.0, 0.070), (0.062, 0.050, 0.060), "lower_collar_neg_x", dark_steel),
        ((0.0, 0.086, 0.070), (0.050, 0.062, 0.060), "lower_collar_pos_y", dark_steel),
        ((0.0, -0.086, 0.070), (0.050, 0.062, 0.060), "lower_collar_neg_y", dark_steel),
        ((0.069, 0.0, 0.255), (0.028, 0.044, 0.390), "guide_rib_pos_x", powder_coat),
        ((-0.069, 0.0, 0.255), (0.028, 0.044, 0.390), "guide_rib_neg_x", powder_coat),
        ((0.0, 0.069, 0.255), (0.044, 0.028, 0.390), "guide_rib_pos_y", powder_coat),
        ((0.0, -0.069, 0.255), (0.044, 0.028, 0.390), "guide_rib_neg_y", powder_coat),
        ((0.078, 0.0, 0.455), (0.044, 0.050, 0.050), "upper_collar_pos_x", dark_steel),
        ((-0.078, 0.0, 0.455), (0.044, 0.050, 0.050), "upper_collar_neg_x", dark_steel),
        ((0.0, 0.078, 0.455), (0.050, 0.044, 0.050), "upper_collar_pos_y", dark_steel),
        ((0.0, -0.078, 0.455), (0.050, 0.044, 0.050), "upper_collar_neg_y", dark_steel),
    ):
        pedestal_base.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )
    pedestal_base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        material=warning_gray,
        name="foot_ring",
    )
    for axis, offset, name in (
        ((0.123, 0.0, 0.320), (0.120, 0.024, 0.028), "foot_ring_strut_pos_x"),
        ((-0.123, 0.0, 0.320), (0.120, 0.024, 0.028), "foot_ring_strut_neg_x"),
        ((0.0, 0.123, 0.320), (0.024, 0.120, 0.028), "foot_ring_strut_pos_y"),
        ((0.0, -0.123, 0.320), (0.024, 0.120, 0.028), "foot_ring_strut_neg_y"),
    ):
        pedestal_base.visual(
            Box(offset[1] if False else offset),
            origin=Origin(xyz=axis),
            material=powder_coat,
            name=name,
        )
    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        pedestal_base.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(
                xyz=(
                    math.cos(angle) * 0.182,
                    math.sin(angle) * 0.182,
                    0.044,
                )
            ),
            material=zinc,
            name=f"anchor_bolt_{index:02d}",
        )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.240, length=0.480),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.055, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        material=dark_steel,
        name="lift_tube",
    )
    upper_column.visual(
        Cylinder(radius=0.074, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=powder_coat,
        name="bearing_shroud",
    )
    upper_column.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.599)),
        material=dark_steel,
        name="swivel_lower_race",
    )
    for index in range(4):
        angle = (2.0 * math.pi * index) / 4.0 + (math.pi / 4.0)
        upper_column.visual(
            Cylinder(radius=0.007, length=0.018),
            origin=Origin(
                xyz=(
                    math.cos(angle) * 0.073,
                    math.sin(angle) * 0.073,
                    0.599,
                )
            ),
            material=zinc,
            name=f"swivel_bolt_{index:02d}",
        )
    upper_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.610),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
    )

    seat_carriage = model.part("seat_carriage")
    seat_carriage.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="swivel_upper_race",
    )
    seat_carriage.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_coat,
        name="center_hub",
    )
    seat_carriage.visual(
        Cylinder(radius=0.130, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=powder_coat,
        name="support_plate",
    )
    seat_carriage.visual(
        support_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=warning_gray,
        name="support_ring",
    )
    for xyz, size, name in (
        ((0.075, 0.0, 0.026), (0.150, 0.020, 0.032), "gusset_pos_x"),
        ((-0.075, 0.0, 0.026), (0.150, 0.020, 0.032), "gusset_neg_x"),
        ((0.0, 0.075, 0.026), (0.020, 0.150, 0.032), "gusset_pos_y"),
        ((0.0, -0.075, 0.026), (0.020, 0.150, 0.032), "gusset_neg_y"),
    ):
        seat_carriage.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=powder_coat,
            name=name,
        )
    seat_carriage.visual(
        Box((0.020, 0.004, 0.025)),
        origin=Origin(xyz=(0.106, -0.039, 0.020)),
        material=dark_steel,
        name="lever_bracket_left",
    )
    seat_carriage.visual(
        Box((0.020, 0.004, 0.025)),
        origin=Origin(xyz=(0.106, -0.057, 0.020)),
        material=dark_steel,
        name="lever_bracket_right",
    )
    for index in range(4):
        angle = (2.0 * math.pi * index) / 4.0 + (math.pi / 4.0)
        seat_carriage.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(
                xyz=(
                    math.cos(angle) * 0.095,
                    math.sin(angle) * 0.095,
                    0.018,
                )
            ),
            material=zinc,
            name=f"mount_bolt_{index:02d}",
        )
    seat_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.052),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.135, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_steel,
        name="seat_mount_pan",
    )
    seat.visual(
        Cylinder(radius=0.170, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="seat_backing_plate",
    )
    seat.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=molded_poly,
        name="seat_shell",
    )
    for index in range(4):
        angle = (2.0 * math.pi * index) / 4.0 + (math.pi / 4.0)
        seat.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(
                xyz=(
                    math.cos(angle) * 0.118,
                    math.sin(angle) * 0.118,
                    0.018,
                )
            ),
            material=zinc,
            name=f"seat_service_screw_{index:02d}",
        )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.220, length=0.101),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
    )

    adjustment_lever = model.part("adjustment_lever")
    adjustment_lever.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="pivot_barrel",
    )
    adjustment_lever.visual(
        Box((0.095, 0.012, 0.010)),
        origin=Origin(xyz=(0.048, 0.0, -0.020)),
        material=warning_gray,
        name="lever_arm",
    )
    adjustment_lever.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.095, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=molded_poly,
        name="lever_grip",
    )
    adjustment_lever.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.010)),
        material=dark_steel,
        name="actuator_tab",
    )
    adjustment_lever.inertial = Inertial.from_geometry(
        Box((0.110, 0.020, 0.045)),
        mass=0.15,
        origin=Origin(xyz=(0.050, 0.0, -0.015)),
    )

    model.articulation(
        "height_adjustment",
        ArticulationType.PRISMATIC,
        parent=pedestal_base,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.12,
            lower=0.0,
            upper=0.12,
        ),
    )
    model.articulation(
        "swivel_rotation",
        ArticulationType.CONTINUOUS,
        parent=upper_column,
        child=seat_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.608)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=4.0,
        ),
    )
    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=seat_carriage,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat_carriage,
        child=adjustment_lever,
        origin=Origin(xyz=(0.106, -0.048, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal_base = object_model.get_part("pedestal_base")
    upper_column = object_model.get_part("upper_column")
    seat_carriage = object_model.get_part("seat_carriage")
    seat = object_model.get_part("seat")
    adjustment_lever = object_model.get_part("adjustment_lever")

    height_adjustment = object_model.get_articulation("height_adjustment")
    swivel_rotation = object_model.get_articulation("swivel_rotation")
    lever_pivot = object_model.get_articulation("lever_pivot")

    guide_rib = pedestal_base.get_visual("guide_rib_pos_x")
    lift_tube = upper_column.get_visual("lift_tube")
    lower_race = upper_column.get_visual("swivel_lower_race")
    upper_race = seat_carriage.get_visual("swivel_upper_race")
    support_plate = seat_carriage.get_visual("support_plate")
    seat_mount_pan = seat.get_visual("seat_mount_pan")
    pivot_barrel = adjustment_lever.get_visual("pivot_barrel")
    lever_grip = adjustment_lever.get_visual("lever_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=8)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(upper_column, pedestal_base, elem_a=lift_tube, elem_b=guide_rib, contact_tol=0.001)
    ctx.expect_contact(seat_carriage, upper_column, elem_a=upper_race, elem_b=lower_race)
    ctx.expect_gap(
        seat_carriage,
        upper_column,
        axis="z",
        positive_elem=upper_race,
        negative_elem=lower_race,
        max_gap=0.0005,
        max_penetration=0.0,
    )
    ctx.expect_contact(seat, seat_carriage, elem_a=seat_mount_pan, elem_b=support_plate)
    ctx.expect_gap(
        seat,
        seat_carriage,
        axis="z",
        positive_elem=seat_mount_pan,
        negative_elem=support_plate,
        max_gap=0.0005,
        max_penetration=0.0,
    )
    ctx.expect_contact(adjustment_lever, seat_carriage, elem_a=pivot_barrel)

    seat_low_aabb = ctx.part_world_aabb(seat)
    base_aabb = ctx.part_world_aabb(pedestal_base)
    assert seat_low_aabb is not None
    assert base_aabb is not None
    low_seat_top = seat_low_aabb[1][2]
    seat_diameter = seat_low_aabb[1][0] - seat_low_aabb[0][0]
    ctx.check(
        "seat_top_height_low",
        0.72 <= low_seat_top <= 0.78,
        f"expected low seat top in 0.72-0.78 m, got {low_seat_top:.3f} m",
    )
    ctx.check(
        "seat_diameter_realistic",
        0.40 <= seat_diameter <= 0.46,
        f"expected seat diameter in 0.40-0.46 m, got {seat_diameter:.3f} m",
    )
    ctx.check(
        "seat_over_base_radius",
        seat_low_aabb[1][0] <= base_aabb[1][0] + 0.01 and seat_low_aabb[0][0] >= base_aabb[0][0] - 0.01,
        "seat footprint should stay above the weighted pedestal base footprint",
    )

    lever_rest = ctx.part_world_position(adjustment_lever)
    assert lever_rest is not None
    lever_rest_radius = math.hypot(lever_rest[0], lever_rest[1])

    with ctx.pose({height_adjustment: 0.12}):
        ctx.fail_if_parts_overlap_in_current_pose(name="height_adjustment_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="height_adjustment_upper_no_floating")
        ctx.expect_contact(upper_column, pedestal_base, elem_a=lift_tube, elem_b=guide_rib, contact_tol=0.001)
        seat_high_aabb = ctx.part_world_aabb(seat)
        assert seat_high_aabb is not None
        high_seat_top = seat_high_aabb[1][2]
        ctx.check(
            "seat_top_height_high",
            0.84 <= high_seat_top <= 0.90,
            f"expected high seat top in 0.84-0.90 m, got {high_seat_top:.3f} m",
        )
        ctx.check(
            "seat_height_travel",
            high_seat_top - low_seat_top >= 0.10,
            f"expected at least 0.10 m of lift travel, got {high_seat_top - low_seat_top:.3f} m",
        )

    with ctx.pose({swivel_rotation: math.pi / 2.0}):
        swivel_quarter = ctx.part_world_position(adjustment_lever)
        assert swivel_quarter is not None
        ctx.check(
            "swivel_preserves_lever_radius",
            abs(math.hypot(swivel_quarter[0], swivel_quarter[1]) - lever_rest_radius) <= 0.01,
            "lever should orbit around the column at a nearly constant radius when the seat swivels",
        )
        ctx.check(
            "swivel_rotates_lever_azimuth",
            abs(swivel_quarter[0] + lever_rest[1]) <= 0.02 and abs(swivel_quarter[1] - lever_rest[0]) <= 0.02,
            "seat swivel should rotate the adjustment lever roughly 90 degrees around the column",
        )
        ctx.expect_contact(seat_carriage, upper_column, elem_a=upper_race, elem_b=lower_race)

    lever_rest_aabb = ctx.part_element_world_aabb(adjustment_lever, elem=lever_grip)
    assert lever_rest_aabb is not None
    with ctx.pose({lever_pivot: 0.26}):
        lever_lifted_aabb = ctx.part_element_world_aabb(adjustment_lever, elem=lever_grip)
        assert lever_lifted_aabb is not None
        ctx.check(
            "lever_lifts_when_pulled",
            lever_lifted_aabb[1][2] >= lever_rest_aabb[1][2] + 0.020,
            "lever grip should visibly rise through its pivot motion",
        )
        ctx.expect_contact(adjustment_lever, seat_carriage, elem_a=pivot_barrel)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
