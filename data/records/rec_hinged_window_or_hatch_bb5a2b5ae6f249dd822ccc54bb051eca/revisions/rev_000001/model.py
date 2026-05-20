from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_ring(
    outer_x: float,
    outer_y: float,
    outer_radius: float,
    inner_x: float,
    inner_y: float,
    inner_radius: float,
    thickness: float,
):
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_x, outer_y, outer_radius, corner_segments=10),
        [rounded_rect_profile(inner_x, inner_y, inner_radius, corner_segments=10)],
        height=thickness,
        center=True,
    )


def _add_latch_clevis(part, prefix: str, x_pos: float, y_pos: float, z_pos: float, material) -> None:
    part.visual(
        Box((0.065, 0.020, 0.008)),
        origin=Origin(xyz=(x_pos, y_pos, z_pos + 0.004)),
        material=material,
        name=f"{prefix}_clevis_base",
    )
    for side_name, dx in (("left", -0.028), ("right", 0.028)):
        part.visual(
            Box((0.011, 0.014, 0.018)),
            origin=Origin(xyz=(x_pos + dx, y_pos, z_pos - 0.001)),
            material=material,
            name=f"{prefix}_clevis_{side_name}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gullwing_service_hatch")

    body_paint = model.material("body_paint", rgba=(0.70, 0.72, 0.74, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    latch_steel = model.material("latch_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    roof_x = 1.18
    roof_y = 0.86
    body_depth = 0.22

    opening_x = 0.776
    opening_y = 0.516
    lip_inner_x = 0.680
    lip_inner_y = 0.420

    frame_outer_x = 0.772
    frame_outer_y = 0.512
    frame_inner_x = 0.684
    frame_inner_y = 0.424
    frame_height = 0.028

    lid_x = 0.820
    lid_y = 0.580
    lid_skin_t = 0.018
    hinge_y = -frame_outer_y * 0.5
    opening_center_local_y = -hinge_y
    hinge_barrel_radius = 0.011

    front_x = -0.235
    rear_x = 0.235
    latch_pivot_y = 0.548
    latch_pivot_z = -0.002

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((roof_x, roof_y, body_depth)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -body_depth * 0.5)),
    )
    body.visual(
        _mesh(
            "body_roof_panel",
            _rounded_ring(
                roof_x,
                roof_y,
                0.060,
                opening_x,
                opening_y,
                0.032,
                0.006,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=body_paint,
        name="roof_panel",
    )
    body.visual(
        _mesh(
            "body_opening_lip",
            _rounded_ring(
                opening_x,
                opening_y,
                0.032,
                lip_inner_x,
                lip_inner_y,
                0.024,
                0.012,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=hinge_steel,
        name="opening_lip",
    )
    body.visual(
        Box((roof_x - 0.04, 0.060, body_depth)),
        origin=Origin(xyz=(0.0, -0.400, -body_depth * 0.5)),
        material=body_paint,
        name="hinge_side_sill",
    )
    body.visual(
        Box((roof_x - 0.04, 0.060, body_depth)),
        origin=Origin(xyz=(0.0, 0.400, -body_depth * 0.5)),
        material=body_paint,
        name="free_side_sill",
    )
    body.visual(
        Box((0.060, roof_y - 0.12, body_depth)),
        origin=Origin(xyz=(-0.560, 0.0, -body_depth * 0.5)),
        material=body_paint,
        name="front_end_sill",
    )
    body.visual(
        Box((0.060, roof_y - 0.12, body_depth)),
        origin=Origin(xyz=(0.560, 0.0, -body_depth * 0.5)),
        material=body_paint,
        name="rear_end_sill",
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_outer_x, frame_outer_y, 0.050)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )
    rail_w = (frame_outer_y - frame_inner_y) * 0.5
    rail_x = (frame_outer_x - frame_inner_x) * 0.5
    frame.visual(
        Box((frame_outer_x, rail_w, frame_height)),
        origin=Origin(xyz=(0.0, hinge_y + rail_w * 0.5, frame_height * 0.5)),
        material=frame_paint,
        name="hinge_rail",
    )
    frame.visual(
        Box((frame_outer_x, rail_w, frame_height)),
        origin=Origin(xyz=(0.0, frame_inner_y * 0.5 + rail_w * 0.5, frame_height * 0.5)),
        material=frame_paint,
        name="free_rail",
    )
    frame.visual(
        Box((rail_x, frame_inner_y, frame_height)),
        origin=Origin(xyz=(-frame_inner_x * 0.5 - rail_x * 0.5, 0.0, frame_height * 0.5)),
        material=frame_paint,
        name="front_rail",
    )
    frame.visual(
        Box((rail_x, frame_inner_y, frame_height)),
        origin=Origin(xyz=(frame_inner_x * 0.5 + rail_x * 0.5, 0.0, frame_height * 0.5)),
        material=frame_paint,
        name="rear_rail",
    )
    frame.visual(
        _mesh(
            "frame_seal_ring",
            _rounded_ring(
                0.736,
                0.476,
                0.022,
                0.694,
                0.434,
                0.016,
                0.004,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=gasket_rubber,
        name="seal_ring",
    )
    for name, x_pos, length in (
        ("hinge_knuckle_front", -0.2875, 0.145),
        ("hinge_knuckle_center", 0.0, 0.135),
        ("hinge_knuckle_rear", 0.2875, 0.145),
    ):
        frame.visual(
            Box((length, 0.032, 0.010)),
            origin=Origin(xyz=(x_pos, hinge_y + hinge_barrel_radius + 0.016, 0.012)),
            material=hinge_steel,
            name=f"{name}_leaf",
        )
        frame.visual(
            Cylinder(radius=hinge_barrel_radius, length=length),
            origin=Origin(xyz=(x_pos, hinge_y, frame_height), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_steel,
            name=name,
        )
    frame.visual(
        Box((0.072, 0.028, 0.012)),
        origin=Origin(xyz=(front_x, 0.270, 0.010)),
        material=hinge_steel,
        name="front_striker",
    )
    frame.visual(
        Box((0.072, 0.028, 0.012)),
        origin=Origin(xyz=(rear_x, 0.270, 0.010)),
        material=hinge_steel,
        name="rear_striker",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_x, lid_y, 0.070)),
        mass=11.5,
        origin=Origin(xyz=(0.0, lid_y * 0.5, 0.004)),
    )
    lid.visual(
        _mesh(
            "lid_outer_skin",
            ExtrudeGeometry(
                rounded_rect_profile(lid_x, lid_y, 0.030, corner_segments=10),
                height=lid_skin_t,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, lid_y * 0.5 + 0.018, lid_skin_t * 0.5)),
        material=body_paint,
        name="outer_skin",
    )
    lid.visual(
        Box((0.580, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.368, -0.006)),
        material=frame_paint,
        name="inner_reinforcement_rear",
    )
    lid.visual(
        Box((0.028, 0.300, 0.012)),
        origin=Origin(xyz=(-0.276, 0.228, -0.006)),
        material=frame_paint,
        name="inner_reinforcement_front",
    )
    lid.visual(
        Box((0.028, 0.300, 0.012)),
        origin=Origin(xyz=(0.276, 0.228, -0.006)),
        material=frame_paint,
        name="inner_reinforcement_rear_side",
    )
    lid.visual(
        Box((0.690, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.456, -0.009)),
        material=frame_paint,
        name="free_edge_stiffener",
    )
    lid.visual(
        Box((0.210, 0.022, 0.014)),
        origin=Origin(xyz=(-0.145, 0.105, -0.007)),
        material=frame_paint,
        name="front_hinge_backer",
    )
    lid.visual(
        Box((0.210, 0.022, 0.014)),
        origin=Origin(xyz=(0.145, 0.105, -0.007)),
        material=frame_paint,
        name="rear_hinge_backer",
    )
    lid.visual(
        Box((0.028, 0.340, 0.016)),
        origin=Origin(xyz=(-0.292, opening_center_local_y, -0.008)),
        material=frame_paint,
        name="front_side_rib",
    )
    lid.visual(
        Box((0.028, 0.340, 0.016)),
        origin=Origin(xyz=(0.292, opening_center_local_y, -0.008)),
        material=frame_paint,
        name="rear_side_rib",
    )
    for name, x_pos in (("front_hinge_strap", -0.1425), ("rear_hinge_strap", 0.1425)):
        lid.visual(
            Box((0.165, 0.046, 0.010)),
            origin=Origin(xyz=(x_pos, hinge_barrel_radius + 0.023, 0.007)),
            material=hinge_steel,
            name=name,
        )
    _add_latch_clevis(lid, "front", front_x, latch_pivot_y, -0.008, hinge_steel)
    _add_latch_clevis(lid, "rear", rear_x, latch_pivot_y, -0.008, hinge_steel)

    front_latch = model.part("front_latch_dog")
    front_latch.inertial = Inertial.from_geometry(
        Box((0.045, 0.030, 0.040)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.002, -0.014)),
    )
    front_latch.visual(
        Cylinder(radius=0.006, length=0.013),
        origin=Origin(xyz=(-0.016, 0.0, -0.009), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_steel,
        name="left_trunnion",
    )
    front_latch.visual(
        Cylinder(radius=0.006, length=0.013),
        origin=Origin(xyz=(0.016, 0.0, -0.009), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_steel,
        name="right_trunnion",
    )
    front_latch.visual(
        Box((0.040, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=latch_steel,
        name="dog_arm",
    )
    front_latch.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.001, -0.024)),
        material=latch_steel,
        name="dog_cam",
    )

    rear_latch = model.part("rear_latch_dog")
    rear_latch.inertial = Inertial.from_geometry(
        Box((0.045, 0.030, 0.040)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.002, -0.014)),
    )
    rear_latch.visual(
        Cylinder(radius=0.006, length=0.013),
        origin=Origin(xyz=(-0.016, 0.0, -0.009), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_steel,
        name="left_trunnion",
    )
    rear_latch.visual(
        Cylinder(radius=0.006, length=0.013),
        origin=Origin(xyz=(0.016, 0.0, -0.009), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_steel,
        name="right_trunnion",
    )
    rear_latch.visual(
        Box((0.040, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=latch_steel,
        name="dog_arm",
    )
    rear_latch.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.001, -0.024)),
        material=latch_steel,
        name="dog_cam",
    )

    model.articulation(
        "body_to_frame",
        ArticulationType.FIXED,
        parent=body,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, frame_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "front_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=front_latch,
        origin=Origin(xyz=(front_x, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=0.0, upper=1.0),
    )
    model.articulation(
        "rear_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=rear_latch,
        origin=Origin(xyz=(rear_x, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    frame = object_model.get_part("frame")
    lid = object_model.get_part("lid")
    front_latch = object_model.get_part("front_latch_dog")
    rear_latch = object_model.get_part("rear_latch_dog")

    hatch_hinge = object_model.get_articulation("hatch_hinge")
    front_latch_pivot = object_model.get_articulation("front_latch_pivot")
    rear_latch_pivot = object_model.get_articulation("rear_latch_pivot")

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

    ctx.check(
        "hinge_axis_is_longitudinal",
        hatch_hinge.axis == (1.0, 0.0, 0.0),
        details=f"expected longitudinal +x hinge axis, got {hatch_hinge.axis}",
    )
    ctx.check(
        "latch_axes_match_hinge_axis",
        front_latch_pivot.axis == (1.0, 0.0, 0.0) and rear_latch_pivot.axis == (1.0, 0.0, 0.0),
        details=(
            "expected both latch pivots to rotate on local longitudinal axes; "
            f"got front={front_latch_pivot.axis}, rear={rear_latch_pivot.axis}"
        ),
    )
    ctx.check(
        "hinge_open_limit_reads_as_gullwing",
        hatch_hinge.motion_limits is not None and hatch_hinge.motion_limits.upper is not None and hatch_hinge.motion_limits.upper >= 1.2,
        details="lid hinge should open to a clear gullwing angle of at least 1.2 rad",
    )

    ctx.expect_contact(frame, body, name="frame_supported_by_body")

    with ctx.pose({hatch_hinge: 0.0, front_latch_pivot: 0.0, rear_latch_pivot: 0.0}):
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            positive_elem="outer_skin",
            negative_elem="seal_ring",
            max_gap=0.0005,
            max_penetration=0.0,
            name="lid_skin_seats_on_seal",
        )
        ctx.expect_overlap(
            lid,
            frame,
            axes="xy",
            elem_a="outer_skin",
            elem_b="seal_ring",
            min_overlap=0.45,
            name="lid_covers_opening",
        )
        ctx.expect_contact(front_latch, lid, name="front_latch_mounted_to_lid")
        ctx.expect_contact(rear_latch, lid, name="rear_latch_mounted_to_lid")
        ctx.expect_contact(
            front_latch,
            frame,
            elem_b="front_striker",
            name="front_latch_engages_striker",
        )
        ctx.expect_contact(
            rear_latch,
            frame,
            elem_b="rear_striker",
            name="rear_latch_engages_striker",
        )

    with ctx.pose({hatch_hinge: 1.2, front_latch_pivot: 0.0, rear_latch_pivot: 0.0}):
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            positive_elem="free_edge_stiffener",
            negative_elem="seal_ring",
            min_gap=0.25,
            name="free_edge_lifts_clear_when_open",
        )

    with ctx.pose({front_latch_pivot: 1.0, rear_latch_pivot: 1.0}):
        ctx.expect_gap(
            front_latch,
            frame,
            axis="y",
            negative_elem="front_striker",
            min_gap=0.008,
            name="front_latch_releases_from_striker",
        )
        ctx.expect_gap(
            rear_latch,
            frame,
            axis="y",
            negative_elem="rear_striker",
            min_gap=0.008,
            name="rear_latch_releases_from_striker",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
