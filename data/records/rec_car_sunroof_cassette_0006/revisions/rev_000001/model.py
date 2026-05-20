from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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


def _panel_mesh(
    name: str,
    *,
    width: float,
    length: float,
    thickness: float,
    radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, length, radius, corner_segments=10),
            thickness,
            center=True,
        ),
        name,
    )


def _ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_length: float,
    inner_width: float,
    inner_length: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_width, outer_length, outer_radius, corner_segments=10),
            [rounded_rect_profile(inner_width, inner_length, inner_radius, corner_segments=10)],
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_sunroof_cassette")

    cassette_black = model.material("cassette_black", rgba=(0.12, 0.13, 0.14, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.60, 0.62, 0.66, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass_border = model.material("glass_border", rgba=(0.07, 0.08, 0.10, 0.92))
    glass_tint = model.material("glass_tint", rgba=(0.31, 0.44, 0.52, 0.42))

    outer_width = 1.20
    outer_length = 1.82
    cassette_height = 0.081
    opening_width = 1.02
    opening_length = 1.52
    lower_opening_width = 0.90
    lower_opening_length = 1.34

    front_panel_length = 0.43
    rear_panel_length = 0.86
    panel_width = 0.99
    pane_thickness = 0.006

    top_bezel_mesh = _ring_mesh(
        "sunroof_top_bezel",
        outer_width=outer_width,
        outer_length=outer_length,
        inner_width=opening_width,
        inner_length=opening_length,
        height=0.014,
        outer_radius=0.080,
        inner_radius=0.055,
    )
    lower_tray_mesh = _ring_mesh(
        "sunroof_lower_tray",
        outer_width=outer_width,
        outer_length=outer_length,
        inner_width=lower_opening_width,
        inner_length=lower_opening_length,
        height=0.067,
        outer_radius=0.076,
        inner_radius=0.040,
    )
    front_outer_pane_mesh = _panel_mesh(
        "front_tilt_outer_pane",
        width=panel_width,
        length=front_panel_length,
        thickness=pane_thickness,
        radius=0.046,
    )
    front_inner_lite_mesh = _panel_mesh(
        "front_tilt_inner_lite",
        width=panel_width - 0.060,
        length=front_panel_length - 0.060,
        thickness=0.004,
        radius=0.034,
    )
    rear_outer_pane_mesh = _panel_mesh(
        "rear_slide_outer_pane",
        width=panel_width,
        length=rear_panel_length,
        thickness=pane_thickness,
        radius=0.050,
    )
    rear_inner_lite_mesh = _panel_mesh(
        "rear_slide_inner_lite",
        width=panel_width - 0.060,
        length=rear_panel_length - 0.060,
        thickness=0.004,
        radius=0.038,
    )

    cassette_frame = model.part("cassette_frame")
    cassette_frame.visual(
        Box((0.100, outer_length, 0.060)),
        origin=Origin(xyz=(-0.550, 0.0, 0.030)),
        material=tray_dark,
        name="left_tray_wall",
    )
    cassette_frame.visual(
        Box((0.100, outer_length, 0.060)),
        origin=Origin(xyz=(0.550, 0.0, 0.030)),
        material=tray_dark,
        name="right_tray_wall",
    )
    cassette_frame.visual(
        Box((1.000, 0.160, 0.060)),
        origin=Origin(xyz=(0.0, -0.830, 0.030)),
        material=tray_dark,
        name="front_header_tray",
    )
    cassette_frame.visual(
        Box((1.060, 0.240, 0.060)),
        origin=Origin(xyz=(0.0, 0.790, 0.030)),
        material=tray_dark,
        name="rear_housing_tray",
    )
    cassette_frame.visual(
        Box((0.042, 1.560, 0.010)),
        origin=Origin(xyz=(-0.499, 0.020, 0.064)),
        material=rail_aluminum,
        name="left_rail",
    )
    cassette_frame.visual(
        Box((0.042, 1.560, 0.010)),
        origin=Origin(xyz=(0.499, 0.020, 0.064)),
        material=rail_aluminum,
        name="right_rail",
    )
    cassette_frame.visual(
        Box((0.900, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.725, 0.063)),
        material=rail_aluminum,
        name="front_hinge_beam",
    )
    cassette_frame.visual(
        Box((0.780, 0.220, 0.010)),
        origin=Origin(xyz=(0.0, 0.790, 0.063)),
        material=cassette_black,
        name="rear_stow_deck",
    )
    cassette_frame.inertial = Inertial.from_geometry(
        Box((outer_width, outer_length, cassette_height)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, cassette_height * 0.5)),
    )

    front_tilt_section = model.part("front_tilt_section")
    front_tilt_section.visual(
        front_outer_pane_mesh,
        origin=Origin(xyz=(0.0, front_panel_length * 0.5, 0.021)),
        material=glass_border,
        name="outer_pane",
    )
    front_tilt_section.visual(
        front_inner_lite_mesh,
        origin=Origin(xyz=(0.0, front_panel_length * 0.5, 0.022)),
        material=glass_tint,
        name="inner_lite",
    )
    front_tilt_section.visual(
        Box((0.880, 0.080, 0.010)),
        origin=Origin(xyz=(0.0, 0.070, 0.015)),
        material=gasket_black,
        name="carrier_plate",
    )
    front_tilt_section.visual(
        Box((0.120, 0.042, 0.010)),
        origin=Origin(xyz=(-0.340, 0.080, 0.017)),
        material=gasket_black,
        name="hinge_shoe_left",
    )
    front_tilt_section.visual(
        Box((0.120, 0.042, 0.010)),
        origin=Origin(xyz=(0.340, 0.080, 0.017)),
        material=gasket_black,
        name="hinge_shoe_right",
    )
    front_tilt_section.visual(
        Box((0.160, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, front_panel_length - 0.048, 0.013)),
        material=gasket_black,
        name="rear_lift_pad",
    )
    front_tilt_section.inertial = Inertial.from_geometry(
        Box((panel_width, front_panel_length, 0.032)),
        mass=7.0,
        origin=Origin(xyz=(0.0, front_panel_length * 0.52, 0.016)),
    )

    rear_slide_section = model.part("rear_slide_section")
    rear_slide_section.visual(
        rear_outer_pane_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=glass_border,
        name="outer_pane",
    )
    rear_slide_section.visual(
        rear_inner_lite_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=glass_tint,
        name="inner_lite",
    )
    rear_slide_section.visual(
        Box((0.045, rear_panel_length - 0.100, 0.012)),
        origin=Origin(xyz=(-0.415, 0.0, 0.013)),
        material=gasket_black,
        name="left_carrier",
    )
    rear_slide_section.visual(
        Box((0.045, rear_panel_length - 0.100, 0.012)),
        origin=Origin(xyz=(0.415, 0.0, 0.013)),
        material=gasket_black,
        name="right_carrier",
    )
    rear_slide_section.visual(
        Box((0.030, 0.120, 0.010)),
        origin=Origin(xyz=(-0.475, -0.220, 0.017)),
        material=rail_aluminum,
        name="left_front_carriage",
    )
    rear_slide_section.visual(
        Box((0.030, 0.120, 0.010)),
        origin=Origin(xyz=(0.475, -0.220, 0.017)),
        material=rail_aluminum,
        name="right_front_carriage",
    )
    rear_slide_section.visual(
        Box((0.030, 0.120, 0.010)),
        origin=Origin(xyz=(-0.475, 0.220, 0.017)),
        material=rail_aluminum,
        name="left_rear_carriage",
    )
    rear_slide_section.visual(
        Box((0.030, 0.120, 0.010)),
        origin=Origin(xyz=(0.475, 0.220, 0.017)),
        material=rail_aluminum,
        name="right_rear_carriage",
    )
    rear_slide_section.inertial = Inertial.from_geometry(
        Box((panel_width, rear_panel_length, 0.034)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    model.articulation(
        "front_tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette_frame,
        child=front_tilt_section,
        origin=Origin(xyz=(0.0, -0.760, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.6,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "rear_slide_rails",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=rear_slide_section,
        origin=Origin(xyz=(0.0, 0.160, 0.057)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.40,
            lower=0.0,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette_frame = object_model.get_part("cassette_frame")
    front_tilt_section = object_model.get_part("front_tilt_section")
    rear_slide_section = object_model.get_part("rear_slide_section")
    front_tilt_hinge = object_model.get_articulation("front_tilt_hinge")
    rear_slide_rails = object_model.get_articulation("rear_slide_rails")

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

    front_limits = front_tilt_hinge.motion_limits
    rear_limits = rear_slide_rails.motion_limits

    ctx.check(
        "front hinge runs across vehicle width",
        tuple(front_tilt_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected front tilt hinge axis (1, 0, 0), got {front_tilt_hinge.axis}",
    )
    ctx.check(
        "rear slide runs fore-aft on rails",
        tuple(rear_slide_rails.axis) == (0.0, 1.0, 0.0),
        f"expected rear slide axis (0, 1, 0), got {rear_slide_rails.axis}",
    )
    ctx.check(
        "front hinge has realistic tilt range",
        front_limits is not None
        and front_limits.lower == 0.0
        and front_limits.upper is not None
        and 0.35 <= front_limits.upper <= 0.50,
        f"unexpected front hinge limits: {front_limits}",
    )
    ctx.check(
        "rear slide has realistic travel range",
        rear_limits is not None
        and rear_limits.lower == 0.0
        and rear_limits.upper is not None
        and 0.22 <= rear_limits.upper <= 0.34,
        f"unexpected rear slide limits: {rear_limits}",
    )

    ctx.expect_origin_distance(
        front_tilt_section,
        cassette_frame,
        axes="x",
        max_dist=0.001,
        name="front tilt section centered in cassette",
    )
    ctx.expect_origin_distance(
        rear_slide_section,
        cassette_frame,
        axes="x",
        max_dist=0.001,
        name="rear slide section centered in cassette",
    )
    ctx.expect_contact(
        front_tilt_section,
        cassette_frame,
        elem_a="hinge_shoe_left",
        elem_b="front_hinge_beam",
        name="left hinge shoe seats on hinge beam",
    )
    ctx.expect_contact(
        front_tilt_section,
        cassette_frame,
        elem_a="hinge_shoe_right",
        elem_b="front_hinge_beam",
        name="right hinge shoe seats on hinge beam",
    )
    ctx.expect_contact(
        rear_slide_section,
        cassette_frame,
        elem_a="left_front_carriage",
        elem_b="left_rail",
        name="left front carriage rides on left rail",
    )
    ctx.expect_contact(
        rear_slide_section,
        cassette_frame,
        elem_a="right_front_carriage",
        elem_b="right_rail",
        name="right front carriage rides on right rail",
    )
    ctx.expect_gap(
        rear_slide_section,
        front_tilt_section,
        axis="y",
        positive_elem="outer_pane",
        negative_elem="outer_pane",
        min_gap=0.045,
        max_gap=0.075,
        name="closed seam gap between tilt and slide panes",
    )

    with ctx.pose({front_tilt_hinge: 0.40}):
        ctx.expect_gap(
            front_tilt_section,
            rear_slide_section,
            axis="z",
            positive_elem="rear_lift_pad",
            negative_elem="outer_pane",
            min_gap=0.090,
            name="front tilt section lifts clearly at its trailing edge",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at full front tilt")

    with ctx.pose({rear_slide_rails: 0.28}):
        ctx.expect_contact(
            rear_slide_section,
            cassette_frame,
            elem_a="left_rear_carriage",
            elem_b="left_rail",
            name="left rear carriage stays on rail at full slide",
        )
        ctx.expect_contact(
            rear_slide_section,
            cassette_frame,
            elem_a="right_rear_carriage",
            elem_b="right_rail",
            name="right rear carriage stays on rail at full slide",
        )
        ctx.expect_gap(
            rear_slide_section,
            front_tilt_section,
            axis="y",
            positive_elem="outer_pane",
            negative_elem="outer_pane",
            min_gap=0.300,
            name="rear slide section retracts aft of the tilt pane",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at full rear slide")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
