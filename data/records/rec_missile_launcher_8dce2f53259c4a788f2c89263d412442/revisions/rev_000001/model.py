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
    model = ArticulatedObject(name="naval_box_missile_launcher")

    deck_gray = model.material("deck_gray", rgba=(0.48, 0.52, 0.55, 1.0))
    naval_gray = model.material("naval_gray", rgba=(0.62, 0.66, 0.69, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.22, 0.25, 1.0))
    canister_green = model.material("canister_green", rgba=(0.35, 0.40, 0.31, 1.0))
    blast_cover_gray = model.material("blast_cover_gray", rgba=(0.67, 0.70, 0.72, 1.0))

    base = model.part("deck_base")
    base.visual(
        Box((3.80, 3.20, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=deck_gray,
        name="foundation_plate",
    )
    base.visual(
        Box((2.70, 1.50, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=dark_gray,
        name="mounting_skid",
    )
    base.visual(
        Cylinder(radius=0.78, length=1.70),
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
        material=naval_gray,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.98, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 2.11)),
        material=dark_gray,
        name="azimuth_bearing_housing",
    )
    base.visual(
        Box((1.15, 0.92, 0.82)),
        origin=Origin(xyz=(-0.96, 0.0, 0.63)),
        material=naval_gray,
        name="drive_access_box",
    )
    base.visual(
        Box((0.95, 0.30, 0.30)),
        origin=Origin(xyz=(-0.35, 0.0, 1.68)),
        material=dark_gray,
        name="column_rear_spine",
    )
    base.inertial = Inertial.from_geometry(
        Box((3.80, 3.20, 2.30)),
        mass=7800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.90, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_gray,
        name="turntable_drum",
    )
    pedestal.visual(
        Box((2.20, 1.90, 0.22)),
        origin=Origin(xyz=(0.10, 0.0, 0.35)),
        material=naval_gray,
        name="rotating_deck",
    )
    pedestal.visual(
        Box((1.10, 0.90, 0.70)),
        origin=Origin(xyz=(-0.48, 0.0, 0.72)),
        material=dark_gray,
        name="slew_drive_pack",
    )
    pedestal.visual(
        Box((0.24, 0.22, 1.35)),
        origin=Origin(xyz=(0.05, 1.02, 0.92)),
        material=naval_gray,
        name="left_yoke_stanchion",
    )
    pedestal.visual(
        Box((0.24, 0.22, 1.35)),
        origin=Origin(xyz=(0.05, -1.02, 0.92)),
        material=naval_gray,
        name="right_yoke_stanchion",
    )
    pedestal.visual(
        Box((0.92, 0.18, 0.18)),
        origin=Origin(xyz=(0.30, 1.02, 0.83), rpy=(0.0, -0.78, 0.0)),
        material=naval_gray,
        name="left_yoke_brace",
    )
    pedestal.visual(
        Box((0.92, 0.18, 0.18)),
        origin=Origin(xyz=(0.30, -1.02, 0.83), rpy=(0.0, -0.78, 0.0)),
        material=naval_gray,
        name="right_yoke_brace",
    )
    pedestal.visual(
        Box((0.28, 1.86, 0.24)),
        origin=Origin(xyz=(-0.10, 0.0, 1.14)),
        material=dark_gray,
        name="yoke_crossmember",
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.18),
        origin=Origin(xyz=(0.58, 1.01, 1.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="left_trunnion_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.18),
        origin=Origin(xyz=(0.58, -1.01, 1.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="right_trunnion_collar",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((2.20, 1.90, 1.65)),
        mass=3200.0,
        origin=Origin(xyz=(0.08, 0.0, 0.86)),
    )

    frame = model.part("canister_frame")
    frame.visual(
        Cylinder(radius=0.16, length=1.84),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="trunnion_shaft",
    )
    frame.visual(
        Box((0.26, 1.58, 0.28)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material=dark_gray,
        name="trunnion_web",
    )
    frame.visual(
        Box((0.12, 1.62, 1.35)),
        origin=Origin(xyz=(-0.12, 0.0, 0.055)),
        material=naval_gray,
        name="rear_bulkhead",
    )
    frame.visual(
        Box((2.10, 0.08, 1.35)),
        origin=Origin(xyz=(0.93, 0.77, 0.055)),
        material=naval_gray,
        name="port_side_wall",
    )
    frame.visual(
        Box((2.10, 0.08, 1.35)),
        origin=Origin(xyz=(0.93, -0.77, 0.055)),
        material=naval_gray,
        name="starboard_side_wall",
    )
    frame.visual(
        Box((2.10, 1.46, 0.08)),
        origin=Origin(xyz=(0.93, 0.0, 0.69)),
        material=naval_gray,
        name="roof_panel",
    )
    frame.visual(
        Box((2.10, 1.46, 0.08)),
        origin=Origin(xyz=(0.93, 0.0, -0.58)),
        material=naval_gray,
        name="floor_panel",
    )
    frame.visual(
        Box((0.10, 1.62, 0.10)),
        origin=Origin(xyz=(2.03, 0.0, 0.655)),
        material=dark_gray,
        name="front_top_lip",
    )
    frame.visual(
        Box((0.10, 1.62, 0.10)),
        origin=Origin(xyz=(2.03, 0.0, -0.545)),
        material=dark_gray,
        name="front_bottom_lip",
    )
    frame.visual(
        Box((0.10, 0.10, 1.25)),
        origin=Origin(xyz=(2.03, 0.76, 0.055)),
        material=dark_gray,
        name="front_port_post",
    )
    frame.visual(
        Box((0.10, 0.10, 1.25)),
        origin=Origin(xyz=(2.03, -0.76, 0.055)),
        material=dark_gray,
        name="front_starboard_post",
    )
    frame.visual(
        Box((0.10, 0.08, 1.25)),
        origin=Origin(xyz=(2.03, 0.0, 0.055)),
        material=dark_gray,
        name="center_mullion",
    )
    frame.visual(
        Box((0.42, 1.62, 0.12)),
        origin=Origin(xyz=(0.15, 0.0, -0.50)),
        material=dark_gray,
        name="lower_recoil_beam",
    )
    for row_index, z_pos in enumerate((0.31, -0.25)):
        for col_index, y_pos in enumerate((0.54, 0.18, -0.18, -0.54)):
            frame.visual(
                Box((1.92, 0.30, 0.44)),
                origin=Origin(xyz=(0.94, y_pos, z_pos)),
                material=canister_green,
                name=f"canister_{row_index}_{col_index}",
            )
    frame.inertial = Inertial.from_geometry(
        Box((2.20, 1.62, 1.35)),
        mass=4100.0,
        origin=Origin(xyz=(0.94, 0.0, 0.055)),
    )

    left_cover = model.part("left_blast_cover")
    left_cover.visual(
        Box((0.04, 0.73, 1.23)),
        origin=Origin(xyz=(0.02, -0.365, 0.0)),
        material=blast_cover_gray,
        name="outer_panel",
    )
    left_cover.visual(
        Box((0.05, 0.59, 0.08)),
        origin=Origin(xyz=(0.065, -0.365, 0.40)),
        material=dark_gray,
        name="upper_stiffener",
    )
    left_cover.visual(
        Box((0.05, 0.59, 0.08)),
        origin=Origin(xyz=(0.065, -0.365, -0.40)),
        material=dark_gray,
        name="lower_stiffener",
    )
    left_cover.visual(
        Cylinder(radius=0.035, length=0.22),
        origin=Origin(xyz=(0.025, 0.0, 0.40)),
        material=dark_gray,
        name="upper_hinge_knuckle",
    )
    left_cover.visual(
        Cylinder(radius=0.035, length=0.22),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=dark_gray,
        name="center_hinge_knuckle",
    )
    left_cover.visual(
        Cylinder(radius=0.035, length=0.22),
        origin=Origin(xyz=(0.025, 0.0, -0.40)),
        material=dark_gray,
        name="lower_hinge_knuckle",
    )
    left_cover.inertial = Inertial.from_geometry(
        Box((0.10, 0.73, 1.23)),
        mass=120.0,
        origin=Origin(xyz=(0.05, -0.365, 0.0)),
    )

    right_cover = model.part("right_blast_cover")
    right_cover.visual(
        Box((0.04, 0.73, 1.23)),
        origin=Origin(xyz=(0.02, 0.365, 0.0)),
        material=blast_cover_gray,
        name="outer_panel",
    )
    right_cover.visual(
        Box((0.05, 0.59, 0.08)),
        origin=Origin(xyz=(0.065, 0.365, 0.40)),
        material=dark_gray,
        name="upper_stiffener",
    )
    right_cover.visual(
        Box((0.05, 0.59, 0.08)),
        origin=Origin(xyz=(0.065, 0.365, -0.40)),
        material=dark_gray,
        name="lower_stiffener",
    )
    right_cover.visual(
        Cylinder(radius=0.035, length=0.22),
        origin=Origin(xyz=(0.025, 0.0, 0.40)),
        material=dark_gray,
        name="upper_hinge_knuckle",
    )
    right_cover.visual(
        Cylinder(radius=0.035, length=0.22),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=dark_gray,
        name="center_hinge_knuckle",
    )
    right_cover.visual(
        Cylinder(radius=0.035, length=0.22),
        origin=Origin(xyz=(0.025, 0.0, -0.40)),
        material=dark_gray,
        name="lower_hinge_knuckle",
    )
    right_cover.inertial = Inertial.from_geometry(
        Box((0.10, 0.73, 1.23)),
        mass=120.0,
        origin=Origin(xyz=(0.05, 0.365, 0.0)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=85000.0,
            velocity=0.50,
            lower=-2.62,
            upper=2.62,
        ),
    )
    model.articulation(
        "frame_elevation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=frame,
        origin=Origin(xyz=(0.58, 0.0, 1.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70000.0,
            velocity=0.45,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "left_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_cover,
        origin=Origin(xyz=(2.09, 0.77, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "right_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_cover,
        origin=Origin(xyz=(2.09, -0.77, 0.055)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("deck_base")
    pedestal = object_model.get_part("pedestal")
    frame = object_model.get_part("canister_frame")
    left_cover = object_model.get_part("left_blast_cover")
    right_cover = object_model.get_part("right_blast_cover")
    pedestal_yaw = object_model.get_articulation("pedestal_yaw")
    frame_elevation = object_model.get_articulation("frame_elevation")
    left_cover_hinge = object_model.get_articulation("left_cover_hinge")
    right_cover_hinge = object_model.get_articulation("right_cover_hinge")

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
        "pedestal_yaw_axis_vertical",
        tuple(round(value, 3) for value in pedestal_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {pedestal_yaw.axis}",
    )
    ctx.check(
        "frame_elevation_axis_horizontal",
        tuple(round(value, 3) for value in frame_elevation.axis) == (0.0, -1.0, 0.0),
        details=f"expected horizontal elevation axis, got {frame_elevation.axis}",
    )
    ctx.check(
        "blast_covers_parented_to_frame",
        left_cover_hinge.parent == frame.name and right_cover_hinge.parent == frame.name,
        details=(
            "blast covers must hinge from the canister frame so they travel with the launcher head; "
            f"got parents {left_cover_hinge.parent!r} and {right_cover_hinge.parent!r}"
        ),
    )

    with ctx.pose(
        {
            frame_elevation: 0.0,
            left_cover_hinge: 0.0,
            right_cover_hinge: 0.0,
        }
    ):
        ctx.expect_contact(
            pedestal,
            base,
            contact_tol=1e-4,
            name="pedestal_seats_on_bearing",
        )
        ctx.expect_contact(
            frame,
            pedestal,
            contact_tol=1e-4,
            name="frame_seats_in_trunnions",
        )
        ctx.expect_contact(
            left_cover,
            frame,
            contact_tol=1e-4,
            name="left_cover_closed_against_frame",
        )
        ctx.expect_contact(
            right_cover,
            frame,
            contact_tol=1e-4,
            name="right_cover_closed_against_frame",
        )
        ctx.expect_overlap(
            left_cover,
            frame,
            axes="yz",
            min_overlap=0.72,
            name="left_cover_spans_front_aperture",
        )
        ctx.expect_overlap(
            right_cover,
            frame,
            axes="yz",
            min_overlap=0.72,
            name="right_cover_spans_front_aperture",
        )
        ctx.expect_gap(
            left_cover,
            frame,
            axis="x",
            max_gap=0.05,
            max_penetration=0.0,
            name="left_cover_stays_proud_of_front_frame",
        )
        ctx.expect_gap(
            right_cover,
            frame,
            axis="x",
            max_gap=0.05,
            max_penetration=0.0,
            name="right_cover_stays_proud_of_front_frame",
        )

    with ctx.pose(
        {
            pedestal_yaw: 0.8,
            frame_elevation: 0.65,
            left_cover_hinge: 0.0,
            right_cover_hinge: 0.0,
        }
    ):
        ctx.expect_contact(
            left_cover,
            frame,
            contact_tol=1e-4,
            name="left_cover_moves_with_elevated_launcher_head",
        )
        ctx.expect_contact(
            right_cover,
            frame,
            contact_tol=1e-4,
            name="right_cover_moves_with_elevated_launcher_head",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
