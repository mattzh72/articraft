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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float, *, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _tube_shell_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
            [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
            segments=32,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(pi / 2.0),
        name,
    )


def _perforated_mesh_panel(name: str, *, width: float, height: float, thickness: float):
    border = 0.03
    columns = 7
    rows = 4
    gap_x = 0.022
    gap_y = 0.020
    usable_w = width - 2.0 * border
    usable_h = height - 2.0 * border
    hole_w = (usable_w - (columns - 1) * gap_x) / columns
    hole_h = (usable_h - (rows - 1) * gap_y) / rows
    hole_profiles: list[list[tuple[float, float]]] = []
    x_start = -usable_w * 0.5 + hole_w * 0.5
    y_start = -usable_h * 0.5 + hole_h * 0.5
    for row in range(rows):
        cy = y_start + row * (hole_h + gap_y)
        for column in range(columns):
            cx = x_start + column * (hole_w + gap_x)
            hole_profiles.append(_rect_profile(hole_w, hole_h, center=(cx, cy)))

    panel = ExtrudeWithHolesGeometry(
        _rect_profile(width, height),
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(panel, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="work_truck_tailgate")

    truck_white = model.material("truck_white", rgba=(0.88, 0.89, 0.90, 1.0))
    utility_grey = model.material("utility_grey", rgba=(0.62, 0.65, 0.67, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.26, 0.27, 0.30, 1.0))
    galvanized = model.material("galvanized", rgba=(0.68, 0.71, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    bed_outer_width = 1.68
    bed_inner_width = 1.56
    bed_depth = 0.82
    bed_side_height = 0.62
    floor_thickness = 0.04
    side_thickness = 0.06

    gate_width = 1.52
    gate_height = 0.58
    gate_thickness = 0.055
    lower_hinge_z = 0.03
    gate_top_hinge_y = -0.083
    gate_top_hinge_z = gate_height - 0.010

    ext_width = 1.48
    ext_height = 0.42
    ext_depth = 0.030
    ext_tube = 0.030

    bed_frame = model.part("bed_frame")
    bed_frame.inertial = Inertial.from_geometry(
        Box((bed_outer_width, bed_depth, bed_side_height)),
        mass=120.0,
        origin=Origin(xyz=(0.0, bed_depth * 0.5, bed_side_height * 0.5)),
    )
    bed_frame.visual(
        Box((bed_outer_width, bed_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, bed_depth * 0.5, -floor_thickness * 0.5)),
        material=truck_white,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((side_thickness, bed_depth, bed_side_height)),
        origin=Origin(
            xyz=(
                -bed_outer_width * 0.5 + side_thickness * 0.5,
                bed_depth * 0.5,
                bed_side_height * 0.5,
            )
        ),
        material=truck_white,
        name="left_bed_side",
    )
    bed_frame.visual(
        Box((side_thickness, bed_depth, bed_side_height)),
        origin=Origin(
            xyz=(
                bed_outer_width * 0.5 - side_thickness * 0.5,
                bed_depth * 0.5,
                bed_side_height * 0.5,
            )
        ),
        material=truck_white,
        name="right_bed_side",
    )
    bed_frame.visual(
        Box((bed_inner_width, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.015, 0.03)),
        material=utility_grey,
        name="rear_sill",
    )
    bed_frame.visual(
        Box((0.05, 0.08, 0.16)),
        origin=Origin(xyz=(-0.81, 0.02, 0.08)),
        material=steel_dark,
        name="left_lower_hinge_bracket",
    )
    bed_frame.visual(
        Box((0.05, 0.08, 0.16)),
        origin=Origin(xyz=(0.81, 0.02, 0.08)),
        material=steel_dark,
        name="right_lower_hinge_bracket",
    )
    bed_frame.visual(
        Box((0.06, 0.10, 0.14)),
        origin=Origin(xyz=(-0.81, 0.02, 0.48)),
        material=steel_dark,
        name="left_jamb_pad",
    )
    bed_frame.visual(
        Box((0.06, 0.10, 0.14)),
        origin=Origin(xyz=(0.81, 0.02, 0.48)),
        material=steel_dark,
        name="right_jamb_pad",
    )
    bed_frame.visual(
        Box((bed_outer_width, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.025, bed_side_height - 0.015)),
        material=utility_grey,
        name="top_rail_cap",
    )

    main_gate = model.part("main_gate")
    main_gate.inertial = Inertial.from_geometry(
        Box((gate_width, gate_thickness, gate_height)),
        mass=28.0,
        origin=Origin(xyz=(0.0, -gate_thickness * 0.5, gate_height * 0.5)),
    )
    main_gate.visual(
        Box((gate_width, gate_thickness, gate_height)),
        origin=Origin(xyz=(0.0, -gate_thickness * 0.5, gate_height * 0.5)),
        material=truck_white,
        name="gate_shell",
    )
    main_gate.visual(
        Box((gate_width - 0.18, gate_thickness * 0.48, gate_height - 0.18)),
        origin=Origin(xyz=(0.0, -gate_thickness * 0.24, gate_height * 0.52)),
        material=utility_grey,
        name="inner_recess",
    )
    main_gate.visual(
        Box((gate_width - 0.08, gate_thickness * 0.92, 0.10)),
        origin=Origin(xyz=(0.0, -gate_thickness * 0.5, 0.05)),
        material=steel_dark,
        name="lower_beam",
    )
    main_gate.visual(
        Box((0.09, gate_thickness * 0.96, gate_height - 0.02)),
        origin=Origin(xyz=(-gate_width * 0.5 + 0.045, -gate_thickness * 0.5, gate_height * 0.5)),
        material=steel_dark,
        name="left_stile",
    )
    main_gate.visual(
        Box((0.09, gate_thickness * 0.96, gate_height - 0.02)),
        origin=Origin(xyz=(gate_width * 0.5 - 0.045, -gate_thickness * 0.5, gate_height * 0.5)),
        material=steel_dark,
        name="right_stile",
    )
    main_gate.visual(
        Box((0.08, 0.03, 0.12)),
        origin=Origin(xyz=(-0.72, -0.04, 0.44)),
        material=rubber,
        name="left_latch_pad",
    )
    main_gate.visual(
        Box((0.08, 0.03, 0.12)),
        origin=Origin(xyz=(0.72, -0.04, 0.44)),
        material=rubber,
        name="right_latch_pad",
    )
    main_gate.visual(
        Cylinder(radius=0.024, length=0.10),
        origin=Origin(xyz=(-0.71, -0.020, lower_hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_lower_hinge_knuckle",
    )
    main_gate.visual(
        Cylinder(radius=0.024, length=0.10),
        origin=Origin(xyz=(0.71, -0.020, lower_hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_lower_hinge_knuckle",
    )
    main_gate.visual(
        Box((gate_width - 0.08, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.064, gate_top_hinge_z + 0.012)),
        material=steel_dark,
        name="top_hinge_channel",
    )

    extension_panel = model.part("extension_panel")
    extension_panel.inertial = Inertial.from_geometry(
        Box((ext_width, ext_depth, ext_height)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.004, ext_height * 0.5)),
    )
    extension_panel.visual(
        Box((ext_width - 0.08, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.004, 0.012)),
        material=galvanized,
        name="lower_clip_bar",
    )
    extension_panel.visual(
        Box((ext_tube, ext_depth, 0.396)),
        origin=Origin(xyz=(-ext_width * 0.5 + ext_tube * 0.5, 0.004, 0.222)),
        material=galvanized,
        name="left_frame_rail",
    )
    extension_panel.visual(
        Box((ext_tube, ext_depth, 0.396)),
        origin=Origin(xyz=(ext_width * 0.5 - ext_tube * 0.5, 0.004, 0.222)),
        material=galvanized,
        name="right_frame_rail",
    )
    extension_panel.visual(
        Box((ext_width - 0.06, ext_depth, ext_tube)),
        origin=Origin(xyz=(0.0, 0.004, ext_height - ext_tube * 0.5)),
        material=galvanized,
        name="top_frame_rail",
    )
    extension_panel.visual(
        _perforated_mesh_panel(
            "extension_mesh_panel",
            width=ext_width - 0.06,
            height=ext_height - 0.06,
            thickness=0.004,
        ),
        origin=Origin(xyz=(0.0, 0.004, (ext_height - 0.06) * 0.5)),
        material=galvanized,
        name="mesh_panel",
    )

    model.articulation(
        "main_gate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=main_gate,
        origin=Origin(xyz=(0.0, 0.0, lower_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "extension_hinge",
        ArticulationType.REVOLUTE,
        parent=main_gate,
        child=extension_panel,
        origin=Origin(xyz=(0.0, gate_top_hinge_y, gate_top_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    main_gate = object_model.get_part("main_gate")
    extension_panel = object_model.get_part("extension_panel")
    main_gate_hinge = object_model.get_articulation("main_gate_hinge")
    extension_hinge = object_model.get_articulation("extension_hinge")

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
        "all_tailgate_parts_present",
        all(part is not None for part in (bed_frame, main_gate, extension_panel)),
        "Bed frame, main gate, and extension panel must all exist.",
    )
    ctx.check(
        "horizontal_hinge_axes",
        tuple(main_gate_hinge.axis) == (1.0, 0.0, 0.0) and tuple(extension_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected both hinge axes along +X, got {main_gate_hinge.axis} and {extension_hinge.axis}.",
    )

    with ctx.pose({main_gate_hinge: 0.0, extension_hinge: 0.0}):
        ctx.expect_contact(
            main_gate,
            bed_frame,
            elem_a="gate_shell",
            elem_b="rear_sill",
            name="main_gate_seats_on_rear_sill",
        )
        ctx.expect_within(
            main_gate,
            bed_frame,
            axes="x",
            inner_elem="gate_shell",
            outer_elem="rear_sill",
            margin=0.0,
            name="main_gate_fits_between_bed_sides",
        )
        ctx.expect_contact(
            extension_panel,
            main_gate,
            elem_a="lower_clip_bar",
            elem_b="top_hinge_channel",
            name="extension_clipped_to_gate_top_edge",
        )
        ctx.expect_within(
            extension_panel,
            main_gate,
            axes="x",
            inner_elem="mesh_panel",
            outer_elem="gate_shell",
            margin=0.03,
            name="extension_width_tracks_main_gate",
        )
        ctx.expect_origin_gap(
            extension_panel,
            main_gate,
            axis="z",
            min_gap=0.54,
            max_gap=0.58,
            name="extension_hinge_sits_at_gate_top",
        )

    with ctx.pose({main_gate_hinge: 1.35, extension_hinge: 0.80}):
        ctx.expect_contact(
            extension_panel,
            main_gate,
            elem_a="lower_clip_bar",
            elem_b="top_hinge_channel",
            contact_tol=0.004,
            name="extension_stays_attached_when_lifted",
        )
        gate_aabb = ctx.part_world_aabb(main_gate)
        gate_is_lowered = (
            gate_aabb is not None
            and gate_aabb[0][1] < -0.48
            and gate_aabb[1][2] < 0.18
        )
        ctx.check(
            "main_gate_rotates_downward_rearward",
            gate_is_lowered,
            f"Expected lowered gate to swing rearward and near horizontal, got AABB={gate_aabb}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
