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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_management_scanner")

    body_plastic = model.material("body_plastic", rgba=(0.86, 0.88, 0.90, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.72, 0.75, 0.79, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.22, 0.24, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.60, 0.62, 0.66, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.16, 0.22, 0.27, 0.78))
    reflective_white = model.material("reflective_white", rgba=(0.95, 0.96, 0.93, 1.0))
    indicator_blue = model.material("indicator_blue", rgba=(0.36, 0.58, 0.92, 1.0))

    body_width = 0.52
    body_depth = 0.34
    lower_shell_height = 0.068
    upper_shell_height = 0.018

    lower_shell_mesh = _mesh(
        "scanner_lower_shell",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(body_width, body_depth, 0.028, corner_segments=8),
            lower_shell_height,
        ),
    )
    upper_shell_mesh = _mesh(
        "scanner_upper_shell",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(body_width - 0.018, body_depth - 0.018, 0.022, corner_segments=8),
            upper_shell_height,
        ),
    )
    bezel_frame_mesh = _mesh(
        "scanner_bezel_frame",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.392, 0.272, 0.018, corner_segments=8),
            [rounded_rect_profile(0.314, 0.214, 0.010, corner_segments=8)],
            height=0.006,
            center=True,
        ),
    )
    lid_shell_mesh = _mesh(
        "scanner_lid_shell",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.404, 0.330, 0.020, corner_segments=8),
            0.024,
        ),
    )

    body = model.part("body")
    body.visual(lower_shell_mesh, material=body_plastic, name="lower_shell")
    body.visual(
        upper_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, lower_shell_height)),
        material=body_plastic,
        name="upper_shell",
    )
    body.visual(
        bezel_frame_mesh,
        origin=Origin(xyz=(-0.045, 0.0, 0.083)),
        material=dark_trim,
        name="bezel_frame",
    )
    body.visual(
        Box((0.322, 0.222, 0.003)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0805)),
        material=glass_tint,
        name="platen_glass",
    )
    body.visual(
        Box((0.050, 0.050, 0.005)),
        origin=Origin(xyz=(0.229, 0.104, 0.0835)),
        material=dark_trim,
        name="control_panel",
    )
    for index, x_pos in enumerate((0.214, 0.229, 0.244), start=1):
        body.visual(
            Box((0.010, 0.012, 0.003)),
            origin=Origin(xyz=(x_pos, 0.104, 0.0865)),
            material=indicator_blue,
            name=f"button_{index}",
        )
    body.visual(
        Box((0.360, 0.030, 0.044)),
        origin=Origin(xyz=(0.0, 0.151, 0.030)),
        material=body_plastic,
        name="intake_housing",
    )
    body.visual(
        Box((0.302, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.170, 0.050)),
        material=dark_trim,
        name="feed_slot",
    )
    for name, x_pos in (
        ("rear_hinge_left_outer", -0.173),
        ("rear_hinge_left_inner", -0.117),
        ("rear_hinge_right_inner", 0.117),
        ("rear_hinge_right_outer", 0.173),
    ):
        body.visual(
            Cylinder(radius=0.010, length=0.022),
            origin=Origin(xyz=(x_pos, -0.176, 0.082), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=name,
        )
        body.visual(
            Box((0.022, 0.036, 0.026)),
            origin=Origin(xyz=(x_pos, -0.162, 0.071)),
            material=dark_trim,
            name=f"{name}_leaf",
        )

    for name, x_pos in (
        ("tray_hinge_left_outer", -0.158),
        ("tray_hinge_left_inner", -0.104),
        ("tray_hinge_right_inner", 0.104),
        ("tray_hinge_right_outer", 0.158),
    ):
        body.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(xyz=(x_pos, 0.176, 0.012), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=name,
        )
        body.visual(
            Box((0.022, 0.022, 0.024)),
            origin=Origin(xyz=(x_pos, 0.162, 0.016)),
            material=dark_trim,
            name=f"{name}_leaf",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.52, 0.34, 0.090)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.0, 0.178, 0.004)),
        material=lid_plastic,
        name="lid_shell",
    )
    lid.visual(
        Box((0.376, 0.286, 0.014)),
        origin=Origin(xyz=(0.0, 0.180, 0.011)),
        material=reflective_white,
        name="reflective_panel",
    )
    lid.visual(
        Box((0.150, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.336, 0.008)),
        material=dark_trim,
        name="front_pull",
    )
    for side, x_pos in (("left", -0.145), ("right", 0.145)):
        lid.visual(
            Box((0.028, 0.026, 0.020)),
            origin=Origin(xyz=(x_pos, 0.015, 0.011)),
            material=dark_trim,
            name=f"{side}_hinge_leaf",
        )
        lid.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"{side}_hinge_knuckle",
        )

    lid.inertial = Inertial.from_geometry(
        Box((0.404, 0.330, 0.024)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.178, 0.016)),
    )

    feed_tray = model.part("feed_tray")
    feed_tray.visual(
        Box((0.340, 0.012, 0.132)),
        origin=Origin(xyz=(0.0, 0.012, 0.074)),
        material=body_plastic,
        name="tray_panel",
    )
    for side, x_pos, guide_x in (
        ("left", -0.131, -0.150),
        ("right", 0.131, 0.150),
    ):
        feed_tray.visual(
            Box((0.028, 0.016, 0.024)),
            origin=Origin(xyz=(x_pos, 0.008, 0.010)),
            material=dark_trim,
            name=f"{side}_mount_ear",
        )
        feed_tray.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"{side}_hinge_knuckle",
        )
        feed_tray.visual(
            Box((0.014, 0.014, 0.116)),
            origin=Origin(xyz=(guide_x, 0.013, 0.082)),
            material=dark_trim,
            name=f"{side}_paper_guide",
        )
    feed_tray.visual(
        Box((0.300, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.013, 0.138)),
        material=dark_trim,
        name="sheet_stop",
    )

    feed_tray.inertial = Inertial.from_geometry(
        Box((0.340, 0.030, 0.142)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.012, 0.071)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.176, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_feed_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=feed_tray,
        origin=Origin(xyz=(0.0, 0.176, 0.012)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    feed_tray = object_model.get_part("feed_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_hinge = object_model.get_articulation("body_to_feed_tray")

    bezel_frame = body.get_visual("bezel_frame")
    intake_housing = body.get_visual("intake_housing")
    feed_slot = body.get_visual("feed_slot")
    lid_shell = lid.get_visual("lid_shell")
    front_pull = lid.get_visual("front_pull")
    tray_panel = feed_tray.get_visual("tray_panel")
    sheet_stop = feed_tray.get_visual("sheet_stop")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

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
        "lid hinge rotates about left-right axis",
        lid_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "feed tray hinge folds about left-right axis toward the front",
        tray_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={tray_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, tray_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_shell,
            negative_elem=bezel_frame,
            max_gap=0.003,
            max_penetration=0.0,
            name="closed lid rests just above the platen bezel",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_shell,
            elem_b=bezel_frame,
            min_overlap=0.26,
            name="closed lid covers the scanner glass area",
        )
        ctx.expect_gap(
            feed_tray,
            body,
            axis="y",
            positive_elem=tray_panel,
            negative_elem=feed_slot,
            max_gap=0.008,
            max_penetration=0.0,
            name="closed feed tray parks flush with the front feed slot",
        )
        ctx.expect_overlap(
            feed_tray,
            body,
            axes="xz",
            elem_a=tray_panel,
            elem_b=intake_housing,
            min_overlap=0.03,
            name="closed feed tray aligns with the intake throat",
        )
        closed_pull_aabb = ctx.part_element_world_aabb(lid, elem="front_pull")
        closed_sheet_stop_aabb = ctx.part_element_world_aabb(feed_tray, elem="sheet_stop")

    with ctx.pose({lid_hinge: 1.15, tray_hinge: 0.0}):
        open_pull_aabb = ctx.part_element_world_aabb(lid, elem="front_pull")

    with ctx.pose({lid_hinge: 0.0, tray_hinge: 0.90}):
        ctx.expect_gap(
            feed_tray,
            body,
            axis="y",
            positive_elem=sheet_stop,
            negative_elem=intake_housing,
            min_gap=0.08,
            name="opened feed tray projects forward for document loading",
        )
        open_sheet_stop_aabb = ctx.part_element_world_aabb(feed_tray, elem="sheet_stop")

    closed_pull_center = _aabb_center(closed_pull_aabb)
    open_pull_center = _aabb_center(open_pull_aabb)
    closed_sheet_stop_center = _aabb_center(closed_sheet_stop_aabb)
    open_sheet_stop_center = _aabb_center(open_sheet_stop_aabb)

    ctx.check(
        "lid front edge lifts upward when the rear hinges open",
        closed_pull_center is not None
        and open_pull_center is not None
        and open_pull_center[2] > closed_pull_center[2] + 0.18,
        details=f"closed={closed_pull_center}, open={open_pull_center}",
    )
    ctx.check(
        "feed tray swings forward when opened",
        closed_sheet_stop_center is not None
        and open_sheet_stop_center is not None
        and open_sheet_stop_center[1] > closed_sheet_stop_center[1] + 0.08,
        details=f"closed={closed_sheet_stop_center}, open={open_sheet_stop_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
