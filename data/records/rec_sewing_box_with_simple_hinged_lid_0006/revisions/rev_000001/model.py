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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

BODY_X = 0.200
BODY_Y = 0.138
BODY_H = 0.082
BODY_OUTER_R = 0.022
BODY_WALL = 0.005
BODY_INNER_R = BODY_OUTER_R - BODY_WALL
TOP_BAND_H = 0.016
FLOOR_T = 0.004

LID_OVERHANG = 0.004
LID_X = BODY_X + 2.0 * LID_OVERHANG
LID_Y = BODY_Y + 2.0 * LID_OVERHANG
LID_OUTER_R = 0.024
LID_SLAB_T = 0.005
LID_FRAME_H = 0.004
LID_INSET_T = 0.0022
LID_LOCATOR_PAD_X = 0.010
LID_LOCATOR_PAD_Y = 0.018
LID_LOCATOR_PAD_Z = 0.007
HINGE_R = 0.0032
HINGE_SUPPORT_R = 0.0024
HINGE_AXIS_Y = -(BODY_Y * 0.5) - 0.008
HINGE_AXIS_Z = BODY_H + HINGE_R
HINGE_BARREL_LEN = 0.028
HINGE_BARREL_CENTER_LEN = 0.078
HINGE_BARREL_X = 0.055
LID_CENTER_Y = -HINGE_AXIS_Y
LID_BOTTOM_Z = BODY_H + 0.0002 - HINGE_AXIS_Z


def _rounded_profile(width: float, depth: float, radius: float) -> list[tuple[float, float]]:
    return rounded_rect_profile(width, depth, radius, corner_segments=10)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / name)


def _plate_mesh(name: str, *, width: float, depth: float, radius: float, height: float):
    return _save_mesh(
        name,
        ExtrudeGeometry(
            _rounded_profile(width, depth, radius),
            height=height,
            center=False,
        ),
    )


def _ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_depth: float,
    outer_radius: float,
    inner_width: float,
    inner_depth: float,
    inner_radius: float,
    height: float,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            _rounded_profile(outer_width, outer_depth, outer_radius),
            [_rounded_profile(inner_width, inner_depth, inner_radius)],
            height=height,
            center=False,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sewing_box", assets=ASSETS)

    shell_matte = model.material("shell_matte", rgba=(0.79, 0.76, 0.72, 1.0))
    shell_satin = model.material("shell_satin", rgba=(0.67, 0.62, 0.54, 1.0))
    lining_satin = model.material("lining_satin", rgba=(0.50, 0.46, 0.42, 1.0))
    hardware_satin = model.material("hardware_satin", rgba=(0.67, 0.69, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        _ring_mesh(
            "sewing_box_body_lower_shell.obj",
            outer_width=BODY_X,
            outer_depth=BODY_Y,
            outer_radius=BODY_OUTER_R,
            inner_width=BODY_X - 2.0 * BODY_WALL,
            inner_depth=BODY_Y - 2.0 * BODY_WALL,
            inner_radius=BODY_INNER_R,
            height=BODY_H - TOP_BAND_H,
        ),
        material=shell_matte,
        name="lower_shell",
    )
    body.visual(
        _ring_mesh(
            "sewing_box_body_upper_shell.obj",
            outer_width=BODY_X,
            outer_depth=BODY_Y,
            outer_radius=BODY_OUTER_R,
            inner_width=BODY_X - 2.0 * BODY_WALL,
            inner_depth=BODY_Y - 2.0 * BODY_WALL,
            inner_radius=BODY_INNER_R,
            height=TOP_BAND_H,
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_BAND_H)),
        material=shell_satin,
        name="upper_shell",
    )
    body.visual(
        _plate_mesh(
            "sewing_box_body_floor.obj",
            width=BODY_X - 2.0 * BODY_WALL,
            depth=BODY_Y - 2.0 * BODY_WALL,
            radius=BODY_INNER_R,
            height=FLOOR_T,
        ),
        material=lining_satin,
        name="floor_panel",
    )
    body.visual(
        _ring_mesh(
            "sewing_box_body_rim_frame.obj",
            outer_width=BODY_X - 0.001,
            outer_depth=BODY_Y - 0.001,
            outer_radius=BODY_OUTER_R - 0.0005,
            inner_width=BODY_X - 2.0 * BODY_WALL - 0.004,
            inner_depth=BODY_Y - 2.0 * BODY_WALL - 0.004,
            inner_radius=max(BODY_INNER_R - 0.002, 0.001),
            height=0.0024,
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - 0.0024)),
        material=lining_satin,
        name="rim_frame",
    )
    body.visual(
        _save_mesh(
            "sewing_box_left_hinge_support.obj",
            tube_from_spline_points(
                [
                    (-HINGE_BARREL_X, -0.0688, BODY_H - 0.0035),
                    (-HINGE_BARREL_X, -0.0710, BODY_H - 0.0010),
                    (-HINGE_BARREL_X, -0.0742, BODY_H + 0.0018),
                    (-HINGE_BARREL_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
                ],
                radius=HINGE_SUPPORT_R,
                samples_per_segment=10,
                radial_segments=12,
            ),
        ),
        material=shell_satin,
        name="left_hinge_support",
    )
    body.visual(
        _save_mesh(
            "sewing_box_right_hinge_support.obj",
            tube_from_spline_points(
                [
                    (HINGE_BARREL_X, -0.0688, BODY_H - 0.0035),
                    (HINGE_BARREL_X, -0.0710, BODY_H - 0.0010),
                    (HINGE_BARREL_X, -0.0742, BODY_H + 0.0018),
                    (HINGE_BARREL_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
                ],
                radius=HINGE_SUPPORT_R,
                samples_per_segment=10,
                radial_segments=12,
            ),
        ),
        material=shell_satin,
        name="right_hinge_support",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=HINGE_BARREL_LEN),
        origin=Origin(
            xyz=(-HINGE_BARREL_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_satin,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=HINGE_BARREL_LEN),
        origin=Origin(
            xyz=(HINGE_BARREL_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_satin,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_X, BODY_Y, BODY_H)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        _plate_mesh(
            "sewing_box_lid_outer_slab.obj",
            width=LID_X,
            depth=LID_Y,
            radius=LID_OUTER_R,
            height=LID_SLAB_T,
        ),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_BOTTOM_Z)),
        material=shell_matte,
        name="outer_slab",
    )
    lid.visual(
        _ring_mesh(
            "sewing_box_lid_frame_ring.obj",
            outer_width=LID_X,
            outer_depth=LID_Y,
            outer_radius=LID_OUTER_R,
            inner_width=LID_X - 0.040,
            inner_depth=LID_Y - 0.036,
            inner_radius=LID_OUTER_R - 0.008,
            height=LID_FRAME_H,
        ),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_BOTTOM_Z + LID_SLAB_T)),
        material=shell_satin,
        name="frame_ring",
    )
    lid.visual(
        _plate_mesh(
            "sewing_box_lid_inset_panel.obj",
            width=LID_X - 0.046,
            depth=LID_Y - 0.042,
            radius=LID_OUTER_R - 0.010,
            height=LID_INSET_T,
        ),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_BOTTOM_Z + LID_SLAB_T)),
        material=lining_satin,
        name="inset_panel",
    )
    lid.visual(
        Box((LID_LOCATOR_PAD_X, LID_LOCATOR_PAD_Y, LID_LOCATOR_PAD_Z)),
        origin=Origin(
            xyz=(
                -0.056,
                LID_CENTER_Y + 0.018,
                LID_BOTTOM_Z - (LID_LOCATOR_PAD_Z * 0.5) + 0.0005,
            )
        ),
        material=lining_satin,
        name="left_locator_pad",
    )
    lid.visual(
        Box((LID_LOCATOR_PAD_X, LID_LOCATOR_PAD_Y, LID_LOCATOR_PAD_Z)),
        origin=Origin(
            xyz=(
                0.056,
                LID_CENTER_Y + 0.018,
                LID_BOTTOM_Z - (LID_LOCATOR_PAD_Z * 0.5) + 0.0005,
            )
        ),
        material=lining_satin,
        name="right_locator_pad",
    )
    lid.visual(
        Box((0.018, 0.012, 0.008)),
        origin=Origin(xyz=(-0.024, 0.006, 0.001)),
        material=shell_satin,
        name="left_hinge_bridge",
    )
    lid.visual(
        Box((0.018, 0.012, 0.008)),
        origin=Origin(xyz=(0.024, 0.006, 0.001)),
        material=shell_satin,
        name="right_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=HINGE_BARREL_CENTER_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_satin,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.022, 0.006, 0.003)),
        origin=Origin(
            xyz=(0.0, LID_CENTER_Y + (LID_Y * 0.5) - 0.010, LID_BOTTOM_Z + LID_SLAB_T + 0.0015)
        ),
        material=hardware_satin,
        name="pull_base",
    )
    lid.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(
            xyz=(0.0, LID_CENTER_Y + (LID_Y * 0.5) - 0.0065, LID_BOTTOM_Z + LID_SLAB_T + 0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_satin,
        name="front_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_X, LID_Y, 0.022)),
        mass=0.34,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    upper_shell = body.get_visual("upper_shell")
    rim_frame = body.get_visual("rim_frame")
    left_hinge_support = body.get_visual("left_hinge_support")
    right_hinge_support = body.get_visual("right_hinge_support")
    left_hinge_barrel = body.get_visual("left_hinge_barrel")
    right_hinge_barrel = body.get_visual("right_hinge_barrel")
    outer_slab = lid.get_visual("outer_slab")
    frame_ring = lid.get_visual("frame_ring")
    left_locator_pad = lid.get_visual("left_locator_pad")
    right_locator_pad = lid.get_visual("right_locator_pad")
    center_hinge_barrel = lid.get_visual("center_hinge_barrel")
    front_pull = lid.get_visual("front_pull")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=outer_slab,
        negative_elem=rim_frame,
        name="lid_seats_cleanly_on_body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.11,
        elem_a=frame_ring,
        elem_b=upper_shell,
        name="lid_covers_body_opening",
    )
    ctx.expect_within(
        body,
        lid,
        axes="xy",
        margin=0.006,
        inner_elem=upper_shell,
        outer_elem=outer_slab,
        name="body_nests_within_lid_footprint",
    )
    ctx.expect_within(
        lid,
        body,
        axes="xy",
        margin=0.003,
        inner_elem=left_locator_pad,
        outer_elem=upper_shell,
        name="left_locator_pad_sits_inside_body_walls",
    )
    ctx.expect_within(
        lid,
        body,
        axes="xy",
        margin=0.003,
        inner_elem=right_locator_pad,
        outer_elem=upper_shell,
        name="right_locator_pad_sits_inside_body_walls",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        min_gap=0.0015,
        max_gap=0.0035,
        positive_elem=center_hinge_barrel,
        negative_elem=left_hinge_barrel,
        name="left_knuckle_clearance_is_tight_but_clean",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="x",
        min_gap=0.0015,
        max_gap=0.0035,
        positive_elem=right_hinge_barrel,
        negative_elem=center_hinge_barrel,
        name="right_knuckle_clearance_is_tight_but_clean",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.005,
        elem_a=center_hinge_barrel,
        elem_b=left_hinge_barrel,
        name="hinge_barrels_share_a_common_pivot_band",
    )

    with ctx.pose({lid_hinge: math.radians(78.0)}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.030,
            positive_elem=front_pull,
            negative_elem=upper_shell,
            name="opened_lid_lifts_clear_for_access",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.005,
            elem_a=center_hinge_barrel,
            elem_b=left_hinge_barrel,
            name="pivot_alignment_holds_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
