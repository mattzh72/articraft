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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

CASE_WIDTH = 0.340
CASE_DEPTH = 0.240
BASE_HEIGHT = 0.118
LID_HEIGHT = 0.056
CORNER_RADIUS = 0.022
WALL_THICKNESS = 0.005
BASE_FLOOR_THICKNESS = 0.006
LID_TOP_THICKNESS = 0.005
SEAM_GAP = 0.0012

LID_WIDTH = CASE_WIDTH + 0.012
LID_DEPTH = CASE_DEPTH + 0.012
LID_CORNER_RADIUS = CORNER_RADIUS + 0.005

HINGE_RADIUS = 0.007
HINGE_Y = (CASE_DEPTH * 0.5) + HINGE_RADIUS + 0.001
HINGE_Z = BASE_HEIGHT - 0.008
LID_SHELL_BOTTOM = (BASE_HEIGHT + SEAM_GAP) - HINGE_Z
LID_SHELL_CENTER_Y = -HINGE_Y
LATCH_KEEPER_Y = LID_SHELL_CENTER_Y - (LID_DEPTH * 0.5) + 0.005
LATCH_KEEPER_Z = LID_SHELL_BOTTOM - 0.0006


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_shell_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    corner_radius: float,
    wall_thickness: float,
    retained_thickness: float,
    open_side: str,
):
    outer_profile = rounded_rect_profile(
        width,
        depth,
        radius=corner_radius,
        corner_segments=10,
    )
    inner_profile = rounded_rect_profile(
        width - (2.0 * wall_thickness),
        depth - (2.0 * wall_thickness),
        radius=max(0.002, corner_radius - wall_thickness),
        corner_segments=10,
    )
    outer = ExtrudeGeometry.from_z0(outer_profile, height)
    if open_side == "top":
        inner = ExtrudeGeometry.from_z0(inner_profile, height - retained_thickness + 0.001)
        inner.translate(0.0, 0.0, retained_thickness)
    else:
        inner = ExtrudeGeometry.from_z0(inner_profile, height - retained_thickness + 0.001)
        inner.translate(0.0, 0.0, -0.001)
    return boolean_difference(outer, inner)


def _build_base_shell_mesh():
    return _rounded_shell_mesh(
        width=CASE_WIDTH,
        depth=CASE_DEPTH,
        height=BASE_HEIGHT,
        corner_radius=CORNER_RADIUS,
        wall_thickness=WALL_THICKNESS,
        retained_thickness=BASE_FLOOR_THICKNESS,
        open_side="top",
    )


def _build_lid_shell_mesh():
    return _rounded_shell_mesh(
        width=LID_WIDTH,
        depth=LID_DEPTH,
        height=LID_HEIGHT,
        corner_radius=LID_CORNER_RADIUS,
        wall_thickness=WALL_THICKNESS,
        retained_thickness=LID_TOP_THICKNESS,
        open_side="bottom",
    )


def _build_handle_mesh():
    return tube_from_spline_points(
        [
            (-0.040, 0.0, 0.0),
            (-0.024, 0.0, 0.016),
            (0.024, 0.0, 0.016),
            (0.040, 0.0, 0.0),
        ],
        radius=0.0045,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="instrument_hard_case", assets=ASSETS)

    shell_charcoal = model.material("shell_charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    latch_black = model.material("latch_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh("case_base_shell.obj", _build_base_shell_mesh()),
        material=shell_charcoal,
        name="base_shell",
    )
    base.visual(
        Box((0.022, 0.012, 0.034)),
        origin=Origin(xyz=(-0.078, -(CASE_DEPTH * 0.5) - 0.006, BASE_HEIGHT - 0.024)),
        material=latch_black,
        name="left_latch_body",
    )
    base.visual(
        Box((0.022, 0.012, 0.034)),
        origin=Origin(xyz=(0.078, -(CASE_DEPTH * 0.5) - 0.006, BASE_HEIGHT - 0.024)),
        material=latch_black,
        name="right_latch_body",
    )
    base.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(-0.078, -(CASE_DEPTH * 0.5) - 0.015, BASE_HEIGHT - 0.011)),
        material=latch_black,
        name="left_latch_tab",
    )
    base.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.078, -(CASE_DEPTH * 0.5) - 0.015, BASE_HEIGHT - 0.011)),
        material=latch_black,
        name="right_latch_tab",
    )
    base.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.084),
        origin=Origin(xyz=(-0.096, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.084),
        origin=Origin(xyz=(0.096, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_hinge_barrel",
    )
    base.visual(
        Box((0.026, 0.006, 0.020)),
        origin=Origin(xyz=(-0.096, 0.122, HINGE_Z)),
        material=hinge_metal,
        name="left_hinge_leaf",
    )
    base.visual(
        Box((0.026, 0.006, 0.020)),
        origin=Origin(xyz=(0.096, 0.122, HINGE_Z)),
        material=hinge_metal,
        name="right_hinge_leaf",
    )
    base.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, BASE_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        _save_mesh("case_lid_shell.obj", _build_lid_shell_mesh()),
        origin=Origin(xyz=(0.0, LID_SHELL_CENTER_Y, LID_SHELL_BOTTOM)),
        material=shell_charcoal,
        name="lid_shell",
    )
    lid.visual(
        Box((0.030, 0.010, LID_SHELL_BOTTOM + 0.004)),
        origin=Origin(xyz=(-0.043, 0.002, (LID_SHELL_BOTTOM + 0.004) * 0.5)),
        material=hinge_metal,
        name="left_hinge_cheek",
    )
    lid.visual(
        Box((0.030, 0.010, LID_SHELL_BOTTOM + 0.004)),
        origin=Origin(xyz=(0.043, 0.002, (LID_SHELL_BOTTOM + 0.004) * 0.5)),
        material=hinge_metal,
        name="right_hinge_cheek",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.116),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(-0.078, LATCH_KEEPER_Y, LATCH_KEEPER_Z)),
        material=latch_black,
        name="left_latch_keeper",
    )
    lid.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(0.078, LATCH_KEEPER_Y, LATCH_KEEPER_Z)),
        material=latch_black,
        name="right_latch_keeper",
    )
    lid.visual(
        Box((0.012, 0.018, 0.016)),
        origin=Origin(
            xyz=(
                -0.040,
                LID_SHELL_CENTER_Y,
                LID_SHELL_BOTTOM + LID_HEIGHT + 0.008,
            )
        ),
        material=grip_black,
        name="left_handle_post",
    )
    lid.visual(
        Box((0.012, 0.018, 0.016)),
        origin=Origin(
            xyz=(
                0.040,
                LID_SHELL_CENTER_Y,
                LID_SHELL_BOTTOM + LID_HEIGHT + 0.008,
            )
        ),
        material=grip_black,
        name="right_handle_post",
    )
    lid.visual(
        _save_mesh("case_handle_grip.obj", _build_handle_mesh()),
        origin=Origin(
            xyz=(
                0.0,
                LID_SHELL_CENTER_Y,
                LID_SHELL_BOTTOM + LID_HEIGHT + 0.016,
            )
        ),
        material=grip_black,
        name="handle_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_HEIGHT + 0.028)),
        mass=0.9,
        origin=Origin(
            xyz=(
                0.0,
                LID_SHELL_CENTER_Y,
                LID_SHELL_BOTTOM + (LID_HEIGHT * 0.5),
            )
        ),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")

    base_shell = base.get_visual("base_shell")
    lid_shell = lid.get_visual("lid_shell")
    left_latch_body = base.get_visual("left_latch_body")
    right_latch_body = base.get_visual("right_latch_body")
    left_latch_keeper = lid.get_visual("left_latch_keeper")
    right_latch_keeper = lid.get_visual("right_latch_keeper")
    handle_grip = lid.get_visual("handle_grip")
    center_hinge_barrel = lid.get_visual("center_hinge_barrel")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    ctx.check(
        "core_visuals_present",
        all(
            visual is not None
            for visual in (
                base_shell,
                lid_shell,
                left_latch_body,
                right_latch_body,
                left_latch_keeper,
                right_latch_keeper,
                handle_grip,
                center_hinge_barrel,
            )
        ),
        details="Expected shell, latch, hinge, and handle visuals were not all created.",
    )
    ctx.check(
        "rear_hinge_axis_and_limit",
        tuple(rear_hinge.axis) == (-1.0, 0.0, 0.0)
        and abs(rear_hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(rear_hinge.motion_limits.upper - math.radians(100.0)) < 1e-9,
        details="Rear hinge must rotate about the rear x-axis through roughly 100 degrees.",
    )
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.220, elem_a=lid_shell, elem_b=base_shell)
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem=lid_shell,
        negative_elem=base_shell,
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.005,
        elem_a=left_latch_keeper,
        elem_b=left_latch_body,
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.005,
        elem_a=right_latch_keeper,
        elem_b=right_latch_body,
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.0025,
        positive_elem=left_latch_keeper,
        negative_elem=left_latch_body,
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.0025,
        positive_elem=right_latch_keeper,
        negative_elem=right_latch_body,
    )
    ctx.expect_origin_distance(lid, base, axes="x", max_dist=1e-6)

    with ctx.pose({rear_hinge: math.radians(100.0)}):
        ctx.expect_overlap(lid, base, axes="x", min_overlap=0.280, elem_a=lid_shell, elem_b=base_shell)
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.100,
            positive_elem=left_latch_keeper,
            negative_elem=base_shell,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
