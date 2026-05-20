from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


BASE_RAIL_TOP = 0.062
BASE_STAGE1_X = 0.090
STAGE1_STAGE2_X = 0.055
STAGE2_STAGE3_X = 0.050

STAGE1_TRAVEL = 0.220
STAGE2_TRAVEL = 0.180
STAGE3_TRAVEL = 0.140

RAIL_Y = 0.085
STAGE1_PAD_Y = RAIL_Y
STAGE2_PAD_Y = 0.063
STAGE3_PAD_Y = 0.051

STAGE1_PAD_H = 0.008
STAGE2_PAD_H = 0.006
STAGE3_PAD_H = 0.005

STAGE1_SHELL_Z = STAGE1_PAD_H
STAGE2_SHELL_Z = STAGE2_PAD_H
STAGE3_SHELL_Z = STAGE3_PAD_H

STAGE1_WALL = 0.008
STAGE2_WALL = 0.006
STAGE3_WALL = 0.005

STAGE1_GUIDE_Z = STAGE1_SHELL_Z + STAGE1_WALL
STAGE2_GUIDE_Z = STAGE2_SHELL_Z + STAGE2_WALL

STAGE1_GUIDE_TOP = STAGE1_GUIDE_Z + 0.006
STAGE2_GUIDE_TOP = STAGE2_GUIDE_Z + 0.005


def _box_wp(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x, y, z))
    )


def _open_front_shell(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    z0: float,
    side_window_x: float,
    side_window_length: float,
    side_window_z: float,
    side_window_height: float,
    top_window_x: float,
    top_window_length: float,
    top_window_width: float,
) -> cq.Workplane:
    shell = _box_wp(length, width, height, z=z0)
    inner = _box_wp(
        length - wall + 0.004,
        width - 2.0 * wall,
        height - 2.0 * wall,
        x=wall,
        z=z0 + wall,
    )
    shell = shell.cut(inner)

    window_width = wall + 0.012
    y_center = 0.5 * width - 0.5 * window_width + 0.001
    left_cut = _box_wp(
        side_window_length,
        window_width,
        side_window_height,
        x=side_window_x,
        y=y_center,
        z=side_window_z,
    )
    right_cut = _box_wp(
        side_window_length,
        window_width,
        side_window_height,
        x=side_window_x,
        y=-y_center,
        z=side_window_z,
    )
    shell = shell.cut(left_cut).cut(right_cut)

    top_cut = _box_wp(
        top_window_length,
        top_window_width,
        wall + 0.012,
        x=top_window_x,
        z=z0 + height - (wall + 0.002),
    )
    return shell.cut(top_cut)


def _base_weldment_shape() -> cq.Workplane:
    weldment = _box_wp(0.110, 0.240, 0.018, x=0.000, z=0.016)
    weldment = weldment.union(_box_wp(0.100, 0.220, 0.018, x=0.290, z=0.016))
    weldment = weldment.union(_box_wp(0.110, 0.240, 0.018, x=0.570, z=0.016))
    weldment = weldment.union(_box_wp(0.580, 0.022, 0.018, x=0.050, y=0.118, z=0.016))
    weldment = weldment.union(_box_wp(0.580, 0.022, 0.018, x=0.050, y=-0.118, z=0.016))
    weldment = weldment.union(_box_wp(0.050, 0.032, 0.026, x=0.060, y=0.045, z=0.034))
    weldment = weldment.union(_box_wp(0.050, 0.032, 0.026, x=0.060, y=-0.045, z=0.034))
    weldment = weldment.union(_box_wp(0.050, 0.032, 0.026, x=0.610, y=0.045, z=0.034))
    weldment = weldment.union(_box_wp(0.050, 0.032, 0.026, x=0.610, y=-0.045, z=0.034))
    return weldment


def _stage1_shell_shape() -> cq.Workplane:
    shell = _open_front_shell(
        length=0.420,
        width=0.160,
        height=0.094,
        wall=STAGE1_WALL,
        z0=STAGE1_SHELL_Z,
        side_window_x=0.095,
        side_window_length=0.210,
        side_window_z=0.030,
        side_window_height=0.042,
        top_window_x=0.180,
        top_window_length=0.150,
        top_window_width=0.082,
    )
    shell = shell.union(_box_wp(0.028, 0.132, 0.028, x=0.010, z=STAGE1_SHELL_Z))
    shell = shell.union(_box_wp(0.032, 0.176, 0.064, x=0.388, z=0.020))
    shell = shell.cut(
        _box_wp(
            0.020,
            0.130,
            0.040,
            x=0.400,
            z=0.030,
        )
    )
    return shell


def _stage2_shell_shape() -> cq.Workplane:
    shell = _open_front_shell(
        length=0.310,
        width=0.118,
        height=0.046,
        wall=STAGE2_WALL,
        z0=STAGE2_SHELL_Z,
        side_window_x=0.070,
        side_window_length=0.155,
        side_window_z=0.018,
        side_window_height=0.018,
        top_window_x=0.132,
        top_window_length=0.104,
        top_window_width=0.050,
    )
    shell = shell.union(_box_wp(0.024, 0.090, 0.018, x=0.008, z=STAGE2_SHELL_Z))
    shell = shell.union(_box_wp(0.026, 0.104, 0.036, x=0.284, z=0.008))
    shell = shell.cut(_box_wp(0.016, 0.072, 0.020, x=0.292, z=0.014))
    shell = shell.cut(_box_wp(0.235, 0.020, 0.016, x=0.028, y=STAGE3_PAD_Y, z=STAGE2_SHELL_Z))
    shell = shell.cut(_box_wp(0.235, 0.020, 0.016, x=0.028, y=-STAGE3_PAD_Y, z=STAGE2_SHELL_Z))
    return shell


def _stage3_shell_shape() -> cq.Workplane:
    shell = _open_front_shell(
        length=0.230,
        width=0.084,
        height=0.020,
        wall=STAGE3_WALL,
        z0=STAGE3_SHELL_Z,
        side_window_x=0.050,
        side_window_length=0.090,
        side_window_z=0.010,
        side_window_height=0.008,
        top_window_x=0.090,
        top_window_length=0.070,
        top_window_width=0.030,
    )
    shell = shell.union(_box_wp(0.020, 0.064, 0.010, x=0.008, z=STAGE3_SHELL_Z))
    shell = shell.union(_box_wp(0.022, 0.072, 0.014, x=0.206, z=0.006))
    shell = shell.cut(_box_wp(0.014, 0.040, 0.008, x=0.214, z=0.010))
    return shell


def _add_base_part(model: ArticulatedObject):
    steel_dark = model.material("steel_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.45, 0.48, 0.52, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.66, 0.68, 0.71, 1.0))
    pad_black = model.material("pad_black", rgba=(0.14, 0.14, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_weldment_shape(), "base_weldment.obj", assets=ASSETS),
        material=steel_dark,
        name="base_weldment",
    )
    base.visual(
        Box((0.680, 0.030, 0.028)),
        origin=Origin(xyz=(0.390, RAIL_Y, 0.048)),
        material=steel_mid,
        name="rail_left",
    )
    base.visual(
        Box((0.680, 0.030, 0.028)),
        origin=Origin(xyz=(0.390, -RAIL_Y, 0.048)),
        material=steel_mid,
        name="rail_right",
    )
    base.visual(
        Box((0.120, 0.016, 0.022)),
        origin=Origin(xyz=(0.096, 0.000, 0.045)),
        material=steel_mid,
        name="rear_stop",
    )
    base.visual(
        Box((0.160, 0.048, 0.004)),
        origin=Origin(xyz=(0.320, 0.118, 0.036)),
        material=cover_gray,
        name="left_access_cover",
    )
    base.visual(
        Box((0.160, 0.048, 0.004)),
        origin=Origin(xyz=(0.320, -0.118, 0.036)),
        material=cover_gray,
        name="right_access_cover",
    )
    for name, x_center, y_center in (
        ("foot_front_left", 0.635, 0.095),
        ("foot_front_right", 0.635, -0.095),
        ("foot_rear_left", 0.055, 0.095),
        ("foot_rear_right", 0.055, -0.095),
    ):
        base.visual(
            Box((0.070, 0.050, 0.016)),
            origin=Origin(xyz=(x_center, y_center, 0.008)),
            material=pad_black,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.780, 0.280, 0.080)),
        mass=18.0,
        origin=Origin(xyz=(0.390, 0.0, 0.040)),
    )
    return base


def _add_stage1_part(model: ArticulatedObject):
    shell_mat = model.material("stage1_shell", rgba=(0.63, 0.65, 0.67, 1.0))
    guide_mat = model.material("guide_bronze", rgba=(0.74, 0.62, 0.39, 1.0))
    cover_mat = model.material("cover_plate", rgba=(0.76, 0.78, 0.80, 1.0))
    polymer = model.material("polymer_pad", rgba=(0.20, 0.20, 0.19, 1.0))

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_stage1_shell_shape(), "stage1_shell.obj", assets=ASSETS),
        material=shell_mat,
        name="carriage_shell",
    )
    stage1.visual(
        Box((0.220, 0.024, STAGE1_PAD_H)),
        origin=Origin(xyz=(0.170, STAGE1_PAD_Y, 0.004)),
        material=polymer,
        name="pad_left",
    )
    stage1.visual(
        Box((0.220, 0.024, STAGE1_PAD_H)),
        origin=Origin(xyz=(0.170, -STAGE1_PAD_Y, 0.004)),
        material=polymer,
        name="pad_right",
    )
    stage1.visual(
        Box((0.320, 0.018, 0.006)),
        origin=Origin(xyz=(0.200, STAGE2_PAD_Y, STAGE1_GUIDE_Z + 0.003)),
        material=guide_mat,
        name="guide_left",
    )
    stage1.visual(
        Box((0.320, 0.018, 0.006)),
        origin=Origin(xyz=(0.200, -STAGE2_PAD_Y, STAGE1_GUIDE_Z + 0.003)),
        material=guide_mat,
        name="guide_right",
    )
    stage1.visual(
        Box((0.110, 0.114, 0.004)),
        origin=Origin(xyz=(0.110, 0.000, STAGE1_SHELL_Z + 0.094 + 0.002)),
        material=cover_mat,
        name="top_cover",
    )
    stage1.visual(
        Box((0.018, 0.018, 0.018)),
        origin=Origin(xyz=(0.370, STAGE2_PAD_Y, STAGE1_GUIDE_Z + 0.009)),
        material=guide_mat,
        name="stop_left",
    )
    stage1.visual(
        Box((0.018, 0.018, 0.018)),
        origin=Origin(xyz=(0.370, -STAGE2_PAD_Y, STAGE1_GUIDE_Z + 0.009)),
        material=guide_mat,
        name="stop_right",
    )
    stage1.inertial = Inertial.from_geometry(
        Box((0.420, 0.176, 0.106)),
        mass=7.5,
        origin=Origin(xyz=(0.210, 0.0, 0.053)),
    )
    return stage1


def _add_stage2_part(model: ArticulatedObject):
    shell_mat = model.material("stage2_shell", rgba=(0.58, 0.60, 0.62, 1.0))
    guide_mat = model.material("guide_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    cover_mat = model.material("cover_light", rgba=(0.80, 0.81, 0.83, 1.0))
    polymer = model.material("pad_dark", rgba=(0.18, 0.18, 0.18, 1.0))

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_stage2_shell_shape(), "stage2_shell.obj", assets=ASSETS),
        material=shell_mat,
        name="carriage_shell",
    )
    stage2.visual(
        Box((0.130, 0.016, STAGE2_PAD_H)),
        origin=Origin(xyz=(0.115, STAGE2_PAD_Y, 0.003)),
        material=polymer,
        name="pad_left",
    )
    stage2.visual(
        Box((0.130, 0.016, STAGE2_PAD_H)),
        origin=Origin(xyz=(0.115, -STAGE2_PAD_Y, 0.003)),
        material=polymer,
        name="pad_right",
    )
    stage2.visual(
        Box((0.240, 0.014, 0.005)),
        origin=Origin(xyz=(0.155, STAGE3_PAD_Y, STAGE2_GUIDE_Z + 0.0025)),
        material=guide_mat,
        name="guide_left",
    )
    stage2.visual(
        Box((0.240, 0.014, 0.005)),
        origin=Origin(xyz=(0.155, -STAGE3_PAD_Y, STAGE2_GUIDE_Z + 0.0025)),
        material=guide_mat,
        name="guide_right",
    )
    stage2.visual(
        Box((0.090, 0.076, 0.004)),
        origin=Origin(xyz=(0.088, 0.000, STAGE2_SHELL_Z + 0.046)),
        material=cover_mat,
        name="top_cover",
    )
    stage2.visual(
        Box((0.016, 0.014, 0.014)),
        origin=Origin(xyz=(0.280, STAGE3_PAD_Y, STAGE2_GUIDE_Z + 0.007)),
        material=guide_mat,
        name="stop_left",
    )
    stage2.visual(
        Box((0.016, 0.014, 0.014)),
        origin=Origin(xyz=(0.280, -STAGE3_PAD_Y, STAGE2_GUIDE_Z + 0.007)),
        material=guide_mat,
        name="stop_right",
    )
    stage2.inertial = Inertial.from_geometry(
        Box((0.310, 0.118, 0.052)),
        mass=4.0,
        origin=Origin(xyz=(0.155, 0.0, 0.026)),
    )
    return stage2


def _add_stage3_part(model: ArticulatedObject):
    shell_mat = model.material("stage3_shell", rgba=(0.54, 0.56, 0.58, 1.0))
    platen_mat = model.material("platen_steel", rgba=(0.52, 0.55, 0.60, 1.0))
    polymer = model.material("pad_black_small", rgba=(0.16, 0.16, 0.16, 1.0))
    cover_mat = model.material("cover_small", rgba=(0.78, 0.79, 0.81, 1.0))

    stage3 = model.part("stage3")
    stage3.visual(
        mesh_from_cadquery(_stage3_shell_shape(), "stage3_shell.obj", assets=ASSETS),
        material=shell_mat,
        name="carriage_shell",
    )
    stage3.visual(
        Box((0.090, 0.012, STAGE3_PAD_H)),
        origin=Origin(xyz=(0.085, STAGE3_PAD_Y, 0.004)),
        material=polymer,
        name="pad_left",
    )
    stage3.visual(
        Box((0.090, 0.012, STAGE3_PAD_H)),
        origin=Origin(xyz=(0.085, -STAGE3_PAD_Y, 0.004)),
        material=polymer,
        name="pad_right",
    )
    stage3.visual(
        Box((0.090, 0.018, 0.008)),
        origin=Origin(xyz=(0.085, 0.042, 0.006)),
        material=platen_mat,
        name="pad_bridge_left",
    )
    stage3.visual(
        Box((0.090, 0.018, 0.008)),
        origin=Origin(xyz=(0.085, -0.042, 0.006)),
        material=platen_mat,
        name="pad_bridge_right",
    )
    stage3.visual(
        Box((0.080, 0.046, 0.004)),
        origin=Origin(xyz=(0.075, 0.000, STAGE3_SHELL_Z + 0.020)),
        material=cover_mat,
        name="top_cover",
    )
    stage3.visual(
        Box((0.146, 0.026, 0.012)),
        origin=Origin(xyz=(0.291, 0.000, 0.014)),
        material=platen_mat,
        name="platen_bridge",
    )
    stage3.visual(
        Box((0.024, 0.092, 0.056)),
        origin=Origin(xyz=(0.352, 0.000, 0.028)),
        material=platen_mat,
        name="mounting_platen",
    )
    stage3.inertial = Inertial.from_geometry(
        Box((0.376, 0.092, 0.056)),
        mass=2.4,
        origin=Origin(xyz=(0.188, 0.0, 0.028)),
    )
    return stage3


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_prismatic_chain", assets=ASSETS)

    base = _add_base_part(model)
    stage1 = _add_stage1_part(model)
    stage2 = _add_stage2_part(model)
    stage3 = _add_stage3_part(model)

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(BASE_STAGE1_X, 0.0, BASE_RAIL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(STAGE1_STAGE2_X, 0.0, STAGE1_GUIDE_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.40,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(STAGE2_STAGE3_X, 0.0, STAGE2_GUIDE_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.45,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    stage3 = object_model.get_part("stage3")

    joint1 = object_model.get_articulation("base_to_stage1")
    joint2 = object_model.get_articulation("stage1_to_stage2")
    joint3 = object_model.get_articulation("stage2_to_stage3")

    base_rail_left = base.get_visual("rail_left")
    base_rail_right = base.get_visual("rail_right")
    stage1_pad_left = stage1.get_visual("pad_left")
    stage1_pad_right = stage1.get_visual("pad_right")
    stage1_shell = stage1.get_visual("carriage_shell")
    stage1_guide_left = stage1.get_visual("guide_left")
    stage1_guide_right = stage1.get_visual("guide_right")
    stage2_pad_left = stage2.get_visual("pad_left")
    stage2_pad_right = stage2.get_visual("pad_right")
    stage2_shell = stage2.get_visual("carriage_shell")
    stage2_guide_left = stage2.get_visual("guide_left")
    stage2_guide_right = stage2.get_visual("guide_right")
    stage3_pad_left = stage3.get_visual("pad_left")
    stage3_pad_right = stage3.get_visual("pad_right")
    stage3_shell = stage3.get_visual("carriage_shell")
    stage3_platen = stage3.get_visual("mounting_platen")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    with ctx.pose({joint1: 0.0, joint2: 0.0, joint3: 0.0}):
        ctx.expect_contact(stage1, base, elem_a=stage1_pad_left, elem_b=base_rail_left)
        ctx.expect_contact(stage1, base, elem_a=stage1_pad_right, elem_b=base_rail_right)
        ctx.expect_contact(stage2, stage1, elem_a=stage2_pad_left, elem_b=stage1_guide_left)
        ctx.expect_contact(stage2, stage1, elem_a=stage2_pad_right, elem_b=stage1_guide_right)
        ctx.expect_contact(stage3, stage2, elem_a=stage3_pad_left, elem_b=stage2_guide_left)
        ctx.expect_contact(stage3, stage2, elem_a=stage3_pad_right, elem_b=stage2_guide_right)

        ctx.expect_within(stage2, stage1, axes="yz", inner_elem=stage2_shell, outer_elem=stage1_shell)
        ctx.expect_within(stage3, stage2, axes="yz", inner_elem=stage3_shell, outer_elem=stage2_shell)
        ctx.expect_within(stage3, stage2, axes="y", inner_elem=stage3_platen, outer_elem=stage2_shell, margin=0.020)

        ctx.expect_origin_gap(stage1, base, axis="x", min_gap=BASE_STAGE1_X, max_gap=BASE_STAGE1_X)
        ctx.expect_origin_gap(stage2, stage1, axis="x", min_gap=STAGE1_STAGE2_X, max_gap=STAGE1_STAGE2_X)
        ctx.expect_origin_gap(stage3, stage2, axis="x", min_gap=STAGE2_STAGE3_X, max_gap=STAGE2_STAGE3_X)
        ctx.expect_origin_gap(stage1, base, axis="z", min_gap=BASE_RAIL_TOP, max_gap=BASE_RAIL_TOP)
        ctx.expect_origin_gap(stage2, stage1, axis="z", min_gap=STAGE1_GUIDE_TOP, max_gap=STAGE1_GUIDE_TOP)
        ctx.expect_origin_gap(stage3, stage2, axis="z", min_gap=STAGE2_GUIDE_TOP, max_gap=STAGE2_GUIDE_TOP)
        ctx.expect_origin_distance(stage1, base, axes="y", max_dist=1e-6)
        ctx.expect_origin_distance(stage2, stage1, axes="y", max_dist=1e-6)
        ctx.expect_origin_distance(stage3, stage2, axes="y", max_dist=1e-6)

    with ctx.pose({joint1: 0.180, joint2: 0.120, joint3: 0.090}):
        ctx.expect_contact(stage1, base, elem_a=stage1_pad_left, elem_b=base_rail_left)
        ctx.expect_contact(stage1, base, elem_a=stage1_pad_right, elem_b=base_rail_right)
        ctx.expect_contact(stage2, stage1, elem_a=stage2_pad_left, elem_b=stage1_guide_left)
        ctx.expect_contact(stage2, stage1, elem_a=stage2_pad_right, elem_b=stage1_guide_right)
        ctx.expect_contact(stage3, stage2, elem_a=stage3_pad_left, elem_b=stage2_guide_left)
        ctx.expect_contact(stage3, stage2, elem_a=stage3_pad_right, elem_b=stage2_guide_right)

        ctx.expect_within(stage2, stage1, axes="yz", inner_elem=stage2_shell, outer_elem=stage1_shell)
        ctx.expect_within(stage3, stage2, axes="yz", inner_elem=stage3_shell, outer_elem=stage2_shell)
        ctx.expect_origin_gap(
            stage1,
            base,
            axis="x",
            min_gap=BASE_STAGE1_X + 0.180,
            max_gap=BASE_STAGE1_X + 0.180,
        )
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=STAGE1_STAGE2_X + 0.120,
            max_gap=STAGE1_STAGE2_X + 0.120,
        )
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="x",
            min_gap=STAGE2_STAGE3_X + 0.090,
            max_gap=STAGE2_STAGE3_X + 0.090,
        )
        ctx.expect_overlap(stage1, base, axes="x", min_overlap=0.180)
        ctx.expect_overlap(stage2, stage1, axes="x", min_overlap=0.120)
        ctx.expect_overlap(stage3, stage2, axes="x", min_overlap=0.080)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
