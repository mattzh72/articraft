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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


BASE_LENGTH = 1.08
BASE_WIDTH = 0.180
BASE_HEIGHT = 0.140
BASE_WALL = 0.010
BASE_REAR_BULKHEAD = 0.018

MID_LENGTH = 0.88
MID_WIDTH = 0.146
MID_HEIGHT = 0.106
MID_WALL = 0.008
MID_REAR_BULKHEAD = 0.016

TIP_LENGTH = 0.72
TIP_WIDTH = 0.114
TIP_HEIGHT = 0.078
TIP_WALL = 0.007

BASE_TO_MID_RETRACTED_X = 0.10
MID_TO_TIP_RETRACTED_X = 0.28

MID_EXTENSION = 0.42
TIP_EXTENSION = 0.18

PAD_THICKNESS = 0.004
PAD_CLEARANCE = 0.003


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _rect_tube(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    rear_bulkhead: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, width, height).translate((length / 2.0, 0.0, 0.0))
    inner = (
        cq.Workplane("XY")
        .box(length - rear_bulkhead + 0.02, width - 2.0 * wall, height - 2.0 * wall)
        .translate(((rear_bulkhead + length) / 2.0, 0.0, 0.0))
    )
    return outer.cut(inner)


def _pedestal_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.44, 0.26, 0.02).translate((0.04, 0.0, -0.17))
    column = cq.Workplane("XY").box(0.12, 0.18, 0.074).translate((0.04, 0.0, -0.123))
    cheek_left = cq.Workplane("XY").box(0.20, 0.010, 0.064).translate((0.09, 0.095, -0.118))
    cheek_right = cq.Workplane("XY").box(0.20, 0.010, 0.064).translate((0.09, -0.095, -0.118))
    def make_gusset() -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .polyline([(-0.02, -0.16), (0.13, -0.16), (0.13, -0.092), (0.04, -0.092)])
            .close()
        )

    left_gusset = make_gusset().extrude(0.012).translate((0.0, 0.054, 0.0))
    right_gusset = make_gusset().extrude(-0.012).translate((0.0, -0.054, 0.0))
    tie_bar = cq.Workplane("XY").box(0.14, 0.18, 0.018).translate((0.11, 0.0, -0.145))
    mount = plate.union(column).union(cheek_left).union(cheek_right)
    mount = mount.union(left_gusset).union(right_gusset).union(tie_bar)
    for x_pos in (-0.12, 0.20):
        for y_pos in (-0.08, 0.08):
            hole = cq.Workplane("XY").circle(0.010).extrude(0.03).translate((x_pos, y_pos, -0.185))
            mount = mount.cut(hole)
    return mount


def _base_outer_shape() -> cq.Workplane:
    shell = _rect_tube(
        length=BASE_LENGTH,
        width=BASE_WIDTH,
        height=BASE_HEIGHT,
        wall=BASE_WALL,
        rear_bulkhead=BASE_REAR_BULKHEAD,
    )
    collar_outer = cq.Workplane("XY").box(0.055, BASE_WIDTH + 0.016, BASE_HEIGHT + 0.016).translate(
        (BASE_LENGTH - 0.0275, 0.0, 0.0)
    )
    collar_inner = cq.Workplane("XY").box(0.07, BASE_WIDTH - 2.0 * BASE_WALL, BASE_HEIGHT - 2.0 * BASE_WALL).translate(
        (BASE_LENGTH - 0.03, 0.0, 0.0)
    )
    shell = shell.union(collar_outer.cut(collar_inner))
    left_window = cq.Workplane("XY").box(0.24, 0.020, 0.070).translate((0.34, 0.080, 0.0))
    right_window = cq.Workplane("XY").box(0.24, 0.020, 0.070).translate((0.34, -0.080, 0.0))
    top_window = cq.Workplane("XY").box(0.18, 0.090, 0.016).translate((0.26, 0.0, 0.062))
    bottom_mount_pad = cq.Workplane("XY").box(0.26, 0.11, 0.014).translate((0.16, 0.0, -0.077))
    shell = shell.cut(left_window).cut(right_window).cut(top_window).union(bottom_mount_pad)
    return shell


def _stage_outer_shape(length: float, width: float, height: float, wall: float, rear_bulkhead: float) -> cq.Workplane:
    shell = _rect_tube(
        length=length,
        width=width,
        height=height,
        wall=wall,
        rear_bulkhead=rear_bulkhead,
    )
    collar_outer = cq.Workplane("XY").box(0.045, width + 0.012, height + 0.012).translate((length - 0.0225, 0.0, 0.0))
    collar_inner = cq.Workplane("XY").box(0.06, width - 2.0 * wall, height - 2.0 * wall).translate((length - 0.025, 0.0, 0.0))
    return shell.union(collar_outer.cut(collar_inner))


def _tip_outer_shape() -> cq.Workplane:
    shell = _rect_tube(length=TIP_LENGTH, width=TIP_WIDTH, height=TIP_HEIGHT, wall=TIP_WALL, rear_bulkhead=0.0)
    nose_block = cq.Workplane("XY").box(0.050, 0.092, 0.060).translate((TIP_LENGTH - 0.010, 0.0, 0.0))
    lug_left = cq.Workplane("XY").box(0.090, 0.012, 0.090).translate((TIP_LENGTH + 0.035, 0.026, 0.0))
    lug_right = cq.Workplane("XY").box(0.090, 0.012, 0.090).translate((TIP_LENGTH + 0.035, -0.026, 0.0))
    lug_bridge = cq.Workplane("XY").box(0.022, 0.064, 0.050).translate((TIP_LENGTH - 0.006, 0.0, 0.0))
    lug_hole = (
        cq.Workplane("XZ")
        .circle(0.015)
        .extrude(0.10)
        .translate((TIP_LENGTH + 0.035, -0.05, 0.0))
    )
    doubler = cq.Workplane("XY").box(0.10, TIP_WIDTH + 0.008, TIP_HEIGHT + 0.006).translate((TIP_LENGTH - 0.05, 0.0, 0.0))
    doubler_core = cq.Workplane("XY").box(0.12, TIP_WIDTH, TIP_HEIGHT).translate((TIP_LENGTH - 0.05, 0.0, 0.0))
    tip = shell.union(nose_block).union(lug_left).union(lug_right).union(lug_bridge).union(doubler.cut(doubler_core))
    return tip.cut(lug_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_boom_study", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.52, 0.54, 0.57, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.42, 0.44, 0.48, 1.0))
    wear_pad = model.material("wear_pad", rgba=(0.76, 0.69, 0.47, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.58, 0.60, 0.64, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(_mesh(_pedestal_shape(), "pedestal.obj"), material=dark_steel, name="mount_frame")
    pedestal.visual(
        Box((0.22, 0.12, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, -0.10)),
        material=steel,
        name="saddle_block",
    )
    pedestal.visual(
        Box((0.012, 0.12, 0.10)),
        origin=Origin(xyz=(-0.006, 0.0, -0.02)),
        material=steel,
        name="rear_thrust_block",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.44, 0.26, 0.24)),
        mass=34.0,
        origin=Origin(xyz=(0.05, 0.0, -0.09)),
    )

    base_stage = model.part("base_stage")
    base_stage.visual(_mesh(_base_outer_shape(), "base_stage_outer.obj"), material=coated_steel, name="outer_sleeve")
    base_stage.visual(
        Box((0.12, 0.11, PAD_THICKNESS)),
        origin=Origin(xyz=(0.22, 0.0, (BASE_HEIGHT / 2.0 - BASE_WALL) - PAD_THICKNESS / 2.0)),
        material=wear_pad,
        name="top_rear_pad",
    )
    base_stage.visual(
        Box((0.14, 0.11, PAD_THICKNESS)),
        origin=Origin(xyz=(0.92, 0.0, (BASE_HEIGHT / 2.0 - BASE_WALL) - PAD_THICKNESS / 2.0)),
        material=wear_pad,
        name="top_front_pad",
    )
    base_stage.visual(
        Box((0.12, 0.11, PAD_THICKNESS)),
        origin=Origin(xyz=(0.22, 0.0, -((BASE_HEIGHT / 2.0 - BASE_WALL) - PAD_THICKNESS / 2.0))),
        material=wear_pad,
        name="bottom_rear_pad",
    )
    base_stage.visual(
        Box((0.14, 0.11, PAD_THICKNESS)),
        origin=Origin(xyz=(0.92, 0.0, -((BASE_HEIGHT / 2.0 - BASE_WALL) - PAD_THICKNESS / 2.0))),
        material=wear_pad,
        name="bottom_front_pad",
    )
    base_stage.visual(
        Box((0.12, PAD_THICKNESS, 0.072)),
        origin=Origin(xyz=(0.90, (BASE_WIDTH / 2.0 - BASE_WALL) - PAD_THICKNESS / 2.0, 0.0)),
        material=wear_pad,
        name="left_inner_pad",
    )
    base_stage.visual(
        Box((0.12, PAD_THICKNESS, 0.072)),
        origin=Origin(xyz=(0.90, -((BASE_WIDTH / 2.0 - BASE_WALL) - PAD_THICKNESS / 2.0), 0.0)),
        material=wear_pad,
        name="right_inner_pad",
    )
    base_stage.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(0.985, 0.054, BASE_HEIGHT / 2.0 + 0.006)),
        material=steel,
        name="left_stop_block",
    )
    base_stage.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(0.985, -0.054, BASE_HEIGHT / 2.0 + 0.006)),
        material=steel,
        name="right_stop_block",
    )
    base_stage.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=22.0,
        origin=Origin(xyz=(BASE_LENGTH / 2.0, 0.0, 0.0)),
    )

    left_cover = model.part("left_access_cover")
    left_cover.visual(
        Box((0.24, 0.006, 0.09)),
        material=cover_finish,
        name="cover_plate",
    )
    for idx, x_pos in enumerate((-0.08, 0.08), start=1):
        for jdx, z_pos in enumerate((-0.024, 0.024), start=1):
            left_cover.visual(
                Cylinder(radius=0.008, length=0.003),
                origin=Origin(xyz=(x_pos, -0.0045, z_pos), rpy=(1.57079632679, 0.0, 0.0)),
                material=steel,
                name=f"boss_{idx}_{jdx}",
            )
    left_cover.inertial = Inertial.from_geometry(Box((0.24, 0.006, 0.09)), mass=0.9)

    right_cover = model.part("right_access_cover")
    right_cover.visual(
        Box((0.24, 0.006, 0.09)),
        material=cover_finish,
        name="cover_plate",
    )
    for idx, x_pos in enumerate((-0.08, 0.08), start=1):
        for jdx, z_pos in enumerate((-0.024, 0.024), start=1):
            right_cover.visual(
                Cylinder(radius=0.008, length=0.003),
                origin=Origin(xyz=(x_pos, 0.0045, z_pos), rpy=(1.57079632679, 0.0, 0.0)),
                material=steel,
                name=f"boss_{idx}_{jdx}",
            )
    right_cover.inertial = Inertial.from_geometry(Box((0.24, 0.006, 0.09)), mass=0.9)

    mid_stage = model.part("mid_stage")
    mid_stage.visual(
        _mesh(
            _stage_outer_shape(
                length=MID_LENGTH,
                width=MID_WIDTH,
                height=MID_HEIGHT,
                wall=MID_WALL,
                rear_bulkhead=MID_REAR_BULKHEAD,
            ),
            "mid_stage_outer.obj",
        ),
        material=steel,
        name="outer_sleeve",
    )
    mid_stage.visual(
        Box((0.10, 0.090, PAD_THICKNESS)),
        origin=Origin(xyz=(0.28, 0.0, (MID_HEIGHT / 2.0 - MID_WALL) - PAD_THICKNESS / 2.0)),
        material=wear_pad,
        name="top_rear_pad",
    )
    mid_stage.visual(
        Box((0.12, 0.090, PAD_THICKNESS)),
        origin=Origin(xyz=(0.74, 0.0, (MID_HEIGHT / 2.0 - MID_WALL) - PAD_THICKNESS / 2.0)),
        material=wear_pad,
        name="top_front_pad",
    )
    mid_stage.visual(
        Box((0.10, 0.090, PAD_THICKNESS)),
        origin=Origin(xyz=(0.28, 0.0, -((MID_HEIGHT / 2.0 - MID_WALL) - PAD_THICKNESS / 2.0))),
        material=wear_pad,
        name="bottom_rear_pad",
    )
    mid_stage.visual(
        Box((0.12, 0.090, PAD_THICKNESS)),
        origin=Origin(xyz=(0.74, 0.0, -((MID_HEIGHT / 2.0 - MID_WALL) - PAD_THICKNESS / 2.0))),
        material=wear_pad,
        name="bottom_front_pad",
    )
    mid_stage.visual(
        Box((0.10, PAD_THICKNESS, 0.056)),
        origin=Origin(xyz=(0.72, (MID_WIDTH / 2.0 - MID_WALL) - PAD_THICKNESS / 2.0, 0.0)),
        material=wear_pad,
        name="left_inner_pad",
    )
    mid_stage.visual(
        Box((0.10, PAD_THICKNESS, 0.056)),
        origin=Origin(xyz=(0.72, -((MID_WIDTH / 2.0 - MID_WALL) - PAD_THICKNESS / 2.0), 0.0)),
        material=wear_pad,
        name="right_inner_pad",
    )
    mid_stage.inertial = Inertial.from_geometry(
        Box((MID_LENGTH, MID_WIDTH, MID_HEIGHT)),
        mass=12.0,
        origin=Origin(xyz=(MID_LENGTH / 2.0, 0.0, 0.0)),
    )

    tip_stage = model.part("tip_stage")
    tip_stage.visual(_mesh(_tip_outer_shape(), "tip_stage.obj"), material=steel, name="outer_sleeve")
    tip_stage.visual(
        Box((0.10, 0.084, 0.006)),
        origin=Origin(xyz=(0.18, 0.0, 0.042)),
        material=wear_pad,
        name="top_guide_pad",
    )
    tip_stage.visual(
        Box((0.10, 0.084, 0.006)),
        origin=Origin(xyz=(0.18, 0.0, -0.042)),
        material=wear_pad,
        name="bottom_guide_pad",
    )
    tip_stage.inertial = Inertial.from_geometry(
        Box((TIP_LENGTH + 0.09, TIP_WIDTH, TIP_HEIGHT + 0.02)),
        mass=7.2,
        origin=Origin(xyz=(TIP_LENGTH / 2.0 + 0.03, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_base",
        ArticulationType.FIXED,
        parent=pedestal,
        child=base_stage,
        origin=Origin(),
    )
    model.articulation(
        "base_to_left_cover",
        ArticulationType.FIXED,
        parent=base_stage,
        child=left_cover,
        origin=Origin(xyz=(0.34, 0.1040, 0.0)),
    )
    model.articulation(
        "base_to_right_cover",
        ArticulationType.FIXED,
        parent=base_stage,
        child=right_cover,
        origin=Origin(xyz=(0.34, -0.1040, 0.0)),
    )
    model.articulation(
        "base_to_mid",
        ArticulationType.PRISMATIC,
        parent=base_stage,
        child=mid_stage,
        origin=Origin(xyz=(BASE_TO_MID_RETRACTED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.40, lower=0.0, upper=MID_EXTENSION),
    )
    model.articulation(
        "mid_to_tip",
        ArticulationType.PRISMATIC,
        parent=mid_stage,
        child=tip_stage,
        origin=Origin(xyz=(MID_TO_TIP_RETRACTED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.45, lower=0.0, upper=TIP_EXTENSION),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    base_stage = object_model.get_part("base_stage")
    left_cover = object_model.get_part("left_access_cover")
    right_cover = object_model.get_part("right_access_cover")
    mid_stage = object_model.get_part("mid_stage")
    tip_stage = object_model.get_part("tip_stage")

    base_to_mid = object_model.get_articulation("base_to_mid")
    mid_to_tip = object_model.get_articulation("mid_to_tip")

    saddle_block = pedestal.get_visual("saddle_block")
    rear_thrust = pedestal.get_visual("rear_thrust_block")
    top_front_pad = base_stage.get_visual("top_front_pad")
    bottom_front_pad = base_stage.get_visual("bottom_front_pad")
    left_inner_pad = base_stage.get_visual("left_inner_pad")
    right_inner_pad = base_stage.get_visual("right_inner_pad")
    mid_top_front_pad = mid_stage.get_visual("top_front_pad")
    mid_bottom_front_pad = mid_stage.get_visual("bottom_front_pad")
    mid_left_inner_pad = mid_stage.get_visual("left_inner_pad")
    mid_right_inner_pad = mid_stage.get_visual("right_inner_pad")
    tip_shell = tip_stage.get_visual("outer_sleeve")
    tip_top_guide = tip_stage.get_visual("top_guide_pad")
    tip_bottom_guide = tip_stage.get_visual("bottom_guide_pad")

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

    for name, part in (
        ("pedestal", pedestal),
        ("base_stage", base_stage),
        ("left_access_cover", left_cover),
        ("right_access_cover", right_cover),
        ("mid_stage", mid_stage),
        ("tip_stage", tip_stage),
    ):
        ctx.check(f"part_present_{name}", part is not None, f"missing part: {name}")

    ctx.expect_contact(base_stage, pedestal, elem_b=saddle_block, contact_tol=0.0005, name="base_sits_on_saddle")
    ctx.expect_gap(
        base_stage,
        pedestal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem=rear_thrust,
        name="base_rear_against_thrust_block",
    )

    ctx.expect_contact(left_cover, base_stage, contact_tol=0.0005, name="left_cover_seated_on_base")
    ctx.expect_contact(right_cover, base_stage, contact_tol=0.0005, name="right_cover_seated_on_base")

    with ctx.pose({base_to_mid: 0.0, mid_to_tip: 0.0}):
        ctx.expect_origin_gap(mid_stage, base_stage, axis="x", min_gap=BASE_TO_MID_RETRACTED_X, max_gap=BASE_TO_MID_RETRACTED_X, name="mid_retracted_origin")
        ctx.expect_origin_gap(tip_stage, mid_stage, axis="x", min_gap=MID_TO_TIP_RETRACTED_X, max_gap=MID_TO_TIP_RETRACTED_X, name="tip_retracted_origin")
        ctx.expect_within(mid_stage, base_stage, axes="yz", margin=0.0, name="mid_within_base_profile_retracted")
        ctx.expect_within(tip_stage, mid_stage, axes="yz", margin=0.0, name="tip_within_mid_profile_retracted")
        ctx.expect_overlap(mid_stage, base_stage, axes="x", min_overlap=0.87, name="mid_overlap_retracted")
        ctx.expect_overlap(tip_stage, mid_stage, axes="x", min_overlap=0.47, name="tip_overlap_retracted")
        ctx.expect_contact(base_stage, mid_stage, elem_a=top_front_pad, contact_tol=0.0005, name="base_top_pad_guides_mid")
        ctx.expect_contact(base_stage, mid_stage, elem_a=bottom_front_pad, contact_tol=0.0005, name="base_bottom_pad_guides_mid")
        ctx.expect_contact(base_stage, mid_stage, elem_a=left_inner_pad, contact_tol=0.0005, name="base_left_pad_guides_mid")
        ctx.expect_contact(base_stage, mid_stage, elem_a=right_inner_pad, contact_tol=0.0005, name="base_right_pad_guides_mid")
        ctx.expect_contact(tip_stage, mid_stage, elem_a=tip_top_guide, contact_tol=0.0005, name="tip_top_guide_contacts_mid")
        ctx.expect_contact(tip_stage, mid_stage, elem_a=tip_bottom_guide, contact_tol=0.0005, name="tip_bottom_guide_contacts_mid")
        ctx.expect_within(tip_stage, mid_stage, axes="yz", margin=0.0, name="tip_within_mid_guide_channel")

    with ctx.pose({base_to_mid: MID_EXTENSION, mid_to_tip: TIP_EXTENSION}):
        ctx.expect_origin_gap(
            mid_stage,
            base_stage,
            axis="x",
            min_gap=BASE_TO_MID_RETRACTED_X + MID_EXTENSION,
            max_gap=BASE_TO_MID_RETRACTED_X + MID_EXTENSION,
            name="mid_extended_origin",
        )
        ctx.expect_origin_gap(
            tip_stage,
            mid_stage,
            axis="x",
            min_gap=MID_TO_TIP_RETRACTED_X + TIP_EXTENSION - 1e-6,
            max_gap=MID_TO_TIP_RETRACTED_X + TIP_EXTENSION + 1e-6,
            name="tip_extended_origin",
        )
        ctx.expect_within(mid_stage, base_stage, axes="yz", margin=0.0, name="mid_within_base_profile_extended")
        ctx.expect_within(tip_stage, mid_stage, axes="yz", margin=0.0, name="tip_within_mid_profile_extended")
        ctx.expect_overlap(mid_stage, base_stage, axes="x", min_overlap=0.55, name="mid_overlap_extended")
        ctx.expect_overlap(tip_stage, mid_stage, axes="x", min_overlap=0.40, name="tip_overlap_extended")

        tip_aabb = ctx.part_element_world_aabb(tip_stage, elem=tip_shell)
        if tip_aabb is None:
            ctx.fail("tip_shell_aabb_available", "tip shell AABB unavailable")
        else:
            tip_front_x = tip_aabb[1][0]
            ctx.check(
                "tip_front_reaches_beyond_base_front",
                tip_front_x > BASE_LENGTH + 0.07,
                f"tip front x={tip_front_x:.4f} should extend beyond base front",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
