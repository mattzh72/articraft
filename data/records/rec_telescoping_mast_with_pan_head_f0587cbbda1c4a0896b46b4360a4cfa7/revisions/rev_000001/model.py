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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.32
BASE_THICKNESS = 0.028
FOOT_HEIGHT = 0.012
FOOT_RADIUS = 0.023
FOOT_X = 0.155
FOOT_Y = 0.105
PEDESTAL_WIDTH = 0.18
PEDESTAL_HEIGHT = 0.10

OUTER_WIDTH = 0.114
OUTER_WALL = 0.006
OUTER_LENGTH = 0.96
OUTER_BOTTOM_Z = FOOT_HEIGHT + BASE_THICKNESS + PEDESTAL_HEIGHT

MIDDLE_WIDTH = 0.090
MIDDLE_WALL = 0.005
MIDDLE_LENGTH = 0.88
MIDDLE_HOME_Z = 0.42
MIDDLE_TRAVEL = 0.44

INNER_WIDTH = 0.070
INNER_WALL = 0.004
INNER_LENGTH = 0.78
INNER_HOME_Z = 0.28
INNER_TRAVEL = 0.36

TOP_WIDTH = 0.050
TOP_WALL = 0.0035
TOP_LENGTH = 0.64
TOP_HOME_Z = 0.26
TOP_TRAVEL = 0.30

GUIDE_HEIGHT = 0.18
CAP_THICKNESS = 0.008
COLLAR_HEIGHT = 0.058
COLLAR_SLIT = 0.006
PAN_SUPPORT_DISK_T = 0.012
PAN_HUB_HEIGHT = 0.023
PAN_JOINT_Z = TOP_LENGTH + PAN_SUPPORT_DISK_T + PAN_HUB_HEIGHT

OUTER_GUIDE_OPEN = OUTER_WIDTH - 2.0 * OUTER_WALL
MIDDLE_GUIDE_OPEN = MIDDLE_WIDTH - 2.0 * MIDDLE_WALL
INNER_GUIDE_OPEN = INNER_WIDTH - 2.0 * INNER_WALL


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_ring(
    part,
    *,
    outer: float,
    inner: float,
    height: float,
    z_center: float,
    material: str,
    prefix: str,
) -> None:
    wall = (outer - inner) / 2.0
    _add_box(part, (wall, outer, height), (-(inner / 2.0 + wall / 2.0), 0.0, z_center), material, f"{prefix}_left")
    _add_box(part, (wall, outer, height), ((inner / 2.0 + wall / 2.0), 0.0, z_center), material, f"{prefix}_right")
    _add_box(part, (inner, wall, height), (0.0, -(inner / 2.0 + wall / 2.0), z_center), material, f"{prefix}_back")
    _add_box(part, (inner, wall, height), (0.0, (inner / 2.0 + wall / 2.0), z_center), material, f"{prefix}_front")


def _add_square_tube(
    part,
    *,
    width: float,
    wall: float,
    length: float,
    z_base: float,
    material: str,
    prefix: str,
) -> None:
    z_center = z_base + length / 2.0
    _add_box(part, (width, wall, length), (0.0, width / 2.0 - wall / 2.0, z_center), material, f"{prefix}_front_panel")
    _add_box(part, (width, wall, length), (0.0, -width / 2.0 + wall / 2.0, z_center), material, f"{prefix}_back_panel")
    _add_box(part, (wall, width, length), (width / 2.0 - wall / 2.0, 0.0, z_center), material, f"{prefix}_right_panel")
    _add_box(part, (wall, width, length), (-width / 2.0 + wall / 2.0, 0.0, z_center), material, f"{prefix}_left_panel")


def _add_slotted_collar(
    part,
    *,
    outer: float,
    inner: float,
    height: float,
    z_center: float,
    material: str,
    prefix: str,
) -> None:
    wall = (outer - inner) / 2.0
    front_seg = max((inner - COLLAR_SLIT) / 2.0, 0.004)
    _add_box(part, (wall, outer, height), (-(inner / 2.0 + wall / 2.0), 0.0, z_center), material, f"{prefix}_left")
    _add_box(part, (wall, outer, height), ((inner / 2.0 + wall / 2.0), 0.0, z_center), material, f"{prefix}_right")
    _add_box(part, (inner, wall, height), (0.0, -(inner / 2.0 + wall / 2.0), z_center), material, f"{prefix}_back")
    _add_box(
        part,
        (front_seg, wall, height),
        (-(COLLAR_SLIT / 2.0 + front_seg / 2.0), inner / 2.0 + wall / 2.0, z_center),
        material,
        f"{prefix}_front_left",
    )
    _add_box(
        part,
        (front_seg, wall, height),
        ((COLLAR_SLIT / 2.0 + front_seg / 2.0), inner / 2.0 + wall / 2.0, z_center),
        material,
        f"{prefix}_front_right",
    )

    ear_y = outer / 2.0 + 0.0065
    ear_z = z_center + 0.004
    gusset_y = outer / 2.0 + 0.002
    _add_box(part, (0.020, 0.014, 0.026), (-outer * 0.22, ear_y, ear_z), material, f"{prefix}_ear_left")
    _add_box(part, (0.020, 0.014, 0.026), (outer * 0.22, ear_y, ear_z), material, f"{prefix}_ear_right")
    _add_box(part, (0.012, 0.010, 0.018), (-outer * 0.22, gusset_y, z_center + 0.002), material, f"{prefix}_gusset_left")
    _add_box(part, (0.012, 0.010, 0.018), (outer * 0.22, gusset_y, z_center + 0.002), material, f"{prefix}_gusset_right")
    _add_cylinder(
        part,
        0.004,
        outer * 0.56,
        (0.0, ear_y + 0.0015, ear_z + 0.002),
        material,
        f"{prefix}_bolt",
        rpy=(0.0, pi / 2.0, 0.0),
    )


def _add_stage(
    part,
    *,
    width: float,
    wall: float,
    length: float,
    material: str,
    prefix: str,
    z_base: float = 0.0,
    guide_outer: float | None = None,
    cap_outer: float | None = None,
    cap_inner: float | None = None,
    collar_outer: float | None = None,
    collar_inner: float | None = None,
    lower_jacket_outer: float | None = None,
    lower_jacket_inner: float | None = None,
    lower_jacket_height: float = 0.0,
) -> None:
    _add_square_tube(part, width=width, wall=wall, length=length, z_base=z_base, material=material, prefix=f"{prefix}_tube")

    if guide_outer is not None:
        _add_ring(
            part,
            outer=guide_outer,
            inner=max(width - 2.0 * wall, width * 0.55),
            height=GUIDE_HEIGHT,
            z_center=z_base + GUIDE_HEIGHT / 2.0,
            material=material,
            prefix=f"{prefix}_guide",
        )

    if lower_jacket_outer is not None and lower_jacket_inner is not None and lower_jacket_height > 0.0:
        _add_ring(
            part,
            outer=lower_jacket_outer,
            inner=lower_jacket_inner,
            height=lower_jacket_height,
            z_center=z_base + lower_jacket_height / 2.0,
            material=material,
            prefix=f"{prefix}_jacket",
        )

    if cap_outer is not None and cap_inner is not None:
        _add_ring(
            part,
            outer=cap_outer,
            inner=cap_inner,
            height=CAP_THICKNESS,
            z_center=z_base + length - CAP_THICKNESS / 2.0,
            material=material,
            prefix=f"{prefix}_cap",
        )

    if collar_outer is not None and collar_inner is not None:
        _add_slotted_collar(
            part,
            outer=collar_outer,
            inner=collar_inner,
            height=COLLAR_HEIGHT,
            z_center=z_base + length - COLLAR_HEIGHT / 2.0,
            material=material,
            prefix=f"{prefix}_collar",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_mast")

    model.material("base_powdercoat", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("stage_outer", rgba=(0.57, 0.60, 0.63, 1.0))
    model.material("stage_middle", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("stage_inner", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("stage_top", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("pan_head_finish", rgba=(0.24, 0.25, 0.27, 1.0))

    base_column = model.part("base_column")
    _add_box(
        base_column,
        (BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, FOOT_HEIGHT + BASE_THICKNESS / 2.0),
        "base_powdercoat",
        "base_plate",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            _add_cylinder(
                base_column,
                FOOT_RADIUS,
                FOOT_HEIGHT,
                (sx * FOOT_X, sy * FOOT_Y, FOOT_HEIGHT / 2.0),
                "base_powdercoat",
                f"foot_{'r' if sx > 0 else 'l'}_{'f' if sy > 0 else 'b'}",
            )
    _add_box(
        base_column,
        (PEDESTAL_WIDTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
        (0.0, 0.0, FOOT_HEIGHT + BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0),
        "base_powdercoat",
        "pedestal",
    )
    _add_ring(
        base_column,
        outer=0.14,
        inner=OUTER_WIDTH + 0.014,
        height=0.05,
        z_center=OUTER_BOTTOM_Z + 0.025,
        material="base_powdercoat",
        prefix="transition_ring",
    )
    _add_stage(
        base_column,
        width=OUTER_WIDTH,
        wall=OUTER_WALL,
        length=OUTER_LENGTH,
        material="stage_outer",
        prefix="outer_stage",
        z_base=OUTER_BOTTOM_Z,
        cap_outer=OUTER_WIDTH + 0.014,
        cap_inner=OUTER_GUIDE_OPEN,
        collar_outer=OUTER_WIDTH + 0.024,
        collar_inner=OUTER_GUIDE_OPEN,
        lower_jacket_outer=OUTER_WIDTH + 0.036,
        lower_jacket_inner=OUTER_GUIDE_OPEN + 0.010,
        lower_jacket_height=0.16,
    )

    middle_section = model.part("middle_section")
    _add_stage(
        middle_section,
        width=MIDDLE_WIDTH,
        wall=MIDDLE_WALL,
        length=MIDDLE_LENGTH,
        material="stage_middle",
        prefix="middle_stage",
        guide_outer=OUTER_GUIDE_OPEN,
        cap_outer=MIDDLE_WIDTH + 0.012,
        cap_inner=MIDDLE_GUIDE_OPEN,
        collar_outer=MIDDLE_WIDTH + 0.020,
        collar_inner=MIDDLE_GUIDE_OPEN,
    )

    inner_section = model.part("inner_section")
    _add_stage(
        inner_section,
        width=INNER_WIDTH,
        wall=INNER_WALL,
        length=INNER_LENGTH,
        material="stage_inner",
        prefix="inner_stage",
        guide_outer=MIDDLE_GUIDE_OPEN,
        cap_outer=INNER_WIDTH + 0.010,
        cap_inner=INNER_GUIDE_OPEN,
        collar_outer=INNER_WIDTH + 0.018,
        collar_inner=INNER_GUIDE_OPEN,
    )

    top_section = model.part("top_section")
    _add_stage(
        top_section,
        width=TOP_WIDTH,
        wall=TOP_WALL,
        length=TOP_LENGTH,
        material="stage_top",
        prefix="top_stage",
        guide_outer=INNER_GUIDE_OPEN,
    )
    _add_box(
        top_section,
        (TOP_WIDTH + 0.012, TOP_WIDTH + 0.012, CAP_THICKNESS),
        (0.0, 0.0, TOP_LENGTH - CAP_THICKNESS / 2.0),
        "stage_top",
        "top_cap_plate",
    )
    _add_cylinder(
        top_section,
        0.042,
        PAN_SUPPORT_DISK_T,
        (0.0, 0.0, TOP_LENGTH + PAN_SUPPORT_DISK_T / 2.0),
        "stage_top",
        "pan_support_disk",
    )
    _add_cylinder(
        top_section,
        0.028,
        PAN_HUB_HEIGHT,
        (0.0, 0.0, TOP_LENGTH + PAN_SUPPORT_DISK_T + PAN_HUB_HEIGHT / 2.0),
        "stage_top",
        "pan_hub",
    )

    pan_head = model.part("pan_head")
    _add_cylinder(pan_head, 0.052, 0.010, (0.0, 0.0, 0.005), "pan_head_finish", "turntable_disk")
    _add_cylinder(pan_head, 0.030, 0.016, (0.0, 0.0, 0.016), "pan_head_finish", "hub_cap")
    _add_box(pan_head, (0.18, 0.14, 0.008), (0.0, 0.0, 0.014), "pan_head_finish", "head_plate")
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            _add_cylinder(
                pan_head,
                0.0045,
                0.008,
                (sx * 0.055, sy * 0.040, 0.014),
                "base_powdercoat",
                f"mount_bolt_{'r' if sx > 0 else 'l'}_{'f' if sy > 0 else 'b'}",
            )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base_column,
        child=middle_section,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=0.0, upper=MIDDLE_TRAVEL),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_section,
        origin=Origin(xyz=(0.0, 0.0, INNER_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.40, lower=0.0, upper=INNER_TRAVEL),
    )
    model.articulation(
        "inner_to_top",
        ArticulationType.PRISMATIC,
        parent=inner_section,
        child=top_section,
        origin=Origin(xyz=(0.0, 0.0, TOP_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=TOP_TRAVEL),
    )
    model.articulation(
        "top_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=top_section,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, PAN_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_column = object_model.get_part("base_column")
    middle_section = object_model.get_part("middle_section")
    inner_section = object_model.get_part("inner_section")
    top_section = object_model.get_part("top_section")
    pan_head = object_model.get_part("pan_head")

    base_to_middle = object_model.get_articulation("base_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_top = object_model.get_articulation("inner_to_top")
    top_to_pan_head = object_model.get_articulation("top_to_pan_head")

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
        "mast extension axes are vertical",
        base_to_middle.axis == (0.0, 0.0, 1.0)
        and middle_to_inner.axis == (0.0, 0.0, 1.0)
        and inner_to_top.axis == (0.0, 0.0, 1.0)
        and top_to_pan_head.axis == (0.0, 0.0, 1.0),
        "all stage joints and the pan head should move about the world-up mast axis",
    )
    ctx.check(
        "pan head is continuous",
        top_to_pan_head.articulation_type == ArticulationType.CONTINUOUS,
        "the survey head should pan continuously at the mast top",
    )

    with ctx.pose(
        {
            base_to_middle: 0.0,
            middle_to_inner: 0.0,
            inner_to_top: 0.0,
            top_to_pan_head: 0.0,
        }
    ):
        ctx.expect_contact(base_column, middle_section, contact_tol=0.0015, name="middle section guided by outer sleeve")
        ctx.expect_contact(middle_section, inner_section, contact_tol=0.0015, name="inner section guided by middle sleeve")
        ctx.expect_contact(inner_section, top_section, contact_tol=0.0015, name="top section guided by inner sleeve")
        ctx.expect_contact(top_section, pan_head, contact_tol=0.0015, name="pan head seats on bearing hub")
        ctx.expect_origin_distance(middle_section, base_column, axes="xy", max_dist=0.001, name="middle section centered in mast")
        ctx.expect_origin_distance(inner_section, middle_section, axes="xy", max_dist=0.001, name="inner section centered in mast")
        ctx.expect_origin_distance(top_section, inner_section, axes="xy", max_dist=0.001, name="top section centered in mast")
        ctx.expect_origin_distance(pan_head, top_section, axes="xy", max_dist=0.001, name="pan head centered on mast")

    with ctx.pose(
        {
            base_to_middle: MIDDLE_TRAVEL,
            middle_to_inner: INNER_TRAVEL,
            inner_to_top: TOP_TRAVEL,
            top_to_pan_head: 1.1,
        }
    ):
        ctx.expect_contact(base_column, middle_section, contact_tol=0.0015, name="middle section stays captured when extended")
        ctx.expect_contact(middle_section, inner_section, contact_tol=0.0015, name="inner section stays captured when extended")
        ctx.expect_contact(inner_section, top_section, contact_tol=0.0015, name="top section stays captured when extended")
        ctx.expect_contact(top_section, pan_head, contact_tol=0.0015, name="pan head remains seated at full extension")
        ctx.expect_gap(middle_section, base_column, axis="z", min_gap=-0.34, max_gap=-0.16, name="middle retains believable overlap")
        ctx.expect_gap(inner_section, middle_section, axis="z", min_gap=-0.34, max_gap=-0.16, name="inner retains believable overlap")
        ctx.expect_gap(top_section, inner_section, axis="z", min_gap=-0.30, max_gap=-0.14, name="top retains believable overlap")
        ctx.expect_origin_distance(pan_head, top_section, axes="xy", max_dist=0.001, name="pan axis stays concentric at full extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
