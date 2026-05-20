from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROOT_Z = 0.145

LINK1_TIP_XZ = (0.240, 0.055)
LINK2_TIP_XZ = (0.210, -0.042)

BASE_CHEEK_THICKNESS = 0.008
LINK1_ROOT_THICKNESS = 0.010
LINK1_FORK_THICKNESS = 0.007
LINK2_ROOT_THICKNESS = 0.009
LINK2_FORK_THICKNESS = 0.006
LINK3_THICKNESS = 0.008

BASE_CHEEK_OFFSET_Y = LINK1_ROOT_THICKNESS / 2.0 + BASE_CHEEK_THICKNESS / 2.0
LINK1_FORK_OFFSET_Y = LINK2_ROOT_THICKNESS / 2.0 + LINK1_FORK_THICKNESS / 2.0
LINK1_FORK_TOTAL_WIDTH = 2.0 * (LINK1_FORK_OFFSET_Y + LINK1_FORK_THICKNESS / 2.0)
LINK2_FORK_OFFSET_Y = LINK3_THICKNESS / 2.0 + LINK2_FORK_THICKNESS / 2.0
LINK2_FORK_TOTAL_WIDTH = 2.0 * (LINK2_FORK_OFFSET_Y + LINK2_FORK_THICKNESS / 2.0)


def _y_cylinder(radius: float, thickness: float, x: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(thickness, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y, z))
    )


def _box_between_xz(
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    size_y: float,
    size_z: float,
    *,
    y: float = 0.0,
    extra_length: float = 0.0,
) -> cq.Workplane:
    dx = end_xz[0] - start_xz[0]
    dz = end_xz[1] - start_xz[1]
    length = math.hypot(dx, dz) + extra_length
    angle_deg = -math.degrees(math.atan2(dz, dx))
    center_x = 0.5 * (start_xz[0] + end_xz[0])
    center_z = 0.5 * (start_xz[1] + end_xz[1])
    return (
        cq.Workplane("XY")
        .box(length, size_y, size_z)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        .translate((center_x, y, center_z))
    )


def _combine(solids: list[cq.Workplane]) -> cq.Workplane:
    shape = solids[0]
    for solid in solids[1:]:
        shape = shape.union(solid)
    return shape


def _bar_origin_between_xz(
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    *,
    y: float = 0.0,
) -> tuple[Origin, float]:
    dx = end_xz[0] - start_xz[0]
    dz = end_xz[1] - start_xz[1]
    length = math.hypot(dx, dz)
    angle = -math.atan2(dz, dx)
    return Origin(
        xyz=((start_xz[0] + end_xz[0]) / 2.0, y, (start_xz[1] + end_xz[1]) / 2.0),
        rpy=(0.0, angle, 0.0),
    ), length


def _add_bar(
    part_obj,
    name: str,
    *,
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    size_y: float,
    size_z: float,
    y: float = 0.0,
    material: str,
) -> None:
    origin, length = _bar_origin_between_xz(start_xz, end_xz, y=y)
    part_obj.visual(
        Box((length, size_y, size_z)),
        origin=origin,
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part_obj,
    name: str,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part_obj.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _make_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.310, 0.150, 0.028).translate((0.0, 0.0, 0.014))
    heel_block = cq.Workplane("XY").box(0.078, 0.106, 0.058).translate((-0.092, 0.0, 0.057))
    rear_brace = _box_between_xz(
        (-0.112, 0.028),
        (-0.056, 0.074),
        0.092,
        0.026,
        y=0.0,
        extra_length=0.008,
    )
    toe_pad = cq.Workplane("XY").box(0.084, 0.092, 0.018).translate((0.070, 0.0, 0.023))

    def cheek(y_center: float) -> cq.Workplane:
        cheek_shape = _combine(
            [
                _y_cylinder(0.028, BASE_CHEEK_THICKNESS, 0.0, y_center, ROOT_Z),
                cq.Workplane("XY")
                .box(0.054, BASE_CHEEK_THICKNESS, 0.038)
                .translate((-0.080, y_center, 0.080)),
                _box_between_xz(
                    (-0.060, 0.090),
                    (-0.012, ROOT_Z - 0.014),
                    BASE_CHEEK_THICKNESS,
                    0.026,
                    y=y_center,
                    extra_length=0.010,
                ),
                _box_between_xz(
                    (-0.016, ROOT_Z - 0.012),
                    (0.020, ROOT_Z - 0.004),
                    BASE_CHEEK_THICKNESS,
                    0.014,
                    y=y_center,
                    extra_length=0.004,
                ),
            ]
        )
        return cheek_shape.cut(_y_cylinder(0.0075, BASE_CHEEK_THICKNESS + 0.002, 0.0, y_center, ROOT_Z))

    left_cheek = cheek(BASE_CHEEK_OFFSET_Y)
    right_cheek = cheek(-BASE_CHEEK_OFFSET_Y)

    return _combine([foot, heel_block, rear_brace, left_cheek, right_cheek, toe_pad])


def _make_link1_shape() -> cq.Workplane:
    body = _combine(
        [
            _y_cylinder(0.022, LINK1_ROOT_THICKNESS, 0.0, 0.0, 0.0),
            _box_between_xz(
                (0.018, 0.004),
                (0.108, 0.024),
                LINK1_ROOT_THICKNESS,
                0.028,
                y=0.0,
                extra_length=0.010,
            ),
            _box_between_xz(
                (0.108, 0.024),
                (0.170, 0.036),
                LINK1_ROOT_THICKNESS,
                0.021,
                y=0.0,
                extra_length=0.008,
            ),
            _box_between_xz(
                (0.050, -0.005),
                (0.154, 0.012),
                LINK1_ROOT_THICKNESS,
                0.012,
                y=0.0,
                extra_length=0.008,
            ),
            cq.Workplane("XY").box(0.026, LINK1_FORK_TOTAL_WIDTH, 0.016).translate((0.178, 0.0, 0.038)),
        ]
    )

    def tip_cheek(y_center: float) -> cq.Workplane:
        return _combine(
            [
                _y_cylinder(0.019, LINK1_FORK_THICKNESS, LINK1_TIP_XZ[0], y_center, LINK1_TIP_XZ[1]),
                _box_between_xz(
                    (0.178, 0.038),
                    LINK1_TIP_XZ,
                    LINK1_FORK_THICKNESS,
                    0.016,
                    y=y_center,
                    extra_length=0.010,
                ),
                _box_between_xz(
                    (0.198, 0.046),
                    (0.228, 0.054),
                    LINK1_FORK_THICKNESS,
                    0.010,
                    y=y_center,
                    extra_length=0.004,
                ),
            ]
        )

    link = _combine([body, tip_cheek(LINK1_FORK_OFFSET_Y), tip_cheek(-LINK1_FORK_OFFSET_Y)])
    link = link.cut(_y_cylinder(0.007, LINK1_ROOT_THICKNESS + 0.002, 0.0, 0.0, 0.0))
    link = link.cut(_y_cylinder(0.0065, LINK1_FORK_TOTAL_WIDTH + 0.002, LINK1_TIP_XZ[0], 0.0, LINK1_TIP_XZ[1]))
    link = link.cut(_y_cylinder(0.0055, LINK1_ROOT_THICKNESS + 0.002, 0.102, 0.0, 0.018))
    return link


def _make_link2_shape() -> cq.Workplane:
    body = _combine(
        [
            _y_cylinder(0.018, LINK2_ROOT_THICKNESS, 0.0, 0.0, 0.0),
            _box_between_xz(
                (0.018, -0.002),
                (0.090, -0.018),
                LINK2_ROOT_THICKNESS,
                0.024,
                y=0.0,
                extra_length=0.010,
            ),
            _box_between_xz(
                (0.090, -0.018),
                (0.150, -0.028),
                LINK2_ROOT_THICKNESS,
                0.018,
                y=0.0,
                extra_length=0.008,
            ),
            _box_between_xz(
                (0.060, 0.008),
                (0.138, -0.004),
                LINK2_ROOT_THICKNESS,
                0.010,
                y=0.0,
                extra_length=0.006,
            ),
            cq.Workplane("XY").box(0.024, LINK2_FORK_TOTAL_WIDTH, 0.014).translate((0.160, 0.0, -0.028)),
        ]
    )

    def tip_cheek(y_center: float) -> cq.Workplane:
        return _combine(
            [
                _y_cylinder(0.016, LINK2_FORK_THICKNESS, LINK2_TIP_XZ[0], y_center, LINK2_TIP_XZ[1]),
                _box_between_xz(
                    (0.160, -0.028),
                    LINK2_TIP_XZ,
                    LINK2_FORK_THICKNESS,
                    0.014,
                    y=y_center,
                    extra_length=0.008,
                ),
                _box_between_xz(
                    (0.172, -0.018),
                    (0.198, -0.038),
                    LINK2_FORK_THICKNESS,
                    0.009,
                    y=y_center,
                    extra_length=0.004,
                ),
            ]
        )

    link = _combine([body, tip_cheek(LINK2_FORK_OFFSET_Y), tip_cheek(-LINK2_FORK_OFFSET_Y)])
    link = link.cut(_y_cylinder(0.0065, LINK2_ROOT_THICKNESS + 0.002, 0.0, 0.0, 0.0))
    link = link.cut(_y_cylinder(0.006, LINK2_FORK_TOTAL_WIDTH + 0.002, LINK2_TIP_XZ[0], 0.0, LINK2_TIP_XZ[1]))
    link = link.cut(_y_cylinder(0.0052, LINK2_ROOT_THICKNESS + 0.002, 0.100, 0.0, -0.014))
    return link


def _make_link3_shape() -> cq.Workplane:
    link = _combine(
        [
            _y_cylinder(0.014, LINK3_THICKNESS, 0.0, 0.0, 0.0),
            _box_between_xz(
                (0.016, 0.003),
                (0.086, 0.016),
                LINK3_THICKNESS,
                0.020,
                y=0.0,
                extra_length=0.008,
            ),
            _box_between_xz(
                (0.086, 0.016),
                (0.154, 0.028),
                LINK3_THICKNESS,
                0.015,
                y=0.0,
                extra_length=0.008,
            ),
            _box_between_xz(
                (0.070, 0.006),
                (0.140, 0.020),
                LINK3_THICKNESS,
                0.009,
                y=0.0,
                extra_length=0.006,
            ),
            _box_between_xz(
                (0.124, 0.023),
                (0.178, 0.031),
                LINK3_THICKNESS,
                0.026,
                y=0.0,
                extra_length=0.006,
            ),
            _y_cylinder(0.013, LINK3_THICKNESS, 0.178, 0.0, 0.031),
        ]
    )
    link = link.cut(_y_cylinder(0.006, LINK3_THICKNESS + 0.002, 0.0, 0.0, 0.0))
    link = link.cut(_y_cylinder(0.005, LINK3_THICKNESS + 0.002, 0.074, 0.0, 0.013))
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_three_joint_chain")

    model.material("base_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("brushed_link", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("blue_gray_link", rgba=(0.55, 0.60, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.310, 0.150, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="base_steel",
        name="foot",
    )
    base.visual(
        Box((0.090, 0.110, 0.058)),
        origin=Origin(xyz=(-0.094, 0.0, 0.057)),
        material="base_steel",
        name="heel_block",
    )
    _add_y_cylinder(
        base,
        "left_root_cheek",
        radius=0.024,
        length=BASE_CHEEK_THICKNESS,
        xyz=(0.0, BASE_CHEEK_OFFSET_Y, ROOT_Z),
        material="base_steel",
    )
    _add_y_cylinder(
        base,
        "right_root_cheek",
        radius=0.024,
        length=BASE_CHEEK_THICKNESS,
        xyz=(0.0, -BASE_CHEEK_OFFSET_Y, ROOT_Z),
        material="base_steel",
    )
    base.visual(
        Box((0.040, BASE_CHEEK_THICKNESS, 0.060)),
        origin=Origin(xyz=(-0.042, BASE_CHEEK_OFFSET_Y, 0.114)),
        material="base_steel",
        name="left_cheek_post",
    )
    base.visual(
        Box((0.040, BASE_CHEEK_THICKNESS, 0.060)),
        origin=Origin(xyz=(-0.042, -BASE_CHEEK_OFFSET_Y, 0.114)),
        material="base_steel",
        name="right_cheek_post",
    )
    base.visual(
        Box((0.030, BASE_CHEEK_THICKNESS, 0.020)),
        origin=Origin(xyz=(-0.014, BASE_CHEEK_OFFSET_Y, ROOT_Z - 0.006)),
        material="base_steel",
        name="left_cheek_lug",
    )
    base.visual(
        Box((0.030, BASE_CHEEK_THICKNESS, 0.020)),
        origin=Origin(xyz=(-0.014, -BASE_CHEEK_OFFSET_Y, ROOT_Z - 0.006)),
        material="base_steel",
        name="right_cheek_lug",
    )

    link1 = model.part("link1")
    _add_y_cylinder(
        link1,
        "root_boss",
        radius=0.018,
        length=LINK1_ROOT_THICKNESS,
        xyz=(0.0, 0.0, 0.0),
        material="brushed_link",
    )
    _add_bar(
        link1,
        "main_spine",
        start_xz=(0.016, 0.004),
        end_xz=(0.120, 0.026),
        size_y=LINK1_ROOT_THICKNESS,
        size_z=0.022,
        material="brushed_link",
    )
    _add_bar(
        link1,
        "secondary_rib",
        start_xz=(0.050, -0.004),
        end_xz=(0.154, 0.012),
        size_y=LINK1_ROOT_THICKNESS,
        size_z=0.010,
        material="brushed_link",
    )
    _add_bar(
        link1,
        "fork_stem",
        start_xz=(0.118, 0.026),
        end_xz=(0.182, 0.040),
        size_y=LINK1_ROOT_THICKNESS,
        size_z=0.018,
        material="brushed_link",
    )
    link1.visual(
        Box((0.028, LINK1_FORK_TOTAL_WIDTH, 0.014)),
        origin=Origin(xyz=(0.184, 0.0, 0.040)),
        material="brushed_link",
        name="fork_bridge",
    )
    _add_y_cylinder(
        link1,
        "left_tip_cheek",
        radius=0.017,
        length=LINK1_FORK_THICKNESS,
        xyz=(LINK1_TIP_XZ[0], LINK1_FORK_OFFSET_Y, LINK1_TIP_XZ[1]),
        material="brushed_link",
    )
    _add_y_cylinder(
        link1,
        "right_tip_cheek",
        radius=0.017,
        length=LINK1_FORK_THICKNESS,
        xyz=(LINK1_TIP_XZ[0], -LINK1_FORK_OFFSET_Y, LINK1_TIP_XZ[1]),
        material="brushed_link",
    )
    _add_bar(
        link1,
        "left_cheek_bar",
        start_xz=(0.186, 0.040),
        end_xz=LINK1_TIP_XZ,
        size_y=LINK1_FORK_THICKNESS,
        size_z=0.015,
        y=LINK1_FORK_OFFSET_Y,
        material="brushed_link",
    )
    _add_bar(
        link1,
        "right_cheek_bar",
        start_xz=(0.186, 0.040),
        end_xz=LINK1_TIP_XZ,
        size_y=LINK1_FORK_THICKNESS,
        size_z=0.015,
        y=-LINK1_FORK_OFFSET_Y,
        material="brushed_link",
    )

    link2 = model.part("link2")
    _add_y_cylinder(
        link2,
        "root_boss",
        radius=0.015,
        length=LINK2_ROOT_THICKNESS,
        xyz=(0.0, 0.0, 0.0),
        material="blue_gray_link",
    )
    _add_bar(
        link2,
        "main_spine",
        start_xz=(0.016, -0.002),
        end_xz=(0.096, -0.020),
        size_y=LINK2_ROOT_THICKNESS,
        size_z=0.020,
        material="blue_gray_link",
    )
    link2.visual(
        Box((0.024, LINK2_ROOT_THICKNESS, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, -0.004)),
        material="blue_gray_link",
        name="root_neck",
    )
    _add_bar(
        link2,
        "secondary_rib",
        start_xz=(0.056, 0.007),
        end_xz=(0.140, -0.004),
        size_y=LINK2_ROOT_THICKNESS,
        size_z=0.009,
        material="blue_gray_link",
    )
    link2.visual(
        Box((0.040, LINK2_ROOT_THICKNESS, 0.016)),
        origin=Origin(xyz=(0.090, 0.0, -0.004)),
        material="blue_gray_link",
        name="rib_gusset",
    )
    _add_bar(
        link2,
        "fork_stem",
        start_xz=(0.096, -0.020),
        end_xz=(0.162, -0.030),
        size_y=LINK2_ROOT_THICKNESS,
        size_z=0.016,
        material="blue_gray_link",
    )
    link2.visual(
        Box((0.024, LINK2_FORK_TOTAL_WIDTH, 0.014)),
        origin=Origin(xyz=(0.166, 0.0, -0.030)),
        material="blue_gray_link",
        name="fork_bridge",
    )
    _add_y_cylinder(
        link2,
        "left_tip_cheek",
        radius=0.016,
        length=LINK2_FORK_THICKNESS,
        xyz=(LINK2_TIP_XZ[0], LINK2_FORK_OFFSET_Y, LINK2_TIP_XZ[1]),
        material="blue_gray_link",
    )
    _add_y_cylinder(
        link2,
        "right_tip_cheek",
        radius=0.016,
        length=LINK2_FORK_THICKNESS,
        xyz=(LINK2_TIP_XZ[0], -LINK2_FORK_OFFSET_Y, LINK2_TIP_XZ[1]),
        material="blue_gray_link",
    )
    _add_bar(
        link2,
        "left_cheek_bar",
        start_xz=(0.168, -0.030),
        end_xz=LINK2_TIP_XZ,
        size_y=LINK2_FORK_THICKNESS,
        size_z=0.012,
        y=LINK2_FORK_OFFSET_Y,
        material="blue_gray_link",
    )
    _add_bar(
        link2,
        "right_cheek_bar",
        start_xz=(0.168, -0.030),
        end_xz=LINK2_TIP_XZ,
        size_y=LINK2_FORK_THICKNESS,
        size_z=0.012,
        y=-LINK2_FORK_OFFSET_Y,
        material="blue_gray_link",
    )

    link3 = model.part("link3")
    _add_y_cylinder(
        link3,
        "root_boss",
        radius=0.013,
        length=LINK3_THICKNESS,
        xyz=(0.0, 0.0, 0.0),
        material="brushed_link",
    )
    _add_bar(
        link3,
        "main_spine",
        start_xz=(0.014, 0.003),
        end_xz=(0.088, 0.016),
        size_y=LINK3_THICKNESS,
        size_z=0.018,
        material="brushed_link",
    )
    link3.visual(
        Box((0.022, LINK3_THICKNESS, 0.015)),
        origin=Origin(xyz=(0.018, 0.0, 0.004)),
        material="brushed_link",
        name="root_neck",
    )
    _add_bar(
        link3,
        "forward_spine",
        start_xz=(0.088, 0.016),
        end_xz=(0.160, 0.029),
        size_y=LINK3_THICKNESS,
        size_z=0.014,
        material="brushed_link",
    )
    _add_bar(
        link3,
        "underslung_rib",
        start_xz=(0.070, 0.006),
        end_xz=(0.142, 0.019),
        size_y=LINK3_THICKNESS,
        size_z=0.008,
        material="brushed_link",
    )
    link3.visual(
        Box((0.032, LINK3_THICKNESS, 0.020)),
        origin=Origin(xyz=(0.168, 0.0, 0.030)),
        material="brushed_link",
        name="tip_pad",
    )
    _add_y_cylinder(
        link3,
        "tip_nose",
        radius=0.012,
        length=LINK3_THICKNESS,
        xyz=(0.180, 0.0, 0.031),
        material="brushed_link",
    )

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, ROOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.15, effort=40.0, velocity=1.4),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_TIP_XZ[0], 0.0, LINK1_TIP_XZ[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.05, upper=0.95, effort=28.0, velocity=1.8),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_TIP_XZ[0], 0.0, LINK2_TIP_XZ[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.00, effort=16.0, velocity=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    joint1 = object_model.get_articulation("base_to_link1")
    joint2 = object_model.get_articulation("link1_to_link2")
    joint3 = object_model.get_articulation("link2_to_link3")

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

    ctx.expect_contact(base, link1, name="base_and_link1_share_root_joint_faces")
    ctx.expect_contact(link1, link2, name="link1_and_link2_share_mid_joint_faces")
    ctx.expect_contact(link2, link3, name="link2_and_link3_share_tip_joint_faces")

    ctx.check(
        "all_joint_axes_pitch_in_one_plane",
        joint1.axis == (0.0, -1.0, 0.0)
        and joint2.axis == (0.0, -1.0, 0.0)
        and joint3.axis == (0.0, -1.0, 0.0),
        details=f"axes were {joint1.axis}, {joint2.axis}, {joint3.axis}",
    )
    ctx.check(
        "serial_chain_progresses_forward_in_rest_pose",
        ctx.part_world_position(link2)[0] > 0.20 and ctx.part_world_position(link3)[0] > 0.42,
        details=(
            f"link2 position={ctx.part_world_position(link2)}, "
            f"link3 position={ctx.part_world_position(link3)}"
        ),
    )

    rest_link3 = ctx.part_world_position(link3)
    with ctx.pose({joint1: 0.55, joint2: 0.35, joint3: 0.30}):
        rocked_link3 = ctx.part_world_position(link3)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_representative_rocked_pose")
        ctx.expect_gap(link3, base, axis="z", min_gap=0.015, name="distal_link_clears_foot_when_rocked")

    ctx.check(
        "rocked_pose_lifts_distal_origin",
        rest_link3 is not None
        and rocked_link3 is not None
        and rocked_link3[2] > rest_link3[2] + 0.08,
        details=f"rest={rest_link3}, rocked={rocked_link3}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
