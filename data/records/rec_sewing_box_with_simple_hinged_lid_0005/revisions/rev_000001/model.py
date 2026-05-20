from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.340
BODY_D = 0.240
BODY_H = 0.160
WALL_T = 0.012
FLOOR_T = 0.014
RIM_H = 0.012

HINGE_AXIS_Y = BODY_D * 0.5 + 0.012
HINGE_AXIS_Z = BODY_H
HINGE_BARREL_R = 0.010
HINGE_BARREL_L = 0.046
HINGE_SUPPORT_W = 0.050
HINGE_SUPPORT_D = 0.024
HINGE_SUPPORT_H = 0.044
HINGE_SUPPORT_X = 0.081

LID_W = BODY_W + 0.004
LID_D = 0.234
LID_PLATE_T = 0.010
LID_FRAME_H = 0.012
LID_LIP_T = 0.008
LID_LIP_H = 0.018
LID_BARREL_L = 0.072


def _cylinder_origin(xyz: tuple[float, float, float], axis: str) -> Origin:
    if axis == "x":
        return Origin(xyz=xyz, rpy=(0.0, pi * 0.5, 0.0))
    if axis == "y":
        return Origin(xyz=xyz, rpy=(pi * 0.5, 0.0, 0.0))
    return Origin(xyz=xyz)


def _add_fastener(
    part,
    *,
    name: str,
    xyz: tuple[float, float, float],
    axis: str,
    radius: float,
    length: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_cylinder_origin(xyz, axis),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_sewing_box", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.31, 0.37, 0.25, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.35, 0.40, 0.28, 1.0))
    molded_black = model.material("molded_black", rgba=(0.13, 0.14, 0.15, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    interior_gray = model.material("interior_gray", rgba=(0.48, 0.49, 0.50, 1.0))

    inner_w = BODY_W - 2.0 * WALL_T
    inner_d = BODY_D - 2.0 * WALL_T

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )
    body.visual(
        Box((BODY_W, BODY_D, FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T * 0.5)),
        material=interior_gray,
        name="floor",
    )
    body.visual(
        Box((BODY_W, WALL_T, BODY_H - FLOOR_T)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + WALL_T * 0.5, FLOOR_T + (BODY_H - FLOOR_T) * 0.5)),
        material=body_paint,
        name="front_wall",
    )
    body.visual(
        Box((BODY_W, WALL_T, BODY_H - FLOOR_T)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - WALL_T * 0.5, FLOOR_T + (BODY_H - FLOOR_T) * 0.5)),
        material=body_paint,
        name="rear_wall",
    )
    body.visual(
        Box((WALL_T, inner_d, BODY_H - FLOOR_T)),
        origin=Origin(xyz=(BODY_W * 0.5 - WALL_T * 0.5, 0.0, FLOOR_T + (BODY_H - FLOOR_T) * 0.5)),
        material=body_paint,
        name="right_wall",
    )
    body.visual(
        Box((WALL_T, inner_d, BODY_H - FLOOR_T)),
        origin=Origin(xyz=(-BODY_W * 0.5 + WALL_T * 0.5, 0.0, FLOOR_T + (BODY_H - FLOOR_T) * 0.5)),
        material=body_paint,
        name="left_wall",
    )

    body.visual(
        Box((inner_w, 0.018, RIM_H)),
        origin=Origin(xyz=(0.0, -0.099, BODY_H - RIM_H * 0.5)),
        material=body_paint,
        name="front_rim",
    )
    body.visual(
        Box((0.018, 0.184, RIM_H)),
        origin=Origin(xyz=(0.149, -0.002, BODY_H - RIM_H * 0.5)),
        material=body_paint,
        name="right_rim",
    )
    body.visual(
        Box((0.018, 0.184, RIM_H)),
        origin=Origin(xyz=(-0.149, -0.002, BODY_H - RIM_H * 0.5)),
        material=body_paint,
        name="left_rim",
    )
    body.visual(
        Box((0.092, 0.018, RIM_H)),
        origin=Origin(xyz=(0.112, 0.099, BODY_H - RIM_H * 0.5)),
        material=body_paint,
        name="rear_rim_right",
    )
    body.visual(
        Box((0.092, 0.018, RIM_H)),
        origin=Origin(xyz=(-0.112, 0.099, BODY_H - RIM_H * 0.5)),
        material=body_paint,
        name="rear_rim_left",
    )

    for prefix, x_sign, y_sign in (
        ("front_right_lower", 1.0, -1.0),
        ("front_left_lower", -1.0, -1.0),
        ("rear_right_lower", 1.0, 1.0),
        ("rear_left_lower", -1.0, 1.0),
    ):
        body.visual(
            Box((0.032, 0.032, 0.070)),
            origin=Origin(xyz=(x_sign * 0.154, y_sign * 0.104, 0.055)),
            material=molded_black,
            name=f"{prefix}_guard",
        )
        body.visual(
            Box((0.028, 0.028, 0.036)),
            origin=Origin(xyz=(x_sign * 0.156, y_sign * 0.106, 0.142)),
            material=molded_black,
            name=f"{prefix}_cap",
        )

    body.visual(
        Box((0.086, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.119, 0.092)),
        material=molded_black,
        name="front_service_pad",
    )
    body.visual(
        Box((0.010, 0.090, 0.040)),
        origin=Origin(xyz=(0.169, 0.0, 0.090)),
        material=molded_black,
        name="right_service_pad",
    )
    body.visual(
        Box((0.010, 0.090, 0.040)),
        origin=Origin(xyz=(-0.169, 0.0, 0.090)),
        material=molded_black,
        name="left_service_pad",
    )

    body.visual(
        Box((HINGE_SUPPORT_W, HINGE_SUPPORT_D, HINGE_SUPPORT_H)),
        origin=Origin(xyz=(HINGE_SUPPORT_X, 0.126, 0.142)),
        material=molded_black,
        name="hinge_support_right",
    )
    body.visual(
        Box((HINGE_SUPPORT_W, HINGE_SUPPORT_D, HINGE_SUPPORT_H)),
        origin=Origin(xyz=(-HINGE_SUPPORT_X, 0.126, 0.142)),
        material=molded_black,
        name="hinge_support_left",
    )
    body.visual(
        Box((0.022, HINGE_SUPPORT_D, 0.030)),
        origin=Origin(xyz=(HINGE_SUPPORT_X, 0.126, 0.110)),
        material=molded_black,
        name="hinge_gusset_right",
    )
    body.visual(
        Box((0.022, HINGE_SUPPORT_D, 0.030)),
        origin=Origin(xyz=(-HINGE_SUPPORT_X, 0.126, 0.110)),
        material=molded_black,
        name="hinge_gusset_left",
    )
    body.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=_cylinder_origin((HINGE_SUPPORT_X, HINGE_AXIS_Y, HINGE_AXIS_Z), "x"),
        material=hinge_steel,
        name="hinge_barrel_right",
    )
    body.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=_cylinder_origin((-HINGE_SUPPORT_X, HINGE_AXIS_Y, HINGE_AXIS_Z), "x"),
        material=hinge_steel,
        name="hinge_barrel_left",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=_cylinder_origin((0.107, HINGE_AXIS_Y, HINGE_AXIS_Z), "x"),
        material=fastener_steel,
        name="hinge_pin_cap_right",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=_cylinder_origin((-0.107, HINGE_AXIS_Y, HINGE_AXIS_Z), "x"),
        material=fastener_steel,
        name="hinge_pin_cap_left",
    )

    for index, x in enumerate((-0.108, -0.036, 0.036, 0.108), start=1):
        _add_fastener(
            body,
            name=f"front_fastener_{index}",
            xyz=(x, -0.1215, 0.104),
            axis="y",
            radius=0.005,
            length=0.003,
            material=fastener_steel,
        )
    for index, y in enumerate((-0.045, 0.0, 0.045), start=1):
        _add_fastener(
            body,
            name=f"right_pad_fastener_{index}",
            xyz=(0.1705, y, 0.094),
            axis="x",
            radius=0.0045,
            length=0.003,
            material=fastener_steel,
        )
        _add_fastener(
            body,
            name=f"left_pad_fastener_{index}",
            xyz=(-0.1705, y, 0.094),
            axis="x",
            radius=0.0045,
            length=0.003,
            material=fastener_steel,
        )
    for index, x in enumerate((-HINGE_SUPPORT_X, HINGE_SUPPORT_X), start=1):
        _add_fastener(
            body,
            name=f"hinge_support_bolt_{index}",
            xyz=(x, 0.126, 0.1515),
            axis="z",
            radius=0.0045,
            length=0.003,
            material=fastener_steel,
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D + 0.030, 0.040)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -0.120, 0.012)),
    )
    lid.visual(
        Box((LID_W, LID_D, LID_PLATE_T)),
        origin=Origin(xyz=(0.0, -0.135, LID_PLATE_T * 0.5)),
        material=lid_paint,
        name="lid_plate",
    )
    lid.visual(
        Box((0.304, 0.036, LID_FRAME_H)),
        origin=Origin(xyz=(0.0, -0.228, 0.016)),
        material=lid_paint,
        name="front_frame",
    )
    lid.visual(
        Box((0.024, 0.186, LID_FRAME_H)),
        origin=Origin(xyz=(0.160, -0.130, 0.016)),
        material=lid_paint,
        name="right_frame",
    )
    lid.visual(
        Box((0.024, 0.186, LID_FRAME_H)),
        origin=Origin(xyz=(-0.160, -0.130, 0.016)),
        material=lid_paint,
        name="left_frame",
    )
    lid.visual(
        Box((0.090, 0.024, LID_FRAME_H)),
        origin=Origin(xyz=(0.108, -0.030, 0.016)),
        material=lid_paint,
        name="rear_frame_right",
    )
    lid.visual(
        Box((0.090, 0.024, LID_FRAME_H)),
        origin=Origin(xyz=(-0.108, -0.030, 0.016)),
        material=lid_paint,
        name="rear_frame_left",
    )
    lid.visual(
        Box((0.088, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -0.003, 0.012)),
        material=molded_black,
        name="hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=HINGE_BARREL_R, length=LID_BARREL_L),
        origin=_cylinder_origin((0.0, 0.0, 0.0), "x"),
        material=hinge_steel,
        name="lid_barrel",
    )
    lid.visual(
        Box((0.180, LID_LIP_T, LID_LIP_H)),
        origin=Origin(xyz=(0.0, -0.205, -0.009)),
        material=interior_gray,
        name="front_lip",
    )
    lid.visual(
        Box((0.012, 0.070, 0.028)),
        origin=Origin(xyz=(0.082, -0.150, -0.014)),
        material=interior_gray,
        name="right_locator_rib",
    )
    lid.visual(
        Box((0.012, 0.070, 0.028)),
        origin=Origin(xyz=(-0.082, -0.150, -0.014)),
        material=interior_gray,
        name="left_locator_rib",
    )

    lid.visual(
        Box((0.038, 0.038, 0.014)),
        origin=Origin(xyz=(0.142, -0.218, 0.017)),
        material=molded_black,
        name="front_right_bumper",
    )
    lid.visual(
        Box((0.038, 0.038, 0.014)),
        origin=Origin(xyz=(-0.142, -0.218, 0.017)),
        material=molded_black,
        name="front_left_bumper",
    )
    lid.visual(
        Box((0.070, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, -0.138, 0.015)),
        material=molded_black,
        name="top_stiffener",
    )
    lid.visual(
        Box((0.032, 0.018, 0.020)),
        origin=Origin(xyz=(0.076, -0.136, 0.020)),
        material=molded_black,
        name="handle_mount_right",
    )
    lid.visual(
        Box((0.032, 0.018, 0.020)),
        origin=Origin(xyz=(-0.076, -0.136, 0.020)),
        material=molded_black,
        name="handle_mount_left",
    )
    lid.visual(
        _save_mesh(
            "sewing_box_handle.obj",
            tube_from_spline_points(
                [
                    (-0.076, -0.136, 0.014),
                    (-0.076, -0.136, 0.046),
                    (-0.040, -0.136, 0.074),
                    (0.0, -0.136, 0.082),
                    (0.040, -0.136, 0.074),
                    (0.076, -0.136, 0.046),
                    (0.076, -0.136, 0.014),
                ],
                radius=0.008,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=molded_black,
        name="carry_handle",
    )

    for index, x in enumerate((-0.118, -0.040, 0.040, 0.118), start=1):
        _add_fastener(
            lid,
            name=f"lid_front_fastener_{index}",
            xyz=(x, -0.228, 0.0235),
            axis="z",
            radius=0.0045,
            length=0.003,
            material=fastener_steel,
        )
    for index, y in enumerate((-0.185, -0.110, -0.055), start=1):
        _add_fastener(
            lid,
            name=f"lid_right_fastener_{index}",
            xyz=(0.160, y, 0.0235),
            axis="z",
            radius=0.004,
            length=0.003,
            material=fastener_steel,
        )
        _add_fastener(
            lid,
            name=f"lid_left_fastener_{index}",
            xyz=(-0.160, y, 0.0235),
            axis="z",
            radius=0.004,
            length=0.003,
            material=fastener_steel,
        )
    _add_fastener(
        lid,
        name="hinge_bridge_fastener",
        xyz=(0.0, -0.006, 0.0205),
        axis="z",
        radius=0.0045,
        length=0.003,
        material=fastener_steel,
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=2.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    floor = body.get_visual("floor")
    front_wall = body.get_visual("front_wall")
    left_wall = body.get_visual("left_wall")
    right_wall = body.get_visual("right_wall")
    front_rim = body.get_visual("front_rim")
    left_barrel = body.get_visual("hinge_barrel_left")
    right_barrel = body.get_visual("hinge_barrel_right")
    left_support = body.get_visual("hinge_support_left")
    right_support = body.get_visual("hinge_support_right")

    lid_plate = lid.get_visual("lid_plate")
    front_lip = lid.get_visual("front_lip")
    left_rib = lid.get_visual("left_locator_rib")
    right_rib = lid.get_visual("right_locator_rib")
    lid_barrel = lid.get_visual("lid_barrel")
    hinge_bridge = lid.get_visual("hinge_bridge")
    handle_mount_left = lid.get_visual("handle_mount_left")
    handle_mount_right = lid.get_visual("handle_mount_right")
    carry_handle = lid.get_visual("carry_handle")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="lid_motion_clearance")

    ctx.check(
        "body_and_lid_present",
        body is not None and lid is not None and lid_hinge is not None,
        "Expected articulated body and lid assembly.",
    )
    ctx.check(
        "lid_hinge_axis",
        tuple(round(v, 3) for v in lid_hinge.axis) == (-1.0, 0.0, 0.0),
        f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "lid_hinge_limits",
        abs(lid_hinge.motion_limits.lower - 0.0) < 1e-9 and lid_hinge.motion_limits.upper >= 2.0,
        f"limits=({lid_hinge.motion_limits.lower}, {lid_hinge.motion_limits.upper})",
    )

    ctx.expect_contact(
        lid,
        body,
        elem_a=lid_plate,
        elem_b=front_rim,
        contact_tol=5e-4,
        name="lid_plate_seats_on_front_rim",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.220, name="lid_covers_body_footprint")
    ctx.expect_gap(
        lid,
        body,
        axis="y",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem=front_lip,
        negative_elem=front_wall,
        name="front_lip_clears_front_wall",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="x",
        min_gap=0.060,
        max_gap=0.080,
        positive_elem=right_wall,
        negative_elem=right_rib,
        name="right_locator_rib_stays_inside_sidewall",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        min_gap=0.060,
        max_gap=0.080,
        positive_elem=left_rib,
        negative_elem=left_wall,
        name="left_locator_rib_stays_inside_sidewall",
    )
    ctx.expect_within(
        lid,
        body,
        axes="xy",
        inner_elem=right_rib,
        outer_elem=floor,
        margin=0.0,
        name="right_locator_rib_within_body_opening",
    )
    ctx.expect_within(
        lid,
        body,
        axes="xy",
        inner_elem=left_rib,
        outer_elem=floor,
        margin=0.0,
        name="left_locator_rib_within_body_opening",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.018,
        elem_a=lid_barrel,
        elem_b=left_barrel,
        name="lid_barrel_aligns_with_left_support",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.018,
        elem_a=lid_barrel,
        elem_b=right_barrel,
        name="lid_barrel_aligns_with_right_support",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        min_overlap=0.080,
        elem_a=hinge_bridge,
        elem_b=floor,
        name="lid_bridge_spans_body_centerline",
    )
    ctx.expect_contact(
        lid,
        lid,
        elem_a=handle_mount_left,
        elem_b=lid_plate,
        contact_tol=1e-6,
        name="left_handle_mount_seats_on_lid",
    )
    ctx.expect_contact(
        lid,
        lid,
        elem_a=handle_mount_right,
        elem_b=lid_plate,
        contact_tol=1e-6,
        name="right_handle_mount_seats_on_lid",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="x",
        min_overlap=0.140,
        elem_a=carry_handle,
        elem_b=lid_plate,
        name="carry_handle_spans_lid_top",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="z",
        min_overlap=0.040,
        elem_a=left_support,
        elem_b=right_support,
        name="hinge_supports_match_height",
    )

    with ctx.pose({lid_hinge: 1.20}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.100,
            positive_elem=front_lip,
            negative_elem=front_rim,
            name="open_lid_lifts_clear_of_opening",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.018,
            elem_a=lid_barrel,
            elem_b=left_barrel,
            name="open_lid_keeps_left_pivot_alignment",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.018,
            elem_a=lid_barrel,
            elem_b=right_barrel,
            name="open_lid_keeps_right_pivot_alignment",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
