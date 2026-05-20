from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.008
BASE_LAYER_Y = 0.008
INNER_LAYER_Y = 0.0
OUTER_LAYER_Y = 0.008
TOP_LAYER_Y = 0.016
JOINT_HOLE_RADIUS = 0.0085


def _extrude_profile_xz(profile: cq.Workplane, thickness: float, y_center: float) -> cq.Workplane:
    return profile.extrude(thickness).translate((0.0, y_center - (thickness / 2.0), 0.0))


def _disc_xz(center_xz: tuple[float, float], radius: float, thickness: float, y_center: float) -> cq.Workplane:
    return _extrude_profile_xz(
        cq.Workplane("XZ").center(center_xz[0], center_xz[1]).circle(radius),
        thickness,
        y_center,
    )


def _bridge_xz(
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    start_width: float,
    end_width: float,
    thickness: float,
    y_center: float,
) -> cq.Workplane:
    dx = end_xz[0] - start_xz[0]
    dz = end_xz[1] - start_xz[1]
    span = math.hypot(dx, dz)
    if span <= 1e-9:
        return _disc_xz(start_xz, max(start_width, end_width) * 0.5, thickness, y_center)

    px = -dz / span
    pz = dx / span
    points = [
        (start_xz[0] + px * (start_width * 0.5), start_xz[1] + pz * (start_width * 0.5)),
        (end_xz[0] + px * (end_width * 0.5), end_xz[1] + pz * (end_width * 0.5)),
        (end_xz[0] - px * (end_width * 0.5), end_xz[1] - pz * (end_width * 0.5)),
        (start_xz[0] - px * (start_width * 0.5), start_xz[1] - pz * (start_width * 0.5)),
    ]
    return _extrude_profile_xz(cq.Workplane("XZ").polyline(points).close(), thickness, y_center)


def _lobed_bar(
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    *,
    start_radius: float,
    end_radius: float,
    start_width: float,
    end_width: float,
    thickness: float,
    y_center: float,
) -> cq.Workplane:
    solid = _disc_xz(start_xz, start_radius, thickness, y_center)
    solid = solid.union(_bridge_xz(start_xz, end_xz, start_width, end_width, thickness, y_center))
    solid = solid.union(_disc_xz(end_xz, end_radius, thickness, y_center))
    return solid


def _hole_cutter(center_xz: tuple[float, float], radius: float, depth: float = 0.040) -> cq.Workplane:
    return _extrude_profile_xz(cq.Workplane("XZ").center(center_xz[0], center_xz[1]).circle(radius), depth, 0.0)


def _lerp_xz(a: tuple[float, float], b: tuple[float, float], t: float) -> tuple[float, float]:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _make_link_plate(
    mid_xz: tuple[float, float],
    distal_xz: tuple[float, float],
    *,
    y_center: float,
    root_radius: float,
    mid_radius: float,
    distal_radius: float,
    root_width: float,
    mid_width: float,
    distal_width: float,
    cut_root_hole: bool = True,
    cut_distal_hole: bool = True,
) -> cq.Workplane:
    shape = _lobed_bar(
        (0.0, 0.0),
        mid_xz,
        start_radius=root_radius,
        end_radius=mid_radius,
        start_width=root_width,
        end_width=mid_width,
        thickness=PLATE_THICKNESS,
        y_center=y_center,
    )
    shape = shape.union(
        _lobed_bar(
            mid_xz,
            distal_xz,
            start_radius=mid_radius,
            end_radius=distal_radius,
            start_width=mid_width,
            end_width=distal_width,
            thickness=PLATE_THICKNESS,
            y_center=y_center,
        )
    )
    if cut_root_hole:
        shape = shape.cut(_hole_cutter((0.0, 0.0), JOINT_HOLE_RADIUS, depth=0.030))
    if cut_distal_hole:
        shape = shape.cut(_hole_cutter(distal_xz, JOINT_HOLE_RADIUS, depth=0.030))
    return shape


def _make_terminal_link(
    mid_xz: tuple[float, float],
    tip_xz: tuple[float, float],
    *,
    y_center: float,
    root_radius: float,
    mid_radius: float,
    root_width: float,
    mid_width: float,
    tip_radius: float,
    tip_width: float,
) -> cq.Workplane:
    shape = _make_link_plate(
        mid_xz,
        tip_xz,
        y_center=y_center,
        root_radius=root_radius,
        mid_radius=mid_radius,
        distal_radius=tip_radius,
        root_width=root_width,
        mid_width=mid_width,
        distal_width=tip_width,
        cut_root_hole=True,
        cut_distal_hole=False,
    )
    return shape.union(_disc_xz(tip_xz, tip_radius * 0.90, PLATE_THICKNESS, y_center))


def _make_base_support() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.24, 0.12, 0.020).translate((-0.03, 0.0, 0.010))
    heel_block = cq.Workplane("XY").box(0.082, 0.084, 0.050).translate((-0.104, 0.0, 0.035))
    toe_block = cq.Workplane("XY").box(0.055, 0.046, 0.030).translate((0.040, 0.0, 0.035))
    upright_web = cq.Workplane("XY").box(0.030, 0.022, 0.086).translate((-0.060, 0.0, 0.083))

    side_bracket = _make_link_plate(
        (-0.030, 0.118),
        (0.0, 0.170),
        y_center=BASE_LAYER_Y,
        root_radius=0.024,
        mid_radius=0.017,
        distal_radius=0.020,
        root_width=0.038,
        mid_width=0.024,
        distal_width=0.026,
        cut_root_hole=False,
        cut_distal_hole=True,
    )
    rear_brace = _lobed_bar(
        (-0.070, 0.060),
        (-0.030, 0.118),
        start_radius=0.020,
        end_radius=0.015,
        start_width=0.028,
        end_width=0.020,
        thickness=PLATE_THICKNESS,
        y_center=BASE_LAYER_Y,
    )

    return foot.union(heel_block).union(toe_block).union(upright_web).union(side_bracket).union(rear_brace)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_chain")

    model.material("base_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("link_root", rgba=(0.43, 0.47, 0.54, 1.0))
    model.material("link_mid", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("link_tip", rgba=(0.64, 0.54, 0.40, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_support(), "offset_chain_base"),
        material="base_metal",
        name="base_shell",
    )

    link_1_shape = _make_link_plate(
        (0.100, 0.024),
        (0.205, 0.055),
        y_center=TOP_LAYER_Y,
        root_radius=0.024,
        mid_radius=0.018,
        distal_radius=0.022,
        root_width=0.040,
        mid_width=0.028,
        distal_width=0.032,
    )
    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(link_1_shape, "offset_chain_link_1"),
        material="link_root",
        name="link_1_shell",
    )

    link_2_shape = _make_link_plate(
        (0.090, -0.018),
        (0.182, -0.067),
        y_center=OUTER_LAYER_Y,
        root_radius=0.022,
        mid_radius=0.016,
        distal_radius=0.019,
        root_width=0.034,
        mid_width=0.024,
        distal_width=0.026,
    )
    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(link_2_shape, "offset_chain_link_2"),
        material="link_mid",
        name="link_2_shell",
    )

    link_3_shape = _make_link_plate(
        (0.074, 0.018),
        (0.155, 0.048),
        y_center=INNER_LAYER_Y,
        root_radius=0.019,
        mid_radius=0.014,
        distal_radius=0.017,
        root_width=0.030,
        mid_width=0.022,
        distal_width=0.024,
    )
    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(link_3_shape, "offset_chain_link_3"),
        material="link_mid",
        name="link_3_shell",
    )

    link_4_shape = _make_terminal_link(
        (0.060, -0.010),
        (0.128, -0.026),
        y_center=OUTER_LAYER_Y,
        root_radius=0.023,
        mid_radius=0.017,
        root_width=0.035,
        mid_width=0.023,
        tip_radius=0.034,
        tip_width=0.052,
    )
    link_4 = model.part("link_4")
    link_4.visual(
        mesh_from_cadquery(link_4_shape, "offset_chain_link_4"),
        material="link_tip",
        name="link_4_shell",
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=-1.0, upper=0.95),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.205, 0.0, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-1.35, upper=1.15),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.182, 0.0, -0.067)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.15, upper=1.25),
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(0.155, 0.0, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-1.05, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")

    joints = [
        object_model.get_articulation("base_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
        object_model.get_articulation("link_3_to_link_4"),
    ]

    ctx.allow_overlap(
        base,
        link_1,
        reason=(
            "The grounded bracket and first rocker intentionally share a compact "
            "hinge-journal envelope; the axle/bushing hardware is omitted as a "
            "separate modeled part."
        ),
    )

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

    for joint in joints:
        ctx.check(
            f"{joint.name}_is_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{joint.name} should be a revolute joint.",
        )
        ctx.check(
            f"{joint.name}_uses_single_motion_plane",
            tuple(round(value, 6) for value in joint.axis) == (0.0, -1.0, 0.0),
            f"{joint.name} axis was {joint.axis}, expected a shared -Y in-plane axis.",
        )

    ctx.expect_contact(base, link_1, contact_tol=1e-5, name="base_supports_link_1")
    ctx.expect_contact(link_1, link_2, contact_tol=1e-5, name="link_1_supports_link_2")
    ctx.expect_contact(link_2, link_3, contact_tol=1e-5, name="link_2_supports_link_3")
    ctx.expect_contact(link_3, link_4, contact_tol=1e-5, name="link_3_supports_link_4")

    rest_tip = ctx.part_world_position(link_4)
    with ctx.pose(base_to_link_1=0.55):
        lifted_tip = ctx.part_world_position(link_4)
    ctx.check(
        "positive_root_rotation_lifts_chain_tip",
        rest_tip is not None and lifted_tip is not None and lifted_tip[2] > rest_tip[2] + 0.04,
        f"Expected positive root rotation to raise the tip. Rest={rest_tip}, lifted={lifted_tip}",
    )

    with ctx.pose(
        base_to_link_1=0.35,
        link_1_to_link_2=-0.30,
        link_2_to_link_3=0.28,
        link_3_to_link_4=-0.18,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_flexed_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
