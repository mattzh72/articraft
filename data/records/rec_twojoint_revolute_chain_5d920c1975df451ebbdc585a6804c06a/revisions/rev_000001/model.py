from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


PLATE_LENGTH = 0.18
PLATE_WIDTH = 0.08
PLATE_THICKNESS = 0.006
LINK_THICKNESS = 0.008

JOINT_RADIUS = 0.010
CENTER_KNUCKLE_WIDTH = 0.010
OUTER_KNUCKLE_WIDTH = 0.006
FORK_WIDTH = CENTER_KNUCKLE_WIDTH + 2.0 * OUTER_KNUCKLE_WIDTH

ROOT_HINGE_X = 0.081
ROOT_HINGE_Z = 0.018

PROXIMAL_LENGTH = 0.108
DISTAL_LENGTH = 0.078
PROXIMAL_ROOT_OFFSET = 0.009


def _extrude_xz_box(
    *,
    x_center: float,
    z_center: float,
    x_size: float,
    z_size: float,
    y_center: float,
    y_size: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_center, z_center)
        .rect(x_size, z_size)
        .extrude(y_size)
        .translate((0.0, y_center - y_size / 2.0, 0.0))
    )


def _extrude_xz_cylinder(
    *,
    x_center: float,
    z_center: float,
    radius: float,
    y_center: float,
    y_size: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_center, z_center)
        .circle(radius)
        .extrude(y_size)
        .translate((0.0, y_center - y_size / 2.0, 0.0))
    )


def _left_ear_center() -> float:
    return -(CENTER_KNUCKLE_WIDTH + OUTER_KNUCKLE_WIDTH) / 2.0


def _right_ear_center() -> float:
    return (CENTER_KNUCKLE_WIDTH + OUTER_KNUCKLE_WIDTH) / 2.0


def _box_centered(
    *,
    x_center: float,
    y_center: float,
    z_center: float,
    x_size: float,
    y_size: float,
    z_size: float,
) -> cq.Workplane:
    return cq.Workplane("XY").box(x_size, y_size, z_size).translate((x_center, y_center, z_center))


def _cylinder_y(
    *,
    x_center: float,
    y_center: float,
    z_center: float,
    radius: float,
    y_size: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(y_size)
        .translate((0.0, 0.0, -y_size / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x_center, y_center, z_center))
    )


def _capsule_along_x(
    *,
    x0: float,
    x1: float,
    y_center: float,
    z_center: float,
    radius: float,
    y_size: float,
) -> cq.Workplane:
    return (
        _box_centered(
            x_center=(x0 + x1) / 2.0,
            y_center=y_center,
            z_center=z_center,
            x_size=max(x1 - x0, 1e-6),
            y_size=y_size,
            z_size=2.0 * radius,
        )
        .union(
            _cylinder_y(
                x_center=x0,
                y_center=y_center,
                z_center=z_center,
                radius=radius,
                y_size=y_size,
            )
        )
        .union(
            _cylinder_y(
                x_center=x1,
                y_center=y_center,
                z_center=z_center,
                radius=radius,
                y_size=y_size,
            )
        )
    )


def _root_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS).translate(
        (0.0, 0.0, PLATE_THICKNESS / 2.0)
    )

    hole_depth = PLATE_THICKNESS + 0.004
    for x_pos in (-0.055, 0.055):
        for y_pos in (-0.024, 0.024):
            cutter = (
                cq.Workplane("XY")
                .center(x_pos, y_pos)
                .circle(0.0045)
                .extrude(hole_depth)
                .translate((0.0, 0.0, -0.002))
            )
            plate = plate.cut(cutter)

    center_riser = _box_centered(
        x_center=0.056,
        y_center=0.0,
        z_center=0.009,
        x_size=0.024,
        y_size=0.024,
        z_size=0.012,
    )
    left_rib = _box_centered(
        x_center=0.064,
        y_center=-0.008,
        z_center=0.012,
        x_size=0.018,
        y_size=OUTER_KNUCKLE_WIDTH,
        z_size=0.012,
    )
    right_rib = _box_centered(
        x_center=0.064,
        y_center=0.008,
        z_center=0.012,
        x_size=0.018,
        y_size=OUTER_KNUCKLE_WIDTH,
        z_size=0.012,
    )
    left_ear = _box_centered(
        x_center=0.0815,
        y_center=-0.008,
        z_center=ROOT_HINGE_Z,
        x_size=0.017,
        y_size=OUTER_KNUCKLE_WIDTH,
        z_size=0.020,
    )
    right_ear = _box_centered(
        x_center=0.0815,
        y_center=0.008,
        z_center=ROOT_HINGE_Z,
        x_size=0.017,
        y_size=OUTER_KNUCKLE_WIDTH,
        z_size=0.020,
    )

    return (
        plate.union(center_riser)
        .union(left_rib)
        .union(right_rib)
        .union(left_ear)
        .union(right_ear)
    )


def _proximal_link_shape() -> cq.Workplane:
    root_tongue = _box_centered(
        x_center=PROXIMAL_ROOT_OFFSET + 0.009,
        y_center=0.0,
        z_center=0.0,
        x_size=0.018,
        y_size=CENTER_KNUCKLE_WIDTH,
        z_size=0.020,
    )
    neck = _box_centered(
        x_center=PROXIMAL_ROOT_OFFSET + 0.020,
        y_center=0.0,
        z_center=0.0,
        x_size=0.010,
        y_size=0.009,
        z_size=0.012,
    )
    main_link = _box_centered(
        x_center=PROXIMAL_ROOT_OFFSET + 0.053,
        y_center=0.0,
        z_center=0.0,
        x_size=0.066,
        y_size=0.008,
        z_size=LINK_THICKNESS,
    )
    clevis_bridge = _box_centered(
        x_center=PROXIMAL_ROOT_OFFSET + 0.091,
        y_center=0.0,
        z_center=0.0,
        x_size=0.010,
        y_size=0.016,
        z_size=0.006,
    )
    left_ear = _box_centered(
        x_center=PROXIMAL_ROOT_OFFSET + 0.101,
        y_center=-0.008,
        z_center=0.0,
        x_size=0.014,
        y_size=OUTER_KNUCKLE_WIDTH,
        z_size=0.020,
    )
    right_ear = _box_centered(
        x_center=PROXIMAL_ROOT_OFFSET + 0.101,
        y_center=0.008,
        z_center=0.0,
        x_size=0.014,
        y_size=OUTER_KNUCKLE_WIDTH,
        z_size=0.020,
    )

    return root_tongue.union(neck).union(main_link).union(clevis_bridge).union(left_ear).union(right_ear)


def _distal_link_shape() -> cq.Workplane:
    root_tongue = _box_centered(
        x_center=0.009,
        y_center=0.0,
        z_center=0.0,
        x_size=0.018,
        y_size=CENTER_KNUCKLE_WIDTH,
        z_size=0.020,
    )
    neck = _box_centered(
        x_center=0.020,
        y_center=0.0,
        z_center=0.0,
        x_size=0.010,
        y_size=0.008,
        z_size=0.012,
    )
    body = _box_centered(
        x_center=0.040,
        y_center=0.0,
        z_center=0.0,
        x_size=0.030,
        y_size=0.008,
        z_size=LINK_THICKNESS,
    )
    end_tab = _box_centered(
        x_center=0.065,
        y_center=0.0,
        z_center=0.0,
        x_size=0.020,
        y_size=0.008,
        z_size=0.010,
    )
    tip_hole = _cylinder_y(
        x_center=0.069,
        y_center=0.0,
        z_center=0.0,
        radius=0.0025,
        y_size=CENTER_KNUCKLE_WIDTH + 0.002,
    )

    return root_tongue.union(neck).union(body).union(end_tab).cut(tip_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_joint_low_profile_chain")

    model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("anodized_mid", rgba=(0.53, 0.56, 0.60, 1.0))
    model.material("anodized_light", rgba=(0.68, 0.71, 0.75, 1.0))

    mounting_plate = model.part("mounting_plate")
    mounting_plate.visual(
        mesh_from_cadquery(_root_plate_shape(), "mounting_plate"),
        material="graphite",
        name="plate_shell",
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(_proximal_link_shape(), "proximal_link"),
        material="anodized_mid",
        name="proximal_shell",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        material="anodized_light",
        name="distal_shell",
    )

    model.articulation(
        "plate_to_proximal",
        ArticulationType.REVOLUTE,
        parent=mounting_plate,
        child=proximal_link,
        origin=Origin(xyz=(ROOT_HINGE_X, 0.0, ROOT_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(PROXIMAL_LENGTH + PROXIMAL_ROOT_OFFSET, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mounting_plate = object_model.get_part("mounting_plate")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    shoulder = object_model.get_articulation("plate_to_proximal")
    elbow = object_model.get_articulation("proximal_to_distal")

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

    ctx.expect_contact(
        mounting_plate,
        proximal_link,
        name="root knuckle seats against mounting clevis",
    )
    ctx.expect_contact(
        proximal_link,
        distal_link,
        name="distal knuckle seats against proximal clevis",
    )

    ctx.check(
        "joint axes are coplanar pitch axes",
        shoulder.axis == (0.0, -1.0, 0.0) and elbow.axis == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    rest_prox_center = _aabb_center(ctx.part_element_world_aabb(proximal_link, elem="proximal_shell"))
    rest_dist_center = _aabb_center(ctx.part_element_world_aabb(distal_link, elem="distal_shell"))

    with ctx.pose({shoulder: 0.9, elbow: 0.0}):
        raised_prox_center = _aabb_center(
            ctx.part_element_world_aabb(proximal_link, elem="proximal_shell")
        )
        raised_dist_center = _aabb_center(
            ctx.part_element_world_aabb(distal_link, elem="distal_shell")
        )

    ctx.check(
        "shoulder lifts both links without lateral drift",
        rest_prox_center is not None
        and rest_dist_center is not None
        and raised_prox_center is not None
        and raised_dist_center is not None
        and raised_prox_center[2] > rest_prox_center[2] + 0.02
        and raised_dist_center[2] > rest_dist_center[2] + 0.05
        and abs(raised_prox_center[1] - rest_prox_center[1]) < 1e-6
        and abs(raised_dist_center[1] - rest_dist_center[1]) < 1e-6,
        details=(
            f"rest_prox={rest_prox_center}, raised_prox={raised_prox_center}, "
            f"rest_dist={rest_dist_center}, raised_dist={raised_dist_center}"
        ),
    )

    with ctx.pose({shoulder: 0.55, elbow: 0.0}):
        elbow_rest_dist_center = _aabb_center(
            ctx.part_element_world_aabb(distal_link, elem="distal_shell")
        )
    with ctx.pose({shoulder: 0.55, elbow: 1.0}):
        folded_dist_center = _aabb_center(ctx.part_element_world_aabb(distal_link, elem="distal_shell"))

    ctx.check(
        "elbow folds distal tab further upward in the same plane",
        elbow_rest_dist_center is not None
        and folded_dist_center is not None
        and folded_dist_center[2] > elbow_rest_dist_center[2]
        and abs(folded_dist_center[1] - elbow_rest_dist_center[1]) < 1e-6,
        details=f"elbow_rest={elbow_rest_dist_center}, folded_dist={folded_dist_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
