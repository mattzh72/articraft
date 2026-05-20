from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

GAP_WIDTH = 0.036
LUG_THICKNESS = 0.012
TOTAL_YOKE_WIDTH = GAP_WIDTH + 2.0 * LUG_THICKNESS
LUG_LENGTH = 0.032

BASE_LENGTH = 0.220
BASE_WIDTH = 0.140
BASE_PLATE_HEIGHT = 0.026
BASE_AXIS_Z = 0.182

LINK_1_JOINT_X = 0.215
LINK_2_JOINT_X = 0.182
LINK_3_JOINT_X = 0.146


def _y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _box(length: float, width: float, height: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _forward_boss(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    boss = _y_cylinder(radius, length, x=x, y=y, z=z)
    keep = _box(
        radius * 1.6,
        length * 1.2,
        radius * 2.3,
        center=(x + radius * 0.8, y, z),
    )
    return boss.intersect(keep)


def _rear_boss(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    boss = _y_cylinder(radius, length, x=x, y=y, z=z)
    keep = _box(
        radius * 1.6,
        length * 1.2,
        radius * 2.3,
        center=(x - radius * 0.8, y, z),
    )
    return boss.intersect(keep)


def _link_profile(beam_end_x: float, depth: float) -> cq.Workplane:
    half = depth / 2.0
    shoulder_x = max(0.060, beam_end_x - 0.040)
    points = [
        (0.020, -0.34 * half),
        (0.042, -0.82 * half),
        (shoulder_x, -half),
        (beam_end_x, -0.78 * half),
        (beam_end_x, 0.78 * half),
        (shoulder_x, half),
        (0.042, 0.82 * half),
        (0.020, 0.34 * half),
    ]
    return cq.Workplane("XZ").polyline(points).close()


def _make_box_link(
    *,
    joint_x: float,
    depth: float,
    proximal_radius: float,
    distal_radius: float,
) -> cq.Workplane:
    beam_end_x = joint_x - 0.050
    beam = _link_profile(beam_end_x, depth).extrude(GAP_WIDTH / 2.0, both=True)
    proximal_boss = _forward_boss(proximal_radius, GAP_WIDTH, x=0.0, z=0.0)
    root_collar = _box(0.030, GAP_WIDTH, depth * 0.72, center=(0.018, 0.0, 0.0))
    spine = _box(
        max(0.050, beam_end_x - 0.032),
        GAP_WIDTH,
        depth * 0.72,
        center=((beam_end_x + 0.032) / 2.0, 0.0, 0.0),
    )
    yoke_body = _box(
        0.050,
        TOTAL_YOKE_WIDTH,
        depth * 0.68,
        center=(joint_x - 0.025, 0.0, 0.0),
    )
    yoke_slot = _box(
        0.024,
        GAP_WIDTH,
        depth * 0.52,
        center=(joint_x - 0.012, 0.0, 0.0),
    )

    shape = beam.union(proximal_boss).union(root_collar).union(spine).union(yoke_body).cut(yoke_slot)

    for sign in (-1.0, 1.0):
        boss_y = sign * (GAP_WIDTH / 2.0 + LUG_THICKNESS / 2.0)
        outer_boss = _rear_boss(
            distal_radius,
            LUG_THICKNESS,
            x=joint_x,
            y=boss_y,
            z=0.0,
        )
        cheek_pad = _box(
            0.016,
            LUG_THICKNESS,
            depth * 0.76,
            center=(joint_x - 0.008, boss_y, 0.0),
        )
        shape = shape.union(outer_boss).union(cheek_pad)

    return shape


def _make_end_tab() -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.000, -0.015),
                (0.020, -0.015),
                (0.050, -0.012),
                (0.067, -0.006),
                (0.073, 0.000),
                (0.067, 0.006),
                (0.050, 0.012),
                (0.020, 0.015),
                (0.000, 0.015),
            ]
        )
        .close()
    )
    tongue = profile.extrude(GAP_WIDTH / 2.0, both=True)
    proximal_boss = _forward_boss(0.017, GAP_WIDTH, x=0.0, z=0.0)
    collar = _box(0.022, GAP_WIDTH, 0.024, center=(0.014, 0.0, 0.0))
    inspection_hole = _y_cylinder(0.0055, GAP_WIDTH * 1.2, x=0.056, z=0.0)
    return tongue.union(proximal_boss).union(collar).cut(inspection_hole)


def _make_base() -> cq.Workplane:
    plate = _box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_PLATE_HEIGHT,
        center=(0.0, 0.0, BASE_PLATE_HEIGHT / 2.0),
    )
    pedestal = _box(
        0.118,
        BASE_WIDTH * 0.82,
        0.052,
        center=(-0.050, 0.0, BASE_PLATE_HEIGHT + 0.026),
    )

    cheek_height = BASE_AXIS_Z - BASE_PLATE_HEIGHT
    cheek_center_z = BASE_PLATE_HEIGHT + cheek_height / 2.0
    cheek_center_x = -0.030
    cheek_length = 0.060

    shape = plate.union(pedestal)
    yoke_body = _box(
        0.060,
        TOTAL_YOKE_WIDTH,
        cheek_height * 0.58,
        center=(-0.030, 0.0, BASE_PLATE_HEIGHT + cheek_height * 0.29),
    )
    yoke_slot = _box(
        0.022,
        GAP_WIDTH,
        cheek_height * 0.50,
        center=(-0.011, 0.0, BASE_PLATE_HEIGHT + cheek_height * 0.34),
    )
    shape = shape.union(yoke_body).cut(yoke_slot)

    for sign in (-1.0, 1.0):
        cheek_center_y = sign * (GAP_WIDTH / 2.0 + LUG_THICKNESS / 2.0)
        cheek = _box(
            cheek_length,
            LUG_THICKNESS,
            cheek_height,
            center=(cheek_center_x, cheek_center_y, cheek_center_z),
        )
        side_brace = _box(
            0.042,
            0.018,
            cheek_height * 0.72,
            center=(-0.048, sign * 0.021, BASE_PLATE_HEIGHT + cheek_height * 0.44),
        )
        boss = _rear_boss(
            0.030,
            LUG_THICKNESS,
            x=0.0,
            y=cheek_center_y,
            z=BASE_AXIS_Z,
        )
        shape = shape.union(cheek).union(side_brace).union(boss)

    rear_web = _box(0.030, GAP_WIDTH, 0.090, center=(-0.058, 0.0, BASE_AXIS_Z - 0.045))
    front_skid = _box(0.040, BASE_WIDTH * 0.72, 0.012, center=(0.058, 0.0, BASE_PLATE_HEIGHT + 0.006))

    return shape.union(rear_web).union(front_skid)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_four_joint_chain")

    model.material("painted_base", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machine_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("link_dark", rgba=(0.38, 0.42, 0.47, 1.0))
    model.material("end_tab_finish", rgba=(0.64, 0.67, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base(), "industrial_chain_base"),
        material="painted_base",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_AXIS_Z + 0.020)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_AXIS_Z + 0.020) / 2.0)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(
            _make_box_link(
                joint_x=LINK_1_JOINT_X,
                depth=0.064,
                proximal_radius=0.028,
                distal_radius=0.026,
            ),
            "industrial_chain_link_1",
        ),
        material="machine_gray",
        name="link_1_shell",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_1_JOINT_X + 0.018, TOTAL_YOKE_WIDTH, 0.064)),
        mass=2.4,
        origin=Origin(xyz=((LINK_1_JOINT_X + 0.018) / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(
            _make_box_link(
                joint_x=LINK_2_JOINT_X,
                depth=0.056,
                proximal_radius=0.024,
                distal_radius=0.023,
            ),
            "industrial_chain_link_2",
        ),
        material="link_dark",
        name="link_2_shell",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_2_JOINT_X + 0.018, TOTAL_YOKE_WIDTH, 0.056)),
        mass=1.8,
        origin=Origin(xyz=((LINK_2_JOINT_X + 0.018) / 2.0, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(
            _make_box_link(
                joint_x=LINK_3_JOINT_X,
                depth=0.048,
                proximal_radius=0.021,
                distal_radius=0.019,
            ),
            "industrial_chain_link_3",
        ),
        material="machine_gray",
        name="link_3_shell",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_3_JOINT_X + 0.018, TOTAL_YOKE_WIDTH, 0.048)),
        mass=1.2,
        origin=Origin(xyz=((LINK_3_JOINT_X + 0.018) / 2.0, 0.0, 0.0)),
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab(), "industrial_chain_end_tab"),
        material="end_tab_finish",
        name="end_tab_shell",
    )
    end_tab.inertial = Inertial.from_geometry(
        Box((0.076, GAP_WIDTH, 0.032)),
        mass=0.45,
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, BASE_AXIS_Z), rpy=(0.0, -0.62, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.20, effort=90.0, velocity=1.0),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_JOINT_X, 0.0, 0.0), rpy=(0.0, 0.18, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.25, effort=70.0, velocity=1.2),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_JOINT_X, 0.0, 0.0), rpy=(0.0, -0.12, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.30, upper=1.20, effort=50.0, velocity=1.4),
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK_3_JOINT_X, 0.0, 0.0), rpy=(0.0, 0.08, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.50, upper=1.50, effort=30.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")

    base_to_link_1 = object_model.get_articulation("base_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_end_tab = object_model.get_articulation("link_3_to_end_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        link_1,
        reason="The shoulder joint is modeled as an interleaved cheek-and-boss knuckle without separate pin bores, so the nominal joint envelope is intentionally shared.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Adjacent boxed links use a compact interleaved revolute knuckle representation with omitted bore voids and pin solids.",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        reason="Adjacent boxed links use a compact interleaved revolute knuckle representation with omitted bore voids and pin solids.",
    )
    ctx.allow_overlap(
        link_3,
        end_tab,
        reason="The compact end tab is captured as a clevis-mounted tab without explicitly subtracting the pin bore volume.",
    )

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
        "all_hinge_axes_parallel",
        all(
            tuple(round(v, 6) for v in articulation.axis) == (0.0, -1.0, 0.0)
            for articulation in (
                base_to_link_1,
                link_1_to_link_2,
                link_2_to_link_3,
                link_3_to_end_tab,
            )
        ),
        "Every revolute axis should remain parallel and aligned to world -Y.",
    )

    ctx.expect_contact(base, link_1, contact_tol=2e-4, name="base_supports_link_1")
    ctx.expect_contact(link_1, link_2, contact_tol=2e-4, name="link_1_supports_link_2")
    ctx.expect_contact(link_2, link_3, contact_tol=2e-4, name="link_2_supports_link_3")
    ctx.expect_contact(link_3, end_tab, contact_tol=2e-4, name="link_3_supports_end_tab")

    with ctx.pose(
        {
            base_to_link_1: 0.22,
            link_1_to_link_2: 0.30,
            link_2_to_link_3: -0.16,
            link_3_to_end_tab: 0.10,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="representative_pose_clearance")
        ctx.expect_origin_gap(
            end_tab,
            base,
            axis="z",
            min_gap=0.10,
            name="representative_pose_lifts_end_tab",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
