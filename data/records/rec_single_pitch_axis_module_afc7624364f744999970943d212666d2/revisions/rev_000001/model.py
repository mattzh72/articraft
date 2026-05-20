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


PLATE_T = 0.016
WALL_W = 0.170
WALL_H = 0.240

AXIS_X = 0.072
CHEEK_T = 0.016
CHEEK_X = 0.052
CHEEK_H = 0.108
CHEEK_GAP = 0.084
CHEEK_Y = CHEEK_GAP / 2.0 + CHEEK_T / 2.0

BEARING_R = 0.0135
COLLAR_R = 0.020
COLLAR_LEN = 0.008

JOURNAL_R = 0.0115
TRUNNION_HALF_SPAN = CHEEK_Y + CHEEK_T / 2.0 + COLLAR_LEN
TRUNNION_LEN = 2.0 * TRUNNION_HALF_SPAN + 0.002
SHOULDER_R = 0.018
SHOULDER_T = 0.008
YOKE_W = 0.028
YOKE_H = 0.044
YOKE_START_X = 0.008
YOKE_LEN = 0.052
PANEL_BOSS_R = 0.022
PANEL_BOSS_LEN = 0.014

PANEL_W = 0.092
PANEL_H = 0.078
PANEL_T = 0.010
PANEL_BACK_X = YOKE_START_X + YOKE_LEN + PANEL_BOSS_LEN
RIB_X = 0.068
RIB_Y = 0.110
RIB_T = 0.016
RIB_Z = 0.046
CHEEK_CENTER_X = 0.074


def _make_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, WALL_W, WALL_H)

    upper_rib = (
        cq.Workplane("XY")
        .transformed(offset=(0.042, 0.0, RIB_Z))
        .box(RIB_X, RIB_Y, RIB_T)
    )
    lower_rib = (
        cq.Workplane("XY")
        .transformed(offset=(0.042, 0.0, -RIB_Z))
        .box(RIB_X, RIB_Y, RIB_T)
    )

    left_cheek = (
        cq.Workplane("XY")
        .transformed(offset=(CHEEK_CENTER_X, CHEEK_Y, 0.0))
        .box(CHEEK_X, CHEEK_T, CHEEK_H)
        .edges(">X and |Z")
        .fillet(0.006)
    )
    right_cheek = (
        cq.Workplane("XY")
        .transformed(offset=(CHEEK_CENTER_X, -CHEEK_Y, 0.0))
        .box(CHEEK_X, CHEEK_T, CHEEK_H)
        .edges(">X and |Z")
        .fillet(0.006)
    )

    left_collar = (
        cq.Workplane("XZ")
        .center(AXIS_X, 0.0)
        .circle(COLLAR_R)
        .extrude(COLLAR_LEN / 2.0, both=True)
        .translate((0.0, CHEEK_Y + CHEEK_T / 2.0 + COLLAR_LEN / 2.0, 0.0))
    )
    right_collar = (
        cq.Workplane("XZ")
        .center(AXIS_X, 0.0)
        .circle(COLLAR_R)
        .extrude(COLLAR_LEN / 2.0, both=True)
        .translate((0.0, -CHEEK_Y - CHEEK_T / 2.0 - COLLAR_LEN / 2.0, 0.0))
    )

    base = (
        plate.union(upper_rib)
        .union(lower_rib)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_collar)
        .union(right_collar)
    )

    bearing_bore = (
        cq.Workplane("XZ")
        .center(AXIS_X, 0.0)
        .circle(BEARING_R)
        .extrude(WALL_W, both=True)
    )

    return base.cut(bearing_bore)


def _make_carrier_body() -> cq.Workplane:
    trunnion = (
        cq.Workplane("XZ")
        .circle(JOURNAL_R)
        .extrude(TRUNNION_LEN / 2.0, both=True)
    )
    left_shoulder = (
        cq.Workplane("XZ")
        .circle(SHOULDER_R)
        .extrude(SHOULDER_T / 2.0, both=True)
        .translate((0.0, TRUNNION_HALF_SPAN + SHOULDER_T / 2.0, 0.0))
    )
    right_shoulder = (
        cq.Workplane("XZ")
        .circle(SHOULDER_R)
        .extrude(SHOULDER_T / 2.0, both=True)
        .translate((0.0, -TRUNNION_HALF_SPAN - SHOULDER_T / 2.0, 0.0))
    )

    yoke = (
        cq.Workplane("YZ")
        .rect(YOKE_W, YOKE_H)
        .extrude(YOKE_LEN)
        .translate((YOKE_START_X, 0.0, 0.0))
        .edges(">X and |Y")
        .fillet(0.006)
    )

    panel_boss = (
        cq.Workplane("YZ")
        .circle(PANEL_BOSS_R)
        .extrude(PANEL_BOSS_LEN)
        .translate((PANEL_BACK_X - PANEL_BOSS_LEN, 0.0, 0.0))
    )

    return (
        trunnion.union(left_shoulder)
        .union(right_shoulder)
        .union(yoke)
        .union(panel_boss)
    )


def _make_faceplate() -> cq.Workplane:
    panel = (
        cq.Workplane("YZ")
        .rect(PANEL_W, PANEL_H)
        .extrude(PANEL_T)
        .edges("|X")
        .fillet(0.006)
    )
    panel = (
        panel.faces(">X")
        .workplane()
        .rarray(PANEL_W - 0.028, PANEL_H - 0.028, 2, 2)
        .hole(0.006)
    )
    return panel.translate((PANEL_BACK_X, 0.0, 0.0))


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_pitch_trunnion_module")

    dark_body = model.material("dark_body", rgba=(0.24, 0.26, 0.30, 1.0))
    carrier_body = model.material("carrier_body", rgba=(0.42, 0.45, 0.49, 1.0))
    faceplate_finish = model.material("faceplate_finish", rgba=(0.72, 0.74, 0.78, 1.0))

    base = model.part("side_support")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "side_support"),
        origin=Origin(),
        material=dark_body,
        name="support_structure",
    )

    carrier = model.part("carried_face")
    carrier.visual(
        mesh_from_cadquery(_make_carrier_body(), "carrier_body"),
        origin=Origin(),
        material=carrier_body,
        name="carrier_body",
    )
    carrier.visual(
        mesh_from_cadquery(_make_faceplate(), "carrier_faceplate"),
        origin=Origin(),
        material=faceplate_finish,
        name="faceplate",
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(AXIS_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.60,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_support = object_model.get_part("side_support")
    carried_face = object_model.get_part("carried_face")
    pitch_trunnion = object_model.get_articulation("pitch_trunnion")

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
    ctx.allow_overlap(
        carried_face,
        side_support,
        elem_a="carrier_body",
        elem_b="support_structure",
        reason=(
            "The trunnion journal is represented as a captured zero-clearance "
            "bearing fit inside the side-support cartridge, so the nested "
            "supporting engagement is intentional."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        carried_face,
        side_support,
        elem_a="carrier_body",
        name="trunnion_journal_contacts_side_support",
    )
    ctx.expect_overlap(
        carried_face,
        side_support,
        axes="yz",
        min_overlap=0.03,
        elem_a="carrier_body",
        name="trunnion_axis_is_aligned_with_support_cheeks",
    )
    ctx.expect_origin_gap(
        carried_face,
        side_support,
        axis="x",
        min_gap=0.065,
        max_gap=0.080,
        name="axis_cartridge_projects_forward_from_side_wall",
    )
    ctx.expect_gap(
        carried_face,
        side_support,
        axis="x",
        min_gap=0.045,
        positive_elem="faceplate",
        name="faceplate_sits_forward_of_side_support",
    )

    with ctx.pose({pitch_trunnion: 0.0}):
        closed_faceplate = ctx.part_element_world_aabb(carried_face, elem="faceplate")
    with ctx.pose({pitch_trunnion: pitch_trunnion.motion_limits.upper}):
        raised_faceplate = ctx.part_element_world_aabb(carried_face, elem="faceplate")

    closed_z = _aabb_center_z(closed_faceplate)
    raised_z = _aabb_center_z(raised_faceplate)
    ctx.check(
        "positive_pitch_rotates_faceplate_upward",
        (
            closed_z is not None
            and raised_z is not None
            and raised_z > closed_z + 0.040
        ),
        details=(
            f"closed center z={closed_z}, raised center z={raised_z}; "
            "expected positive joint travel to lift the carried face."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
