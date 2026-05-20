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


BASE_LEN = 0.82
BASE_W = 0.22
BASE_T = 0.02
RAIL_LEN = 0.60
RAIL_W = 0.032
RAIL_H = 0.045
RAIL_Y = 0.065

SLIDE_HOME_X = -0.19
SLIDE_TRAVEL = 0.38
SHOULDER_Z = BASE_T + RAIL_H + 0.06

UPPER_LEN = 0.28
def _box(sx: float, sy: float, sz: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _guide_body_shape() -> cq.Workplane:
    base = _box(BASE_LEN, BASE_W, BASE_T, (0.0, 0.0, BASE_T / 2.0))
    center_relief = _box(0.52, 0.084, 0.008, (0.0, 0.0, BASE_T - 0.004))
    base = base.cut(center_relief)

    rail_left = _box(RAIL_LEN, RAIL_W, RAIL_H, (0.0, -RAIL_Y, BASE_T + RAIL_H / 2.0))
    rail_right = _box(RAIL_LEN, RAIL_W, RAIL_H, (0.0, RAIL_Y, BASE_T + RAIL_H / 2.0))
    end_cap_left = _box(0.05, 0.17, 0.032, (-0.275, 0.0, BASE_T + 0.016))
    end_cap_right = _box(0.05, 0.17, 0.032, (0.275, 0.0, BASE_T + 0.016))

    return base.union(rail_left).union(rail_right).union(end_cap_left).union(end_cap_right)


def _carriage_shape() -> cq.Workplane:
    deck = _box(0.18, 0.148, 0.026, (-0.11, 0.0, -0.041))
    shoe_left = _box(0.11, 0.028, 0.014, (-0.11, -RAIL_Y, -0.055))
    shoe_right = _box(0.11, 0.028, 0.014, (-0.11, RAIL_Y, -0.055))
    lower_bridge = _box(0.11, 0.064, 0.03, (-0.075, 0.0, -0.022))
    shoulder_block = _box(0.038, 0.052, 0.072, (-0.019, 0.0, 0.0))
    top_cap = _box(0.05, 0.072, 0.014, (-0.048, 0.0, 0.029))
    rear_web = _box(0.042, 0.06, 0.026, (-0.06, 0.0, 0.002))

    carriage = deck.union(shoe_left).union(shoe_right).union(lower_bridge).union(shoulder_block).union(top_cap).union(rear_web)
    underside_relief = _box(0.106, 0.082, 0.016, (-0.05, 0.0, -0.049))
    return carriage.cut(underside_relief)


def _upper_link_shape() -> cq.Workplane:
    root_plate = _box(0.014, 0.044, 0.058, (0.007, 0.0, 0.0))
    beam = _box(0.214, 0.014, 0.03, (0.121, 0.0, 0.0))
    top_rib = _box(0.105, 0.01, 0.012, (0.15, 0.0, 0.013))
    lower_rib = _box(0.085, 0.01, 0.01, (0.095, 0.0, -0.013))
    elbow_block = _box(0.052, 0.05, 0.048, (0.254, 0.0, 0.0))
    elbow_fairing = _box(0.04, 0.022, 0.028, (0.23, 0.0, 0.0))

    return root_plate.union(beam).union(top_rib).union(lower_rib).union(elbow_fairing).union(elbow_block)


def _distal_link_shape() -> cq.Workplane:
    root_plate = _box(0.012, 0.044, 0.046, (0.006, 0.0, 0.0))
    blade = _box(0.108, 0.012, 0.024, (0.066, 0.0, 0.0))
    pad_stem = _box(0.034, 0.03, 0.034, (0.126, 0.0, 0.0))
    square_pad = _box(0.018, 0.082, 0.082, (0.145, 0.0, 0.0))

    return root_plate.union(blade).union(pad_stem).union(square_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_slide_two_link_arm")

    guide_mat = model.material("guide_mat", rgba=(0.26, 0.28, 0.31, 1.0))
    carriage_mat = model.material("carriage_mat", rgba=(0.72, 0.74, 0.76, 1.0))
    arm_mat = model.material("arm_mat", rgba=(0.33, 0.43, 0.60, 1.0))
    distal_mat = model.material("distal_mat", rgba=(0.22, 0.29, 0.42, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        mesh_from_cadquery(_guide_body_shape(), "guide_body"),
        name="guide_body_shell",
        material=guide_mat,
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        name="carriage_body",
        material=carriage_mat,
    )

    upper = model.part("upper_link")
    upper.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link"),
        name="upper_link_body",
        material=arm_mat,
    )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        name="distal_link_body",
        material=distal_mat,
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, SHOULDER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_upper",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.35,
        ),
    )

    model.articulation(
        "upper_to_distal",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=distal,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=-1.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_body")
    carriage = object_model.get_part("carriage")
    upper = object_model.get_part("upper_link")
    distal = object_model.get_part("distal_link")
    slide = object_model.get_articulation("guide_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper")
    elbow = object_model.get_articulation("upper_to_distal")

    def center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

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
        "slide_axis_is_positive_x",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected slide axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "shoulder_axis_is_negative_y",
        tuple(shoulder.axis) == (0.0, -1.0, 0.0),
        f"expected shoulder axis (0, -1, 0), got {shoulder.axis}",
    )
    ctx.check(
        "elbow_axis_is_negative_y",
        tuple(elbow.axis) == (0.0, -1.0, 0.0),
        f"expected elbow axis (0, -1, 0), got {elbow.axis}",
    )

    ctx.expect_contact(carriage, guide, name="carriage_supported_by_guide")
    ctx.expect_contact(upper, carriage, name="upper_link_seated_in_shoulder_block")
    ctx.expect_contact(distal, upper, name="distal_link_seated_in_elbow_fork")
    ctx.expect_gap(upper, guide, axis="z", min_gap=0.03, name="arm_clears_guide_at_rest")

    home_carriage_pos = ctx.part_world_position(carriage)
    home_upper_aabb = ctx.part_element_world_aabb(upper, elem="upper_link_body")
    home_upper_z = center_z(home_upper_aabb)

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(carriage, guide, name="carriage_supported_at_full_extension")
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "slide_moves_carriage_forward",
        home_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > home_carriage_pos[0] + 0.30,
        f"expected carriage x to increase by more than 0.30 m, got home={home_carriage_pos} extended={extended_carriage_pos}",
    )

    with ctx.pose({shoulder: 1.0}):
        raised_upper_aabb = ctx.part_element_world_aabb(upper, elem="upper_link_body")
        raised_upper_z = center_z(raised_upper_aabb)
        ctx.expect_contact(upper, carriage, name="upper_link_stays_supported_when_raised")

    ctx.check(
        "shoulder_positive_rotation_lifts_upper_link",
        home_upper_z is not None and raised_upper_z is not None and raised_upper_z > home_upper_z + 0.10,
        f"expected upper link center z to rise by more than 0.10 m, got home={home_upper_z} raised={raised_upper_z}",
    )

    with ctx.pose({shoulder: 0.55, elbow: 0.0}):
        straight_distal_aabb = ctx.part_element_world_aabb(distal, elem="distal_link_body")
        straight_distal_z = center_z(straight_distal_aabb)

    with ctx.pose({shoulder: 0.55, elbow: 0.95}):
        bent_distal_aabb = ctx.part_element_world_aabb(distal, elem="distal_link_body")
        bent_distal_z = center_z(bent_distal_aabb)
        ctx.expect_contact(distal, upper, name="distal_link_stays_supported_when_bent")

    ctx.check(
        "elbow_positive_rotation_repositions_distal_link",
        straight_distal_z is not None and bent_distal_z is not None and bent_distal_z > straight_distal_z + 0.025,
        f"expected distal link center z to rise by more than 0.025 m, got straight={straight_distal_z} bent={bent_distal_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
