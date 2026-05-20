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


BEAM_LENGTH = 1.30
BEAM_WIDTH = 0.18
BEAM_HEIGHT = 0.12
BEAM_WALL = 0.014
BEAM_SLOT_WIDTH = 0.070
SHUTTLE_DROP = 0.125
PIN_RADIUS = 0.007


def _beam_track_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT)
    inner = (
        cq.Workplane("XY")
        .box(
            BEAM_LENGTH - 2.0 * BEAM_WALL,
            BEAM_WIDTH - 2.0 * BEAM_WALL,
            BEAM_HEIGHT - 0.036,
        )
        .translate((0.0, 0.0, 0.004))
    )
    slot = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH - 0.10, BEAM_SLOT_WIDTH, 0.045)
        .translate((0.0, 0.0, -0.040))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH - 0.12, 0.018, 0.010)
        .translate((0.0, 0.044, -0.048))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH - 0.12, 0.018, 0.010)
        .translate((0.0, -0.044, -0.048))
    )
    stop_left = (
        cq.Workplane("XY")
        .box(0.018, 0.056, 0.024)
        .translate((-(BEAM_LENGTH * 0.5 - 0.045), 0.0, -0.022))
    )
    stop_right = (
        cq.Workplane("XY")
        .box(0.018, 0.056, 0.024)
        .translate(((BEAM_LENGTH * 0.5 - 0.045), 0.0, -0.022))
    )

    return (
        outer.cut(inner)
        .cut(slot)
        .union(rail_left)
        .union(rail_right)
        .union(stop_left)
        .union(stop_right)
    )


def _beam_mounts_shape() -> cq.Workplane:
    mounts = cq.Workplane("XY")
    for x_pos in (-0.42, 0.42):
        post = cq.Workplane("XY").box(0.050, 0.088, 0.090).translate((x_pos, 0.0, 0.105))
        cap = cq.Workplane("XY").box(0.120, 0.100, 0.012).translate((x_pos, 0.0, 0.156))
        saddle = cq.Workplane("XY").box(0.080, 0.088, 0.012).translate((x_pos, 0.0, 0.066))
        mounts = mounts.union(post).union(cap).union(saddle)
    return mounts


def _internal_carriage_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.140, 0.032, 0.012).translate((0.0, 0.0, 0.039))
    for x_pos in (-0.042, 0.042):
        left_shoe = cq.Workplane("XY").box(0.050, 0.014, 0.020).translate((x_pos, 0.044, 0.055))
        right_shoe = cq.Workplane("XY").box(0.050, 0.014, 0.020).translate((x_pos, -0.044, 0.055))
        left_web = cq.Workplane("XY").box(0.016, 0.020, 0.026).translate((x_pos, 0.024, 0.043))
        right_web = cq.Workplane("XY").box(0.016, 0.020, 0.026).translate((x_pos, -0.024, 0.043))
        frame = frame.union(left_shoe).union(right_shoe).union(left_web).union(right_web)
    return frame


def _shuttle_stem_shape() -> cq.Workplane:
    neck = cq.Workplane("XY").box(0.018, 0.016, 0.052).translate((0.0, 0.0, 0.000))
    bridge = cq.Workplane("XY").box(0.024, 0.020, 0.010).translate((0.0, 0.0, -0.022))
    yoke_left = cq.Workplane("XY").box(0.016, 0.004, 0.020).translate((0.0, 0.010, -0.036))
    yoke_right = cq.Workplane("XY").box(0.016, 0.004, 0.020).translate((0.0, -0.010, -0.036))
    return neck.union(bridge).union(yoke_left).union(yoke_right)


def _carriage_cover_shape() -> cq.Workplane:
    cover = cq.Workplane("XY").box(0.198, 0.110, 0.024).translate((0.0, 0.0, 0.045))
    crown = cq.Workplane("XY").box(0.080, 0.048, 0.014).translate((0.0, 0.0, 0.032))
    return cover.union(crown)


def _first_link_shape() -> cq.Workplane:
    top_lug = cq.Workplane("XY").box(0.014, 0.020, 0.026).translate((0.0, 0.0, -0.013))
    shoulder_block = cq.Workplane("XY").box(0.018, 0.012, 0.030).translate((0.012, 0.0, -0.040))
    body = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.118)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
        .translate((0.026, 0.0, -0.102))
    )
    elbow_block = cq.Workplane("XY").box(0.018, 0.012, 0.022).translate((0.045, 0.0, -0.178))
    ear_left = cq.Workplane("XY").box(0.014, 0.004, 0.028).translate((0.045, 0.008, -0.178))
    ear_right = cq.Workplane("XY").box(0.014, 0.004, 0.028).translate((0.045, -0.008, -0.178))

    return top_lug.union(shoulder_block).union(body).union(elbow_block).union(ear_left).union(ear_right)


def _second_link_shape() -> cq.Workplane:
    top_lug = cq.Workplane("XY").box(0.012, 0.014, 0.026).translate((0.0, 0.0, -0.013))
    neck = cq.Workplane("XY").box(0.016, 0.010, 0.028).translate((0.012, 0.0, -0.032))
    body = (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.215)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)
        .translate((0.046, 0.0, -0.145))
    )
    lower_neck = cq.Workplane("XY").box(0.020, 0.010, 0.050).translate((0.066, 0.0, -0.255))
    plate = (
        cq.Workplane("XZ")
        .rect(0.140, 0.180)
        .extrude(0.006, both=True)
        .faces(">Y")
        .workplane()
        .pushPoints([(-0.040, -0.060), (0.040, -0.060), (-0.040, 0.060), (0.040, 0.060)])
        .hole(0.014)
        .translate((0.074, 0.0, -0.340))
    )

    return top_lug.union(neck).union(body).union(lower_neck).union(plate)


def _box_inertial(size: tuple[float, float, float], mass: float, xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Inertial:
    return Inertial.from_geometry(Box(size), mass=mass, origin=Origin(xyz=xyz))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhead_inspection_shuttle")

    beam_gray = model.material("beam_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    arm_blue = model.material("arm_blue", rgba=(0.25, 0.35, 0.48, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.78, 0.80, 0.82, 1.0))

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_beam_track_shape(), "beam_track"),
        origin=Origin(),
        material=beam_gray,
        name="track",
    )
    beam.visual(
        mesh_from_cadquery(_beam_mounts_shape(), "beam_mounts"),
        origin=Origin(),
        material=beam_gray,
        name="mounts",
    )
    beam.inertial = _box_inertial((BEAM_LENGTH, BEAM_WIDTH, 0.20), 16.0, xyz=(0.0, 0.0, 0.04))

    shuttle = model.part("shuttle")
    shuttle.visual(
        mesh_from_cadquery(_internal_carriage_shape(), "shuttle_internal_carriage"),
        origin=Origin(),
        material=carriage_gray,
        name="internal_carriage",
    )
    shuttle.visual(
        mesh_from_cadquery(_shuttle_stem_shape(), "shuttle_stem"),
        origin=Origin(),
        material=carriage_gray,
        name="hanger_stem",
    )
    shuttle.visual(
        mesh_from_cadquery(_carriage_cover_shape(), "shuttle_cover"),
        origin=Origin(),
        material=carriage_gray,
        name="carriage_cover",
    )
    shuttle.inertial = _box_inertial((0.22, 0.14, 0.18), 4.8, xyz=(0.0, 0.0, 0.08))

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_first_link_shape(), "inspection_first_link"),
        origin=Origin(),
        material=arm_blue,
        name="link_body",
    )
    first_link.inertial = _box_inertial((0.12, 0.06, 0.24), 2.2, xyz=(0.04, 0.0, -0.12))

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(_second_link_shape(), "inspection_second_link"),
        origin=Origin(),
        material=plate_gray,
        name="arm_and_plate",
    )
    second_link.inertial = _box_inertial((0.18, 0.03, 0.48), 2.8, xyz=(0.07, 0.0, -0.26))

    model.articulation(
        "beam_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, -SHUTTLE_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.8, lower=-0.43, upper=0.43),
    )
    model.articulation(
        "shuttle_to_first",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-0.60, upper=0.95),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.045, 0.0, -0.178)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-1.55, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    shuttle = object_model.get_part("shuttle")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    slide = object_model.get_articulation("beam_to_shuttle")
    shoulder = object_model.get_articulation("shuttle_to_first")
    elbow = object_model.get_articulation("first_to_second")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        shuttle,
        first_link,
        elem_a="hanger_stem",
        elem_b="link_body",
        reason="The shoulder hinge is modeled with a compact carried yoke and enclosed arm root that intentionally share hinge-barrel volume.",
    )
    ctx.allow_overlap(
        first_link,
        second_link,
        elem_a="link_body",
        elem_b="arm_and_plate",
        reason="The elbow clevis and second-link root intentionally share the hinge barrel around the elbow pin.",
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

    ctx.expect_contact(beam, shuttle, name="shuttle_runs_on_track")
    ctx.expect_contact(shuttle, first_link, name="shoulder_hinge_is_carried_by_shuttle")
    ctx.expect_contact(first_link, second_link, name="elbow_hinge_keeps_arm_supported")

    ctx.expect_gap(
        beam,
        shuttle,
        axis="z",
        positive_elem="track",
        negative_elem="carriage_cover",
        min_gap=0.004,
        max_gap=0.014,
        name="carriage_cover_stays_below_track",
    )
    ctx.expect_gap(
        beam,
        first_link,
        axis="z",
        min_gap=0.050,
        name="first_link_hangs_below_beam",
    )

    with ctx.pose({slide: slide.motion_limits.lower}):
        ctx.expect_within(
            shuttle,
            beam,
            axes="x",
            inner_elem="internal_carriage",
            outer_elem="track",
            margin=0.01,
            name="carriage_stays_inside_left_end_of_track",
        )
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            shuttle,
            beam,
            axes="x",
            inner_elem="internal_carriage",
            outer_elem="track",
            margin=0.01,
            name="carriage_stays_inside_right_end_of_track",
        )

    with ctx.pose({shoulder: 0.82, elbow: -1.10}):
        ctx.expect_gap(
            beam,
            second_link,
            axis="z",
            min_gap=0.016,
            name="folded_arm_clears_beam",
        )
        ctx.expect_gap(
            shuttle,
            second_link,
            axis="z",
            min_gap=0.010,
            name="folded_plate_clears_carriage_cover",
        )

    with ctx.pose({shoulder: -0.42, elbow: -0.72}):
        ctx.expect_gap(
            beam,
            second_link,
            axis="z",
            min_gap=0.020,
            name="rear_swing_still_clears_track",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
