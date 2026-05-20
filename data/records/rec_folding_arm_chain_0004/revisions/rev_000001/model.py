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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PIN_RADIUS = 0.0035
LINK_WIDTH = 0.018
BAR_WIDTH = 0.014
TONGUE_THICKNESS = 0.004
EAR_THICKNESS = 0.0035
JOINT_CLEARANCE = 0.0006
CLEVIS_GAP = TONGUE_THICKNESS + 2.0 * JOINT_CLEARANCE
OFFSET_Y = 0.0065
TONGUE_LENGTH = 0.014
EAR_LENGTH = 0.018

LINK_1_LENGTH = 0.082
LINK_2_LENGTH = 0.074
LINK_3_LENGTH = 0.060


def _centered_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _lug(length_x: float, width_z: float, thickness_y: float, hole_radius: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(length_x, thickness_y, width_z)
    cutter = cq.Workplane("XZ").circle(hole_radius).extrude(thickness_y + 0.002).translate(
        (0.0, -0.5 * (thickness_y + 0.002), 0.0)
    )
    return solid.cut(cutter)


def _pivot_pin(radius: float, length_y: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length_y)
        .translate((0.0, -0.5 * length_y, 0.0))
    )


def _add_visual(part, shape: cq.Workplane, filename: str, *, material, name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, filename, assets=ASSETS),
        material=material,
        name=name,
    )


def _build_base_shapes() -> dict[str, cq.Workplane]:
    back_plate = _centered_box((0.010, 0.006, 0.070), (-0.020, 0.0, 0.0))
    back_plate = (
        back_plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, 0.020), (0.0, -0.020)])
        .circle(0.0045)
        .cutThruAll()
    )

    saddle = _centered_box((0.012, 0.010, 0.028), (-0.013, 0.0, 0.0))
    ear_center_y = 0.5 * (CLEVIS_GAP + EAR_THICKNESS)
    left_ear = _lug(0.024, LINK_WIDTH, EAR_THICKNESS, PIN_RADIUS).translate((0.0, ear_center_y, 0.0))
    right_ear = _lug(0.024, LINK_WIDTH, EAR_THICKNESS, PIN_RADIUS).translate((0.0, -ear_center_y, 0.0))
    pivot_pin = _pivot_pin(PIN_RADIUS, CLEVIS_GAP + 2.0 * EAR_THICKNESS)
    return {
        "back_plate": back_plate,
        "saddle": saddle,
        "left_ear": left_ear,
        "right_ear": right_ear,
        "pivot_pin": pivot_pin,
    }


def _build_link_shapes(
    *,
    length: float,
    distal_pivot_y: float,
    distal_clevis: bool,
    add_pad: bool,
) -> dict[str, cq.Workplane]:
    root_tongue = _lug(TONGUE_LENGTH, LINK_WIDTH, TONGUE_THICKNESS, PIN_RADIUS)
    root_shank = _centered_box((0.034, TONGUE_THICKNESS, BAR_WIDTH), (0.017, 0.0, 0.0))
    transition = _centered_box(
        (0.026, TONGUE_THICKNESS + abs(distal_pivot_y), BAR_WIDTH),
        (0.042, 0.5 * distal_pivot_y, 0.0),
    )

    body_solids = [root_tongue, root_shank, transition]

    distal_span_start = 0.055
    distal_body_end = length - (0.014 if distal_clevis else -0.004)
    distal_shank_length = distal_body_end - distal_span_start
    if distal_shank_length > 0.001:
        body_solids.append(
            _centered_box(
                (distal_shank_length, TONGUE_THICKNESS, BAR_WIDTH),
                (0.5 * (distal_span_start + distal_body_end), distal_pivot_y, 0.0),
            )
        )

    if distal_clevis:
        ear_offset_y = 0.5 * (CLEVIS_GAP + EAR_THICKNESS)
        left_ear = _lug(EAR_LENGTH, LINK_WIDTH, EAR_THICKNESS, PIN_RADIUS).translate(
            (length, distal_pivot_y + ear_offset_y, 0.0)
        )
        right_ear = _lug(EAR_LENGTH, LINK_WIDTH, EAR_THICKNESS, PIN_RADIUS).translate(
            (length, distal_pivot_y - ear_offset_y, 0.0)
        )
        left_connector = _centered_box(
            (0.010, ear_offset_y + 0.0008, BAR_WIDTH),
            (length - 0.015, distal_pivot_y + 0.5 * ear_offset_y, 0.0),
        )
        right_connector = _centered_box(
            (0.010, ear_offset_y + 0.0008, BAR_WIDTH),
            (length - 0.015, distal_pivot_y - 0.5 * ear_offset_y, 0.0),
        )
        distal_pin = _pivot_pin(PIN_RADIUS, CLEVIS_GAP + 2.0 * EAR_THICKNESS).translate((length, distal_pivot_y, 0.0))
        body_solids.extend([left_connector, right_connector, left_ear, right_ear, distal_pin])

    if add_pad:
        pad_arm = _centered_box((0.020, TONGUE_THICKNESS, BAR_WIDTH), (length + 0.004, distal_pivot_y, 0.0))
        pad = _centered_box((0.018, 0.006, 0.020), (length + 0.021, distal_pivot_y, 0.0))
        body_solids.extend([pad_arm, pad])

    body = body_solids[0]
    for solid in body_solids[1:]:
        body = body.union(solid)

    return {"body": body}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_support_arm", assets=ASSETS)

    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.40, 0.43, 0.47, 1.0))
    pad_black = model.material("pad_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_bracket")
    base_shapes = _build_base_shapes()
    _add_visual(base, base_shapes["back_plate"], "base_back_plate.obj", material=bracket_steel, name="back_plate")
    _add_visual(base, base_shapes["saddle"], "base_saddle.obj", material=bracket_steel, name="saddle")
    _add_visual(base, base_shapes["left_ear"], "base_left_ear.obj", material=steel, name="left_ear")
    _add_visual(base, base_shapes["right_ear"], "base_right_ear.obj", material=steel, name="right_ear")
    base.inertial = Inertial.from_geometry(
        Box((0.040, 0.016, 0.070)),
        mass=0.28,
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
    )

    link1 = model.part("link_1")
    link1_shapes = _build_link_shapes(length=LINK_1_LENGTH, distal_pivot_y=-OFFSET_Y, distal_clevis=True, add_pad=False)
    for shape_name, shape in link1_shapes.items():
        _add_visual(link1, shape, f"link1_{shape_name}.obj", material=steel, name=shape_name)
    link1.inertial = Inertial.from_geometry(
        Box((LINK_1_LENGTH + 0.024, 0.018, LINK_WIDTH)),
        mass=0.18,
        origin=Origin(xyz=(0.5 * LINK_1_LENGTH, -0.5 * OFFSET_Y, 0.0)),
    )

    link2 = model.part("link_2")
    link2_shapes = _build_link_shapes(length=LINK_2_LENGTH, distal_pivot_y=OFFSET_Y, distal_clevis=True, add_pad=False)
    for shape_name, shape in link2_shapes.items():
        _add_visual(link2, shape, f"link2_{shape_name}.obj", material=steel, name=shape_name)
    link2.inertial = Inertial.from_geometry(
        Box((LINK_2_LENGTH + 0.024, 0.018, LINK_WIDTH)),
        mass=0.16,
        origin=Origin(xyz=(0.5 * LINK_2_LENGTH, 0.5 * OFFSET_Y, 0.0)),
    )

    link3 = model.part("link_3")
    link3_shapes = _build_link_shapes(length=LINK_3_LENGTH, distal_pivot_y=0.0, distal_clevis=False, add_pad=True)
    for shape_name, shape in link3_shapes.items():
        material = pad_black if shape_name == "body" else steel
        _add_visual(link3, shape, f"link3_{shape_name}.obj", material=material, name=shape_name)
    link3.inertial = Inertial.from_geometry(
        Box((LINK_3_LENGTH + 0.030, 0.012, 0.022)),
        mass=0.13,
        origin=Origin(xyz=(0.5 * LINK_3_LENGTH + 0.006, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK_1_LENGTH, -OFFSET_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK_2_LENGTH, OFFSET_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.6, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_bracket")
    link1 = object_model.get_part("link_1")
    link2 = object_model.get_part("link_2")
    link3 = object_model.get_part("link_3")

    base_to_link1 = object_model.get_articulation("base_to_link_1")
    link1_to_link2 = object_model.get_articulation("link_1_to_link_2")
    link2_to_link3 = object_model.get_articulation("link_2_to_link_3")

    back_plate = base.get_visual("back_plate")
    base_left_ear = base.get_visual("left_ear")
    link1_body = link1.get_visual("body")
    link2_body = link2.get_visual("body")
    link3_body = link3.get_visual("body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(
        contact_tol=0.001,
        name="fail_if_isolated_parts(running_clearance_tol=0.001)",
    )
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

    ctx.check("part_count", len(object_model.parts) == 4, f"expected 4 parts, found {len(object_model.parts)}")
    ctx.check(
        "joint_count",
        len(object_model.articulations) == 3,
        f"expected 3 articulations, found {len(object_model.articulations)}",
    )

    ctx.expect_gap(
        base,
        link1,
        axis="y",
        min_gap=0.0002,
        max_gap=0.0012,
        positive_elem=base_left_ear,
        negative_elem=link1_body,
        name="base_left_clevis_clearance",
    )
    ctx.expect_gap(
        link1,
        base,
        axis="x",
        min_gap=0.0075,
        max_gap=0.0085,
        positive_elem=link1_body,
        negative_elem=back_plate,
        name="base_back_plate_stays_behind_arm",
    )
    ctx.expect_contact(
        link1,
        link2,
        contact_tol=0.001,
        elem_a=link1_body,
        elem_b=link2_body,
        name="joint_2_bodies_meet_at_clevis",
    )
    ctx.expect_overlap(
        link1,
        link2,
        axes="xz",
        min_overlap=0.014,
        elem_a=link1_body,
        elem_b=link2_body,
        name="joint_2_shares_pivot_footprint",
    )
    ctx.expect_contact(
        link2,
        link3,
        contact_tol=0.001,
        elem_a=link2_body,
        elem_b=link3_body,
        name="joint_3_bodies_meet_at_clevis",
    )
    ctx.expect_overlap(
        link2,
        link3,
        axes="xz",
        min_overlap=0.014,
        elem_a=link2_body,
        elem_b=link3_body,
        name="joint_3_shares_pivot_footprint",
    )

    with ctx.pose({base_to_link1: 0.0, link1_to_link2: 0.0, link2_to_link3: 0.0}):
        ctx.expect_origin_gap(
            link2,
            link1,
            axis="x",
            min_gap=LINK_1_LENGTH - 0.0005,
            max_gap=LINK_1_LENGTH + 0.0005,
            name="link_2_reaches_link_1_distal_pivot",
        )
        ctx.expect_origin_gap(
            link3,
            link2,
            axis="x",
            min_gap=LINK_2_LENGTH - 0.0005,
            max_gap=LINK_2_LENGTH + 0.0005,
            name="link_3_reaches_link_2_distal_pivot",
        )
        ctx.expect_origin_gap(
            link3,
            base,
            axis="x",
            min_gap=LINK_1_LENGTH + LINK_2_LENGTH - 0.0005,
            max_gap=LINK_1_LENGTH + LINK_2_LENGTH + 0.0005,
            name="extended_chain_has_long_reach",
        )
        ctx.expect_origin_distance(
            link3,
            base,
            axes="z",
            max_dist=0.0005,
            name="extended_chain_stays_straight_in_plane",
        )

    with ctx.pose({base_to_link1: -1.6, link1_to_link2: 1.6, link2_to_link3: -1.6}):
        ctx.expect_origin_distance(
            link3,
            base,
            axes="x",
            max_dist=0.075,
            name="folded_chain_reduces_forward_reach",
        )
        ctx.expect_origin_gap(
            link3,
            base,
            axis="z",
            min_gap=0.070,
            max_gap=0.090,
            name="folded_chain_stacks_up_above_bracket",
        )
        ctx.expect_contact(
            link1,
            link2,
            contact_tol=0.001,
            elem_a=link1_body,
            elem_b=link2_body,
            name="folded_joint_2_keeps_seated_alignment",
        )
        ctx.expect_contact(
            link2,
            link3,
            contact_tol=0.001,
            elem_a=link2_body,
            elem_b=link3_body,
            name="folded_joint_3_keeps_seated_alignment",
        )

    with ctx.pose({base_to_link1: 1.6, link1_to_link2: -1.6, link2_to_link3: 1.6}):
        ctx.expect_origin_distance(
            link3,
            base,
            axes="x",
            max_dist=0.075,
            name="reverse_fold_also_reduces_forward_reach",
        )
        ctx.expect_origin_gap(
            link3,
            base,
            axis="z",
            min_gap=-0.090,
            max_gap=-0.070,
            name="reverse_fold_stacks_to_opposite_side_of_plane",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_adjacent=True, ignore_fixed=True)
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
