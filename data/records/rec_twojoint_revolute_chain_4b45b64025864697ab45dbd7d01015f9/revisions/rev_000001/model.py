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


BRIDGE_SPAN = 0.250
BRIDGE_DEPTH = 0.060
FOOT_WIDTH = 0.078
FOOT_HEIGHT = 0.016
UPRIGHT_WIDTH = 0.034
UPRIGHT_DEPTH = 0.052
UPRIGHT_HEIGHT = 0.246
UPRIGHT_X = 0.104
BEAM_HEIGHT = 0.038
BEAM_Z = 0.281
ROOT_BLOCK_WIDTH = 0.056
ROOT_BLOCK_DEPTH = 0.034
ROOT_BLOCK_HEIGHT = 0.028
ROOT_BLOCK_Z = 0.248
ROOT_PIN_Z = 0.242
ROOT_EAR_THICKNESS = 0.012
ROOT_EAR_GAP = 0.014
ROOT_EAR_WIDTH = 0.034
ROOT_EAR_RADIUS = 0.016
ROOT_EAR_PLATE_HEIGHT = 0.028

PROXIMAL_LENGTH = 0.118
PROXIMAL_ROOT_EYE_RADIUS = 0.014
PROXIMAL_ROOT_EYE_LENGTH = 0.012
ELBOW_EAR_THICKNESS = 0.009
ELBOW_EAR_GAP = 0.012
ELBOW_EAR_WIDTH = 0.028
ELBOW_EAR_RADIUS = 0.012
ELBOW_EAR_PLATE_HEIGHT = 0.020
ELBOW_JOINT_Z = -PROXIMAL_LENGTH

DISTAL_BEAM_LENGTH = 0.084
DISTAL_ROOT_EYE_RADIUS = 0.011
DISTAL_ROOT_EYE_LENGTH = ELBOW_EAR_GAP
JOINT_CONTACT_TOL = 0.0025


def _rounded_rect_sketch(width: float, depth: float, radius: float) -> cq.Sketch:
    return cq.Sketch().rect(width, depth).vertices().fillet(radius).reset()


def _tapered_bar(sections: list[tuple[float, float, float, float]]) -> cq.Workplane:
    sketches = [
        _rounded_rect_sketch(width, depth, radius).moved(z=z_pos)
        for z_pos, width, depth, radius in sections
    ]
    return cq.Workplane("XY").placeSketch(*sketches).loft(combine=True)


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    x_pos, y_pos, z_pos = center_xyz
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x_pos, y_pos + length / 2.0, z_pos))
    )


def _ear_with_round_lobe(
    *,
    width: float,
    thickness: float,
    pin_z: float,
    y_center: float,
    lobe_radius: float,
    upper_plate_height: float,
) -> cq.Workplane:
    lobe = _y_cylinder(lobe_radius, thickness, (0.0, y_center, pin_z))
    plate = cq.Workplane("XY").box(
        width,
        thickness,
        upper_plate_height,
    ).translate((0.0, y_center, pin_z + upper_plate_height / 2.0))
    return lobe.union(plate)


def _bridge_support_shape() -> cq.Workplane:
    left_foot = cq.Workplane("XY").box(FOOT_WIDTH, BRIDGE_DEPTH, FOOT_HEIGHT).translate(
        (-UPRIGHT_X, 0.0, FOOT_HEIGHT / 2.0)
    )
    right_foot = cq.Workplane("XY").box(FOOT_WIDTH, BRIDGE_DEPTH, FOOT_HEIGHT).translate(
        (UPRIGHT_X, 0.0, FOOT_HEIGHT / 2.0)
    )
    left_upright = cq.Workplane("XY").box(
        UPRIGHT_WIDTH,
        UPRIGHT_DEPTH,
        UPRIGHT_HEIGHT,
    ).translate((-UPRIGHT_X, 0.0, FOOT_HEIGHT + UPRIGHT_HEIGHT / 2.0))
    right_upright = cq.Workplane("XY").box(
        UPRIGHT_WIDTH,
        UPRIGHT_DEPTH,
        UPRIGHT_HEIGHT,
    ).translate((UPRIGHT_X, 0.0, FOOT_HEIGHT + UPRIGHT_HEIGHT / 2.0))
    top_beam = cq.Workplane("XY").box(BRIDGE_SPAN, BRIDGE_DEPTH, BEAM_HEIGHT).translate(
        (0.0, 0.0, BEAM_Z)
    )
    left_ear = _ear_with_round_lobe(
        width=ROOT_EAR_WIDTH,
        thickness=ROOT_EAR_THICKNESS,
        pin_z=ROOT_PIN_Z,
        y_center=-(ROOT_EAR_GAP + ROOT_EAR_THICKNESS) / 2.0,
        lobe_radius=ROOT_EAR_RADIUS,
        upper_plate_height=ROOT_EAR_PLATE_HEIGHT,
    )
    right_ear = _ear_with_round_lobe(
        width=ROOT_EAR_WIDTH,
        thickness=ROOT_EAR_THICKNESS,
        pin_z=ROOT_PIN_Z,
        y_center=(ROOT_EAR_GAP + ROOT_EAR_THICKNESS) / 2.0,
        lobe_radius=ROOT_EAR_RADIUS,
        upper_plate_height=ROOT_EAR_PLATE_HEIGHT,
    )

    return (
        left_foot.union(right_foot)
        .union(left_upright)
        .union(right_upright)
        .union(top_beam)
        .union(left_ear)
        .union(right_ear)
    )


def _proximal_link_shape() -> cq.Workplane:
    root_eye = _y_cylinder(
        PROXIMAL_ROOT_EYE_RADIUS,
        PROXIMAL_ROOT_EYE_LENGTH,
        (0.0, 0.0, 0.0),
    )
    beam = _tapered_bar(
        [
            (-0.010, 0.018, 0.012, 0.0018),
            (-0.058, 0.022, 0.018, 0.0018),
            (-0.106, 0.024, 0.030, 0.0018),
        ]
    )
    distal_knuckle = _y_cylinder(
        ELBOW_EAR_RADIUS,
        ELBOW_EAR_GAP + 2.0 * ELBOW_EAR_THICKNESS,
        (0.0, 0.0, ELBOW_JOINT_Z),
    )
    clevis_slot = cq.Workplane("XY").box(
        0.030,
        ELBOW_EAR_GAP,
        0.044,
    ).translate(
        (
            0.0,
            0.0,
            ELBOW_JOINT_Z + 0.010,
        )
    )
    return root_eye.union(beam).union(distal_knuckle).cut(clevis_slot)


def _distal_link_shape() -> cq.Workplane:
    root_eye = _y_cylinder(
        DISTAL_ROOT_EYE_RADIUS,
        DISTAL_ROOT_EYE_LENGTH,
        (0.0, 0.0, 0.0),
    )
    beam = _tapered_bar(
        [
            (-0.008, 0.020, 0.010, 0.0018),
            (-0.044, 0.017, 0.009, 0.0015),
            (-0.074, 0.014, 0.008, 0.0012),
        ]
    )
    end_tab = (
        cq.Workplane("XZ")
        .slot2D(0.028, 0.014, 90)
        .extrude(0.008)
        .translate((0.0, 0.004, -0.087))
    )
    tab_hole = _y_cylinder(0.0045, 0.010, (0.0, 0.0, -0.101))

    return root_eye.union(beam).union(end_tab).cut(tab_hole)


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_revolute_chain")

    bridge_finish = model.material("bridge_finish", rgba=(0.18, 0.20, 0.23, 1.0))
    primary_link_finish = model.material("primary_link_finish", rgba=(0.63, 0.66, 0.70, 1.0))
    distal_link_finish = model.material("distal_link_finish", rgba=(0.75, 0.78, 0.81, 1.0))

    bridge_support = model.part("bridge_support")
    bridge_support.visual(
        mesh_from_cadquery(_bridge_support_shape(), "bridge_support"),
        material=bridge_finish,
        name="support_body",
    )
    bridge_support.inertial = Inertial.from_geometry(
        Box((BRIDGE_SPAN, BRIDGE_DEPTH, BEAM_Z + BEAM_HEIGHT / 2.0)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, (BEAM_Z + BEAM_HEIGHT / 2.0) / 2.0)),
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(_proximal_link_shape(), "proximal_link"),
        material=primary_link_finish,
        name="proximal_body",
    )
    proximal_link.inertial = Inertial.from_geometry(
        Box((0.036, 0.030, PROXIMAL_LENGTH + 0.030)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, -(PROXIMAL_LENGTH / 2.0))),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        material=distal_link_finish,
        name="distal_body",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((0.022, 0.014, DISTAL_BEAM_LENGTH + 0.034)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    model.articulation(
        "root_revolute",
        ArticulationType.REVOLUTE,
        parent=bridge_support,
        child=proximal_link,
        origin=Origin(xyz=(0.0, 0.0, ROOT_PIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.8, lower=-1.10, upper=1.15),
    )
    model.articulation(
        "elbow_revolute",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(0.0, 0.0, ELBOW_JOINT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.30, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    root_joint = object_model.get_articulation("root_revolute")
    elbow_joint = object_model.get_articulation("elbow_revolute")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=JOINT_CONTACT_TOL)
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
        "all required parts exist",
        all(part is not None for part in (bridge_support, proximal_link, distal_link)),
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "both joints are revolute with pitch axes",
        root_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(root_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(elbow_joint.axis) == (0.0, -1.0, 0.0),
        details=(
            f"root_type={root_joint.articulation_type}, root_axis={root_joint.axis}, "
            f"elbow_type={elbow_joint.articulation_type}, elbow_axis={elbow_joint.axis}"
        ),
    )
    ctx.expect_contact(
        proximal_link,
        bridge_support,
        contact_tol=JOINT_CONTACT_TOL,
        name="proximal root eye seats against the bridge clevis",
    )
    ctx.expect_contact(
        distal_link,
        proximal_link,
        contact_tol=JOINT_CONTACT_TOL,
        name="distal root eye seats against the proximal clevis",
    )
    neutral_distal_center = _center_from_aabb(ctx.part_element_world_aabb(distal_link, elem="distal_body"))
    ctx.check(
        "neutral chain hangs beneath the bridge span",
        neutral_distal_center is not None and neutral_distal_center[2] < ROOT_PIN_Z - 0.080,
        details=f"distal_center={neutral_distal_center}, root_pin_z={ROOT_PIN_Z}",
    )

    prox_rest_center = _center_from_aabb(ctx.part_element_world_aabb(proximal_link, elem="proximal_body"))
    with ctx.pose({root_joint: 0.70, elbow_joint: 0.0}):
        prox_lifted_center = _center_from_aabb(
            ctx.part_element_world_aabb(proximal_link, elem="proximal_body")
        )
    ctx.check(
        "positive root rotation lifts the proximal link toward +x",
        prox_rest_center is not None
        and prox_lifted_center is not None
        and prox_lifted_center[0] > prox_rest_center[0] + 0.020,
        details=f"rest={prox_rest_center}, lifted={prox_lifted_center}",
    )

    with ctx.pose({root_joint: 0.45, elbow_joint: 0.0}):
        distal_rest_center = _center_from_aabb(
            ctx.part_element_world_aabb(distal_link, elem="distal_body")
        )
    with ctx.pose({root_joint: 0.45, elbow_joint: 0.75}):
        distal_folded_center = _center_from_aabb(
            ctx.part_element_world_aabb(distal_link, elem="distal_body")
        )
    ctx.check(
        "positive elbow rotation advances the compact end tab outward",
        distal_rest_center is not None
        and distal_folded_center is not None
        and distal_folded_center[0] > distal_rest_center[0] + 0.015,
        details=f"rest={distal_rest_center}, folded={distal_folded_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
