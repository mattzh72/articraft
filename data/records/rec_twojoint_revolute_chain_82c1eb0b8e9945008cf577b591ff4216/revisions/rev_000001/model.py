from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.30
FOOT_WIDTH = 0.09
FOOT_THICKNESS = 0.016

SHOULDER_X = -0.055
SHOULDER_Z = 0.092

HINGE_BLOCK_LENGTH = 0.030
HINGE_BLOCK_WIDTH = 0.024
HINGE_BLOCK_HEIGHT = 0.014

CLEAR_PLATE_THICKNESS = 0.004
HINGE_HUB_RADIUS = 0.010
HINGE_HUB_LENGTH = 0.018
CLEAR_PLATE_CENTER_Y = HINGE_HUB_LENGTH / 2.0 + CLEAR_PLATE_THICKNESS / 2.0

PEDESTAL_LENGTH = 0.072
PEDESTAL_WIDTH = 0.038
PEDESTAL_HEIGHT = SHOULDER_Z - FOOT_THICKNESS - HINGE_HUB_RADIUS - HINGE_BLOCK_HEIGHT

LOWER_LINK_LENGTH = 0.165
LOWER_LINK_WIDTH = 0.012
LOWER_LINK_HEIGHT = 0.014

DISTAL_LINK_LENGTH = 0.145
DISTAL_LINK_WIDTH = 0.011
DISTAL_LINK_HEIGHT = 0.013

END_TAB_LENGTH = 0.032
END_TAB_WIDTH = 0.020
END_TAB_THICKNESS = 0.010
END_TAB_HOLE_RADIUS = 0.004


def _add_mesh_visual(part, shape, name: str, material: str, origin: Origin | None = None) -> None:
    part.visual(
        mesh_from_cadquery(shape, name),
        origin=origin or Origin(),
        material=material,
        name=name,
    )


def _plate_pair_shape(
    *,
    joint_x: float,
    joint_z: float,
    plate_length: float,
    plate_height: float,
    z_drop: float = 0.004,
):
    pair = None
    for sign in (-1.0, 1.0):
        plate = cq.Workplane("XY").box(
            plate_length,
            CLEAR_PLATE_THICKNESS,
            plate_height,
        ).translate((joint_x, sign * CLEAR_PLATE_CENTER_Y, joint_z - z_drop))
        pair = plate if pair is None else pair.union(plate)
    return pair


def _base_body_shape():
    foot = cq.Workplane("XY").box(
        FOOT_LENGTH,
        FOOT_WIDTH,
        FOOT_THICKNESS,
    ).translate((0.0, 0.0, FOOT_THICKNESS / 2.0))

    pedestal = cq.Workplane("XY").box(
        PEDESTAL_LENGTH,
        PEDESTAL_WIDTH,
        PEDESTAL_HEIGHT,
    ).translate((SHOULDER_X, 0.0, FOOT_THICKNESS + PEDESTAL_HEIGHT / 2.0))

    shoulder_block = cq.Workplane("XY").box(
        HINGE_BLOCK_LENGTH,
        HINGE_BLOCK_WIDTH,
        HINGE_BLOCK_HEIGHT,
    ).translate(
        (
            SHOULDER_X,
            0.0,
            SHOULDER_Z - HINGE_HUB_RADIUS - HINGE_BLOCK_HEIGHT / 2.0,
        )
    )

    rear_rib = cq.Workplane("XZ").polyline(
        [
            (SHOULDER_X - 0.028, FOOT_THICKNESS),
            (SHOULDER_X + 0.004, FOOT_THICKNESS),
            (SHOULDER_X + 0.006, SHOULDER_Z - 0.030),
            (SHOULDER_X - 0.016, SHOULDER_Z - 0.030),
        ]
    ).close().extrude(0.026, both=True)

    return foot.union(pedestal).union(shoulder_block).union(rear_rib)


def _base_plate_shape():
    return _plate_pair_shape(
        joint_x=SHOULDER_X,
        joint_z=SHOULDER_Z,
        plate_length=0.028,
        plate_height=0.048,
        z_drop=0.002,
    )


def _lower_link_body_shape():
    shoulder_hub = cq.Workplane("XZ").circle(HINGE_HUB_RADIUS).extrude(
        HINGE_HUB_LENGTH / 2.0,
        both=True,
    )

    beam_length = LOWER_LINK_LENGTH - 0.021
    beam = cq.Workplane("XY").box(
        beam_length,
        LOWER_LINK_WIDTH,
        LOWER_LINK_HEIGHT,
    ).translate((beam_length / 2.0, 0.0, 0.0))

    elbow_web = cq.Workplane("XZ").polyline(
        [
            (LOWER_LINK_LENGTH - 0.038, -LOWER_LINK_HEIGHT / 2.0),
            (LOWER_LINK_LENGTH - 0.022, -LOWER_LINK_HEIGHT / 2.0),
            (LOWER_LINK_LENGTH - 0.011, -0.012),
            (LOWER_LINK_LENGTH - 0.011, -0.024),
            (LOWER_LINK_LENGTH - 0.030, -0.024),
        ]
    ).close().extrude(LOWER_LINK_WIDTH / 2.0, both=True)

    elbow_block = cq.Workplane("XY").box(
        0.024,
        HINGE_BLOCK_WIDTH,
        0.012,
    ).translate(
        (
            LOWER_LINK_LENGTH + 0.001,
            0.0,
            -0.018,
        )
    )

    return shoulder_hub.union(beam).union(elbow_web).union(elbow_block)


def _lower_link_plate_shape():
    return _plate_pair_shape(
        joint_x=LOWER_LINK_LENGTH,
        joint_z=0.0,
        plate_length=0.026,
        plate_height=0.044,
        z_drop=0.002,
    )


def _distal_link_body_shape():
    elbow_hub = cq.Workplane("XZ").circle(HINGE_HUB_RADIUS).extrude(
        HINGE_HUB_LENGTH / 2.0,
        both=True,
    )

    beam = cq.Workplane("XY").box(
        DISTAL_LINK_LENGTH,
        DISTAL_LINK_WIDTH,
        DISTAL_LINK_HEIGHT,
    ).translate((DISTAL_LINK_LENGTH / 2.0, 0.0, 0.0))

    return elbow_hub.union(beam)


def _distal_tab_shape():
    tab = cq.Workplane("XY").slot2D(END_TAB_LENGTH, END_TAB_WIDTH).extrude(
        END_TAB_THICKNESS / 2.0,
        both=True,
    ).translate((DISTAL_LINK_LENGTH + 0.010, 0.0, 0.0))

    return tab.faces(">Z").workplane().circle(END_TAB_HOLE_RADIUS).cutThruAll()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_revolute_chain")

    model.material("base_graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("link_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("clear_poly", rgba=(0.72, 0.86, 0.96, 0.40))
    model.material("tab_black", rgba=(0.12, 0.13, 0.14, 1.0))

    base_foot = model.part("base_foot")
    _add_mesh_visual(base_foot, _base_body_shape(), "base_body", "base_graphite")
    _add_mesh_visual(base_foot, _base_plate_shape(), "shoulder_plates", "clear_poly")
    base_foot.inertial = None

    lower_link = model.part("lower_link")
    _add_mesh_visual(lower_link, _lower_link_body_shape(), "lower_body", "link_aluminum")
    _add_mesh_visual(lower_link, _lower_link_plate_shape(), "elbow_plates", "clear_poly")
    lower_link.inertial = None

    distal_link = model.part("distal_link")
    _add_mesh_visual(distal_link, _distal_link_body_shape(), "distal_body", "link_aluminum")
    _add_mesh_visual(distal_link, _distal_tab_shape(), "distal_tab", "tab_black")
    distal_link.inertial = None

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_foot,
        child=lower_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=distal_link,
        origin=Origin(xyz=(LOWER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_foot = object_model.get_part("base_foot")
    lower_link = object_model.get_part("lower_link")
    distal_link = object_model.get_part("distal_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

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
        "expected parts exist",
        all(part is not None for part in (base_foot, lower_link, distal_link)),
        "Missing one or more authored parts.",
    )
    ctx.check(
        "joint axes share one motion plane",
        shoulder_joint.axis == (0.0, -1.0, 0.0) and elbow_joint.axis == (0.0, -1.0, 0.0),
        f"Shoulder axis={shoulder_joint.axis}, elbow axis={elbow_joint.axis}",
    )

    ctx.expect_contact(
        base_foot,
        lower_link,
        contact_tol=0.001,
        name="shoulder hub seats against base support",
    )
    ctx.expect_contact(
        lower_link,
        distal_link,
        contact_tol=0.001,
        name="elbow hub seats against lower-link hinge block",
    )

    with ctx.pose():
        closed_elbow_origin = ctx.part_world_position(distal_link)
        closed_tab_aabb = ctx.part_element_world_aabb(distal_link, elem="distal_tab")

    with ctx.pose(shoulder_joint=0.85):
        raised_elbow_origin = ctx.part_world_position(distal_link)

    with ctx.pose(shoulder_joint=0.20, elbow_joint=1.00):
        bent_tab_aabb = ctx.part_element_world_aabb(distal_link, elem="distal_tab")

    closed_tab_center_z = (closed_tab_aabb[0][2] + closed_tab_aabb[1][2]) / 2.0
    bent_tab_center_z = (bent_tab_aabb[0][2] + bent_tab_aabb[1][2]) / 2.0

    ctx.check(
        "positive shoulder rotation lifts the elbow",
        raised_elbow_origin is not None
        and closed_elbow_origin is not None
        and raised_elbow_origin[2] > closed_elbow_origin[2] + 0.08,
        f"Closed elbow origin={closed_elbow_origin}, raised elbow origin={raised_elbow_origin}",
    )
    ctx.check(
        "positive elbow rotation lifts the distal tab",
        bent_tab_center_z > closed_tab_center_z + 0.035,
        f"Closed tab center z={closed_tab_center_z:.4f}, bent tab center z={bent_tab_center_z:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
