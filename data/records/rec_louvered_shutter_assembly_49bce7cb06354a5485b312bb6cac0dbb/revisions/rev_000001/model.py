from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


FRAME_OUTER_WIDTH = 0.90
FRAME_OUTER_HEIGHT = 1.48
FRAME_DEPTH = 0.11
FRAME_JAMB_WIDTH = 0.065
FRAME_HEAD_HEIGHT = 0.075
FRAME_SILL_HEIGHT = 0.085
FRAME_CLEAR_WIDTH = FRAME_OUTER_WIDTH - 2.0 * FRAME_JAMB_WIDTH

LEAF_HEIGHT = 0.74
LEAF_THICKNESS = 0.028
LEAF_CENTER_GAP = 0.008
LEAF_WIDTH = (FRAME_CLEAR_WIDTH - LEAF_CENTER_GAP) / 2.0
STILE_WIDTH = 0.032
RAIL_HEIGHT = 0.070
OPENING_WIDTH = LEAF_WIDTH - 2.0 * STILE_WIDTH
OPENING_HEIGHT = LEAF_HEIGHT - 2.0 * RAIL_HEIGHT
LEAF_BASE_Z = FRAME_SILL_HEIGHT + 0.014
LEAF_MID_Z = LEAF_BASE_Z + LEAF_HEIGHT * 0.5

LOUVER_COUNT = 7
LOUVER_CHORD = 0.048
LOUVER_THICKNESS = 0.010
LOUVER_PIN_RADIUS = 0.0042
LOUVER_PIN_LENGTH = 0.012
LOUVER_BLADE_LENGTH = OPENING_WIDTH - 2.0 * LOUVER_PIN_LENGTH
LOUVER_PITCH = OPENING_HEIGHT / (LOUVER_COUNT + 1)

ROD_WIDTH = 0.018
ROD_THICKNESS = 0.010
ROD_HEIGHT = OPENING_HEIGHT - 0.040
ROD_CENTER_Y = 0.014
ROD_TRAVEL = 0.028

TAB_WIDTH = 0.010
TAB_THICKNESS = 0.004
TAB_HEIGHT = 0.026
TAB_CENTER_Y = 0.005 + TAB_THICKNESS * 0.5

KNOB_STEM_RADIUS = 0.0032
KNOB_STEM_LENGTH = 0.006
KNOB_BODY_RADIUS = 0.012
KNOB_BODY_LENGTH = 0.010
KNOB_CAP_RADIUS = 0.008
KNOB_CAP_LENGTH = 0.004

PIVOT_CLIP_PROJECTION = 0.006
PIVOT_CLIP_DEPTH = 0.004
PIVOT_CLIP_HEIGHT = 0.018
PIVOT_CLIP_Y = 0.010


def _build_louver_mesh() -> object:
    profile = superellipse_profile(
        LOUVER_THICKNESS,
        LOUVER_CHORD,
        exponent=2.2,
        segments=32,
    )
    geom = ExtrudeGeometry.centered(profile, LOUVER_BLADE_LENGTH, cap=True, closed=True)
    geom.rotate_z(-math.pi / 2.0)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, "cafe_shutter_louver")


def _louver_z_positions() -> list[float]:
    bottom_opening = -OPENING_HEIGHT * 0.5
    return [
        bottom_opening + LOUVER_PITCH * float(index + 1)
        for index in range(LOUVER_COUNT)
    ]


def _leaf_opening_center_x(side_sign: float) -> float:
    return side_sign * LEAF_WIDTH * 0.5


def _add_leaf_frame_visuals(model, leaf_part, side_sign: float, material) -> None:
    opening_center_x = _leaf_opening_center_x(side_sign)

    leaf_part.visual(
        Box((STILE_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        origin=Origin(xyz=(side_sign * STILE_WIDTH * 0.5, 0.0, 0.0)),
        material=material,
        name="outer_stile",
    )
    leaf_part.visual(
        Box((STILE_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        origin=Origin(
            xyz=(side_sign * (LEAF_WIDTH - STILE_WIDTH * 0.5), 0.0, 0.0)
        ),
        material=material,
        name="inner_stile",
    )
    leaf_part.visual(
        Box((OPENING_WIDTH, LEAF_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(xyz=(opening_center_x, 0.0, LEAF_HEIGHT * 0.5 - RAIL_HEIGHT * 0.5)),
        material=material,
        name="top_rail",
    )
    leaf_part.visual(
        Box((OPENING_WIDTH, LEAF_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(opening_center_x, 0.0, -LEAF_HEIGHT * 0.5 + RAIL_HEIGHT * 0.5)
        ),
        material=material,
        name="bottom_rail",
    )

    outer_clip_x = side_sign * (STILE_WIDTH + PIVOT_CLIP_PROJECTION * 0.5)
    inner_clip_x = side_sign * (LEAF_WIDTH - STILE_WIDTH - PIVOT_CLIP_PROJECTION * 0.5)
    for louver_index, z_pos in enumerate(_louver_z_positions(), start=1):
        for clip_x, edge_name in (
            (outer_clip_x, "outer"),
            (inner_clip_x, "inner"),
        ):
            for clip_y, face_name in (
                (PIVOT_CLIP_Y, "front"),
                (-PIVOT_CLIP_Y, "rear"),
            ):
                leaf_part.visual(
                    Box(
                        (
                            PIVOT_CLIP_PROJECTION,
                            PIVOT_CLIP_DEPTH,
                            PIVOT_CLIP_HEIGHT,
                        )
                    ),
                    origin=Origin(xyz=(clip_x, clip_y, z_pos)),
                    material=material,
                    name=f"{edge_name}_pivot_clip_{face_name}_{louver_index}",
                )

    leaf_part.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(opening_center_x, 0.0, 0.0)),
    )


def _add_louver(
    model,
    *,
    prefix: str,
    index: int,
    z_pos: float,
    opening_center_x: float,
    parent_leaf,
    louver_mesh,
    louver_material,
) -> None:
    louver = model.part(f"{prefix}_louver_{index}")
    louver.visual(
        louver_mesh,
        material=louver_material,
        name="blade",
    )
    left_pin_x = -(LOUVER_BLADE_LENGTH * 0.5 + LOUVER_PIN_LENGTH * 0.5)
    right_pin_x = -left_pin_x
    for pin_x, pin_name in ((left_pin_x, "left_pin"), (right_pin_x, "right_pin")):
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(xyz=(pin_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=louver_material,
            name=pin_name,
        )
    louver.visual(
        Box((TAB_WIDTH, TAB_THICKNESS, TAB_HEIGHT)),
        origin=Origin(xyz=(0.0, TAB_CENTER_Y, 0.0)),
        material=louver_material,
        name="drive_tab",
    )
    louver.inertial = Inertial.from_geometry(
        Box((OPENING_WIDTH, LOUVER_THICKNESS, LOUVER_CHORD)),
        mass=0.11,
        origin=Origin(),
    )
    model.articulation(
        f"{prefix}_louver_{index}_pivot",
        ArticulationType.REVOLUTE,
        parent=parent_leaf,
        child=louver,
        origin=Origin(xyz=(opening_center_x, 0.0, z_pos)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-0.65,
            upper=0.65,
        ),
    )


def _add_tilt_rod_and_knob(
    model,
    *,
    prefix: str,
    opening_center_x: float,
    parent_leaf,
    rod_material,
    knob_material,
) -> None:
    rod = model.part(f"{prefix}_tilt_rod")
    rod.visual(
        Box((ROD_WIDTH, ROD_THICKNESS, ROD_HEIGHT)),
        material=rod_material,
        name="rod_bar",
    )
    rod.inertial = Inertial.from_geometry(
        Box((ROD_WIDTH, ROD_THICKNESS, ROD_HEIGHT)),
        mass=0.18,
        origin=Origin(),
    )
    model.articulation(
        f"{prefix}_tilt_rod_slide",
        ArticulationType.PRISMATIC,
        parent=parent_leaf,
        child=rod,
        origin=Origin(xyz=(opening_center_x, ROD_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=-ROD_TRAVEL,
            upper=ROD_TRAVEL,
        ),
    )

    knob = model.part(f"{prefix}_tilt_knob")
    knob.visual(
        Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_LENGTH),
        origin=Origin(
            xyz=(0.0, KNOB_STEM_LENGTH * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_material,
        name="stem",
    )
    knob.visual(
        Cylinder(radius=KNOB_BODY_RADIUS, length=KNOB_BODY_LENGTH),
        origin=Origin(
            xyz=(0.0, KNOB_STEM_LENGTH + KNOB_BODY_LENGTH * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_material,
        name="body",
    )
    knob.visual(
        Cylinder(radius=KNOB_CAP_RADIUS, length=KNOB_CAP_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                KNOB_STEM_LENGTH + KNOB_BODY_LENGTH + KNOB_CAP_LENGTH * 0.5,
                0.0,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_material,
        name="cap",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.024, 0.022, 0.024)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
    )
    model.articulation(
        f"{prefix}_tilt_knob_spin",
        ArticulationType.REVOLUTE,
        parent=rod,
        child=knob,
        origin=Origin(xyz=(0.0, ROD_THICKNESS * 0.5, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=-2.2,
            upper=2.2,
        ),
    )


def _add_leaf_assembly(
    model,
    *,
    prefix: str,
    side_sign: float,
    hinge_x: float,
    lower_limit: float,
    upper_limit: float,
    frame_part,
    frame_material,
    louver_material,
    rod_material,
    knob_material,
    louver_mesh,
) -> None:
    leaf = model.part(f"{prefix}_leaf")
    _add_leaf_frame_visuals(model, leaf, side_sign, frame_material)
    model.articulation(
        f"{prefix}_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame_part,
        child=leaf,
        origin=Origin(xyz=(hinge_x, 0.0, LEAF_MID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=lower_limit,
            upper=upper_limit,
        ),
    )

    opening_center_x = _leaf_opening_center_x(side_sign)
    for louver_index, z_pos in enumerate(_louver_z_positions(), start=1):
        _add_louver(
            model,
            prefix=prefix,
            index=louver_index,
            z_pos=z_pos,
            opening_center_x=opening_center_x,
            parent_leaf=leaf,
            louver_mesh=louver_mesh,
            louver_material=louver_material,
        )

    _add_tilt_rod_and_knob(
        model,
        prefix=prefix,
        opening_center_x=opening_center_x,
        parent_leaf=leaf,
        rod_material=rod_material,
        knob_material=knob_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_shutter_set")

    frame_paint = model.material("frame_paint", rgba=(0.94, 0.93, 0.90, 1.0))
    louver_paint = model.material("louver_paint", rgba=(0.95, 0.94, 0.91, 1.0))
    rod_finish = model.material("rod_finish", rgba=(0.88, 0.87, 0.84, 1.0))
    knob_brass = model.material("knob_brass", rgba=(0.69, 0.57, 0.31, 1.0))

    frame = model.part("window_frame")
    frame.visual(
        Box((FRAME_JAMB_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                -(FRAME_CLEAR_WIDTH * 0.5 + FRAME_JAMB_WIDTH * 0.5),
                0.0,
                FRAME_OUTER_HEIGHT * 0.5,
            )
        ),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((FRAME_JAMB_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                FRAME_CLEAR_WIDTH * 0.5 + FRAME_JAMB_WIDTH * 0.5,
                0.0,
                FRAME_OUTER_HEIGHT * 0.5,
            )
        ),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_HEAD_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, FRAME_OUTER_HEIGHT - FRAME_HEAD_HEIGHT * 0.5)
        ),
        material=frame_paint,
        name="head",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_SILL_HEIGHT * 0.5)),
        material=frame_paint,
        name="sill",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, 0.014, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(0.0, -FRAME_DEPTH * 0.5 + 0.007, FRAME_OUTER_HEIGHT * 0.5)),
        material=frame_paint,
        name="rear_casing",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, FRAME_OUTER_HEIGHT * 0.5)),
    )

    louver_mesh = _build_louver_mesh()
    _add_leaf_assembly(
        model,
        prefix="left",
        side_sign=1.0,
        hinge_x=-FRAME_CLEAR_WIDTH * 0.5,
        lower_limit=0.0,
        upper_limit=1.35,
        frame_part=frame,
        frame_material=frame_paint,
        louver_material=louver_paint,
        rod_material=rod_finish,
        knob_material=knob_brass,
        louver_mesh=louver_mesh,
    )
    _add_leaf_assembly(
        model,
        prefix="right",
        side_sign=-1.0,
        hinge_x=FRAME_CLEAR_WIDTH * 0.5,
        lower_limit=-1.35,
        upper_limit=0.0,
        frame_part=frame,
        frame_material=frame_paint,
        louver_material=louver_paint,
        rod_material=rod_finish,
        knob_material=knob_brass,
        louver_mesh=louver_mesh,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    def _has_part(name: str) -> bool:
        try:
            object_model.get_part(name)
            return True
        except Exception:
            return False

    def _has_joint(name: str) -> bool:
        try:
            object_model.get_articulation(name)
            return True
        except Exception:
            return False

    expected_parts = [
        "window_frame",
        "left_leaf",
        "right_leaf",
        "left_tilt_rod",
        "right_tilt_rod",
        "left_tilt_knob",
        "right_tilt_knob",
    ] + [
        f"{side}_louver_{index}"
        for side in ("left", "right")
        for index in range(1, LOUVER_COUNT + 1)
    ]
    expected_joints = [
        "left_leaf_hinge",
        "right_leaf_hinge",
        "left_tilt_rod_slide",
        "right_tilt_rod_slide",
        "left_tilt_knob_spin",
        "right_tilt_knob_spin",
    ] + [
        f"{side}_louver_{index}_pivot"
        for side in ("left", "right")
        for index in range(1, LOUVER_COUNT + 1)
    ]
    for part_name in expected_parts:
        ctx.check(f"part exists: {part_name}", _has_part(part_name), part_name)
    for joint_name in expected_joints:
        ctx.check(f"joint exists: {joint_name}", _has_joint(joint_name), joint_name)

    frame = object_model.get_part("window_frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    left_rod = object_model.get_part("left_tilt_rod")
    right_rod = object_model.get_part("right_tilt_rod")
    left_knob = object_model.get_part("left_tilt_knob")
    right_knob = object_model.get_part("right_tilt_knob")

    left_leaf_hinge = object_model.get_articulation("left_leaf_hinge")
    right_leaf_hinge = object_model.get_articulation("right_leaf_hinge")
    left_rod_slide = object_model.get_articulation("left_tilt_rod_slide")
    right_rod_slide = object_model.get_articulation("right_tilt_rod_slide")
    left_knob_spin = object_model.get_articulation("left_tilt_knob_spin")
    right_knob_spin = object_model.get_articulation("right_tilt_knob_spin")

    ctx.expect_contact(left_leaf, frame, contact_tol=5e-5, name="left leaf bears on frame")
    ctx.expect_contact(right_leaf, frame, contact_tol=5e-5, name="right leaf bears on frame")
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        min_gap=0.006,
        max_gap=0.010,
        name="meeting stiles keep a narrow center gap",
    )

    for side in ("left", "right"):
        leaf = object_model.get_part(f"{side}_leaf")
        rod = object_model.get_part(f"{side}_tilt_rod")
        knob = object_model.get_part(f"{side}_tilt_knob")
        ctx.expect_within(
            rod,
            leaf,
            axes="xz",
            margin=0.001,
            name=f"{side} tilt rod stays inside leaf bounds",
        )
        ctx.expect_contact(
            knob,
            rod,
            contact_tol=5e-5,
            name=f"{side} knob mounts onto tilt rod",
        )
        ctx.expect_origin_distance(
            knob,
            rod,
            axes="xz",
            max_dist=0.001,
            name=f"{side} knob stays centered on tilt rod",
        )
        for index in range(1, LOUVER_COUNT + 1):
            louver = object_model.get_part(f"{side}_louver_{index}")
            ctx.expect_contact(
                louver,
                leaf,
                contact_tol=5e-5,
                name=f"{side} louver {index} stays captured in stile pivots",
            )
            ctx.expect_contact(
                louver,
                rod,
                contact_tol=5e-5,
                name=f"{side} louver {index} engages the tilt rod",
            )
            ctx.expect_within(
                louver,
                leaf,
                axes="xz",
                margin=0.001,
                name=f"{side} louver {index} sits inside framed opening",
            )

    frame_aabb = ctx.part_world_aabb(frame)
    left_leaf_aabb = ctx.part_world_aabb(left_leaf)
    if frame_aabb is not None and left_leaf_aabb is not None:
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        leaf_height = left_leaf_aabb[1][2] - left_leaf_aabb[0][2]
        ratio = leaf_height / frame_height if frame_height > 0.0 else 0.0
        ctx.check(
            "leaves read as half-height cafe shutters",
            0.42 <= ratio <= 0.58,
            f"leaf/frame height ratio was {ratio:.3f}",
        )

    ctx.check(
        "left leaf hinge rotates about vertical axis",
        tuple(left_leaf_hinge.axis) == (0.0, 0.0, 1.0)
        and left_leaf_hinge.motion_limits is not None
        and left_leaf_hinge.motion_limits.lower == 0.0
        and left_leaf_hinge.motion_limits.upper == 1.35,
        str(left_leaf_hinge.axis),
    )
    ctx.check(
        "right leaf hinge rotates about vertical axis",
        tuple(right_leaf_hinge.axis) == (0.0, 0.0, 1.0)
        and right_leaf_hinge.motion_limits is not None
        and right_leaf_hinge.motion_limits.lower == -1.35
        and right_leaf_hinge.motion_limits.upper == 0.0,
        str(right_leaf_hinge.axis),
    )
    ctx.check(
        "tilt rods translate vertically",
        tuple(left_rod_slide.axis) == (0.0, 0.0, 1.0)
        and tuple(right_rod_slide.axis) == (0.0, 0.0, 1.0),
        f"left={left_rod_slide.axis}, right={right_rod_slide.axis}",
    )
    ctx.check(
        "tilt rod knobs spin on local mounting axes",
        tuple(left_knob_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_knob_spin.axis) == (0.0, 1.0, 0.0),
        f"left={left_knob_spin.axis}, right={right_knob_spin.axis}",
    )
    for side in ("left", "right"):
        for index in range(1, LOUVER_COUNT + 1):
            pivot = object_model.get_articulation(f"{side}_louver_{index}_pivot")
            ctx.check(
                f"{side} louver {index} pivots on a horizontal axis",
                tuple(pivot.axis) == (1.0, 0.0, 0.0),
                str(pivot.axis),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
