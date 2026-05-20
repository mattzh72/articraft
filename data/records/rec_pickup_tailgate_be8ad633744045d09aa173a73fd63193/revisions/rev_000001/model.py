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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


TAILGATE_WIDTH = 1.49
TAILGATE_HEIGHT = 0.54
TAILGATE_THICKNESS = 0.054
TAILGATE_OUTER_SKIN = 0.006
OPENING_WIDTH = 0.36
OPENING_HEIGHT = 0.36
OPENING_BOTTOM = 0.10
OPENING_HINGE_X = -OPENING_WIDTH / 2.0 + 0.010
OPENING_HINGE_Y = -0.020
OPENING_HINGE_Z = OPENING_BOTTOM + 0.008
DOOR_WIDTH = 0.340
DOOR_HEIGHT = 0.344
DOOR_THICKNESS = 0.040


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _ring_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    hole_width: float,
    hole_height: float,
    hole_bottom: float,
    center_y: float,
    outer_radius: float,
    hole_radius: float,
    name: str,
):
    outer_profile = _translate_profile(
        rounded_rect_profile(width, height, outer_radius),
        0.0,
        height * 0.5,
    )
    hole_profile = _translate_profile(
        rounded_rect_profile(hole_width, hole_height, hole_radius),
        0.0,
        hole_bottom + hole_height * 0.5,
    )
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [hole_profile],
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, center_y, 0.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_pass_through")

    body_paint = model.material("body_paint", rgba=(0.70, 0.72, 0.75, 1.0))
    bed_liner = model.material("bed_liner", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.24, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.45, 0.47, 0.49, 1.0))
    latch_black = model.material("latch_black", rgba=(0.08, 0.08, 0.09, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((1.68, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.27, -0.02)),
        material=bed_liner,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((0.08, 0.32, 0.58)),
        origin=Origin(xyz=(-0.80, 0.16, 0.29)),
        material=body_paint,
        name="left_bed_side",
    )
    bed_frame.visual(
        Box((0.08, 0.32, 0.58)),
        origin=Origin(xyz=(0.80, 0.16, 0.29)),
        material=body_paint,
        name="right_bed_side",
    )
    bed_frame.visual(
        Box((0.08, 0.08, 0.03)),
        origin=Origin(xyz=(-0.80, 0.04, 0.595)),
        material=body_paint,
        name="left_rail_cap",
    )
    bed_frame.visual(
        Box((0.08, 0.08, 0.03)),
        origin=Origin(xyz=(0.80, 0.04, 0.595)),
        material=body_paint,
        name="right_rail_cap",
    )
    bed_frame.visual(
        Box((0.09, 0.07, 0.10)),
        origin=Origin(xyz=(-0.745, 0.055, 0.035)),
        material=dark_metal,
        name="left_hinge_bracket",
    )
    bed_frame.visual(
        Box((0.09, 0.07, 0.10)),
        origin=Origin(xyz=(0.745, 0.055, 0.035)),
        material=dark_metal,
        name="right_hinge_bracket",
    )
    bed_frame.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(
            xyz=(-0.774, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="left_hinge_pin",
    )
    bed_frame.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(
            xyz=(0.774, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="right_hinge_pin",
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((1.68, 0.50, 0.64)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.25, 0.28)),
    )

    side_frame_width = (TAILGATE_WIDTH - OPENING_WIDTH) * 0.5
    top_frame_height = TAILGATE_HEIGHT - OPENING_BOTTOM - OPENING_HEIGHT

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((TAILGATE_WIDTH, 0.048, OPENING_BOTTOM)),
        origin=Origin(xyz=(0.0, -0.024, OPENING_BOTTOM * 0.5)),
        material=body_paint,
        name="gate_core",
    )
    tailgate.visual(
        Box((side_frame_width, 0.048, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH * 0.5 + side_frame_width * 0.5),
                -0.024,
                OPENING_BOTTOM + OPENING_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="left_stile",
    )
    tailgate.visual(
        Box((side_frame_width, 0.048, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH * 0.5 + side_frame_width * 0.5,
                -0.024,
                OPENING_BOTTOM + OPENING_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="right_stile",
    )
    tailgate.visual(
        Box((TAILGATE_WIDTH, 0.048, top_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                -0.024,
                OPENING_BOTTOM + OPENING_HEIGHT + top_frame_height * 0.5,
            )
        ),
        material=body_paint,
        name="top_cap",
    )
    tailgate.visual(
        Box((TAILGATE_WIDTH - 0.020, TAILGATE_OUTER_SKIN, OPENING_BOTTOM - 0.012)),
        origin=Origin(xyz=(0.0, -0.051, 0.050)),
        material=body_paint,
        name="outer_skin_bottom",
    )
    tailgate.visual(
        Box((side_frame_width - 0.016, TAILGATE_OUTER_SKIN, OPENING_HEIGHT - 0.016)),
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH * 0.5 + side_frame_width * 0.5),
                -0.051,
                OPENING_BOTTOM + OPENING_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="outer_skin_left",
    )
    tailgate.visual(
        Box((side_frame_width - 0.016, TAILGATE_OUTER_SKIN, OPENING_HEIGHT - 0.016)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH * 0.5 + side_frame_width * 0.5,
                -0.051,
                OPENING_BOTTOM + OPENING_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="outer_skin_right",
    )
    tailgate.visual(
        Box((TAILGATE_WIDTH - 0.020, TAILGATE_OUTER_SKIN, top_frame_height - 0.012)),
        origin=Origin(xyz=(0.0, -0.051, 0.500)),
        material=body_paint,
        name="outer_skin_top",
    )
    tailgate.visual(
        Box((0.020, 0.010, 0.052)),
        origin=Origin(xyz=(0.190, -0.005, OPENING_BOTTOM + DOOR_HEIGHT * 0.5)),
        material=dark_metal,
        name="door_striker",
    )
    tailgate.visual(
        Box((0.050, 0.028, 0.050)),
        origin=Origin(xyz=(-0.729, 0.0, 0.025)),
        material=dark_metal,
        name="left_hinge_ear",
    )
    tailgate.visual(
        Box((0.050, 0.028, 0.050)),
        origin=Origin(xyz=(0.729, 0.0, 0.025)),
        material=dark_metal,
        name="right_hinge_ear",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(
            xyz=(-0.754, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="left_gate_hinge",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(
            xyz=(0.754, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="right_gate_hinge",
    )
    tailgate.visual(
        Box((0.012, 0.022, 0.092)),
        origin=Origin(
            xyz=(
                OPENING_HINGE_X - 0.012,
                OPENING_HINGE_Y,
                OPENING_HINGE_Z + 0.046,
            )
        ),
        material=satin_metal,
        name="door_hinge_knuckle_lower",
    )
    tailgate.visual(
        Box((0.012, 0.022, 0.092)),
        origin=Origin(
            xyz=(
                OPENING_HINGE_X - 0.012,
                OPENING_HINGE_Y,
                OPENING_HINGE_Z + 0.298,
            )
        ),
        material=satin_metal,
        name="door_hinge_knuckle_upper",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((TAILGATE_WIDTH, 0.060, TAILGATE_HEIGHT)),
        mass=26.0,
        origin=Origin(xyz=(0.0, -0.03, TAILGATE_HEIGHT * 0.5)),
    )

    pass_through_door = model.part("pass_through_door")
    pass_through_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.0, DOOR_HEIGHT * 0.5)),
        material=body_paint,
        name="door_shell",
    )
    pass_through_door.visual(
        Box((DOOR_WIDTH - 0.022, TAILGATE_OUTER_SKIN, DOOR_HEIGHT - 0.022)),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, -0.023, DOOR_HEIGHT * 0.5)),
        material=body_paint,
        name="door_outer_skin",
    )
    pass_through_door.visual(
        Box((0.024, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.012, 0.0, DOOR_HEIGHT * 0.5)),
        material=dark_metal,
        name="hinge_stile",
    )
    pass_through_door.visual(
        Box((0.220, 0.010, 0.180)),
        origin=Origin(xyz=(0.190, 0.010, 0.176)),
        material=bed_liner,
        name="inner_reinforcement",
    )
    pass_through_door.visual(
        Cylinder(radius=0.006, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=satin_metal,
        name="door_hinge_leaf_lower",
    )
    pass_through_door.visual(
        Cylinder(radius=0.006, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.296)),
        material=satin_metal,
        name="door_hinge_leaf_upper",
    )
    pass_through_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=6.0,
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.0, DOOR_HEIGHT * 0.5)),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="latch_hub",
    )
    latch.visual(
        Box((0.052, 0.012, 0.020)),
        origin=Origin(xyz=(0.014, 0.014, 0.0)),
        material=latch_black,
        name="latch_paddle",
    )
    latch.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.032, 0.014, 0.0)),
        material=latch_black,
        name="latch_tip",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.060, 0.014, 0.026)),
        mass=0.15,
        origin=Origin(xyz=(0.026, 0.007, 0.0)),
    )

    tailgate_hinge = model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=1.5,
            lower=0.0,
            upper=1.60,
        ),
    )
    door_hinge = model.articulation(
        "tailgate_to_pass_through",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=pass_through_door,
        origin=Origin(xyz=(OPENING_HINGE_X, OPENING_HINGE_Y, OPENING_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    latch_joint = model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=pass_through_door,
        child=latch,
        origin=Origin(xyz=(0.286, DOOR_THICKNESS * 0.5, 0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=5.0,
            lower=-1.1,
            upper=1.1,
        ),
    )

    model.meta["primary_articulations"] = (
        tailgate_hinge.name,
        door_hinge.name,
        latch_joint.name,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    pass_through_door = object_model.get_part("pass_through_door")
    latch = object_model.get_part("latch")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    door_hinge = object_model.get_articulation("tailgate_to_pass_through")
    latch_joint = object_model.get_articulation("door_to_latch")

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
        "parts present",
        all(part is not None for part in (bed_frame, tailgate, pass_through_door, latch)),
        "Expected bed frame, tailgate, pass-through door, and latch parts.",
    )
    ctx.check(
        "joint axes match mechanisms",
        tailgate_hinge.axis == (1.0, 0.0, 0.0)
        and door_hinge.axis == (0.0, 0.0, 1.0)
        and latch_joint.axis == (0.0, 1.0, 0.0),
        "Tailgate should hinge on X, access door on Z, and latch on Y.",
    )

    with ctx.pose({tailgate_hinge: 0.0, door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_contact(
            tailgate,
            bed_frame,
            elem_a="left_gate_hinge",
            elem_b="left_hinge_pin",
            name="left tailgate hinge seated on bed pin",
        )
        ctx.expect_contact(
            tailgate,
            bed_frame,
            elem_a="right_gate_hinge",
            elem_b="right_hinge_pin",
            name="right tailgate hinge seated on bed pin",
        )
        ctx.expect_contact(
            pass_through_door,
            tailgate,
            elem_a="door_hinge_leaf_lower",
            elem_b="door_hinge_knuckle_lower",
            name="door lower hinge leaf clipped to gate",
        )
        ctx.expect_contact(
            pass_through_door,
            tailgate,
            elem_a="door_hinge_leaf_upper",
            elem_b="door_hinge_knuckle_upper",
            name="door upper hinge leaf clipped to gate",
        )
        ctx.expect_contact(
            latch,
            pass_through_door,
            elem_a="latch_hub",
            elem_b="door_shell",
            name="latch mounted on door face",
        )
        ctx.expect_within(
            pass_through_door,
            tailgate,
            axes="xz",
            margin=0.02,
            name="pass-through door sits inside tailgate envelope",
        )

    with ctx.pose({door_hinge: 1.1}):
        ctx.expect_contact(
            pass_through_door,
            tailgate,
            elem_a="door_hinge_leaf_lower",
            elem_b="door_hinge_knuckle_lower",
            name="door remains attached at lower hinge when open",
        )
        ctx.expect_contact(
            pass_through_door,
            tailgate,
            elem_a="door_hinge_leaf_upper",
            elem_b="door_hinge_knuckle_upper",
            name="door remains attached at upper hinge when open",
        )

    with ctx.pose({tailgate_hinge: 1.3, door_hinge: 1.1, latch_joint: 0.7}):
        ctx.expect_contact(
            tailgate,
            bed_frame,
            elem_a="left_gate_hinge",
            elem_b="left_hinge_pin",
            name="tailgate stays attached when lowered",
        )
        ctx.expect_contact(
            pass_through_door,
            tailgate,
            elem_a="door_hinge_leaf_lower",
            elem_b="door_hinge_knuckle_lower",
            name="door stays clipped through combined swing",
        )
        ctx.expect_contact(
            latch,
            pass_through_door,
            elem_a="latch_hub",
            elem_b="door_shell",
            name="latch stays mounted through combined swing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
