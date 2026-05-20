from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


OUTER_FRAME_WIDTH = 0.66
OUTER_FRAME_HEIGHT = 1.42
OUTER_FRAME_DEPTH = 0.032
OUTER_MEMBER_WIDTH = 0.045

PANEL_SIDE_CLEARANCE = 0.004
PANEL_TOP_BOTTOM_CLEARANCE = 0.004
PANEL_WIDTH = OUTER_FRAME_WIDTH - 2.0 * OUTER_MEMBER_WIDTH - 2.0 * PANEL_SIDE_CLEARANCE
PANEL_HEIGHT = OUTER_FRAME_HEIGHT - 2.0 * OUTER_MEMBER_WIDTH - 2.0 * PANEL_TOP_BOTTOM_CLEARANCE
PANEL_DEPTH = 0.028

PANEL_STILE_WIDTH = 0.055
PANEL_RAIL_HEIGHT = 0.075
PANEL_OPENING_WIDTH = PANEL_WIDTH - 2.0 * PANEL_STILE_WIDTH
PANEL_OPENING_HEIGHT = PANEL_HEIGHT - 2.0 * PANEL_RAIL_HEIGHT

HINGE_AXIS_Y = OUTER_FRAME_DEPTH * 0.5
PANEL_BODY_CENTER_Y = -HINGE_AXIS_Y
PANEL_FRONT_FACE_Y = PANEL_BODY_CENTER_Y + PANEL_DEPTH * 0.5

HINGE_BARREL_RADIUS = 0.004
HINGE_BARREL_LENGTH = 0.09
HINGE_LEAF_WIDTH = 0.014
HINGE_LEAF_THICKNESS = 0.004
HINGE_Z_POSITIONS = (0.21, PANEL_HEIGHT * 0.5, PANEL_HEIGHT - 0.21)

LOUVER_COUNT = 12
LOUVER_HEIGHT = 0.080
LOUVER_THICKNESS = 0.011
LOUVER_PIN_RADIUS = 0.0035
LOUVER_PIN_LENGTH = 0.020
LOUVER_BLADE_LENGTH = PANEL_OPENING_WIDTH - 2.0 * LOUVER_PIN_LENGTH
LOUVER_PITCH = PANEL_OPENING_HEIGHT / LOUVER_COUNT
LOUVER_CENTER_X = PANEL_WIDTH * 0.5
LOUVER_CENTER_Y = PANEL_BODY_CENTER_Y

ROD_RADIUS = 0.0045
ROD_X = PANEL_STILE_WIDTH + PANEL_OPENING_WIDTH - 0.075
ROD_Y = 0.002
ROD_TOP_Z = PANEL_HEIGHT - PANEL_RAIL_HEIGHT - 0.024
ROD_BOTTOM_Z = PANEL_RAIL_HEIGHT + 0.024
ROD_CENTER_Z = 0.5 * (ROD_TOP_Z + ROD_BOTTOM_Z)
ROD_LENGTH = ROD_TOP_Z - ROD_BOTTOM_Z

ROD_GUIDE_DEPTH = 0.005
ROD_GUIDE_HEIGHT = 0.008
ROD_GUIDE_Y = ROD_Y - ROD_RADIUS - ROD_GUIDE_DEPTH * 0.5
ROD_GUIDE_LENGTH = PANEL_WIDTH - PANEL_STILE_WIDTH - ROD_X

CONNECTOR_TAB_WIDTH = 0.012
CONNECTOR_TAB_HEIGHT = 0.012
CONNECTOR_TAB_DEPTH = (ROD_Y - LOUVER_CENTER_Y - ROD_RADIUS) - (LOUVER_THICKNESS * 0.5)
CONNECTOR_TAB_X = ROD_X - LOUVER_CENTER_X
CONNECTOR_TAB_Y = (LOUVER_THICKNESS * 0.5) + CONNECTOR_TAB_DEPTH * 0.5

PANEL_HINGE_X = (-OUTER_FRAME_WIDTH * 0.5) + OUTER_MEMBER_WIDTH + PANEL_SIDE_CLEARANCE
PANEL_BOTTOM_Z = OUTER_MEMBER_WIDTH + PANEL_TOP_BOTTOM_CLEARANCE

LOUVER_PART_NAMES = [f"louver_{index + 1:02d}" for index in range(LOUVER_COUNT)]
LOUVER_JOINT_NAMES = [f"{name}_tilt" for name in LOUVER_PART_NAMES]


def _louver_section(x_pos: float, width: float, thickness: float) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_thickness = thickness * 0.5
    return [
        (x_pos, 0.00 * half_thickness, half_width),
        (x_pos, 0.55 * half_thickness, 0.82 * half_width),
        (x_pos, 0.92 * half_thickness, 0.36 * half_width),
        (x_pos, 1.00 * half_thickness, 0.00),
        (x_pos, 0.92 * half_thickness, -0.36 * half_width),
        (x_pos, 0.55 * half_thickness, -0.82 * half_width),
        (x_pos, 0.00 * half_thickness, -half_width),
        (x_pos, -0.55 * half_thickness, -0.82 * half_width),
        (x_pos, -0.92 * half_thickness, -0.36 * half_width),
        (x_pos, -1.00 * half_thickness, 0.00),
        (x_pos, -0.92 * half_thickness, 0.36 * half_width),
        (x_pos, -0.55 * half_thickness, 0.82 * half_width),
    ]


def _build_louver_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _louver_section(-LOUVER_BLADE_LENGTH * 0.5, LOUVER_HEIGHT, LOUVER_THICKNESS),
                _louver_section(LOUVER_BLADE_LENGTH * 0.5, LOUVER_HEIGHT, LOUVER_THICKNESS),
            ]
        ),
        "plantation_louver_blade",
    )


def _louver_center_z(index: int) -> float:
    return PANEL_RAIL_HEIGHT + (LOUVER_PITCH * (index + 0.5))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter")

    painted_wood = model.material("painted_wood", rgba=(0.94, 0.93, 0.89, 1.0))
    warmer_wood = model.material("warmer_wood", rgba=(0.96, 0.95, 0.91, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.72, 0.72, 0.70, 1.0))

    louver_mesh = _build_louver_mesh()

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((OUTER_MEMBER_WIDTH, OUTER_FRAME_DEPTH, OUTER_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OUTER_FRAME_WIDTH * 0.5) + (OUTER_MEMBER_WIDTH * 0.5),
                0.0,
                OUTER_FRAME_HEIGHT * 0.5,
            )
        ),
        material=painted_wood,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((OUTER_MEMBER_WIDTH, OUTER_FRAME_DEPTH, OUTER_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                (OUTER_FRAME_WIDTH * 0.5) - (OUTER_MEMBER_WIDTH * 0.5),
                0.0,
                OUTER_FRAME_HEIGHT * 0.5,
            )
        ),
        material=painted_wood,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((OUTER_FRAME_WIDTH - (2.0 * OUTER_MEMBER_WIDTH), OUTER_FRAME_DEPTH, OUTER_MEMBER_WIDTH)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_FRAME_HEIGHT - (OUTER_MEMBER_WIDTH * 0.5))),
        material=painted_wood,
        name="head_rail",
    )
    outer_frame.visual(
        Box((OUTER_FRAME_WIDTH - (2.0 * OUTER_MEMBER_WIDTH), OUTER_FRAME_DEPTH, OUTER_MEMBER_WIDTH)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_MEMBER_WIDTH * 0.5)),
        material=painted_wood,
        name="sill_rail",
    )

    for hinge_name, hinge_z in zip(("lower", "middle", "upper"), HINGE_Z_POSITIONS):
        outer_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(PANEL_HINGE_X, HINGE_AXIS_Y, PANEL_BOTTOM_Z + hinge_z)),
            material=hinge_metal,
            name=f"hinge_barrel_{hinge_name}",
        )

    panel_frame = model.part("panel_frame")
    panel_frame.visual(
        Box((PANEL_STILE_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                PANEL_STILE_WIDTH * 0.5,
                PANEL_BODY_CENTER_Y,
                PANEL_HEIGHT * 0.5,
            )
        ),
        material=painted_wood,
        name="hinge_stile",
    )
    panel_frame.visual(
        Box((PANEL_STILE_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                PANEL_WIDTH - (PANEL_STILE_WIDTH * 0.5),
                PANEL_BODY_CENTER_Y,
                PANEL_HEIGHT * 0.5,
            )
        ),
        material=painted_wood,
        name="latch_stile",
    )
    panel_frame.visual(
        Box((PANEL_OPENING_WIDTH, PANEL_DEPTH, PANEL_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                PANEL_WIDTH * 0.5,
                PANEL_BODY_CENTER_Y,
                PANEL_HEIGHT - (PANEL_RAIL_HEIGHT * 0.5),
            )
        ),
        material=painted_wood,
        name="top_rail",
    )
    panel_frame.visual(
        Box((PANEL_OPENING_WIDTH, PANEL_DEPTH, PANEL_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                PANEL_WIDTH * 0.5,
                PANEL_BODY_CENTER_Y,
                PANEL_RAIL_HEIGHT * 0.5,
            )
        ),
        material=painted_wood,
        name="bottom_rail",
    )

    for hinge_name, hinge_z in zip(("lower", "middle", "upper"), HINGE_Z_POSITIONS):
        panel_frame.visual(
            Box((HINGE_LEAF_WIDTH, HINGE_LEAF_THICKNESS, HINGE_BARREL_LENGTH)),
            origin=Origin(
                xyz=(
                    HINGE_BARREL_RADIUS + (HINGE_LEAF_WIDTH * 0.5),
                    0.0,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"hinge_leaf_{hinge_name}",
        )

    panel_frame.visual(
        Box((ROD_GUIDE_LENGTH, ROD_GUIDE_DEPTH, ROD_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                ROD_X + (ROD_GUIDE_LENGTH * 0.5),
                ROD_GUIDE_Y,
                ROD_TOP_Z,
            )
        ),
        material=painted_wood,
        name="rod_guide_top",
    )
    panel_frame.visual(
        Box((ROD_GUIDE_LENGTH, ROD_GUIDE_DEPTH, ROD_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                ROD_X + (ROD_GUIDE_LENGTH * 0.5),
                ROD_GUIDE_Y,
                ROD_BOTTOM_Z,
            )
        ),
        material=painted_wood,
        name="rod_guide_bottom",
    )

    tilt_rod = model.part("tilt_rod")
    tilt_rod.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        material=warmer_wood,
        name="rod_body",
    )
    tilt_rod.visual(
        Cylinder(radius=ROD_RADIUS * 1.15, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, (ROD_LENGTH * 0.5) - 0.006)),
        material=warmer_wood,
        name="rod_cap_top",
    )
    tilt_rod.visual(
        Cylinder(radius=ROD_RADIUS * 1.15, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -(ROD_LENGTH * 0.5) + 0.006)),
        material=warmer_wood,
        name="rod_cap_bottom",
    )

    for index, louver_name in enumerate(LOUVER_PART_NAMES):
        louver = model.part(louver_name)
        louver.visual(louver_mesh, material=warmer_wood, name="blade")
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(
                    -((LOUVER_BLADE_LENGTH * 0.5) + (LOUVER_PIN_LENGTH * 0.5)),
                    0.0,
                    0.0,
                ),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=painted_wood,
            name="left_pin",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(
                    (LOUVER_BLADE_LENGTH * 0.5) + (LOUVER_PIN_LENGTH * 0.5),
                    0.0,
                    0.0,
                ),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=painted_wood,
            name="right_pin",
        )
        louver.visual(
            Box((CONNECTOR_TAB_WIDTH, CONNECTOR_TAB_DEPTH, CONNECTOR_TAB_HEIGHT)),
            origin=Origin(
                xyz=(
                    CONNECTOR_TAB_X,
                    CONNECTOR_TAB_Y,
                    0.0,
                )
            ),
            material=painted_wood,
            name="connector_tab",
        )

        model.articulation(
            f"{louver_name}_tilt",
            ArticulationType.REVOLUTE,
            parent=panel_frame,
            child=louver,
            origin=Origin(xyz=(LOUVER_CENTER_X, LOUVER_CENTER_Y, _louver_center_z(index))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.0, upper=1.0),
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=panel_frame,
        origin=Origin(xyz=(PANEL_HINGE_X, HINGE_AXIS_Y, PANEL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.57),
    )
    model.articulation(
        "panel_to_tilt_rod",
        ArticulationType.FIXED,
        parent=panel_frame,
        child=tilt_rod,
        origin=Origin(xyz=(ROD_X, ROD_Y, ROD_CENTER_Z)),
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

    def require_part(name: str):
        try:
            result = object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"{name} present", str(exc))
            return None
        ctx.check(f"{name} present", True)
        return result

    def require_joint(name: str):
        try:
            result = object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"{name} present", str(exc))
            return None
        ctx.check(f"{name} present", True)
        return result

    outer_frame = require_part("outer_frame")
    panel_frame = require_part("panel_frame")
    tilt_rod = require_part("tilt_rod")
    louver_parts = [require_part(name) for name in LOUVER_PART_NAMES]
    panel_hinge = require_joint("frame_to_panel")
    rod_joint = require_joint("panel_to_tilt_rod")
    louver_joints = [require_joint(name) for name in LOUVER_JOINT_NAMES]

    if (
        outer_frame is None
        or panel_frame is None
        or tilt_rod is None
        or panel_hinge is None
        or rod_joint is None
        or any(part is None for part in louver_parts)
        or any(joint is None for joint in louver_joints)
    ):
        return ctx.report()

    ctx.check(
        "panel hinge axis is vertical",
        tuple(panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"got axis={panel_hinge.axis}",
    )
    panel_limits = panel_hinge.motion_limits
    ctx.check(
        "panel hinge has shutter-like swing limits",
        panel_limits is not None
        and panel_limits.lower is not None
        and panel_limits.upper is not None
        and abs(panel_limits.lower - 0.0) < 1e-9
        and panel_limits.upper >= 1.5,
        details=f"got limits={panel_limits}",
    )
    ctx.check(
        "tilt rod mounted with fixed articulation",
        rod_joint.joint_type == ArticulationType.FIXED,
        details=f"got type={rod_joint.joint_type}",
    )
    ctx.check(
        "all louvers share a long-axis tilt joint",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in louver_joints),
        details="One or more louver joints are not aligned to the louver long axis.",
    )

    ctx.expect_contact(
        panel_frame,
        outer_frame,
        elem_a="hinge_leaf_middle",
        elem_b="hinge_barrel_middle",
        name="panel hangs from hinge barrel",
    )
    ctx.expect_contact(
        tilt_rod,
        panel_frame,
        elem_a="rod_body",
        elem_b="rod_guide_top",
        name="tilt rod captured at top guide",
    )
    ctx.expect_contact(
        tilt_rod,
        panel_frame,
        elem_a="rod_body",
        elem_b="rod_guide_bottom",
        name="tilt rod captured at bottom guide",
    )
    ctx.expect_within(
        tilt_rod,
        panel_frame,
        axes="z",
        margin=0.0,
        name="tilt rod stays within panel height",
    )

    for louver in louver_parts:
        ctx.expect_contact(
            louver,
            panel_frame,
            elem_a="left_pin",
            elem_b="hinge_stile",
            name=f"{louver.name} left pivot seats in stile",
        )
        ctx.expect_contact(
            louver,
            tilt_rod,
            elem_a="connector_tab",
            elem_b="rod_body",
            name=f"{louver.name} connector tab reaches tilt rod",
        )
        ctx.expect_within(
            louver,
            panel_frame,
            axes="xz",
            margin=0.0,
            name=f"{louver.name} stays inside panel footprint",
        )

    latch_closed = ctx.part_element_world_aabb(panel_frame, elem="latch_stile")
    mid_louver = louver_parts[LOUVER_COUNT // 2]
    mid_joint = louver_joints[LOUVER_COUNT // 2]
    blade_closed = ctx.part_element_world_aabb(mid_louver, elem="blade")

    with ctx.pose({panel_hinge: 1.15}):
        latch_open = ctx.part_element_world_aabb(panel_frame, elem="latch_stile")
    ctx.check(
        "panel swings clear on its side hinge",
        latch_closed is not None
        and latch_open is not None
        and (latch_open[1][1] > latch_closed[1][1] + 0.30),
        details=f"closed={latch_closed}, open={latch_open}",
    )

    with ctx.pose({mid_joint: 0.70}):
        blade_open = ctx.part_element_world_aabb(mid_louver, elem="blade")
        ctx.expect_contact(
            mid_louver,
            panel_frame,
            elem_a="left_pin",
            elem_b="hinge_stile",
            name="representative louver keeps its left pivot engaged while tilted",
        )
    ctx.check(
        "representative louver rotates out of the panel plane",
        blade_closed is not None
        and blade_open is not None
        and (blade_open[1][1] > blade_closed[1][1] + 0.02),
        details=f"closed={blade_closed}, open={blade_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
