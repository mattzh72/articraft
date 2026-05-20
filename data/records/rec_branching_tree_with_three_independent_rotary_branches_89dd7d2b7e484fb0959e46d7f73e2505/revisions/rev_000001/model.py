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


BRIDGE_WIDTH = 0.16
BRIDGE_LENGTH = 1.28
BRIDGE_HEIGHT = 0.09

BRACKET_PLATE_X = 0.09
BRACKET_PLATE_Y = 0.07
BRACKET_PLATE_Z = 0.018
BRACKET_EAR_X = 0.058
BRACKET_EAR_T = 0.006
BRACKET_EAR_DROP = 0.102
BRACKET_CLEAR_GAP = 0.024
BRACKET_PIVOT_Z = -(BRACKET_PLATE_Z + 0.072)

BARREL_RADIUS = 0.012
BARREL_LENGTH = BRACKET_CLEAR_GAP
ARM_BODY_WIDTH = 0.014

BRANCH_LAYOUT = [
    ("left", -0.40, 0.58),
    ("center", 0.00, 0.68),
    ("right", 0.40, 0.58),
]


def make_bridge() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(BRIDGE_WIDTH, BRIDGE_LENGTH, BRIDGE_HEIGHT)
        .edges("|Y")
        .fillet(0.012)
    )
    spine = (
        cq.Workplane("XY")
        .box(0.07, BRIDGE_LENGTH * 0.82, 0.032)
        .translate((0.0, 0.0, BRIDGE_HEIGHT / 2 + 0.016))
    )
    end_caps = (
        cq.Workplane("XY")
        .box(BRIDGE_WIDTH * 0.82, 0.11, 0.03)
        .translate((0.0, BRIDGE_LENGTH * 0.5 - 0.055, BRIDGE_HEIGHT / 2 + 0.015))
        .union(
            cq.Workplane("XY")
            .box(BRIDGE_WIDTH * 0.82, 0.11, 0.03)
            .translate((0.0, -(BRIDGE_LENGTH * 0.5 - 0.055), BRIDGE_HEIGHT / 2 + 0.015))
        )
    )
    return beam.union(spine).union(end_caps)


def make_bracket() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BRACKET_PLATE_X, BRACKET_PLATE_Y, BRACKET_PLATE_Z, centered=(True, True, False))
        .translate((0.0, 0.0, -BRACKET_PLATE_Z))
    )
    hanger_web = (
        cq.Workplane("XY")
        .box(0.022, 0.030, 0.062)
        .translate((0.0, 0.0, -0.049))
    )
    pivot_pad = (
        cq.Workplane("XY")
        .box(0.038, 0.024, 0.010)
        .translate((0.0, 0.0, BRACKET_PIVOT_Z + 0.005))
    )
    rib_profile = [
        (0.019, -BRACKET_PLATE_Z),
        (0.030, -BRACKET_PLATE_Z),
        (0.024, BRACKET_PIVOT_Z + 0.010),
        (0.011, BRACKET_PIVOT_Z + 0.010),
    ]
    rib_a = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(0.006)
        .translate((0.0, 0.012, 0.0))
    )
    rib_b = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(-0.006)
        .translate((0.0, -0.012, 0.0))
    )
    return plate.union(hanger_web).union(pivot_pad).union(rib_a).union(rib_b)


def make_arm(length: float) -> cq.Workplane:
    top_cap = (
        cq.Workplane("XY")
        .box(0.034, ARM_BODY_WIDTH, 0.010)
        .translate((0.0, 0.0, -0.005))
    )

    shoulder_half = 0.022
    mid_half = 0.018
    tip_half = 0.024
    profile = [
        (0.007, -0.010),
        (0.010, -0.085),
        (shoulder_half, -0.148),
        (mid_half, -(0.48 * length)),
        (0.024, -(length - 0.048)),
        (tip_half, -length),
        (-tip_half, -length),
        (-0.024, -(length - 0.048)),
        (-mid_half, -(0.48 * length)),
        (-shoulder_half, -0.148),
        (-0.010, -0.085),
        (-0.007, -0.010),
    ]
    blade = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(ARM_BODY_WIDTH * 0.5, both=True)
    )

    return top_cap.union(blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_three_branch_rotary_frame")

    bridge_color = model.material("bridge_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    bracket_color = model.material("bracket_steel", rgba=(0.56, 0.59, 0.62, 1.0))
    arm_color = model.material("arm_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bridge = model.part("top_bridge")
    bridge.visual(
        mesh_from_cadquery(make_bridge(), "top_bridge"),
        material=bridge_color,
        name="bridge_shell",
    )

    for branch_name, y_pos, arm_length in BRANCH_LAYOUT:
        bracket = model.part(f"{branch_name}_bracket")
        bracket.visual(
            mesh_from_cadquery(make_bracket(), f"{branch_name}_bracket"),
            material=bracket_color,
            name="bracket_shell",
        )

        arm = model.part(f"{branch_name}_arm")
        arm.visual(
            mesh_from_cadquery(make_arm(arm_length), f"{branch_name}_arm"),
            material=arm_color,
            name="arm_shell",
        )

        model.articulation(
            f"bridge_to_{branch_name}_bracket",
            ArticulationType.FIXED,
            parent=bridge,
            child=bracket,
            origin=Origin(xyz=(0.0, y_pos, -BRIDGE_HEIGHT * 0.5)),
        )
        model.articulation(
            f"{branch_name}_bracket_to_{branch_name}_arm",
            ArticulationType.REVOLUTE,
            parent=bracket,
            child=arm,
            origin=Origin(xyz=(0.0, 0.0, BRACKET_PIVOT_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.5,
                lower=-0.8,
                upper=0.8,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("top_bridge")

    brackets = {}
    arms = {}
    hinges = {}
    for branch_name, _, _ in BRANCH_LAYOUT:
        brackets[branch_name] = object_model.get_part(f"{branch_name}_bracket")
        arms[branch_name] = object_model.get_part(f"{branch_name}_arm")
        hinges[branch_name] = object_model.get_articulation(f"{branch_name}_bracket_to_{branch_name}_arm")

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

    for branch_name, _, _ in BRANCH_LAYOUT:
        bracket = brackets[branch_name]
        arm = arms[branch_name]
        hinge = hinges[branch_name]

        ctx.expect_contact(
            bracket,
            bridge,
            name=f"{branch_name}_bracket_contacts_bridge",
        )
        ctx.expect_overlap(
            bracket,
            bridge,
            axes="xy",
            min_overlap=0.055,
            name=f"{branch_name}_bracket_has_real_mount_footprint",
        )
        ctx.expect_overlap(
            arm,
            bracket,
            axes="xy",
            min_overlap=0.012,
            name=f"{branch_name}_arm_is_centered_under_its_bracket",
        )
        ctx.expect_gap(
            bridge,
            arm,
            axis="z",
            min_gap=0.018,
            max_gap=0.10,
            name=f"{branch_name}_arm_hangs_below_bridge",
        )

        limits = hinge.motion_limits
        axis_ok = tuple(float(v) for v in hinge.axis) == (0.0, -1.0, 0.0)
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
        ctx.check(
            f"{branch_name}_arm_has_independent_revolute_joint",
            hinge.articulation_type == ArticulationType.REVOLUTE and axis_ok and limits_ok,
            details=(
                f"{hinge.name} should be a revolute joint around -Y with "
                f"bidirectional limits, got type={hinge.articulation_type}, "
                f"axis={hinge.axis}, limits={limits}"
            ),
        )

    rest_boxes = {name: ctx.part_world_aabb(arm) for name, arm in arms.items()}
    for branch_name in arms:
        hinge = hinges[branch_name]
        with ctx.pose({hinge: 0.6}):
            posed_box = ctx.part_world_aabb(arms[branch_name])
            other_boxes = {
                other_name: ctx.part_world_aabb(other_arm)
                for other_name, other_arm in arms.items()
                if other_name != branch_name
            }

        rest_box = rest_boxes[branch_name]
        moved_ok = (
            rest_box is not None
            and posed_box is not None
            and posed_box[1][0] > rest_box[1][0] + 0.12
        )
        stationary_ok = True
        stationary_details = []
        for other_name, other_box in other_boxes.items():
            rest_other = rest_boxes[other_name]
            unchanged = (
                rest_other is not None
                and other_box is not None
                and abs(other_box[0][0] - rest_other[0][0]) < 1e-5
                and abs(other_box[1][0] - rest_other[1][0]) < 1e-5
                and abs(other_box[0][2] - rest_other[0][2]) < 1e-5
                and abs(other_box[1][2] - rest_other[1][2]) < 1e-5
            )
            stationary_ok = stationary_ok and unchanged
            if not unchanged:
                stationary_details.append(other_name)

        ctx.check(
            f"{branch_name}_arm_swings_without_dragging_other_branches",
            moved_ok and stationary_ok,
            details=(
                f"{branch_name} posed_box={posed_box}, rest_box={rest_box}, "
                f"unexpectedly_moved={stationary_details}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
